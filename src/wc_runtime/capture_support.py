"""Session provenance, progress and durable final publication helpers."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tarfile
import time

from .source_archive import atomic_json, digest


def data_capabilities(profile):
    """Describe the actual acquisition boundary, without claiming device internals."""
    cameras=profile in ('all_sensors','mapping_cameras')
    return dict(schema_version=1,profile=profile,
        lidar=dict(recorded=['independent SDK raw and filtered PointCloud2',
            'frame sequence, device and host timestamps, serial, endpoint, readback, SDK queue statistics'],
            unavailable=['original UDP datagrams','raw optical measurements','verified per-point exposure times']),
        imu=dict(recorded=['serial byte batches including undecoded bytes','decoded raw packets and native measurements',
            'device timestamp fields, host arrival and monotonic times'],
            unavailable=['unexposed internal factory calibration','verified device-to-host clock synchronization']),
        wheel=dict(recorded=['request and response bytes, CRC, timeout and raw register words',
            'configured left/right register indexes','manual control transactions when --manual-drive is enabled'],
            unavailable=['registers outside the reviewed speed feedback read interface','independent ground truth motion']),
        cameras=dict(selected=cameras,recorded=['all successfully captured pre-rotation BGR8 frames, lossless compressed',
            'role, USB identity, actual image properties and capture times'] if cameras else [],
            unavailable=['original UVC bytes','proof every device exposure reached the application']),
        ultrasonic=dict(selected=profile=='all_sensors',
            recorded=['addresses 1..4 requests, responses, CRC, timeout, raw values, host times'] if profile=='all_sensors' else [],
            unavailable=['unconfirmed address-to-physical-position mapping','unverified raw-value distance units']))


def progress(directory, stage, **detail):
    directory = Path(directory)
    (directory/'events').mkdir(exist_ok=True)
    value = dict(stage=stage, host_monotonic_ns=time.monotonic_ns(), **detail)
    atomic_json(directory/'progress.json', value)
    with (directory/'events/lifecycle.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(value, ensure_ascii=False)+'\n')
    print(json.dumps(value, ensure_ascii=False), flush=True)


def software_snapshot(root, directory):
    root, directory = Path(root), Path(directory)
    destination = directory/'software'
    destination.mkdir()
    members = {}
    with tarfile.open(destination/'source_and_install.tar.gz', 'w:gz', compresslevel=1) as archive:
        for base in ('AGENTS.md', 'src', 'scripts', 'config', 'tests', 'docs', 'vendor_patches', 'SDKs', 'install/main'):
            parent = root/base
            if not parent.exists():
                continue
            paths = [parent] + list(parent.rglob('*')) if parent.is_dir() else [parent]
            for path in paths:
                relative = path.relative_to(root)
                if any(part in ('__pycache__', '.git', '.pytest_cache') for part in relative.parts):
                    continue
                if path.is_symlink():
                    members[str(relative)] = dict(symlink=str(path.readlink()))
                    archive.add(path, arcname=str(relative), recursive=False)
                elif path.is_file():
                    before = path.stat()
                    value = digest(path)
                    archive.add(path, arcname=str(relative), recursive=False)
                    after = path.stat()
                    if (before.st_size,before.st_mtime_ns) != (after.st_size,after.st_mtime_ns) or digest(path) != value:
                        raise ValueError('software changed while snapshotting: '+str(relative))
                    members[str(relative)] = dict(bytes=before.st_size, sha256=value)
    atomic_json(destination/'files.json', members)
    commands = {
        'git_status.txt':['git','status','--porcelain=v1','-uall'],
        'git_worktree.diff':['git','diff','--binary'],
        'git_index.diff':['git','diff','--cached','--binary'],
        'git_head.txt':['git','rev-parse','HEAD'],
        'packages.tsv':['dpkg-query','-W','-f=${Package}\t${Version}\n'],
    }
    result = {}
    for name, command in commands.items():
        process = subprocess.run(command, cwd=root, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
        (destination/name).write_bytes(process.stdout)
        result[name] = dict(returncode=process.returncode, stderr=process.stderr.decode('utf-8', errors='replace'))
        if process.returncode and name != 'packages.tsv':
            raise ValueError('software provenance failed: '+name)
    atomic_json(destination/'provenance.json', result)
    return {str(path.relative_to(directory)):digest(path) for path in destination.iterdir() if path.is_file()}


def synchronize_tree(directory):
    """Persist every added source file, then children before parent directories."""
    directory = Path(directory).absolute()
    if any(path.is_symlink() for path in (directory, *directory.parents)):
        raise ValueError('capture durability refuses linked directory ancestry')
    paths = list(directory.rglob('*'))
    count = 0
    for path in paths:
        if path.is_symlink():
            raise ValueError('capture archive must not contain symlinks: '+str(path))
        if path.is_file():
            with path.open('rb') as stream:
                os.fsync(stream.fileno())
            count += 1
    dirs = sorted([path for path in paths if path.is_dir()]+[directory], key=lambda p:len(p.parts), reverse=True)
    for path in dirs:
        fd = os.open(path, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    # The session root is a new entry in experiments/. Persist that entry and
    # any newly created ancestor entries, not only entries inside the session.
    ancestry = list(directory.parents)
    for path in ancestry:
        fd = os.open(path, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    return dict(complete=True, file_count=count, directory_count=len(dirs),
                ancestor_directory_count=len(ancestry),
                completed_monotonic_ns=time.monotonic_ns(),
                method='FILE_FSYNC_THEN_BOTTOM_UP_DIRECTORY_AND_ANCESTOR_FSYNC')


def create_source_indexes(directory, contract):
    """Small source-specific indexes point to the authoritative bag bytes."""
    directory = Path(directory)
    handles = {}
    try:
        for logical, spec in contract['sources'].items():
            kind = spec['kind']
            if logical.startswith('camera_'):
                continue
            if logical.startswith('lidar_'):
                path = directory/'sources/lidar'/logical.removeprefix('lidar_')/'bag_index.jsonl'
            elif logical.startswith('wheel_'):
                path = directory/'sources/wheel'/logical.removeprefix('wheel_')/'index.jsonl'
            elif logical.startswith('ultrasonic_address_'):
                path = directory/'sources/ultrasonic'/('address_'+logical.rsplit('_',1)[1])/'index.jsonl'
            else:
                path = directory/'sources/imu/bag_index.jsonl'
            path.parent.mkdir(parents=True,exist_ok=True)
            handles[logical] = path.open('x',encoding='utf-8')
        with (directory/'records.jsonl').open(encoding='utf-8') as records:
            for line_number, line in enumerate(records,1):
                row = json.loads(line)
                topic = row['topic']
                logicals = (['lidar_left'] if '/lidar_left/' in topic else
                            ['lidar_right'] if '/lidar_right/' in topic else
                            ['imu'] if '/imu/' in topic else
                            ['wheel_left','wheel_right'] if '/wheel/' in topic else
                            ['ultrasonic_address_'+str(row['address'])] if '/ultrasonic/' in topic and 'address' in row else [])
                for logical in logicals:
                    if logical not in handles:
                        continue
                    value = dict(row, logical_source=logical, records_line=line_number, bag_path='bag',
                                 register_index=contract['sources'][logical].get('register_index'))
                    handles[logical].write(json.dumps(value)+'\n')
    finally:
        for handle in handles.values():
            handle.close()
