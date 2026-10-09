"""Read-only native map file validation and offline RViz viewing.

The PLY/PGM, SQLite and ROS display helpers were promoted from the exercised
check_mapping_app_export integration tool. No sensor or control is started.
"""
import argparse
from contextlib import closing
import hashlib
import json
import math
import os
from pathlib import Path
from .project_paths import ros_setup_path
import re
import shutil
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time

import numpy as np
import yaml


MAX_FILE = 4 * 1024**3
MAX_POINTS = 50_000_000
MAX_CELLS = 20_000_000
# Large native maps on external storage can exceed three minutes.
NATIVE_EXPORT_TIMEOUT_S = 900
SCALARS = {'char':'i1', 'int8':'i1', 'uchar':'u1', 'uint8':'u1',
           'short':'i2', 'int16':'i2', 'ushort':'u2', 'uint16':'u2',
           'int':'i4', 'int32':'i4', 'uint':'u4', 'uint32':'u4',
           'float':'f4', 'float32':'f4', 'double':'f8', 'float64':'f8'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def contained(root, value):
    root = Path(root).absolute()
    value = Path(value)
    path = value if value.is_absolute() else root/value
    require('..' not in path.parts, 'Parent traversal is not allowed')
    require(not any(p.is_symlink() or getattr(p, 'is_junction', lambda: False)()
                    for p in (path, *path.parents)), 'Linked paths are not allowed')
    require(path.resolve().is_relative_to(root.resolve()), 'Path outside expected root: '+str(path))
    return path


def fingerprint(path):
    require(path.is_file() and 0 < path.stat().st_size <= MAX_FILE, 'Missing/oversized file: '+str(path))
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024**2), b''):
            digest.update(block)
    return {'bytes':path.stat().st_size, 'sha256':digest.hexdigest()}


def read_json(path):
    require(path.is_file() and path.stat().st_size <= 10_000_000, 'Missing/oversized JSON: '+str(path))
    return json.loads(path.read_text(encoding='utf-8'),
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))


def ply_header(stream):
    require(stream.readline(256).strip() == b'ply', 'Invalid PLY magic')
    form = None
    elements = []
    for _ in range(10000):
        line = stream.readline(8192)
        require(line.endswith(b'\n') and len(line) < 8192, 'Invalid/oversized PLY header')
        fields = line.decode('ascii').strip().split()
        if not fields or fields[0] in ('comment','obj_info'):
            continue
        if fields[0] == 'format':
            require(len(fields)==3 and fields[2]=='1.0', 'Unsupported PLY version')
            form = fields[1]
        elif fields[0] == 'element':
            require(len(fields)==3 and 0 <= int(fields[2]) <= MAX_POINTS, 'Invalid PLY element count')
            require(fields[1] not in [e['name'] for e in elements], 'Duplicate PLY element')
            elements.append({'name':fields[1], 'count':int(fields[2]), 'properties':[]})
        elif fields[0] == 'property':
            require(elements, 'PLY property without element')
            require((len(fields)==3 and fields[1] in SCALARS) or
                    (len(fields)==5 and fields[1]=='list' and fields[2] in SCALARS
                     and fields[3] in SCALARS and SCALARS[fields[2]][0] in 'iu'), 'Unsupported PLY property')
            prop = (fields[-1], fields[1]) if len(fields)==3 else (fields[-1],fields[3],fields[2])
            require(prop[0] not in [p[0] for p in elements[-1]['properties']], 'Duplicate PLY property')
            elements[-1]['properties'].append(prop)
        elif fields[0] == 'end_header':
            require(form in ('ascii','binary_little_endian','binary_big_endian'), 'Unsupported PLY format')
            require(any(e['name']=='vertex' for e in elements), 'No PLY vertex element')
            return form, elements, stream.tell()
        else:
            raise ValueError('Unknown PLY header item')
    raise ValueError('PLY header exceeds bound')


def ply_blocks(path, chunk_size=65536):
    """Yield real XYZ blocks while checking the complete declared PLY payload."""
    size = path.stat().st_size
    require(0 < size <= MAX_FILE, 'PLY file exceeds bound')
    with path.open('rb') as stream:
        form, elements, _ = ply_header(stream)
        endian = '>' if form == 'binary_big_endian' else '<'
        for element in elements:
            props, count = element['properties'], element['count']
            names = [p[0] for p in props]
            vertex = element['name']=='vertex'
            if vertex:
                require(all(name in names for name in ('x','y','z')), 'PLY lacks XYZ coordinates')
                require(all(len(p)==2 for p in props), 'Vertex list properties are unsupported')
            if not count:
                continue
            require(props, 'Nonempty PLY element has no properties')
            if form != 'ascii' and all(len(p)==2 for p in props):
                dtype = np.dtype([(p[0],endian+SCALARS[p[1]]) for p in props])
                require(stream.tell()+count*dtype.itemsize <= size, 'Truncated binary PLY payload')
                for start in range(0,count,chunk_size):
                    n = min(chunk_size,count-start)
                    block = np.frombuffer(stream.read(n*dtype.itemsize),dtype=dtype)
                    if vertex:
                        yield np.column_stack([block[axis] for axis in ('x','y','z')]).astype(np.float64)
            else:
                pending = []
                for _ in range(count):
                    values = []
                    tokens = None
                    if form == 'ascii':
                        line = stream.readline(1024**2)
                        require(line and len(line)<1024**2, 'Truncated/oversized ASCII PLY row')
                        tokens = iter(line.split())
                    for prop in props:
                        dtype = np.dtype(endian+SCALARS[prop[1]])
                        if len(prop)==2:
                            if tokens is not None:
                                try: value = float(next(tokens))
                                except StopIteration as error: raise ValueError('Short ASCII PLY row') from error
                            else:
                                raw = stream.read(dtype.itemsize)
                                require(len(raw)==dtype.itemsize, 'Short binary PLY property')
                                value = np.frombuffer(raw,dtype=dtype)[0]
                            values.append(value)
                        else:
                            count_type = np.dtype(endian+SCALARS[prop[2]])
                            if tokens is not None:
                                try: list_count = int(next(tokens))
                                except StopIteration as error: raise ValueError('Short PLY list') from error
                            else:
                                raw = stream.read(count_type.itemsize)
                                require(len(raw)==count_type.itemsize, 'Short PLY list count')
                                list_count = int(np.frombuffer(raw,dtype=count_type)[0])
                            require(0<=list_count<=1_000_000, 'PLY list exceeds bound')
                            if tokens is not None:
                                for _ in range(list_count):
                                    try: float(next(tokens))
                                    except StopIteration as error: raise ValueError('Short ASCII PLY list') from error
                            else:
                                require(stream.tell()+list_count*dtype.itemsize<=size, 'Short binary PLY list')
                                stream.seek(list_count*dtype.itemsize,1)
                    if tokens is not None:
                        require(next(tokens,None) is None, 'Extra ASCII PLY values')
                    if vertex:
                        pending.append([values[names.index(axis)] for axis in ('x','y','z')])
                        if len(pending)==chunk_size:
                            yield np.asarray(pending,dtype=float)
                            pending=[]
                if pending:
                    yield np.asarray(pending,dtype=float)
        if form!='ascii':
            require(not stream.read(1), 'Unexpected trailing PLY payload')
        else:
            for extra in iter(lambda: stream.read(65536),b''):
                require(not extra.strip(), 'Unexpected trailing PLY payload')


def inspect_ply(path):
    with path.open('rb') as stream:
        form,elements,_ = ply_header(stream)
    vertices = next(e['count'] for e in elements if e['name']=='vertex')
    count=finite=0
    lo=np.full(3,np.inf); hi=np.full(3,-np.inf)
    for points in ply_blocks(path):
        count+=len(points)
        good=points[np.isfinite(points).all(axis=1)]
        finite+=len(good)
        if len(good):lo=np.minimum(lo,good.min(axis=0));hi=np.maximum(hi,good.max(axis=0))
    require(count==vertices and finite>0, 'PLY has no finite geometry or a count mismatch')
    return {'file':str(path),'format':form,'vertices':vertices,'finite_xyz':finite,
            'nonfinite_xyz':count-finite,'min_m':lo.tolist(),'max_m':hi.tolist(),
            'extent_m':(hi-lo).tolist(),**fingerprint(path)}


def pgm_token(stream):
    token=bytearray()
    while True:
        char=stream.read(1)
        require(bool(char), 'Truncated PGM header')
        if char==b'#' and not token:
            require(len(stream.readline(8192))<8192, 'Oversized PGM comment')
        elif char.isspace():
            if token:
                if char==b'\r':
                    following=stream.read(1)
                    if following!=b'\n':stream.seek(-len(following),1)
                return bytes(token)
        else:
            token.extend(char)
            require(len(token)<=128, 'Oversized PGM token')


def read_pgm(path):
    require(path.stat().st_size<=MAX_FILE, 'PGM exceeds size bound')
    with path.open('rb') as stream:
        form=pgm_token(stream); width=int(pgm_token(stream));height=int(pgm_token(stream));maximum=int(pgm_token(stream))
        require(form in (b'P5',b'P2') and min(width,height)>0 and width*height<=MAX_CELLS
                and 0<maximum<=65535, 'Unsupported/invalid bounded PGM')
        if form==b'P5':
            dtype=np.dtype('u1' if maximum<256 else '>u2')
            raw=stream.read(width*height*dtype.itemsize+1)
            require(len(raw)==width*height*dtype.itemsize,'PGM raster length mismatch')
            pixels=np.frombuffer(raw,dtype=dtype).reshape(height,width)
        else:
            raw=stream.read(MAX_CELLS*8+1)
            require(len(raw)<=MAX_CELLS*8,'ASCII PGM exceeds bound')
            values=re.sub(rb'#[^\r\n]*',b'',raw).split()
            require(len(values)==width*height,'ASCII PGM pixel count mismatch')
            pixels=np.array([int(v) for v in values],dtype=np.int64).reshape(height,width)
        require(((pixels>=0)&(pixels<=maximum)).all(),'PGM sample outside declared range')
    return pixels,maximum


def inspect_grid(specification):
    require(specification.stat().st_size<=1_000_000,'Oversized map YAML')
    meta=yaml.safe_load(specification.read_text(encoding='utf-8'))
    require(isinstance(meta,dict) and meta.get('mode','trinary')=='trinary','Expected trinary map YAML')
    resolution,origin=meta.get('resolution'),meta.get('origin')
    require(type(resolution) in (int,float) and math.isfinite(resolution) and resolution>0,'Invalid metric grid resolution')
    require(isinstance(origin,list) and len(origin)==3 and
            all(type(v) in (int,float) and math.isfinite(v) for v in origin),'Invalid x/y/yaw grid origin')
    free,occupied=meta.get('free_thresh'),meta.get('occupied_thresh')
    require(all(type(v) in (int,float) and math.isfinite(v) for v in (free,occupied))
            and 0<=free<occupied<=1 and meta.get('negate') in (0,1),'Invalid map-server thresholds')
    require(isinstance(meta.get('image'),str),'Missing map image')
    image=contained(specification.parent,meta['image'])
    require(image.suffix.lower()=='.pgm','Native PGM expected')
    pixels,maximum=read_pgm(image)
    probability=pixels.astype(np.float64)/maximum
    if not meta['negate']:probability=1-probability
    grid=np.full(pixels.shape,-1,dtype=np.int8)
    grid[probability>occupied]=100;grid[probability<free]=0
    known=int((grid>=0).sum())
    require(known>0,'Map contains only unknown cells')
    height,width=pixels.shape
    corners=np.array([[0,0],[width*resolution,0],[0,height*resolution],[width*resolution,height*resolution]])
    c,s=math.cos(origin[2]),math.sin(origin[2])
    corners=corners@np.array([[c,s],[-s,c]])+np.asarray(origin[:2])
    stats={'yaml':str(specification),'image':str(image),'width':width,'height':height,
           'resolution_m':resolution,'origin_x_y_yaw':origin,'image_max_value':maximum,
           'occupied_cells':int((grid==100).sum()),'free_cells':int((grid==0).sum()),
           'unknown_cells':int((grid<0).sum()),'world_xy_min_m':corners.min(axis=0).tolist(),
           'world_xy_max_m':corners.max(axis=0).tolist(),
           'yaml_fingerprint':fingerprint(specification),'image_fingerprint':fingerprint(image)}
    return stats,np.flipud(grid).copy(),meta


def sqlite_read(path, callback):
    for suffix in ('-wal','-journal'):
        require(not Path(str(path)+suffix).exists(),'Database not closed: '+str(path))
    require(path.is_file(),'Missing SQLite database')
    before=fingerprint(path)
    with closing(sqlite3.connect(path.as_uri()+'?mode=ro',uri=True)) as connection:
        connection.execute('PRAGMA query_only=ON')
        require(connection.execute('PRAGMA integrity_check').fetchall()==[('ok',)],'SQLite integrity failed: '+str(path))
        result=callback(connection)
    require(fingerprint(path)==before,'Database changed during read-only audit')
    return result,before


def node_signature(connection):
    rows=connection.execute('SELECT id,stamp FROM Node ORDER BY id').fetchall()
    require(0<len(rows)<=1_000_000,'Native database has no/bounded-overflow nodes')
    require(all(math.isfinite(row[1]) for row in rows),'Nonfinite node timestamps')
    return {'nodes':len(rows),'first_id':rows[0][0],'last_id':rows[-1][0],
            'id_stamp_sha256':hashlib.sha256(json.dumps(rows,separators=(',',':')).encode()).hexdigest()}


def offline_rviz(report=None):
    qos={'Depth':1,'History Policy':'Keep Last','Reliability Policy':'Reliable','Durability Policy':'Transient Local'}
    view={'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'},{'Class':'rviz_common/Views','Name':'Views'}],
        'Visualization Manager':{'Class':'','Enabled':True,'Name':'root',
            'Global Options':{'Fixed Frame':'mapping_map','Background Color':'24; 29; 36','Frame Rate':20},
            'Displays':[{'Class':'rviz_default_plugins/PointCloud2','Name':'Saved 3D map','Enabled':True,'Value':True,
                'Color Transformer':'AxisColor','Axis':'Z','Position Transformer':'XYZ','Style':'Points','Size (Pixels)':2,
                'Topic':{'Value':'/wc_mapping/offline/cloud_map',**qos}},
                {'Class':'rviz_default_plugins/Map','Name':'Saved 2D map','Enabled':True,'Value':True,'Alpha':.55,
                 'Draw Behind':True,'Use Timestamp':False,'Topic':{'Value':'/wc_mapping/offline/grid_map',**qos}}],
            'Tools':[{'Class':'rviz_default_plugins/MoveCamera'},{'Class':'rviz_default_plugins/Select'}],
            'Views':{'Current':{'Class':'rviz_default_plugins/Orbit','Distance':10,'Pitch':.7,'Yaw':.8,
                               'Target Frame':'<Fixed Frame>','Focal Point':{'X':0,'Y':0,'Z':0}},
                     'Saved':[{'Class':'rviz_default_plugins/TopDownOrtho','Name':'2D top','Scale':50,'Angle':0,'X':0,'Y':0,'Target Frame':'<Fixed Frame>'}]}},
        'Window Geometry':{'Width':1280,'Height':900}}
    if report is not None:
        cloud=report['clouds'][0]
        low=np.asarray(cloud['min_m'],dtype=float)
        high=np.asarray(cloud['max_m'],dtype=float)
        require(low.shape==(3,) and high.shape==(3,) and np.isfinite(low).all()
                and np.isfinite(high).all() and (low<=high).all(), 'Invalid saved-cloud view bounds')
        # The displayed grid lies at Z=0. Include it when it extends beyond the
        # cloud; this only positions the camera, never transforms saved points.
        if report.get('maps'):
            grid=report['maps'][0]
            if 'world_xy_min_m' in grid and 'world_xy_max_m' in grid:
                grid_low=np.asarray(grid['world_xy_min_m'],dtype=float)
                grid_high=np.asarray(grid['world_xy_max_m'],dtype=float)
                require(grid_low.shape==(2,) and grid_high.shape==(2,) and np.isfinite(grid_low).all()
                        and np.isfinite(grid_high).all() and (grid_low<=grid_high).all(),
                        'Invalid saved-grid view bounds')
                low=np.minimum(low,np.r_[grid_low,0.])
                high=np.maximum(high,np.r_[grid_high,0.])
        extent=high-low
        center=low+extent*.5
        diagonal=float(np.linalg.norm(extent))
        require(np.isfinite(center).all() and math.isfinite(diagonal), 'Saved view bounds exceed numeric range')
        views=view['Visualization Manager']['Views']
        views['Current']['Focal Point']=dict(zip(('X','Y','Z'),center.tolist()))
        # A sphere of radius diagonal/2 fits with margin in the normal RViz
        # perspective view. Preserve the existing oblique pitch/yaw.
        views['Current']['Distance']=max(3.,1.65*diagonal)
        # RViz TopDownOrtho Scale is pixels/metre (viewport size / Scale).
        # Reserve a conservative 640x400 map area within the initial window's
        # panels, then leave 15% margin. Later user resizing/zoom stays manual.
        top=views['Saved'][0]
        top.update(X=float(center[0]),Y=float(center[1]),
                   Scale=.85*min(640./max(float(extent[0]),1.),400./max(float(extent[1]),1.)))
    return view


def publish_saved(report, duration, max_points):
    """Publish a bounded deterministic subset of the saved PLY, never fake points."""
    os.environ['ROS_DOMAIN_ID']='84';os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import PointCloud2,PointField
    from nav_msgs.msg import OccupancyGrid
    cloud=report['clouds'][0]
    step=max(1,math.ceil(cloud['vertices']/max_points));blocks=[];seen=0
    for points in ply_blocks(Path(cloud['file'])):
        selected=points[(-seen)%step::step];seen+=len(points)
        blocks.append(selected[np.isfinite(selected).all(axis=1)])
    xyz=np.ascontiguousarray(np.concatenate(blocks),dtype='<f4')
    require(len(xyz)>0 and np.isfinite(xyz).all(),'Saved coordinates cannot be represented as finite viewer FLOAT32')
    grid_stats,grid,meta=inspect_grid(Path(report['maps'][0]['yaml']))
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('saved_mapping_files_viewer')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    point_pub=node.create_publisher(PointCloud2,'/wc_mapping/offline/cloud_map',qos)
    map_pub=node.create_publisher(OccupancyGrid,'/wc_mapping/offline/grid_map',qos)
    message=PointCloud2();message.header.frame_id='mapping_map';message.height=1;message.width=len(xyz)
    message.fields=[PointField(name=axis,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,axis in enumerate('xyz')]
    message.point_step=12;message.row_step=12*len(xyz);message.is_dense=True;message.is_bigendian=False;message.data=xyz.tobytes()
    occupancy=OccupancyGrid();occupancy.header.frame_id='mapping_map';occupancy.info.resolution=float(meta['resolution'])
    occupancy.info.width=grid_stats['width'];occupancy.info.height=grid_stats['height']
    x,y,yaw=meta['origin'];occupancy.info.origin.position.x=float(x);occupancy.info.origin.position.y=float(y)
    occupancy.info.origin.orientation.z=math.sin(yaw/2);occupancy.info.origin.orientation.w=math.cos(yaw/2)
    occupancy.data=grid.reshape(-1).tolist()
    stopped=[False]
    old={sig:signal.signal(sig,lambda *_:stopped.__setitem__(0,True)) for sig in (signal.SIGINT,signal.SIGTERM)}
    print(json.dumps({'status':'OFFLINE_FILES_PUBLISHED','domain':84,'points_shown':len(xyz),
                      'source_vertices':cloud['vertices'],'stride':step,'duration_s':duration}),flush=True)
    started=time.monotonic();last=0
    try:
        while not stopped[0] and rclpy.ok() and (duration == 0 or time.monotonic()-started<duration):
            if time.monotonic()-last>=2:
                stamp=node.get_clock().now().to_msg();message.header.stamp=stamp;occupancy.header.stamp=stamp
                point_pub.publish(message);map_pub.publish(occupancy);last=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for sig,handler in old.items():signal.signal(sig,handler)


def map_source(value):
    """Resolve an explicitly chosen saved directory or database, including outside the project."""
    path = Path(value).expanduser().resolve(strict=True)
    if path.is_file():
        require(path.suffix.lower() == '.db', 'Choose a map directory, export directory or .db file')
        return {'path': path, 'database': path, 'geometry': None}
    require(path.is_dir(), 'Saved map path is not a directory or database')
    geometry = path/'export' if (path/'export').is_dir() else path
    clouds = sorted(geometry.glob('*.ply'))
    maps = sorted(geometry.glob('*.yaml'))
    has_geometry = bool(clouds and maps)
    databases = [path/'slam/rtabmap.db', path/'rtabmap.db', path/'native_export_copy.db']
    database = next((p.resolve(strict=True) for p in databases if p.is_file()), None)
    require(has_geometry or database is not None,
            'No saved PLY/YAML pair or slam/rtabmap.db found in '+str(path))
    return {'path': path, 'database': database, 'geometry': geometry if has_geometry else None}


def inspect_geometry(directory):
    """Validate saved geometry, without requiring the old bag or declaring a session successful."""
    directory = Path(directory).resolve(strict=True)
    clouds = sorted(directory.glob('*.ply'))
    maps = []
    for path in sorted(directory.glob('*.yaml')):
        require(path.stat().st_size <= 1_000_000, 'Oversized map YAML')
        description = yaml.safe_load(path.read_text(encoding='utf-8'))
        if isinstance(description, dict) and 'image' in description:
            maps.append(path)
    require(len(clouds) == 1 and len(maps) == 1,
            'Choose an export directory containing exactly one PLY and one map YAML')
    return {'clouds': [inspect_ply(clouds[0])], 'maps': [inspect_grid(maps[0])[0]]}


def inspect_source(source):
    report = {'status': 'SAVED_FILES_READABLE', 'source': str(source['path']),
              'scope': 'Saved geometry/database only; not full-session, coverage or accuracy acceptance',
              'hardware_started': False, 'navigation_validated': False,
              'geometry_accuracy_validated': False}
    if source['database'] is not None:
        signature, digest = sqlite_read(source['database'], node_signature)
        report['native_database'] = {'file': str(source['database']), **signature, **digest}
    if source['geometry'] is not None:
        report.update(inspect_geometry(source['geometry']))
    report['requires_temporary_export'] = source['geometry'] is None
    return report


def export_database(source, report, temporary, *, project_root, runner=subprocess.run):
    """Export a private SQLite backup. No native tool is given the source database."""
    from .cli import ros_command
    database = source['database']
    require(database is not None, 'No source database for temporary export')
    temporary = Path(temporary).resolve(strict=True)
    require(shutil.disk_usage(temporary).free >= database.stat().st_size+512*1024**2,
            'Temporary export needs source database size plus 512 MiB free space')
    output = temporary/'export'
    output.mkdir()
    copy = output/'native_export_copy.db'
    before = report['native_database']['sha256']
    try:
        with closing(sqlite3.connect(database.as_uri()+'?mode=ro', uri=True)) as original:
            original.execute('PRAGMA query_only=ON')
            with closing(sqlite3.connect(copy)) as destination:
                original.backup(destination)
        require(fingerprint(database)['sha256'] == before, 'Original database changed during copying')
        command = [str(ros_setup_path().parent/'bin/rtabmap-export'), '--scan', '--cloud', '--map', '--poses',
                   '--poses_format', '11', '--opt', '2', '--voxel', '.03', '--output', 'map_3d',
                   '--output_dir', str(output), str(copy)]
        report['native_export'] = {'timeout_s': NATIVE_EXPORT_TIMEOUT_S,
                                   'source_database_modified': False}
        started = time.monotonic()
        try:
            with (output/'export.log').open('xb') as log:
                result = runner(ros_command(command), cwd=project_root, stdout=log,
                                stderr=subprocess.STDOUT, timeout=NATIVE_EXPORT_TIMEOUT_S)
        except subprocess.TimeoutExpired as error:
            detail = (output/'export.log').read_text(errors='replace')[-3000:]
            raise RuntimeError('地图导出超过 '+str(NATIVE_EXPORT_TIMEOUT_S)
                               +' 秒；已关闭的原始地图数据库仍保留，可单独重试导出。导出日志末尾：'
                               +(detail or '（无输出）')) from error
        report['native_export'].update(elapsed_s=time.monotonic()-started,
                                       exit_code=result.returncode)
        if result.returncode:
            detail = (output/'export.log').read_text(errors='replace')[-3000:]
            raise RuntimeError('Temporary native export failed: '+detail)
        report.update(inspect_geometry(output))
        report['temporary_export'] = True
    finally:
        require(fingerprint(database)['sha256'] == before,
                'Original database changed during temporary export')


def verify_unchanged(report):
    """A view is not allowed to alter any original file it validated."""
    source = report.get('native_database')
    if source:
        require(fingerprint(Path(source['file']))['sha256'] == source['sha256'],
                'Original database changed while viewing')
    for cloud in report.get('clouds', []):
        require(fingerprint(Path(cloud['file']))['sha256'] == cloud['sha256'],
                'Saved PLY changed while viewing')
    for grid in report.get('maps', []):
        for key, digest in (('yaml', 'yaml_fingerprint'), ('image', 'image_fingerprint')):
            require(fingerprint(Path(grid[key])) == grid[digest], 'Saved map grid changed while viewing')


def stop_owned(child):
    """Only stop and reap a direct child created by this viewer."""
    if child is None or child.poll() is not None:
        return
    for action, timeout in ((lambda: child.send_signal(signal.SIGINT), 10),
                            (child.terminate, 3), (child.kill, 2)):
        action()
        try:
            child.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError('Owned offline viewer child did not stop')


def run_view(report, temporary, *, project_root, renderer='software', max_points=1_000_000,
             spawn=subprocess.Popen, environment=None, wait=time.sleep, stop_requested=None):
    from .cli import ros_command
    from .sensor_viewer import desktop_environment, confirm_display
    if environment is None:
        environment = desktop_environment()
        confirm_display(environment)
    env = dict(environment)
    env.update(PYTHONNOUSERSITE='1', PYTHONPATH=str(Path(project_root)/'src'),
               ROS_DOMAIN_ID='84', ROS_LOCALHOST_ONLY='1')
    if renderer == 'software':
        # Only these owned children receive Mesa selection; never change the
        # caller's environment or the desktop/system renderer configuration.
        env.update(LIBGL_ALWAYS_SOFTWARE='1', GALLIUM_DRIVER='llvmpipe', __GLX_VENDOR_LIBRARY_NAME='mesa')
    binary = Path(project_root)/'install/main/wc_bringup/lib/wc_bringup/mapping_rviz'
    require(binary.is_file(), 'Installed mapping_rviz is missing: '+str(binary))
    temporary = Path(temporary)
    rviz = temporary/'saved_map.rviz'
    rviz.write_text(yaml.safe_dump(offline_rviz(report), sort_keys=False), encoding='utf-8')
    payload = temporary/'files.json'
    payload.write_text(json.dumps(report, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    component = [sys.executable, '-s', '-m', 'wc_runtime.component', '--parent', str(os.getpid()), '--']
    publisher_command = component+[sys.executable, '-s', '-m', 'wc_runtime.map_viewer',
                                    '--publish-report', str(payload), '--max-view-points', str(max_points)]
    gui_command = component+[str(binary), '-d', str(rviz), '--ros-args', '-r', '__node:=saved_map_view']
    publisher = gui = None
    stopped = [False]
    previous = {}
    if stop_requested is None:
        previous = {sig: signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
                    for sig in (signal.SIGINT, signal.SIGTERM)}
        stop_requested = lambda: stopped[0]
    try:
        publisher = spawn(ros_command(publisher_command), cwd=project_root, env=env,
                          stdin=subprocess.DEVNULL, start_new_session=True)
        if not stop_requested():
            gui = spawn(ros_command(gui_command), cwd=project_root, env=env,
                        stdin=subprocess.DEVNULL, start_new_session=True)
        while not stop_requested():
            code = gui.poll()
            if code is not None:
                require(code == 0, 'Offline RViz exited with code '+str(code))
                return {'status': 'VIEW_CLOSED', 'reason': 'RVIZ_WINDOW_CLOSED'}
            require(publisher.poll() is None, 'Saved-map publisher exited before window close')
            wait(.1)
        return {'status': 'VIEW_CLOSED', 'reason': 'USER_STOP_REQUEST'}
    finally:
        errors = []
        for child in (gui, publisher):
            try:
                stop_owned(child)
            except Exception as error:
                errors.append(str(error))
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if errors:
            raise RuntimeError('; '.join(errors))


def main(argv=None, *, project_root=None):
    from .project_paths import project_root as discover_project
    project_root = discover_project() if project_root is None else Path(project_root)
    parser = argparse.ArgumentParser(description='只读查看已保存地图；支持会话目录、export 目录或 RTAB-Map .db。')
    parser.add_argument('path', nargs='?', help='已保存的地图目录或数据库，可以位于工程外')
    parser.add_argument('--check', action='store_true', help='只读检查并打印摘要，不打开窗口、不导出副本')
    parser.add_argument('--renderer', choices=('software', 'system'), default='software', help='RViz 渲染方式')
    parser.add_argument('--max-view-points', type=int, default=1_000_000, help='显示点数上限，默认 1000000；不改原地图')
    parser.add_argument('--publish-report', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not 1000 <= args.max_view_points <= 2_000_000:
        parser.error('--max-view-points must be in [1000, 2000000]')
    if args.publish_report:
        require(not args.path and not args.check, 'Internal publisher cannot be combined with a map path')
        publish_saved(read_json(args.publish_report), 0, args.max_view_points)
        return 0
    if not args.path:
        parser.error('请选择地图目录或数据库 path')
    try:
        # Preserve explicit external read-only maps while resolving relocated
        # project data paths through the UUID-checked storage policy.
        from .storage_policy import StoragePolicy
        storage = StoragePolicy(project_root)
        requested = Path(args.path).expanduser()
        if (not requested.is_absolute() or requested.is_relative_to(storage.project_root)
                or requested.is_relative_to(storage.original_project_root)
                or (storage.enabled and requested.is_relative_to(storage.archive_root))):
            requested = storage.resolve(requested)
        source = map_source(requested)
        report = inspect_source(source)
        if args.check:
            print(json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
            return 0
        from .cli import target
        target()
        import fcntl
        locks = Path(project_root)/'.phase1_runtime/locks'
        locks.mkdir(parents=True, exist_ok=True)
        with (locks/'domain-84-offline-review.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError('已有离线地图查看占用 domain 84，请先关闭该查看窗口') from error
            with tempfile.TemporaryDirectory(prefix='wheelchair_map_view_') as temporary:
                if source['geometry'] is None:
                    export_database(source, report, temporary, project_root=project_root)
                print(json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
                try:
                    result = run_view(report, temporary, project_root=project_root,
                                      renderer=args.renderer, max_points=args.max_view_points)
                finally:
                    verify_unchanged(report)
        print(json.dumps({**result, 'source_unchanged': True}, ensure_ascii=False), flush=True)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError, sqlite3.Error, subprocess.SubprocessError, yaml.YAMLError) as error:
        print(json.dumps({'status': 'VIEW_FAILED', 'reason': str(error)}, ensure_ascii=False), flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())


