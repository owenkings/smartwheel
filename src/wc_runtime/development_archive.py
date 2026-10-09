"""Create structured development records on the selected, verified data volume."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import uuid

from .project_paths import project_root, shared_lock_root
from .storage_policy import StoragePolicy

CATEGORIES = {
    'tasks': '任务要求、实施计划与清单。',
    'prompts': '历史任务输入原件；封存内容不参与程序运行。',
    'handoffs': '阶段交接与后续工作记录。',
    'reports': '审查、实验和部署结论。',
    'validation': '测试日志、截图、结果清单与对照证据。',
    'snapshots': '不可变源码、配置和工作区快照。',
    'backups': '回滚材料及可恢复的版本历史。',
}


@contextmanager
def _index_lock(root):
    import fcntl
    key = hashlib.sha256(str(root).encode('utf-8')).hexdigest()[:20]
    path = shared_lock_root() / ('development-' + key + '.lock')
    with path.open('a+b') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def _new_text(path, text):
    with path.open('x', encoding='utf-8') as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())


def create_task(root, name, *, now=None, storage_policy=None):
    """Return a new task directory; never overwrite an existing task or fall back."""
    if not isinstance(name, str) or re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', name) is None:
        raise ValueError('Use a task name with 1-64 letters, digits, underscores or hyphens')
    policy = storage_policy if storage_policy is not None else StoragePolicy(root)
    if policy.project_root != Path(root).absolute():
        raise ValueError('Development archive policy belongs to a different project')
    archive = policy.resolve('dev_archive')
    policy.check()
    archive.mkdir(parents=True, exist_ok=True)
    instant = now or datetime.now(timezone.utc)
    stamp = instant.astimezone(timezone.utc).strftime('%Y%m%d_%H%M%S')
    with _index_lock(archive):
        policy.check()
        readme = archive / 'README.md'
        if not readme.exists():
            _new_text(readme, '# 开发归档\n\n每次任务使用独立子目录，时间采用 UTC。index.jsonl 为追加式任务索引。\n\n'
                      '源码、测试源码和当前手册保留在项目；历史过程资料与验证产物保存在此处。'
                      '原始录包、地图和运行锁不迁入开发归档。快照内部保持原始字节，仅在外层添加说明。\n')
        index = archive / 'index.jsonl'
        if index.is_symlink() or readme.is_symlink():
            raise ValueError('Development archive metadata must not be linked')
        base = stamp + '_' + name
        task = archive / base
        number = 0
        while task.exists():
            number += 1
            task = archive / (base + '_' + str(number).zfill(2))
        task.mkdir()
        record = {'schema_version': 1, 'task': task.name, 'name': name,
                  'created_utc': instant.astimezone(timezone.utc).isoformat(),
                  'status': 'OPEN', 'project_root': str(Path(root).absolute()), 'files': []}
        for category, description in CATEGORIES.items():
            folder = task / category
            folder.mkdir()
            _new_text(folder / 'README.md', '# ' + category + '\n\n' + description + '\n\n'
                      '新增资料按本目录职责归档，记录来源与验证结果；同名文件不覆盖。'
                      '原始快照或证据内部不改写。返回[任务说明](../README.md)。\n')
        links = '\n'.join('- [' + k + '](' + k + '/README.md)：' + v for k, v in CATEGORIES.items())
        _new_text(task / 'README.md', '# ' + name + '\n\n' + links + '\n\n'
                  'manifest.json 保存本任务身份和文件迁移清单；完成后补充结论与实际验证范围。\n')
        _new_text(task / 'manifest.json', json.dumps(record, ensure_ascii=False, indent=2) + '\n')
        policy.check()
        with index.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps({k: v for k, v in record.items() if k != 'files'}, ensure_ascii=False) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
    return task


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, default=project_root())
    parser.add_argument('name', help='Short task name, for example sensor_review')
    args = parser.parse_args(argv)
    print(create_task(args.project_root, args.name))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
