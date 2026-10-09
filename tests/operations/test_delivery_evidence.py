"""Never turn a denied or malformed process inspection into proof of shutdown."""
import errno
import importlib.util
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('delivery_evidence',Path(__file__).parents[2]/'scripts/collect_delivery_evidence.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('code,expected',[(errno.ENOENT,'DEAD'),(errno.ESRCH,'DEAD'),(errno.EACCES,'UNKNOWN'),(errno.EIO,'UNKNOWN')])
def test_read_error_is_classified(monkeypatch,code,expected):
    def read(_):raise OSError(code,'injected process read failure')
    monkeypatch.setattr(Path,'read_text',read)
    assert module.process_state(10,'55')==expected


def test_identity_reuse_zombie_alive_and_malformed(tmp_path):
    directory=tmp_path/'10';directory.mkdir()
    path=directory/'stat'
    columns=['S']+['0']*18+['55']
    path.write_text('10 (name with spaces) '+' '.join(columns))
    assert module.process_state(10,'55',tmp_path)=='ALIVE'
    assert module.process_state(10,'54',tmp_path)=='REPLACED'
    columns[0]='Z';path.write_text('10 (zombie) '+' '.join(columns))
    assert module.process_state(10,'55',tmp_path)=='ZOMBIE'
    path.write_text('malformed')
    assert module.process_state(10,'55',tmp_path)=='UNKNOWN'
    assert module.process_state(None,None,tmp_path)=='UNKNOWN'
