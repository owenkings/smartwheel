"""Exercise actual CLI failure cleanup in a subprocess with no device access."""
import json
import os
from pathlib import Path
import subprocess
import sys


def test_cli_no_response_exits_nonzero_after_closing_fake_lease(tmp_path):
    root=Path(__file__).resolve().parents[2]
    out=tmp_path/'transactions.jsonl'
    code='''
import sys
from wc_motion import feedback_transport as ft
class FakeLease:
    host_configuration={'fixture':'NO_DEVICE'}
    def __init__(self,*args):pass
    def open(self):return self
    def exchange(self,*args):raise ft.QueryFailure('fixture response timeout', transmitted_bytes=8)
    def close(self):pass
ft.FeedbackSerialLease=FakeLease
raise SystemExit(ft.main(['--config',sys.argv[1],'--output',sys.argv[2],
    '--run-root',sys.argv[3],'--samples','1','--allow-read-queries']))
'''
    env=os.environ.copy();env['PYTHONPATH']=str(root/'src')+os.pathsep+env.get('PYTHONPATH','')
    result=subprocess.run([sys.executable,'-s','-c',code,str(root/'config/wheel_feedback_current.json'),
        str(out),str(tmp_path)],capture_output=True,text=True,env=env,timeout=8)
    assert result.returncode==1, result.stdout+result.stderr
    assert json.loads(result.stdout)['state']=='FAILED'
    rows=[json.loads(x) for x in out.read_text().splitlines()]
    assert rows[-1]['event']=='lease_closed'
    assert [x['event'] for x in rows].count('transaction_failed')==1
    assert not any(x['event']=='transaction_complete' for x in rows)
