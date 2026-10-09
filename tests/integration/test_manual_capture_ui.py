"""Real native Qt entry + synthetic private socket; no ROS or physical devices."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time

import pytest

from wc_runtime.mapping_control_paths import manual_socket_path

ROOT=Path(__file__).resolve().parents[2]
EXE=ROOT/'install/main/wc_bringup/lib/wc_bringup/manual_capture_ui'
pytestmark=pytest.mark.skipif(os.name!='posix' or not EXE.is_file(),reason='requires built native Qt executable on Linux')

def metadata(root):
    (root/'configuration').mkdir()
    (root/'configuration/manual_runtime.json').write_text(json.dumps({'session_id':'synthetic_ui','source_mode':'real',
        'status':'EXPERIMENT','manual_controls':{'arm_allowed':True,'interaction_policy':'hybrid_manual'}}))
    # The native entry validates the per-project short socket directory,
    # even when the backend and its messages are entirely synthetic.
    project=root/'synthetic_project'; project.mkdir(mode=0o700)
    socket_path=manual_socket_path(project,root,'synthetic_ui')
    socket_path.parent.mkdir(mode=0o700)
    (root/'capture_manifest.json').write_text(json.dumps({'session_id':'synthetic_ui','status':'RECORDING','manual_drive':True,
        'project_root':str(project),'manual_socket_directory':str(socket_path.parent)}))
    return socket_path

def cleanup_socket(socket_path):
    # These directories belong to the unique synthetic project above;
    # rmdir refuses unexpected extra files instead of deleting them.
    socket_path.unlink(missing_ok=True)
    socket_path.parent.rmdir()
    socket_path.parent.parent.rmdir()

def test_native_entry_accepts_only_matching_capture_and_sigint_closes_cleanly():
    with tempfile.TemporaryDirectory(prefix='wc_ui_',dir='/tmp') as folder:
        root=Path(folder);socket_path=metadata(root)
        server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        server.bind(str(socket_path));os.chmod(socket_path,0o600);server.listen(1);server.settimeout(5)
        messages=[];stop=threading.Event();errors=[]
        def serve():
            try:
                peer,_=server.accept();peer.settimeout(.05)
                pending=b''
                with peer:
                    while not stop.is_set():
                        try:
                            block=peer.recv(65536)
                            if not block:break
                            pending+=block
                            while b'\n' in pending:
                                line,pending=pending.split(b'\n',1);messages.append(json.loads(line))
                        except socket.timeout:pass
                        status={'type':'status','session_id':'synthetic_ui','state':'READY','reason':'SYNTHETIC_ONLY',
                            'arm_allowed':True,'arm_generation':0,'max_linear_m_s':.1,'max_angular_rad_s':.2,
                            'interaction_policy':'hybrid_manual','push_mode':False,'push_mode_requested':False}
                        try:peer.sendall(json.dumps(status).encode()+b'\n')
                        except (BrokenPipeError,ConnectionResetError):break
            except Exception as exc:errors.append(str(exc))
        thread=threading.Thread(target=serve,daemon=True);thread.start()
        process=subprocess.Popen([str(EXE),'--session-root',folder,'--session-id','synthetic_ui'],
            env={**os.environ,'QT_QPA_PLATFORM':'offscreen'},stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            deadline=time.monotonic()+5
            while time.monotonic()<deadline and not any(m.get('type')=='keys' for m in messages) and process.poll() is None:
                time.sleep(.02)
            assert process.poll() is None
            assert any(m.get('type')=='hello' for m in messages)
            assert any(m.get('type')=='keys' and type(m.get('foreground')) is bool for m in messages)
            assert all(m.get('keys',[])==[] for m in messages)
            process.send_signal(signal.SIGINT)
            stdout,stderr=process.communicate(timeout=5)
            assert process.returncode==0,stderr
            assert any(m.get('type') in ('disarm','close') for m in messages)
        finally:
            if process.poll() is None:process.kill();process.wait()
            stop.set();server.close();thread.join(2)
            cleanup_socket(socket_path)

def test_native_entry_rejects_mismatched_session_before_socket_use():
    with tempfile.TemporaryDirectory(prefix='wc_ui_',dir='/tmp') as folder:
        socket_path=metadata(Path(folder))
        try:
            result=subprocess.run([str(EXE),'--session-root',folder,'--session-id','another_session'],
                env={**os.environ,'QT_QPA_PLATFORM':'offscreen'},capture_output=True,text=True,timeout=5)
            assert result.returncode==2
            assert not (Path(folder)/'manual.sock').exists()
        finally:
            cleanup_socket(socket_path)
