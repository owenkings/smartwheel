#!/usr/bin/env python3
"""Show a saved synthetic map in actual Orin RViz; capture only owned windows.

This checks real DDS subscriptions and saves genuine window screenshots for
separate visual review. It does not fabricate TF or establish field map quality.
"""
import argparse
import ctypes as C
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys
import time
import zlib


def desktop_environment():
    env = os.environ.copy()
    for comm in Path('/proc').glob('[0-9]*/comm'):
        try:
            if comm.stat().st_uid == os.getuid() and comm.read_text().strip() == 'gnome-shell':
                for item in (comm.parent/'environ').read_bytes().split(b'\0'):
                    if item.startswith((b'DISPLAY=', b'XAUTHORITY=', b'DBUS_SESSION_BUS_ADDRESS=')):
                        key, value = item.decode().split('=', 1); env[key] = value
        except (OSError, UnicodeError):
            pass
    if not env.get('DISPLAY') or not env.get('XAUTHORITY'):
        raise RuntimeError('No authorized X11 session; actual RViz test BLOCKED')
    return env


def window_metadata(window_id, properties, geometry):
    """Parse only X11 metadata; titles and screenshots are not selection evidence."""
    def integer(pattern, text):
        match = re.search(pattern, text, re.MULTILINE)
        return int(match[1]) if match else None
    types = re.search(r'^_NET_WM_WINDOW_TYPE\(ATOM\)\s*=\s*(.*)$', properties, re.MULTILINE)
    transient = re.search(r'^WM_TRANSIENT_FOR.*?0x([0-9a-fA-F]+)', properties, re.MULTILINE)
    return dict(window_id=window_id,
        pid=integer(r'^_NET_WM_PID\(CARDINAL\)\s*=\s*(\d+)', properties),
        types=re.findall(r'_NET_WM_WINDOW_TYPE_[A-Z_]+', types[1]) if types else [],
        mapped=bool(re.search(r'Map State:\s*IsViewable\b', geometry)),
        normal_state=bool(re.search(r'window state:\s*Normal\b', properties)),
        hidden='_NET_WM_STATE_HIDDEN' in properties,
        modal='_NET_WM_STATE_MODAL' in properties,
        transient_for=int(transient[1], 16) if transient else 0,
        x=integer(r'Absolute upper-left X:\s*(-?\d+)', geometry),
        y=integer(r'Absolute upper-left Y:\s*(-?\d+)', geometry),
        width=integer(r'^\s*Width:\s*(\d+)', geometry),
        height=integer(r'^\s*Height:\s*(\d+)', geometry))


def select_main_window(candidates, owned_pids, display_size):
    """Choose the largest visible NORMAL client, never a same-PID dialog."""
    screen_width, screen_height = display_size
    valid = []
    for item in candidates:
        if (item['pid'] not in owned_pids or item['types'] != ['_NET_WM_WINDOW_TYPE_NORMAL']
                or not item['mapped'] or not item['normal_state'] or item['hidden']
                or item['modal'] or item['transient_for']
                or any(item[key] is None for key in ('x', 'y', 'width', 'height'))):
            continue
        visible_width = max(0, min(item['x'] + item['width'], screen_width) - max(item['x'], 0))
        visible_height = max(0, min(item['y'] + item['height'], screen_height) - max(item['y'], 0))
        # Evidence framing floor, not a statement about point-cloud quality.
        if visible_width >= 640 and visible_height >= 360:
            valid.append(dict(item, visible_area=visible_width * visible_height))
    if not valid:
        raise RuntimeError('No mapped owned NORMAL main window of at least 640x360 visible pixels; screenshot refused')
    return max(valid, key=lambda item: (item['visible_area'], item['width'] * item['height'], int(item['window_id'], 16)))


def png_dimensions(data):
    """Check the native screenshot's PNG header without decoding/repainting it."""
    if (len(data) < 33 or data[:8] != b'\x89PNG\r\n\x1a\n'
            or data[8:16] != struct.pack('>I', 13) + b'IHDR'
            or struct.unpack('>I', data[29:33])[0] != zlib.crc32(data[12:29]) & 0xffffffff):
        raise RuntimeError('Invalid native PNG screenshot header')
    width, height = struct.unpack('>II', data[16:24])
    if not 0 < width <= 16384 or not 0 < height <= 16384:
        raise RuntimeError('Invalid native PNG screenshot dimensions')
    return width, height


def require_active_main_window(properties, selected_id):
    match = re.search(r'0x[0-9a-fA-F]+', properties)
    if not match or int(match[0], 16) != int(selected_id, 16):
        raise RuntimeError('Selected owned NORMAL main window is not active; screenshot refused')


def request_window_activation(window_id, env):
    """Ask the WM to present a separately verified owned client; no input keys."""
    class Data(C.Union):_fields_=[('b',C.c_char*20),('s',C.c_short*10),('l',C.c_long*5)]
    class Client(C.Structure):
        _fields_=[('type',C.c_int),('serial',C.c_ulong),('send_event',C.c_int),('display',C.c_void_p),
                  ('window',C.c_ulong),('message_type',C.c_ulong),('format',C.c_int),('data',Data)]
    class Event(C.Union):_fields_=[('client',Client),('pad',C.c_long*24)]
    lib=C.CDLL('libX11.so.6');lib.XOpenDisplay.argtypes=[C.c_char_p];lib.XOpenDisplay.restype=C.c_void_p
    lib.XDefaultRootWindow.argtypes=[C.c_void_p];lib.XDefaultRootWindow.restype=C.c_ulong
    lib.XInternAtom.argtypes=[C.c_void_p,C.c_char_p,C.c_int];lib.XInternAtom.restype=C.c_ulong
    lib.XSendEvent.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_long,C.POINTER(Event)]
    lib.XFlush.argtypes=[C.c_void_p];lib.XCloseDisplay.argtypes=[C.c_void_p]
    os.environ.update({k:env[k] for k in ('DISPLAY','XAUTHORITY')})
    display=lib.XOpenDisplay(env['DISPLAY'].encode())
    if not display:raise RuntimeError('Cannot open authorized desktop for owned activation')
    try:
        event=Event();event.client.type=33;event.client.send_event=1;event.client.display=display
        event.client.window=int(window_id,16);event.client.message_type=lib.XInternAtom(display,b'_NET_ACTIVE_WINDOW',False)
        event.client.format=32;event.client.data.l[0]=2
        if not lib.XSendEvent(display,lib.XDefaultRootWindow(display),False,(1<<20)|(1<<19),C.byref(event)):
            raise RuntimeError('Owned client activation request failed')
        lib.XFlush(display)
    finally:lib.XCloseDisplay(display)


# Separate process: it neither enters RViz's Qt event loop nor asks its widgets
# to redraw. QScreen reads the selected X11 client's existing native pixels.
_INDEPENDENT_QT_CAPTURE = r'''
import json, sys
from PyQt5.QtCore import QFile, QIODevice
from PyQt5.QtGui import QGuiApplication
app = QGuiApplication(['wc_owned_window_capture'])
if app.platformName() != 'xcb':
    raise RuntimeError('Independent native screenshot requires the X11 xcb platform')
screen = app.primaryScreen()
if screen is None:
    raise RuntimeError('No authorized Qt screen')
pixmap = screen.grabWindow(int(sys.argv[1], 16))
if pixmap.isNull():
    raise RuntimeError('QScreen.grabWindow returned a null native pixmap')
output = QFile(sys.argv[2])
if not output.open(QIODevice.WriteOnly | QIODevice.NewOnly):
    raise RuntimeError('Cannot create exclusive Qt screenshot: ' + output.errorString())
try:
    if not pixmap.save(output, 'PNG') or not output.flush():
        raise RuntimeError('Cannot save native Qt screenshot')
finally:
    output.close()
print(json.dumps(dict(pixel_width=pixmap.width(), pixel_height=pixmap.height(),
    device_pixel_ratio=pixmap.devicePixelRatio(), qt_platform=app.platformName(),
    screen_name=screen.name(), window_id=sys.argv[1])))
'''


def capture_independent_qt(filename, window_id, env, selected, verify_window, identity):
    """Capture native Qt evidence; the caller decides whether it is required."""
    screenshot = filename.with_name(filename.stem + '_qt.png')
    metadata = filename.with_name(filename.stem + '_qt.json')
    for path in (screenshot, metadata):
        if path.exists() or path.is_symlink():
            raise RuntimeError('Independent Qt screenshot evidence already exists')
    # Reserve the sidecar before starting the child, never overwrite evidence.
    with metadata.open('x', encoding='utf-8') as stream:
        record = dict(status='FAIL', screenshot=str(screenshot), metadata=str(metadata),
            capture_method='independent_pyqt5_qscreen_grabWindow_owned_x11_client',
            window_id=window_id, window=selected, **identity)
        started = time.monotonic()
        ownership_error = None
        try:
            try:
                verify_window()
            except Exception as error:
                ownership_error = error
                raise
            record['window_verified_before'] = True
            try:
                completed = subprocess.run(
                    [sys.executable, '-c', _INDEPENDENT_QT_CAPTURE, window_id, str(screenshot)],
                    env=env, timeout=10, check=True, capture_output=True, text=True)
            finally:
                # Also check ownership when the capture process times out/fails.
                try:
                    verify_window()
                    record['window_verified_after'] = True
                except Exception as error:
                    ownership_error = error
                    raise
            native = json.loads(completed.stdout)
            ratio = native.get('device_pixel_ratio')
            if (not isinstance(ratio, (float, int)) or isinstance(ratio, bool)
                    or not math.isfinite(ratio) or not 0 < ratio <= 8
                    or native.get('window_id') != window_id or native.get('qt_platform') != 'xcb'):
                raise RuntimeError('Invalid independent Qt native capture metadata')
            png = screenshot.read_bytes()
            width, height = png_dimensions(png)
            if (native.get('pixel_width') != width or native.get('pixel_height') != height
                    or abs(width - selected['width'] * ratio) > 1
                    or abs(height - selected['height'] * ratio) > 1):
                raise RuntimeError('Independent Qt screenshot dimensions do not match the owned client')
            record.update(native, sha256=hashlib.sha256(png).hexdigest(), status='PASS',
                          active_window_verified_before_and_after=True)
        except Exception as error:
            record['error'] = str(error)
            stderr = getattr(error, 'stderr', None)
            if stderr:
                record['capture_stderr'] = (stderr.decode(errors='replace')
                    if isinstance(stderr, bytes) else str(stderr))[-4096:]
            # Never retain a capture whose attribution could not be established.
            screenshot.unlink(missing_ok=True)
        finally:
            record['duration_s'] = time.monotonic() - started
            json.dump(record, stream, indent=2)
            stream.write('\n')
            stream.flush()
        if ownership_error is not None:
            raise ownership_error
    return record


def capture_owned_window(child, filename, env, *, activate_owned=False, independent_qt=False, qt_only=False):
    from wc_runtime.component import process, descendants, belongs_to
    def read(command):
        return subprocess.check_output(command, env=env, text=True, timeout=5)
    def inspect(window_id):
        properties = read(['xprop', '-id', window_id, '_NET_WM_PID', '_NET_WM_WINDOW_TYPE',
                           '_NET_WM_STATE', 'WM_STATE', 'WM_TRANSIENT_FOR'])
        geometry = read(['xwininfo', '-id', window_id, '-stats'])
        return window_metadata(window_id, properties, geometry)
    owner = process(child.pid)
    if owner is None or child.poll() is not None:
        raise RuntimeError('Owned RViz wrapper is not alive; screenshot refused')
    owned = {item.pid: item for item in descendants(owner)}
    clients = read(['xprop', '-root', '_NET_CLIENT_LIST'])
    root_info = read(['xwininfo', '-root', '-stats'])
    root = window_metadata('0x0', '', root_info)
    if root['width'] is None or root['height'] is None:
        raise RuntimeError('Cannot verify display geometry; screenshot refused')
    candidates = []
    for window_id in re.findall(r'0x[0-9a-fA-F]+', clients):
        # Inspect further only after the client PID is attributed to our wrapper.
        properties = read(['xprop', '-id', window_id, '_NET_WM_PID'])
        pid = window_metadata(window_id, properties, '')['pid']
        if pid in owned and belongs_to(owned[pid], owner):
            candidates.append(inspect(window_id))
    selected = select_main_window(candidates, owned, (root['width'], root['height']))
    window_id, pid = selected['window_id'], selected['pid']
    if not belongs_to(owned[pid], owner):
        raise RuntimeError('RViz ownership changed before screenshot')
    if filename.exists():
        raise RuntimeError('Screenshot output already exists')
    if activate_owned:
        request_window_activation(window_id,env)
        deadline=time.monotonic()+2
        while True:
            try:require_active_main_window(read(['xprop','-root','_NET_ACTIVE_WINDOW']),window_id);break
            except RuntimeError:
                if time.monotonic()>=deadline:raise
                time.sleep(.05)
        selected=select_main_window([inspect(window_id)],owned,(root['width'],root['height']))
        if not belongs_to(owned[pid],owner) or child.poll() is not None:
            raise RuntimeError('Ownership changed after activation')
        time.sleep(.2)  # Let the WM expose the client's native rendering child.
    # The target xwd multi-visual path rewrites pixel layout; never guess its
    # contradictory header. GNOME captures its active client, so require that
    # exact client to be our independently selected NORMAL main window.
    require_active_main_window(read(['xprop', '-root', '_NET_ACTIVE_WINDOW']), window_id)
    def verify_selected_window():
        require_active_main_window(read(['xprop', '-root', '_NET_ACTIVE_WINDOW']), window_id)
        current = select_main_window([inspect(window_id)], owned, (root['width'], root['height']))
        if current != selected or not belongs_to(owned[pid], owner) or child.poll() is not None:
            raise RuntimeError('Owned main window changed while captured; screenshot refused')
    qt_capture = None
    if independent_qt or qt_only:
        qt_capture = capture_independent_qt(filename, window_id, env, selected,
            verify_selected_window, dict(rviz_pid=pid,
                rviz_start_ticks=owned[pid].start_ticks, owner_pid=owner.pid,
                owner_start_ticks=owner.start_ticks))
        # Qt capture is evidence for its own time interval. GNOME must start
        # from a still-owned active client and retain its original failure path.
        verify_selected_window()
        if qt_only:
            if qt_capture.get('status') != 'PASS':
                raise RuntimeError('Qt-only native screenshot failed: ' + str(qt_capture.get('error')))
            return dict(qt_capture, capture_mode='qt_only', activation_requested=activate_owned,
                        gnome_capture_requested=False)
    try:
        subprocess.run(['gnome-screenshot', '--window', '--file', str(filename)],
                       env=env, timeout=15, check=True)
        require_active_main_window(read(['xprop', '-root', '_NET_ACTIVE_WINDOW']), window_id)
        after = select_main_window([inspect(window_id)], owned, (root['width'], root['height']))
        if after != selected or not belongs_to(owned[pid], owner) or child.poll() is not None:
            raise RuntimeError('Owned main window changed while captured; screenshot refused')
        png = filename.read_bytes()
        width, height = png_dimensions(png)
        # GNOME may include window-manager decorations, but a dialog or a
        # full-desktop fallback must not pass as the selected client window.
        if (len(png) < 1000 or not selected['width'] <= width <= selected['width'] + 128
                or not selected['height'] <= height <= selected['height'] + 128):
            raise RuntimeError('Screenshot dimensions do not match the owned main window')
    except Exception:
        # This invocation owns the previously absent output; discard a capture
        # whose ownership/dimensions could not be established, while JSON fails.
        filename.unlink(missing_ok=True)
        raise
    result = dict(window_id=window_id, rviz_pid=pid, screenshot=str(filename),
                sha256=hashlib.sha256(png).hexdigest(), capture_method='gnome_verified_active_owned_normal_client',
                active_window_verified_before_and_after=True,
                activation_requested=activate_owned,
                window=selected, pixel_width=width, pixel_height=height)
    if independent_qt:
        result['independent_qt'] = qt_capture
    return result



def main():
    import fcntl
    from wc_runtime.cli import ROOT, target, name
    from wc_runtime.component import process, descendants
    from wc_maps.ros_view import create_viewer_node
    from wc_maps.store import MapStore
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-id', required=True, type=name)
    parser.add_argument('--version', default='v1', type=name)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--qt-only-capture', action='store_true',
                        help='Explicitly use verified native QScreen capture; never invoke GNOME or fall back')
    args = parser.parse_args()
    target()
    output = args.output_dir.absolute()
    if not output.resolve().is_relative_to(ROOT/'reports') or any(p.is_symlink() for p in (output, *output.parents)):
        raise ValueError('Test evidence must be a new directory under reports')
    output.mkdir(parents=True, exist_ok=False)
    if os.environ.get('ROS_DOMAIN_ID') != '86' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('This graphical test requires independent localhost ROS domain 86')
    lock = (ROOT/'.phase1_runtime/locks/domain-86-display.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    store = MapStore(ROOT/'maps')
    manifest = store.verify(args.map_id, args.version)
    if manifest['snapshot']['source_mode'] != 'synthetic':
        raise RuntimeError('This fixture requires an explicitly synthetic saved map')
    env = desktop_environment()
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    probe = Node('wc_rviz_test_preflight')
    deadline = time.monotonic()+2
    while time.monotonic() < deadline: rclpy.spin_once(probe, timeout_sec=.1)
    foreign = [n for n, ns in probe.get_node_names_and_namespaces() if n != probe.get_name()]
    probe.destroy_node()
    if foreign: raise RuntimeError('Independent display domain is occupied: '+repr(foreign))
    viewer = create_viewer_node(store, args.map_id, args.version)
    evidence = dict(test='actual_orin_rviz_saved_synthetic_map', status='FAIL', source_mode='synthetic',
        map_id=args.map_id, version=args.version, graph_revision=manifest['graph_revision'],
        ros_domain_id=86, display=env['DISPLAY'], views={}, navigation_validated=False,
        visual_review='PENDING_INDEPENDENT_SCREENSHOT_INSPECTION')
    try:
        for view in ('3d', '2d'):
            node_name = 'wc_rviz_test_'+view
            required_topics = ('map/cloud', 'map/grid') if view == '3d' else ('map/grid',)
            command = [sys.executable, '-m', 'wc_runtime.component', '--parent', str(os.getpid()), '--',
                'rviz2', '-d', str(ROOT/'config/rviz'/('mapping_'+view+'.rviz')),
                '--ros-args', '-r', '__node:='+node_name]
            with (output/(view+'.log')).open('x') as log:
                child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    env=env, start_new_session=True)
                fd = os.pidfd_open(child.pid)
                try:
                    deadline = time.monotonic()+30
                    observed = {}
                    ready_since = None
                    while time.monotonic() < deadline:
                        if child.poll() is not None: raise RuntimeError('RViz exited before display checks')
                        rclpy.spin_once(viewer, timeout_sec=.1)
                        for topic in required_topics:
                            infos = viewer.get_subscriptions_info_by_topic('/wc_mapping/'+topic)
                            observed[topic] = [i.node_name for i in infos if i.node_name == node_name]
                        if all(observed.values()):
                            ready_since = ready_since or time.monotonic()
                            if time.monotonic()-ready_since > 5: break
                    if not all(observed.values()): raise RuntimeError('RViz did not subscribe to the map topics enabled in this view')
                    capture = capture_owned_window(child, output/(view+'.png'), env, qt_only=args.qt_only_capture)
                    evidence['views'][view] = dict(status='DDS_AND_WINDOW_CAPTURE_PASS', subscriptions=observed, **capture)
                finally:
                    if child.poll() is None: signal.pidfd_send_signal(fd, signal.SIGINT)
                    child.wait(timeout=18)
                    os.close(fd)
            if child.returncode != 0: raise RuntimeError('Owned RViz wrapper cleanup failed')
        evidence['status'] = 'PASS'
    except Exception as error:
        evidence['error'] = str(error)
    finally:
        viewer.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        with (output/'result.json').open('x') as stream: json.dump(evidence, stream, indent=2)
        lock.close()
    print(json.dumps(evidence, indent=2))
    return 0 if evidence['status'] == 'PASS' else 1


if __name__ == '__main__': raise SystemExit(main())
