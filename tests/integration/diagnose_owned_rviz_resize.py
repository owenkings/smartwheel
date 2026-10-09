"""Resize only a freshly captured owned RViz client by 16 px; no input events."""
import ctypes as C
import os
from pathlib import Path
import re
import subprocess
import time


def probe(child, captured, env):
    from wc_runtime.component import process, descendants, belongs_to
    owner = process(child.pid)
    if owner is None or child.poll() is not None:
        raise RuntimeError('App owner gone before resize diagnostic')
    owned = {p.pid: p for p in descendants(owner)}
    pid, wid = captured['rviz_pid'], captured['window_id']
    properties = subprocess.check_output(['xprop', '-id', wid, '_NET_WM_PID',
        '_NET_WM_WINDOW_TYPE', '_NET_WM_STATE', 'WM_TRANSIENT_FOR'], env=env, text=True, timeout=5)
    match = re.search(r'^_NET_WM_PID\(CARDINAL\) = (\d+)$', properties, re.MULTILINE)
    if (pid not in owned or not belongs_to(owned[pid], owner) or not match or int(match[1]) != pid
            or '_NET_WM_WINDOW_TYPE_NORMAL' not in properties or '_NET_WM_STATE_MODAL' in properties
            or re.search(r'WM_TRANSIENT_FOR.*0x[1-9a-fA-F]', properties)):
        raise RuntimeError('Resize diagnostic refused: owned normal client not verified')
    width, height = captured['window']['width'], captured['window']['height']
    if not (640 <= width <= 1550 and 360 <= height <= 950):
        raise RuntimeError('Captured dimensions outside bounded resize diagnostic')
    graphics_keys = {'QT_QPA_PLATFORM', 'QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_QPA_FONTDIR',
        'QT_PLUGIN_PATH', 'QT_X11_NO_MITSHM', 'LIBGL_ALWAYS_SOFTWARE', '__GLX_VENDOR_LIBRARY_NAME',
        'LD_LIBRARY_PATH', 'OGRE_RESOURCE_PATH', 'DISPLAY'}
    graphics = {}
    for entry in (Path('/proc')/str(pid)/'environ').read_bytes().split(b'\0'):
        key, _, value = entry.partition(b'=')
        if key.decode(errors='replace') in graphics_keys:
            graphics[key.decode()] = value.decode(errors='replace')
    lib = C.CDLL('libX11.so.6')
    lib.XOpenDisplay.argtypes = [C.c_char_p]; lib.XOpenDisplay.restype = C.c_void_p
    lib.XResizeWindow.argtypes = [C.c_void_p, C.c_ulong, C.c_uint, C.c_uint]
    lib.XSync.argtypes = [C.c_void_p, C.c_int]; lib.XCloseDisplay.argtypes = [C.c_void_p]
    os.environ.update({key: env[key] for key in ('DISPLAY', 'XAUTHORITY')})
    display = lib.XOpenDisplay(env['DISPLAY'].encode())
    if not display:
        raise RuntimeError('Cannot open current authorized desktop')
    started = time.monotonic()
    try:
        lib.XResizeWindow(display, int(wid, 16), width+16, height)
        lib.XSync(display, False)
    finally:
        lib.XCloseDisplay(display)
    time.sleep(1)
    return {'scope': 'Owned client resize only; no vehicle/control or keyboard events',
            'original_client': [width, height], 'requested_client': [width+16, height],
            'request_monotonic': started, 'graphics_environment_whitelist': graphics}
