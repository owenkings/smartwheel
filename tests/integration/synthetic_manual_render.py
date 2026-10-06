#!/usr/bin/env python3
"""Build/run an isolated synthetic DISARMED panel in the current native RViz wrapper.

Only new reports files are written. Production source, build objects, installed
binaries, real manual.sock and hardware/control launchers are never modified.
The run command reuses the reviewed saved-map/camera diagnostic through a private
generated copy; no production runtime identity is fabricated.
"""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SESSION = 'reports/maps/map_dashboard_left_20260914_04'


def fingerprint(path):
    data = path.read_bytes()
    return {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


def contained(root, supplied):
    """Lexically inspect every existing component before resolving symlinks."""
    root = root.resolve()
    path = Path(os.path.abspath(root / supplied))
    if not path.is_relative_to(root):
        raise ValueError('Diagnostic path must remain inside the selected project')
    for item in (path, *path.parents):
        if item.is_symlink():
            raise ValueError('Diagnostic paths cannot traverse symlinks')
        if item == root:
            break
    return path


def once(source, before, after):
    if source.count(before) != 1:
        raise ValueError('Reviewed diagnostic patch anchor changed: '+before[:100])
    return source.replace(before, after, 1)


def disabled_reason(source):
    """Read a literal through AST, without importing any wheel/control module."""
    values = [node.value.value for node in ast.parse(source).body
              if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'DISABLED_REASON'
                                                       for t in node.targets)
              and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str)]
    if len(values) != 1 or not 1 <= len(values[0].encode('utf-8')) <= 2048:
        raise ValueError('Expected one bounded literal mapping_wheel.DISABLED_REASON')
    return values[0]


def diagnostic_cpp(source, header, reason):
    source = once(source, '    auto * teleop_dock = new QDockWidget(',
        '    synthetic_manual_render::Fixture synthetic_fixture(*frame, application);\n'
        '    auto * teleop_dock = new QDockWidget(')
    source = once(source,
        'wc_bringup::teleop_session_from_arguments(original_qt_arguments), frame, bottom)',
        'synthetic_fixture.session(), frame, bottom, []() {return false; })')
    source = once(source, '    arrange_initial_docks(*frame, *teleop_dock);',
        '    arrange_initial_docks(*frame, *teleop_dock);\n    synthetic_fixture.start(*teleop_dock);')
    source = once(source, '    return application.exec();',
        '    const int application_code = application.exec();\n    return synthetic_fixture.finish(application_code);')
    # JSON string quoting is used only as a C++ string literal, never a shell command.
    return ('// GENERATED SYNTHETIC DIAGNOSTIC; NEVER INSTALL AS mapping_rviz.\n'
            '#define WC_SYNTHETIC_BLOCK_REASON '+json.dumps(reason, ensure_ascii=False)+'\n'
            '#include '+json.dumps(str(header).replace('\\', '/'))+'\n'+source)


def diagnostic_harness(source, integration_dir):
    source = once(source, 'import argparse',
        'import sys\nsys.path.insert(0, '+repr(str(integration_dir))+')\nimport argparse')
    source = once(source, "SYNTHETIC_TITLE = 'SYNTHETIC CAMERA DISPLAY DIAGNOSTIC'",
        "SYNTHETIC_TITLE = 'SYNTHETIC DISARMED MANUAL AND CAMERA DISPLAY DIAGNOSTIC'")
    source = once(source, "RVIZ_NODE = 'mapping_saved_dashboard_validation'",
        "RVIZ_NODE = 'mapping_synthetic_manual_validation'")
    source = once(source, "'teleop_socket_started': False,",
        "'teleop_socket_started': True, 'teleop_source_mode': 'synthetic', 'control_process_started': False,")
    source = once(source,
        "binary = contained(ROOT, 'install/main/wc_bringup/lib/wc_bringup/mapping_rviz')",
        "binary = contained(ROOT, os.environ['WC_SYNTHETIC_MANUAL_BINARY'])")
    source = once(source,
        "        result['window'] = capture_owned_window(gui, out/'saved_dashboard.png', env, activate_owned=True)",
        "        native_root = out/'manual_native'\n"
        "        native_deadline = time.monotonic()+25\n"
        "        while not (native_root/'phases_complete.json').exists():\n"
        "            if stopped[0] or gui.poll() is not None or publisher.poll() is not None or cameras.poll() is not None:\n"
        "                raise RuntimeError('Owned diagnostic stopped before synthetic phases completed')\n"
        "            if time.monotonic() >= native_deadline:\n"
        "                raise RuntimeError('Synthetic manual phases did not finish within the bounded wait')\n"
        "            executor.spin_once(timeout_sec=.1)\n"
        "        result['synthetic_manual_phases'] = read_json(native_root/'phases_complete.json')\n"
        "        if result['synthetic_manual_phases'].get('status') != 'PASS':\n"
        "            raise RuntimeError('Synthetic manual phase validation failed')\n"
        "        result['window'] = capture_owned_window(gui, out/'saved_dashboard.png', env, activate_owned=True)")
    source = once(source, "        result['status'] = 'PASS'",
        "        native_final = read_json(native_root/'final.json')\n"
        "        result['synthetic_manual_final'] = native_final\n"
        "        if (native_final.get('status') != 'PASS' or native_final.get('arm_allowed') is not False\n"
        "                or native_final.get('first_status_visible') is not True\n"
        "                or native_final.get('hello_count') != 1 or native_final.get('source_mode') != 'synthetic'\n"
        "                or any(row.get('type') != 'hello' for row in native_final.get('client_messages', []))):\n"
        "            raise RuntimeError('Synthetic DISARMED final evidence rejected')\n"
        "        result['status'] = 'PASS'")
    return source


def compiler_commands(flags_text, link_text, generated, teleop, output):
    values = {}
    for line in flags_text.splitlines():
        key, separator, value = line.partition(' = ')
        if separator and key in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
            values[key] = shlex.split(value)
    if set(values) != {'CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'}:
        raise ValueError('Unsupported CMake flags.make format')
    link = shlex.split(link_text)
    if not link or link.count('-o') != 1 or any(item.startswith('@') for item in link):
        raise ValueError('Unsupported CMake linker command')
    objects = {'src/'+name+'.o': output/(name+'.o')
               for name in ('mapping_rviz.cpp', 'mapping_teleop.cpp')}
    seen = set()
    for index, value in enumerate(link):
        for suffix, replacement in objects.items():
            if value.endswith('/'+suffix):
                if suffix in seen:
                    raise ValueError('Duplicate original wrapper object')
                seen.add(suffix); link[index] = str(replacement)
    if seen != set(objects):
        raise ValueError('Expected exactly the two reviewed wrapper/teleop objects')
    link[link.index('-o')+1] = str(output/'synthetic_manual_rviz')
    flags = values['CXX_DEFINES']+values['CXX_INCLUDES']+values['CXX_FLAGS']
    compile_commands = [[link[0], *flags, '-c', str(source), '-o', str(output/(name+'.o'))]
                        for source, name in ((generated, 'mapping_rviz.cpp'), (teleop, 'mapping_teleop.cpp'))]
    return compile_commands+[link]


def build(args):
    root = args.project_root.resolve()
    out = contained(root, args.output_root)
    if not out.is_relative_to(root/'reports') or out.exists():
        raise ValueError('Build requires a NEW directory under project/reports')
    integration = contained(root, 'tests/integration')
    source = contained(root, 'src/wc_bringup/src/mapping_rviz.cpp')
    teleop = contained(root, 'src/wc_bringup/src/mapping_teleop.cpp')
    wheel = contained(root, 'src/wc_runtime/mapping_wheel.py')
    header = integration/'synthetic_manual_render.hpp'
    harness = integration/'check_dashboard_saved_render.py'
    cmake_root = contained(root, 'build/main/wc_bringup')
    cmake = cmake_root/'CMakeFiles/mapping_rviz.dir'
    files = (source, teleop, wheel, header, harness, cmake/'flags.make', cmake/'link.txt')
    fingerprints = {str(path): fingerprint(path) for path in files}
    generated = diagnostic_cpp(source.read_text(encoding='utf-8'), header,
                               disabled_reason(wheel.read_text(encoding='utf-8')))
    runner = diagnostic_harness(harness.read_text(encoding='utf-8'), integration)
    commands = compiler_commands((cmake/'flags.make').read_text(), (cmake/'link.txt').read_text(),
                                 out/'synthetic_mapping_rviz.cpp', teleop, out)
    out.mkdir(parents=True, exist_ok=False)
    (out/'synthetic_mapping_rviz.cpp').write_text(generated, encoding='utf-8')
    (out/'saved_render_synthetic_manual.py').write_text(runner, encoding='utf-8')
    with (out/'build.log').open('xb') as log:
        for command in commands:
            # Argument vectors only. Never invoke a shell or overwrite installed targets.
            subprocess.run(command, cwd=cmake_root, stdout=log, stderr=subprocess.STDOUT, check=True)
    if fingerprints != {str(path): fingerprint(path) for path in files}:
        raise RuntimeError('Diagnostic source/build inputs changed while compiling')
    result = {'status': 'BUILT', 'source_mode': 'synthetic', 'hardware_started': False,
              'production_source_modified': False, 'installed_binary_modified': False,
              'inputs': fingerprints, 'commands': commands,
              'binary': {'path': str(out/'synthetic_manual_rviz'), **fingerprint(out/'synthetic_manual_rviz')},
              'harness': fingerprint(out/'saved_render_synthetic_manual.py')}
    (out/'build.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))


def run(args):
    root = args.project_root.resolve()
    built, out = contained(root, args.build_root), contained(root, args.output_root)
    if not out.is_relative_to(root/'reports') or out.exists():
        raise ValueError('Run requires a NEW report directory')
    manifest = json.loads((built/'build.json').read_text(encoding='utf-8'))
    binary, harness = built/'synthetic_manual_rviz', built/'saved_render_synthetic_manual.py'
    if (manifest.get('status') != 'BUILT' or manifest.get('source_mode') != 'synthetic'
            or fingerprint(binary) != {key: manifest['binary'][key] for key in ('bytes', 'sha256')}
            or fingerprint(harness) != manifest['harness']):
        raise ValueError('Synthetic build evidence or artifact hash mismatch')
    for path, expected in manifest['inputs'].items():
        if fingerprint(contained(root, path)) != expected:
            raise ValueError('Diagnostic input changed since build: '+path)
    session = contained(root, args.session_root)
    env = os.environ.copy()
    env.update(WC_SYNTHETIC_MANUAL_BINARY=str(binary), WC_SYNTHETIC_MANUAL_OUTPUT=str(out/'manual_native'),
               ROS_DOMAIN_ID='84', ROS_LOCALHOST_ONLY='1', PYTHONNOUSERSITE='1')
    # The existing harness owns domain locking, audited publishers, lifecycle,
    # screenshots and bounded child shutdown. Keep its PID/signal ownership intact.
    os.execve(sys.executable, [sys.executable, '-s', str(harness), '--session-root', str(session),
                              '--output-root', str(out), '--synthetic-camera-images'], env)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, default=ROOT)
    commands = parser.add_subparsers(dest='command', required=True)
    compile_parser = commands.add_parser('build', help='Compile only new private diagnostic artifacts')
    compile_parser.add_argument('--output-root', type=Path, required=True)
    run_parser = commands.add_parser('run', help='Audit/reload real saved maps plus synthetic cameras/manual status')
    run_parser.add_argument('--build-root', type=Path, required=True)
    run_parser.add_argument('--output-root', type=Path, required=True)
    run_parser.add_argument('--session-root', type=Path, default=Path(DEFAULT_SESSION))
    args = parser.parse_args(argv)
    if os.name != 'posix':
        parser.error('Build/run requires the sourced target Linux ROS environment; local pure tests remain available')
    return build(args) if args.command == 'build' else run(args)


if __name__ == '__main__':
    main()
