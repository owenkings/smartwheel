"""Pure source/build-plan checks; no compiler, ROS, GUI, socket or hardware."""
import ast
import importlib.util
from pathlib import Path
import unittest


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('synthetic_manual_render_builder', HERE/'synthetic_manual_render.py')
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class SyntheticManualBuildTest(unittest.TestCase):
    def test_current_wrapper_uses_direct_synthetic_session_and_denies_confirmation(self):
        root = HERE.parents[1]
        source = (root/'src/wc_bringup/src/mapping_rviz.cpp').read_text(encoding='utf-8')
        reason = builder.disabled_reason((root/'src/wc_runtime/mapping_wheel.py').read_text(encoding='utf-8'))
        result = builder.diagnostic_cpp(source, HERE/'synthetic_manual_render.hpp', reason)
        self.assertNotIn('teleop_session_from_arguments(original_qt_arguments)', result)
        self.assertIn('synthetic_fixture.session(), frame, bottom, []() {return false; }', result)
        self.assertIn('synthetic_fixture.finish(application_code)', result)
        self.assertIn(reason, result)
        self.assertEqual(result.count('native_rviz.init(argc, argv)'), 1)

    def test_source_anchor_drift_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'anchor changed'):
            builder.diagnostic_cpp('int main() {return 0;}', HERE/'synthetic_manual_render.hpp', 'synthetic')

    def test_live_reason_is_literal_only_and_does_not_import(self):
        self.assertEqual(builder.disabled_reason("raise RuntimeError('must not execute')\nDISABLED_REASON='未验证'"), '未验证')
        with self.assertRaises(ValueError):
            builder.disabled_reason("DISABLED_REASON = load_from_device()")

    def test_link_replaces_both_objects_and_output_without_overwriting_original(self):
        flags = 'CXX_DEFINES = -DTEST=1\nCXX_INCLUDES = -I"/project with spaces/include"\nCXX_FLAGS = -std=c++14 -fPIC\n'
        link = '/usr/bin/c++ -O2 CMakeFiles/mapping_rviz.dir/src/mapping_rviz.cpp.o CMakeFiles/mapping_rviz.dir/src/mapping_teleop.cpp.o -o mapping_rviz /opt/lib/librviz_common.so'
        out = Path('/project/reports/synthetic build')
        commands = builder.compiler_commands(flags, link, out/'generated.cpp', Path('/project/teleop.cpp'), out)
        self.assertEqual(len(commands), 3)
        self.assertIn('-I/project with spaces/include', commands[0])
        self.assertEqual(commands[2][commands[2].index('-o')+1], str(out/'synthetic_manual_rviz'))
        self.assertNotIn('CMakeFiles/mapping_rviz.dir/src/mapping_teleop.cpp.o', commands[2])
        self.assertIn(str(out/'mapping_teleop.cpp.o'), commands[2])
        self.assertIn('/opt/lib/librviz_common.so', commands[2])
        with self.assertRaises(ValueError):
            builder.compiler_commands(flags, link.replace('mapping_teleop.cpp.o', 'unknown.cpp.o'), out/'g.cpp', out/'t.cpp', out)

    def test_current_harness_stays_offline_and_waits_for_native_evidence(self):
        source = (HERE/'check_dashboard_saved_render.py').read_text(encoding='utf-8')
        result = builder.diagnostic_harness(source, HERE)
        ast.parse(result)
        self.assertIn("offline_view = out/'offline_dashboard.rviz'", result)
        self.assertIn("'teleop_source_mode': 'synthetic'", result)
        self.assertIn("native_root/'phases_complete.json'", result)
        self.assertIn("native_root/'final.json'", result)
        self.assertIn("'.phase1_runtime/locks/domain-84-offline-review.lock'", result)
        self.assertNotIn("install/main/wc_bringup/lib/wc_bringup/mapping_rviz", result)
        self.assertLess(result.index("native_root/'phases_complete.json'"), result.index("capture_owned_window(gui, out/'saved_dashboard.png'"))

    def test_output_path_cannot_escape_project(self):
        root = HERE.parents[1]
        self.assertEqual(builder.contained(root, 'reports/new'), root/'reports/new')
        with self.assertRaises(ValueError):
            builder.contained(root, '../outside')


if __name__ == '__main__':
    unittest.main()
