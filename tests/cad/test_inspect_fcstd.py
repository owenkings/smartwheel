"""Local synthetic checks of saved-shape placement and inert metadata parsing."""
import io
import unittest

from inspect_fcstd import brep_metadata, compare, parse_xml


class CadInspectionTests(unittest.TestCase):
    def test_brep_root_placement_is_applied_exactly_once(self):
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
        from OCP.BRepTools import BRepTools
        from OCP.TopLoc import TopLoc_Location
        from OCP.gp import gp_Trsf, gp_Vec
        shape = BRepPrimAPI_MakeBox(38.5, 41, 65).Shape()
        transform = gp_Trsf()
        transform.SetTranslation(gp_Vec(61.5, 0, 47.3))
        shape.Location(TopLoc_Location(transform))
        stream = io.BytesIO()
        BRepTools.Write_s(shape, stream)
        result = brep_metadata(stream.getvalue())
        self.assertEqual(result['saved_shape_bbox'], [61.5, 0, 47.3, 100, 41, 112.3])
        self.assertFalse(result['extra_xml_placement_applied'])
        self.assertEqual(result['saved_shape_bbox_size'], [38.5, 41, 65])

    def test_mirror_comparison_is_only_a_bbox_relation(self):
        right = {'name': 'radar', 'label': 'envelope', 'type': 'Part::Box',
                 'saved_shape_bbox': [61.5, 0, 47.3, 100, 41, 112.3], 'shape_sha256': 'right'}
        left = {**right, 'type': 'Part::Feature', 'shape_sha256': 'different',
                'saved_shape_bbox': [0, 0, 47.3, 38.5, 41, 112.3]}
        result = compare({'objects': [right]}, {'objects': [left]})
        row = result['same_named_objects'][0]
        self.assertEqual(row['bbox_x_reflection_about_50_max_abs_difference'], 0)
        self.assertGreater(row['bbox_direct_max_abs_difference'], 0)
        self.assertFalse(row['same_saved_brep_bytes'])
        self.assertTrue(result['reflection_comparison_is_bbox_only_not_a_proved_shape_or_rigid_transform'])

    def test_xml_entities_are_not_accepted(self):
        with self.assertRaises(ValueError):
            parse_xml(b'<!DOCTYPE doc [<!ENTITY outside SYSTEM "file:///must-not-read">]><doc>&outside;</doc>')

    def test_document_code_text_is_inert(self):
        root = parse_xml(b'<Document><Property name="Macro"><String value="raise RuntimeError()"/></Property></Document>')
        self.assertEqual(root.find('./Property/String').get('value'), 'raise RuntimeError()')


if __name__ == '__main__':
    unittest.main()
