#!/usr/bin/env python3
"""Read FCStd ZIP/XML and saved BRep geometry without opening a FreeCAD document.

No archive extraction, document recompute, Python proxy, macro or source write.
Requires the already installed OCP bindings only for BRep decoding. Geometry is
reported in the saved shape's frame, including its root Location exactly once.
"""
import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import platform
import sys
import xml.etree.ElementTree as ET
import zipfile


MAX_MEMBER_BYTES = 8 * 1024 * 1024
MAX_TOTAL_BYTES = 64 * 1024 * 1024
KEYWORDS = ('imu', 'h30', 'xt-m60', 'xt_m60', 'optical', 'datum', 'origin', 'coordinate',
            '惯导', '惯性', '光心', '光学原点', '坐标系')


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def read_member(archive, name):
    info = archive.getinfo(name)
    if info.file_size > MAX_MEMBER_BYTES:
        raise ValueError('archive member exceeds bounded read size: '+name)
    data = archive.read(info)
    if len(data) != info.file_size:
        raise ValueError('archive member size differs from directory')
    return data


def parse_xml(data):
    # Metadata is data, never an instruction or a document proxy to instantiate.
    if b'<!DOCTYPE' in data.upper() or b'<!ENTITY' in data.upper():
        raise ValueError('DTD/entity declarations are not accepted')
    return ET.fromstring(data)


def property_value(prop):
    return {'type': prop.get('type'),
            'children': [{'tag': node.tag, 'attributes': dict(node.attrib),
                          'text': (node.text or '').strip()}
                         for node in prop.iter() if node is not prop]}


def finite_numbers(values):
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError('nonfinite geometric metadata')
    return result


def brep_metadata(data):
    from OCP.BRep import BRep_Builder
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepTools import BRepTools
    from OCP.Bnd import Bnd_Box
    from OCP.TopoDS import TopoDS_Shape
    shape = TopoDS_Shape()
    BRepTools.Read_s(shape, io.BytesIO(data), BRep_Builder())
    if shape.IsNull():
        raise ValueError('saved BRep decoded to a null shape')
    bounds = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, bounds, False, False)
    limits = finite_numbers(bounds.Get())
    transform = shape.Location().Transformation()
    return {'saved_shape_bbox': limits,
            'saved_shape_bbox_size': [limits[i+3]-limits[i] for i in range(3)],
            'saved_shape_root_location_3x4': [[transform.Value(i, j) for j in range(1, 5)] for i in range(1, 4)],
            'shape_type': str(shape.ShapeType()),
            'bbox_method': 'OCP.BRepBndLib.AddOptimal_s(shape, box, False, False)',
            'extra_xml_placement_applied': False}


def inspect_file(path):
    source = Path(path).resolve(strict=True)
    if source.suffix.lower() != '.fcstd':
        raise ValueError('only explicitly supplied FCStd files are accepted')
    data = source.read_bytes()
    if len(data) > MAX_TOTAL_BYTES:
        raise ValueError('FCStd exceeds bounded source size')
    before_hash = sha256(data)
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        members = archive.infolist()
        if len(members) > 1000 or len({i.filename for i in members}) != len(members):
            raise ValueError('duplicate member names or too many archive members')
        if sum(i.file_size for i in members) > MAX_TOTAL_BYTES:
            raise ValueError('archive exceeds bounded uncompressed size')
        document_bytes = read_member(archive, 'Document.xml')
        document = parse_xml(document_bytes)
        types = {o.get('name'): o.get('type') for o in document.findall('./Objects/Object')}
        gui_bytes = read_member(archive, 'GuiDocument.xml')
        gui = parse_xml(gui_bytes)
        visible = {}
        for provider in gui.findall('.//ViewProvider'):
            value = provider.find("./Properties/Property[@name='Visibility']/Bool")
            if value is not None:
                visible[provider.get('name')] = value.get('value')
        objects = []
        for obj in document.findall('./ObjectData/Object'):
            props = {p.get('name'): p for p in obj.findall('./Properties/Property')}
            label = props.get('Label')
            label = label.find('String').get('value') if label is not None else None
            placement = props.get('Placement')
            placement = placement.find('PropertyPlacement') if placement is not None else None
            selected = {name: property_value(prop) for name, prop in props.items()
                        if name not in ('Label', 'Placement', 'Shape') and
                        (name in ('Length', 'Width', 'Height', 'Radius', 'Group', 'ExpressionEngine', 'Description')
                         or any(key in name.lower() for key in KEYWORDS))}
            row = {'name': obj.get('name'), 'type': types.get(obj.get('name')), 'label': label,
                   'visible_in_saved_gui': visible.get(obj.get('name'), 'UNKNOWN'),
                   'xml_placement': dict(placement.attrib) if placement is not None else None,
                   'properties': selected}
            shape = props.get('Shape')
            part = shape.find('Part') if shape is not None else None
            if part is not None:
                member = part.get('file')
                shape_bytes = read_member(archive, member)
                row.update({'shape_member': member, 'shape_sha256': sha256(shape_bytes),
                            'shape_bytes': len(shape_bytes), **brep_metadata(shape_bytes)})
            objects.append(row)
        raw_text = document_bytes.decode('utf-8').casefold()
        result = {'source_path': str(source), 'source_bytes': len(data), 'source_sha256': before_hash,
                  'document_xml_sha256': sha256(document_bytes), 'gui_xml_sha256': sha256(gui_bytes),
                  'document_attributes': dict(document.attrib),
                  'document_properties': {p.get('name'): property_value(p) for p in document.findall('./Properties/Property')},
                  'archive_member_count': len(members), 'object_count': len(objects),
                  'object_types': sorted(set(types.values())),
                  'keyword_counts_in_document_xml': {key: raw_text.count(key.casefold()) for key in KEYWORDS},
                  'objects': objects}
    result['source_unchanged_after_read'] = sha256(source.read_bytes()) == before_hash
    if not result['source_unchanged_after_read']:
        raise ValueError('source changed during read-only inspection')
    return result


def compare(right, left):
    right_by_name = {row['name']: row for row in right['objects']}
    left_by_name = {row['name']: row for row in left['objects']}
    same_names = sorted(set(right_by_name) & set(left_by_name))
    rows = []
    for name in same_names:
        r, l = right_by_name[name], left_by_name[name]
        row = {'name': name, 'same_label': r['label'] == l['label'],
               'same_object_type': r['type'] == l['type']}
        if 'saved_shape_bbox' in r and 'saved_shape_bbox' in l:
            rb, lb = r['saved_shape_bbox'], l['saved_shape_bbox']
            reflected = [100-rb[3], rb[1], rb[2], 100-rb[0], rb[4], rb[5]]
            row.update({'same_saved_brep_bytes': r['shape_sha256'] == l['shape_sha256'],
                        'bbox_direct_max_abs_difference': max(abs(a-b) for a, b in zip(rb, lb)),
                        'bbox_x_reflection_about_50_max_abs_difference': max(abs(a-b) for a, b in zip(reflected, lb))})
        rows.append(row)
    return {'same_named_objects': rows,
            'right_only_names': sorted(set(right_by_name)-set(left_by_name)),
            'left_only_names': sorted(set(left_by_name)-set(right_by_name)),
            'reflection_comparison_is_bbox_only_not_a_proved_shape_or_rigid_transform': True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--right', type=Path, required=True)
    parser.add_argument('--left', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='new JSON report; never overwrite')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    sources = [args.right.resolve(strict=True), args.left.resolve(strict=True)]
    if output in sources or output.exists() or output.suffix.lower() != '.json':
        raise ValueError('report output must be a new separate JSON file')
    import OCP
    right, left = inspect_file(sources[0]), inspect_file(sources[1])
    report = {'status': 'READ_ONLY_CAD_INSPECTION_COMPLETED', 'verification_level': 'STATIC_REVIEW',
              'physical_mount_verified': False, 'extrinsics_generated': False,
              'document_opened_or_recomputed': False, 'source_files_modified': False,
              'runtime': {'python': platform.python_version(), 'python_executable': sys.executable,
                          'OCP_version': OCP.__version__, 'OCP_module': OCP.__file__},
              'units': 'Document UnitSystem index 0 = standard mm, kg, s; geometric numbers are CAD mm, not measurement accuracy.',
              'right': right, 'left': left, 'comparison': compare(right, left)}
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'status': report['status'], 'report': str(output),
                      'right_objects': right['object_count'], 'left_objects': left['object_count'],
                      'right_sha256': right['source_sha256'], 'left_sha256': left['source_sha256']}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
