"""生成可打印的 A4 棋盘格 PDF（纯矢量，不依赖第三方库）。

默认 10×7 个方格（OpenCV 内角点 9×6），方格边长 25 mm，A4 横放居中。
页面带 100 mm 比例尺：打印后用直尺量比例尺和任意 4 个方格（应为 100 mm），确认打印机没有缩放。
"""
import argparse
from pathlib import Path

MM = 72 / 25.4  # PDF 点/毫米
A4_LANDSCAPE_MM = (297.0, 210.0)


def board_spec(inner_cols=9, inner_rows=6, square_mm=25.0):
    cols, rows = inner_cols + 1, inner_rows + 1
    width, height = cols * square_mm, rows * square_mm
    page_w, page_h = A4_LANDSCAPE_MM
    if width > page_w - 20 or height > page_h - 30:
        raise ValueError('棋盘格放不进 A4（需至少留 10 mm 边距和 20 mm 文字区）')
    return dict(inner_cols=inner_cols, inner_rows=inner_rows, square_mm=square_mm,
                cols=cols, rows=rows, width_mm=width, height_mm=height,
                left_mm=(page_w - width) / 2, bottom_mm=(page_h - height) / 2 + 5)


def page_content(spec):
    ops = ['0 g']
    s = spec['square_mm'] * MM
    x0, y0 = spec['left_mm'] * MM, spec['bottom_mm'] * MM
    for r in range(spec['rows']):
        for c in range(spec['cols']):
            if (r + c) % 2 == 0:  # 左下角为黑格
                ops.append('%.3f %.3f %.3f %.3f re f' % (x0 + c * s, y0 + r * s, s, s))
    # 100 mm 比例尺（棋盘格下方）
    bar_y = (spec['bottom_mm'] - 9) * MM
    ops.append('%.3f %.3f %.3f %.3f re f' % (x0, bar_y, 100 * MM, 1.2 * MM))
    for k in range(0, 101, 10):
        ops.append('%.3f %.3f %.3f %.3f re f' % (x0 + k * MM - 0.2, bar_y, 0.4, (3 if k % 50 == 0 else 2) * MM))
    text = ('Checkerboard %dx%d inner corners, square %.1f mm. Print at 100%% (actual size, no fit-to-page). '
            'Scale bar = 100 mm.' % (spec['inner_cols'], spec['inner_rows'], spec['square_mm']))
    ty = (spec['bottom_mm'] + spec['height_mm'] + 5) * MM
    ops.append('BT /F1 9 Tf %.3f %.3f Td (%s) Tj ET' % (x0, ty, text.replace('(', r'\(').replace(')', r'\)')))
    return '\n'.join(ops).encode('ascii')


def write_pdf(path, spec):
    w, h = (v * MM for v in A4_LANDSCAPE_MM)
    content = page_content(spec)
    objects = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        ('<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.3f %.3f] /Contents 4 0 R '
         '/Resources << /Font << /F1 5 0 R >> >> >>' % (w, h)).encode('ascii'),
        b'<< /Length %d >>\nstream\n' % len(content) + content + b'\nendstream',
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>',
    ]
    out, offsets = bytearray(b'%PDF-1.4\n'), []
    for i, body in enumerate(objects, 1):
        offsets.append(len(out))
        out += b'%d 0 obj\n' % i + body + b'\nendobj\n'
    xref = len(out)
    out += b'xref\n0 %d\n0000000000 65535 f \n' % (len(objects) + 1)
    out += b''.join(b'%010d 00000 n \n' % o for o in offsets)
    out += b'trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n' % (len(objects) + 1, xref)
    Path(path).write_bytes(bytes(out))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--inner-cols', type=int, default=9)
    parser.add_argument('--inner-rows', type=int, default=6)
    parser.add_argument('--square-mm', type=float, default=25.0)
    args = parser.parse_args(argv)
    spec = board_spec(args.inner_cols, args.inner_rows, args.square_mm)
    write_pdf(args.output, spec)
    print(args.output, spec)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
