"""Frozen pre-C++ cloud encoder for protocol regression and benchmarks only."""
import math
import struct
HEADER = struct.Struct('<4sIf3f')
POINT = struct.Struct('<hhhBBBB')

def _field_map(msg):
    return {field.name: field for field in msg.fields}


def _read_number(data, offset, datatype):
    formats = {1: '<b', 2: '<B', 3: '<h', 4: '<H', 5: '<i', 6: '<I',
               7: '<f', 8: '<d'}
    fmt = formats.get(datatype)
    if not fmt or offset + struct.calcsize(fmt) > len(data):
        return 0.0
    return struct.unpack_from(fmt, data, offset)[0]


def _read_rgb(data, offset, field):
    """Decode the packed PCL/ROS rgb field into three 8-bit channels."""
    if field is None:
        return None
    packed = _read_number(data, offset, field.datatype)
    if field.datatype == 7:  # PointCloud2 convention: float32 bit-packed RGB.
        packed = struct.unpack('<I', struct.pack('<f', float(packed)))[0]
    elif field.datatype != 6:
        return None
    packed = int(packed) & 0x00FFFFFF
    return (packed >> 16, (packed >> 8) & 0xFF, packed & 0xFF)


def _intensity_rgb(value):
    value = float(value) if math.isfinite(float(value)) else 0.0
    if 0.0 <= value <= 1.0:
        value *= 255.0
    value = max(0, min(255, int(round(value))))
    return value, value, value


def encode_cloud(msg, max_points, voxel_size, magic=b'LIO3'):
    fields = _field_map(msg)
    if not all(name in fields for name in ('x', 'y', 'z')) or msg.point_step <= 0:
        return b'', 0
    intensity = fields.get('intensity') or fields.get('reflectivity')
    rgb = fields.get('rgb') or fields.get('rgba')
    raw = msg.data
    total = min(msg.width * msg.height, len(raw) // msg.point_step)
    points = {}
    # Bound Python work even when the ROS preview contains millions of points.
    sample_step = max(1, math.ceil(total / max(1, max_points * 4)))
    for index in range(0, total, sample_step):
        row, column = divmod(index, msg.width or 1)
        base = row * msg.row_step + column * msg.point_step
        x = float(_read_number(raw, base + fields['x'].offset, fields['x'].datatype))
        y = float(_read_number(raw, base + fields['y'].offset, fields['y'].datatype))
        z = float(_read_number(raw, base + fields['z'].offset, fields['z'].datatype))
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        key = (math.floor(x / voxel_size), math.floor(y / voxel_size), math.floor(z / voxel_size))
        if key not in points:
            value = (_read_number(raw, base + intensity.offset, intensity.datatype)
                     if intensity else 0.0)
            colour = (_read_rgb(raw, base + rgb.offset, rgb)
                      if rgb else None) or _intensity_rgb(value)
            points[key] = (x, y, z, *colour, _intensity_rgb(value)[0])
    values = list(points.values())
    if len(values) > max_points:
        stride = len(values) / max_points
        values = [values[min(len(values) - 1, int(i * stride))] for i in range(max_points)]
    if not values:
        return b'', 0
    # Quantise relative to a moving origin so int16 covers up to ~1 km at 1 cm.
    origin = tuple(min(point[i] for point in values) for i in range(3))
    scale = max(voxel_size, 0.01)
    packed = bytearray(HEADER.pack(magic, len(values), scale, *origin))
    for x, y, z, red, green, blue, strength in values:
        q = [max(-32768, min(32767, int(round((value - origin[i]) / scale))))
             for i, value in enumerate((x, y, z))]
        packed.extend(POINT.pack(q[0], q[1], q[2], red, green, blue, strength))
    return bytes(packed), len(values)
