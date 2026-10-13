"""Small ROS adapter; all point iteration runs in the required native extension."""
from lio_web import _native


def encode_cloud(msg, max_points, voxel_size, magic=b'LIO3'):
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ('x', 'y', 'z')) or msg.point_step <= 0:
        return b'', 0
    selected = [fields['x'], fields['y'], fields['z'],
                fields.get('intensity') or fields.get('reflectivity'),
                fields.get('rgb') or fields.get('rgba')]
    descriptors = [(field.offset, field.datatype) if field else (-1, 0)
                   for field in selected]
    return _native.encode_cloud(
        msg.data, msg.width, msg.height, msg.point_step, msg.row_step,
        bool(msg.is_bigendian), descriptors, max_points, voxel_size,
        magic.decode('ascii'))
