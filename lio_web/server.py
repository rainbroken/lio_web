import asyncio
import base64
import hashlib
import logging
import math
import os
import struct
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


LOG = logging.getLogger('lio_web')
HEADER = struct.Struct('<4sIf3f')  # magic, count, quantisation metres, origin xyz
POINT = struct.Struct('<hhhB')


def _field_map(msg):
    return {field.name: field for field in msg.fields}


def _read_number(data, offset, datatype):
    formats = {7: '<f', 8: '<d', 2: '<H', 4: '<I', 1: '<b', 3: '<h', 5: '<i'}
    fmt = formats.get(datatype)
    if not fmt or offset + struct.calcsize(fmt) > len(data):
        return 0.0
    return struct.unpack_from(fmt, data, offset)[0]


def encode_cloud(msg, max_points, voxel_size):
    fields = _field_map(msg)
    if not all(name in fields for name in ('x', 'y', 'z')) or msg.point_step <= 0:
        return b'', 0
    intensity = fields.get('intensity') or fields.get('reflectivity')
    raw = msg.data
    total = min(msg.width * msg.height, len(raw) // msg.point_step)
    points = {}
    for index in range(total):
        row, column = divmod(index, msg.width or 1)
        base = row * msg.row_step + column * msg.point_step
        x = float(_read_number(raw, base + fields['x'].offset, fields['x'].datatype))
        y = float(_read_number(raw, base + fields['y'].offset, fields['y'].datatype))
        z = float(_read_number(raw, base + fields['z'].offset, fields['z'].datatype))
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        key = (math.floor(x / voxel_size), math.floor(y / voxel_size), math.floor(z / voxel_size))
        if key not in points:
            value = _read_number(raw, base + intensity.offset, intensity.datatype) if intensity else 0.0
            points[key] = (x, y, z, max(0, min(255, int(value))))
    values = list(points.values())
    if len(values) > max_points:
        stride = len(values) / max_points
        values = [values[min(len(values) - 1, int(i * stride))] for i in range(max_points)]
    if not values:
        return b'', 0
    # Quantise relative to a moving origin so int16 covers up to ~1 km at 1 cm.
    origin = tuple(min(point[i] for point in values) for i in range(3))
    scale = max(voxel_size, 0.01)
    packed = bytearray(HEADER.pack(b'LIO1', len(values), scale, *origin))
    for x, y, z, intensity_value in values:
        q = [max(-32768, min(32767, int(round((value - origin[i]) / scale))))
             for i, value in enumerate((x, y, z))]
        packed.extend(POINT.pack(q[0], q[1], q[2], intensity_value))
    return bytes(packed), len(values)


class CloudNode(Node):
    def __init__(self):
        super().__init__('lio_web_server')
        self.declare_parameter('pointcloud_topic', '/lio/cloud_registered')
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('voxel_size', 0.15)
        self.topic = self.get_parameter('pointcloud_topic').value
        self.max_points = int(self.get_parameter('max_points').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self._lock = threading.Lock()
        self._latest = None
        self._last_publish = 0.0
        self._stats = {'frames': 0, 'points': 0, 'bytes': 0, 'stamp': 0.0}
        self.create_subscription(PointCloud2, self.topic, self._on_cloud, 1)
        self.get_logger().info(
            f'Web viewer subscribes to {self.topic} (max_points={self.max_points}, '
            f'rate={self.rate_hz:.2f} Hz, voxel={self.voxel_size:.3f} m)')

    def _on_cloud(self, msg):
        now = time.monotonic()
        if now - self._last_publish < 1.0 / max(self.rate_hz, 0.1):
            return
        payload, points = encode_cloud(msg, self.max_points, self.voxel_size)
        if not payload:
            return
        self._last_publish = now
        with self._lock:
            self._latest = payload
            self._stats = {'frames': self._stats['frames'] + 1, 'points': points,
                           'bytes': len(payload), 'stamp': now}

    def snapshot(self):
        with self._lock:
            return self._latest, dict(self._stats)


class WebServer:
    def __init__(self, node, host, port):
        self.node, self.host, self.port = node, host, port
        self.clients = set()

    async def start(self):
        self.server = await asyncio.start_server(self._handle, self.host, self.port)
        LOG.info('Open http://%s:%d/ from the Web subnet', self.host if self.host != '0.0.0.0' else '<robot-ip>', self.port)

    async def _handle(self, reader, writer):
        try:
            request = await asyncio.wait_for(reader.readuntil(b'\r\n\r\n'), 3)
            first = request.split(b'\r\n', 1)[0].decode('ascii', 'replace')
            headers = request.decode('ascii', 'replace').lower()
            if 'upgrade: websocket' in headers and first.startswith('GET /ws'):
                await self._websocket(reader, writer, request)
            else:
                await self._http(writer)
        except (asyncio.IncompleteReadError, asyncio.TimeoutError, ConnectionError):
            writer.close()

    async def _http(self, writer):
        try:
            root = Path(get_package_share_directory('lio_web'))
        except Exception:
            root = Path(__file__).parent.parent
        path = root / 'web' / 'index.html'
        body = path.read_bytes()
        writer.write(b'HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n'
                     + ('Content-Length: %d\r\nConnection: close\r\n\r\n' % len(body)).encode() + body)
        await writer.drain()
        writer.close()

    async def _websocket(self, reader, writer, request):
        key = next((line.split(':', 1)[1].strip() for line in request.decode().split('\r\n')
                    if line.lower().startswith('sec-websocket-key:')), '')
        accept = base64.b64encode(hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
        writer.write(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                      'Sec-WebSocket-Accept: %s\r\n\r\n' % accept).encode())
        await writer.drain()
        self.clients.add(writer)
        last_stamp = 0.0
        try:
            while not writer.is_closing():
                await asyncio.sleep(0.2)
                payload, metrics = self.node.snapshot()
                if payload and metrics['stamp'] != last_stamp:
                    writer.write(self._frame(payload))
                    await writer.drain()
                    last_stamp = metrics['stamp']
                # Drain client control frames without blocking the producer.
                if reader.at_eof():
                    break
        except (ConnectionError, asyncio.CancelledError):
            pass
        finally:
            self.clients.discard(writer)
            writer.close()

    @staticmethod
    def _frame(payload):
        size = len(payload)
        if size < 126:
            return bytes([0x82, size]) + payload
        if size < 65536:
            return bytes([0x82, 126]) + struct.pack('>H', size) + payload
        return bytes([0x82, 127]) + struct.pack('>Q', size) + payload


async def run(node):
    server = WebServer(node, str(node.get_parameter('bind_host').value), int(node.get_parameter('port').value))
    await server.start()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(0.005)


def main(args=None):
    logging.basicConfig(level=logging.INFO, format='[lio_web] %(message)s')
    rclpy.init(args=args)
    node = CloudNode()
    try:
        asyncio.run(run(node))
    finally:
        node.destroy_node()
        rclpy.shutdown()
