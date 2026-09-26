import asyncio
import base64
import hashlib
import json
import logging
import math
import signal
import struct
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger

from lio_web.trajectory import TrajectoryHistory


LOG = logging.getLogger('lio_web')
HEADER = struct.Struct('<4sIf3f')  # magic, count, quantisation metres, origin xyz
# Quantised xyz plus RGB.  The explicit little-endian format avoids native
# alignment padding so the browser can decode the stream deterministically.
POINT = struct.Struct('<hhhBBB')


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


def encode_cloud(msg, max_points, voxel_size, magic=b'LIO2'):
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
            points[key] = (x, y, z, *colour)
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
    for x, y, z, red, green, blue in values:
        q = [max(-32768, min(32767, int(round((value - origin[i]) / scale))))
             for i, value in enumerate((x, y, z))]
        packed.extend(POINT.pack(q[0], q[1], q[2], red, green, blue))
    return bytes(packed), len(values)


class CloudNode(Node):
    def __init__(self):
        super().__init__('lio_web_server')
        self.declare_parameter('pointcloud_topic', '/lio/global_map')
        self.declare_parameter('fallback_pointcloud_topic', '/lio/cloud_colored')
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('max_points', 200000)
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('tile_cache_max_points', 1500000)
        self.declare_parameter('tile_draw_max_points', 300000)
        self.declare_parameter('tile_request_max_points', 50000)
        self.declare_parameter('odometry_topic', '/lio/odom')
        self.declare_parameter('trajectory_max_points', 2000)
        self.topic = self.get_parameter('pointcloud_topic').value
        self.fallback_topic = self.get_parameter('fallback_pointcloud_topic').value
        self.max_points = int(self.get_parameter('max_points').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.tile_cache_max_points = int(self.get_parameter('tile_cache_max_points').value)
        self.tile_draw_max_points = int(self.get_parameter('tile_draw_max_points').value)
        self.tile_request_max_points = int(self.get_parameter('tile_request_max_points').value)
        self.trajectory_max_points = int(self.get_parameter('trajectory_max_points').value)
        if self.max_points <= 0 or not math.isfinite(self.voxel_size) or self.voxel_size <= 0:
            raise ValueError('max_points and voxel_size must be positive')
        if (self.tile_cache_max_points <= 0 or self.tile_draw_max_points <= 0 or
                not 1 <= self.tile_request_max_points <= 100000):
            raise ValueError('tile point budgets must be positive and tile_request_max_points <= 100000')
        self._lock = threading.Lock()
        self._latest = None
        self._last_publish = 0.0
        self._last_primary = 0.0
        self._latest_topic = self.topic
        self._encoder = ThreadPoolExecutor(max_workers=1)
        self._update_encoder = ThreadPoolExecutor(max_workers=1)
        self._encode_future = None
        self._delta_queue = deque(maxlen=64)
        self._delta_sequence = 0
        self._pending_delta_encodes = 0
        self._pending_tiles = {}
        self._next_tile_id = 0
        self._stats = {'frames': 0, 'points': 0, 'source_points': 0,
                       'bytes': 0, 'stamp': 0.0}
        self._colored_map_points = None
        self._pose = None
        self._distance = 0.0
        self._speed = None
        self._first_odom_stamp = None
        self._last_odom_stamp = None
        self._scan_duration = None
        self._trajectory = TrajectoryHistory(self.trajectory_max_points)
        self._pose_version = 0
        # Match the best-effort QoS used by LiDAR/PointCloud2 publishers.
        self.create_subscription(
            PointCloud2, self.topic,
            lambda msg: self._on_cloud(msg, self.topic), qos_profile_sensor_data)
        if self.fallback_topic and self.fallback_topic != self.topic:
            self.create_subscription(
                PointCloud2, self.fallback_topic,
                lambda msg: self._on_cloud(msg, self.fallback_topic),
                qos_profile_sensor_data)
        self.create_subscription(Odometry, self.get_parameter('odometry_topic').value,
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(UInt64, '/lio/colored_map_point_count',
                                 self._on_colored_count, 10)
        self.create_subscription(PointCloud2, '/lio/colored_map_updates',
                                 self._on_map_update, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lio/colored_map_tile',
                                 self._on_tile, qos_profile_sensor_data)
        self.tile_request_publisher = self.create_publisher(
            String, '/lio/colored_map_tile_request', 10)
        self.save_client = self.create_client(Trigger, '/lio/save_pcd')
        self.get_logger().info(
            f'Web viewer subscribes to {self.topic}, fallback={self.fallback_topic or "disabled"} '
            f'(max_points={self.max_points}, '
            f'rate={self.rate_hz:.2f} Hz, voxel={self.voxel_size:.3f} m)')

    def _on_colored_count(self, msg):
        with self._lock:
            self._colored_map_points = int(msg.data)
            self._pose_version += 1

    def _on_map_update(self, msg):
        with self._lock:
            if self._pending_delta_encodes >= 8:
                return  # A later tile fetch recovers the visible area.
            self._pending_delta_encodes += 1
        self._update_encoder.submit(self._encode_update, msg)

    def _encode_update(self, msg):
        try:
            payload, _ = encode_cloud(msg, 100000, 0.01, b'LIOD')
            if payload:
                with self._lock:
                    self._delta_sequence += 1
                    self._delta_queue.append((self._delta_sequence, payload))
        except Exception:
            LOG.exception('Map update encoding failed')
        finally:
            with self._lock:
                self._pending_delta_encodes -= 1

    def updates_since(self, sequence):
        with self._lock:
            return [item for item in self._delta_queue if item[0] > sequence]

    def _on_tile(self, msg):
        if not msg.header.frame_id.startswith('tile:'):
            return
        try:
            request_id = int(msg.header.frame_id[5:])
        except ValueError:
            return
        future = self._pending_tiles.pop(request_id, None)
        if future is not None and not future.done():
            future.set_result(msg)

    def request_tile(self, x, y, z, limit, loop):
        self._next_tile_id += 1
        request_id = self._next_tile_id
        future = loop.create_future()
        self._pending_tiles[request_id] = future
        request = String()
        request.data = f'{request_id} {x} {y} {z} {limit}'
        self.tile_request_publisher.publish(request)
        return request_id, future

    def _on_cloud(self, msg, topic):
        now = time.monotonic()
        primary = topic == self.topic
        if primary:
            self._last_primary = now
        elif self._last_primary and now - self._last_primary < 15.0:
            return
        if not primary and now - self._last_publish < 1.0 / max(self.rate_hz, 0.1):
            return
        if self._encode_future is not None and not self._encode_future.done():
            return
        if self._encode_future is not None:
            error = self._encode_future.exception()
            if error is not None:
                self.get_logger().error(f'Point cloud encoding failed: {error}')
        self._last_publish = now
        self._encode_future = self._encoder.submit(self._encode_cloud, msg, topic)

    def _encode_cloud(self, msg, topic):
        payload, points = encode_cloud(msg, self.max_points, self.voxel_size)
        if not payload:
            return
        source_points = min(msg.width * msg.height, len(msg.data) // msg.point_step)
        with self._lock:
            self._latest = payload
            self._latest_topic = topic
            self._pose_version += 1
            self._stats = {'frames': self._stats['frames'] + 1, 'points': points,
                           'source_points': source_points, 'bytes': len(payload),
                           'stamp': time.monotonic()}

    def destroy_node(self):
        self._encoder.shutdown(wait=True)
        self._update_encoder.shutdown(wait=True)
        return super().destroy_node()

    def snapshot(self):
        with self._lock:
            return self._latest, dict(self._stats)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        pose = [float(p.x), float(p.y), float(p.z)]
        if not all(math.isfinite(v) for v in pose):
            return
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        v = msg.twist.twist.linear
        velocity = [float(v.x), float(v.y), float(v.z)]
        speed = math.sqrt(sum(value * value for value in velocity)) if all(
            math.isfinite(value) for value in velocity) else None
        with self._lock:
            if math.isfinite(stamp):
                if self._last_odom_stamp is not None and stamp < self._last_odom_stamp:
                    self._trajectory.clear()
                    self._distance = 0.0
                    self._pose = None
                    self._first_odom_stamp = stamp
                if self._first_odom_stamp is None:
                    self._first_odom_stamp = stamp
                self._last_odom_stamp = stamp
                self._scan_duration = max(0.0, stamp - self._first_odom_stamp)
            if self._pose is not None:
                step = math.dist(pose, self._pose)
                if step >= 10.0:  # Start a new track on an odometry reset.
                    self._trajectory.clear()
                    self._distance = 0.0
                else:
                    self._distance += step
            self._pose = pose
            self._speed = speed
            self._trajectory.append(pose)
            self._pose_version += 1

    def telemetry(self):
        with self._lock:
            return self._pose_version, {'type': 'telemetry', 'pose': self._pose,
                                        'distance': self._distance,
                                        'speed': self._speed,
                                        'scan_duration': self._scan_duration,
                                        'source_points': self._stats['source_points'],
                                        'colored_map_points': self._colored_map_points,
                                        'tile_cache_max_points': self.tile_cache_max_points,
                                        'tile_draw_max_points': self.tile_draw_max_points,
                                        'tile_request_max_points': self.tile_request_max_points,
                                        'trajectory': self._trajectory.snapshot(),
                                        'topic': self._latest_topic,
                                        'odometry_topic': self.get_parameter('odometry_topic').value}


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
            if first == 'POST /save-pcd HTTP/1.1':
                await self._save_pcd(writer, request)
            elif first.startswith('GET /tile?'):
                await self._tile(writer, first)
            elif 'upgrade: websocket' in headers and first.startswith('GET /ws'):
                await self._websocket(reader, writer, request)
            else:
                await self._http(writer)
        except (asyncio.IncompleteReadError, asyncio.TimeoutError, ConnectionError):
            writer.close()

    async def _save_pcd(self, writer, request):
        headers = request.decode('ascii', 'replace').split('\r\n')
        values = {}
        for line in headers[1:]:
            if ':' in line:
                key, value = line.split(':', 1)
                values[key.lower()] = value.strip()
        host = values.get('host', '')
        origin = values.get('origin', '')
        valid_origin = origin in ('http://' + host, 'https://' + host)
        if not host or not valid_origin:
            status, result = 403, {'success': False, 'message': 'Invalid origin'}
        elif not self.node.save_client.service_is_ready():
            status, result = 503, {'success': False, 'message': 'LIO save service unavailable'}
        else:
            future = self.node.save_client.call_async(Trigger.Request())
            try:
                response = await asyncio.wait_for(self._await_ros_future(future), 300)
                status = 200 if response.success else 500
                result = {'success': response.success, 'message': response.message}
            except asyncio.TimeoutError:
                status, result = 504, {'success': False, 'message': 'Save timed out'}
            except Exception as exc:
                LOG.exception('PCD save failed')
                status, result = 500, {'success': False, 'message': str(exc)}
        body = json.dumps(result).encode('utf-8')
        writer.write(f'HTTP/1.1 {status} Save Result\r\nContent-Type: application/json\r\n'
                     f'Content-Length: {len(body)}\r\nConnection: close\r\n\r\n'.encode() + body)
        await writer.drain()
        writer.close()

    @staticmethod
    async def _await_ros_future(future):
        while not future.done():
            await asyncio.sleep(0.05)
        return future.result()

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

    async def _tile(self, writer, first):
        try:
            path = first.split(' ', 2)[1]
            args = parse_qs(urlsplit(path).query)
            x, y, z = (int(args[axis][0]) for axis in ('x', 'y', 'z'))
            limit = int(args.get('max', ['30000'])[0])
            if any(abs(value) > 100000 for value in (x, y, z)) or not 1 <= limit <= 100000:
                raise ValueError('tile coordinates or point limit out of range')
            if len(self.node._pending_tiles) >= 4:
                status, body = '429 Too Many Requests', b'tile requests busy'
                writer.write(f'HTTP/1.1 {status}\r\nContent-Length: {len(body)}\r\n'
                             'Connection: close\r\n\r\n'.encode() + body)
                await writer.drain()
                writer.close()
                return
            request_id, future = self.node.request_tile(
                x, y, z, limit, asyncio.get_running_loop())
            try:
                cloud = await asyncio.wait_for(future, 8)
            finally:
                self.node._pending_tiles.pop(request_id, None)
            body, _ = await asyncio.to_thread(
                encode_cloud, cloud, limit, 0.01, b'LIOT')
            status = '200 OK'
        except (KeyError, ValueError, IndexError):
            status, body = '400 Bad Request', b'invalid tile request'
        except asyncio.TimeoutError:
            status, body = '504 Gateway Timeout', b'tile request timed out'
        writer.write(f'HTTP/1.1 {status}\r\nContent-Type: application/octet-stream\r\n'
                     f'Content-Length: {len(body)}\r\nConnection: close\r\n\r\n'.encode() + body)
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
        last_snapshot_sent = 0.0
        last_delta_sequence = 0
        last_pose_version = -1
        try:
            while not writer.is_closing():
                await asyncio.sleep(0.2)
                payload, metrics = self.node.snapshot()
                if (payload and metrics['stamp'] != last_stamp and
                        (last_snapshot_sent == 0.0 or
                         time.monotonic() - last_snapshot_sent >= 10.0)):
                    writer.write(self._frame(payload))
                    await writer.drain()
                    last_stamp = metrics['stamp']
                    last_snapshot_sent = time.monotonic()
                for sequence, update in self.node.updates_since(last_delta_sequence):
                    writer.write(self._frame(update))
                    last_delta_sequence = sequence
                if last_delta_sequence:
                    await writer.drain()
                pose_version, telemetry = self.node.telemetry()
                if pose_version != last_pose_version:
                    writer.write(self._frame(json.dumps(telemetry).encode('utf-8'), opcode=1))
                    await writer.drain()
                    last_pose_version = pose_version
                # Drain client control frames without blocking the producer.
                if reader.at_eof():
                    break
        except (ConnectionError, asyncio.CancelledError):
            pass
        finally:
            self.clients.discard(writer)
            writer.close()

    @staticmethod
    def _frame(payload, opcode=2):
        size = len(payload)
        if size < 126:
            return bytes([0x80 | opcode, size]) + payload
        if size < 65536:
            return bytes([0x80 | opcode, 126]) + struct.pack('>H', size) + payload
        return bytes([0x80 | opcode, 127]) + struct.pack('>Q', size) + payload


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
    except KeyboardInterrupt:
        pass
    finally:
        # launch may send a second SIGINT while asyncio is unwinding.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
