"""Check the real HTTP router without requiring LiDAR hardware."""
import asyncio
import json
from types import SimpleNamespace

from lio_web.server import WebServer


class DeviceStub:
    def get_parameter(self, name):
        return SimpleNamespace(value={
            'device_name': 'Test scanner', 'device_model': '',
            'device_serial': '',
        }[name])

    def telemetry(self):
        return 1, {'scan_duration': 12.5, 'colored_map_points': 1234}


def test_app_json_routes():
    async def run():
        server = WebServer(DeviceStub(), '127.0.0.1', 0)
        await server.start()
        port = server.server.sockets[0].getsockname()[1]

        async def request(method, path):
            reader, writer = await asyncio.open_connection('127.0.0.1', port)
            writer.write(f'{method} {path} HTTP/1.1\r\nHost: localhost\r\n\r\n'.encode())
            await writer.drain()
            reply = await reader.read()
            writer.close()
            await writer.wait_closed()
            headers, body = reply.split(b'\r\n\r\n', 1)
            assert b'Content-Type: application/json' in headers
            return headers, json.loads(body)

        try:
            headers, device = await request('GET', '/api/device')
            assert b'200' in headers
            assert device['name'] == 'Test scanner'
            assert device['battery_percent'] is None
            assert device['capabilities']['scan_control'] is False
            headers, state = await request('GET', '/api/status')
            assert b'200' in headers
            assert state['duration_seconds'] == 12.5
            assert state['point_count'] == 1234
            assert state['state'] == 'unknown'
            for method, path in [('POST', '/api/scan/start'),
                                 ('GET', '/api/tasks'), ('GET', '/api/missing')]:
                headers, result = await request(method, path)
                assert b'501' in headers
                assert result['success'] is False
        finally:
            server.server.close()
            await server.server.wait_closed()

    asyncio.run(run())
