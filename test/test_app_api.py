"""The preview server forwards read-only API data, never fabricates task state."""
import asyncio
import json
from types import SimpleNamespace

from aiohttp import web
from lio_web.server import WebServer


def test_app_api_forwarding():
    async def run():
        async def device(request):
            if request.headers.get('Authorization') != 'Bearer test-token':
                return web.json_response({'code': 'UNAUTHORIZED'}, status=401)
            return web.json_response({'name': 'Gateway-owned scanner', 'battery_percent': None})
        app = web.Application()
        app.router.add_get('/api/device', device)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '127.0.0.1', 0)
        await site.start()
        upstream = site._server.sockets[0].getsockname()[1]
        node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=f'http://127.0.0.1:{upstream}'))
        server = WebServer(node, '127.0.0.1', 0)
        await server.start()
        port = server.server.sockets[0].getsockname()[1]

        async def request(method, path, auth=''):
            reader, writer = await asyncio.open_connection('127.0.0.1', port)
            writer.write(f'{method} {path} HTTP/1.1\r\nHost: localhost\r\nAuthorization: {auth}\r\n\r\n'.encode())
            await writer.drain()
            reply = await reader.read()
            writer.close()
            await writer.wait_closed()
            headers, body = reply.split(b'\r\n\r\n', 1)
            assert b'Content-Type: application/json' in headers
            return int(headers.split()[1]), json.loads(body)
        try:
            code, result = await request('GET', '/api/device')
            assert code == 401
            code, result = await request('GET', '/api/device', 'Bearer test-token')
            assert code == 200
            assert result['name'] == 'Gateway-owned scanner'
            code, result = await request('POST', '/api/scan/start')
            assert code == 410
            await runner.cleanup()
            code, result = await request('GET', '/api/device')
            assert code == 503
            assert result['code'] == 'API_UNAVAILABLE'
        finally:
            server.server.close()
            await server.server.wait_closed()
            await runner.cleanup()
    asyncio.run(run())
