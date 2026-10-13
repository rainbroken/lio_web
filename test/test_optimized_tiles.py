import struct
import asyncio
import threading
import pytest
from types import SimpleNamespace
from concurrent.futures import ThreadPoolExecutor
from lio_web.server import CloudNode, WebServer
from lio_web.optimized_tiles import OptimizedTiles
from lio_web.optimized_blocks import OptimizedBlocks
import json
import numpy as np


def test_local_blocks_rotation_cross_tile_and_append_prefix(tmp_path):
    archive = tmp_path/'colored_points.bin'
    records = [(.5,0,0,42,0x123456), (1,0,0,42,0xabcdef)]
    archive.write_bytes(b''.join(struct.pack('<ffffI',*r) for r in records))
    manifest = tmp_path/'optimized_preview.json'
    manifest.write_text(json.dumps({'format':'scanforge-blocks-v1','archive':str(archive),
        'points':2,'blocks':[{'first':0,'count':2,
            'transform':[0,-1,0,5, 1,0,0,0, 0,0,1,0],
            'bounds':[.5,0,0,1,0,0]}]}))
    index = OptimizedBlocks(manifest.open('rb'))
    try:
        # Later appends must never enter this version's immutable prefix.
        with archive.open('ab') as f:f.write(struct.pack('<ffffI',100,100,100,1,0))
        payload,count = index.overview(100)
        assert count == 2
        origin = struct.unpack_from('<3f',payload,12)
        assert np.allclose(origin,[5,.5,0])
        assert payload[30:34] == bytes([0x12,0x34,0x56,42])
        assert struct.unpack_from('<I',index.tile(0,0,0,100),4)[0] == 0
        tile = index.tile(1,0,0,100)
        assert struct.unpack_from('<I',tile,4)[0] == 2
        assert struct.unpack_from('<hhh',tile,24) == (0,50,0)
    finally:index.close()


def test_superseded_archive_stops_indexing(tmp_path):
    path = tmp_path/'dense.pcd'
    path.write_bytes(b'FIELDS x y z intensity rgb\nSIZE 4 4 4 4 4\nTYPE F F F F F\nCOUNT 1 1 1 1 1\nPOINTS 1\nDATA binary\n'+struct.pack('<ffffI',1,2,3,42,0x123456))
    with pytest.raises(InterruptedError):
        OptimizedTiles(path, cancelled=lambda: True)


def test_dense_tile_keeps_rgb_and_pins_snapshot(tmp_path):
    path = tmp_path/'cloud_color_optimized.pcd'
    header = ('FIELDS x y z intensity rgb\nSIZE 4 4 4 4 4\nTYPE F F F F F\n'
              'COUNT 1 1 1 1 1\nPOINTS 1000\nDATA binary\n').encode()
    with path.open('wb') as f:
        f.write(header)
        for i in range(1000):
            f.write(struct.pack('<ffffI',(i%100)*.02,(i//100)*.02,0,42,0x123456))
    index = OptimizedTiles(path)
    try:
        payload = index.tile(0,0,0,2000)
        assert struct.unpack_from('<I',payload,4)[0] == 1000
        assert payload[30:34] == bytes([0x12,0x34,0x56,42])
        overview,count = index.overview(100)
        assert count == 100 and len(overview) == 1024
        other = tmp_path/'next.pcd'
        other.write_bytes(b'not a pcd')
        other.replace(path)
        assert index.tile(0,0,0,2000) == payload
        assert struct.unpack_from('<I',index.tile(-1,0,0,20),4)[0] == 0
    finally:
        index.close()


def test_archive_commit_and_http_version_guard(tmp_path):
    path = tmp_path/'dense.pcd'
    path.write_bytes(b'FIELDS x y z intensity rgb\nSIZE 4 4 4 4 4\nTYPE F F F F F\nCOUNT 1 1 1 1 1\nPOINTS 1\nDATA binary\n'+struct.pack('<ffffI',1,2,3,42,0x123456))
    node = SimpleNamespace(_lock=threading.Lock(),max_points=100,_preview_epoch=0,
                           _archive_notice=1,_optimized_trajectory=None,
                           _map_generation=4,_delta_queue=[],_stats={'frames':0},
                           _pose_version=0,_pending_tiles={})
    CloudNode._prepare_archive(node,str(path),1,0)
    assert node._optimized_ready and node._map_generation == 5
    assert node._latest_topic == '/lio/optimized_colored_map'
    async def run():
        server = WebServer(node,'127.0.0.1',0)
        await server.start()
        port = server.server.sockets[0].getsockname()[1]
        try:
            for version,code in [(4,b'409'),(5,b'200')]:
                reader,writer = await asyncio.open_connection('127.0.0.1',port)
                writer.write(f'GET /tile?x=0&y=0&z=0&max=10&generation={version} HTTP/1.1\r\nHost: test\r\n\r\n'.encode())
                await writer.drain()
                headers,body = (await reader.read()).split(b'\r\n\r\n',1)
                assert code in headers
                if version==5:assert body[:4]==b'LIT3'
                writer.close();await writer.wait_closed()
        finally:
            server.server.close();await server.server.wait_closed()
    try:
        asyncio.run(run())
    finally:
        node._optimized_tiles.close()
