"""Synthetic ROS -> worker/native encoder -> HTTP/WebSocket runtime smoke test.

The isolated DDS domain and unique topic never use scanner data.
"""
import asyncio
import struct
import uuid

import aiohttp
from aiohttp import ClientSession, WSMsgType
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from lio_web.cloud import encode_cloud
from lio_web.server import CloudNode, WebServer


def test_ros_pointcloud_reaches_websocket():
    topic = '/lio_web_native_test_'+uuid.uuid4().hex
    rclpy.init(domain_id=177,args=['--ros-args','-p',f'pointcloud_topic:={topic}',
                                '-p',"fallback_pointcloud_topic:=''",
                                '-p','voxel_size:=0.01'])
    node = CloudNode()
    producer = Node('lio_web_native_test_producer')
    publisher = producer.create_publisher(PointCloud2,topic,qos_profile_sensor_data)
    msg = PointCloud2()
    msg.header.frame_id = 'map'
    msg.width, msg.height, msg.point_step, msg.row_step = 2,1,20,40
    msg.fields = [PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                  for i,name in enumerate(['x','y','z','intensity','rgb'])]
    msg.data = struct.pack('<ffffIffffI',1,2,3,42,0x123456,1.5,2,3,.5,0xabcdef)
    expected,count = encode_cloud(msg,200000,.01)

    async def run():
        server = WebServer(node,'127.0.0.1',0)
        await server.start()
        port = server.server.sockets[0].getsockname()[1]
        try:
            # Spin both nodes until DDS discovery and native worker completion.
            async def publish_until_ready():
                while node.snapshot()[0] is None:
                    publisher.publish(msg)
                    rclpy.spin_once(producer,timeout_sec=0)
                    rclpy.spin_once(node,timeout_sec=0)
                    await asyncio.sleep(.02)
            await asyncio.wait_for(publish_until_ready(),10)
            assert node.snapshot()[0] == expected
            async with ClientSession() as client:
                async with client.get(f'http://127.0.0.1:{port}/') as response:
                    assert response.status == 200
                    assert '<canvas' in await response.text()
                # aiohttp 3.8 (Humble apt) and newer releases use different APIs.
                close_timeout = (aiohttp.ClientWSTimeout(ws_close=.5)
                                 if hasattr(aiohttp,'ClientWSTimeout') else .5)
                async with client.ws_connect(f'http://127.0.0.1:{port}/ws',timeout=close_timeout) as websocket:
                    for _ in range(10):
                        frame = await websocket.receive(timeout=3)
                        if frame.type == WSMsgType.BINARY:
                            assert frame.data == expected
                            assert struct.unpack_from('<I',frame.data,4)[0] == count == 2
                            break
                    else:
                        raise AssertionError('no native point-cloud frame on WebSocket')
        finally:
            for writer in list(server.clients):
                writer.close()
            server.server.close()
            await server.server.wait_closed()
    try:
        asyncio.run(run())
    finally:
        node.destroy_node()
        producer.destroy_node()
        rclpy.shutdown()
