from types import SimpleNamespace
import threading

from lio_web.server import CloudNode
from lio_web.trajectory import TrajectoryHistory


def odom(stamp, x):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)),
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0, z=0))),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=1, y=0, z=0))))


def test_new_replay_unfreezes_optimized_view():
    node = SimpleNamespace(
        _lock=threading.Lock(), optimized_view=True,
        _last_raw_odom_stamp=100, _raw_odom_publisher=None, _preview_epoch=0,
        _optimized_ready=True,
        _optimized_path_ready=True, _optimized_trajectory=[[8, 0, 0]],
        _trajectory=TrajectoryHistory(32), _pose=[8, 0, 0],
        _distance=8, _first_odom_stamp=1, _last_odom_stamp=100,
        _scan_duration=99, _latest=b'old', _last_primary=1,
        _last_publish=1, _delta_queue=[b'old'], _colored_map_points=100,
        _map_generation=2, _pose_version=1, _camera_version=1,
        _camera_frame=b'old-camera', _stats={'stamp': 1})
    node._reset_preview_locked = lambda: CloudNode._reset_preview_locked(node)
    node._update_odom = lambda msg: CloudNode._update_odom(node, msg)
    CloudNode._on_odom(node, odom(101, 9))
    assert node._optimized_path_ready
    assert node._pose == [8, 0, 0]
    CloudNode._on_odom(node, odom(1, 0))
    assert not node._optimized_ready and not node._optimized_path_ready
    assert node._latest is None and not node._delta_queue
    assert node._map_generation == 3
    assert node._trajectory.snapshot() == [[0, 0, 0]]
    CloudNode._on_odom(node, odom(2, 1))
    assert node._trajectory.snapshot() == [[0, 0, 0], [1, 0, 0]]
    assert node._scan_duration == 1
    node._optimized_ready = True
    node._optimized_path_ready = True
    node._raw_odom_publisher = b'previous-publisher'
    CloudNode._on_odom(node, odom(3, 2), publisher_gid=b'new-publisher')
    assert not node._optimized_ready and not node._optimized_path_ready
    assert node._trajectory.snapshot() == [[2, 0, 0]]
    assert node._camera_frame is None
    node._camera_frame = b'current-camera'
    node._raw_odom_publisher = b'new-publisher'
    node.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(endpoint_gid=b'new-publisher')]
    CloudNode.refresh_preview_session(node)
    assert node._camera_frame == b'current-camera'  # Same-run reconnect keeps data.
    node.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(endpoint_gid=b'restarted-publisher')]
    CloudNode.refresh_preview_session(node)
    assert node._camera_frame is None
    assert node._pose is None and node._scan_duration is None
    assert not node._trajectory.snapshot()
