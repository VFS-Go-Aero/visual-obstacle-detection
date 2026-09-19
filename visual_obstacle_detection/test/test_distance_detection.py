"""Regression checks for sparse surfaces, noise and confirmation history."""
import numpy as np
from visual_obstacle_detection.obstacle_detection import build_sector_map
from visual_obstacle_detection.detection_support import required_points, CandidateConfirmation


def surface(n, distance):
    return np.column_stack((np.full(n, distance),
                            np.linspace(0.02, 0.06, n), np.full(n, 0.04)))


def detected(points):
    ranges = {}
    mask, sectors = build_sector_map(points, nearest_ranges=ranges)
    return mask.sum(), ranges


def test_distance_thresholds():
    assert list(required_points(np.array([1, 2, 3, 4, 5]))) == [100, 100, 45, 25, 20]
    assert detected(surface(25, 4.0))[0] == 1
    assert detected(surface(24, 4.0))[0] == 0
    assert detected(surface(25, 2.0))[0] == 0
    assert detected(surface(100, 2.0))[0] == 1
    assert detected(surface(20, 4.9))[0] == 1
    assert detected(surface(100, 5.01))[0] == 0


def test_shell_boundary_and_nearest_range():
    points = surface(50, 3.0)
    points[:, 0] = np.linspace(2.96, 3.04, 50)
    count, ranges = detected(points)
    assert count == 1
    assert np.isclose(next(iter(ranges.values())), np.linalg.norm(points, axis=1).min())


def test_disconnected_noise_and_invalid_data():
    # Isolated points in the same sector and radial shell cannot pool support.
    yz = np.array([(y, z) for y in np.arange(0.1, 2.0, 0.4)
                   for z in np.arange(0.1, 1.6, 0.4)])
    x = np.sqrt(4.5 ** 2 - (yz ** 2).sum(axis=1))
    points = np.column_stack((x, yz))
    assert detected(points)[0] == 0
    assert detected(np.array([[np.nan, 0, 0], [0, 0, 0], [np.inf, 0, 0]]))[0] == 0
    assert detected(np.empty((0, 3)))[0] == 0


def test_confirmation_and_expiry():
    history = CandidateConfirmation()
    points, sectors = surface(1, 4), np.array([32])
    assert not history.update(points, sectors, 1.0)[0]
    assert history.update(points, sectors, 1.15)[0]
    assert not history.update(surface(1, 3), sectors, 1.3)[0]
    history.update(np.empty((0, 3)), np.empty(0), 1.4)
    assert not history.update(points, sectors, 1.5)[0]
    assert not history.update(points, sectors, 2.6)[0]
    assert history.update(points, sectors, 2.7)[0]


def test_empty_cloud_publishes_and_resets_confirmation():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from visual_obstacle_detection.obstacle_detection import ObstacleDetection
    from builtin_interfaces.msg import Time
    history = CandidateConfirmation()
    history.update(surface(1, 4), np.array([32]), 1.0)
    logger = Mock()
    node = SimpleNamespace(
        cloud=np.empty((0, 3), dtype=np.float32),
        _empty_detect_count=0, _zero_obs_streak=0,
        _confirmation=history, _last_n_obs=None, _frame_id='base_link',
        _verbose=False, pub=Mock(), get_logger=lambda: logger,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())),
    )
    ObstacleDetection._detect_and_publish(node)
    assert node.pub.publish.call_count == 1
    assert node.pub.publish.call_args.args[0].width == 0
    assert history.previous == {}


def test_one_miss_is_tolerated_and_hold_expires():
    history = CandidateConfirmation()
    points, sectors = surface(1, 4), np.array([32])
    empty_points, empty_sectors = np.empty((0, 3)), np.empty(0, dtype=int)
    assert len(history.stabilize(points, sectors, 1.0)[0]) == 0
    history.stabilize(empty_points, empty_sectors, 1.1)
    assert len(history.stabilize(points, sectors, 1.2)[0]) == 1
    assert len(history.stabilize(empty_points, empty_sectors, 1.4)[0]) == 1
    assert len(history.stabilize(empty_points, empty_sectors, 1.7)[0]) == 1
    assert len(history.stabilize(empty_points, empty_sectors, 1.81)[0]) == 0


def test_slow_frame_does_not_clear_confirmed_obstacle():
    history = CandidateConfirmation()
    points, sectors = surface(1, 4), np.array([32])
    history.stabilize(points, sectors, 1.0)
    assert len(history.stabilize(points, sectors, 1.65)[0]) == 1
    assert len(history.stabilize(surface(1, 3.9), sectors, 2.3)[0]) == 1


def test_bridge_stops_republishing_when_detector_stalls(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from builtin_interfaces.msg import Time
    import visual_obstacle_detection.obstacle_to_mavlink as bridge
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: 2.0)
    node = SimpleNamespace(
        _received_at=1.0, _latest_cloud=[(4.0, 0.0, 0.0, 32)],
        _pub_3d=Mock(), get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())),
    )
    bridge.ObstacleToMavlink._publish(node)
    assert node._latest_cloud is None
    node._pub_3d.publish.assert_not_called()
