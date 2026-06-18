"""Unit tests for the FallbackManager class."""

import pytest
from rf_threshold.localization.pose import (
    LocalizationResult,
    LocalizationStatus,
    RobotPose,
)
from rf_threshold.localization.fallback_manager import FallbackManager, FallbackOutput


@pytest.fixture
def base_pose() -> RobotPose:
    """Create a sample SVD resolved RobotPose."""
    return RobotPose(
        stamp=100.0,
        frame_id="map_frame",
        child_frame_id="lidar_frame",
        x=2.5,
        y=1.2,
        yaw=0.05,
        residual_rmse=0.02,
        num_matches=4,
    )


def test_fallback_manager_ok_update(base_pose: RobotPose) -> None:
    """Verify that an OK localization updates last_valid_pose and resets fallback count."""
    cfg = {"fallback": {"enabled": True, "max_consecutive_fallback_frames": 3}}
    manager = FallbackManager(cfg)

    # Initially None
    assert manager.last_valid_pose is None
    assert manager.consecutive_fallback_count == 0

    res = LocalizationResult(
        stamp=100.0,
        status=LocalizationStatus.OK,
        pose=base_pose,
        matched_pairs=[],
        residual_rmse=0.02,
        reason="Success",
        debug_info={},
    )

    out = manager.handle_result(100, 100.0, res)

    assert out.status == LocalizationStatus.OK
    assert out.pose == base_pose
    assert not out.is_fallback
    assert out.consecutive_fallback_count == 0
    assert manager.last_valid_pose == base_pose


def test_fallback_manager_trigger_fallback(base_pose: RobotPose) -> None:
    """Verify that a failed localization triggers fallback to the updated stamp."""
    cfg = {"fallback": {"enabled": True, "max_consecutive_fallback_frames": 3}}
    manager = FallbackManager(cfg)

    # Setup last valid pose
    res_ok = LocalizationResult(
        stamp=100.0,
        status=LocalizationStatus.OK,
        pose=base_pose,
        matched_pairs=[],
        residual_rmse=0.02,
        reason="Success",
        debug_info={},
    )
    manager.handle_result(100, 100.0, res_ok)

    # Trigger failure
    res_fail = LocalizationResult(
        stamp=101.0,
        status=LocalizationStatus.INSUFFICIENT_DETECTIONS,
        pose=None,
        matched_pairs=[],
        residual_rmse=None,
        reason="Only 2 detections",
        debug_info={},
    )

    out = manager.handle_result(101, 101.0, res_fail)

    assert out.status == LocalizationStatus.FALLBACK_LAST_VALID_POSE
    assert out.is_fallback
    assert out.fallback_source == "last_valid_pose"
    assert out.consecutive_fallback_count == 1
    assert out.pose is not None
    assert out.pose.stamp == 101.0  # Stamp is updated
    assert out.pose.x == 2.5
    assert out.pose.y == 1.2
    assert out.pose.yaw == 0.05


def test_fallback_manager_no_previous_pose() -> None:
    """Verify that if no last_valid_pose exists, failure is returned directly."""
    cfg = {"fallback": {"enabled": True, "max_consecutive_fallback_frames": 3}}
    manager = FallbackManager(cfg)

    res_fail = LocalizationResult(
        stamp=100.0,
        status=LocalizationStatus.ASSOCIATION_FAILED,
        pose=None,
        matched_pairs=[],
        residual_rmse=None,
        reason="Association failed",
        debug_info={},
    )

    out = manager.handle_result(100, 100.0, res_fail)

    assert out.status == LocalizationStatus.ASSOCIATION_FAILED
    assert out.pose is None
    assert not out.is_fallback
    assert out.consecutive_fallback_count == 0


def test_fallback_manager_limit_exceeded(base_pose: RobotPose) -> None:
    """Verify that fallback stops after exceeding max_consecutive_fallback_frames."""
    cfg = {"fallback": {"enabled": True, "max_consecutive_fallback_frames": 2}}
    manager = FallbackManager(cfg)

    # 1. OK frame
    res_ok = LocalizationResult(
        stamp=100.0,
        status=LocalizationStatus.OK,
        pose=base_pose,
        matched_pairs=[],
        residual_rmse=0.02,
        reason="Success",
        debug_info={},
    )
    manager.handle_result(100, 100.0, res_ok)

    # 2. Failure 1 -> fallback ok
    res_fail = LocalizationResult(
        stamp=101.0,
        status=LocalizationStatus.INSUFFICIENT_DETECTIONS,
        pose=None,
        matched_pairs=[],
        residual_rmse=None,
        reason="Fail",
        debug_info={},
    )
    out1 = manager.handle_result(101, 101.0, res_fail)
    assert out1.status == LocalizationStatus.FALLBACK_LAST_VALID_POSE
    assert out1.consecutive_fallback_count == 1

    # 3. Failure 2 -> fallback ok
    out2 = manager.handle_result(102, 102.0, res_fail)
    assert out2.status == LocalizationStatus.FALLBACK_LAST_VALID_POSE
    assert out2.consecutive_fallback_count == 2

    # 4. Failure 3 -> fallback rejected (limit is 2)
    out3 = manager.handle_result(103, 103.0, res_fail)
    assert out3.status == LocalizationStatus.INSUFFICIENT_DETECTIONS
    assert out3.pose is None
    assert not out3.is_fallback


def test_fallback_manager_disabled(base_pose: RobotPose) -> None:
    """Verify that if fallback is disabled, failed frames are never recovered."""
    cfg = {"fallback": {"enabled": False, "max_consecutive_fallback_frames": 5}}
    manager = FallbackManager(cfg)

    # OK frame
    res_ok = LocalizationResult(
        stamp=100.0,
        status=LocalizationStatus.OK,
        pose=base_pose,
        matched_pairs=[],
        residual_rmse=0.02,
        reason="Success",
        debug_info={},
    )
    manager.handle_result(100, 100.0, res_ok)

    # Failure
    res_fail = LocalizationResult(
        stamp=101.0,
        status=LocalizationStatus.INSUFFICIENT_DETECTIONS,
        pose=None,
        matched_pairs=[],
        residual_rmse=None,
        reason="Fail",
        debug_info={},
    )
    out = manager.handle_result(101, 101.0, res_fail)

    assert out.status == LocalizationStatus.INSUFFICIENT_DETECTIONS
    assert out.pose is None
    assert not out.is_fallback
