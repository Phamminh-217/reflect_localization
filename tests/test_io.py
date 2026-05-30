"""Tests for the IO module (pointcloud_parser and bag_reader)."""

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from rf_threshold.core.frame import LidarFrame
from rf_threshold.io.pointcloud_parser import parse_pointcloud_message


class MockField:
    """Mock class for ROS message fields."""

    def __init__(self, name: str) -> None:
        self.name = name


def test_parse_pointcloud_message_success() -> None:
    """Test successful parsing of a valid PointCloud2 message."""
    # Create a mock PointCloud2 message
    msg = MagicMock()
    msg._type = "sensor_msgs/PointCloud2"
    msg.fields = [
        MockField("x"),
        MockField("y"),
        MockField("z"),
        MockField("intensity"),
    ]
    msg.header.stamp.to_sec.return_value = 123.456
    msg.header.frame_id = "livox_frame"

    # Mock data to be returned by pc2.read_points
    mock_points = [
        (1.0, 2.0, 3.0, 150.0),
        (4.0, 5.0, 6.0, 200.0),
        (7.0, 8.0, 9.0, 100.0),
    ]

    with patch("sensor_msgs.point_cloud2.read_points", return_value=mock_points):
        frame = parse_pointcloud_message(msg)

    # Verify LidarFrame properties
    assert isinstance(frame, LidarFrame)
    assert frame.stamp == 123.456
    assert frame.frame_id == "livox_frame"
    assert frame.points_xyz.shape == (3, 3)
    assert frame.intensity.shape == (3,)

    # Verify exact values
    np.testing.assert_array_equal(
        frame.points_xyz,
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]),
    )
    np.testing.assert_array_equal(frame.intensity, np.array([150.0, 200.0, 100.0]))


def test_parse_pointcloud_message_invalid_type() -> None:
    """Test that parser raises TypeError for unsupported message types."""
    msg = MagicMock()
    msg._type = "sensor_msgs/LaserScan"

    with pytest.raises(TypeError, match="Unsupported message type"):
        parse_pointcloud_message(msg)


def test_parse_pointcloud_message_missing_fields() -> None:
    """Test that parser raises ValueError when required fields are missing."""
    msg = MagicMock()
    msg._type = "sensor_msgs/PointCloud2"
    # Missing 'intensity' field
    msg.fields = [MockField("x"), MockField("y"), MockField("z")]

    with pytest.raises(ValueError, match="Required field 'intensity' not found"):
        parse_pointcloud_message(msg)


def test_parse_pointcloud_message_empty_data() -> None:
    """Test that parser handles empty point cloud data correctly."""
    msg = MagicMock()
    msg._type = "sensor_msgs/PointCloud2"
    msg.fields = [
        MockField("x"),
        MockField("y"),
        MockField("z"),
        MockField("intensity"),
    ]
    msg.header.stamp.to_sec.return_value = 12.0
    msg.header.frame_id = "empty_frame"

    with patch("sensor_msgs.point_cloud2.read_points", return_value=[]):
        frame = parse_pointcloud_message(msg)

    assert frame.points_xyz.shape == (0, 3)
    assert frame.intensity.shape == (0,)
