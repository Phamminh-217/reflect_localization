"""Module for managing pose continuity and SVD fallback logic."""

from dataclasses import dataclass
from typing import Any, Dict, Optional

from rf_threshold.localization.pose import (
    LocalizationResult,
    LocalizationStatus,
    RobotPose,
)


@dataclass(frozen=True)
class FallbackOutput:
    """Combines fallback pose output, status, and diagnostic flags.

    Args:
        status: The finalized output status (either OK, a fallback code, or original error).
        pose: The resulting pose object (either SVD resolved, fallback updated, or None).
        is_fallback: True if the output is derived from historical fallback.
        fallback_source: Source of fallback, e.g. "last_valid_pose" or None.
        consecutive_fallback_count: Number of consecutive fallback frames up to this frame.
    """

    status: LocalizationStatus
    pose: Optional[RobotPose]
    is_fallback: bool
    fallback_source: Optional[str]
    consecutive_fallback_count: int

    def __post_init__(self) -> None:
        """Validate fallback output."""
        if not isinstance(self.status, LocalizationStatus):
            raise TypeError("status must be a LocalizationStatus enum")
        if not isinstance(self.is_fallback, bool):
            raise TypeError("is_fallback must be a boolean")
        if self.consecutive_fallback_count < 0:
            raise ValueError("consecutive_fallback_count must be non-negative")


class FallbackManager:
    """Manages robot pose continuity on SVD localization failures."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        """Initialize the FallbackManager with system configuration.

        Args:
            cfg: System configuration dictionary.
        """
        self.cfg = cfg
        fallback_cfg = cfg.get("fallback", {})
        self.enabled = fallback_cfg.get("enabled", True)
        self.mode = fallback_cfg.get("mode", "last_valid_pose")
        self.max_consecutive = fallback_cfg.get("max_consecutive_fallback_frames", 5)

        self.last_valid_pose: Optional[RobotPose] = None
        self.consecutive_fallback_count = 0

    def handle_result(
        self,
        frame_index: int,
        stamp: float,
        localization_result: LocalizationResult,
    ) -> FallbackOutput:
        """Process the frame result and apply fallback logic if necessary.

        Args:
            frame_index: Index of the current frame.
            stamp: Frame timestamp in seconds.
            localization_result: Raw result from the RFLocalizer pipeline.

        Returns:
            A FallbackOutput containing the final output status and pose.
        """
        if localization_result.status == LocalizationStatus.OK:
            # SVD localization succeeded
            self.last_valid_pose = localization_result.pose
            self.consecutive_fallback_count = 0

            return FallbackOutput(
                status=LocalizationStatus.OK,
                pose=localization_result.pose,
                is_fallback=False,
                fallback_source=None,
                consecutive_fallback_count=0,
            )

        # SVD localization failed
        if (
            self.enabled
            and self.last_valid_pose is not None
            and self.consecutive_fallback_count < self.max_consecutive
        ):
            # Apply fallback from last_valid_pose
            self.consecutive_fallback_count += 1
            fallback_pose = RobotPose(
                stamp=stamp,  # Update stamp to match current frame
                frame_id=self.last_valid_pose.frame_id,
                child_frame_id=self.last_valid_pose.child_frame_id,
                x=self.last_valid_pose.x,
                y=self.last_valid_pose.y,
                yaw=self.last_valid_pose.yaw,
                residual_rmse=self.last_valid_pose.residual_rmse,
                num_matches=self.last_valid_pose.num_matches,
            )

            return FallbackOutput(
                status=LocalizationStatus.FALLBACK_LAST_VALID_POSE,
                pose=fallback_pose,
                is_fallback=True,
                fallback_source="last_valid_pose",
                consecutive_fallback_count=self.consecutive_fallback_count,
            )

        # Fallback cannot be applied
        return FallbackOutput(
            status=localization_result.status,
            pose=None,
            is_fallback=False,
            fallback_source=None,
            consecutive_fallback_count=self.consecutive_fallback_count,
        )
