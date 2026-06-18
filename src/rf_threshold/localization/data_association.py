"""Module for Triplet Distance-Constrained Data Association."""

from dataclasses import dataclass
import logging
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

from rf_threshold.core.frame import RFDetection
from rf_threshold.localization.pose import LocalizationStatus, MatchedPair
from rf_threshold.localization.svd_pose import estimate_pose_svd_2d

logger = logging.getLogger("data_association")


@dataclass(frozen=True)
class TripletDescriptor:
    """Represents a geometric descriptor for a triplet of points.

    Args:
        ids: Tuple of the 3 point identifiers.
        points_xy: NumPy array of shape (3, 2) containing point coordinates.
        edge_lengths_sorted: Sorted edge lengths of the triplet, shape (3,).
    """
    ids: Tuple[int, int, int]
    points_xy: np.ndarray
    edge_lengths_sorted: np.ndarray

    def __post_init__(self) -> None:
        """Validate descriptor data."""
        if len(self.ids) != 3:
            raise ValueError(f"ids must have length 3, got {len(self.ids)}")
        if self.points_xy.shape != (3, 2):
            raise ValueError(f"points_xy must have shape (3, 2), got {self.points_xy.shape}")
        if self.edge_lengths_sorted.shape != (3,):
            raise ValueError(f"edge_lengths_sorted must have shape (3,), got {self.edge_lengths_sorted.shape}")


@dataclass(frozen=True)
class AssociationCandidate:
    """Represents a matched candidate registration solution.

    Args:
        matched_pairs: List of matched detection-landmark pairs.
        residual_rmse: RMSE of the registered inlier set.
        max_residual: Maximum residual distance among inliers.
        num_inliers: Count of inlier matches.
        score: Evaluated score tuple for ranking.
    """
    matched_pairs: List[MatchedPair]
    residual_rmse: float
    max_residual: float
    num_inliers: int
    score: Tuple[int, float, float]


@dataclass(frozen=True)
class AssociationResult:
    """Output results of the data association pipeline.

    Args:
        status: Localization outcome status.
        matched_pairs: Optimal matching pairs found, if any.
        residual_rmse: RMSE of the best registration, if successful.
        num_inliers: Count of inlier matches.
        reason: Diagnostic reason string.
        debug_info: Custom diagnostics dictionary.
    """
    status: LocalizationStatus
    matched_pairs: List[MatchedPair]
    residual_rmse: Optional[float]
    num_inliers: int
    reason: str
    debug_info: Dict[str, Any]

    def __post_init__(self) -> None:
        """Validate association result."""
        if not isinstance(self.status, LocalizationStatus):
            raise TypeError("status must be a LocalizationStatus enum")
        if not isinstance(self.matched_pairs, list):
            raise TypeError("matched_pairs must be a list")
        if not isinstance(self.debug_info, dict):
            raise TypeError("debug_info must be a dict")


def compute_adaptive_tolerance(
    distance: float,
    min_abs: float,
    relative_ratio: float,
    max_abs: float,
) -> float:
    """Compute adaptive tolerance dynamic threshold based on distance.

    Formula:
        tolerance = min(max_abs, max(min_abs, relative_ratio * distance))

    Args:
        distance: Distance value in meters.
        min_abs: Minimum absolute tolerance floor.
        relative_ratio: Ratio scaling factor.
        max_abs: Maximum absolute tolerance cap.

    Returns:
        The calculated adaptive tolerance.
    """
    tol = max(min_abs, relative_ratio * distance)
    return float(min(max_abs, tol))


def _enforce_one_to_one(
    potential_matches: List[Tuple[int, int, float]]
) -> List[Tuple[int, int]]:
    """Greedily resolve potential matches to enforce one-to-one correspondence.

    Args:
        potential_matches: List of tuples (det_idx, landmark_idx, distance).

    Returns:
        List of unique (det_idx, landmark_idx) matches.
    """
    # Sort matches by distance ascending
    sorted_matches = sorted(potential_matches, key=lambda x: x[2])

    matched_dets = set()
    matched_landmarks = set()
    inliers = []

    for det_idx, landmark_idx, _ in sorted_matches:
        if det_idx not in matched_dets and landmark_idx not in matched_landmarks:
            matched_dets.add(det_idx)
            matched_landmarks.add(landmark_idx)
            inliers.append((det_idx, landmark_idx))

    return inliers


def associate_detections_to_map(
    detections: List[RFDetection],
    landmarks: Any,  # List[RFMapLandmark]
    cfg: Dict[str, Any],
) -> AssociationResult:
    """Find the optimal association between observed detections and map landmarks.

    Uses the Triplet Distance-Constrained Data Association algorithm.

    Args:
        detections: Observed RF detections in lidar frame.
        landmarks: Known RF landmarks in map frame.
        cfg: Configuration dictionary containing "data_association" settings.

    Returns:
        An AssociationResult containing status, best matches, and diagnostic details.
    """
    # 1. Pre-check inputs
    da_cfg = cfg.get("data_association", {})
    min_detections = da_cfg.get("min_detections", 3)
    min_matches = da_cfg.get("min_matches", 3)

    if len(detections) < min_detections:
        msg = f"Insufficient detections: {len(detections)} < {min_detections}"
        logger.warning(msg)
        return AssociationResult(
            status=LocalizationStatus.INSUFFICIENT_DETECTIONS,
            matched_pairs=[],
            residual_rmse=None,
            num_inliers=0,
            reason=msg,
            debug_info={},
        )

    if len(landmarks) < min_matches:
        msg = f"Insufficient landmarks in map: {len(landmarks)} < {min_matches}"
        logger.warning(msg)
        return AssociationResult(
            status=LocalizationStatus.MAP_ERROR,
            matched_pairs=[],
            residual_rmse=None,
            num_inliers=0,
            reason=msg,
            debug_info={},
        )

    # Convert positions to numpy 2D arrays
    dets_xy = np.array([d.center_lidar[:2] for d in detections], dtype=np.float64)
    landmarks_xy = np.array([l.position_map[:2] for l in landmarks], dtype=np.float64)

    # 2. Generate observed triplets
    obs_triplets = []
    n_dets = len(detections)
    for i in range(n_dets):
        for j in range(i + 1, n_dets):
            for k in range(j + 1, n_dets):
                pts = dets_xy[[i, j, k]]
                d12 = np.linalg.norm(pts[0] - pts[1])
                d23 = np.linalg.norm(pts[1] - pts[2])
                d31 = np.linalg.norm(pts[2] - pts[0])
                edges_sorted = np.sort([d12, d23, d31])
                obs_triplets.append(TripletDescriptor((i, j, k), pts, edges_sorted))

    # 3. Generate map triplets
    map_triplets = []
    n_landmarks = len(landmarks)
    for i in range(n_landmarks):
        for j in range(i + 1, n_landmarks):
            for k in range(j + 1, n_landmarks):
                pts = landmarks_xy[[i, j, k]]
                d12 = np.linalg.norm(pts[0] - pts[1])
                d23 = np.linalg.norm(pts[1] - pts[2])
                d31 = np.linalg.norm(pts[2] - pts[0])
                edges_sorted = np.sort([d12, d23, d31])
                map_triplets.append(TripletDescriptor((i, j, k), pts, edges_sorted))

    # 4. Filter triplets using adaptive edge tolerance
    tol_cfg = da_cfg.get("triplet_distance_tolerance", {})
    min_abs = tol_cfg.get("min_abs", 0.08)
    relative_ratio = tol_cfg.get("relative_ratio", 0.03)
    max_abs = tol_cfg.get("max_abs", 0.20)

    matching_candidates = []

    for obs_tr in obs_triplets:
        for map_tr in map_triplets:
            # Compare edges
            match_ok = True
            errors = []
            for e_idx in range(3):
                obs_e = obs_tr.edge_lengths_sorted[e_idx]
                map_e = map_tr.edge_lengths_sorted[e_idx]
                eps = compute_adaptive_tolerance(map_e, min_abs, relative_ratio, max_abs)
                err = abs(obs_e - map_e)
                if err > eps:
                    match_ok = False
                    break
                errors.append(err)

            if match_ok:
                total_err = float(sum(errors))
                matching_candidates.append((obs_tr, map_tr, total_err))

    # 5. Limit excessive candidates by triplet distance error
    matching_candidates.sort(key=lambda x: x[2])
    max_candidates = da_cfg.get("max_candidates", 300)
    matching_candidates = matching_candidates[:max_candidates]

    # Permutations mapping index (0,1,2) to all 6 permutations
    permutations = [
        (0, 1, 2), (0, 2, 1),
        (1, 0, 2), (1, 2, 0),
        (2, 0, 1), (2, 1, 0)
    ]

    valid_candidates: List[AssociationCandidate] = []

    # Helper NN configuration
    nn_cfg = da_cfg.get("nearest_neighbor_gate", {})
    nn_min_abs = nn_cfg.get("min_abs", 0.10)
    nn_relative_ratio = nn_cfg.get("relative_ratio", 0.03)
    nn_max_abs = nn_cfg.get("max_abs", 0.25)

    max_candidate_rmse = da_cfg.get("max_candidate_rmse", 0.08)
    max_candidate_residual = da_cfg.get("max_candidate_residual", 0.18)

    # 6. Try all filtered candidate combinations & permutations
    for obs_tr, map_tr, _ in matching_candidates:
        for perm in permutations:
            # Match: obs_tr.ids[i] maps to map_tr.ids[perm[i]]
            pts_lidar = obs_tr.points_xy
            pts_map = map_tr.points_xy[list(perm)]

            # 7. Run SVD for each candidate
            try:
                # estimate_pose_svd_2d requires N >= 3 points
                svd_res = estimate_pose_svd_2d(pts_lidar, pts_map)
            except ValueError:
                # Catch only ValueError from SVD candidates and continue safely
                continue

            R_cand = svd_res.R
            t_cand = svd_res.t

            # 8. Transform all detections to map frame
            transformed_xy = (R_cand @ dets_xy.T).T + t_cand

            # 9. Nearest-neighbor verification with adaptive range-based gate
            potential_matches = []
            for d_idx in range(n_dets):
                # Calculate distance in lidar frame (detection range)
                r = float(np.linalg.norm(dets_xy[d_idx]))
                gate = compute_adaptive_tolerance(r, nn_min_abs, nn_relative_ratio, nn_max_abs)

                for l_idx in range(n_landmarks):
                    dist = float(np.linalg.norm(transformed_xy[d_idx] - landmarks_xy[l_idx]))
                    if dist <= gate:
                        potential_matches.append((d_idx, l_idx, dist))

            # 10. Enforce one-to-one matching
            inliers = _enforce_one_to_one(potential_matches)

            # 11. Re-estimate SVD using all inliers
            if len(inliers) < min_matches:
                continue

            inlier_lidar_xy = dets_xy[[m[0] for m in inliers]]
            inlier_map_xy = landmarks_xy[[m[1] for m in inliers]]

            try:
                final_svd = estimate_pose_svd_2d(inlier_lidar_xy, inlier_map_xy)
            except ValueError:
                continue

            # 12. Reject by min_matches, RMSE, max_residual
            if len(inliers) < min_matches:
                continue
            if final_svd.residual_rmse > max_candidate_rmse:
                continue
            
            # Check maximum residual constraint
            max_res_val = float(np.max(final_svd.residuals))
            if max_res_val > max_candidate_residual:
                continue

            # Construct MatchedPair list
            matched_pairs = []
            for det_idx, l_idx in inliers:
                matched_pairs.append(
                    MatchedPair(
                        detection_id=detections[det_idx].detection_id,
                        landmark_id=landmarks[l_idx].landmark_id,
                        point_lidar=detections[det_idx].center_lidar,
                        point_map=landmarks[l_idx].position_map,
                    )
                )

            # 13. Select best candidate score
            # Score tuple ordered: (inliers count, negative RMSE, negative max_residual)
            score = (len(inliers), -final_svd.residual_rmse, -max_res_val)

            valid_candidates.append(
                AssociationCandidate(
                    matched_pairs=matched_pairs,
                    residual_rmse=final_svd.residual_rmse,
                    max_residual=max_res_val,
                    num_inliers=len(inliers),
                    score=score,
                )
            )

    # 14. Return AssociationResult
    debug_stats = {
        "num_observed_triplets": len(obs_triplets),
        "num_map_triplets": len(map_triplets),
        "num_triplet_candidates": len(matching_candidates),
        "num_svd_candidates": len(valid_candidates),
    }

    if not valid_candidates:
        msg = "No valid SVD correspondence candidate found under threshold."
        logger.warning(msg)
        return AssociationResult(
            status=LocalizationStatus.ASSOCIATION_FAILED,
            matched_pairs=[],
            residual_rmse=None,
            num_inliers=0,
            reason=msg,
            debug_info=debug_stats,
        )

    # Sort candidates to find the best (highest score)
    valid_candidates.sort(key=lambda x: x.score, reverse=True)
    best = valid_candidates[0]

    debug_stats.update({
        "best_num_inliers": best.num_inliers,
        "best_residual_rmse": best.residual_rmse,
        "best_max_residual": best.max_residual,
    })

    logger.info(
        "Data association succeeded: inliers=%d, rmse=%.3fm",
        best.num_inliers,
        best.residual_rmse,
    )

    return AssociationResult(
        status=LocalizationStatus.OK,
        matched_pairs=best.matched_pairs,
        residual_rmse=best.residual_rmse,
        num_inliers=best.num_inliers,
        reason="Optimal registration candidate found successfully.",
        debug_info=debug_stats,
    )
