"""Module for clustering high-intensity points using DBSCAN."""

import logging
from typing import Any, Dict, List

import numpy as np
from sklearn.cluster import DBSCAN

from rf_threshold.core.frame import LidarFrame, RFCluster

logger = logging.getLogger("clustering")


def cluster_bright_points(
    frame: LidarFrame,
    cfg: Dict[str, Any],
) -> List[RFCluster]:
    """Cluster bright points in a LidarFrame using DBSCAN.

    Supports clustering in 2D (xy planar) or 3D (xyz space).

    Args:
        frame: LidarFrame containing only bright points.
        cfg: Clustering configuration dictionary.

    Returns:
        A list of RFCluster objects representing candidate landmarks.

    Raises:
        KeyError: If required configuration keys are missing.
        ValueError: If an unsupported clustering method or dimension is specified.
    """
    if frame.points_xyz.shape[0] == 0:
        logger.debug("No bright points available for clustering. Returning empty list.")
        return []

    try:
        method = cfg["method"]
        eps = float(cfg["eps"])
        min_samples = int(cfg["min_samples"])
        use_dimension = cfg["use_dimension"]
    except KeyError as exc:
        logger.error("Missing required clustering configuration key.")
        raise KeyError(
            "Missing clustering keys: method, eps, min_samples, or use_dimension"
        ) from exc

    if method != "dbscan":
        logger.error("Unsupported clustering method: %s", method)
        raise ValueError(f"Unsupported clustering method: {method}")

    # Select features based on dimension configuration (xy or xyz)
    if use_dimension == "xy":
        features = frame.points_xyz[:, :2]
    elif use_dimension == "xyz":
        features = frame.points_xyz
    else:
        logger.error("Unsupported clustering dimension: %s", use_dimension)
        raise ValueError(f"Unsupported use_dimension: {use_dimension}")

    # Run DBSCAN
    try:
        dbscan = DBSCAN(eps=eps, min_samples=min_samples).fit(features)
    except Exception as exc:
        logger.error("DBSCAN clustering failed: %s", exc)
        raise ValueError(f"DBSCAN failed: {exc}") from exc

    labels = dbscan.labels_
    unique_labels = set(labels)

    clusters: List[RFCluster] = []
    for label in unique_labels:
        # Label -1 indicates noise in DBSCAN
        if label == -1:
            continue

        cluster_mask = labels == label
        point_indices = np.where(cluster_mask)[0]
        points_xyz = frame.points_xyz[cluster_mask]
        intensity = frame.intensity[cluster_mask]

        cluster = RFCluster(
            cluster_id=int(label),
            point_indices=point_indices,
            points_xyz=points_xyz,
            intensity=intensity,
        )
        clusters.append(cluster)

    logger.debug(
        "Clustering found %d valid clusters (noise points: %d)",
        len(clusters),
        np.sum(labels == -1),
    )

    return clusters
