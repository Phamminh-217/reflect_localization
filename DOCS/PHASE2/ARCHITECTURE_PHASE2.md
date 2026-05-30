# ARCHITECTURE_PHASE2.md

# Kiến trúc Phase 2 — SVD-Based RF Localization

## 1. Tổng quan kiến trúc

Phase 2 được thiết kế theo mô hình:

```text
Modular Localization Pipeline
```

Dữ liệu đi theo dòng:

```text
Phase 1 RFDetection
        ↓
RF Map Loader
        ↓
Data Association
        ↓
Geometry Check
        ↓
SVD Pose Solver
        ↓
Localization Result
        ↓
Pose Output Writer
```

Nguyên tắc kiến trúc:

```text
Detection và Localization phải tách biệt.
Localization không được gọi thresholding, clustering hoặc preprocessing.
Localization chỉ nhận RFDetection và RF map.
```

---

## 2. Input / Output kiến trúc

## 2.1. Input chính

Phase 2 nhận:

```python
List[RFDetection]
```

Trong đó trường quan trọng nhất là:

```python
detection.center_lidar  # np.ndarray shape (3,)
```

và RF map:

```python
List[RFMapLandmark]
```

---

## 2.2. Output chính

Phase 2 trả về:

```python
LocalizationResult
```

Trong đó có:

```text
status
pose
matched_pairs
residual_rmse
num_detections
num_matches
debug_info
```

---

## 3. Cấu trúc thư mục Phase 2

```text
src/rf_threshold/localization/
│
├── __init__.py
├── pose.py
├── map_loader.py
├── data_association.py
├── geometry_check.py
├── svd_pose.py
├── localizer_pipeline.py
└── localization_writer.py
```

Tests:

```text
tests/
├── test_pose.py
├── test_map_loader.py
├── test_data_association.py
├── test_geometry_check.py
├── test_svd_pose.py
└── test_localizer_pipeline.py
```

Scripts:

```text
scripts/
├── run_svd_localization.py
└── evaluate_localization.py
```

Data:

```text
data/
├── maps/
│   └── rf_map_v1.json
└── results/
    └── <run_name>/
        ├── detections.json
        └── localization/
            ├── poses.csv
            ├── poses.json
            ├── rejected_frames.csv
            └── association_debug.csv
```

---

## 4. Module chính

## 4.1. `pose.py`

### Trách nhiệm

Định nghĩa các data class cho localization.

### Class cần có

```python
RFMapLandmark
MatchedPair
RobotPose
LocalizationResult
LocalizationStatus
```

### Không được làm

- Không đọc file.
- Không chạy SVD.
- Không matching.
- Không ghi output.

---

## 4.2. `map_loader.py`

### Trách nhiệm

Đọc bản đồ RF từ JSON.

### Input

```text
data/maps/rf_map_v1.json
```

### Output

```python
List[RFMapLandmark]
```

### Không được làm

- Không chạy matching.
- Không chạy SVD.
- Không sửa tọa độ map.
- Không tự sinh map giả nếu file lỗi.

---

## 4.3. `data_association.py`

### Trách nhiệm

Thực hiện giải thuật so khớp bộ ba giới hạn khoảng cách (Triplet Distance-Constrained Matching / Pairwise-Distance-Constrained RF Correspondence Search) để tìm các cặp tương ứng hợp lệ giữa các điểm mốc LiDAR quan sát được và bản đồ landmark toàn cục mà không dùng brute-force hay so khớp trực tiếp khi chưa có Pose.

### Cấu trúc dữ liệu thiết kế

#### 1. `TripletDescriptor`
Lưu trữ thông tin mô tả cho một bộ ba điểm (quan sát từ LiDAR hoặc từ bản đồ).
```python
@dataclass(frozen=True)
class TripletDescriptor:
    ids: Tuple[int, int, int]          # IDs của 3 điểm tạo nên triplet (detection IDs hoặc landmark IDs)
    points_xy: np.ndarray              # Tọa độ 2D của 3 điểm, shape (3, 2)
    edge_lengths_sorted: np.ndarray    # Chiều dài 3 cạnh được sắp xếp tăng dần, shape (3,)
```

#### 2. `AssociationCandidate`
Đại diện cho một phương án so khớp ứng viên sau khi chạy thử bộ giải SVD.
```python
@dataclass(frozen=True)
class AssociationCandidate:
    matched_pairs: List[MatchedPair]   # Danh sách các cặp điểm khớp
    residual_rmse: float               # Sai số RMSE của phép khớp SVD
    max_residual: float                # Sai số lớn nhất trong các điểm inliers
    num_inliers: int                   # Số lượng điểm inliers được giữ lại
    score: float                       # Điểm số đánh giá ứng viên (để xếp hạng)
```

#### 3. `AssociationResult`
Kết quả cuối cùng trả về từ pipeline so khớp.
```python
@dataclass(frozen=True)
class AssociationResult:
    status: LocalizationStatus         # Trạng thái kết quả (OK, INSUFFICIENT_DETECTIONS, v.v.)
    matched_pairs: List[MatchedPair]   # Danh sách cặp khớp tốt nhất tìm được
    residual_rmse: Optional[float]     # RMSE của kết quả khớp tốt nhất
    num_inliers: int                   # Số lượng inliers
    reason: str                        # Chi tiết lý do trạng thái (đặc biệt khi lỗi)
    debug_info: Dict[str, Any]         # Thông tin chẩn đoán
```

> **⚠️ Lưu ý tương thích Python 3.8:**
> Cấm sử dụng các kiểu gợi ý kiểu dữ liệu (typing) của Python 3.10+ như `MatchedPair | None`, `list[MatchedPair]`, hay `dict[str, Any]`.
> Bắt buộc phải import và sử dụng: `Optional[MatchedPair]`, `List[MatchedPair]`, `Dict[str, Any]`, `Tuple[int, int, int]`.

### Hàm tính sai số thích nghi (Adaptive Tolerance Helper)
Hàm helper tính toán sai số khoảng cách cho phép thích nghi theo độ dài cạnh thực tế:
```python
def compute_adaptive_tolerance(
    distance: float,
    min_abs: float,
    relative_ratio: float,
    max_abs: float,
) -> float:
    """Tính toán sai số cho phép động dựa trên khoảng cách."""
    tolerance = max(min_abs, relative_ratio * distance)
    tolerance = min(max_abs, tolerance)
    return tolerance
```

### Quy trình và sự tương tác với `svd_pose.py`
1. Từ danh sách `RFDetection`, tạo ra các `TripletDescriptor` quan sát.
2. Từ danh sách `RFMapLandmark`, tạo sẵn các `TripletDescriptor` bản đồ toàn cục.
3. Lọc nhanh các cặp triplet tương ứng bằng cách so sánh độ dài 3 cạnh đã sắp xếp:
   `abs(obs_edge_i - map_edge_i) <= compute_adaptive_tolerance(map_edge_i, ...)`
   *(Sử dụng map_edge_i để tính epsilon vì map là giá trị chuẩn hơn detection).*
4. Đối với mỗi cặp bộ ba ứng viên vượt qua vòng lọc cạnh, thử nghiệm tất cả $3! = 6$ hoán vị thứ tự khớp điểm.
5. Với mỗi hoán vị, gọi hàm `estimate_pose_svd_2d()` trong `svd_pose.py` để tìm Pose ứng viên.
6. Dùng Pose ứng viên thu được để transform tất cả `RFDetection` còn lại sang `map_frame`.
7. Áp dụng Nearest-Neighbor xác thực với ngưỡng thích nghi `nearest_neighbor_gate` để lọc inliers và đếm số lượng điểm khớp chính xác.
8. Tính toán điểm số `score` cho từng ứng viên theo thứ tự ưu tiên:
   - Số inliers nhiều nhất (`num_inliers` lớn nhất).
   - RMSE nhỏ nhất (`residual_rmse` nhỏ nhất).
   - Sai số cực đại nhỏ nhất (`max_residual` nhỏ nhất).
9. Trả về `AssociationResult` tối ưu nhất chứa `List[MatchedPair]`.


---

## 4.4. `geometry_check.py`

### Trách nhiệm

Kiểm tra điều kiện hình học trước khi chạy SVD.

### Input

```python
matched_pairs: List[MatchedPair]
```

### Output

```python
GeometryCheckResult
```

### Điều kiện cần kiểm tra

```text
num_pairs >= 3
spatial spread đủ lớn
không có duplicate landmark id
không có duplicate detection id
points không chứa NaN
condition number trong ngưỡng cho phép
```

### Lý do cần module này

Nếu RF gần như thẳng hàng hoặc quá sát nhau, SVD có thể cho pose nhạy với nhiễu. Vì hành lang có nhiều RF theo dạng gần thẳng hàng, module này là bắt buộc.

---

## 4.5. `svd_pose.py`

### Trách nhiệm

Tính SE(2) transform từ các cặp điểm tương ứng.

### Bài toán

```text
p_map ≈ R * p_lidar + t
```

Cực tiểu hóa:

```text
sum ||p_map_i - (R * p_lidar_i + t)||^2
```

### Input

```python
source_points_lidar: np.ndarray  # shape (N, 2)
target_points_map: np.ndarray    # shape (N, 2)
weights: Optional[np.ndarray]    # shape (N,)
```

### Output

```python
SVDPoseResult
```

Gồm:

```text
R
t
yaw
residuals
residual_rmse
```

### Không được làm

- Không đọc RF map.
- Không parse detection JSON.
- Không tự matching.
- Không fallback.

---

## 4.6. `localizer_pipeline.py`

### Trách nhiệm

Điều phối toàn bộ localization.

### Input

```python
detections: List[RFDetection]
landmarks: List[RFMapLandmark]
```

### Output

```python
LocalizationResult
```

### Flow

```text
Filter detections
  ↓
Check detection count
  ↓
Data association
  ↓
Check matched count
  ↓
Geometry check
  ↓
SVD solve
  ↓
Residual check
  ↓
Return LocalizationResult
```

### Không được làm

- Không viết lại SVD trong file pipeline.
- Không viết lại map loader trong file pipeline.
- Không gọi threshold detector.

---

## 4.7. `localization_writer.py`

### Trách nhiệm

Ghi output localization ra file.

### Output

```text
poses.csv
poses.json
rejected_frames.csv
association_debug.csv
localization_summary.csv
```

### Không được làm

- Không tính pose.
- Không matching.
- Không sửa result.

---

## 5. Data class đề xuất

## 5.1. `RFMapLandmark`

```python
@dataclass(frozen=True)
class RFMapLandmark:
    landmark_id: int
    position_map: np.ndarray  # shape (3,)
    frame_id: str = "map_frame"
```

---

## 5.2. `MatchedPair`

```python
@dataclass(frozen=True)
class MatchedPair:
    detection_id: int
    landmark_id: int
    point_lidar: np.ndarray  # shape (3,)
    point_map: np.ndarray    # shape (3,)
    weight: float = 1.0
```

---

## 5.3. `RobotPose`

```python
@dataclass(frozen=True)
class RobotPose:
    stamp: float
    frame_id: str
    child_frame_id: str
    x: float
    y: float
    yaw: float
    residual_rmse: float
    num_matches: int
```

---

## 5.4. `LocalizationStatus`

```python
class LocalizationStatus(Enum):
    OK = "OK"
    INSUFFICIENT_DETECTIONS = "INSUFFICIENT_DETECTIONS"
    INSUFFICIENT_MATCHES = "INSUFFICIENT_MATCHES"
    DEGENERATE_GEOMETRY = "DEGENERATE_GEOMETRY"
    HIGH_RESIDUAL = "HIGH_RESIDUAL"
    MAP_ERROR = "MAP_ERROR"
    ASSOCIATION_FAILED = "ASSOCIATION_FAILED"
    ERROR = "ERROR"
```

---

## 5.5. `LocalizationResult`

```python
@dataclass(frozen=True)
class LocalizationResult:
    stamp: float
    status: LocalizationStatus
    pose: Optional[RobotPose]
    matched_pairs: List[MatchedPair]
    residual_rmse: Optional[float]
    reason: str
    debug_info: Dict[str, Any]
```

Vì project dùng Python 3.8, không dùng:

```python
RobotPose | None
list[MatchedPair]
dict[str, Any]
```

Phải dùng:

```python
Optional[RobotPose]
List[MatchedPair]
Dict[str, Any]
```

---

## 6. Data flow chi tiết

```text
[detections.json]
      ↓
[Detection Loader]
      ↓
List[RFDetection]
      ↓
[RF Map Loader] ← data/maps/rf_map_v1.json
      ↓
List[RFMapLandmark]
      ↓
[Data Association]
      ↓
List[MatchedPair]
      ↓
[Geometry Check]
      ↓
Valid matched pairs
      ↓
[SVD Pose Solver]
      ↓
RobotPose
      ↓
[Localization Writer]
      ↓
poses.csv / poses.json / rejected_frames.csv
```

---

## 7. Quy tắc hệ tọa độ

Phase 2 phải đặt tên biến theo hệ tọa độ rõ ràng:

```text
point_lidar
point_map
center_lidar
position_map
T_map_lidar
pose_map_lidar
```

Không dùng biến mơ hồ:

```text
point
center
position
pose
T
```

Trong Phase 2 bản đầu:

```text
source = lidar_frame
target = map_frame
transform = T_map_lidar
```

---

## 8. Kiểm tra suy biến hình học

Trước khi chạy SVD, bắt buộc kiểm tra:

```text
N >= 3
spread_lidar >= min_spread
spread_map >= min_spread
không có duplicate ids
không có NaN
```

Nếu không đạt:

```python
return LocalizationResult(
    status=LocalizationStatus.DEGENERATE_GEOMETRY,
    pose=None,
    reason="Matched landmarks are geometrically degenerate.",
)
```

---

## 9. Residual check

Sau SVD, cần tính residual:

```text
e_i = ||p_map_i - (R * p_lidar_i + t)||
```

RMSE:

```text
sqrt(mean(e_i^2))
```

Nếu:

```text
residual_rmse > max_residual_rmse
```

thì reject pose:

```text
HIGH_RESIDUAL
```

---

## 10. Output chuẩn

## `poses.csv`

```csv
frame_index,stamp,status,x,y,yaw,num_matches,residual_rmse,reason
0,1716000000.0,OK,1.24,0.35,0.02,3,0.018,
1,1716000000.1,INSUFFICIENT_DETECTIONS,,,,,2,,only 2 detections
```

## `association_debug.csv`

```csv
frame_index,detection_id,landmark_id,x_lidar,y_lidar,x_map,y_map,weight
0,0,3,1.20,0.35,4.37,0.00,0.82
```

## `rejected_frames.csv`

```csv
frame_index,stamp,status,reason,num_detections,num_matches
1,1716000000.1,INSUFFICIENT_DETECTIONS,only 2 detections,2,0
```

---

## 11. Roadmap module

```text
2.1 pose.py
2.2 map_loader.py
2.3 svd_pose.py
2.4 data_association.py
2.5 geometry_check.py
2.6 localizer_pipeline.py
2.7 localization_writer.py
2.8 scripts/run_svd_localization.py
```

Không nhảy cóc sang `run_svd_localization.py` trước khi các module core có test.

---

## 12. Kết luận kiến trúc

Phase 2 phải giữ nguyên nguyên tắc:

```text
Phase 1 tạo RFDetection.
Phase 2 tạo RobotPose.
Không module nào làm cả hai việc.
```

Nếu kiến trúc này được giữ chặt, sau này có thể thay `ThresholdRFDetector` bằng neural detector mà không phải viết lại localization backend.