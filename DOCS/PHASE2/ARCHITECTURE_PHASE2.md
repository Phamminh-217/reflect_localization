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

Khảo sát và đánh giá chất lượng phân bố hình học của các cặp điểm đã so khớp (`MatchedPair`) trước khi chạy SVD cuối cùng. 
* Module này chỉ thực hiện tính toán hình học thuần túy.
* **NGHIÊM CẤM:** Không được chạy SVD, không được tự ý so khớp (matching), không được đọc file map hay file detections trực tiếp trong module này.

### Cấu trúc dữ liệu thiết kế

```python
@dataclass(frozen=True)
class GeometryCheckResult:
    is_valid: bool                            # Khớp hình học thành công (được tiếp tục chạy SVD hay không)
    is_degenerate: bool                       # Có bị suy biến hình học (near-collinear hoặc trùng lặp) hay không
    warning: Optional[str]                    # Chuỗi cảnh báo (ví dụ "NEAR_COLLINEAR")
    reason: str                               # Lý do chi tiết của trạng thái
    num_pairs: int                            # Số lượng cặp điểm khảo sát
    condition_number_lidar: Optional[float]   # Condition number tính trên các điểm LiDAR
    condition_number_map: Optional[float]     # Condition number tính trên các điểm Map
    spread_lidar: float                       # Độ phân tán không gian (spatial spread) của LiDAR
    spread_map: float                         # Độ phân tán không gian (spatial spread) của Map
    debug_info: Dict[str, Any]                # Thông tin chẩn đoán nâng cao
```

> **⚠️ Lưu ý tương thích Python 3.8:**
> Bắt buộc sử dụng các định dạng kiểu dữ liệu chuẩn: `Optional[str]`, `Optional[float]`, `Dict[str, Any]`. Cấm sử dụng các cú pháp dạng `str | None` hay `dict[str, Any]`.

### Hàm chính

```python
def check_geometry_validity(
    matched_pairs: List[MatchedPair],
    cfg: Dict[str, Any],
) -> GeometryCheckResult:
    """Đánh giá tính hợp lệ phân bố hình học của các cặp điểm đã khớp."""
```

### Các trường hợp phải từ chối cứng (Hard Rejection Cases)
Nếu vi phạm một trong các điều kiện sau, hệ thống trả về `is_valid = False` lập tức:
* Số lượng điểm khớp không đạt tối thiểu (`len(matched_pairs) < min_matches`).
* Có hiện tượng trùng lặp mã nhận diện mốc LiDAR (`detection_id` bị lặp).
* Có hiện tượng trùng lặp mã landmark bản đồ (`landmark_id` bị lặp).
* Tọa độ các điểm `point_lidar` hoặc `point_map` chứa giá trị không xác định (`NaN` hoặc `Inf`).
* Độ phân tán không gian quá nhỏ (`spread_lidar < min_spread` hoặc `spread_map < min_spread`), cho thấy các điểm gần như trùng khít lên nhau và không tạo ra hệ khung hình học ổn định.

### Các trường hợp chỉ Cảnh báo (Warning-only Cases)
Ngược lại với phép từ chối cứng, các trường hợp sau đây **mặc định không được phép reject**:
* Các điểm nằm gần thẳng hàng (Near-collinear) nhưng độ phân tán không gian vẫn đủ lớn (`spread >= min_spread`).
* Chỉ số Condition number vượt quá ngưỡng cho phép (`condition_number > max_condition_number`) nhưng tùy chọn `hard_reject` trong cấu hình được đặt là `false`.

Khi đó, hàm sẽ trả về kết quả:
```text
is_valid = True
is_degenerate = True
warning = "NEAR_COLLINEAR"
reason = "Near-collinear geometry detected, continuing with warning."
```
Pipeline định vị vẫn tiếp tục chạy phép ước lượng SVD bình thường, và chất lượng của Pose robot cuối cùng sẽ được quyết định thông qua kiểm chứng sai số dư (residual RMSE check) sau đó.

### Khi nào mới kích hoạt từ chối cứng cho Condition Number?
Hệ thống chỉ hard reject các trường hợp gần thẳng hàng khi cấu hình YAML chỉ định rõ ràng:
```yaml
geometry_check:
  condition_number:
    hard_reject: true
```
Khi đó nếu condition number lớn, trả về `is_valid = False` và lý do `"Condition number exceeds threshold."`.

### Lý do thiết kế
Trong môi trường thực tế như hành lang (corridor), các mốc phản quang RF thường được bố trí thẳng hàng dọc hai bên tường hành lang. Nếu geometry check mặc định loại bỏ toàn bộ các bộ ba gần thẳng hàng, robot sẽ không thể định vị được trong hầu hết thời gian di chuyển. Do đó, hiện tượng near-collinear chỉ được coi là một **cảnh báo hình học**, không phải lỗi chết mặc định.


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

Ghi toàn bộ kết quả localization và mọi bằng chứng debug (debug evidence) ra file một cách đầy đủ và đồng bộ.

### Output bắt buộc

```text
poses.csv
poses.json
rejected_frames.csv
localization_summary.csv
association_debug.csv
svd_debug.csv
geometry_debug.csv
frame_debug.csv
```

### Không được làm

* Không chạy data association hay bất kỳ thuật toán so khớp nào.
* Không giải SVD hay ước lượng pose.
* Không thay đổi hoặc sửa đổi nội dung của `LocalizationResult`.
* Không đọc trực tiếp ROS bag hay tệp detections thô.
* Không tự ý filter hay biến đổi detections.

---

## 4.8. `run_svd_localization.py` (scripts/)

### Trách nhiệm

CLI runner để thực hiện chạy ước lượng định vị robot offline từ các tệp dữ liệu đã kết xuất của Phase 1.

### Quy trình Pipeline của Runner

1. **Tải dữ liệu đầu vào (Load Inputs)**: Đọc tệp tin kết quả phát hiện `detections.json` từ Phase 1 và bản đồ toàn cục `rf_map_v1.json`.
2. **Tải cấu hình (Load Config)**: Đọc tệp cấu hình hệ thống `threshold_v1.yaml`.
3. **Khởi tạo định vị (Initialize RFLocalizer)**: Khởi tạo thực thể `RFLocalizer` với bản đồ và các tham số tương ứng.
4. **Xử lý tuần tự theo khung hình (Frame-by-Frame Execution)**: Duyệt qua từng khung hình dữ liệu quan sát:
   * Gọi hàm `localizer.localize()` để thực hiện định vị.
   * Thu thập toàn bộ các đối tượng kết quả `LocalizationResult`.
5. **Đồng bộ hóa ghi tệp (Write Outputs)**: Chuyển toàn bộ danh sách kết quả cho `LocalizationWriter` để ghi tất cả các tệp CSV/JSON và tài liệu debug liên quan.

### Không được làm

* Không tự viết lại SVD hay logic giải toán học phẳng.
* Không tự viết lại thuật toán so khớp bộ ba (data association).
* Không tự kiểm tra tính suy biến hình học (geometry check) bằng logic riêng.
* Không được phép gọi hay tích hợp trực tiếp bộ phát hiện phân ngưỡng (threshold detector) của Phase 1.
* Không đọc trực tiếp ROS bag.

---

## 4.9. `plot_localization_debug.py` (scripts/)

### Trách nhiệm

Script cung cấp công cụ trực quan hóa (visualize) đồ họa tĩnh hoặc động cho một khung hình cụ thể hoặc chuỗi khung hình để hỗ trợ kỹ sư debug nhanh chóng.

### Trực quan hóa bắt buộc (Visualization Elements)

* Hiển thị toàn bộ tọa độ các cột mốc phản xạ RF toàn cục (`RF Map landmarks`) dưới dạng các điểm mốc tĩnh.
* Vẽ các điểm quan sát được phát hiện (`detections`) sau khi đã áp dụng phép biến đổi không gian (transform) sang hệ tọa độ bản đồ `map_frame`.
* Vẽ các đường liên kết nối (matched lines) từ tâm mốc quan sát sang mốc bản đồ tương ứng để kiểm tra tính chính xác của thuật toán so khớp.
* Vẽ vector mũi tên biểu diễn tư thế của robot (Robot pose arrow) bao gồm hướng xoay Yaw và vị trí tịnh tiến $x, y$.
* Trực quan hóa sai số dư (residual vectors) bằng cách nối điểm đích thực tế với điểm khớp bản đồ.
* Hiển thị trực quan thông điệp lý do từ chối định vị (`Rejected reason`) nếu khung hình bị lỗi.

### Mục tiêu chẩn đoán nhanh

Giúp các kỹ sư nhanh chóng khoanh vùng và phát hiện các lỗi thực nghiệm:
* So khớp sai ID cột mốc (nhảy ID).
* Trục tọa độ $x/y$ bị đảo ngược.
* Hướng xoay Yaw bị ngược chiều ($180^\circ$).
* Lệch gốc tọa độ map (map origin) toàn cục.
* Pose robot bị dịch chuyển lệch cố định do thiếu bù trừ extrinsic giữa LiDAR và robot base.

---

## 4.10. Luồng dữ liệu chi tiết của Phase 2.6 (Data Flow)

```text
detections.json
       ↓
[DetectionFrameLoader]
       ↓
List[RFDetection] (per frame)
       ↓
[RFLocalizer.localize()]
       ↓
LocalizationResult (per frame)
       ↓
[LocalizationWriter]
       ↓
poses.csv / rejected_frames.csv / debug files
```

---

## 4.11. Yêu cầu tính truy vết Debug (Debug Traceability)

Quy tắc tối cao của Phase 2.6 là: **"Mỗi robot pose được xuất ra phải truy vết ngược được nguồn gốc bằng chứng hình thành."**

Mỗi bản ghi ghi nhận trong `poses.csv` bắt buộc phải có khả năng liên kết ánh xạ 1-1 với các dòng dữ liệu trong:
* `frame_debug.csv` (thông số tổng quan khung hình).
* `association_debug.csv` (vết so khớp landmark).
* `geometry_debug.csv` (vết khảo sát hình học).
* `svd_debug.csv` (vết giải ma trận và tính toán residual).

Sử dụng khóa liên kết duy nhất là:
* `frame_index`
* `stamp` (thời gian timestamp hệ thống).

Nghiêm cấm tạo ra hay ghi nhận bất kỳ một Pose robot thành công nào mà không có đầy đủ bằng chứng debug đi kèm trong các tệp lưu vết tương ứng.

---


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