# README_PHASE2.md

# Phase 2 — SVD-Based RF Localization

## 1. Mục tiêu Phase 2

Phase 2 phát triển module **định vị robot dựa trên các mốc phản xạ RF** đã được phát hiện ở Phase 1.

Ở Phase 1, hệ thống đã hoàn thành pipeline:

```text
ROS Bag
  ↓
LiDAR Frame
  ↓
Threshold-based RF Detection
  ↓
RFDetection.center_lidar
```

Sang Phase 2, hệ thống sử dụng các tâm RF đã phát hiện trong hệ `lidar_frame`, so khớp với bản đồ RF trong `map_frame`, sau đó dùng SVD để ước lượng pose của robot.

Pipeline Phase 2:

```text
RFDetection List
  ↓
RF Map Loader
  ↓
Data Association
  ↓
Geometric Validity Check
  ↓
SVD Pose Estimation
  ↓
Robot Pose Output
```

Mục tiêu cuối cùng:

```text
Từ các RF phát hiện được trong LiDAR frame
→ ước lượng được vị trí và hướng của robot trong map frame.
```

---

## 2. Vai trò của Phase 2 trong toàn hệ thống

Toàn bộ hệ thống định vị robot bằng phân ngưỡng gồm:

```text
Phase 1: RF Detection
  Input : ROS bag / LiDAR point cloud
  Output: RFDetection.center_lidar

Phase 2: RF Localization
  Input : RFDetection.center_lidar + RF map
  Output: Robot pose trong map_frame
```

Nguyên tắc quan trọng:

```text
Phase 1 chỉ phát hiện RF.
Phase 2 chỉ định vị từ RF.
Không trộn detection và localization.
```

---

## 3. Cấu hình chuẩn đã chốt

Từ Phase 2 trở đi, cấu hình `height_filter` dùng chung cho toàn bộ hệ thống được chốt là:

```yaml
preprocessing:
  height_filter:
    enabled: true
    min_z: 0.05
    max_z: 0.30
```

Quy tắc:

- Không tự ý thay đổi `min_z` và `max_z` trong các config chạy chính.
- Nếu cần thử nghiệm giá trị khác, phải tạo file config riêng dạng `threshold_experiment_*.yaml`.
- Không sửa trực tiếp config chuẩn nếu chưa có lý do thực nghiệm rõ ràng.

---

## 4. Input của Phase 2

Phase 2 nhận input từ Phase 1 dưới dạng:

```python
valid_detections: List[RFDetection]
```

Mỗi `RFDetection` có trường quan trọng nhất:

```python
center_lidar: np.ndarray  # shape (3,)
```

Ý nghĩa:

```text
center_lidar = [x, y, z]
```

trong hệ tọa độ `lidar_frame`.

Ví dụ:

```python
RFDetection(
    detection_id=0,
    stamp=1716000000.0,
    frame_id="livox_frame",
    center_lidar=np.array([1.20, 0.35, 0.12]),
    score=0.82,
    num_points=15,
    mean_intensity=195.0,
    max_intensity=245.0,
    bbox_min=np.array([1.16, 0.31, 0.10]),
    bbox_max=np.array([1.24, 0.39, 0.14]),
    cluster_id=0,
)
```

---

## 5. RF Map

Phase 2 cần một bản đồ RF đã biết trước.

File đề xuất:

```text
data/maps/rf_map_v1.json
```

Format đề xuất:

```json
{
  "map_name": "corridor_rf_map_v1",
  "frame_id": "map_frame",
  "unit": "meter",
  "landmarks": [
    {
      "id": 0,
      "position_map": [0.0, 0.0, 0.0]
    },
    {
      "id": 1,
      "position_map": [1.24, 0.0, 0.0]
    },
    {
      "id": 2,
      "position_map": [3.14, 0.0, 0.0]
    }
  ]
}
```

Ở Phase 2 bản đầu, ưu tiên dùng bài toán 2D:

```text
p_lidar = [x_lidar, y_lidar]
p_map   = [x_map, y_map]
```

Tức là bỏ qua `z` khi ước lượng pose phẳng của robot.

---

## 6. Output của Phase 2

Phase 2 tạo ra pose robot:

```python
RobotPose(
    stamp=...,
    frame_id="map_frame",
    child_frame_id="lidar_frame",
    x=...,
    y=...,
    yaw=...,
    num_matched=...,
    residual_rmse=...,
    status="OK"
)
```

Output file đề xuất:

```text
data/results/<run_name>/
├── poses.csv
├── poses.json
├── localization_summary.csv
├── rejected_frames.csv
└── association_debug.csv
```

---

## 7. Điều kiện tối thiểu để chạy SVD

Một frame chỉ được định vị nếu thỏa:

```text
Số RF detection hợp lệ >= 3
Số cặp RF matching hợp lệ >= 3
Tập điểm không bị suy biến hình học nghiêm trọng
SVD residual nhỏ hơn ngưỡng cho phép
```

Nếu không thỏa, không được cố chạy SVD.

Trạng thái fallback:

```text
INSUFFICIENT_DETECTIONS
INSUFFICIENT_MATCHES
DEGENERATE_GEOMETRY
HIGH_RESIDUAL
NO_MAP
ERROR
```

---

## 8. Roadmap Phase 2

## Phase 2.1 — Localization data structures

### Mục tiêu

Tạo các data class chuẩn cho localization.

### Files

```text
src/rf_threshold/localization/pose.py
tests/test_pose.py
```

### Cần có

```text
RFMapLandmark
MatchedPair
RobotPose
LocalizationResult
LocalizationStatus
```

### Definition of Done

- Tạo được object pose hợp lệ.
- Reject pose có NaN.
- Accept ID bằng 0.
- Unit test pass.

---

## Phase 2.2 — RF map loader

### Mục tiêu

Đọc bản đồ RF từ JSON.

### Files

```text
src/rf_threshold/localization/map_loader.py
tests/test_map_loader.py
data/maps/rf_map_v1.json
```

### Input

```text
data/maps/rf_map_v1.json
```

### Output

```python
List[RFMapLandmark]
```

### Definition of Done

- Đọc được map JSON hợp lệ.
- Reject map thiếu `landmarks`.
- Reject landmark thiếu `id`.
- Reject `position_map` sai shape.
- Unit test pass.

---

## Phase 2.3 — 2D SVD pose solver

### Mục tiêu

Từ các cặp điểm tương ứng `(p_lidar, p_map)`, tính biến đổi SE(2):

```text
p_map ≈ R * p_lidar + t
```

### Files

```text
src/rf_threshold/localization/svd_pose.py
tests/test_svd_pose.py
```

### Output

```text
R_2d
t_2d
yaw
residual_rmse
```

### Definition of Done

- Khôi phục đúng transform giả lập.
- Reject khi số cặp < 3.
- Reject khi điểm suy biến nghiêm trọng.
- Unit test pass.

---

## Phase 2.4 — Triplet Distance-Constrained Data Association

### Mục tiêu

Tìm cặp tương ứng (correspondence) giữa RF quan sát trong `lidar_frame` và RF landmarks trong `map_frame` để tạo ra `List[MatchedPair]` làm đầu vào cho SVD pose solver.

### Files

```text
src/rf_threshold/localization/data_association.py
tests/test_data_association.py
```

### Tại sao không dùng Brute-Force?

Nếu dùng brute-force duyệt qua toàn bộ hoán vị kiểu $P(K, M)$ với `M = 5` detections và `K = 20` landmarks:
$$P(20, 5) = 1.860.480 \text{ candidates}$$
Số lượng phép thử quá lớn sẽ gây quá tải CPU và không thể đáp ứng tần số chạy thời gian thực (real-time).

### Tại sao không dùng Nearest-Neighbor trực tiếp?

**NGHIÊM CẤM:** So sánh trực tiếp `center_lidar` với `position_map` bằng khoảng cách Euclidean thông thường khi chưa có Pose dự đoán hoặc Pose ban đầu.
* Lý do: `center_lidar` nằm trong `lidar_frame`, `position_map` nằm trong `map_frame`. Hai tập điểm đang ở hai hệ tọa độ khác nhau hoàn toàn.
* Nearest-neighbor chỉ được phép sử dụng làm bước xác thực sau khi đã có Pose ứng viên (Candidate Pose).

### Chiến lược Triplet Distance-Constrained Matching

Sử dụng đặc tính bất biến khoảng cách: Khoảng cách giữa các mốc RF là không đổi dưới phép biến đổi cứng (Rigid Transformation).

#### 1. Ràng buộc sai số thích nghi (Adaptive Pairwise-Distance Tolerance)
Thay vì sử dụng ngưỡng cố định (ví dụ `0.05m`), ta áp dụng công thức tính sai số thích nghi theo khoảng cách giữa các điểm mốc:
$$\epsilon(d) = \min(\text{max\_abs}, \max(\text{min\_abs}, \text{relative\_ratio} \times d))$$
Trong đó:
* `d`: Khoảng cách thực tế giữa hai landmark trong map.
* `min_abs`: Ngưỡng sai số tối thiểu (khi điểm mốc ở rất gần).
* `relative_ratio`: Hệ số tăng dần theo khoảng cách.
* `max_abs`: Ngưỡng sai số tối đa (khi điểm mốc ở xa).

*Lý do:* Điểm mốc càng ở xa LiDAR thì cụm điểm nhận diện được càng thưa, dẫn đến sai số ước lượng tâm mốc càng lớn. Mốc ở gần có độ chính xác cao hơn nên cần siết ngưỡng chặt hơn.

#### 2. Pipeline thực thi
```text
List[RFDetection] + List[RFMapLandmark]
        ↓
Generate observed triplets (Tạo bộ ba quan sát)
        ↓
Compare with precomputed map triplets (So khớp độ dài cạnh với map)
        ↓
Filter candidate triplets using adaptive pairwise-distance tolerance
        ↓
Try 3! permutations for each triplet candidate (Thử 6 hoán vị cho mỗi bộ ba)
        ↓
Run SVD to create candidate pose (Tính Pose ứng viên)
        ↓
Transform all detections to map_frame (Chuyển đổi toàn bộ detections)
        ↓
Nearest-neighbor verification (Xác thực số lượng Inliers)
        ↓
Score candidates by inlier count + residual RMSE
        ↓
Return best AssociationResult
```

#### 3. Cấu hình YAML tiêu chuẩn
```yaml
data_association:
  method: "triplet_distance"

  min_detections: 3
  min_matches: 3

  triplet_distance_tolerance:
    mode: "adaptive"
    min_abs: 0.08
    relative_ratio: 0.03
    max_abs: 0.20

  nearest_neighbor_gate:
    mode: "adaptive"
    min_abs: 0.10
    relative_ratio: 0.03
    max_abs: 0.25

  max_candidate_rmse: 0.08
  max_candidate_residual: 0.18

  max_candidates: 300
  use_detection_score_weight: true

  reject_duplicate_landmarks: true
  reject_duplicate_detections: true
```

### Tiêu chí nghiệm thu (Definition of Done)

* Không dùng brute-force $P(K, M)$ mù quáng.
* Không sử dụng nearest-neighbor trực tiếp khi chưa có Pose dự đoán/pose ban đầu.
* Sử dụng bộ lọc khoảng cách bộ ba (triplet pairwise-distance constraint) kết hợp với sai số thích nghi (adaptive tolerance).
* Chạy SVD pose solver để kiểm chứng sai số dư (residual RMSE) của các điểm còn lại.
* Trả về kết quả khớp tối ưu dựa trên số lượng inliers lớn nhất và RMSE nhỏ nhất.
* Bộ kiểm thử `tests/test_data_association.py` phải bao phủ đầy đủ 14 test cases bắt buộc và PASS 100%.


---

## Phase 2.5 — Localizer Pipeline & Geometry Check

### Mục tiêu

Phase 2.5 tích hợp các module đã hoàn thành gồm RF map loader, data association và SVD pose solver để tạo ra pipeline định vị hoàn chỉnh.

### Files

```text
src/rf_threshold/localization/localizer_pipeline.py
tests/test_localizer_pipeline.py
```

### Pipeline

```text
RFDetection List
  ↓
Detection score filtering
  ↓
Data Association
  ↓
Geometry Check
  ↓
Final SVD Pose Estimation
  ↓
Residual Validation
  ↓
RobotPose / LocalizationResult
```

### Nguyên tắc quan trọng về near-collinear geometry

Do các mốc RF trong hành lang có thể được bố trí gần thẳng hàng dọc theo hành lang, Phase 2.5 không được mặc định loại bỏ mọi trường hợp có condition number lớn.

Quy tắc mặc định:
```text
Near-collinear geometry is a warning, not a hard rejection.
```

**Các trường hợp phải reject cứng:**
* `matched_pairs < 3`
* Có duplicate `detection_id`
* Có duplicate `landmark_id`
* `point_lidar` hoặc `point_map` chứa NaN/Inf
* Spatial spread quá nhỏ (các điểm thực tế gần trùng nhau)

**Các trường hợp chỉ cảnh báo (Warning-only):**
* Condition number lớn
* Các điểm khớp gần thẳng hàng nhưng vẫn có spatial spread đủ lớn (`spread >= min_spread`)

*Cơ chế:* Nếu geometry gần thẳng hàng nhưng residual sau SVD nhỏ và thỏa mãn, hệ thống vẫn có thể chấp nhận pose `OK`. Cảnh báo hình học được lưu giữ nhưng không chặn đứng luồng thực thi của robot.

### Cấu hình đề xuất (Config)

```yaml
geometry_check:
  min_matches: 3
  min_spread: 0.30

  condition_number:
    enabled: true
    max_condition_number: 50.0
    hard_reject: false
```

*Ý nghĩa:* `hard_reject: false` là cấu hình mặc định vì môi trường hành lang có thể tạo ra các tập RF gần thẳng hàng. Nếu sau này RF phân bố 2D tốt hơn hoặc cần kiểm tra nghiêm ngặt hơn, người dùng có thể đặt `hard_reject: true`, nhưng đây không phải là mặc định cho hệ thống hiện tại.

### Tiêu chí nghiệm thu (Definition of Done)

* Tích hợp thành công các module con thành pipeline định vị hoàn chỉnh.
* Thực hiện đúng kiểm tra hình học dynamic warning cho các trường hợp near-collinear.
* Không tự ý crash khi gặp lỗi dữ liệu hoặc thiếu điểm, trả về status thích hợp.
* Bộ unit test `tests/test_localizer_pipeline.py` và `tests/test_geometry_check.py` phải pass 100% với các kịch bản kiểm thử quy định.


## Phase 2.6 — Localization I/O, CLI Runner & Debuggable End-to-End Integration

### Mục tiêu

Phase 2.6 tích hợp toàn bộ localization backend đã hoàn thành để chạy trên output thật của Phase 1.

**Input:**
```text
data/results/<run_name>/detections.json
data/maps/rf_map_v1.json
```

**Output:**
```text
data/results/<run_name>/localization/
├── poses.csv
├── poses.json
├── rejected_frames.csv
├── localization_summary.csv
├── association_debug.csv
├── svd_debug.csv
├── geometry_debug.csv
└── frame_debug.csv
```

### Nguyên tắc quan trọng

Quy tắc trung tâm của Phase 2.6:
```text
Không chỉ xuất pose. Phải xuất được bằng chứng hình thành pose.
```
Nghĩa là mỗi pose phải truy vết ngược được nguồn gốc hình thành của nó:
```text
pose ← final SVD result ← matched pairs ← association result ← detections ← RF map
```
Nếu robot định vị sai hoặc bị mất dấu, người dùng phải biết chính xác lỗi ở bước nào (do phát hiện thiếu, so khớp nhảy ID hay do suy biến hình học), không phải đoán mò.

### Pipeline Phase 2.6

```text
 detections.json
        ↓
 Detection Loader
        ↓
   RFLocalizer
        ↓
Fallback Manager
        ↓
Localization Writer
        ↓
poses.csv / debug files
```

### Files cần triển khai

```text
src/rf_threshold/localization/detection_loader.py
src/rf_threshold/localization/fallback_manager.py
src/rf_threshold/localization/localization_writer.py
scripts/run_svd_localization.py
scripts/plot_localization_debug.py
tests/test_detection_loader.py
tests/test_fallback_manager.py
tests/test_localization_writer.py
```

### Tiêu chí nghiệm thu (Definition of Done)

Phase 2.6 chỉ được nghiệm thu khi thỏa mãn cả 15 tiêu chí sau:
1. Đọc được `detections.json` từ Phase 1 thành danh sách `RFDetection`.
2. Đọc được bản đồ landmarks RF từ JSON.
3. Chạy `RFLocalizer` theo từng frame tuần tự.
4. Nếu frame OK, xuất pose OK (status `OK`, `is_fallback` = false).
5. Nếu frame fail, ghi `rejected_frames.csv`.
6. Nếu fallback enabled và hợp lệ, xuất pose fallback (status `FALLBACK_LAST_VALID_POSE`, `is_fallback` = true).
7. Nếu fallback disabled hoặc vượt giới hạn liên tiếp, không ghi nhầm thành OK.
8. Xuất đầy đủ `poses.csv` và `poses.json` theo đúng cấu trúc cột mới.
9. Xuất đầy đủ các tệp debug trung gian: `association_debug.csv`, `svd_debug.csv`, `geometry_debug.csv`, `frame_debug.csv`.
10. Xuất đầy đủ tệp summary `localization_summary.csv` theo dạng Key-Value và không gộp fallback vào OK.
11. Đảm bảo tính truy vết đồng bộ (traceability): Mỗi dòng trong `poses.csv` phải ánh xạ 1-1 với các tệp debug thông qua `frame_index` và `stamp`.
12. Có khả năng chọn bất kỳ một frame nào và truy ngược toàn bộ nguồn gốc pose của nó: `detections` $\rightarrow$ `matched pairs` $\rightarrow$ `SVD result` $\rightarrow$ `final pose`.
13. Có visualization kiểm tra và trực quan hóa ít nhất một frame OK, một frame fallback và một frame fail.
14. Không crash khi gặp nhiều frame định vị lỗi liên tiếp.
15. Hoàn thiện đầy đủ unit tests cho cả detection loader, fallback manager và localization writer.

### Cảnh báo thực nghiệm

Unit test của các module con pass không đồng nghĩa với việc pose thực nghiệm trên xe thật sẽ đúng. Phase 2.6 có fallback phải được thiết kế để dễ dàng debug các lỗi vật lý phức tạp như:
* SVD đúng nhưng pose robot bị sai lệch cố định.
* So khớp đúng trong test nhưng nhảy ID trên bản đồ thật.
* Bản đồ RF map bị lệch tọa độ gốc (origin) hoặc bị đảo trục $x/y$.
* Hướng xoay Yaw bị ngược ($180^\circ$).
* Biến đổi $T_{\text{map\_lidar}}$ bị hiểu nhầm thành chiều ngược lại $T_{\text{lidar\_map}}$.
* Thiếu bù trừ hệ trục tọa độ extrinsic $T_{\text{base\_link\_lidar}}$ khiến pose LiDAR lệch so với tâm robot.
* Trôi pose (drift) do lạm dụng dùng fallback vượt ngưỡng tối đa.

## Phase 2.7 — Real Bag End-to-End Validation

### Mục tiêu
Kết nối Phase 1 (RF detection) và Phase 2 (localization) thành một pipeline chạy trực tiếp trên file ROS bag thực tế từ robot, đồng thời chẩn đoán chất lượng định vị và chẩn đoán các lỗi vật lý.

### Files
- `src/rf_threshold/localization/pose_evaluator.py`
- `tests/test_pose_evaluator.py`
- `scripts/run_phase1_to_phase2.py`
- `scripts/generate_validation_report.py`
- `config/threshold_field_v1.yaml`
- `DOCS/PHASE2/FIELD_VALIDATION_LOG.md`

### Definition of Done
1. `PoseEvaluator` phân tích chính xác `poses.csv` và đưa ra cảnh báo drift/fallback.
2. Script `run_phase1_to_phase2.py` chạy tự động cả 2 Phase và tạo báo cáo thực nghiệm.
3. Có file cấu hình chuẩn `threshold_field_v1.yaml` cho môi trường thực tế.
4. Ghi nhận nhật ký trong `FIELD_VALIDATION_LOG.md`.

---

## 10. Tiêu chí nghiệm thu Phase 2

Phase 2 được xem là hoàn thành khi:

- [ ] Đọc được RF map JSON.
- [ ] Đọc được detections từ Phase 1.
- [ ] Match được RF observation với RF map.
- [ ] Chạy được SVD 2D.
- [ ] Trả ra pose `[x, y, yaw]`.
- [ ] Có fallback khi `< 3 RF`.
- [ ] Có residual check.
- [ ] Có degeneracy check.
- [ ] Xuất được `poses.csv`.
- [ ] Xuất được `rejected_frames.csv`.
- [ ] Unit test pass.
- [ ] Có ít nhất một bag thực nghiệm chạy được end-to-end.

---

## 11. Kết luận

Phase 2 không chỉ là viết công thức SVD. Phần quan trọng hơn là:

```text
Data association đúng
Geometric validity check chặt
Fallback rõ ràng
Output dễ debug
```

SVD chỉ là solver cuối cùng. Nếu matching sai hoặc RF input thiếu, pose sẽ sai dù solver đúng.