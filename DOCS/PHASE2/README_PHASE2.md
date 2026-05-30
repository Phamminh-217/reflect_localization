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

## Phase 2.4 — Data association

### Mục tiêu

Tìm cặp tương ứng giữa RF quan sát trong `lidar_frame` và RF trong `map_frame`.

### Files

```text
src/rf_threshold/localization/data_association.py
tests/test_data_association.py
```

### Chiến lược bản đầu

Vì chưa có pose ban đầu ổn định, không dùng nearest-neighbor trực tiếp giữa `lidar_frame` và `map_frame`.

Ưu tiên:

```text
Triplet / pairwise-distance matching
hoặc enumerate correspondence với residual nhỏ nhất
```

### Definition of Done

- Không match trực tiếp nếu chưa có initial pose.
- Có thể tạo matched pairs >= 3 khi dữ liệu hợp lệ.
- Reject matching có residual cao.
- Unit test pass.

---

## Phase 2.5 — Localizer pipeline

### Mục tiêu

Ghép các module thành pipeline định vị hoàn chỉnh.

### Files

```text
src/rf_threshold/localization/localizer_pipeline.py
tests/test_localizer_pipeline.py
```

### Pipeline

```text
RFDetection list
  ↓
Filter detections
  ↓
Associate with RF map
  ↓
Check geometry
  ↓
Run SVD
  ↓
Return LocalizationResult
```

### Definition of Done

- Frame có đủ RF → trả pose OK.
- Frame thiếu RF → trả status hợp lệ, không crash.
- Matching sai → trả status hợp lệ, không crash.
- Unit test pass.

---

## Phase 2.6 — Output writer and evaluation

### Mục tiêu

Xuất pose và log localization.

### Files

```text
src/rf_threshold/localization/localization_writer.py
scripts/run_svd_localization.py
scripts/evaluate_localization.py
```

### Output

```text
poses.csv
poses.json
localization_summary.csv
rejected_frames.csv
association_debug.csv
```

### Definition of Done

- Chạy được từ detections.json.
- Xuất được pose cho frame hợp lệ.
- Xuất được rejected reason cho frame không hợp lệ.
- Không crash khi nhiều frame thiếu RF.

---

## 9. Lệnh chạy dự kiến

Sau khi Phase 2 hoàn thành:

```bash
python3 scripts/run_svd_localization.py \
  --detections data/results/sample_run/detections.json \
  --map data/maps/rf_map_v1.json \
  --output data/results/sample_run/localization
```

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