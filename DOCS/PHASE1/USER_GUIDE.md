# RF Threshold Localization — Hướng Dẫn Sử Dụng (User Guide)

**Phiên bản:** v1.0 | **Cập nhật:** 2026-05-30

---

## Mục Lục

1. [Tổng Quan Hệ Thống](#1-tổng-quan-hệ-thống)
2. [Yêu Cầu Cài Đặt](#2-yêu-cầu-cài-đặt)
3. [Cấu Trúc Thư Mục](#3-cấu-trúc-thư-mục)
4. [Bắt Đầu Với File Bag Mới](#4-bắt-đầu-với-file-bag-mới)
5. [Giải Thích Chi Tiết Từng Bước Pipeline](#5-giải-thích-chi-tiết-từng-bước-pipeline)
6. [Cấu Hình YAML — Giải Thích Từng Tham Số](#6-cấu-hình-yaml--giải-thích-từng-tham-số)
7. [Phân Tích Kết Quả](#7-phân-tích-kết-quả)
8. [Xử Lý Lỗi (Troubleshooting)](#8-xử-lý-lỗi-troubleshooting)
9. [Lưu Ý Về Hệ Tọa Độ](#9-lưu-ý-về-hệ-tọa-độ)
10. [Hạn Chế & Hướng Phát Triển](#10-hạn-chế--hướng-phát-triển)

---

## 1. Tổng Quan Hệ Thống

Hệ thống này **phát hiện các mốc phản xạ (Reflective Features — RF)** từ dữ liệu LiDAR 3D được thu thập trong file ROS bag. Mục tiêu là xác định vị trí trung tâm của từng mốc RF trong **hệ tọa độ LiDAR (`lidar_frame`)** — đây là bước tiền xử lý cho bài toán **định vị robot** sử dụng phép biến đổi SVD.

### Pipeline Tổng Thể

```
ROS Bag
  │
  ▼
[Frame Reader]           ← Đọc từng message từ topic LiDAR
  │
  ▼
[Preprocessing]          ← Lọc NaN, lọc khoảng cách, lọc chiều cao
  │
  ▼
[Thresholding]           ← Chọn điểm sáng (Fixed hoặc Adaptive)
  │
  ▼
[DBSCAN Clustering]      ← Gom nhóm điểm sáng thành cụm
  │
  ▼
[Cluster Validation]     ← Loại bỏ cụm không hợp lệ (kích thước, cường độ)
  │
  ▼
[Center Estimation]      ← Tính tâm RF theo trọng số cường độ
  │
  ▼
[Result Writer]          ← Xuất JSON, CSV, ảnh debug
```

### Nguyên Lý Hoạt Động

Mốc RF là các vật liệu phản xạ cao (tape phản quang, biển báo,...) được **lắp đặt có chủ đích** dọc theo hành lang. Chúng phản xạ tia laser LiDAR mạnh hơn đáng kể so với môi trường xung quanh. Thuật toán khai thác đặc tính này để phân biệt mốc RF với nền.

---

## 2. Yêu Cầu Cài Đặt

### 2.1. Phần Mềm

| Yêu Cầu | Phiên Bản |
|---------|-----------|
| Python | ≥ 3.8 |
| ROS | Noetic (hoặc bất kỳ phiên bản hỗ trợ `rosbag`) |
| pip | ≥ 21 |

### 2.2. Cài Đặt Thư Viện Python

```bash
# Clone project (nếu chưa có)
git clone <repo-url>
cd rf_threshold_localization

# Cài đặt các thư viện
pip install -r requirements.txt
```

Nội dung `requirements.txt`:

```
numpy
scipy
scikit-learn
matplotlib
pyyaml
pandas
pytest
```

### 2.3. Cài Đặt Source Package

Để có thể `import rf_threshold` từ bất kỳ đâu:

```bash
# Từ thư mục gốc dự án
pip install -e .
# hoặc thêm vào PYTHONPATH
export PYTHONPATH=$(pwd)/src:$PYTHONPATH
```

### 2.4. Kiểm Tra Môi Trường

```bash
python scripts/check_environment.py
```

Kết quả thành công sẽ in ra tất cả các import cần thiết đều khả dụng.

---

## 3. Cấu Trúc Thư Mục

```
rf_threshold_localization/
│
├── config/
│   └── threshold_v1.yaml        ← File cấu hình chính (sửa tại đây)
│
├── data/
│   ├── bags/                    ← Đặt file .bag vào đây
│   └── results/                 ← Kết quả được ghi vào đây
│
├── scripts/
│   ├── run_threshold_bag.py     ← ★ Script chạy pipeline chính
│   ├── visualize_detections.py  ← Phân tích & vẽ biểu đồ kết quả
│   ├── check_bag_info.py        ← Kiểm tra thông tin bag
│   ├── check_environment.py     ← Kiểm tra môi trường cài đặt
│   ├── run_preprocess_bag.py    ← Debug riêng bước preprocessing
│   └── run_read_bag.py          ← Debug riêng bước đọc bag
│
├── src/rf_threshold/
│   ├── core/
│   │   ├── preprocessing.py     ← Lọc điểm
│   │   ├── thresholding.py      ← Ngưỡng cường độ
│   │   ├── clustering.py        ← DBSCAN
│   │   ├── cluster_validation.py ← Kiểm tra hợp lệ
│   │   ├── center_estimation.py ← Tính tâm RF
│   │   ├── detector_pipeline.py ← Điều phối toàn bộ pipeline
│   │   └── frame.py             ← Data classes (LidarFrame, RFDetection)
│   ├── io/
│   │   ├── bag_reader.py        ← Đọc file .bag
│   │   └── result_writer.py     ← Ghi JSON/CSV
│   └── visualization/
│       └── plot_frame.py        ← Vẽ ảnh debug từng frame
│
├── tests/                       ← Unit tests (29 tests, 100% pass)
│
└── DOCS/
    ├── USER_GUIDE.md            ← File này
    ├── ARCHITECTURE.md          ← Chi tiết kiến trúc
    ├── CONTRIBUTING.md          ← Quy tắc đóng góp
    └── DEBUG_LOG.md             ← Nhật ký debug
```

---

## 4. Bắt Đầu Với File Bag Mới

Làm theo **4 bước** sau để chạy pipeline trên một file bag mới:

### Bước 1: Kiểm Tra File Bag

```bash
# Xem thông tin bag: các topic, thời lượng, số message
python scripts/check_bag_info.py --bag data/bags/TEN_BAG_MOI.bag
```

Ghi lại:
- **Tên topic** LiDAR (ví dụ: `/livox/lidar`, `/points_raw`, `/velodyne_points`)
- **Số lượng message** (= số frame sẽ xử lý)
- **Thời lượng** (giây)

### Bước 2: Tạo File Cấu Hình Mới

Sao chép config mặc định và chỉnh sửa:

```bash
cp config/threshold_v1.yaml config/my_new_bag.yaml
```

Mở `config/my_new_bag.yaml` và sửa các trường quan trọng sau:

```yaml
bag:
  path: "data/bags/TEN_BAG_MOI.bag"   # ← Đường dẫn đến file bag
  topic: "/livox/lidar"               # ← Topic LiDAR đúng với bag của bạn

preprocessing:
  height_filter:
    enabled: true
    min_z: -0.30    # ← Điều chỉnh theo độ cao LiDAR (xem Mục 6)
    max_z:  0.20    # ← Điều chỉnh theo độ cao LiDAR (xem Mục 6)

threshold:
  mode: "adaptive"  # ← Khuyến nghị dùng "adaptive" cho bag mới

output:
  output_dir: "data/results/ten_chay_moi"  # ← Đặt tên thư mục output rõ ràng
```

> **💡 Lưu ý:** Không cần thay đổi các tham số clustering và validation cho hầu hết trường hợp. Chỉ cần chỉnh `height_filter` và `bag.path`.

### Bước 3: Chạy Pipeline

```bash
# Từ thư mục gốc dự án
python scripts/run_threshold_bag.py --config config/my_new_bag.yaml
```

**CLI Overrides** — có thể ghi đè cấu hình YAML từ command line:

```bash
# Ghi đè bag path
python scripts/run_threshold_bag.py \
    --config config/threshold_v1.yaml \
    --bag data/bags/TEN_BAG_MOI.bag \
    --output data/results/ten_chay_moi
```

**Theo dõi tiến trình** — Output sẽ in ra từng frame:

```
[Frame 000000 | t=0.000]
raw=24000 | preprocessed=11203 | bright=42 | clusters=4 | valid=3 | threshold=185.2
[Frame 000001 | t=0.100]
...
```

| Trường | Ý Nghĩa |
|--------|---------|
| `raw` | Số điểm thô từ LiDAR |
| `preprocessed` | Số điểm sau khi lọc NaN + khoảng cách + chiều cao |
| `bright` | Số điểm vượt ngưỡng cường độ |
| `clusters` | Số cụm DBSCAN được tạo |
| `valid` | Số cụm được công nhận là RF ✅ |
| `threshold` | Giá trị ngưỡng cường độ đã dùng |

### Bước 4: Phân Tích Kết Quả

```bash
python scripts/visualize_detections.py \
    --result data/results/ten_chay_moi/detections.json \
    --save-plot data/results/ten_chay_moi/rf_trajectory.png
```

Script sẽ in ra bảng thống kê và lưu biểu đồ quỹ đạo.

---

## 5. Giải Thích Chi Tiết Từng Bước Pipeline

### Bước 1 — Preprocessing (Tiền Xử Lý)

**File:** `src/rf_threshold/core/preprocessing.py`

```
Điểm thô (raw) → Xóa NaN → Lọc khoảng cách → Lọc chiều cao → Điểm sạch
```

| Bộ Lọc | Mục Đích | Tham Số |
|--------|---------|---------|
| **NaN removal** | Xóa điểm có tọa độ/cường độ không hợp lệ | `remove_nan: true` |
| **Range filter** | Loại điểm quá gần (nhiễu) và quá xa (độ tin cậy thấp) | `min_range`, `max_range` |
| **Height filter** | Giữ lại chỉ điểm ở độ cao tương đương với mốc RF | `min_z`, `max_z` |

**Lọc chiều cao là quan trọng nhất** vì nó loại bỏ hàng nghìn điểm sàn/trần nhà không liên quan, giúp các bước sau hoạt động chính xác hơn.

Công thức tính `min_z` và `max_z`:
- `h_lidar` = chiều cao lắp LiDAR so với mặt đất (ví dụ: 1.2m)
- `h_RF_min` = độ cao tối thiểu của mốc RF so với mặt đất (ví dụ: 1.0m)
- `h_RF_max` = độ cao tối đa của mốc RF so với mặt đất (ví dụ: 1.3m)

```
min_z = h_RF_min - h_lidar = 1.0 - 1.2 = -0.20m
max_z = h_RF_max - h_lidar = 1.3 - 1.2 = +0.10m
```

> Thêm biên độ dự phòng ±0.05~0.10m để không bỏ sót mốc.

---

### Bước 2 — Thresholding (Lọc Ngưỡng Cường Độ)

**File:** `src/rf_threshold/core/thresholding.py`

Chọn tất cả điểm có cường độ phản xạ ≥ ngưỡng. Hỗ trợ 2 chế độ:

#### Chế độ `fixed` (Ngưỡng Cố Định)

```yaml
threshold:
  mode: "fixed"
  fixed_intensity: 140.0
```

Chọn điểm có `intensity >= 140.0`. Đơn giản, nhanh, nhưng cần biết trước đặc tính môi trường.

- **Dùng khi:** Môi trường ổn định, đã biết khoảng cường độ RF từ thực nghiệm.
- **Nhược điểm:** Quá thấp → nhiều false positive; Quá cao → bỏ sót RF.

#### Chế độ `adaptive` (Ngưỡng Thích Nghi) ⭐ Khuyến nghị

```yaml
threshold:
  mode: "adaptive"
  adaptive_percentile: 99.5
```

Tính ngưỡng = percentile 99.5 của toàn bộ cường độ trong frame. Chỉ giữ lại **0.5% điểm sáng nhất**.

- **Dùng khi:** Không biết đặc tính môi trường, chạy bag mới lần đầu.
- **Ưu điểm:** Tự điều chỉnh theo điều kiện ánh sáng, bổ sung và mật độ điểm.
- **Nhược điểm:** Ở frame trống (không có RF nào), có thể lấy nhầm điểm môi trường. Giải quyết bằng Cluster Validation (Bước 4).

---

### Bước 3 — DBSCAN Clustering (Gom Cụm)

**File:** `src/rf_threshold/core/clustering.py`

Thuật toán DBSCAN gom các điểm sáng gần nhau thành cụm dựa trên **khoảng cách không gian**.

```yaml
clustering:
  method: "dbscan"
  use_dimension: "xy"   # Chỉ dùng tọa độ X-Y (bỏ qua Z)
  eps: 0.08             # Bán kính tìm láng giềng (0.08m = 8cm)
  min_samples: 3        # Cần ít nhất 3 điểm để tạo cụm
```

**`eps`** là tham số nhạy cảm nhất:
- Quá nhỏ → một mốc RF bị tách thành nhiều cụm nhỏ
- Quá lớn → nhiều mốc RF bị gộp thành một cụm, hoặc nền bị gộp vào cụm RF

> **Khuyến nghị:** Giữ nguyên `eps=0.08` (8cm) nếu RF là tape phản quang cỡ 5–10cm. Tăng lên 0.12–0.15 nếu RF lớn hơn.

---

### Bước 4 — Cluster Validation (Kiểm Tra Hợp Lệ)

**File:** `src/rf_threshold/core/cluster_validation.py`

Loại bỏ cụm không phải RF dựa trên **kiểm tra hình học và cường độ**:

```yaml
cluster_validation:
  min_points: 3          # Cụm có < 3 điểm → bị loại (quá ít, thiếu tin cậy)
  max_points: 200        # Cụm có > 200 điểm → bị loại (có thể là cả mảng tường)
  max_extent_x: 0.30     # Kích thước theo X không vượt 30cm
  max_extent_y: 0.30     # Kích thước theo Y không vượt 30cm
  max_extent_z: 0.50     # Kích thước theo Z không vượt 50cm
  min_mean_intensity: 120.0  # Cường độ trung bình phải ≥ 120 (trên thang 0–255)
```

**Luồng kiểm tra:**

```
Cụm DBSCAN
  ↓
① Số điểm: min_points ≤ N ≤ max_points?   Không → LOẠI (too_few / too_many)
  ↓
② Kích thước: extent_x ≤ 0.30m?            Không → LOẠI (too_large_x)
③             extent_y ≤ 0.30m?            Không → LOẠI (too_large_y)
④             extent_z ≤ 0.50m?            Không → LOẠI (too_large_z)
  ↓
⑤ Cường độ: mean_intensity ≥ 120.0?        Không → LOẠI (low_intensity)
  ↓
✅ HỢP LỆ → Tiếp tục sang Bước 5
```

---

### Bước 5 — Center Estimation (Tính Tâm RF)

**File:** `src/rf_threshold/core/center_estimation.py`

Tính tọa độ tâm RF sử dụng **trọng số cường độ** (không phải centroid đơn giản):

```
center = Σ(point_i × weight_i) / Σ(weight_i)
```

Trong đó `weight_i = (intensity_i - threshold)^power`

Điểm có cường độ cao hơn đóng góp nhiều hơn vào tọa độ tâm, giúp tâm ước lượng **chính xác hơn và ít bị ảnh hưởng bởi điểm ngoại vi**.

```yaml
center_estimation:
  method: "intensity_weighted"
  intensity_weight_power: 1.0   # Luỹ thừa trọng số (1.0 = tuyến tính)
  clamp_percentile: 95.0        # Clamp cường độ tại percentile 95 trước khi tính trọng số
```

**Output:** Tọa độ `(x, y, z)` trong `lidar_frame` tính bằng mét.

---

### Bước 6 — Confidence Score (Điểm Tin Cậy)

Score được tính theo công thức có trọng số:

```
score = 0.5 × intensity_score
      + 0.3 × compactness_score
      + 0.2 × point_count_score
```

| Thành Phần | Ý Nghĩa | Cao khi... |
|-----------|---------|-----------|
| `intensity_score` | Cường độ so với max (255) | RF sáng |
| `compactness_score` | Cụm nhỏ gọn (extent ≤ 30cm) | Cụm nhỏ |
| `point_count_score` | Số điểm phong phú (≈ 15–20) | Nhiều điểm |

Score = 1.0 là hoàn hảo, score < 0.5 nghĩa là cụm có nghi vấn.

---

## 6. Cấu Hình YAML — Giải Thích Từng Tham Số

File cấu hình đầy đủ với giải thích:

```yaml
# ============================================================
# CẤU HÌNH NGUỒN DỮ LIỆU
# ============================================================
bag:
  path: "data/bags/lan4_u_.bag"     # Đường dẫn đến file .bag
  topic: "/livox/lidar"              # Topic LiDAR (xem bằng check_bag_info.py)
  message_type: "auto"               # "auto" = tự nhận dạng kiểu message

# ============================================================
# BƯỚC 1: TIỀN XỬ LÝ
# ============================================================
preprocessing:
  remove_nan: true                   # Luôn để true

  range_filter:
    enabled: true
    min_range: 0.20                  # Loại điểm < 20cm (nhiễu gần)
    max_range: 8.00                  # Loại điểm > 8m (RF thường < 5m)

  height_filter:
    enabled: true
    # QUAN TRỌNG: Điều chỉnh theo chiều cao LiDAR và RF
    # Công thức: min_z = h_RF_min - h_lidar, max_z = h_RF_max - h_lidar
    # Ví dụ: LiDAR cao 1.2m, RF ở độ cao 1.0-1.3m so với sàn
    # → min_z = 1.0 - 1.2 = -0.20, max_z = 1.3 - 1.2 = +0.10
    min_z: -0.30                     # Thêm biên ±0.10m để an toàn
    max_z:  0.20

# ============================================================
# BƯỚC 2: NGƯỠNG CƯỜNG ĐỘ
# ============================================================
threshold:
  mode: "adaptive"                   # "fixed" hoặc "adaptive"
  fixed_intensity: 140.0             # Chỉ dùng khi mode = "fixed"
  adaptive_percentile: 99.5          # Chỉ dùng khi mode = "adaptive"
                                     # 99.5 = chỉ giữ 0.5% điểm sáng nhất

# ============================================================
# BƯỚC 3: GOM CỤM DBSCAN
# ============================================================
clustering:
  method: "dbscan"
  use_dimension: "xy"                # "xy" = bỏ qua chiều cao khi gom cụm
  eps: 0.08                          # Bán kính tìm láng giềng (m)
  min_samples: 3                     # Số điểm tối thiểu để tạo cụm

# ============================================================
# BƯỚC 4: KIỂM TRA CỤM HỢP LỆ
# ============================================================
cluster_validation:
  min_points: 3                      # Cụm tối thiểu 3 điểm
  max_points: 200                    # Cụm tối đa 200 điểm
  max_extent_x: 0.30                 # Kích thước tối đa theo X (m)
  max_extent_y: 0.30                 # Kích thước tối đa theo Y (m)
  max_extent_z: 0.50                 # Kích thước tối đa theo Z (m)
  min_mean_intensity: 120.0          # Cường độ trung bình tối thiểu

# ============================================================
# BƯỚC 5: TÍNH TÂM RF
# ============================================================
center_estimation:
  method: "intensity_weighted"
  intensity_weight_power: 1.0
  clamp_percentile: 95.0

# ============================================================
# OUTPUT
# ============================================================
output:
  output_dir: "data/results/sample_run"
  save_json: true                    # Lưu detections.json
  save_csv: true                     # Lưu detections_summary.csv
  save_debug_image: true             # Lưu ảnh debug từng frame (chậm hơn)

# ============================================================
# DEBUG
# ============================================================
debug:
  enabled: true
  print_every_n_frames: 1           # In log mỗi N frame
```

---

## 7. Phân Tích Kết Quả

### 7.1. File Output

Sau khi chạy pipeline, thư mục `output_dir` sẽ chứa:

```
data/results/sample_run/
├── detections.json          ← Kết quả chi tiết từng frame (JSON)
├── detections_summary.csv   ← Tóm tắt mỗi frame (CSV)
├── config_used.yaml         ← Bản sao config đã dùng (để tái hiện)
└── debug_images/            ← Ảnh debug (nếu save_debug_image = true)
    ├── frame_000000.png
    ├── frame_000001.png
    └── ...
```

### 7.2. Cấu Trúc `detections.json`

```json
{
  "metadata": {
    "bag_path": "data/bags/lan4_u_.bag",
    "total_frames": 1925,
    "generated_at": "2026-05-30T02:00:00"
  },
  "detections": [
    {
      "frame_index": 0,
      "stamp": 0.000,
      "num_valid_detections": 3,
      "objects": [
        {
          "detection_id": 0,
          "center_lidar": [0.42, 2.15, -0.05],   ← [X, Y, Z] trong lidar_frame (m)
          "score": 0.812,                          ← Điểm tin cậy [0.0, 1.0]
          "num_points": 18,
          "mean_intensity": 198.3,
          "max_intensity": 245.0
        }
      ]
    }
  ]
}
```

### 7.3. Vẽ Biểu Đồ Quỹ Đạo

```bash
python scripts/visualize_detections.py \
    --result data/results/sample_run/detections.json \
    --save-plot data/results/sample_run/rf_trajectory.png
```

Script in ra bảng thống kê:

```
THỐNG KÊ KẾT QUẢ PHÁT HIỆN RF:
  Tổng số frame đã xử lý         : 1925
  Tổng số RF phát hiện được      : 4820
  Số RF trung bình mỗi frame     : 2.50
  Tỷ lệ frame có RF (Recall rate): 85.3%
  Số mốc RF tối đa trong 1 frame : 5
  Cường độ trung bình của RF      : 192.4 (dải [121.0, 251.0])
  Điểm tin cậy trung bình (Score) : 0.734 (dải [0.501, 0.981])
```

### 7.4. Đánh Giá Chất Lượng

| Chỉ Số | Tốt | Trung Bình | Kém |
|--------|-----|-----------|-----|
| Recall rate | > 80% | 50–80% | < 50% |
| RF trung bình/frame | ≥ 3 | 2–3 | < 2 |
| Score trung bình | > 0.70 | 0.50–0.70 | < 0.50 |
| Cường độ trung bình | > 150 | 120–150 | = 120 (ngưỡng tối thiểu) |

> **⚠️ Lưu ý:** Bài toán định vị cần **tối thiểu 3 RF** trong một frame để thực hiện phép biến đổi SVD. Frame có < 3 RF hiện tại chưa được dùng cho định vị (fallback chưa được triển khai).

---

## 8. Xử Lý Lỗi (Troubleshooting)

### ❌ Lỗi: `ModuleNotFoundError: No module named 'rf_threshold'`

**Nguyên nhân:** Python không tìm thấy package `rf_threshold`.

**Cách sửa:**
```bash
# Cách 1: Cài editable install
pip install -e .

# Cách 2: Thêm vào PYTHONPATH (tạm thời)
export PYTHONPATH=/home/minh/rf_threshold_localization/src:$PYTHONPATH
```

---

### ❌ Lỗi: `rosbag.bag.ROSBagException` hoặc không đọc được bag

**Nguyên nhân:** Topic sai, file bag bị hỏng, hoặc kiểu message không tương thích.

**Cách sửa:**
```bash
# Kiểm tra topic trong bag
python scripts/check_bag_info.py --bag data/bags/TEN_BAG.bag

# Hoặc dùng rosbag trực tiếp
rosbag info data/bags/TEN_BAG.bag
```
Đảm bảo `topic` trong YAML khớp với tên topic trong bag.

---

### ❌ Lỗi: `valid=0` mọi frame — Không phát hiện được RF nào

**Nguyên nhân và cách sửa:**

| Bước Kiểm Tra | Lệnh / Hành Động |
|--------------|-----------------|
| **1. Kiểm tra `preprocessed` có > 0 không** | Nếu = 0, `height_filter` quá nghiêm — thử `min_z=-0.5, max_z=0.5` |
| **2. Kiểm tra `bright` có > 0 không** | Nếu = 0, ngưỡng quá cao — giảm `fixed_intensity` hoặc dùng `adaptive` |
| **3. Kiểm tra `clusters` có > 0 không** | Nếu = 0, `eps` quá nhỏ — tăng lên 0.10–0.15 |
| **4. Kiểm tra `valid=0` dù có `clusters`** | Bộ lọc validation quá nghiêm — xem log `rejected_clusters` |

Xem lý do loại bỏ cluster bằng cách bật `debug.enabled: true` và quan sát log.

---

### ❌ Lỗi: Quá nhiều false positive (phát hiện vật không phải RF)

**Nguyên nhân:** Ngưỡng quá thấp hoặc môi trường có nhiều vật thể phản xạ tốt.

**Cách sửa:**
```yaml
# Tăng ngưỡng cường độ tối thiểu
cluster_validation:
  min_mean_intensity: 150.0    # Từ 120 → 150

# Giảm kích thước tối đa (mốc RF nhỏ hơn)
  max_extent_x: 0.20
  max_extent_y: 0.20

# Dùng adaptive với percentile cao hơn
threshold:
  mode: "adaptive"
  adaptive_percentile: 99.8    # Từ 99.5 → 99.8 (nghiêm hơn)
```

---

### ❌ Lỗi: Phát hiện RF nhưng tọa độ Y có vẻ sai (ví dụ Y = 6m)

**Nguyên nhân:** Đây **KHÔNG phải lỗi**. Tọa độ trong `lidar_frame` là hệ tọa độ cục bộ của LiDAR tại thời điểm frame đó. Y = 6m nghĩa là RF cách LiDAR **6 mét theo hướng Y của LiDAR** (dọc theo hành lang), không phải chiều ngang.

**Điều này bình thường** vì robot di chuyển trong hành lang, LiDAR luôn nhìn thấy RF ở phía trước/sau với khoảng cách thay đổi.

> Để chuyển về hệ tọa độ bản đồ (`map_frame`), cần triển khai Phase 1.1 — SVD Localization (chưa triển khai trong v1.0).

---

### ❌ Lỗi: `MemoryError` hoặc chậm bất thường

**Nguyên nhân:** Bag quá lớn và `save_debug_image: true`.

**Cách sửa:**
```yaml
output:
  save_debug_image: false    # Tắt lưu ảnh debug để tiết kiệm bộ nhớ và tốc độ
```

---

### ❌ Lỗi: `ImportError: cannot import name 'Any' from ...`

**Nguyên nhân:** Lỗi trong script test tự viết — thiếu import `from typing import Any`.

**Cách sửa:**
```python
# Thêm vào đầu file script
from typing import Any, Dict, List, Optional, Tuple
```

---

## 9. Lưu Ý Về Hệ Tọa Độ

Toàn bộ tọa độ trong kết quả `detections.json` được biểu diễn trong **hệ tọa độ LiDAR (`lidar_frame`)**:

```
         X (phía trước robot)
         ↑
         │
Y ←──────┼── (bên trái robot)
         │
    LiDAR ở gốc (0,0,0)
```

| Trục | Hướng | Ghi Chú |
|------|-------|---------|
| **X** | Phía trước robot | Dọc theo hướng di chuyển |
| **Y** | Bên trái robot | Ngang hành lang |
| **Z** | Lên trên | Chiều cao |

**Quan trọng:**
- LiDAR cao 1.2m so với mặt đất → Z = 0 trong `lidar_frame` tương ứng với độ cao 1.2m thực tế.
- RF ở độ cao 1.0–1.3m thực tế → `z ∈ [-0.2m, +0.1m]` trong `lidar_frame`.
- Mọi khoảng cách và tọa độ đều tính bằng **mét (m)**.

---

## 10. Hạn Chế & Hướng Phát Triển

### v1.0 — Tính Năng Hiện Có

✅ Phát hiện RF trong `lidar_frame`  
✅ Hai chế độ threshold: fixed & adaptive  
✅ DBSCAN clustering  
✅ Cluster validation đa tiêu chí  
✅ Center estimation theo trọng số cường độ  
✅ Xuất JSON + CSV  
✅ Ảnh debug từng frame  
✅ Biểu đồ quỹ đạo  
✅ 29 unit tests (100% pass)  

### Hạn Chế Hiện Tại

⚠️ **Chưa có SVD Localization** — Tọa độ RF vẫn ở `lidar_frame`, chưa chuyển về `map_frame`.  
⚠️ **Fallback < 3 RF** — Frame có < 3 RF không được xử lý cho định vị.  
⚠️ **Không có data association** — Chưa khớp RF quan sát được với bản đồ RF đã biết.  

### Hướng Phát Triển v1.1+

- [ ] **Phase 1.1:** SVD-based Localization — khớp RF với bản đồ và tính pose robot
- [ ] **Phase 1.2:** Data Association — Hungarian algorithm hoặc nearest-neighbor
- [ ] **Phase 1.3:** Kalman Filter — lọc và dự đoán pose theo thời gian
- [ ] **Fallback handler:** Xử lý frame có < 3 RF bằng pose dự đoán từ frame trước

---

*Tài liệu được tạo bởi Antigravity AI — rf_threshold_localization v1.0*
