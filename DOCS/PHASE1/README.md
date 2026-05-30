# RF Threshold Localization v1.0

## 1. Giới thiệu dự án

Dự án này xây dựng một hệ thống phát hiện mốc phản xạ `RF - Reflective Feature / Reflective Landmark` từ dữ liệu LiDAR 3D của **Livox Mid-360**, sử dụng phương pháp **phân ngưỡng cường độ phản xạ**. Đây là phiên bản baseline `v1.0`, được thiết kế để phục vụ hai mục tiêu chính:

1. Xây dựng một pipeline phân ngưỡng rõ ràng, dễ kiểm tra và dễ debug.
2. Tạo baseline đủ chặt chẽ để so sánh với phương pháp mạng nơ-ron trong bài báo khoa học.

Ở phiên bản `v1.0`, dự án tập trung vào bài toán:

```text
ROS bag
  ↓
Đọc dữ liệu LiDAR
  ↓
Tiền xử lý point cloud
  ↓
Lọc điểm có intensity cao
  ↓
Gom cụm điểm phản xạ
  ↓
Kiểm tra hình học cụm
  ↓
Ước lượng tâm RF
  ↓
Xuất kết quả detection và dữ liệu debug
```

Phiên bản này **chưa tập trung vào mạng nơ-ron** và **chưa bắt buộc tích hợp định vị SVD hoàn chỉnh**. Mục tiêu là hoàn thiện module phân ngưỡng trước, làm nền cho các phiên bản sau.

---

## 2. Công nghệ sử dụng

Dự án sử dụng các công nghệ chính:

| Thành phần      | Công nghệ              |
| --------------- | ---------------------- |
| Hệ robot        | ROS1                   |
| Ngôn ngữ chính  | Python                 |
| Cảm biến        | Livox Mid-360 3D LiDAR |
| Dữ liệu đầu vào | ROS bag                |
| Xử lý dữ liệu   | NumPy, scikit-learn    |
| Trực quan hóa   | Matplotlib, RViz       |
| Lưu kết quả     | JSON, CSV              |

Môi trường khuyến nghị:

```text
Ubuntu 20.04
ROS Noetic
Python 3.8+
```

---

## 3. Mục tiêu của version v1.0

### 3.1. Mục tiêu chính

Version `v1.0` cần đạt được:

* Đọc được file `.bag` chứa dữ liệu LiDAR Livox Mid-360.
* Trích xuất được point cloud gồm tọa độ điểm và intensity.
* Lọc được vùng quan tâm của point cloud.
* Phát hiện được các điểm có intensity cao bằng phương pháp phân ngưỡng.
* Gom cụm các điểm phản xạ bằng DBSCAN hoặc Euclidean clustering.
* Loại bỏ cụm nhiễu bằng các điều kiện hình học cơ bản.
* Ước lượng được tâm RF trong hệ tọa độ LiDAR.
* Xuất kết quả ra file `JSON`, `CSV`.
* Có log và dữ liệu trung gian để debug từng bước.

### 3.2. Những việc chưa làm trong v1.0

Version `v1.0` **chưa bắt buộc** làm các phần sau:

* Huấn luyện mạng nơ-ron.
* Tích hợp SVD localization hoàn chỉnh.
* Tối ưu real-time.
* ROS node chạy online.
* Tự động matching RF với bản đồ.
* Đánh giá toàn bộ quỹ đạo robot.

Các phần này sẽ được đưa vào các version sau.

---

## 4. Kiến thức cần chuẩn bị

## 4.1. Kiến thức lý thuyết cốt lõi

### 4.1.1. Point Cloud 3D

Cần hiểu point cloud là tập hợp các điểm 3D:

```text
P = {(x_i, y_i, z_i, I_i)}
```

Trong đó:

* `x, y, z`: tọa độ điểm trong hệ LiDAR.
* `I`: cường độ phản xạ của điểm.

**Tại sao cần học?**

Toàn bộ dữ liệu đầu vào của dự án là point cloud. Nếu không hiểu cấu trúc point cloud, rất dễ nhầm giữa tọa độ, intensity, range và frame tọa độ.

---

### 4.1.2. Intensity của LiDAR

Intensity là giá trị biểu diễn mức độ phản xạ của tia laser khi gặp vật thể. Vật liệu phản xạ như RF thường tạo ra intensity cao hơn môi trường xung quanh.

**Tại sao cần học?**

Phương pháp phân ngưỡng dựa trực tiếp trên intensity. Việc chọn ngưỡng không phù hợp sẽ dẫn đến:

* Bỏ sót RF thật.
* Phát hiện nhầm vật phản xạ giả.
* Tạo cụm nhiễu.
* Làm sai tâm RF.

---

### 4.1.3. Phân ngưỡng cường độ

Phân ngưỡng là quá trình chọn các điểm có intensity lớn hơn một giá trị ngưỡng:

```text
I_i >= T_I
```

Có hai dạng chính:

```text
Fixed threshold    : T_I cố định
Adaptive threshold : T_I thay đổi theo từng frame
```

**Tại sao cần học?**

Đây là thuật toán baseline chính của dự án. Nó cũng là phương pháp đối chứng khi so sánh với mạng nơ-ron trong bài báo.

---

### 4.1.4. Clustering

Clustering là quá trình gom các điểm gần nhau thành một cụm. Trong dự án này, clustering được dùng để gom các điểm intensity cao thành từng RF candidate.

Phương pháp khuyến nghị trong v1.0:

```text
DBSCAN
```

**Tại sao cần học?**

Sau khi phân ngưỡng, các điểm sáng có thể thuộc nhiều RF khác nhau hoặc thuộc nhiễu. Clustering giúp tách chúng thành từng cụm riêng biệt để xử lý.

---

### 4.1.5. Hệ tọa độ trong robot

Cần phân biệt rõ:

```text
lidar_frame : hệ tọa độ của LiDAR
base_frame  : hệ tọa độ thân robot
map_frame   : hệ tọa độ bản đồ
```

**Tại sao cần học?**

Trong v1.0, tâm RF chủ yếu được xuất trong `lidar_frame`. Tuy nhiên, các version sau sẽ cần chuyển sang `base_frame` hoặc `map_frame` để định vị robot. Nếu đặt tên biến không rõ hệ tọa độ, bug sẽ rất khó tìm.

---

### 4.1.6. Ước lượng tâm cụm

Tâm RF có thể được ước lượng bằng centroid:

```text
c = mean(points)
```

hoặc centroid có trọng số intensity:

```text
c = sum(w_i * p_i) / sum(w_i)
```

**Tại sao cần học?**

Tâm RF là đầu ra chính của detector. Nếu tâm bị lệch, bước matching và định vị robot phía sau cũng bị ảnh hưởng.

---

## 4.2. Kỹ năng công cụ và thư viện cần biết

### 4.2.1. ROS1 cơ bản

Cần biết:

* ROS topic.
* ROS message.
* ROS bag.
* `rostopic echo`.
* `rosbag info`.
* `rosbag play`.

**Tại sao cần học?**

Dữ liệu đầu vào của dự án là file `.bag`. Cần biết topic nào chứa LiDAR, message type là gì, tần số bao nhiêu và dữ liệu có đúng không.

---

### 4.2.2. Python

Cần biết:

* Function.
* Class.
* Dataclass.
* Type hint.
* Exception handling.
* Đọc/ghi file JSON, CSV.

**Tại sao cần học?**

Dự án được viết bằng Python. Code cần rõ ràng, dễ test và dễ debug, không nên viết thành một script quá dài.

---

### 4.2.3. NumPy

Cần biết:

* Mảng `np.ndarray`.
* Indexing.
* Boolean mask.
* Mean, min, max, percentile.
* Kiểm tra NaN/Inf.

**Tại sao cần học?**

Point cloud được xử lý chủ yếu dưới dạng mảng NumPy. Hầu hết các bước filtering, thresholding và center estimation đều dùng NumPy.

---

### 4.2.4. scikit-learn

Cần biết:

* `DBSCAN`.
* Input shape của thuật toán clustering.
* Ý nghĩa `eps`, `min_samples`.

**Tại sao cần học?**

DBSCAN là thuật toán clustering chính trong v1.0. Chọn sai `eps` hoặc `min_samples` sẽ làm mất RF hoặc gom sai cụm.

---

### 4.2.5. Matplotlib

Cần biết:

* Scatter plot 2D.
* Vẽ điểm raw, điểm bright, cụm RF, tâm RF.
* Lưu ảnh debug.

**Tại sao cần học?**

Dự án phải có khả năng debug trực quan. Một ảnh scatter plot tốt có thể giúp phát hiện lỗi nhanh hơn nhiều so với chỉ nhìn log.

---

### 4.2.6. Git và GitHub

Cần biết:

* Tạo repo.
* Commit theo từng phase.
* Viết README.
* Quản lý nhánh.
* Không commit file bag quá lớn.

**Tại sao cần học?**

Dự án cần được tổ chức để có thể upload GitHub, phục vụ công bố, tái lập kết quả và mở rộng sau này.

---

## 5. Cấu trúc thư mục đề xuất

```text
rf_threshold_localization/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── config/
│   └── threshold_v1.yaml
│
├── data/
│   ├── bags/
│   ├── maps/
│   └── results/
│
├── scripts/
│   ├── run_threshold_bag.py
│   ├── visualize_detections.py
│   └── check_bag_info.py
│
├── src/
│   └── rf_threshold/
│       │
│       ├── io/
│       │   ├── bag_reader.py
│       │   ├── pointcloud_parser.py
│       │   └── result_writer.py
│       │
│       ├── core/
│       │   ├── frame.py
│       │   ├── preprocessing.py
│       │   ├── thresholding.py
│       │   ├── clustering.py
│       │   ├── cluster_validation.py
│       │   ├── center_estimation.py
│       │   └── detector_pipeline.py
│       │
│       ├── visualization/
│       │   └── plot_frame.py
│       │
│       └── utils/
│           ├── config.py
│           ├── logger.py
│           └── geometry.py
│
└── tests/
    ├── test_thresholding.py
    ├── test_clustering.py
    └── test_center_estimation.py
```

Nguyên tắc tổ chức:

* `io/`: chỉ đọc và ghi dữ liệu.
* `core/`: chứa thuật toán chính.
* `visualization/`: chứa code vẽ debug.
* `utils/`: chứa hàm phụ trợ.
* `scripts/`: chứa file chạy từ terminal.
* `tests/`: chứa unit test cho từng module.

Không đặt toàn bộ code vào một file duy nhất.

---

## 6. Kiến trúc dữ liệu nội bộ

## 6.1. LidarFrame

```python
@dataclass
class LidarFrame:
    stamp: float
    frame_id: str
    points_xyz: np.ndarray
    intensity: np.ndarray
```

Ý nghĩa:

* `stamp`: thời gian của frame.
* `frame_id`: tên hệ tọa độ LiDAR.
* `points_xyz`: mảng điểm 3D, shape `(N, 3)`.
* `intensity`: mảng intensity, shape `(N,)`.

---

## 6.2. RFCluster

```python
@dataclass
class RFCluster:
    cluster_id: int
    point_indices: np.ndarray
    points_xyz: np.ndarray
    intensity: np.ndarray
```

Ý nghĩa:

* Đại diện cho một cụm điểm intensity cao.
* Chưa chắc cụm này là RF thật.
* Cần qua bước validation.

---

## 6.3. RFDetection

```python
@dataclass
class RFDetection:
    detection_id: int
    stamp: float
    frame_id: str
    center_lidar: np.ndarray
    score: float
    num_points: int
    mean_intensity: float
    max_intensity: float
    bbox_min: np.ndarray
    bbox_max: np.ndarray
```

Ý nghĩa:

* Đại diện cho một RF đã được detector chấp nhận.
* `center_lidar` là tâm RF trong hệ tọa độ LiDAR.
* `score` là điểm tin cậy dạng rule-based, không phải xác suất của mạng nơ-ron.

Lưu ý quan trọng:

```python
if detection_id:
    ...
```

là cách viết sai nếu `detection_id = 0`.

Cần viết:

```python
if detection_id is not None:
    ...
```

---

## 7. File cấu hình v1.0

File đề xuất:

```text
config/threshold_v1.yaml
```

Nội dung mẫu:

```yaml
bag:
  path: "data/bags/sample.bag"
  topic: "/livox/lidar"
  message_type: "auto"

preprocessing:
  remove_nan: true

  range_filter:
    enabled: true
    min_range: 0.20
    max_range: 8.00

  height_filter:
    enabled: true
    min_z: -0.50
    max_z: 0.50

threshold:
  mode: "fixed"
  fixed_intensity: 180.0

  adaptive:
    enabled: false
    percentile: 99.5
    min_intensity: 120.0

clustering:
  method: "dbscan"
  use_dimension: "xy"
  eps: 0.08
  min_samples: 3

cluster_validation:
  min_points: 3
  max_points: 200
  max_extent_x: 0.30
  max_extent_y: 0.30
  max_extent_z: 0.50
  min_mean_intensity: 120.0

center_estimation:
  method: "intensity_weighted"
  intensity_weight_power: 1.0
  clamp_percentile: 95.0

output:
  output_dir: "data/results/sample_run"
  save_json: true
  save_csv: true
  save_debug_image: true

debug:
  enabled: true
  print_every_n_frames: 1
```

Nguyên tắc:

* Không hard-code tham số trong code.
* Mọi tham số thuật toán phải nằm trong file YAML.
* Mỗi lần chạy cần copy file YAML vào thư mục kết quả dưới tên `config_used.yaml`.

---

## 8. Chiến lược quản lý lỗi và debug

## 8.1. Nguyên tắc debug chung

Khi gặp lỗi, luôn debug theo thứ tự sau:

```text
1. Kiểm tra file bag
2. Kiểm tra topic LiDAR
3. Kiểm tra message type
4. Kiểm tra parse point cloud
5. Kiểm tra số điểm raw
6. Kiểm tra số điểm sau preprocessing
7. Kiểm tra số điểm sau threshold
8. Kiểm tra số cụm sau clustering
9. Kiểm tra số cụm hợp lệ
10. Kiểm tra tâm RF xuất ra
```

Không debug từ cuối pipeline trước. Phải debug từ đầu vào.

---

## 8.2. Log chuẩn của mỗi frame

Mỗi frame nên có log dạng:

```text
[Frame 000123 | t=12.340]
raw=9821 | preprocessed=4210 | bright=64 | clusters=5 | valid=2 | threshold=180.0
```

Ý nghĩa:

| Trường         | Ý nghĩa                            |
| -------------- | ---------------------------------- |
| `raw`          | Số điểm ban đầu đọc từ LiDAR       |
| `preprocessed` | Số điểm sau lọc NaN, range, height |
| `bright`       | Số điểm vượt ngưỡng intensity      |
| `clusters`     | Số cụm sau DBSCAN                  |
| `valid`        | Số RF detection hợp lệ             |
| `threshold`    | Giá trị ngưỡng intensity được dùng |

---

## 8.3. File debug cần xuất

Mỗi lần chạy nên tạo thư mục:

```text
data/results/sample_run/
│
├── config_used.yaml
├── detections.json
├── detections.csv
├── frame_summary.csv
├── rejected_clusters.csv
└── debug_images/
    ├── frame_000000.png
    ├── frame_000001.png
    └── ...
```

### `frame_summary.csv`

```csv
frame_index,stamp,raw_points,preprocessed_points,bright_points,num_clusters,num_valid,threshold
0,12.000,9821,4210,64,5,2,180.0
1,12.100,9788,4189,51,4,2,180.0
```

### `rejected_clusters.csv`

```csv
frame_index,cluster_id,reason,num_points,extent_x,extent_y,mean_intensity
0,3,extent_x_too_large,35,0.72,0.06,190.4
0,4,num_points_too_small,1,0.00,0.00,230.1
```

### `detections.csv`

```csv
frame_index,stamp,detection_id,x_lidar,y_lidar,z_lidar,score,num_points,mean_intensity,max_intensity
0,12.000,0,1.24,0.53,0.02,0.86,12,212.4,245.0
```

---

## 8.4. Quy trình test cô lập module

Khi module bị lỗi, không chạy toàn bộ bag. Chỉ test riêng module đó.

Ví dụ:

```text
Lỗi threshold
  → chạy test_thresholding.py

Lỗi clustering
  → chạy test_clustering.py

Lỗi tâm RF sai
  → chạy test_center_estimation.py

Lỗi đọc bag
  → chạy scripts/check_bag_info.py
```

---

## 8.5. Quy tắc chống bug bắt buộc

### Quy tắc 1: Không dùng biến global cho tham số thuật toán

Không viết:

```python
THRESHOLD = 180
EPS = 0.08
```

Phải lấy từ config:

```python
threshold = cfg["threshold"]["fixed_intensity"]
eps = cfg["clustering"]["eps"]
```

---

### Quy tắc 2: Luôn xử lý input rỗng

Mọi module cần xử lý được trường hợp không có điểm:

```python
if points_xyz.shape[0] == 0:
    return []
```

---

### Quy tắc 3: Không trộn hệ tọa độ

Tên biến phải có hậu tố hệ tọa độ:

```python
center_lidar
center_base
center_map
```

Không dùng tên chung chung:

```python
center
point
position
```

---

### Quy tắc 4: Không kiểm tra ID bằng boolean

Sai:

```python
if mirror_id:
    ...
```

Đúng:

```python
if mirror_id is not None:
    ...
```

---

### Quy tắc 5: Luôn lưu output trung gian

Nếu chỉ lưu kết quả cuối cùng, khi sai sẽ không biết lỗi phát sinh ở đâu.

---

# 9. Lộ trình phát triển từng bước

Dự án v1.0 được chia thành 5 phase. Mỗi phase chỉ giải quyết một bài toán cụ thể.

---

# Phase 1: Setup môi trường và khung project

## Mục tiêu của Phase

Hoàn thành cấu trúc project ban đầu, cài đặt môi trường và đảm bảo có thể chạy một script Python tối thiểu trong môi trường ROS1.

Sau phase này, project phải có:

* Cấu trúc thư mục chuẩn.
* File config đầu tiên.
* File requirements.
* Script kiểm tra môi trường.
* Git repo sạch.

---

## Đầu vào và đầu ra

### Inputs

```text
Ubuntu 20.04
ROS Noetic
Python 3.8+
Git
```

### Outputs

```text
Project folder hoàn chỉnh
requirements.txt
config/threshold_v1.yaml
scripts/check_environment.py
README.md bản đầu tiên
```

---

## Các bước triển khai chi tiết

### Bước 1.1: Tạo thư mục project

```bash
mkdir -p rf_threshold_localization
cd rf_threshold_localization
```

### Bước 1.2: Tạo cấu trúc thư mục

```bash
mkdir -p config
mkdir -p data/bags data/maps data/results
mkdir -p scripts
mkdir -p src/rf_threshold/io
mkdir -p src/rf_threshold/core
mkdir -p src/rf_threshold/visualization
mkdir -p src/rf_threshold/utils
mkdir -p tests
```

### Bước 1.3: Tạo file `.gitignore`

Nội dung khuyến nghị:

```gitignore
__pycache__/
*.pyc
*.bag
data/bags/
data/results/
.venv/
.vscode/
*.log
```

Không commit file `.bag` lớn lên GitHub.

### Bước 1.4: Tạo `requirements.txt`

```txt
numpy
scipy
scikit-learn
matplotlib
pyyaml
pandas
pytest
```

### Bước 1.5: Tạo file config đầu tiên

Tạo:

```text
config/threshold_v1.yaml
```

Dùng nội dung config mẫu ở phần trên.

### Bước 1.6: Tạo script kiểm tra môi trường

File:

```text
scripts/check_environment.py
```

Script cần kiểm tra:

* Python version.
* Import NumPy.
* Import sklearn.
* Import yaml.
* Import matplotlib.
* Import rosbag nếu chạy trong môi trường ROS.

### Bước 1.7: Khởi tạo Git

```bash
git init
git add .
git commit -m "Initialize threshold RF localization project structure"
```

---

## Tiêu chí nghiệm thu

Phase này hoàn thành khi:

* Chạy được:

```bash
python scripts/check_environment.py
```

* Không báo lỗi import thư viện cơ bản.
* Cấu trúc thư mục đúng như kế hoạch.
* Có file `threshold_v1.yaml`.
* Có commit đầu tiên trong Git.

---

## Chỉ dẫn Debug cho Phase này

### Lỗi 1: Không import được `rosbag`

Nguyên nhân thường gặp:

* Chưa source ROS.
* Không chạy trong môi trường ROS1.

Cách xử lý:

```bash
source /opt/ros/noetic/setup.bash
```

Sau đó chạy lại:

```bash
python scripts/check_environment.py
```

### Lỗi 2: Không import được `sklearn`

Cài lại:

```bash
pip install scikit-learn
```

### Lỗi 3: Python dùng sai version

Kiểm tra:

```bash
which python
python --version
```

Với ROS Noetic, nên dùng Python 3.

---

# Phase 2: Đọc ROS bag và parse dữ liệu Livox LiDAR

## Mục tiêu của Phase

Đọc được file `.bag`, lấy đúng topic LiDAR, chuyển message LiDAR thành object `LidarFrame`.

Sau phase này, chương trình phải in được:

```text
Frame index
Timestamp
Frame ID
Number of points
Intensity min/max/mean
```

---

## Đầu vào và đầu ra

### Inputs

```text
File .bag chứa dữ liệu Livox Mid-360
Tên topic LiDAR, ví dụ: /livox/lidar
```

### Outputs

```text
LidarFrame:
    stamp
    frame_id
    points_xyz
    intensity
```

---

## Các bước triển khai chi tiết

### Bước 2.1: Kiểm tra thông tin bag

Tạo script:

```text
scripts/check_bag_info.py
```

Script này cần in:

```text
Bag path
List of topics
Message type của từng topic
Số lượng message
Thời lượng bag
```

Có thể kiểm tra ngoài terminal bằng:

```bash
rosbag info data/bags/sample.bag
```

### Bước 2.2: Xác định topic LiDAR

Trong file config:

```yaml
bag:
  path: "data/bags/sample.bag"
  topic: "/livox/lidar"
```

Topic có thể khác tùy bag:

```text
/livox/lidar
/livox/imu
/livox_points
```

Chỉ chọn topic point cloud.

### Bước 2.3: Tạo dataclass `LidarFrame`

File:

```text
src/rf_threshold/core/frame.py
```

Nội dung chính:

```python
from dataclasses import dataclass
import numpy as np

@dataclass
class LidarFrame:
    stamp: float
    frame_id: str
    points_xyz: np.ndarray
    intensity: np.ndarray
```

### Bước 2.4: Viết module parse point cloud

File:

```text
src/rf_threshold/io/pointcloud_parser.py
```

Module này cần hỗ trợ tối thiểu:

```text
sensor_msgs/PointCloud2
```

Đầu ra bắt buộc:

```python
points_xyz.shape == (N, 3)
intensity.shape == (N,)
```

Nếu không tìm thấy intensity field, phải báo lỗi rõ ràng.

### Bước 2.5: Viết module đọc bag

File:

```text
src/rf_threshold/io/bag_reader.py
```

Hàm chính:

```python
def read_lidar_frames(bag_path: str, topic: str, cfg: dict):
    ...
```

Hàm này trả về từng `LidarFrame`.

### Bước 2.6: Tạo script chạy thử

File:

```text
scripts/run_read_bag.py
```

Output mẫu:

```text
[Frame 000000] stamp=12.000 | frame_id=livox_frame | points=9821 | intensity=[0.0, 255.0], mean=32.4
[Frame 000001] stamp=12.100 | frame_id=livox_frame | points=9788 | intensity=[0.0, 248.0], mean=31.9
```

---

## Tiêu chí nghiệm thu

Phase này hoàn thành khi:

* Đọc được ít nhất 10 frame từ bag.
* Mỗi frame có `points_xyz` shape `(N, 3)`.
* Mỗi frame có `intensity` shape `(N,)`.
* `N > 0`.
* In được min, max, mean intensity.
* Không có lỗi khi gặp frame rỗng hoặc message không đúng topic.

---

## Chỉ dẫn Debug cho Phase này

### Lỗi 1: Không tìm thấy topic

Kiểm tra bằng:

```bash
rosbag info data/bags/sample.bag
```

Sau đó sửa lại:

```yaml
bag:
  topic: "topic_đúng"
```

### Lỗi 2: Không có intensity

Nguyên nhân:

* Message không phải point cloud.
* Parser chưa đọc đúng field.
* Livox dùng custom message.

Cách xử lý:

* In danh sách field của message.
* Kiểm tra message type.
* Viết parser riêng cho Livox custom message nếu cần.

### Lỗi 3: Số điểm bằng 0

Kiểm tra:

* Bag có dữ liệu không.
* Topic có đúng không.
* Message có bị lọc nhầm không.

---

# Phase 3: Tiền xử lý point cloud

## Mục tiêu của Phase

Lọc point cloud thô để loại bỏ điểm không hợp lệ và giới hạn vùng quan tâm.

Sau phase này, mỗi frame cần có:

```text
raw_points
valid_points
range_filtered_points
height_filtered_points
```

---

## Đầu vào và đầu ra

### Inputs

```text
LidarFrame thô từ Phase 2
```

### Outputs

```text
LidarFrame đã lọc
Frame summary chứa số điểm trước/sau từng bước
```

---

## Các bước triển khai chi tiết

### Bước 3.1: Viết hàm loại NaN và Inf

File:

```text
src/rf_threshold/core/preprocessing.py
```

Hàm:

```python
def remove_invalid_points(frame: LidarFrame) -> LidarFrame:
    ...
```

Điều kiện giữ điểm:

```python
np.isfinite(x)
np.isfinite(y)
np.isfinite(z)
np.isfinite(intensity)
```

### Bước 3.2: Viết range filter

Tính range:

```text
r = sqrt(x^2 + y^2 + z^2)
```

Giữ lại điểm thỏa:

```text
min_range <= r <= max_range
```

Tham số lấy từ config:

```yaml
range_filter:
  min_range: 0.20
  max_range: 8.00
```

### Bước 3.3: Viết height filter

Giữ lại điểm thỏa:

```text
min_z <= z <= max_z
```

Tham số lấy từ config:

```yaml
height_filter:
  min_z: -0.50
  max_z: 0.50
```

### Bước 3.4: Gom thành hàm preprocessing chính

Hàm:

```python
def preprocess_frame(frame: LidarFrame, cfg: dict) -> tuple[LidarFrame, dict]:
    ...
```

Trả về:

```python
filtered_frame
summary
```

Trong đó `summary` chứa:

```python
{
    "raw_points": ...,
    "valid_points": ...,
    "range_filtered_points": ...,
    "height_filtered_points": ...
}
```

### Bước 3.5: Tạo ảnh debug 2D

File:

```text
src/rf_threshold/visualization/plot_frame.py
```

Vẽ:

* Raw points màu nhạt.
* Filtered points màu đậm hơn.
* Trục `x-y`.
* Giới hạn vùng quan sát.

---

## Tiêu chí nghiệm thu

Phase này hoàn thành khi:

* Chạy preprocessing được trên ít nhất 100 frame.
* Không crash khi frame có NaN hoặc Inf.
* Số điểm sau lọc nhỏ hơn hoặc bằng số điểm raw.
* Xuất được `frame_summary.csv`.
* Vẽ được ít nhất một ảnh debug cho frame bất kỳ.

---

## Chỉ dẫn Debug cho Phase này

### Lỗi 1: Sau preprocessing không còn điểm nào

Nguyên nhân có thể:

* `min_range`, `max_range` quá chặt.
* `min_z`, `max_z` không phù hợp với hệ tọa độ LiDAR.
* Trục z không đúng như giả định.

Cách xử lý:

* In min/max của x, y, z.
* Tạm thời tắt height filter.
* Tạm thời tăng `max_range`.

### Lỗi 2: Điểm bị lệch bất thường trên ảnh debug

Nguyên nhân có thể:

* Đọc sai đơn vị.
* Đọc sai field x, y, z.
* Dữ liệu không thuộc hệ LiDAR như giả định.

Cách xử lý:

* In vài điểm đầu tiên.
* So sánh với RViz.
* Kiểm tra frame_id.

### Lỗi 3: Intensity toàn 0

Nguyên nhân có thể:

* Parser đọc sai field intensity.
* Topic không chứa intensity.
* Message type không đúng.

Cách xử lý:

* In danh sách field của PointCloud2.
* Kiểm tra lại parser.

---

# Phase 4: Phân ngưỡng, clustering và kiểm tra cụm RF

## Mục tiêu của Phase

Từ point cloud đã lọc, phát hiện các cụm điểm có khả năng là RF bằng phương pháp phân ngưỡng intensity và clustering.

Sau phase này, hệ thống phải tạo được danh sách `RFCluster` hợp lệ.

---

## Đầu vào và đầu ra

### Inputs

```text
LidarFrame đã preprocessing
Config threshold
Config clustering
Config cluster_validation
```

### Outputs

```text
Danh sách RFCluster hợp lệ
Danh sách cụm bị reject kèm lý do
Ảnh debug hiển thị bright points và cluster
```

---

## Các bước triển khai chi tiết

### Bước 4.1: Viết fixed threshold

File:

```text
src/rf_threshold/core/thresholding.py
```

Hàm:

```python
def apply_fixed_threshold(frame: LidarFrame, threshold: float) -> LidarFrame:
    ...
```

Điều kiện:

```text
I_i >= T_I
```

### Bước 4.2: Viết adaptive threshold dạng đơn giản

Trong v1.0, adaptive threshold chỉ cần dùng percentile:

```text
T_I = max(T_min, percentile(I, p))
```

Ví dụ:

```yaml
adaptive:
  enabled: true
  percentile: 99.5
  min_intensity: 120.0
```

Hàm:

```python
def compute_adaptive_threshold(intensity: np.ndarray, cfg: dict) -> float:
    ...
```

### Bước 4.3: Viết hàm chọn bright points

Hàm:

```python
def select_bright_points(frame: LidarFrame, threshold: float) -> LidarFrame:
    ...
```

Nếu không có điểm bright, trả về frame rỗng hợp lệ, không crash.

### Bước 4.4: Viết DBSCAN clustering

File:

```text
src/rf_threshold/core/clustering.py
```

Hàm:

```python
def cluster_bright_points(frame: LidarFrame, cfg: dict) -> list[RFCluster]:
    ...
```

Nếu `use_dimension = xy`:

```python
features = points_xyz[:, :2]
```

Nếu `use_dimension = xyz`:

```python
features = points_xyz
```

### Bước 4.5: Viết kiểm tra cụm

File:

```text
src/rf_threshold/core/cluster_validation.py
```

Với mỗi cụm, tính:

```text
num_points
extent_x
extent_y
extent_z
mean_intensity
max_intensity
bbox_min
bbox_max
```

Điều kiện giữ cụm:

```text
min_points <= num_points <= max_points
extent_x <= max_extent_x
extent_y <= max_extent_y
extent_z <= max_extent_z
mean_intensity >= min_mean_intensity
```

Hàm:

```python
def validate_cluster_with_reason(cluster: RFCluster, cfg: dict) -> tuple[bool, str]:
    ...
```

### Bước 4.6: Lưu cụm bị reject

Mỗi cụm bị loại cần lưu lý do vào:

```text
rejected_clusters.csv
```

Ví dụ reason:

```text
num_points_too_small
num_points_too_large
extent_x_too_large
extent_y_too_large
extent_z_too_large
mean_intensity_too_low
```

### Bước 4.7: Vẽ ảnh debug clustering

Ảnh debug cần có:

* Tất cả điểm sau preprocessing.
* Bright points.
* Cluster hợp lệ.
* Cluster bị reject.
* Tâm cụm nếu đã có.

---

## Tiêu chí nghiệm thu

Phase này hoàn thành khi:

* Chạy được phân ngưỡng trên ít nhất 100 frame.
* Không crash nếu không có điểm bright.
* DBSCAN không crash với input rỗng.
* Có `rejected_clusters.csv`.
* Có ảnh debug cho một số frame.
* Có log dạng:

```text
raw=9821 | preprocessed=4210 | bright=64 | clusters=5 | valid=2 | threshold=180.0
```

---

## Chỉ dẫn Debug cho Phase này

### Lỗi 1: Không phát hiện RF nào

Nguyên nhân có thể:

* Threshold quá cao.
* Height filter loại mất RF.
* Range filter quá hẹp.
* Intensity parser sai.

Cách xử lý:

* In `max_intensity`.
* Giảm `fixed_intensity`.
* Tạm thời tắt height filter.
* Vẽ histogram intensity.

### Lỗi 2: Quá nhiều điểm bright

Nguyên nhân có thể:

* Threshold quá thấp.
* Có nhiều vật phản xạ mạnh.
* Dữ liệu intensity chưa chuẩn hóa.

Cách xử lý:

* Tăng threshold.
* Dùng adaptive threshold.
* Tăng `min_mean_intensity`.
* Thêm kiểm tra extent cụm.

### Lỗi 3: DBSCAN gom nhiều RF thành một cụm

Nguyên nhân:

* `eps` quá lớn.

Cách xử lý:

* Giảm `eps`.
* Dùng clustering theo `xy`.
* Kiểm tra ảnh debug.

### Lỗi 4: Một RF bị tách thành nhiều cụm

Nguyên nhân:

* `eps` quá nhỏ.
* Điểm RF thưa.

Cách xử lý:

* Tăng `eps`.
* Giảm `min_samples`.

---

# Phase 5: Ước lượng tâm RF, xuất kết quả và hoàn thiện baseline v1.0

## Mục tiêu của Phase

Từ các cụm RF hợp lệ, ước lượng tâm RF và xuất kết quả detection ở định dạng chuẩn để dùng cho bài báo và các version sau.

Sau phase này, dự án v1.0 phải có thể chạy một lệnh duy nhất để xử lý bag và xuất kết quả.

---

## Đầu vào và đầu ra

### Inputs

```text
Danh sách RFCluster hợp lệ
Thông tin frame
Config center_estimation
```

### Outputs

```text
detections.json
detections.csv
frame_summary.csv
debug_images/
```

---

## Các bước triển khai chi tiết

### Bước 5.1: Viết centroid estimator

File:

```text
src/rf_threshold/core/center_estimation.py
```

Centroid thường:

```text
c = mean(points_xyz)
```

Hàm:

```python
def estimate_centroid(points_xyz: np.ndarray) -> np.ndarray:
    ...
```

### Bước 5.2: Viết intensity-weighted centroid

Công thức:

```text
c = sum(w_i * p_i) / sum(w_i)
```

Trong đó:

```text
w_i = max(I_i - T_I, 0)^gamma
```

Cần clamp intensity để tránh một điểm quá sáng kéo lệch tâm:

```text
I_clamped = min(I, percentile(I, 95))
```

Hàm:

```python
def estimate_intensity_weighted_center(cluster: RFCluster, threshold: float, cfg: dict) -> np.ndarray:
    ...
```

### Bước 5.3: Tạo object RFDetection

Từ mỗi cụm hợp lệ, tạo:

```python
RFDetection(
    detection_id=...,
    stamp=...,
    frame_id=...,
    center_lidar=...,
    score=...,
    num_points=...,
    mean_intensity=...,
    max_intensity=...,
    bbox_min=...,
    bbox_max=...
)
```

### Bước 5.4: Tính score đơn giản cho detection

Score rule-based có thể tính theo:

```text
score = 0.5 * intensity_score
      + 0.3 * compactness_score
      + 0.2 * point_count_score
```

Trong v1.0, score chỉ dùng để debug, chưa dùng cho SVD.

### Bước 5.5: Viết result writer

File:

```text
src/rf_threshold/io/result_writer.py
```

Cần hỗ trợ:

```text
detections.json
detections.csv
frame_summary.csv
rejected_clusters.csv
```

### Bước 5.6: Viết pipeline chính

File:

```text
src/rf_threshold/core/detector_pipeline.py
```

Class chính:

```python
class ThresholdRFDetector:
    def __init__(self, cfg: dict):
        self.cfg = cfg

    def detect(self, frame: LidarFrame) -> list[RFDetection]:
        ...
```

Pipeline:

```text
preprocess_frame
  ↓
compute_threshold
  ↓
select_bright_points
  ↓
cluster_bright_points
  ↓
validate_cluster
  ↓
estimate_center
  ↓
create_detection
```

### Bước 5.7: Viết script chạy chính

File:

```text
scripts/run_threshold_bag.py
```

Lệnh chạy:

```bash
python scripts/run_threshold_bag.py \
  --config config/threshold_v1.yaml
```

Hoặc:

```bash
python scripts/run_threshold_bag.py \
  --bag data/bags/sample.bag \
  --topic /livox/lidar \
  --config config/threshold_v1.yaml \
  --output data/results/sample_run
```

### Bước 5.8: Viết script visualize detection

File:

```text
scripts/visualize_detections.py
```

Chức năng:

* Đọc `detections.json`.
* Vẽ tâm RF theo từng frame.
* Vẽ overlay lên point cloud nếu có debug data.

---

## Tiêu chí nghiệm thu

Phase này hoàn thành khi:

* Chạy được lệnh chính trên một file bag.
* Tạo đủ các file:

```text
config_used.yaml
detections.json
detections.csv
frame_summary.csv
rejected_clusters.csv
```

* Có ảnh debug trong `debug_images/`.
* Mỗi detection có đầy đủ:

```text
frame_index
stamp
detection_id
center_lidar
score
num_points
mean_intensity
max_intensity
bbox
```

* Không crash khi frame không có RF.
* Không crash khi không có bright points.
* Không crash khi tất cả cluster bị reject.
* Unit test cơ bản pass.

---

## Chỉ dẫn Debug cho Phase này

### Lỗi 1: Tâm RF bị NaN

Nguyên nhân:

* Tổng trọng số bằng 0.
* Cluster rỗng.
* Intensity bị NaN.

Cách xử lý:

* Kiểm tra cluster có điểm không.
* Nếu tổng weight bằng 0, fallback sang centroid thường.
* Loại NaN từ preprocessing.

### Lỗi 2: `detections.json` rỗng

Nguyên nhân:

* Không có cụm hợp lệ.
* Threshold quá cao.
* Validation quá chặt.

Cách xử lý:

* Kiểm tra `frame_summary.csv`.
* Kiểm tra số `bright_points`.
* Kiểm tra `rejected_clusters.csv`.
* Mở ảnh debug.

### Lỗi 3: detection_id bắt đầu từ 1 hoặc bị nhảy ID

Trong mỗi frame, có thể cho detection ID bắt đầu từ 0:

```python
detection_id = len(detections)
```

Không dùng ID để kiểm tra boolean.

### Lỗi 4: File output bị ghi đè

Cách xử lý:

* Mỗi lần chạy tạo thư mục output riêng.
* Lưu `config_used.yaml`.
* Có thể thêm timestamp vào tên thư mục.

---

# 10. Command chạy dự án v1.0

## 10.1. Kiểm tra môi trường

```bash
python scripts/check_environment.py
```

## 10.2. Kiểm tra thông tin bag

```bash
python scripts/check_bag_info.py \
  --bag data/bags/sample.bag
```

## 10.3. Chạy detector phân ngưỡng

```bash
python scripts/run_threshold_bag.py \
  --config config/threshold_v1.yaml
```

## 10.4. Chạy visualize

```bash
python scripts/visualize_detections.py \
  --result data/results/sample_run/detections.json
```

## 10.5. Chạy test

```bash
pytest tests/
```

---

# 11. Definition of Done cho toàn bộ v1.0

Version `v1.0` được xem là hoàn thành khi đạt tất cả tiêu chí sau:

## 11.1. Về chức năng

* Đọc được file bag chứa dữ liệu Livox Mid-360.
* Parse được `x, y, z, intensity`.
* Lọc được point cloud theo range và height.
* Phân ngưỡng được điểm intensity cao.
* Gom cụm được bằng DBSCAN.
* Loại được cụm nhiễu bằng điều kiện hình học.
* Ước lượng được tâm RF.
* Xuất được detection ra JSON và CSV.
* Có ảnh debug.

## 11.2. Về code

* Không có script quá dài chứa toàn bộ logic.
* Mỗi module chỉ làm một nhiệm vụ.
* Không dùng biến global cho tham số thuật toán.
* Mọi tham số lấy từ YAML.
* Có xử lý input rỗng.
* Có unit test cơ bản.

## 11.3. Về debug

* Có `frame_summary.csv`.
* Có `rejected_clusters.csv`.
* Có log theo từng frame.
* Có ảnh debug.
* Có thể biết lỗi nằm ở bước nào mà không cần đọc toàn bộ code.

## 11.4. Về GitHub

* Có README rõ ràng.
* Có requirements.
* Có config mẫu.
* Không commit file bag lớn.
* Có `.gitignore`.
* Có hướng dẫn chạy.

---

# 12. Hướng phát triển sau v1.0

Sau khi hoàn thành v1.0, có thể phát triển tiếp theo các hướng sau.

## v1.1: Tích hợp RF map và SVD localization

Thêm:

```text
RF map loader
RF matching
SVD pose estimation
Pose output
```

Pipeline:

```text
RF detections
  ↓
Match with RF map
  ↓
Estimate robot pose using SVD
```

---

## v1.2: Đánh giá định lượng

Thêm:

```text
Ground truth loader
RF detection evaluation
Center error evaluation
Pose error evaluation
```

Metrics:

```text
Precision
Recall
F1-score
MAE
RMSE
Max error
```

---

## v2.0: Neural detector

Thay block:

```text
Threshold detector
```

bằng:

```text
Neural network detector
```

Nhưng giữ nguyên output format:

```text
detections.json
detections.csv
```

Nhờ đó backend matching và SVD không cần thay đổi.

---

# 13. Ghi chú thiết kế quan trọng

Dự án này phải được thiết kế theo nguyên tắc:

```text
Detector tạo RF observations.
Localization backend sử dụng RF observations.
Detector và localization backend không phụ thuộc cứng vào nhau.
```

Trong v1.0:

```text
Detector = threshold-based detector
```

Trong v2.0:

```text
Detector = neural network-based detector
```

Nếu output của hai detector giống nhau, việc so sánh trong bài báo sẽ công bằng và dễ triển khai.

---

# 14. Tóm tắt pipeline v1.0

```text
Input:
    ROS bag chứa Livox Mid-360 point cloud

Processing:
    1. Đọc LiDAR frame
    2. Parse x, y, z, intensity
    3. Remove NaN/Inf
    4. Range filter
    5. Height filter
    6. Intensity threshold
    7. DBSCAN clustering
    8. Cluster validation
    9. RF center estimation
    10. Export result

Output:
    detections.json
    detections.csv
    frame_summary.csv
    rejected_clusters.csv
    debug_images/
```

Version `v1.0` không cần phức tạp. Mục tiêu quan trọng nhất là tạo ra một baseline phân ngưỡng **sạch, ổn định, dễ debug và có thể tái lập kết quả**. Đây sẽ là nền tảng để so sánh công bằng với phương pháp mạng nơ-ron trong bài báo khoa học.
