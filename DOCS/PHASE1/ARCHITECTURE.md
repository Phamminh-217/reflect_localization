# ARCHITECTURE.md

# Kiến trúc hệ thống định vị robot bằng phân ngưỡng

## 0. Mục đích tài liệu

Tài liệu này mô tả kiến trúc phần mềm của dự án **hệ thống định vị robot bằng phân ngưỡng**.

Mục tiêu của tài liệu là giúp:

* Người phát triển hiểu rõ cấu trúc tổng thể của hệ thống.
* AI agents như Cursor, GitHub Copilot, ChatGPT Code Assistant biết cần chỉnh sửa file nào, thêm module ở đâu và không làm gãy pipeline.
* Dự án có thể mở rộng từ baseline phân ngưỡng sang các phương pháp nâng cao như mạng nơ-ron hoặc localization backend mà không cần viết lại toàn bộ hệ thống.

Dự án sử dụng các công nghệ core:

```text
Python
ROS1
LiDAR
NumPy
```

Đầu vào chính của hệ thống là dữ liệu LiDAR 3D, ưu tiên dạng ROS bag chứa point cloud từ Livox Mid-360. Mục tiêu đầu ra là phát hiện các mốc phản xạ RF và sử dụng chúng để hỗ trợ ước lượng vị trí robot.

---

# 1. Tổng quan kiến trúc

## 1.1. Mô hình kiến trúc tổng thể

Hệ thống được thiết kế theo mô hình:

```text
Modular Architecture + Pipeline-based Processing
```

Trong đó:

* **Modular Architecture**: mỗi khối xử lý được tách thành một module độc lập, có trách nhiệm rõ ràng.
* **Pipeline-based Processing**: dữ liệu đi tuần tự từ đầu vào thô đến các bước xử lý trung gian, sau đó tạo ra output cuối cùng.

Pipeline tổng quát:

```text
ROS Bag / LiDAR Data
        ↓
Bag Reader
        ↓
PointCloud Parser
        ↓
Preprocessing
        ↓
Intensity Thresholding
        ↓
Clustering
        ↓
Cluster Validation
        ↓
RF Center Estimation
        ↓
RF Detection Output
        ↓
Optional: Matching + SVD Localization
```

Ở version đầu tiên `v1.0`, trọng tâm là:

```text
ROS Bag → RF Detection Output
```

Các phần như RF matching, SVD localization và neural detector sẽ được thiết kế để mở rộng sau, nhưng không được làm ảnh hưởng đến module phân ngưỡng hiện tại.

---

## 1.2. Lý do chọn kiến trúc này

Kiến trúc modular pipeline phù hợp với dự án vì các lý do sau:

## 1.2.1. Dễ debug

Mỗi bước xử lý có input và output rõ ràng. Khi kết quả sai, có thể kiểm tra từng tầng:

```text
Raw point cloud có đúng không?
Intensity có đúng không?
Threshold có chọn đúng điểm sáng không?
Clustering có gom đúng cụm không?
Validation có loại nhầm RF không?
Center estimation có bị lệch không?
```

Không cần đọc toàn bộ hệ thống để tìm lỗi.

---

## 1.2.2. Dễ thay thế thuật toán

Detector phân ngưỡng và detector mạng nơ-ron sau này có thể dùng chung định dạng output:

```text
RFDetection
```

Do đó backend phía sau, ví dụ matching hoặc SVD localization, không cần biết RF được phát hiện bằng phân ngưỡng hay bằng mạng nơ-ron.

Nguyên tắc thiết kế:

```text
Detector nào cũng phải trả về list[RFDetection]
```

---

## 1.2.3. Phù hợp với bài báo khoa học

Dự án cần so sánh:

```text
Threshold-based RF detection + Localization
```

với:

```text
Neural-network-based RF detection + Localization
```

Nếu hai phương pháp dùng chung pipeline phía sau, kết quả so sánh sẽ công bằng và dễ giải thích trong bài báo.

---

## 1.2.4. Dễ mở rộng sang ROS node online

Ở version đầu, hệ thống chạy offline trên ROS bag. Tuy nhiên, kiến trúc vẫn cho phép mở rộng sang ROS node online bằng cách thay `BagReader` bằng `ROSSubscriber`.

Pipeline xử lý core không cần thay đổi.

---

# 2. Cấu trúc thư mục dự án

## 2.1. Directory tree chuẩn

```text
rf_threshold_localization/
│
├── README.md
├── ARCHITECTURE.md
├── requirements.txt
├── .gitignore
│
├── config/
│   ├── threshold_v1.yaml
│   └── debug.yaml
│
├── data/
│   ├── bags/
│   ├── maps/
│   ├── samples/
│   └── results/
│
├── scripts/
│   ├── check_environment.py
│   ├── check_bag_info.py
│   ├── run_read_bag.py
│   ├── run_threshold_bag.py
│   └── visualize_detections.py
│
├── src/
│   └── rf_threshold/
│       │
│       ├── __init__.py
│       │
│       ├── io/
│       │   ├── __init__.py
│       │   ├── bag_reader.py
│       │   ├── pointcloud_parser.py
│       │   └── result_writer.py
│       │
│       ├── core/
│       │   ├── __init__.py
│       │   ├── frame.py
│       │   ├── preprocessing.py
│       │   ├── thresholding.py
│       │   ├── clustering.py
│       │   ├── cluster_validation.py
│       │   ├── center_estimation.py
│       │   ├── scoring.py
│       │   └── detector_pipeline.py
│       │
│       ├── localization/
│       │   ├── __init__.py
│       │   ├── map_loader.py
│       │   ├── rf_matcher.py
│       │   └── svd_pose.py
│       │
│       ├── visualization/
│       │   ├── __init__.py
│       │   ├── plot_frame.py
│       │   └── rviz_marker.py
│       │
│       └── utils/
│           ├── __init__.py
│           ├── config.py
│           ├── logger.py
│           ├── geometry.py
│           └── transform.py
│
└── tests/
    ├── test_thresholding.py
    ├── test_clustering.py
    ├── test_center_estimation.py
    └── test_svd_pose.py
```

---

## 2.2. Chức năng từng thư mục

## 2.2.1. `config/`

Chứa các file cấu hình YAML.

Ví dụ:

```text
config/threshold_v1.yaml
```

File này chứa toàn bộ tham số thuật toán:

* Topic LiDAR.
* Range filter.
* Height filter.
* Threshold intensity.
* Tham số DBSCAN.
* Điều kiện validation cụm.
* Tham số ước lượng tâm RF.
* Đường dẫn output.

Quy tắc:

```text
Không hard-code tham số thuật toán trong source code.
Mọi tham số phải đọc từ file YAML.
```

---

## 2.2.2. `data/`

Chứa dữ liệu đầu vào, dữ liệu mẫu và kết quả đầu ra.

```text
data/bags/      : chứa ROS bag local, không commit lên GitHub nếu file lớn
data/maps/      : chứa bản đồ RF nếu dùng localization
data/samples/   : chứa dữ liệu mẫu nhỏ có thể commit
data/results/   : chứa kết quả chạy detector
```

Quy tắc:

```text
Không commit ROS bag lớn.
Không commit kết quả chạy quá nặng.
Chỉ commit sample nhỏ nếu cần tái lập demo.
```

---

## 2.2.3. `scripts/`

Chứa các script chạy từ terminal.

Các file trong `scripts/` chỉ đóng vai trò entry-point. Không viết thuật toán chính trong thư mục này.

Ví dụ:

```text
scripts/run_threshold_bag.py
```

File này chỉ nên làm các việc:

1. Đọc arguments.
2. Load config.
3. Gọi detector pipeline.
4. Ghi output.

Không đặt logic thresholding, clustering, validation trực tiếp trong script.

---

## 2.2.4. `src/rf_threshold/io/`

Chứa module đọc và ghi dữ liệu.

Nhiệm vụ:

* Đọc ROS bag.
* Parse message point cloud.
* Ghi kết quả JSON, CSV.
* Không xử lý thuật toán detection.

Các file chính:

```text
bag_reader.py
pointcloud_parser.py
result_writer.py
```

---

## 2.2.5. `src/rf_threshold/core/`

Chứa thuật toán chính của detector phân ngưỡng.

Nhiệm vụ:

* Tiền xử lý point cloud.
* Phân ngưỡng intensity.
* Clustering.
* Kiểm tra cụm.
* Ước lượng tâm RF.
* Tạo object detection.
* Tổ chức pipeline detector.

Đây là phần cốt lõi của version `v1.0`.

---

## 2.2.6. `src/rf_threshold/localization/`

Chứa các module phục vụ định vị robot.

Ở version `v1.0`, thư mục này có thể chưa hoàn thiện. Tuy nhiên, nó được tạo sẵn để tách rõ detector và localization backend.

Các module dự kiến:

```text
map_loader.py  : đọc bản đồ RF
rf_matcher.py  : so khớp RF quan sát với RF trong bản đồ
svd_pose.py    : ước lượng pose robot bằng SVD
```

Quy tắc:

```text
Localization không được phụ thuộc vào cách RF được phát hiện.
Localization chỉ nhận list[RFDetection].
```

---

## 2.2.7. `src/rf_threshold/visualization/`

Chứa module trực quan hóa.

Nhiệm vụ:

* Vẽ point cloud 2D.
* Vẽ điểm bright sau threshold.
* Vẽ cluster hợp lệ và cluster bị loại.
* Vẽ tâm RF.
* Xuất ảnh debug.

Không viết thuật toán detection trong thư mục này.

---

## 2.2.8. `src/rf_threshold/utils/`

Chứa hàm tiện ích dùng chung.

Ví dụ:

```text
config.py     : load file YAML
logger.py     : cấu hình log
geometry.py   : hàm hình học cơ bản
transform.py  : chuyển đổi hệ tọa độ
```

Quy tắc:

```text
utils không được chứa business logic chính.
```

---

## 2.2.9. `tests/`

Chứa unit test cho các module quan trọng.

Bắt buộc có test cho:

* Thresholding.
* Clustering.
* Center estimation.
* SVD pose nếu dùng localization.

Mỗi khi thêm module mới, cần thêm test tương ứng.

---

# 3. Dòng chảy dữ liệu và luồng xử lý

## 3.1. Sơ đồ pipeline tổng thể

```text
[ROS Bag]
    ↓
[BagReader]
    ↓
[PointCloudParser]
    ↓
[LidarFrame]
    ↓
[Preprocessor]
    ↓
[Filtered LidarFrame]
    ↓
[ThresholdSelector]
    ↓
[Bright Points]
    ↓
[ClusterExtractor]
    ↓
[RFCluster List]
    ↓
[ClusterValidator]
    ↓
[Valid RFCluster List]
    ↓
[CenterEstimator]
    ↓
[RFDetection List]
    ↓
[ResultWriter]
    ↓
[detections.json / detections.csv / debug files]
```

Nếu tích hợp localization:

```text
[RFDetection List]
    ↓
[MapLoader]
    ↓
[RFMatcher]
    ↓
[Correspondence Set]
    ↓
[SVDPoseEstimator]
    ↓
[Robot Pose]
```

---

## 3.2. Block 1: ROS Bag

### Nhiệm vụ

Chứa dữ liệu LiDAR thô được ghi lại từ robot.

### Kiểu dữ liệu

Input vật lý:

```text
*.bag
```

Trong bag có thể chứa nhiều topic:

```text
/livox/lidar
/livox/imu
/tf
/odom
```

Trong version `v1.0`, chỉ yêu cầu topic LiDAR.

### Output mong muốn

Message point cloud từ topic LiDAR.

---

## 3.3. Block 2: BagReader

### File đảm nhiệm

```text
src/rf_threshold/io/bag_reader.py
```

### Nhiệm vụ

* Mở file ROS bag.
* Đọc message từ topic LiDAR.
* Gửi từng message sang `PointCloudParser`.

### Input

```text
bag_path: str
topic: str
config: dict
```

### Output

```text
Iterator[LidarFrame]
```

### Giao tiếp với module khác

Gọi trực tiếp:

```text
pointcloud_parser.parse_pointcloud_message(...)
```

---

## 3.4. Block 3: PointCloudParser

### File đảm nhiệm

```text
src/rf_threshold/io/pointcloud_parser.py
```

### Nhiệm vụ

Chuyển ROS message thành dữ liệu NumPy chuẩn.

### Input

```text
ROS PointCloud2 message
hoặc Livox custom message nếu được hỗ trợ
```

### Output

```python
LidarFrame(
    stamp=float,
    frame_id=str,
    points_xyz=np.ndarray,   # shape: (N, 3)
    intensity=np.ndarray     # shape: (N,)
)
```

### Quy tắc bắt buộc

Nếu không đọc được intensity, module phải báo lỗi rõ ràng.

Không được tự tạo intensity giả nếu dữ liệu không có intensity.

---

## 3.5. Block 4: LidarFrame

### File đảm nhiệm

```text
src/rf_threshold/core/frame.py
```

### Nhiệm vụ

Là kiểu dữ liệu nội bộ đại diện cho một frame LiDAR.

### Cấu trúc đề xuất

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

### Quy tắc shape

```text
points_xyz.shape == (N, 3)
intensity.shape == (N,)
```

Nếu hai shape không khớp, phải raise error.

---

## 3.6. Block 5: Preprocessor

### File đảm nhiệm

```text
src/rf_threshold/core/preprocessing.py
```

### Nhiệm vụ

Lọc dữ liệu thô trước khi threshold.

Các bước chính:

```text
Remove NaN / Inf
Range filtering
Height filtering
Optional ROI filtering
```

### Input

```text
LidarFrame raw
```

### Output

```text
LidarFrame filtered
summary: dict
```

### Ví dụ summary

```python
{
    "raw_points": 9821,
    "valid_points": 9790,
    "range_filtered_points": 6210,
    "height_filtered_points": 4210
}
```

---

## 3.7. Block 6: ThresholdSelector

### File đảm nhiệm

```text
src/rf_threshold/core/thresholding.py
```

### Nhiệm vụ

Chọn các điểm có intensity cao.

### Input

```text
Filtered LidarFrame
threshold config
```

### Output

```text
Bright LidarFrame
threshold_value: float
```

### Hai chế độ threshold

Fixed threshold:

```text
I_i >= T_I
```

Adaptive threshold:

```text
T_I = max(T_min, percentile(I, p))
```

### Quy tắc

Nếu frame rỗng, trả về bright frame rỗng hợp lệ.

Không được crash khi `intensity.shape[0] == 0`.

---

## 3.8. Block 7: ClusterExtractor

### File đảm nhiệm

```text
src/rf_threshold/core/clustering.py
```

### Nhiệm vụ

Gom các bright points thành các cụm candidate RF.

### Input

```text
Bright LidarFrame
clustering config
```

### Output

```text
list[RFCluster]
```

### Kiểu dữ liệu RFCluster

```python
@dataclass
class RFCluster:
    cluster_id: int
    point_indices: np.ndarray
    points_xyz: np.ndarray
    intensity: np.ndarray
```

### Thuật toán đề xuất

```text
DBSCAN
```

### Quy tắc

Nếu không có bright point:

```text
return []
```

Không gọi DBSCAN với mảng rỗng.

---

## 3.9. Block 8: ClusterValidator

### File đảm nhiệm

```text
src/rf_threshold/core/cluster_validation.py
```

### Nhiệm vụ

Loại bỏ các cụm không giống RF.

### Input

```text
RFCluster
validation config
```

### Output

```text
valid: bool
reason: str
features: dict
```

### Tiêu chí kiểm tra

```text
min_points
max_points
max_extent_x
max_extent_y
max_extent_z
min_mean_intensity
```

### Ví dụ reason

```text
valid
num_points_too_small
num_points_too_large
extent_x_too_large
extent_y_too_large
extent_z_too_large
mean_intensity_too_low
```

### Quy tắc

Không chỉ trả về `True/False`.

Phải trả thêm reason để ghi vào `rejected_clusters.csv`.

---

## 3.10. Block 9: CenterEstimator

### File đảm nhiệm

```text
src/rf_threshold/core/center_estimation.py
```

### Nhiệm vụ

Ước lượng tâm RF từ cụm hợp lệ.

### Input

```text
RFCluster valid
threshold_value
center estimation config
```

### Output

```text
center_lidar: np.ndarray, shape (3,)
```

### Hai phương pháp

Centroid thường:

```text
center = mean(points_xyz)
```

Intensity-weighted centroid:

```text
center = sum(w_i * p_i) / sum(w_i)
```

với:

```text
w_i = max(I_i - T_I, 0)^gamma
```

### Quy tắc

Nếu tổng trọng số bằng 0, fallback về centroid thường.

Không được trả về NaN.

---

## 3.11. Block 10: Scoring

### File đảm nhiệm

```text
src/rf_threshold/core/scoring.py
```

### Nhiệm vụ

Tính điểm tin cậy rule-based cho detection.

### Input

```text
RFCluster
threshold_value
score config
```

### Output

```text
score: float trong khoảng [0, 1]
```

### Ý nghĩa

Score này dùng để:

* Debug.
* Sắp xếp RF detection.
* Có thể dùng làm trọng số ở weighted SVD trong version sau.

Không gọi score này là xác suất của mạng nơ-ron.

---

## 3.12. Block 11: DetectorPipeline

### File đảm nhiệm

```text
src/rf_threshold/core/detector_pipeline.py
```

### Nhiệm vụ

Điều phối toàn bộ các module core để tạo RF detection.

### Input

```text
LidarFrame raw
config
```

### Output

```text
list[RFDetection]
frame_summary: dict
rejected_clusters: list[dict]
```

### Flow nội bộ

```text
preprocess_frame
    ↓
compute_threshold
    ↓
select_bright_points
    ↓
cluster_bright_points
    ↓
validate_cluster_with_reason
    ↓
estimate_center
    ↓
compute_score
    ↓
create RFDetection
```

### Quy tắc

`DetectorPipeline` chỉ điều phối.

Không viết chi tiết thuật toán thresholding, clustering hoặc center estimation trực tiếp trong file này.

---

## 3.13. Block 12: RFDetection

### File đảm nhiệm

```text
src/rf_threshold/core/frame.py
```

hoặc file riêng nếu cần:

```text
src/rf_threshold/core/detection.py
```

### Cấu trúc đề xuất

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

### Quy tắc

`detection_id = 0` là giá trị hợp lệ.

Không bao giờ kiểm tra ID bằng boolean:

```python
# Sai
if detection_id:
    ...
```

Phải dùng:

```python
# Đúng
if detection_id is not None:
    ...
```

---

## 3.14. Block 13: ResultWriter

### File đảm nhiệm

```text
src/rf_threshold/io/result_writer.py
```

### Nhiệm vụ

Ghi output ra file.

### Output bắt buộc

```text
detections.json
detections.csv
frame_summary.csv
rejected_clusters.csv
config_used.yaml
```

### Quy tắc

Mỗi lần chạy phải lưu lại `config_used.yaml` để tái lập kết quả.

Không chỉ lưu kết quả cuối cùng.

Phải lưu summary trung gian để debug.

---

## 3.15. Block 14: Visualization

### File đảm nhiệm

```text
src/rf_threshold/visualization/plot_frame.py
```

### Nhiệm vụ

Tạo ảnh debug cho từng frame hoặc một số frame được chọn.

Ảnh nên thể hiện:

```text
Filtered points
Bright points
Valid clusters
Rejected clusters
Estimated RF centers
```

### Output

```text
data/results/<run_name>/debug_images/frame_000001.png
```

---

## 3.16. Block 15: Localization Backend

### File đảm nhiệm

```text
src/rf_threshold/localization/
```

### Trạng thái trong v1.0

Ở version `v1.0`, module localization có thể chỉ là khung chuẩn bị.

Các version sau sẽ triển khai:

```text
map_loader.py
rf_matcher.py
svd_pose.py
```

### Nguyên tắc thiết kế

Localization backend không được gọi trực tiếp các hàm thresholding.

Nó chỉ nhận:

```text
list[RFDetection]
RF map
```

Sau đó trả về:

```text
robot_pose
matching_result
pose_error nếu có ground truth
```

---

# 4. Các module cốt lõi

## 4.1. `bag_reader.py`

### Đường dẫn

```text
src/rf_threshold/io/bag_reader.py
```

### Chức năng

Đọc ROS bag và trích xuất các message thuộc topic LiDAR.

### Giao tiếp

* Nhận config từ script.
* Gọi `pointcloud_parser.py`.
* Trả về `LidarFrame`.

### Không được làm

* Không threshold.
* Không clustering.
* Không vẽ hình.
* Không ghi detection.

---

## 4.2. `pointcloud_parser.py`

### Đường dẫn

```text
src/rf_threshold/io/pointcloud_parser.py
```

### Chức năng

Chuyển ROS message thành `points_xyz` và `intensity`.

### Giao tiếp

* Được gọi bởi `bag_reader.py`.
* Trả về dữ liệu cho `LidarFrame`.

### Không được làm

* Không lọc range.
* Không lọc height.
* Không sửa intensity.
* Không tự tạo dữ liệu giả.

---

## 4.3. `frame.py`

### Đường dẫn

```text
src/rf_threshold/core/frame.py
```

### Chức năng

Định nghĩa các kiểu dữ liệu nội bộ:

```text
LidarFrame
RFCluster
RFDetection
```

### Giao tiếp

Được import bởi hầu hết các module core.

### Quy tắc

Không đặt thuật toán xử lý trong file này.

File này chỉ định nghĩa data structure.

---

## 4.4. `preprocessing.py`

### Đường dẫn

```text
src/rf_threshold/core/preprocessing.py
```

### Chức năng

Lọc point cloud trước khi threshold.

### Input

```text
LidarFrame raw
```

### Output

```text
LidarFrame filtered
summary dict
```

### Giao tiếp

Được gọi bởi `detector_pipeline.py`.

---

## 4.5. `thresholding.py`

### Đường dẫn

```text
src/rf_threshold/core/thresholding.py
```

### Chức năng

Tính threshold và chọn bright points.

### Input

```text
Filtered LidarFrame
threshold config
```

### Output

```text
Bright LidarFrame
threshold_value
```

### Giao tiếp

Được gọi bởi `detector_pipeline.py`.

---

## 4.6. `clustering.py`

### Đường dẫn

```text
src/rf_threshold/core/clustering.py
```

### Chức năng

Gom bright points thành cụm.

### Input

```text
Bright LidarFrame
clustering config
```

### Output

```text
list[RFCluster]
```

### Giao tiếp

Được gọi bởi `detector_pipeline.py`.

---

## 4.7. `cluster_validation.py`

### Đường dẫn

```text
src/rf_threshold/core/cluster_validation.py
```

### Chức năng

Kiểm tra một cụm có hợp lệ để xem là RF candidate hay không.

### Input

```text
RFCluster
validation config
```

### Output

```text
valid: bool
reason: str
features: dict
```

### Giao tiếp

Được gọi bởi `detector_pipeline.py`.

---

## 4.8. `center_estimation.py`

### Đường dẫn

```text
src/rf_threshold/core/center_estimation.py
```

### Chức năng

Ước lượng tâm RF từ cụm hợp lệ.

### Input

```text
RFCluster valid
threshold_value
center config
```

### Output

```text
center_lidar: np.ndarray
```

### Giao tiếp

Được gọi bởi `detector_pipeline.py`.

---

## 4.9. `scoring.py`

### Đường dẫn

```text
src/rf_threshold/core/scoring.py
```

### Chức năng

Tính score rule-based cho RF detection.

### Input

```text
RFCluster
cluster features
threshold_value
score config
```

### Output

```text
score: float
```

### Giao tiếp

Được gọi bởi `detector_pipeline.py`.

---

## 4.10. `detector_pipeline.py`

### Đường dẫn

```text
src/rf_threshold/core/detector_pipeline.py
```

### Chức năng

Điều phối toàn bộ pipeline phân ngưỡng.

### Class chính

```python
class ThresholdRFDetector:
    def __init__(self, cfg: dict):
        self.cfg = cfg

    def detect(self, frame: LidarFrame):
        ...
```

### Input

```text
LidarFrame
```

### Output

```text
list[RFDetection]
frame_summary
rejected_clusters
```

### Giao tiếp

Gọi các module:

```text
preprocessing.py
thresholding.py
clustering.py
cluster_validation.py
center_estimation.py
scoring.py
```

### Không được làm

Không viết lại logic chi tiết của từng module trong file pipeline.

---

## 4.11. `result_writer.py`

### Đường dẫn

```text
src/rf_threshold/io/result_writer.py
```

### Chức năng

Ghi kết quả ra file.

### Input

```text
list[RFDetection]
frame_summary
rejected_clusters
config
```

### Output

```text
detections.json
detections.csv
frame_summary.csv
rejected_clusters.csv
config_used.yaml
```

### Giao tiếp

Được gọi bởi script chạy chính.

---

## 4.12. `plot_frame.py`

### Đường dẫn

```text
src/rf_threshold/visualization/plot_frame.py
```

### Chức năng

Tạo ảnh debug.

### Input

```text
Filtered LidarFrame
Bright LidarFrame
RFCluster list
RFDetection list
```

### Output

```text
PNG debug image
```

### Không được làm

Không thực hiện thresholding hoặc clustering trong file visualization.

---

## 4.13. `map_loader.py`

### Đường dẫn

```text
src/rf_threshold/localization/map_loader.py
```

### Chức năng

Đọc bản đồ RF.

### Input

```text
JSON map file
```

### Output

```text
list[MapReflector]
```

### Trạng thái

Module này thuộc version sau `v1.1`.

---

## 4.14. `rf_matcher.py`

### Đường dẫn

```text
src/rf_threshold/localization/rf_matcher.py
```

### Chức năng

So khớp RF quan sát được với RF trong bản đồ.

### Input

```text
list[RFDetection]
list[MapReflector]
```

### Output

```text
Correspondence set
```

### Trạng thái

Module này thuộc version sau `v1.1`.

---

## 4.15. `svd_pose.py`

### Đường dẫn

```text
src/rf_threshold/localization/svd_pose.py
```

### Chức năng

Ước lượng pose robot bằng SVD từ các cặp điểm tương ứng.

### Input

```text
Observed RF points
Map RF points
Optional weights
```

### Output

```text
R
t
pose
residual
```

### Trạng thái

Module này thuộc version sau `v1.1`.

---

# 5. Quy tắc mở rộng hệ thống cho AI Agent

Phần này là hướng dẫn bắt buộc cho AI agents hoặc nhà phát triển khi muốn thêm tính năng mới.

---

## 5.1. Nguyên tắc chung

Khi thêm tính năng mới, không được sửa trực tiếp vào script chạy chính nếu tính năng đó là logic thuật toán.

Cần làm theo quy trình:

```text
1. Xác định tính năng thuộc nhóm nào
2. Tạo module đúng thư mục
3. Định nghĩa input/output rõ ràng
4. Thêm config nếu có tham số
5. Tích hợp vào pipeline
6. Thêm log/debug output nếu cần
7. Thêm unit test
8. Cập nhật README.md và ARCHITECTURE.md nếu thay đổi kiến trúc
```

---

## 5.2. Nếu thêm bước tiền xử lý mới

Ví dụ:

```text
Voxel downsampling
Ground removal
ROI polygon filter
```

Phải thêm vào:

```text
src/rf_threshold/core/preprocessing.py
```

hoặc tạo file riêng nếu đủ lớn:

```text
src/rf_threshold/core/downsampling.py
```

Sau đó gọi trong:

```text
detector_pipeline.py
```

Tham số phải thêm vào:

```text
config/threshold_v1.yaml
```

Không hard-code trong hàm.

---

## 5.3. Nếu thêm phương pháp threshold mới

Ví dụ:

```text
MAD-based threshold
Range-compensated threshold
Local contrast threshold
```

Phải thêm vào:

```text
src/rf_threshold/core/thresholding.py
```

Interface bắt buộc:

```python
def compute_threshold(intensity: np.ndarray, cfg: dict) -> float:
    ...
```

hoặc nếu cần frame đầy đủ:

```python
def compute_threshold_from_frame(frame: LidarFrame, cfg: dict) -> float:
    ...
```

Output bắt buộc:

```text
threshold_value: float
```

Không trả về nhiều format khác nhau.

---

## 5.4. Nếu thêm thuật toán clustering mới

Ví dụ:

```text
Euclidean clustering
Connected component clustering
Range-image clustering
```

Phải thêm vào:

```text
src/rf_threshold/core/clustering.py
```

Interface bắt buộc:

```python
def cluster_bright_points(frame: LidarFrame, cfg: dict) -> list[RFCluster]:
    ...
```

Dù dùng thuật toán nào, output vẫn phải là:

```text
list[RFCluster]
```

---

## 5.5. Nếu thêm điều kiện validation mới

Ví dụ:

```text
range_std
linearity
compactness
PCA eigenvalue ratio
intensity variance
```

Phải thêm vào:

```text
src/rf_threshold/core/cluster_validation.py
```

Không viết validation trong `detector_pipeline.py`.

Output validation vẫn phải giữ dạng:

```text
valid: bool
reason: str
features: dict
```

Nếu cluster bị reject, reason phải rõ ràng.

Ví dụ:

```text
range_std_too_large
compactness_too_low
linearity_too_high
```

---

## 5.6. Nếu thêm phương pháp ước lượng tâm mới

Ví dụ:

```text
RANSAC center
PCA-based center
Circle/line fitting
Robust median center
```

Phải thêm vào:

```text
src/rf_threshold/core/center_estimation.py
```

Interface bắt buộc:

```python
def estimate_center(cluster: RFCluster, threshold_value: float, cfg: dict) -> np.ndarray:
    ...
```

Output bắt buộc:

```text
np.ndarray shape (3,)
```

Không được trả về tuple tùy ý nếu pipeline chưa hỗ trợ.

---

## 5.7. Nếu thêm neural network detector sau này

Không sửa phá `ThresholdRFDetector`.

Tạo module mới:

```text
src/rf_threshold/neural/
```

Cấu trúc đề xuất:

```text
src/rf_threshold/neural/
├── __init__.py
├── model.py
├── inference.py
├── neural_detector.py
└── checkpoint_loader.py
```

Neural detector phải tuân theo interface giống threshold detector:

```python
class NeuralRFDetector:
    def __init__(self, cfg: dict):
        self.cfg = cfg

    def detect(self, frame: LidarFrame) -> list[RFDetection]:
        ...
```

Output bắt buộc:

```text
list[RFDetection]
```

Nhờ đó backend localization không cần biết detector là threshold hay neural.

---

## 5.8. Nếu thêm localization backend

Thêm vào:

```text
src/rf_threshold/localization/
```

Không để localization gọi trực tiếp thresholding hoặc clustering.

Localization chỉ nhận:

```text
list[RFDetection]
map data
```

Pipeline đúng:

```text
Detector → RFDetection → Matcher → SVD Pose
```

Pipeline sai:

```text
SVD Pose → gọi lại thresholding
Matcher → đọc trực tiếp bag
Localization → parse point cloud
```

---

## 5.9. Nếu thêm ROS node online

Không viết lại core algorithm trong node.

Tạo thư mục:

```text
src/rf_threshold/ros_nodes/
```

Ví dụ:

```text
src/rf_threshold/ros_nodes/threshold_detector_node.py
```

ROS node chỉ làm:

```text
Subscribe PointCloud2
Convert message to LidarFrame
Call ThresholdRFDetector.detect()
Publish detection message / marker
```

Core detection vẫn nằm trong:

```text
src/rf_threshold/core/
```

---

## 5.10. Nếu thêm output format mới

Ví dụ:

```text
ROS Marker
PLY
PCD
NumPy NPZ
```

Thêm vào:

```text
src/rf_threshold/io/result_writer.py
```

hoặc tạo file riêng:

```text
src/rf_threshold/io/pcd_writer.py
```

Không ghi file trực tiếp trong detector.

---

## 5.11. Nếu thêm visualization mới

Ví dụ:

```text
3D plot
RViz marker
Trajectory plot
Cluster animation
```

Thêm vào:

```text
src/rf_threshold/visualization/
```

Không đặt visualization trong core detector.

---

## 5.12. Nếu thêm test mới

Mỗi module core phải có test riêng.

Quy tắc:

```text
core/thresholding.py         → tests/test_thresholding.py
core/clustering.py           → tests/test_clustering.py
core/center_estimation.py    → tests/test_center_estimation.py
localization/svd_pose.py     → tests/test_svd_pose.py
```

Không chỉ test end-to-end. Phải có unit test cho module nhỏ.

---

# 6. Chuẩn giao tiếp giữa các module

## 6.1. Không truyền dữ liệu rời rạc nếu đã có dataclass

Không nên viết:

```python
process(points_xyz, intensity, stamp, frame_id)
```

Nên viết:

```python
process(frame: LidarFrame)
```

Lý do:

* Ít nhầm thứ tự biến.
* Dễ mở rộng.
* Dễ debug.
* Dễ đọc code.

---

## 6.2. Không thay đổi object input nếu không cần thiết

Các hàm nên trả về object mới thay vì sửa trực tiếp object đầu vào.

Ví dụ:

```python
filtered_frame = preprocess_frame(raw_frame, cfg)
```

Không nên âm thầm sửa `raw_frame` tại chỗ.

---

## 6.3. Shape convention

Mọi module phải tuân thủ:

```text
points_xyz: shape (N, 3)
intensity : shape (N,)
center    : shape (3,)
```

Nếu shape sai, raise error sớm.

---

## 6.4. Coordinate naming convention

Mọi biến tọa độ phải ghi rõ hệ tọa độ:

```text
point_lidar
center_lidar
center_base
center_map
pose_map_base
T_base_lidar
```

Không dùng tên mơ hồ:

```text
center
position
point
pose
```

trừ khi phạm vi hàm cực kỳ rõ ràng.

---

## 6.5. Empty input convention

Mọi module phải xử lý input rỗng.

Ví dụ:

```python
if frame.points_xyz.shape[0] == 0:
    return empty_result
```

Không được để lỗi như:

```text
ValueError: Found array with 0 sample(s)
```

khi gọi DBSCAN hoặc NumPy reduction.

---

# 7. Output chuẩn của hệ thống

## 7.1. Thư mục kết quả

Mỗi lần chạy nên tạo thư mục riêng:

```text
data/results/<run_name>/
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

---

## 7.2. `detections.json`

Dùng cho xử lý tiếp theo hoặc tái lập kết quả.

Format đề xuất:

```json
{
  "bag_name": "sample.bag",
  "topic": "/livox/lidar",
  "detector": "threshold",
  "frames": [
    {
      "frame_index": 0,
      "stamp": 12.0,
      "frame_id": "livox_frame",
      "threshold": 180.0,
      "detections": [
        {
          "detection_id": 0,
          "center_lidar": [1.24, 0.53, 0.02],
          "score": 0.86,
          "num_points": 12,
          "mean_intensity": 212.4,
          "max_intensity": 245.0,
          "bbox_min": [1.20, 0.49, -0.03],
          "bbox_max": [1.28, 0.57, 0.05]
        }
      ]
    }
  ]
}
```

---

## 7.3. `frame_summary.csv`

Dùng để debug số lượng điểm qua từng bước.

```csv
frame_index,stamp,raw_points,preprocessed_points,bright_points,num_clusters,num_valid,threshold
0,12.000,9821,4210,64,5,2,180.0
1,12.100,9788,4189,51,4,2,180.0
```

---

## 7.4. `rejected_clusters.csv`

Dùng để hiểu vì sao cụm bị loại.

```csv
frame_index,cluster_id,reason,num_points,extent_x,extent_y,extent_z,mean_intensity
0,3,extent_x_too_large,35,0.72,0.06,0.03,190.4
0,4,num_points_too_small,1,0.00,0.00,0.00,230.1
```

---

# 8. Nguyên tắc bảo vệ kiến trúc

Các quy tắc sau bắt buộc tuân thủ để không làm gãy hệ thống.

## 8.1. Không viết thuật toán trong script

Sai:

```text
scripts/run_threshold_bag.py chứa toàn bộ threshold, DBSCAN, center estimation
```

Đúng:

```text
scripts/run_threshold_bag.py chỉ gọi ThresholdRFDetector
```

---

## 8.2. Không để module IO xử lý thuật toán

Sai:

```text
bag_reader.py tự lọc intensity và cluster điểm
```

Đúng:

```text
bag_reader.py chỉ đọc bag và trả về LidarFrame
```

---

## 8.3. Không để visualization ảnh hưởng kết quả

Sai:

```text
plot_frame.py tính lại cluster rồi vẽ
```

Đúng:

```text
plot_frame.py nhận cluster đã có và chỉ vẽ
```

---

## 8.4. Không để localization phụ thuộc detector cụ thể

Sai:

```text
svd_pose.py gọi thresholding.py
```

Đúng:

```text
svd_pose.py nhận RFDetection list
```

---

## 8.5. Không hard-code tham số

Sai:

```python
threshold = 180
eps = 0.08
```

Đúng:

```python
threshold = cfg["threshold"]["fixed_intensity"]
eps = cfg["clustering"]["eps"]
```

---

# 9. Roadmap kiến trúc

## v1.0: Threshold RF Detection Baseline

Mục tiêu:

```text
ROS bag → RF detections
```

Thành phần cần hoàn thiện:

```text
BagReader
PointCloudParser
Preprocessor
Thresholding
Clustering
ClusterValidation
CenterEstimation
ResultWriter
Visualization
```

---

## v1.1: RF Map Matching + SVD Localization

Mục tiêu:

```text
RF detections + RF map → robot pose
```

Thành phần cần thêm:

```text
MapLoader
RFMatcher
SVDPoseEstimator
PoseResultWriter
```

---

## v1.2: Evaluation Framework

Mục tiêu:

```text
So sánh định lượng threshold baseline
```

Thành phần cần thêm:

```text
GroundTruthLoader
DetectionEvaluator
CenterErrorEvaluator
PoseErrorEvaluator
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

## v2.0: Neural Detector Integration

Mục tiêu:

```text
Neural detector thay thế threshold detector nhưng giữ nguyên backend
```

Nguyên tắc:

```text
ThresholdRFDetector.detect(frame) → list[RFDetection]
NeuralRFDetector.detect(frame)    → list[RFDetection]
```

Backend phía sau không thay đổi.

---

# 10. Tóm tắt kiến trúc

Hệ thống được tổ chức theo nguyên tắc:

```text
Read Data
  ↓
Normalize Data
  ↓
Detect Reflective Landmarks
  ↓
Export RF Observations
  ↓
Optional Localization
```

Ranh giới quan trọng nhất:

```text
Detector tạo RFDetection.
Localization sử dụng RFDetection.
Detector và localization không phụ thuộc cứng vào nhau.
```

Trong version `v1.0`, detector là:

```text
Threshold-based RF Detector
```

Trong các version sau, detector có thể là:

```text
Neural Network-based RF Detector
```

Miễn là output vẫn giữ format:

```text
list[RFDetection]
```

thì toàn bộ hệ thống phía sau vẫn hoạt động ổn định.

Mục tiêu cuối cùng của kiến trúc này là tạo ra một project:

```text
ngắn gọn
dễ debug
dễ mở rộng
dễ tái lập kết quả
phù hợp để đưa lên GitHub
phù hợp để làm baseline trong bài báo khoa học
```
