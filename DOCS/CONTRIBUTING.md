# CONTRIBUTING.md

# Quy chuẩn đóng góp cho dự án hệ thống định vị robot bằng phân ngưỡng

## 0. Thông tin dự án

**Tên dự án:** Hệ thống định vị robot bằng phân ngưỡng
**Công nghệ core:** Python, ROS1 Noetic, LiDAR, NumPy
**Phong cách dự án:** Code tường minh, dễ debug, dễ bảo trì, ưu tiên tính ổn định và khả năng tái lập kết quả hơn là viết code ngắn nhưng khó hiểu.

Dự án này xử lý dữ liệu LiDAR để phát hiện các mốc phản xạ `RF - Reflective Feature / Reflective Landmark` bằng phương pháp phân ngưỡng cường độ phản xạ, sau đó phục vụ các bước định vị robot.

Mục tiêu của file này là thiết lập tiêu chuẩn bắt buộc cho:

* Người phát triển.
* AI coding agents.
* Cursor.
* GitHub Copilot.
* Các công cụ tự động sinh code.

Mọi thay đổi code phải tuân thủ tài liệu này và không được làm gãy kiến trúc đã mô tả trong `ARCHITECTURE.md`.

---

# 1. Quy chuẩn code

## 1.1. Nguyên tắc chung

Code trong dự án phải tuân thủ các nguyên tắc sau:

* **Rõ ràng hơn ngắn gọn.**
* **Dễ debug hơn thông minh quá mức.**
* **Không viết logic quan trọng trong script chạy chính.**
* **Không hard-code tham số thuật toán.**
* **Không phá interface hiện có nếu không được yêu cầu.**
* **Mọi module phải có input/output rõ ràng.**
* **Mọi thuật toán phải có test tối thiểu.**

Không chấp nhận kiểu code:

```python
# Không chấp nhận
def process(data):
    ...
```

Ưu tiên kiểu code:

```python
def preprocess_frame(frame: LidarFrame, cfg: dict) -> tuple[LidarFrame, dict]:
    ...
```

---

## 1.2. Quy tắc đặt tên trong Python

Dự án sử dụng chuẩn đặt tên theo PEP8.

## 1.2.1. Tên file

Dùng `snake_case`.

Đúng:

```text
bag_reader.py
pointcloud_parser.py
center_estimation.py
cluster_validation.py
detector_pipeline.py
```

Sai:

```text
BagReader.py
pointCloudParser.py
centerEstimation.py
DetectorPipeline.py
```

---

## 1.2.2. Tên biến

Dùng `snake_case`.

Đúng:

```python
points_xyz
center_lidar
threshold_value
frame_summary
valid_clusters
```

Sai:

```python
pointsXYZ
center
ThresholdValue
FrameSummary
validClusters
```

---

## 1.2.3. Tên hàm

Dùng `snake_case`, bắt đầu bằng động từ nếu hàm thực hiện hành động.

Đúng:

```python
read_lidar_frames()
parse_pointcloud_message()
preprocess_frame()
compute_threshold()
select_bright_points()
cluster_bright_points()
validate_cluster_with_reason()
estimate_center()
write_detections_json()
```

Sai:

```python
lidarFrames()
pointcloudParser()
threshold()
cluster()
center()
```

---

## 1.2.4. Tên class

Dùng `PascalCase`.

Đúng:

```python
LidarFrame
RFCluster
RFDetection
ThresholdRFDetector
SVDPoseEstimator
```

Sai:

```python
lidarFrame
rf_cluster
rfDetection
threshold_detector
```

---

## 1.2.5. Tên hằng số

Dùng `UPPER_SNAKE_CASE`.

Đúng:

```python
DEFAULT_MIN_RANGE = 0.2
DEFAULT_MAX_RANGE = 8.0
EPSILON = 1e-8
```

Tuy nhiên, các tham số thuật toán chính vẫn phải ưu tiên lấy từ file YAML, không lạm dụng hằng số trong code.

---

## 1.3. Quy tắc đặt tên biến liên quan đến hệ tọa độ

Bất kỳ biến nào biểu diễn tọa độ, điểm, tâm, pose hoặc transform phải ghi rõ hệ tọa độ.

Đúng:

```python
point_lidar
center_lidar
center_base
center_map
pose_map_base
T_base_lidar
T_map_base
```

Sai:

```python
point
center
position
pose
T
```

Lý do: dự án liên quan đến LiDAR, robot và bản đồ. Nếu không ghi rõ frame tọa độ, lỗi sẽ rất khó debug.

---

## 1.4. Quy tắc định dạng code

## 1.4.1. Indentation

* Dùng **4 spaces**.
* Không dùng tab.

Đúng:

```python
def compute_threshold(intensity: np.ndarray, cfg: dict) -> float:
    if intensity.size == 0:
        return 0.0
```

Sai:

```python
def compute_threshold(intensity, cfg):
	if len(intensity) == 0:
		return 0
```

---

## 1.4.2. Độ dài dòng

* Giới hạn khuyến nghị: **88 ký tự**.
* Nếu dòng quá dài, phải xuống dòng rõ ràng.

Đúng:

```python
filtered_frame, frame_summary = preprocess_frame(
    frame=raw_frame,
    cfg=cfg["preprocessing"],
)
```

Sai:

```python
filtered_frame, frame_summary = preprocess_frame(frame=raw_frame, cfg=cfg["preprocessing"])
```

---

## 1.4.3. Import

Thứ tự import:

```python
# 1. Standard library
import json
import logging
from dataclasses import dataclass
from pathlib import Path

# 2. Third-party libraries
import numpy as np
from sklearn.cluster import DBSCAN

# 3. Local modules
from rf_threshold.core.frame import LidarFrame, RFCluster
```

Không dùng wildcard import.

Sai:

```python
from utils import *
```

Đúng:

```python
from rf_threshold.utils.geometry import compute_range
```

---

## 1.5. Type hint bắt buộc

Mọi hàm public trong module phải có type hint.

Đúng:

```python
def compute_range(points_xyz: np.ndarray) -> np.ndarray:
    ...
```

Sai:

```python
def compute_range(points_xyz):
    ...
```

Với hàm trả nhiều giá trị:

```python
def preprocess_frame(frame: LidarFrame, cfg: dict) -> tuple[LidarFrame, dict]:
    ...
```

---

## 1.6. Docstring bắt buộc

Mọi hàm public, class public và module quan trọng phải có docstring.

Chuẩn docstring dùng format:

```python
def compute_threshold(intensity: np.ndarray, cfg: dict) -> float:
    """Compute the intensity threshold for one LiDAR frame.

    Args:
        intensity: Intensity array with shape (N,).
        cfg: Threshold configuration loaded from YAML.

    Returns:
        Threshold value used to select high-reflectance points.

    Raises:
        ValueError: If the configuration is invalid.
    """
```

Docstring phải làm rõ:

* Hàm làm gì.
* Input là gì.
* Shape dữ liệu nếu có NumPy array.
* Output là gì.
* Có raise exception nào không.

---

## 1.7. Comment trong code

Comment chỉ dùng để giải thích **tại sao**, không dùng để lặp lại **code đang làm gì**.

Không nên:

```python
# Add 1 to i
i = i + 1
```

Nên:

```python
# Fallback to centroid when all intensity weights are zero to avoid NaN.
if weight_sum <= EPSILON:
    return np.mean(cluster.points_xyz, axis=0)
```

Comment nên ngắn gọn, kỹ thuật, không viết lan man.

---

# 2. Quy trình xử lý lỗi và logging

## 2.1. Nguyên tắc xử lý lỗi

Không được bỏ qua lỗi tiềm ẩn. Mọi lỗi phải được xử lý theo một trong ba cách:

1. **Raise exception rõ ràng** nếu lỗi làm hệ thống không thể tiếp tục.
2. **Log warning và fallback an toàn** nếu có phương án thay thế.
3. **Trả về output rỗng hợp lệ** nếu input rỗng là tình huống bình thường.

---

## 2.2. Cấm tuyệt đối `try: pass`

Không bao giờ viết:

```python
try:
    ...
except:
    pass
```

Không bao giờ viết:

```python
try:
    ...
except Exception:
    pass
```

Nếu bắt lỗi, phải chỉ rõ lỗi và log đầy đủ.

Đúng:

```python
try:
    points_xyz, intensity = parse_pointcloud_message(msg)
except ValueError as exc:
    logger.error("Failed to parse point cloud message: %s", exc)
    raise
```

---

## 2.3. Không bắt lỗi chung chung nếu không cần thiết

Sai:

```python
try:
    threshold = compute_threshold(intensity, cfg)
except Exception:
    threshold = 0
```

Đúng:

```python
try:
    threshold = compute_threshold(intensity, cfg)
except KeyError as exc:
    logger.error("Missing threshold configuration key: %s", exc)
    raise
except ValueError as exc:
    logger.error("Invalid threshold configuration: %s", exc)
    raise
```

---

## 2.4. Trường hợp input rỗng

Input rỗng không phải lúc nào cũng là lỗi.

Ví dụ: frame không có bright point sau threshold là tình huống hợp lệ.

Hàm clustering phải xử lý:

```python
if frame.points_xyz.shape[0] == 0:
    logger.debug("No bright points available for clustering.")
    return []
```

Không được gọi DBSCAN trực tiếp với mảng rỗng.

---

## 2.5. Quy chuẩn logging

Dự án sử dụng thư viện `logging` của Python.

Không dùng `print()` trong module core.

Sai:

```python
print("threshold =", threshold)
```

Đúng:

```python
logger.info("threshold=%.3f", threshold)
```

`print()` chỉ được phép dùng trong script rất nhỏ để hiển thị CLI help hoặc kết quả kiểm tra môi trường. Trong module chính, luôn dùng `logger`.

---

## 2.6. Định dạng log chuẩn

Log nên có format:

```text
[2026-05-29 10:30:12] [INFO] [thresholding] threshold=180.000 bright_points=64
```

Format khuyến nghị:

```python
LOG_FORMAT = (
    "[%(asctime)s] "
    "[%(levelname)s] "
    "[%(name)s] "
    "%(message)s"
)
```

---

## 2.7. Mức logging

Sử dụng đúng mức log:

| Level      | Khi nào dùng                                          |
| ---------- | ----------------------------------------------------- |
| `DEBUG`    | Thông tin chi tiết khi debug từng frame hoặc từng cụm |
| `INFO`     | Tiến trình chính của pipeline                         |
| `WARNING`  | Có bất thường nhưng hệ thống vẫn có thể chạy          |
| `ERROR`    | Lỗi khiến frame/module không xử lý được               |
| `CRITICAL` | Lỗi nghiêm trọng khiến toàn bộ chương trình phải dừng |

Ví dụ:

```python
logger.debug("Cluster %d features: %s", cluster.cluster_id, features)
logger.info("Frame %d: valid_detections=%d", frame_index, len(detections))
logger.warning("No bright points found in frame %d.", frame_index)
logger.error("Failed to read bag file: %s", bag_path)
```

---

## 2.8. Log bắt buộc cho mỗi frame

Pipeline chính phải log hoặc lưu summary dạng:

```text
raw_points=9821
preprocessed_points=4210
bright_points=64
num_clusters=5
num_valid=2
threshold=180.0
```

Thông tin này phải được lưu vào:

```text
frame_summary.csv
```

---

## 2.9. Lỗi cấu hình

Nếu file YAML thiếu key quan trọng, phải báo lỗi rõ ràng.

Không dùng default âm thầm cho tham số thuật toán quan trọng như:

* `fixed_intensity`
* `eps`
* `min_samples`
* `min_range`
* `max_range`
* `min_points`
* `max_points`

Ví dụ đúng:

```python
if "fixed_intensity" not in cfg:
    raise KeyError("Missing required config: threshold.fixed_intensity")
```

---

# 3. Những điều cấm kỵ và hạn chế

## 3.1. Không dùng magic numbers

Sai:

```python
bright_mask = intensity > 180
dbscan = DBSCAN(eps=0.08, min_samples=3)
```

Đúng:

```python
threshold = cfg["threshold"]["fixed_intensity"]
eps = cfg["clustering"]["eps"]
min_samples = cfg["clustering"]["min_samples"]

bright_mask = intensity > threshold
dbscan = DBSCAN(eps=eps, min_samples=min_samples)
```

---

## 3.2. Không dùng biến global cho trạng thái thuật toán

Sai:

```python
CURRENT_FRAME = None
DETECTIONS = []
```

Đúng:

```python
detections = detector.detect(frame)
```

Chỉ được dùng hằng số global cho giá trị bất biến rõ nghĩa:

```python
EPSILON = 1e-8
```

---

## 3.3. Không viết hàm quá dài

Quy định:

* Hàm thông thường: tối đa khoảng **50 dòng**.
* Nếu hàm dài hơn, phải tách thành hàm nhỏ.
* Hàm pipeline có thể dài hơn một chút nhưng chỉ được điều phối, không chứa logic chi tiết.

Sai:

```python
def run_all():
    # đọc bag
    # parse
    # preprocess
    # threshold
    # cluster
    # validate
    # estimate center
    # write result
    # plot
    ...
```

Đúng:

```python
def detect(self, frame: LidarFrame) -> DetectionResult:
    filtered_frame, summary = preprocess_frame(frame, self.cfg)
    bright_frame, threshold = select_bright_points(filtered_frame, self.cfg)
    clusters = cluster_bright_points(bright_frame, self.cfg)
    ...
```

---

## 3.4. Không viết thuật toán chính trong `scripts/`

`scripts/` chỉ chứa entry-point.

Sai:

```text
scripts/run_threshold_bag.py chứa toàn bộ thuật toán threshold và DBSCAN
```

Đúng:

```text
scripts/run_threshold_bag.py gọi ThresholdRFDetector từ src/rf_threshold/core/
```

---

## 3.5. Không tự ý thêm thư viện ngoài

AI agent hoặc lập trình viên không được tự ý thêm dependency mới vào `requirements.txt`.

Nếu cần thêm thư viện, phải giải thích:

* Thư viện dùng để làm gì.
* Có thể thay bằng thư viện hiện có không.
* Ảnh hưởng đến cài đặt ROS1 Noetic không.
* Có làm project nặng hơn không.

Chỉ thêm dependency khi thật cần thiết.

---

## 3.6. Không phá interface hiện có

Không được đổi chữ ký hàm public nếu không được yêu cầu.

Ví dụ không tự ý đổi:

```python
def detect(self, frame: LidarFrame) -> list[RFDetection]:
    ...
```

thành:

```python
def detect(self, points, intensity, timestamp):
    ...
```

Nếu cần mở rộng output, hãy tạo dataclass mới hoặc thêm wrapper nhưng phải giữ backward compatibility nếu có thể.

---

## 3.7. Không trộn detector và localization

Detector chỉ tạo RF observations.

Localization chỉ dùng RF observations để định vị.

Cấm kiểu:

```python
# Sai
thresholding.py gọi SVD localization
```

Đúng:

```text
thresholding.py → RFDetection
RFDetection → localization/svd_pose.py
```

---

## 3.8. Không trộn visualization với thuật toán

Visualization chỉ vẽ dữ liệu đã được tạo ra.

Cấm kiểu:

```python
# Sai
plot_frame.py tự chạy DBSCAN lại để vẽ
```

Đúng:

```python
plot_frame(
    filtered_frame=filtered_frame,
    bright_frame=bright_frame,
    clusters=clusters,
    detections=detections,
)
```

---

## 3.9. Không âm thầm sửa dữ liệu đầu vào

Không nên sửa object input tại chỗ nếu không cần thiết.

Sai:

```python
def preprocess_frame(frame):
    frame.points_xyz = frame.points_xyz[mask]
    return frame
```

Đúng:

```python
def preprocess_frame(frame: LidarFrame, cfg: dict) -> LidarFrame:
    return LidarFrame(
        stamp=frame.stamp,
        frame_id=frame.frame_id,
        points_xyz=frame.points_xyz[mask],
        intensity=frame.intensity[mask],
    )
```

---

## 3.10. Không kiểm tra ID bằng boolean

Trong dự án, ID bằng `0` là hợp lệ.

Sai:

```python
if detection_id:
    ...
```

Đúng:

```python
if detection_id is not None:
    ...
```

Quy tắc này áp dụng cho:

* `frame_index`
* `cluster_id`
* `detection_id`
* `mirror_id`
* `landmark_id`

---

## 3.11. Không để lỗi shape NumPy trôi qua

Mọi hàm nhận `points_xyz` và `intensity` phải kiểm tra shape nếu đó là hàm public.

Yêu cầu:

```text
points_xyz.shape == (N, 3)
intensity.shape == (N,)
```

Nếu sai, raise `ValueError`.

---

## 3.12. Không dùng tên biến mơ hồ

Cấm dùng trong phạm vi lớn:

```python
data
result
output
temp
arr
p
c
```

Trừ khi phạm vi rất ngắn và rõ nghĩa.

Ưu tiên:

```python
filtered_frame
bright_points_xyz
cluster_features
center_lidar
detection_score
```

---

# 4. Đảm bảo chất lượng code và testing

## 4.1. Yêu cầu trước khi bàn giao code

Trước khi bàn giao một module mới, bắt buộc phải có:

* Code nằm đúng thư mục theo `ARCHITECTURE.md`.
* Type hint đầy đủ.
* Docstring cho hàm public.
* Không hard-code tham số thuật toán.
* Có xử lý input rỗng.
* Có logging phù hợp.
* Có unit test tối thiểu.
* Không phá interface hiện tại.
* Chạy được import module mà không lỗi.

---

## 4.2. Unit test bắt buộc cho module core

Mỗi module core phải có test tương ứng.

| Module                 | Test file                         |
| ---------------------- | --------------------------------- |
| `thresholding.py`      | `tests/test_thresholding.py`      |
| `clustering.py`        | `tests/test_clustering.py`        |
| `center_estimation.py` | `tests/test_center_estimation.py` |
| `svd_pose.py`          | `tests/test_svd_pose.py`          |

---

## 4.3. Test cho `thresholding.py`

Cần kiểm tra tối thiểu:

* Fixed threshold hoạt động đúng.
* Adaptive threshold trả về giá trị hợp lệ.
* Input intensity rỗng không crash.
* Input toàn giá trị giống nhau không crash.
* Threshold không trả về NaN.

Ví dụ test case:

```python
def test_fixed_threshold_selects_correct_points():
    ...
```

---

## 4.4. Test cho `clustering.py`

Cần kiểm tra tối thiểu:

* Input rỗng trả về `[]`.
* Một cụm đơn giản được phát hiện đúng.
* Hai cụm tách xa được tách thành hai cluster.
* Nhiễu lẻ bị loại nếu DBSCAN đánh dấu noise.

---

## 4.5. Test cho `center_estimation.py`

Cần kiểm tra tối thiểu:

* Centroid thường đúng.
* Intensity-weighted centroid đúng.
* Tổng trọng số bằng 0 thì fallback về centroid.
* Output shape là `(3,)`.
* Output không chứa NaN.

---

## 4.6. Test cho `svd_pose.py`

Khi triển khai localization, cần test:

* Tạo tập điểm map.
* Apply transform giả.
* SVD khôi phục lại transform.
* Sai số tịnh tiến gần 0.
* Sai số góc gần 0.
* Trường hợp thiếu số điểm tối thiểu phải raise lỗi rõ ràng.

---

## 4.7. Test end-to-end tối thiểu

Ngoài unit test, cần có test nhỏ chạy qua pipeline:

```text
Synthetic LidarFrame
    ↓
Preprocessing
    ↓
Thresholding
    ↓
Clustering
    ↓
Validation
    ↓
Center estimation
    ↓
RFDetection
```

Test này không cần ROS bag thật. Nên dùng dữ liệu giả nhỏ để chạy nhanh.

---

## 4.8. Kiểm tra hiệu năng

Vì dự án xử lý point cloud, cần tránh các lỗi hiệu năng cơ bản:

* Không dùng vòng lặp Python nếu có thể dùng NumPy vectorization.
* Không copy mảng lớn quá nhiều lần nếu không cần thiết.
* Không vẽ debug image cho mọi frame nếu bag quá dài, trừ khi debug mode bật.
* Không lưu point cloud full vào JSON detection.

Trong v1.0, ưu tiên code rõ ràng. Tuy nhiên, không được viết code quá chậm một cách không cần thiết.

---

## 4.9. Lệnh kiểm tra trước khi commit

Trước khi commit, chạy:

```bash
pytest tests/
```

Nếu có formatter/linter:

```bash
python -m compileall src scripts
```

Tối thiểu phải đảm bảo:

* Không lỗi syntax.
* Unit test pass.
* Không import lỗi.

---

# 5. Quy trình làm việc với Git

## 5.1. Commit nhỏ theo từng thay đổi

Không gom quá nhiều thay đổi vào một commit.

Ví dụ commit tốt:

```text
Add LidarFrame dataclass
Implement fixed intensity thresholding
Add DBSCAN clustering module
Add center estimation tests
```

Ví dụ commit xấu:

```text
update code
fix
final
new version
```

---

## 5.2. Không commit dữ liệu lớn

Không commit:

```text
*.bag
data/bags/
data/results/
debug_images/
```

Trừ khi là sample rất nhỏ và có lý do rõ ràng.

---

## 5.3. Nội dung commit message

Format khuyến nghị:

```text
<type>: <short description>
```

Ví dụ:

```text
feat: add thresholding module
fix: handle empty point cloud before DBSCAN
test: add center estimation unit tests
docs: update architecture guidelines
refactor: split cluster validation logic
```

---

# 6. Quy chuẩn cấu hình

## 6.1. Tất cả tham số thuật toán phải nằm trong YAML

Ví dụ:

```yaml
threshold:
  mode: "fixed"
  fixed_intensity: 180.0

clustering:
  method: "dbscan"
  eps: 0.08
  min_samples: 3
```

Không hard-code trong code.

---

## 6.2. Lưu lại config đã dùng

Mỗi lần chạy pipeline phải copy config vào thư mục kết quả:

```text
config_used.yaml
```

Mục đích:

* Tái lập kết quả.
* Debug.
* Viết bài báo.
* So sánh các lần chạy.

---

## 6.3. Không tự đổi ý nghĩa tham số

Nếu đã có key:

```yaml
fixed_intensity: 180.0
```

thì không được dùng nó cho mục đích khác.

Nếu cần tham số mới, thêm key mới với tên rõ ràng.

---

# 7. Quy chuẩn output

## 7.1. Output bắt buộc

Mỗi lần chạy detector nên tạo:

```text
detections.json
detections.csv
frame_summary.csv
rejected_clusters.csv
config_used.yaml
```

Nếu bật debug:

```text
debug_images/
```

---

## 7.2. Không ghi output lung tung

Tất cả output phải nằm trong:

```text
data/results/<run_name>/
```

Không ghi file trực tiếp vào thư mục gốc project.

---

## 7.3. Không lưu dữ liệu quá nặng vào JSON

`detections.json` chỉ lưu thông tin detection đã chuẩn hóa.

Không lưu toàn bộ point cloud raw vào JSON.

Nếu cần debug point cloud, dùng định dạng riêng như `.npz`, `.pcd`, `.ply` trong version sau.

---

# 8. Hướng dẫn prompting cho AI agents

## 8.1. Quy tắc bắt buộc khi AI nhận yêu cầu viết code

Khi bạn là AI agent nhận yêu cầu viết code cho dự án này, bắt buộc làm theo thứ tự:

1. **Đọc kỹ `ARCHITECTURE.md` trước khi quyết định tạo hoặc sửa file.**
2. **Xác định module thuộc thư mục nào: `io/`, `core/`, `localization/`, `visualization/`, `utils/`, hay `scripts/`.**
3. **Không tự ý thay đổi API hoặc interface hiện tại nếu người dùng không yêu cầu.**
4. **Không viết thuật toán chính trong `scripts/`.**
5. **Không hard-code tham số thuật toán.**
6. **Nếu cần thêm thư viện mới, phải giải thích lý do trước khi viết code.**
7. **Nếu thêm module core mới, phải thêm unit test tương ứng.**
8. **Nếu sửa logic xử lý dữ liệu, phải đảm bảo input rỗng không làm crash.**
9. **Nếu thêm output mới, phải cập nhật `result_writer.py` hoặc module IO tương ứng.**
10. **Nếu thay đổi pipeline, phải cập nhật `ARCHITECTURE.md`.**

---

## 8.2. Khi AI không chắc nên đặt code ở đâu

Không được đoán bừa.

Phải dựa theo nguyên tắc:

```text
Đọc/ghi dữ liệu     → src/rf_threshold/io/
Thuật toán detector → src/rf_threshold/core/
Định vị robot       → src/rf_threshold/localization/
Vẽ hình/debug       → src/rf_threshold/visualization/
Hàm phụ trợ chung   → src/rf_threshold/utils/
Script terminal     → scripts/
Test                → tests/
```

---

## 8.3. Khi AI cần thêm một hàm mới

Hàm mới phải có:

* Tên rõ ràng bằng `snake_case`.
* Type hint.
* Docstring.
* Xử lý input rỗng nếu liên quan đến dữ liệu mảng.
* Log nếu có khả năng lỗi hoặc fallback.
* Test nếu là logic core.

---

## 8.4. Khi AI cần sửa bug

Quy trình sửa bug bắt buộc:

1. Xác định bug thuộc module nào.
2. Viết hoặc cập nhật test tái hiện bug nếu có thể.
3. Sửa đúng module đó, không sửa lan sang module khác.
4. Đảm bảo không phá interface.
5. Chạy test liên quan.
6. Mô tả ngắn gọn nguyên nhân và cách sửa.

Không được sửa bug bằng cách che lỗi.

Sai:

```python
try:
    ...
except Exception:
    return []
```

Đúng:

```python
if frame.points_xyz.shape[0] == 0:
    logger.debug("Empty frame received. Returning no detections.")
    return []
```

---

## 8.5. Khi AI cần tối ưu hiệu năng

Không tối ưu mù.

Trước khi tối ưu, cần xác định:

* Bottleneck nằm ở bước nào.
* Kích thước point cloud trung bình.
* Số frame cần xử lý.
* Debug mode có đang bật không.
* Có thể dùng NumPy vectorization không.

Không được hy sinh tính đúng đắn để đổi lấy tốc độ.

---

# 9. Checklist cho Pull Request hoặc module mới

Trước khi merge hoặc bàn giao code, kiểm tra:

* [ ] Code nằm đúng thư mục theo `ARCHITECTURE.md`.
* [ ] Tên file, hàm, class đúng convention.
* [ ] Không có magic numbers.
* [ ] Không có `try: pass`.
* [ ] Không có `except Exception` chung chung nếu không có lý do rõ ràng.
* [ ] Không dùng `print()` trong module core.
* [ ] Có type hint.
* [ ] Có docstring cho hàm public.
* [ ] Có xử lý input rỗng.
* [ ] Có logging phù hợp.
* [ ] Có unit test cho logic mới.
* [ ] Không phá interface hiện có.
* [ ] Không commit file `.bag` lớn.
* [ ] Không ghi output ngoài `data/results/`.
* [ ] Nếu thêm config mới, đã cập nhật YAML mẫu.
* [ ] Nếu thay đổi kiến trúc, đã cập nhật `ARCHITECTURE.md`.

---

# 10. Tóm tắt quy tắc quan trọng nhất

Nếu chỉ nhớ một số quy tắc, hãy nhớ các quy tắc sau:

1. **Đọc `ARCHITECTURE.md` trước khi code.**
2. **Không viết thuật toán chính trong `scripts/`.**
3. **Không hard-code tham số thuật toán.**
4. **Detector chỉ tạo `RFDetection`; localization chỉ dùng `RFDetection`.**
5. **Không trộn IO, thuật toán, visualization và localization.**
6. **Mọi hàm public phải có type hint và docstring.**
7. **Mọi module core phải xử lý input rỗng.**
8. **Không dùng `try: pass`.**
9. **Không dùng ID như boolean vì ID bằng 0 là hợp lệ.**
10. **Code phải dễ debug trước khi tối ưu hiệu năng.**

Mục tiêu cuối cùng của dự án là xây dựng một baseline phân ngưỡng sạch, ổn định và có thể tái lập để phục vụ nghiên cứu định vị robot và so sánh với phương pháp mạng nơ-ron trong các phiên bản sau.
