# DEBUG_LOG.md

# Nhật ký lỗi và quy trình debug hệ thống định vị robot bằng phân ngưỡng

## 0. Thông tin dự án

**Tên dự án:** Hệ thống định vị robot bằng phân ngưỡng
**Công nghệ core:** Python, ROS1 Noetic, LiDAR, NumPy
**Mục tiêu hệ thống:** Sử dụng phân ngưỡng cường độ phản xạ từ dữ liệu LiDAR để phát hiện các mốc phản xạ RF, sau đó phục vụ bài toán định vị robot.

---

## 1. Mục đích của file này

File `DEBUG_LOG.md` là nơi lưu trữ dài hạn các lỗi nghiêm trọng đã gặp trong quá trình phát triển hệ thống.

Mỗi lỗi sau khi được sửa cần được ghi lại với đầy đủ:

* Hiện tượng lỗi.
* Log lỗi.
* Nguyên nhân gốc rễ.
* File đã sửa.
* Giải pháp khắc phục.
* Bài học để không lặp lại lỗi.

File này giúp:

* Người phát triển không lặp lại lỗi cũ.
* AI agents hiểu lịch sử lỗi của hệ thống.
* Dự án duy trì được chất lượng và độ ổn định khi mở rộng.
* Quá trình debug có quy trình thay vì sửa theo cảm tính.

---

# 2. Quy trình chẩn đoán lỗi chuẩn

Khi phát hiện lỗi, không sửa code ngay lập tức. Bắt buộc thực hiện theo quy trình sau.

---

## 2.1. Bước 1: Ghi nhận chính xác hiện tượng lỗi

Cần trả lời rõ:

* Lỗi xảy ra khi chạy lệnh nào?
* Lỗi xảy ra ở frame nào?
* Lỗi xảy ra với bag nào?
* Lỗi xảy ra trong module nào?
* Có crash chương trình không?
* Output sai hay không có output?

Ví dụ cần ghi lại:

```bash
python scripts/run_threshold_bag.py \
  --config config/threshold_v1.yaml
```

Nếu có traceback, phải copy đầy đủ traceback vào mục log lỗi.

Không được mô tả mơ hồ kiểu:

```text
Code bị lỗi.
Không chạy được.
Có vẻ bị sai dữ liệu.
```

---

## 2.2. Bước 2: Kiểm tra dữ liệu đầu vào trước khi sửa code

Trước khi kết luận code sai, cần kiểm tra:

```text
File bag có tồn tại không?
Topic LiDAR có đúng không?
Message type có đúng không?
Point cloud có chứa x, y, z, intensity không?
Số điểm mỗi frame có hợp lý không?
Intensity có toàn 0 hoặc NaN không?
```

Lệnh kiểm tra bag:

```bash
rosbag info data/bags/sample.bag
```

Nếu có script kiểm tra bag:

```bash
python scripts/check_bag_info.py \
  --bag data/bags/sample.bag
```

Nếu nghi ngờ topic sai, kiểm tra danh sách topic trong bag:

```bash
rosbag info data/bags/sample.bag | less
```

---

## 2.3. Bước 3: Chạy pipeline ở chế độ DEBUG

Bật debug trong file config:

```yaml
debug:
  enabled: true
  print_every_n_frames: 1
```

Sau đó chạy lại:

```bash
python scripts/run_threshold_bag.py \
  --config config/threshold_v1.yaml
```

Cần kiểm tra log dạng:

```text
raw_points
preprocessed_points
bright_points
num_clusters
num_valid
threshold
```

Nếu một giá trị giảm về 0 bất thường, lỗi thường nằm ở bước ngay trước đó.

Ví dụ:

```text
raw_points=9821
preprocessed_points=0
bright_points=0
num_clusters=0
num_valid=0
```

Kết luận sơ bộ: lỗi có thể nằm ở `preprocessing.py`, không phải clustering.

---

## 2.4. Bước 4: Cô lập module để test riêng

Không sửa toàn bộ pipeline khi chưa biết lỗi thuộc module nào.

Quy tắc cô lập:

| Hiện tượng                  | Module cần kiểm tra trước                                    |
| --------------------------- | ------------------------------------------------------------ |
| Không đọc được bag          | `io/bag_reader.py`                                           |
| Không có intensity          | `io/pointcloud_parser.py`                                    |
| Sau lọc không còn điểm      | `core/preprocessing.py`                                      |
| Không có bright point       | `core/thresholding.py`                                       |
| DBSCAN crash                | `core/clustering.py`                                         |
| Cụm RF bị loại hết          | `core/cluster_validation.py`                                 |
| Tâm RF bị NaN               | `core/center_estimation.py`                                  |
| Không ghi được file kết quả | `io/result_writer.py`                                        |
| Pose robot sai              | `localization/rf_matcher.py` hoặc `localization/svd_pose.py` |

Chạy test liên quan:

```bash
pytest tests/test_thresholding.py
pytest tests/test_clustering.py
pytest tests/test_center_estimation.py
pytest tests/test_svd_pose.py
```

---

## 2.5. Bước 5: Kiểm tra output trung gian

Mỗi lần debug cần kiểm tra các file sau:

```text
data/results/<run_name>/frame_summary.csv
data/results/<run_name>/rejected_clusters.csv
data/results/<run_name>/detections.csv
data/results/<run_name>/detections.json
data/results/<run_name>/debug_images/
```

Ý nghĩa từng file:

| File                    | Dùng để kiểm tra                                |
| ----------------------- | ----------------------------------------------- |
| `frame_summary.csv`     | Dòng chảy số lượng điểm qua từng bước           |
| `rejected_clusters.csv` | Vì sao cluster bị loại                          |
| `detections.csv`        | Tâm RF và score có hợp lý không                 |
| `detections.json`       | Output chuẩn cho module sau                     |
| `debug_images/`         | Kiểm tra trực quan threshold, cluster và tâm RF |

---

## 2.6. Bước 6: Xác định nguyên nhân gốc rễ trước khi sửa

Không sửa theo kiểu che lỗi.

Ví dụ không được sửa như sau:

```python
try:
    clusters = cluster_bright_points(frame, cfg)
except Exception:
    clusters = []
```

Cách đúng là xác định nguyên nhân:

```text
DBSCAN crash vì input rỗng.
Module clustering chưa xử lý trường hợp bright_points = 0.
```

Sau đó sửa đúng điểm lỗi:

```python
if frame.points_xyz.shape[0] == 0:
    logger.debug("No bright points available for clustering.")
    return []
```

---

## 2.7. Bước 7: Viết hoặc cập nhật test tái hiện lỗi

Nếu lỗi thuộc logic core, phải có test tương ứng.

Ví dụ lỗi DBSCAN crash khi input rỗng:

```python
def test_clustering_returns_empty_list_for_empty_frame():
    ...
```

Lỗi chỉ được xem là đã sửa khi:

* Test cũ pass.
* Test mới tái hiện bug pass sau khi sửa.
* Pipeline chạy lại không crash.

---

## 2.8. Bước 8: Ghi lỗi vào `DEBUG_LOG.md`

Sau khi sửa thành công lỗi nghiêm trọng, phải thêm một mục mới vào phần `Bug History`.

Mục mới phải dùng đúng template ở phần 3.

---

# 3. Công cụ và lệnh debug nhanh

## 3.1. Kiểm tra ROS environment

```bash
echo $ROS_DISTRO
which python
python --version
```

Với ROS1 Noetic, nên thấy:

```text
noetic
Python 3.x
```

Source ROS nếu cần:

```bash
source /opt/ros/noetic/setup.bash
```

---

## 3.2. Kiểm tra thông tin bag

```bash
rosbag info data/bags/sample.bag
```

---

## 3.3. Play bag để kiểm tra topic

```bash
rosbag play data/bags/sample.bag
```

Mở terminal khác:

```bash
rostopic list
rostopic hz /livox/lidar
rostopic echo -n 1 /livox/lidar
```

---

## 3.4. Chạy detector

```bash
python scripts/run_threshold_bag.py \
  --config config/threshold_v1.yaml
```

---

## 3.5. Chạy test

```bash
pytest tests/
```

Chạy riêng từng test:

```bash
pytest tests/test_thresholding.py
pytest tests/test_clustering.py
pytest tests/test_center_estimation.py
```

---

## 3.6. Kiểm tra syntax Python

```bash
python -m compileall src scripts
```

---

## 3.7. Xem nhanh output CSV

```bash
head data/results/sample_run/frame_summary.csv
head data/results/sample_run/rejected_clusters.csv
head data/results/sample_run/detections.csv
```

---

## 3.8. Tìm lỗi trong log

Nếu có file log:

```bash
grep -n "ERROR" data/results/sample_run/*.log
grep -n "WARNING" data/results/sample_run/*.log
```

---

# 4. Biểu mẫu ghi nhật ký lỗi

Khi có bug mới, copy toàn bộ template dưới đây và điền đầy đủ.

---

## Template chuẩn

````markdown
## #BUG-XXX - YYYY-MM-DD - Tên ngắn của lỗi

### 1. Trạng thái

- **Status:** Open / Fixed / Monitoring
- **Severity:** Low / Medium / High / Critical
- **Module liên quan:** `src/rf_threshold/...`
- **Người phát hiện:** Tên người hoặc AI agent
- **Commit liên quan:** `commit_hash` nếu có

---

### 2. Mô tả hiện tượng

Mô tả ngắn gọn lỗi xảy ra như thế nào.

Ví dụ:

- Chương trình crash khi không có bright point.
- `detections.json` rỗng dù ảnh debug có RF.
- Tâm RF bị NaN.
- DBSCAN báo lỗi input rỗng.
- Không đọc được intensity từ PointCloud2.

Lệnh gây lỗi:

```bash
<lệnh đã chạy>
````

Log lỗi:

```text
<paste traceback hoặc log lỗi tại đây>
```

---

### 3. Điều kiện tái hiện lỗi

* File bag:
* Config:
* Topic LiDAR:
* Frame index nếu biết:
* OS / ROS:
* Python version:

Các bước tái hiện:

```text
1. ...
2. ...
3. ...
```

---

### 4. Phân tích nguyên nhân gốc rễ

Ghi rõ nguyên nhân thật sự.

Phân loại nguyên nhân:

* [ ] Logic code
* [ ] Thiếu kiểm tra input rỗng
* [ ] Sai cấu hình YAML
* [ ] Sai topic ROS
* [ ] Sai message type
* [ ] Sai hệ tọa độ
* [ ] Dữ liệu LiDAR nhiễu
* [ ] Lỗi dependency / môi trường
* [ ] Khác

Root cause:

```text
<giải thích nguyên nhân gốc rễ>
```

---

### 5. Giải pháp khắc phục

File đã sửa:

```text
src/rf_threshold/...
tests/...
config/...
```

Ý tưởng sửa:

```text
<mô tả ngắn gọn cách sửa>
```

Code trước khi sửa:

```python
<đoạn code lỗi nếu cần>
```

Code sau khi sửa:

```python
<đoạn code đã sửa>
```

---

### 6. Test xác nhận

Đã chạy:

```bash
pytest tests/...
python scripts/run_threshold_bag.py --config config/threshold_v1.yaml
```

Kết quả:

```text
<test pass / output sau khi sửa>
```

---

### 7. Bài học phòng tránh

Bài học:

```text
<ghi rõ cách tránh lỗi này trong tương lai>
```

Có cần cập nhật `CONTRIBUTING.md` hoặc `.cursorrules` không?

* [ ] Không
* [ ] Có

Nếu có, cần thêm quy tắc:

```text
<quy tắc mới>
```

````

---

# 5. Bug History

Phần này dùng để ghi các lỗi thật sự đã gặp trong dự án.

Các mục dưới đây là các lỗi dự đoán thường gặp, được ghi sẵn để làm checklist phòng tránh.

---

# 6. Common Pitfalls & Known Issues

## #BUG-PREDICT-001 - 2026-05-29 - DBSCAN crash khi không có bright point

### 1. Trạng thái

- **Status:** Known Risk
- **Severity:** High
- **Module liên quan:** `src/rf_threshold/core/clustering.py`
- **Người phát hiện:** Dự đoán QA/SRE

---

### 2. Mô tả hiện tượng

Khi threshold quá cao hoặc frame không có điểm phản xạ mạnh, danh sách bright points rỗng. Nếu vẫn gọi DBSCAN, chương trình có thể crash.

Log lỗi thường gặp:

```text
ValueError: Found array with 0 sample(s) while a minimum of 1 is required by DBSCAN.
````

---

### 3. Điều kiện tái hiện lỗi

* Config có `fixed_intensity` quá cao.
* Frame không chứa RF.
* Height filter hoặc range filter đã loại hết điểm trước threshold.

Ví dụ:

```yaml
threshold:
  mode: "fixed"
  fixed_intensity: 255.0
```

---

### 4. Phân tích nguyên nhân gốc rễ

Phân loại nguyên nhân:

* [x] Thiếu kiểm tra input rỗng
* [x] Logic code
* [ ] Sai topic ROS
* [ ] Sai message type

Root cause:

```text
Module clustering gọi DBSCAN trực tiếp mà không kiểm tra frame.points_xyz.shape[0] == 0.
```

---

### 5. Giải pháp khắc phục

File cần sửa:

```text
src/rf_threshold/core/clustering.py
```

Cách sửa:

```python
def cluster_bright_points(frame: LidarFrame, cfg: dict) -> list[RFCluster]:
    """Cluster high-reflectance points into RF candidates."""
    if frame.points_xyz.shape[0] == 0:
        logger.debug("No bright points available for clustering.")
        return []

    features = frame.points_xyz[:, :2]
    ...
```

---

### 6. Test xác nhận

Cần có test:

```text
tests/test_clustering.py
```

Test case:

```python
def test_clustering_returns_empty_list_for_empty_frame():
    ...
```

Kỳ vọng:

```text
Input empty frame → output []
No exception raised
```

---

### 7. Bài học phòng tránh

Bất kỳ module nào nhận point cloud hoặc NumPy array đều phải xử lý input rỗng trước khi gọi thuật toán bên ngoài.

Quy tắc cần nhớ:

```text
Không gọi DBSCAN, np.min, np.max, np.mean trên mảng rỗng nếu chưa kiểm tra.
```

---

## #BUG-PREDICT-002 - 2026-05-29 - Không đọc được intensity từ PointCloud2 hoặc Livox message

### 1. Trạng thái

* **Status:** Known Risk
* **Severity:** Critical
* **Module liên quan:** `src/rf_threshold/io/pointcloud_parser.py`
* **Người phát hiện:** Dự đoán QA/SRE

---

### 2. Mô tả hiện tượng

Detector không tìm thấy RF hoặc toàn bộ intensity bằng 0. Trong một số trường hợp parser báo lỗi vì không tìm thấy field `intensity`.

Log lỗi thường gặp:

```text
ValueError: Intensity field not found in point cloud message.
```

Hoặc output bất thường:

```text
intensity_min=0.0 intensity_max=0.0 intensity_mean=0.0
bright_points=0
```

---

### 3. Điều kiện tái hiện lỗi

* Topic LiDAR không phải `sensor_msgs/PointCloud2`.
* Livox dùng custom message.
* Field intensity có tên khác.
* Parser chỉ đọc `x, y, z` nhưng bỏ qua `intensity`.

Lệnh kiểm tra:

```bash
rosbag info data/bags/sample.bag
rostopic echo -n 1 /livox/lidar
```

---

### 4. Phân tích nguyên nhân gốc rễ

Phân loại nguyên nhân:

* [x] Sai message type
* [x] Logic parser chưa hỗ trợ format dữ liệu
* [x] Sai topic ROS
* [ ] Lỗi thresholding

Root cause:

```text
PointCloudParser giả định message luôn có field intensity theo chuẩn PointCloud2. Tuy nhiên dữ liệu Livox có thể dùng custom message hoặc field layout khác.
```

---

### 5. Giải pháp khắc phục

File cần sửa:

```text
src/rf_threshold/io/pointcloud_parser.py
```

Cách xử lý chuẩn:

1. In hoặc log message type.
2. Kiểm tra danh sách field nếu là PointCloud2.
3. Nếu không có intensity, raise lỗi rõ ràng.
4. Không tự tạo intensity giả.
5. Nếu là Livox custom message, viết parser riêng.

Pseudo-code:

```python
def parse_pointcloud_message(msg) -> LidarFrame:
    """Parse a ROS point cloud message into LidarFrame."""
    if is_pointcloud2_message(msg):
        return parse_pointcloud2(msg)

    if is_livox_custom_message(msg):
        return parse_livox_custom_msg(msg)

    raise TypeError(f"Unsupported point cloud message type: {type(msg)}")
```

---

### 6. Test xác nhận

Cần có test parser với:

```text
PointCloud2 có intensity
PointCloud2 thiếu intensity
Message type không hỗ trợ
```

Kỳ vọng:

```text
Thiếu intensity → raise ValueError rõ ràng
Message không hỗ trợ → raise TypeError rõ ràng
```

---

### 7. Bài học phòng tránh

Không được giả định mọi LiDAR message đều có cùng field layout.

Quy tắc cần nhớ:

```text
Parser phải xác nhận field trước khi xử lý.
Không tạo dữ liệu intensity giả.
Không để lỗi intensity lan sang thresholding.
```

---

## #BUG-PREDICT-003 - 2026-05-29 - Sau preprocessing không còn điểm nào

### 1. Trạng thái

* **Status:** Known Risk
* **Severity:** High
* **Module liên quan:** `src/rf_threshold/core/preprocessing.py`
* **Người phát hiện:** Dự đoán QA/SRE

---

### 2. Mô tả hiện tượng

Sau bước preprocessing, số điểm còn lại bằng 0. Do đó các bước thresholding, clustering và detection đều không có kết quả.

Log thường gặp:

```text
raw_points=9821
preprocessed_points=0
bright_points=0
num_clusters=0
num_valid=0
```

Không có lỗi crash, nhưng `detections.json` rỗng.

---

### 3. Điều kiện tái hiện lỗi

* `min_z`, `max_z` không phù hợp với hệ tọa độ LiDAR.
* `min_range`, `max_range` quá chặt.
* Trục z bị hiểu sai.
* LiDAR lắp nghiêng nhưng chưa tính transform.
* Dữ liệu đang ở frame khác giả định.

Ví dụ config dễ gây lỗi:

```yaml
height_filter:
  enabled: true
  min_z: 0.0
  max_z: 0.1
```

---

### 4. Phân tích nguyên nhân gốc rễ

Phân loại nguyên nhân:

* [x] Sai cấu hình YAML
* [x] Sai giả định hệ tọa độ
* [ ] Logic thresholding
* [ ] Lỗi DBSCAN

Root cause:

```text
Bộ lọc height hoặc range loại bỏ toàn bộ điểm trước khi đến bước thresholding.
```

---

### 5. Giải pháp khắc phục

File cần kiểm tra:

```text
config/threshold_v1.yaml
src/rf_threshold/core/preprocessing.py
```

Cách xử lý:

1. Log min/max của x, y, z trước khi lọc.
2. Tạm tắt height filter để kiểm tra.
3. Mở rộng khoảng z.
4. Kiểm tra frame_id và orientation của LiDAR.
5. Vẽ debug image sau preprocessing.

Log đề xuất:

```python
logger.debug(
    "Raw xyz range: x=[%.3f, %.3f], y=[%.3f, %.3f], z=[%.3f, %.3f]",
    np.min(points_xyz[:, 0]),
    np.max(points_xyz[:, 0]),
    np.min(points_xyz[:, 1]),
    np.max(points_xyz[:, 1]),
    np.min(points_xyz[:, 2]),
    np.max(points_xyz[:, 2]),
)
```

---

### 6. Test xác nhận

Cần test:

```text
Input frame có điểm trong range → giữ lại đúng điểm
Input frame có điểm ngoài range → loại đúng điểm
Input frame rỗng → không crash
```

---

### 7. Bài học phòng tránh

Khi không có detection, không kết luận ngay threshold sai. Phải kiểm tra `frame_summary.csv` để xác định điểm bị mất ở bước nào.

Quy tắc cần nhớ:

```text
Nếu preprocessed_points = 0, không debug thresholding trước.
Hãy debug preprocessing trước.
```

---

## #BUG-PREDICT-004 - 2026-05-29 - Tâm RF bị NaN do tổng trọng số bằng 0

### 1. Trạng thái

* **Status:** Known Risk
* **Severity:** High
* **Module liên quan:** `src/rf_threshold/core/center_estimation.py`
* **Người phát hiện:** Dự đoán QA/SRE

---

### 2. Mô tả hiện tượng

Khi dùng intensity-weighted centroid, tâm RF trả về NaN.

Log hoặc warning thường gặp:

```text
RuntimeWarning: invalid value encountered in divide
center_lidar=[nan, nan, nan]
```

Trong output:

```csv
frame_index,detection_id,x_lidar,y_lidar,z_lidar
0,0,nan,nan,nan
```

---

### 3. Điều kiện tái hiện lỗi

* Tất cả weight bằng 0.
* Công thức weight dùng `I_i - threshold`, nhưng `I_i == threshold`.
* Intensity bị clamp hoặc normalize sai.
* Cluster rỗng nhưng vẫn đưa vào center estimator.

Công thức gây lỗi:

```python
weights = np.maximum(intensity - threshold_value, 0.0)
center = np.sum(weights[:, None] * points_xyz, axis=0) / np.sum(weights)
```

Nếu `np.sum(weights) == 0`, phép chia tạo NaN.

---

### 4. Phân tích nguyên nhân gốc rễ

Phân loại nguyên nhân:

* [x] Thiếu kiểm tra chia cho 0
* [x] Logic center estimation
* [ ] Sai clustering
* [ ] Sai parser

Root cause:

```text
Center estimator không kiểm tra tổng trọng số trước khi chia. Khi tổng weight bằng 0, kết quả center_lidar trở thành NaN.
```

---

### 5. Giải pháp khắc phục

File cần sửa:

```text
src/rf_threshold/core/center_estimation.py
```

Cách sửa:

```python
EPSILON = 1e-8

def estimate_intensity_weighted_center(
    cluster: RFCluster,
    threshold_value: float,
    cfg: dict,
) -> np.ndarray:
    """Estimate RF center using intensity-weighted centroid."""
    points_xyz = cluster.points_xyz
    intensity = cluster.intensity

    if points_xyz.shape[0] == 0:
        raise ValueError("Cannot estimate center from an empty cluster.")

    weights = np.maximum(intensity - threshold_value, 0.0)
    weight_sum = float(np.sum(weights))

    if weight_sum <= EPSILON:
        logger.warning(
            "All intensity weights are zero. Falling back to geometric centroid."
        )
        return np.mean(points_xyz, axis=0)

    center_lidar = np.sum(weights[:, None] * points_xyz, axis=0) / weight_sum

    if not np.all(np.isfinite(center_lidar)):
        raise ValueError(f"Invalid center estimate: {center_lidar}")

    return center_lidar
```

---

### 6. Test xác nhận

Cần test:

```python
def test_weighted_center_fallback_to_centroid_when_weights_are_zero():
    ...
```

Kỳ vọng:

```text
Output không chứa NaN.
Output bằng centroid thường khi tổng weight bằng 0.
```

---

### 7. Bài học phòng tránh

Mọi phép chia trong thuật toán phải kiểm tra mẫu số.

Quy tắc cần nhớ:

```text
Không bao giờ chia cho tổng trọng số nếu chưa kiểm tra weight_sum > EPSILON.
Không bao giờ cho phép center_lidar chứa NaN đi vào output.
```

---

## #BUG-PREDICT-005 - 2026-05-29 - Lỗi bỏ qua ID bằng 0

### 1. Trạng thái

* **Status:** Known Risk
* **Severity:** Medium
* **Module liên quan:** Toàn hệ thống
* **Người phát hiện:** Dự đoán QA/SRE

---

### 2. Mô tả hiện tượng

Detection, cluster hoặc mirror có ID bằng 0 bị bỏ qua. Điều này thường làm mất RF đầu tiên hoặc landmark đầu tiên.

Code gây lỗi thường gặp:

```python
if detection_id:
    save_detection(detection_id)
```

Với `detection_id = 0`, điều kiện trên trả về `False`.

---

### 3. Điều kiện tái hiện lỗi

* `detection_id = 0`
* `cluster_id = 0`
* `mirror_id = 0`
* `frame_index = 0`

Các ID này đều hợp lệ nhưng bị coi như false trong Python.

---

### 4. Phân tích nguyên nhân gốc rễ

Phân loại nguyên nhân:

* [x] Logic code
* [x] Sai cách kiểm tra ID
* [ ] Dữ liệu lỗi

Root cause:

```text
Code dùng kiểm tra boolean cho ID số nguyên, trong khi ID bằng 0 là giá trị hợp lệ.
```

---

### 5. Giải pháp khắc phục

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

Áp dụng cho:

```text
frame_index
cluster_id
detection_id
mirror_id
landmark_id
```

---

### 6. Test xác nhận

Cần test:

```python
def test_detection_id_zero_is_valid():
    ...
```

Kỳ vọng:

```text
Detection có ID bằng 0 vẫn được lưu và xử lý.
```

---

### 7. Bài học phòng tránh

Không bao giờ kiểm tra ID bằng boolean.

Quy tắc cần nhớ:

```text
ID bằng 0 là hợp lệ.
Luôn dùng `is not None` khi kiểm tra ID tồn tại.
```

---

# 7. Hướng dẫn cập nhật file cho AI agents

> **Quy tắc bắt buộc cho AI Agent:**
> Khi bạn đồng hành cùng người dùng và sửa thành công một lỗi logic phức tạp, bạn có nhiệm vụ chủ động cập nhật hoặc đề xuất thêm một mục mới vào `DEBUG_LOG.md` theo đúng template trong file này. Không được sửa lỗi xong rồi bỏ qua bước ghi nhớ.

---

## 7.1. Khi nào AI bắt buộc cập nhật `DEBUG_LOG.md`?

AI agent phải cập nhật hoặc đề xuất cập nhật file này khi:

* Sửa lỗi crash pipeline.
* Sửa lỗi làm sai kết quả detection.
* Sửa lỗi liên quan đến hệ tọa độ.
* Sửa lỗi liên quan đến parse dữ liệu LiDAR.
* Sửa lỗi làm mất RF thật hoặc sinh RF giả bất thường.
* Sửa lỗi làm output JSON/CSV sai format.
* Sửa lỗi khiến SVD localization sai pose.
* Sửa lỗi đã tốn nhiều hơn một bước debug để tìm nguyên nhân.
* Sửa lỗi có nguy cơ lặp lại trong tương lai.

---

## 7.2. Khi nào không cần cập nhật?

Không cần cập nhật `DEBUG_LOG.md` nếu chỉ sửa:

* Typo trong README.
* Format Markdown.
* Đổi tên biến nhỏ không ảnh hưởng logic.
* Thêm comment.
* Sửa lỗi style không ảnh hưởng chạy hệ thống.

---

## 7.3. Quy trình AI phải làm sau khi sửa lỗi

Sau khi sửa lỗi, AI agent cần thực hiện:

```text
1. Tóm tắt lỗi đã sửa.
2. Xác định root cause.
3. Xác định file đã sửa.
4. Xác định test đã chạy hoặc cần chạy.
5. Tạo một mục mới theo template #BUG-XXX.
6. Đề xuất thêm mục đó vào DEBUG_LOG.md.
7. Nếu lỗi bắt nguồn từ quy tắc code chưa chặt, đề xuất cập nhật CONTRIBUTING.md hoặc .cursorrules.
```

---

## 7.4. Cách đặt ID lỗi

Quy tắc đặt ID:

```text
#BUG-001
#BUG-002
#BUG-003
...
```

Không dùng ID trùng nhau.

Nếu là lỗi dự đoán trước, dùng:

```text
#BUG-PREDICT-001
```

Nếu là lỗi thật đã gặp, dùng:

```text
#BUG-001
```

---

# 8. Bảng tổng hợp lỗi thường gặp

| ID                 | Lỗi                         | Module                 | Mức độ   | Cách phòng tránh chính                       |
| ------------------ | --------------------------- | ---------------------- | -------- | -------------------------------------------- |
| `#BUG-PREDICT-001` | DBSCAN crash khi input rỗng | `clustering.py`        | High     | Kiểm tra empty frame trước DBSCAN            |
| `#BUG-PREDICT-002` | Không đọc được intensity    | `pointcloud_parser.py` | Critical | Kiểm tra field và message type               |
| `#BUG-PREDICT-003` | Preprocessing loại hết điểm | `preprocessing.py`     | High     | Kiểm tra `frame_summary.csv` và min/max xyz  |
| `#BUG-PREDICT-004` | Tâm RF bị NaN               | `center_estimation.py` | High     | Kiểm tra tổng trọng số trước khi chia        |
| `#BUG-PREDICT-005` | Bỏ qua ID bằng 0            | Toàn hệ thống          | Medium   | Dùng `is not None`, không dùng boolean check |

---

# 9. Checklist debug nhanh

Khi hệ thống lỗi, kiểm tra theo thứ tự:

* [ ] Lệnh chạy đã được ghi lại chưa?
* [ ] Traceback đã được copy đầy đủ chưa?
* [ ] Đã kiểm tra `rosbag info` chưa?
* [ ] Topic LiDAR có đúng không?
* [ ] Message có intensity không?
* [ ] `frame_summary.csv` cho thấy lỗi bắt đầu ở bước nào?
* [ ] `rejected_clusters.csv` có lý do reject rõ không?
* [ ] Có ảnh debug để xem trực quan không?
* [ ] Đã chạy test module liên quan chưa?
* [ ] Đã xác định root cause trước khi sửa chưa?
* [ ] Đã thêm test để tránh lỗi lặp lại chưa?
* [ ] Đã cập nhật `DEBUG_LOG.md` chưa?

---

# 10. Nguyên tắc cuối cùng

Không sửa lỗi bằng cách che lỗi.

Không sửa lỗi bằng cách bỏ qua exception.

Không sửa lỗi bằng cách thay đổi output format tùy tiện.

Không sửa lỗi bằng cách viết lại toàn bộ pipeline khi chỉ một module sai.

Mọi lỗi nghiêm trọng phải để lại dấu vết trong `DEBUG_LOG.md`.

Mục tiêu của file này là biến mỗi lỗi đã gặp thành tri thức tái sử dụng, giúp hệ thống ngày càng ổn định và giúp AI agents không lặp lại cùng một sai lầm trong tương lai.
