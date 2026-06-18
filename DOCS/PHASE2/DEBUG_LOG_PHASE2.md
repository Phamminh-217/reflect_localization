# DEBUG_LOG_PHASE2.md

# Nhật ký lỗi và debug cho Phase 2 — SVD-Based RF Localization

## 1. Mục đích

File này lưu lại các lỗi nghiêm trọng trong Phase 2.

Phase 2 có nguy cơ sinh lỗi “nguy hiểm” hơn Phase 1 vì hệ thống có thể:

```text
không crash
nhưng pose robot sai
```

Do đó mọi lỗi liên quan đến:

```text
data association
SVD
hệ tọa độ
RF map
```

## 2. Phase 2.6 Debugging Workflow — Khi pose robot sai

Khi pose robot bị sai lệch hoặc định vị thất bại trên dữ liệu thực tế (bag thật), các kỹ sư bắt buộc phải tiến hành phân tích, chẩn đoán lỗi theo quy trình tuần tự 8 bước sau đây:

```text
1. Kiểm tra detections input (Bước 1)
2. Kiểm tra RF map (Bước 2)
3. Kiểm tra association_debug.csv (Bước 3)
4. Kiểm tra geometry_debug.csv (Bước 4)
5. Kiểm tra svd_debug.csv (Bước 5)
6. Kiểm tra transform convention (Bước 6)
7. Kiểm tra frame robot: lidar_frame vs base_link (Bước 7)
8. Kiểm tra trajectory-level consistency (Bước 8)
```

---

### Bước 1 — Kiểm tra Detections Input

* **Tệp tin kiểm tra**: `detections.json`, `frame_debug.csv`
* **Câu hỏi chẩn đoán**:
  * Khung hình đó có thu nhận đủ $\ge 3$ mốc phản quang RF hợp lệ không?
  * Giá trị tọa độ tâm `center_lidar` có chứa giá trị không hợp lệ (`NaN`, `Inf`) không?
  * Cường độ phản xạ có bị suy giảm hoặc có mốc nào bị phát hiện lệch khoảng cách quá xa bất thường so với thực tế không?
* **Quy tắc**: Nếu khâu phát hiện (Phase 1) trả về kết quả sai hoặc thiếu, tuyệt đối không được debug SVD hay Association trước. Phải sửa lỗi từ detector trước.

---

### Bước 2 — Kiểm tra Bản đồ RF Map

* **Tệp tin kiểm tra**: `data/maps/rf_map_v1.json`
* **Câu hỏi chẩn đoán**:
  * Tệp bản đồ nạp vào có đúng phiên bản cấu hình không?
  * Đơn vị đo tọa độ của các mốc đã được chuẩn hóa về `meter` chưa?
  * Gốc tọa độ bản đồ (map origin) có bị dịch chuyển hay đặt sai lệch không?
  * Trục tọa độ $x/y$ của bản đồ có bị đảo ngược hay xoay góc lệch không?
  * Cao độ $z$ của các mốc có nằm trong khoảng lọc `height_filter` dự kiến không?

---

### Bước 3 — Kiểm tra Data Association (So Khớp)

* **Tệp tin kiểm tra**: `association_debug.csv`
* **Câu hỏi chẩn đoán**:
  * Mốc quan sát LiDAR nào đang được ghép cặp với landmark nào trên bản đồ?
  * ID của các landmark được ghép cặp có bị nhảy vọt/thay đổi liên tục giữa các khung hình kề nhau không?
  * Có xảy ra hiện tượng so khớp sai nghiệm do môi trường/bản đồ có tính đối xứng hoặc cách đều tuần hoàn không?
  * Sai số dư khoảng cách của các cặp ghép nối ban đầu có lớn không?

---

### Bước 4 — Kiểm tra Geometry (Khảo sát Hình học)

* **Tệp tin kiểm tra**: `geometry_debug.csv`
* **Câu hỏi chẩn đoán**:
  * Có xuất hiện cảnh báo thẳng hàng `NEAR_COLLINEAR` không?
  * Độ phân tán không gian của các điểm `spread_lidar` và `spread_map` có bị quá nhỏ (gần trùng tụ) không?
  * Chỉ số condition number của các ma trận điểm có vượt quá ngưỡng cấu hình cho phép (`max_condition_number`) không?

---

### Bước 5 — Kiểm tra SVD Solver

* **Tệp tin kiểm tra**: `svd_debug.csv`
* **Câu hỏi chẩn đoán**:
  * Định thức của ma trận xoay $\det(R)$ có xấp xỉ bằng $1.0$ (ma trận trực giao chuẩn) không? Có bị hiện tượng $\det(R) = -1.0$ (phép phản chiếu đối xứng) không?
  * Góc xoay Yaw tính toán ra có bị nhảy đột ngột hoặc dao động nhiễu mạnh không?
  * Sai số RMSE sau tối ưu `residual_rmse` và sai số cực đại `max_residual` của các inliers có vượt quá ngưỡng kiểm duyệt không?

---

### Bước 6 — Kiểm tra Transform Convention (Quy ước Biến đổi)

* **Câu hỏi chẩn đoán**:
  * Có tuân thủ tuyệt đối quy ước toán học của hệ thống:
    $$p_{\text{map}} \approx R \times p_{\text{lidar}} + t$$
  * Tệp poses kết xuất ra có đúng là biến đổi $T_{\text{map\_lidar}}$ không?
* **Quy tắc**: Nếu quỹ đạo xe chạy bị phản chiếu đối gương, hoặc góc xoay Yaw bị quay ngược hướng ($180^\circ$), hãy kiểm tra xem có bị dùng nhầm chiều transform thành $T_{\text{lidar\_map}}$ hay không.

---

### Bước 7 — Kiểm tra Robot Frame & Extrinsic

* **Câu hỏi chẩn đoán**:
  * Vị trí LiDAR thực tế lắp trên robot có trùng khít hoàn toàn với tâm quay robot base không?
  * Đã áp dụng phép bù trừ extrinsic transform $T_{\text{base\_lidar}}$ hoặc $T_{\text{lidar\_base}}$ chưa?
* **Quy tắc**: Nếu pose tính toán cho LiDAR hoàn toàn khớp với mốc nhưng robot chạy vẫn lệch một khoảng dịch chuyển cố định so với Ground Truth, lỗi chắc chắn nằm ở việc ngầm định sai `lidar_frame == base_link` khi chưa cấu hình bù trừ.

---

### Bước 8 — Kiểm tra Trajectory-level Consistency (Quỹ đạo liên tục)

* **Câu hỏi chẩn đoán**:
  * Vận tốc di chuyển của robot tính từ sai lệch Pose giữa 2 khung hình liên tiếp có nằm trong giới hạn động học vật lý thực tế của xe không?
  * Quỹ đạo di chuyển tổng thể có bị đứt gãy hay gián đoạn ở các khung hình bị mất dấu không?
  * Có cần thiết phải bổ sung bộ lọc Kalman (KF) hoặc Pose Prior để hạn chế nhảy vọt không?

---

## 3. Command debug nhanh

Chạy localization:

```bash
python3 scripts/run_svd_localization.py \
  --detections data/results/sample_run/detections.json \
  --map data/maps/rf_map_v1.json \
  --output data/results/sample_run/localization
```

Xem pose:

```bash
head data/results/sample_run/localization/poses.csv
```

Xem frame bị reject:

```bash
head data/results/sample_run/localization/rejected_frames.csv
```

Xem association:

```bash
head data/results/sample_run/localization/association_debug.csv
```

Chạy test:

```bash
python3 -m pytest tests/test_svd_pose.py
python3 -m pytest tests/test_data_association.py
python3 -m pytest tests/test_localizer_pipeline.py
```

Chạy toàn bộ test:

```bash
python3 -m pytest
```

---

## 4. Bug Log Template

Khi có bug mới, copy template này.

```markdown
## #P2-BUG-XXX - YYYY-MM-DD - Tên lỗi

### 1. Trạng thái

- **Status:** Open / Fixed / Monitoring
- **Severity:** Low / Medium / High / Critical
- **Module liên quan:** `src/rf_threshold/localization/...`
- **Người phát hiện:**
- **Commit liên quan:**

---

### 2. Mô tả hiện tượng

Mô tả lỗi:

```text
<pose sai như thế nào, crash ở đâu, output gì bất thường>
```

Lệnh gây lỗi:

```bash
<lệnh đã chạy>
```

Log lỗi:

```text
<paste traceback hoặc log tại đây>
```

---

### 3. Điều kiện tái hiện

- Detections file:
- RF map file:
- Config:
- Frame index:
- Số detections:
- Số matches:
- Residual RMSE:
- Status output:

Các bước tái hiện:

```text
1. ...
2. ...
3. ...
```

---

### 4. Root Cause

Phân loại nguyên nhân:

- [ ] Sai RF map
- [ ] Sai hệ tọa độ
- [ ] Sai data association
- [ ] Thiếu RF
- [ ] Suy biến hình học
- [ ] Lỗi SVD solver
- [ ] Không kiểm tra residual
- [ ] Không xử lý NaN
- [ ] Khác

Root cause:

```text
<giải thích nguyên nhân gốc rễ>
```

---

### 5. Solution

File đã sửa:

```text
src/rf_threshold/localization/...
tests/...
```

Code trước khi sửa:

```python
<code lỗi nếu cần>
```

Code sau khi sửa:

```python
<code đã sửa>
```

---

### 6. Test xác nhận

Đã chạy:

```bash
python3 -m pytest tests/...
```

Kết quả:

```text
<test pass / output>
```

---

### 7. Prevention

Bài học:

```text
<làm sao tránh lỗi này lặp lại>
```

Có cần cập nhật `CONTRIBUTING_PHASE2.md` không?

- [ ] Không
- [ ] Có

Quy tắc cần thêm:

```text
<quy tắc mới>
```
```

---

## 5. Known Issues dự đoán trước

## #P2-PREDICT-001 - Chạy SVD khi số RF < 3

### Trạng thái

- **Status:** Known Risk
- **Severity:** Critical
- **Module:** `localizer_pipeline.py`, `svd_pose.py`

### Hiện tượng

Frame chỉ có 1–2 RF nhưng hệ thống vẫn cố chạy SVD.

### Root cause

Thiếu kiểm tra:

```python
if len(matched_pairs) < 3:
    ...
```

### Solution

Bắt buộc return:

```text
LocalizationStatus.INSUFFICIENT_MATCHES
```

Không chạy SVD.

### Prevention

Unit test bắt buộc:

```python
def test_localizer_rejects_frame_with_less_than_three_matches():
    ...
```

---

## #P2-PREDICT-002 - Nearest-neighbor sai do khác hệ tọa độ

### Trạng thái

- **Status:** Known Risk
- **Severity:** Critical
- **Module:** `data_association.py`

### Hiện tượng

Matching sai hàng loạt, pose nhảy xa hoặc yaw sai.

### Root cause

Code so sánh trực tiếp:

```text
center_lidar với position_map
```

trong khi hai điểm thuộc hai hệ tọa độ khác nhau.

### Solution

Chỉ dùng nearest-neighbor nếu có initial pose.

Nếu không có initial pose, dùng:

```text
pairwise distance
triplet matching
enumerate candidate correspondence + residual selection
```

### Prevention

Cấm nearest-neighbor trực tiếp nếu chưa có transform dự đoán.

---

## #P2-PREDICT-003 - Pose OK nhưng residual quá lớn

### Trạng thái

- **Status:** Known Risk
- **Severity:** High
- **Module:** `localizer_pipeline.py`, `svd_pose.py`

### Hiện tượng

Output status là:

```text
OK
```

nhưng `residual_rmse` lớn.

### Root cause

Pipeline không kiểm tra residual sau SVD.

### Solution

Thêm rule:

```python
if residual_rmse > max_residual_rmse:
    return LocalizationStatus.HIGH_RESIDUAL
```

### Prevention

Unit test:

```python
def test_localizer_rejects_high_residual_pose():
    ...
```

---

## #P2-PREDICT-004 - Landmark hoặc detection ID bằng 0 bị bỏ qua

### Trạng thái

- **Status:** Known Risk
- **Severity:** Medium
- **Module:** Toàn Phase 2

### Hiện tượng

Landmark `id=0` hoặc detection `id=0` không được match.

### Root cause

Dùng boolean check:

```python
if landmark_id:
    ...
```

### Solution

Dùng:

```python
if landmark_id is not None:
    ...
```

### Prevention

Test:

```python
def test_landmark_id_zero_is_valid():
    ...
```

---

## #P2-PREDICT-005 - SVD bị sai chiều transform

### Trạng thái

- **Status:** Known Risk
- **Severity:** Critical
- **Module:** `svd_pose.py`

### Hiện tượng

Pose bị ngược: thay vì tính `T_map_lidar`, code lại tính `T_lidar_map`.

### Root cause

Nhầm source và target trong SVD.

### Quy ước bắt buộc

Phase 2 dùng:

```text
source = points_lidar
target = points_map
```

Tức là:

```text
points_map ≈ R * points_lidar + t
```

Transform output là:

```text
T_map_lidar
```

### Prevention

Test synthetic bắt buộc:

```text
Tạo points_lidar
Apply known T_map_lidar để tạo points_map
SVD phải khôi phục đúng T_map_lidar
```

---

## #P2-PREDICT-006 - Suy biến hình học do RF gần thẳng hàng

### Trạng thái

- **Status:** Known Risk
- **Severity:** High
- **Module:** `geometry_check.py`

### Hiện tượng

Pose yaw dao động mạnh, dù residual có thể không quá lớn.

### Root cause

Các RF nằm gần thẳng hàng hoặc spatial spread quá nhỏ.

### Solution

Trước SVD kiểm tra:

```text
spatial spread
condition number
minimum pairwise distance
```

Nếu không đạt:

```text
DEGENERATE_GEOMETRY
```

### Prevention

Unit test với các điểm thẳng hàng hoặc gần trùng nhau.

---

## #P2-PREDICT-007 — Fixed tolerance quá chặt làm mất correspondence đúng

### Trạng thái

- **Status:** Known Risk
- **Severity:** High
- **Module:** `data_association.py`

### Hiện tượng

Hệ thống không tìm ra giải pháp so khớp cho các khung hình (frames) mặc dù số lượng mốc phản quang thực tế $\ge 3$. Robot bị mất dấu định vị.

### Root cause

Sử dụng giá trị sai số khoảng cách cố định (`distance_tolerance` cố định `0.05m`) quá chặt. Thực tế, khi mốc ở xa LiDAR, cụm điểm quét thưa dần dẫn đến sai số ước lượng tâm lớn hơn $5\text{ cm}$.

### Solution

Chuyển đổi sang sử dụng sai số thích nghi (Adaptive Tolerance):
$$\epsilon(d) = \min(\text{max\_abs}, \max(\text{min\_abs}, \text{relative\_ratio} \times d))$$
Cho phép nới lỏng sai số khi mốc ở xa và thắt chặt khi mốc ở gần.

### Prevention

Viết unit test kiểm chứng độ chính xác và tính đúng đắn của logic tính toán Adaptive Tolerance theo khoảng cách.

---

## #P2-PREDICT-008 — Triplet candidate quá nhiều do tolerance quá lỏng

### Trạng thái

- **Status:** Known Risk
- **Severity:** Medium
- **Module:** `data_association.py`

### Hiện tượng

Thời gian xử lý dữ liệu của một frame tăng đột biến (lag/delay), hệ thống sinh ra quá nhiều ứng viên bộ ba không hợp lệ.

### Root cause

Các tham số cấu hình thích nghi (`min_abs`, `relative_ratio`, hoặc `max_abs`) đặt quá lớn khiến bộ lọc độ dài cạnh bị lỏng lẻo, dẫn đến bùng nổ tổ hợp ứng viên.

### Solution

- Khống chế số lượng ứng viên tối đa duyệt qua bằng tham số `max_candidates: 300`.
- Sắp xếp và ưu tiên các ứng viên theo điểm số `score` (số lượng inliers nhiều nhất, RMSE nhỏ nhất).
- Tinh chỉnh lại các tham số thích nghi trong file YAML cấu hình tiêu chuẩn.

### Prevention

Viết unit test xác thực giới hạn `max_candidates` và kiểm tra thuật toán chấm điểm candidate hoạt động đúng để loại bỏ ứng viên kém chất lượng.

---

## #P2-PREDICT-009 — Map có khoảng cách lặp gây nhiều nghiệm tương tự

### Trạng thái

- **Status:** Known Risk
- **Severity:** High
- **Module:** `data_association.py`

### Hiện tượng

Pose ước lượng của robot bị nhảy cóc (jump) hoặc xoay ngược hướng $180^\circ$ mặc dù SVD solver vẫn trả về kết quả thành công và sai số RMSE nhỏ.

### Root cause

Hành lang có thiết kế các mốc RF cách đều nhau, dẫn đến nhiều bộ ba landmark khác nhau trên bản đồ có cùng cấu hình khoảng cách (edge signature) tương tự nhau, gây ra hiện tượng nghiệm giả.

### Solution

- Thực hiện bước xác thực mở rộng (Verification stage) bằng cách dùng Pose ứng viên để transform toàn bộ detections còn lại và kiểm tra inliers trên toàn bộ bản đồ.
- Sử dụng tổng `residual_rmse` và `max_residual` của toàn bộ các điểm inliers để xếp hạng và bác bỏ các nghiệm giả.
- Ở phiên bản sau, tích hợp thêm thông tin Pose trước đó (Previous Pose Prior) làm vùng tìm kiếm giới hạn.

### Prevention

Viết unit test mô phỏng bản đồ tuần hoàn/lặp lại để kiểm tra xem thuật toán có chọn đúng bộ ba mốc thực tế dựa trên số lượng inliers lớn nhất và RMSE tổng thể nhỏ nhất hay không.

---

## #P2-PREDICT-010 — Hard reject near-collinear geometry làm mất frame hợp lệ

### Trạng thái

- **Status:** Known Risk
- **Severity:** High
- **Module:** `geometry_check.py`, `localizer_pipeline.py`

### Hiện tượng

Hệ thống liên tục từ chối xử lý và trả về mã lỗi `DEGENERATE_GEOMETRY`, làm robot mất dấu định vị liên tục, đặc biệt là trong môi trường hành lang (corridor) mặc dù khâu phát hiện (detection) và so khớp (association) hoàn toàn chính xác.

### Root cause

Khâu kiểm tra hình học (`geometry_check.py`) được thiết lập mặc định từ chối cứng tất cả các trường hợp có chỉ số condition number lớn:
```text
condition_number > max_condition_number
```
Tuy nhiên, trong hành lang thực tế, các mốc phản quang RF hầu hết được lắp đặt thẳng hàng dọc hai bên tường hành lang. Khi robot quét, các mốc này gần như thẳng hàng, dẫn đến trị riêng thứ hai rất nhỏ $\rightarrow$ chỉ số condition number cực kỳ lớn. Do đó, việc tự động coi đây là lỗi chết và hard reject sẽ loại bỏ hầu hết các frame hợp lệ.

### Solution

Không được phép hard reject near-collinear geometry theo mặc định. Sử dụng cơ chế cảnh báo thích nghi (Warning-only):
* Đảm bảo cấu hình YAML mặc định là `hard_reject: false`.
* Khi phát hiện near-collinear nhưng độ phân tán không gian vẫn đủ lớn (`spread >= min_spread`), hệ thống chỉ phát cảnh báo `warning = "NEAR_COLLINEAR"`, đặt `is_valid = True` và tiếp tục luồng chạy giải SVD.
* Chất lượng Pose cuối cùng sẽ được thẩm định trực quan thông qua sai số dư (residual RMSE check) sau đó.

### Prevention

Bắt buộc có 2 unit tests kiểm chứng:
1. `test_near_collinear_geometry_returns_warning_not_rejection()`: Kiểm tra các điểm gần thẳng hàng có spread lớn thì chỉ cảnh báo và tiếp tục.
2. `test_tiny_spread_geometry_is_rejected()`: Kiểm tra các điểm trùng khít nhau (spread cực nhỏ) thì bắt buộc phải hard reject.

---

## #P2-PREDICT-011 - Pose sai nhưng tất cả unit tests đều pass

### Trạng thái

- **Status:** Known Risk
- **Severity:** Critical
- **Module:** Phase 2.6 runner / real data integration

### Hiện tượng

Toàn bộ các unit tests thô của hệ thống đều báo xanh (100% pass) nhưng khi tích hợp chạy trên file ROS bag thực tế, pose robot bị sai lệch nghiêm trọng hoặc quỹ đạo di chuyển bị méo mó.

### Root cause

Unit tests kiểm tra từng module nhỏ lẻ bằng dữ liệu giả lập lý tưởng (synthetic), tuy nhiên dữ liệu quét thực tế từ xe thật thường đi kèm:
* Sự sai lệch tâm mốc quan sát do LiDAR quét thưa/bụi.
* Sai sót trong thiết lập tọa độ landmark bản đồ gốc.
* Hiện tượng mơ hồ so khớp (association ambiguity) do môi trường cách đều.
* Sai lệch hoặc nhầm lẫn trong quy ước chiều transform không gian.
* Thiếu bù trừ extrinsic transform giữa `base_link` và cảm biến LiDAR.

### Solution

Thiết kế Phase 2.6 bắt buộc phải kết xuất đầy đủ các tệp bằng chứng debug chi tiết ra thư mục kết quả (`poses.csv`, `poses.json`, `rejected_frames.csv`, `association_debug.csv`, `svd_debug.csv`, `geometry_debug.csv`, `frame_debug.csv`) để phục vụ kiểm vết.

### Prevention

Tuyệt đối không nghiệm thu Phase 2.6 nếu hệ thống chỉ xuất ra tệp `poses.csv` mà thiếu đi các tệp lưu vết debug trung gian.

---

## #P2-PREDICT-012 - Dùng nhầm T_map_lidar và T_lidar_map

### Trạng thái

- **Status:** Known Risk
- **Severity:** Critical
- **Module:** `run_svd_localization.py`, `localizer_pipeline.py`

### Hiện tượng

Robot pose xoay yaw bị ngược, vị trí dịch chuyển tịnh tiến bị phản chiếu đối xứng qua các trục, hoặc quỹ đạo di chuyển bay xa bất thường.

### Root cause

Nhầm lẫn chiều biến đổi ma trận trong khâu giải SVD hoặc khâu biến đổi hệ tọa độ. Quy ước toán học đúng của dự án là:
$$p_{\text{map}} \approx R \times p_{\text{lidar}} + t$$
Với đầu ra SVD là $T_{\text{map\_lidar}}$. Nếu lập trình viên nhầm lẫn thứ tự nguồn-đích, phép giải sẽ biến thành $T_{\text{lidar\_map}}$, dẫn tới sai lệch.

### Solution

Trong tệp bằng chứng debug `svd_debug.csv`, bắt buộc phải ghi nhận đầy đủ các phần tử ma trận xoay `R00, R01, R10, R11`, vector dịch chuyển `tx, ty` và góc xoay `yaw` thực tế, và thực thi kiểm chứng độc lập bằng công thức:
$$p_{\text{map\_pred}} = R \times p_{\text{lidar}} + t$$

---

## #P2-PREDICT-013 - Pose robot lệch cố định do thiếu transform base_link ↔ lidar_frame

### Trạng thái

- **Status:** Known Risk
- **Severity:** High
- **Module:** Phase 2.6 / future TF integration

### Hiện tượng

Quỹ đạo di chuyển của robot (trajectory) có hình dạng rất chuẩn xác so với Ground Truth nhưng bị dịch chuyển tịnh tiến lệch cố định (constant offset) một khoảng cách cố định so với tâm robot hoặc bản đồ thực tế.

### Root cause

Thuật toán SVD hiện tại chỉ giải ra pose cho cảm biến LiDAR ($T_{\text{map\_lidar}}$). Nếu LiDAR không được lắp đặt trùng khớp hoàn toàn tại tâm quay vật lý của robot (`base_link`), chúng ta cần một extrinsic transform bù trừ ($T_{\text{map\_base}} = T_{\text{map\_lidar}} \times T_{\text{lidar\_base}}$). Việc ngầm định cẩu thả `lidar_frame == base_link` sẽ gây ra offset cố định này.

### Solution

Không được cứng hóa mặc định hệ trục. Cần bổ sung các tham số cấu hình linh hoạt trong YAML ở phiên bản sau:
```yaml
frames:
  map_frame: "map"
  lidar_frame: "livox_frame"
  base_frame: "base_link"
extrinsic:
  base_to_lidar:
    x: 0.15
    y: 0.0
    yaw: 0.0
```

---

## #P2-PREDICT-014 - Map origin hoặc trục map sai làm pose lệch toàn cục

### Trạng thái

- **Status:** Known Risk
- **Severity:** High
- **Module:** RF map / Phase 2.6 integration

### Hiện tượng

Kết quả định vị tương đối ổn định nhưng toàn bộ quỹ đạo của robot bị xoay nghiêng hoặc lệch hẳn so với bản đồ thực tế.

### Root cause

Bản đồ cột mốc RF nạp vào (`rf_map_v1.json`) sử dụng sai gốc tọa độ origin, sai tỷ lệ xích đơn vị đo, hoặc bị đảo ngược hai trục tọa độ $x \leftrightarrow y$.

### Solution

Bắt buộc sử dụng công cụ trực quan hóa đồ họa (`plot_localization_debug.py`) để kiểm tra trực quan:
* Bản đồ landmarks RF toàn cục.
* Tọa độ các detections sau khi áp dụng transform $T_{\text{map\_lidar}}$ phải chồng khít hoàn hảo lên các điểm landmarks bản đồ tương ứng.

---

## #P2-PREDICT-015 - Trôi Pose robot (drift) do dùng fallback liên tiếp quá dài

### Trạng thái

- **Status:** Known Risk
- **Severity:** High
- **Module:** `fallback_manager.py`

### Hiện tượng

Quỹ đạo robot bị đứng im hoặc bị lệch rất xa so với vị trí di chuyển thực tế (trôi vị trí) mặc dù hệ thống vẫn xuất dữ liệu Pose đều đặn và không báo lỗi crash.

### Root cause

Khi robot di chuyển vào vùng mù không phát hiện đủ số mốc RF, thuật toán SVD không thể tính toán pose mới. Nếu `max_consecutive_fallback_frames` đặt quá lớn, `FallbackManager` liên tục dùng lại `last_valid_pose` cũ khiến vị trí robot bị đứng yên trong khi xe thực tế vẫn đang chuyển động.

### Solution

* Khống chế chặt chẽ giới hạn số frame liên tiếp tối đa được phép dùng fallback thông qua cấu hình `max_consecutive_fallback_frames: 5`.
* Khi vượt quá giới hạn này, buộc phải dừng fallback, chuyển trạng thái lỗi gốc và báo mất dấu định vị.

---

## #P2-PREDICT-016 - Gộp nhầm Pose fallback vào Pose OK làm sai lệch báo cáo chất lượng định vị

### Trạng thái

- **Status:** Known Risk
- **Severity:** Medium
- **Module:** `localization_writer.py`

### Hiện tượng

Báo cáo thống kê chất lượng định vị SRE báo cáo tỷ lệ định vị thành công rất cao (ví dụ: 99%), nhưng trên thực tế xe chạy bị trôi pose và va chạm nhiều lần do mất dấu.

### Root cause

`LocalizationWriter` gộp chung cả frame định vị SVD thành công (`OK`) và frame dự phòng (`FALLBACK_LAST_VALID_POSE`) vào cùng một danh mục thành công trong báo cáo summary.

### Solution

Tách biệt hoàn toàn hai chỉ số này trong tệp summary định dạng **Key-Value**:
```csv
metric,value
num_frames,1000
num_ok,820
num_fallback,120
num_rejected_without_fallback,60
```

---

## 6. Checklist debug nhanh Phase 2

- [ ] `detections.json` có tồn tại không?
- [ ] Frame có >= 3 RF không?
- [ ] `center_lidar` có NaN không?
- [ ] RF map có đúng format không?
- [ ] Landmark ID có bị trùng không?
- [ ] Có match >= 3 cặp không?
- [ ] Có duplicate detection id trong matched pairs không?
- [ ] Có duplicate landmark id trong matched pairs không?
- [ ] SVD input có đúng chiều `lidar → map` không?
- [ ] Geometry check có pass không?
- [ ] Residual RMSE có dưới ngưỡng không?
- [ ] Pose có NaN/Inf không?
- [ ] Frame fail có được ghi vào `rejected_frames.csv` không?
- [ ] Bug mới đã được ghi vào file này chưa?

---

## 7. Quy tắc bắt buộc cho AI Agent

Khi AI sửa lỗi Phase 2:

1. Không sửa bằng cách che lỗi.
2. Không bỏ qua frame lỗi mà không ghi `rejected_frames.csv`.
3. Không đổi chiều transform nếu chưa test synthetic.
4. Không dùng nearest-neighbor trực tiếp nếu chưa có initial pose.
5. Nếu sửa bug logic phức tạp, phải thêm mục mới vào file này.
6. Nếu bug do thiếu quy tắc coding, cập nhật `CONTRIBUTING_PHASE2.md`.

---

## 8. Physical Diagnostic Checklist

Khi pose robot tính toán bị lệch hoặc sai trên dữ liệu bag thực tế (mặc dù code không crash và test pass), kỹ sư hãy kiểm tra theo trình tự các bước vật lý sau:

### Step 1 — Kiểm tra RF Detection Rate (Phase 1)
- **Hành động:** Kiểm tra `frame_debug.csv` hoặc log chạy. Xem tỷ lệ phần trăm số frame thu nhận được $\ge 3$ detections.
- **Biện pháp:** Nếu tỷ lệ này $< 50\%$, hệ thống không đủ đầu vào cho SVD. Cần kiểm tra lại độ nhạy/ngưỡng cường độ phản xạ trong Phase 1 (file config) hoặc kiểm tra xem các landmark vật lý có bị bụi bẩn, che khuất không.

### Step 2 — Kiểm tra Association ID Consistency (So khớp ID)
- **Hành động:** Kiểm tra `association_debug.csv`. Xem các `landmark_id` được gán cho các detection qua các frame liên tiếp có bị nhảy loạn hoặc hoán vị sai lệch không.
- **Biện pháp:** Nếu có, cần tinh chỉnh các ngưỡng khoảng cách thích nghi (`triplet_distance_tolerance`) trong config để thắt chặt hoặc nới lỏng phù hợp.

### Step 3 — Kiểm tra SVD Residual RMSE
- **Hành động:** Kiểm tra `svd_debug.csv` và `validation_report.md`.
- **Biện pháp:** 
  - `residual_rmse > 0.05 m`: Có hiện tượng nhiễu lớn hoặc lệch nhẹ.
  - `residual_rmse > 0.10 m`: Khả năng cực kỳ cao là tọa độ của các landmark trên bản đồ thực tế (`rf_map_v1.json`) bị đo đạc sai lệch vật lý hoặc bị nhầm lẫn vị trí.

### Step 4 — Kiểm tra Yaw & Xoay Trục
- **Hành động:** Xem biểu đồ quỹ đạo `01_trajectory.png` và các giá trị `yaw` trong `poses.csv`.
- **Biện pháp:** 
  - Nếu robot quay nhưng yaw không đổi hoặc quỹ đạo bị phản xạ đối xứng qua trục: kiểm tra xem có dùng nhầm chiều transform thành $T_{\text{lidar\_map}}$ thay vì $T_{\text{map\_lidar}}$ hay không.
  - Nếu yaw lệch đúng $180^\circ$ hoặc đảo chiều: kiểm tra quy ước chiều dương góc quay (quy tắc bàn tay phải).

### Step 5 — Kiểm tra Quy ước Hệ Tọa độ (Coordinate Convention)
- **Hành động:** Xác minh quan hệ transform:
  $$p_{\text{map}} \approx R \times p_{\text{lidar}} + t$$
- **Biện pháp:** Đảm bảo tất cả các điểm đo và map landmark đều sử dụng chung hệ đơn vị mét và đúng chiều dương của các trục tọa độ.

### Step 6 — Kiểm tra Fallback Drift
- **Hành động:** Kiểm tra cột `is_fallback` và `consecutive_fallback_count` trong `poses.csv`.
- **Biện pháp:** Nếu chuỗi fallback quá dài (vượt quá 5 frame liên tiếp) trong khi robot di chuyển với vận tốc cao, sai số tích lũy (drift) sẽ rất lớn. Khuyến nghị bổ sung mốc RF vật lý ở vùng mù này hoặc tích hợp cảm biến Odometry/IMU.