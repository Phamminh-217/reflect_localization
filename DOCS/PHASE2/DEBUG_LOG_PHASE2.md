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
residual
fallback
```

phải được ghi lại rõ ràng.

---

## 2. Standard Debugging Workflow

Khi pose sai hoặc localization fail, debug theo thứ tự sau.

---

## Bước 1 — Kiểm tra input từ Phase 1

Kiểm tra:

```text
detections.json có tồn tại không?
Frame có bao nhiêu RF?
center_lidar có NaN không?
score có hợp lý không?
frame_id có đúng là livox_frame không?
```

Lệnh:

```bash
head data/results/sample_run/detections.csv
```

---

## Bước 2 — Kiểm tra RF map

Kiểm tra:

```text
map file có tồn tại không?
landmark id có bị trùng không?
position_map có shape (3,) không?
đơn vị có phải meter không?
frame_id có phải map_frame không?
```

Lệnh:

```bash
cat data/maps/rf_map_v1.json
```

---

## Bước 3 — Kiểm tra số lượng RF

Trước khi matching:

```text
num_detections >= 3
```

Sau khi matching:

```text
num_matches >= 3
```

Nếu không đạt, không debug SVD trước. Lỗi nằm ở detection hoặc association.

---

## Bước 4 — Kiểm tra data association

Kiểm tra file:

```text
association_debug.csv
```

Cần xem:

```text
detection_id
landmark_id
x_lidar
y_lidar
x_map
y_map
weight
```

Nếu pair sai, SVD sẽ sai.

---

## Bước 5 — Kiểm tra suy biến hình học

Nếu các RF gần như thẳng hàng hoặc quá gần nhau, SVD có thể không ổn định.

Kiểm tra:

```text
spatial_spread_lidar
spatial_spread_map
condition_number
duplicate ids
```

---

## Bước 6 — Kiểm tra residual sau SVD

Nếu pose `OK` nhưng residual lớn, đây là bug.

Cần kiểm tra:

```text
residual_rmse
max_residual
```

Nếu vượt ngưỡng, status phải là:

```text
HIGH_RESIDUAL
```

---

## Bước 7 — Kiểm tra output pose

Kiểm tra:

```text
x
y
yaw
num_matches
residual_rmse
status
```

Pose không được chứa:

```text
NaN
Inf
```

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