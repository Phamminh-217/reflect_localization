# CONTRIBUTING_PHASE2.md

# Quy chuẩn đóng góp cho Phase 2 — SVD-Based RF Localization

## 1. Mục tiêu quy chuẩn

File này đặt ra các nguyên tắc code bắt buộc cho Phase 2.

Phase 2 nhạy cảm hơn Phase 1 vì một lỗi nhỏ trong:

```text
hệ tọa độ
data association
SVD
residual check
```

có thể làm pose robot sai hoàn toàn nhưng chương trình vẫn không crash.

Do đó, Phase 2 ưu tiên:

```text
đúng toán học
rõ hệ tọa độ
test kỹ
debug đầy đủ
không che lỗi
```

---

## 2. Quy chuẩn Python

Dự án dùng:

```text
Ubuntu 20.04
ROS1 Noetic
Python 3.8
```

Do đó code phải tương thích Python 3.8.

Không dùng:

```python
A | B
list[int]
dict[str, Any]
tuple[int, int]
```

Phải dùng:

```python
Union[A, B]
List[int]
Dict[str, Any]
Tuple[int, int]
Optional[A]
```

---

## 3. Naming convention

## 3.1. File names

Dùng `snake_case`:

```text
pose.py
map_loader.py
svd_pose.py
data_association.py
geometry_check.py
localizer_pipeline.py
localization_writer.py
```

Không dùng:

```text
SVDPose.py
dataAssociation.py
poseSolver.py
```

---

## 3.2. Function names

Dùng `snake_case`, có động từ rõ ràng:

```python
load_rf_map()
associate_detections_to_map()
estimate_pose_svd_2d()
check_geometry_validity()
compute_residual_rmse()
write_pose_results()
```

---

## 3.3. Class names

Dùng `PascalCase`:

```python
RFMapLandmark
MatchedPair
RobotPose
LocalizationResult
SVDPoseResult
SVDPoseEstimator
RFLocalizer
```

---

## 3.4. Coordinate variable names

Bắt buộc ghi rõ frame tọa độ.

Đúng:

```python
point_lidar
point_map
points_lidar_xy
points_map_xy
center_lidar
position_map
T_map_lidar
pose_map_lidar
```

Sai:

```python
point
points
center
position
T
pose
```

---

## 4. Quy tắc module

## 4.1. `pose.py`

Chỉ chứa data class và enum.

Không được:

- đọc file;
- chạy SVD;
- matching;
- ghi output.

---

## 4.2. `map_loader.py`

Chỉ đọc RF map.

Không được:

- sửa tọa độ;
- tự sinh landmark;
- matching;
- chạy SVD.

Nếu map lỗi, raise lỗi rõ ràng.

---

## 4.3. `data_association.py`

Chỉ tạo correspondence.

Không được:

- tự đọc map file;
- tự đọc detections.json;
- ghi output;
- chạy pipeline hoàn chỉnh.

---

## 4.4. `svd_pose.py`

Chỉ giải bài toán SVD.

Không được:

- đọc file;
- chọn correspondence;
- filter detection;
- gọi detector Phase 1.

---

## 4.5. `localizer_pipeline.py`

Chỉ điều phối.

Không được viết lại chi tiết SVD hoặc map loader trong file này.

---

## 5. Quy tắc toán học

## 5.1. Số lượng điểm tối thiểu

Không chạy SVD nếu:

```text
num_matched_pairs < 3
```

Bắt buộc trả:

```text
LocalizationStatus.INSUFFICIENT_MATCHES
```

---

## 5.2. Không dùng nearest-neighbor trực tiếp nếu chưa có initial pose

Sai:

```text
So sánh center_lidar trực tiếp với position_map bằng Euclidean distance.
```

Lý do: hai tập điểm nằm ở hai hệ tọa độ khác nhau.

Chỉ được dùng nearest-neighbor nếu đã có:

```text
initial_pose
hoặc transform dự đoán
```

Nếu chưa có initial pose, dùng:

```text
pairwise-distance matching
triplet matching
enumerate correspondence + residual selection
```

---

## 5.3. Quy tắc bắt buộc cho Geometry Check

Không được mặc định reject tất cả các trường hợp near-collinear geometry.

Sai:
```python
if condition_number > max_condition_number:
    return GeometryCheckResult(is_valid=False, ...)
```

Đúng:
```python
if condition_number > max_condition_number:
    if hard_reject:
        return GeometryCheckResult(
            is_valid=False,
            is_degenerate=True,
            warning="NEAR_COLLINEAR",
            reason="Condition number exceeds threshold.",
            # ...
        )
    return GeometryCheckResult(
        is_valid=True,
        is_degenerate=True,
        warning="NEAR_COLLINEAR",
        reason="Near-collinear geometry detected, continuing with warning.",
        # ...
    )
```

Hard reject (`is_valid = False`) chỉ áp dụng bắt buộc cho:
* Thiếu số lượng match tối thiểu ($< 3$).
* Duplicate detection/landmark IDs.
* Tọa độ chứa NaN/Inf.
* Spatial spread quá nhỏ (nhỏ hơn `min_spread`).

Trường hợp gần thẳng hàng (Near-collinear) với spread đủ lớn phải được coi là **cảnh báo (warning) mặc định**, không được chặn luồng thực thi chạy SVD.

---

## 5.4. Bắt buộc kiểm tra residual sau SVD

Sau SVD phải tính:

```text
residual_rmse
max_residual
```

Nếu residual vượt ngưỡng:

```text
LocalizationStatus.HIGH_RESIDUAL
```

Không được trả pose `OK`.

---

## 5.5. Không trả pose chứa NaN

Bất kỳ pose nào chứa:

```text
NaN
Inf
```

phải bị reject.

---

## 6. Error handling

Không dùng:

```python
try:
    ...
except Exception:
    return None
```

Không dùng:

```python
try:
    ...
except:
    pass
```

Cách đúng:

```python
if len(matched_pairs) < min_required_matches:
    return LocalizationResult(
        status=LocalizationStatus.INSUFFICIENT_MATCHES,
        pose=None,
        matched_pairs=[],
        residual_rmse=None,
        reason="Need at least 3 matched RF landmarks.",
        debug_info={...},
    )
```

Exception chỉ dùng cho lỗi lập trình hoặc dữ liệu không hợp lệ nghiêm trọng.

---

## 7. Logging

Phase 2 phải log các giá trị sau cho mỗi frame:

```text
frame_index
stamp
num_detections
num_filtered_detections
num_matches
status
residual_rmse
reason
```

Log mẫu:

```text
[INFO] frame=123 detections=4 matches=3 status=OK rmse=0.018
[WARNING] frame=124 detections=2 status=INSUFFICIENT_DETECTIONS
[WARNING] frame=125 matches=3 status=HIGH_RESIDUAL rmse=0.245
```

Không dùng `print()` trong module `localization/`.

---

## 8. Testing requirement

Mỗi module mới phải có test.

| Module | Test |
|---|---|
| `pose.py` | `tests/test_pose.py` |
| `map_loader.py` | `tests/test_map_loader.py` |
| `svd_pose.py` | `tests/test_svd_pose.py` |
| `data_association.py` | `tests/test_data_association.py` |
| `geometry_check.py` | `tests/test_geometry_check.py` |
| `localizer_pipeline.py` | `tests/test_localizer_pipeline.py` |

---

## 9. Test bắt buộc cho SVD

`test_svd_pose.py` phải có:

- Identity transform.
- Translation-only transform.
- Rotation-only transform.
- Rotation + translation transform.
- Noise nhỏ.
- Insufficient points.
- NaN input.
- Degenerate geometry.

---

## 10. Test bắt buộc cho data association

`test_data_association.py` phải có đầy đủ 14 kịch bản kiểm thử bắt buộc sau:

1. Reject nếu detections < 3
2. Reject nếu landmarks < 3
3. Adaptive tolerance min_abs hoạt động đúng
4. Adaptive tolerance relative_ratio hoạt động đúng
5. Adaptive tolerance max_abs hoạt động đúng
6. Match đúng synthetic transform known
7. Match đúng khi detection bị shuffle thứ tự
8. Match đúng khi có thêm detection nhiễu
9. Reject nếu residual_rmse > max_candidate_rmse
10. Reject duplicate landmark id
11. Reject duplicate detection id
12. Chấp nhận detection_id = 0 và landmark_id = 0
13. Không dùng nearest-neighbor trực tiếp khi chưa có initial pose
14. Trả AssociationResult thay vì crash

## 10.1. Test bắt buộc cho geometry check

`test_geometry_check.py` phải có ít nhất hai test riêng biệt:
1. Points nearly identical / spread too small (spatial spread < min_spread) → `is_valid = False`, `is_degenerate = True` (Từ chối cứng).
2. Near-collinear but well-spread points (condition_number > max_condition_number) → `is_valid = True`, `is_degenerate = True`, `warning = "NEAR_COLLINEAR"` (Cảnh báo mặc định, không reject).

Không được viết test mặc định mong đợi near-collinear trả về `is_valid = False` trừ khi cấu hình chỉ định `hard_reject: true`.

---

## 10.2. Test bắt buộc cho Phase 2.5 localizer pipeline

`test_localizer_pipeline.py` phải có đầy đủ 11 kịch bản kiểm thử bắt buộc sau:
1. Success scenario với các cặp matches hợp lệ.
2. Insufficient detections → Trạng thái `INSUFFICIENT_DETECTIONS`.
3. Association failure → Trạng thái `ASSOCIATION_FAILED` hoặc mã lỗi tương ứng.
4. Tiny spatial spread → Trạng thái `DEGENERATE_GEOMETRY` (Hard reject).
5. Near-collinear but well-spread → Trả về warning `"NEAR_COLLINEAR"`, pipeline vẫn tiếp tục giải Pose.
6. Near-collinear + good residual → Trạng thái định vị `OK` thành công.
7. Near-collinear + high residual → Trạng thái lỗi `HIGH_RESIDUAL`.
8. Trùng lặp mã detection_id → Trạng thái lỗi `DEGENERATE_GEOMETRY` (Hard reject).
9. Trùng lặp mã landmark_id → Trạng thái lỗi `DEGENERATE_GEOMETRY` (Hard reject).
10. SVD residual vượt ngưỡng → Trạng thái lỗi `HIGH_RESIDUAL`.
11. Chấp nhận và giải thành công với điểm số ID 0.

---

## 11. Output requirement

Phase 2 output bắt buộc gồm:

```text
poses.csv
poses.json
localization_summary.csv
rejected_frames.csv
association_debug.csv
```

Nếu frame bị reject, vẫn phải ghi vào `rejected_frames.csv`.

Không được bỏ qua frame âm thầm.

---

## 12. Git workflow

Phase 2 phải làm trên branch riêng:

```bash
git checkout -b phase-2-svd-localization
```

Commit theo từng module:

```text
feat: add localization pose dataclasses
feat: add RF map loader
feat: implement 2D SVD pose solver
feat: add localization geometry checks
feat: add data association module
feat: add localizer pipeline
test: add SVD pose tests
docs: add phase 2 architecture
```

---

## 13. Checklist trước khi commit

- [ ] Code tương thích Python 3.8.
- [ ] Không dùng `A | B`.
- [ ] Không dùng `list[int]`, `dict[str, Any]`.
- [ ] Tên biến tọa độ có hậu tố frame.
- [ ] Không chạy SVD khi `< 3 matches`.
- [ ] Không dùng fixed distance_tolerance mặc định cho triplet matching.
- [ ] Bắt buộc dùng adaptive tolerance.
- [ ] Mọi association candidate phải được verify bằng SVD residual.
- [ ] Có degeneracy check.
- [ ] Có residual check.
- [ ] Không dùng nearest-neighbor trực tiếp nếu chưa có initial pose.
- [ ] Có unit test cho module mới.
- [ ] Không phá Phase 1.
- [ ] Không import thresholding trong localization.
- [ ] Không commit file bag.

---

## 13.5. Quy tắc bắt buộc cho Phase 2.6

### 1. Không viết thuật toán mới trong CLI runner
Tệp `scripts/run_svd_localization.py` chỉ đóng vai trò điều phối dòng chảy (load đầu vào, gọi `RFLocalizer`, và lưu đầu ra thông qua `LocalizationWriter`). Tuyệt đối không tự triển khai SVD, matching bộ ba hay geometry check trong tệp runner này.

### 2. Mọi pose đều phải có debug evidence đầy đủ
Nếu một frame định vị thành công (`status == OK`), hệ thống bắt buộc phải ghi lại chi tiết mọi bằng chứng trung gian vào các tệp debug tương ứng (`poses.csv`, `association_debug.csv`, `svd_debug.csv`, `geometry_debug.csv`, `frame_debug.csv`).

### 3. Không che giấu lỗi định vị sai bằng cách bỏ qua frame
Nghiêm cấm việc bỏ qua âm thầm các frame bị lỗi (skip/continue mà không ghi nhận). Mọi frame không thể tính được pose thành công bắt buộc phải được kết xuất vào `rejected_frames.csv` kèm theo nguyên nhân lỗi chi tiết.

### 4. Sử dụng status cụ thể thay vì lỗi chung chung
Khi định vị thất bại, bắt buộc phải phân loại và gán mã trạng thái lỗi thích hợp nhất từ `LocalizationStatus` (ví dụ: `INSUFFICIENT_DETECTIONS`, `ASSOCIATION_FAILED`, `DEGENERATE_GEOMETRY`, `HIGH_RESIDUAL`, `MAP_ERROR`), không được lạm dụng trả về `ERROR` chung chung cho mọi kịch bản lỗi.

### 5. Tuân thủ tuyệt đối chiều transform quy ước
Hệ thống sử dụng quy ước thống nhất:
$$p_{\text{map}} \approx R \times p_{\text{lidar}} + t$$
Do đó biến đổi đầu ra của SVD là $T_{\text{map\_lidar}}$. Khi ghi kết quả hoặc debug, phải tuân thủ nghiêm ngặt chiều này để tránh hiện tượng đảo trục hay phản chiếu trajectory robot.

### 6. Không tự ngầm định hệ trục robot
Robot base và cảm biến LiDAR có thể có các hệ trục tọa độ khác nhau. Tuyệt đối không tự ý giả định `lidar_frame == base_link` trong mã nguồn mà phải xử lý qua tham số extrinsic bù trừ một cách tường minh khi chuyển đổi pose robot sau này.

---

## 13.6. Test bắt buộc cho Phase 2.6 (I/O & Integration)

Tệp `tests/test_localization_writer.py` bắt buộc phải được triển khai và kiểm chứng đầy đủ 7 kịch bản kiểm thử (test cases) sau đây:

1. **Ghi poses.csv thành công**: Kiểm tra xem `LocalizationWriter` có ghi đúng định dạng và dữ liệu pose robot cho các frame thành công hay không.
2. **Ghi rejected_frames.csv thành công**: Xác thực các frame định vị lỗi được ghi nhận đầy đủ kèm cột nguyên nhân lỗi rõ ràng.
3. **Ghi association_debug.csv thành công**: Kiểm tra xem tất cả các cặp tương ứng đã so khớp (matched pairs) của frame OK có được xuất chi tiết hay không.
4. **Ghi svd_debug.csv thành công**: Xác thực tệp chứa đầy đủ các phần tử ma trận xoay $R$, vector tịnh tiến $t$, góc xoay yaw, và sai số residual cho từng cặp điểm.
5. **Ghi geometry_debug.csv thành công**: Kiểm tra xem độ phân tán không gian (spatial spread) và chỉ số condition number của từng frame có được ghi nhận chính xác.
6. **Đồng bộ hóa khóa liên kết (traceability)**: Kiểm chứng xem tất cả các tệp debug kết xuất ra có đồng bộ nhất quán về hai trường `frame_index` và `stamp` để hỗ trợ truy vết ngược hay không.
7. **Khả năng chống crash**: Kiểm thử xem runner CLI (`run_svd_localization.py`) có xử lý trơn tru và không crash khi gặp hỗn hợp frame thành công/thất bại, hoặc thậm chí khi toàn bộ tất cả các frame trong tệp vào đều bị lỗi định vị.

---

## 14. Quy tắc cho AI Agent

Khi AI viết code Phase 2, bắt buộc:

1. Đọc `DOCS/PHASE2/ARCHITECTURE_PHASE2.md`.
2. Không sửa Phase 1 nếu không được yêu cầu.
3. Không gọi threshold detector trong localization.
4. Không tự ý đổi `RFDetection`.
5. Không tạo pose nếu matching không đủ điều kiện.
6. Không che lỗi bằng `try/except`.
7. Nếu sửa bug phức tạp, cập nhật `DEBUG_LOG_PHASE2.md`.
8. Nếu cần thay đổi API, phải giải thích trước.