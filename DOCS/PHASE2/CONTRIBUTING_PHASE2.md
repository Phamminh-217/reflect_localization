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

## 5.3. Bắt buộc kiểm tra suy biến hình học

Trước SVD phải kiểm tra:

```text
N >= 3
không có duplicate ids
không có NaN
spatial spread đủ lớn
condition number hợp lý
```

Nếu không đạt, không chạy SVD.

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

`test_data_association.py` phải có:

- Matching đúng với dữ liệu synthetic.
- Reject duplicate detection id.
- Reject duplicate landmark id.
- Reject nếu không đủ candidate.
- Reject nếu residual cao.
- Không dùng nearest-neighbor trực tiếp khi không có initial pose.

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
- [ ] Có degeneracy check.
- [ ] Có residual check.
- [ ] Không dùng nearest-neighbor trực tiếp nếu chưa có initial pose.
- [ ] Có unit test cho module mới.
- [ ] Không phá Phase 1.
- [ ] Không import thresholding trong localization.
- [ ] Không commit file bag.
- [ ] Test pass.

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