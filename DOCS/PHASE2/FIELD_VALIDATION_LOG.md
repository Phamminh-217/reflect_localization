# Nhật Ký Thực Nghiệm Thực Địa (Field Validation Log)

Tài liệu này lưu lại lịch sử các lần chạy thử nghiệm hệ thống định vị RF-SVD trên các tệp dữ liệu ROS bag thực địa thu thập từ robot.

---

## Mẫu Nhật Ký (Template)

```markdown
### Run: <run_name> — <YYYY-MM-DD>
- **ROS Bag:** `<bag_name>.bag`
- **RF Map:** `<map_name>.json`
- **Cấu hình:** `config/<config_name>.yaml`
- **Kết quả tổng quan:**
  - **Tổng số frame:** <N>
  - **OK rate:** <X>%
  - **Fallback rate:** <Y>%
  - **Rejection rate:** <Z>%
  - **Mean SVD RMSE:** <R> m
  - **Max Consecutive Fallbacks:** <M>
- **Hiện tượng & Lỗi phát hiện (Issues):**
  - [Mô tả chi tiết các vấn đề về drift, collinearity, map error...]
- **Hành động xử lý (Actions Taken):**
  - [Các điều chỉnh config, hiệu chuẩn cảm biến extrinsic, đo đạc lại map...]
```

---

## Lịch Sử Thực Nghiệm (Run History)

### Run: run_001_synthetic_sim — 2026-06-06
- **ROS Bag:** `data/bags/lan4_u_.bag` *(Giả lập/Thực tế ban đầu)*
- **RF Map:** `data/maps/your_map_simple.json` *(18 landmarks)*
- **Cấu hình:** `config/threshold_field_v1.yaml`
- **Kết quả tổng quan:**
  - **Tổng số frame:** 100
  - **OK rate:** 98.0%
  - **Fallback rate:** 2.0%
  - **Rejection rate:** 0.0%
  - **Mean SVD RMSE:** 0.0245 m
  - **Max Consecutive Fallbacks:** 1
- **Hiện tượng & Lỗi phát hiện (Issues):** Một vài frame ở góc cua có số lượng landmark giảm xuống sát ngưỡng 3, kích hoạt fallback ngắn hạn.
- **Hành động xử lý (Actions Taken):** Điều chỉnh config.

### Run: run_real_trial_01 — 2026-06-06
- **ROS Bag:** `/home/minh/rf_threshold_localization/data/bags/lan4.bag` *(Dữ liệu chạy thật)*
- **RF Map:** `data/maps/your_map_simple.json` *(18 landmarks)*
- **Cấu hình:** `config/threshold_field_v1.yaml`
- **Kết quả tổng quan:**
  - **Tổng số frame:** 1925
  - **OK rate:** 97.5% (1877 frames)
  - **Fallback rate:** 2.5% (48 frames)
  - **Rejection rate:** 0.0% (0 frames)
  - **Mean SVD RMSE:** 0.0135 m (1.35 cm)
  - **Max Consecutive Fallbacks:** 3
- **Hiện tượng & Lỗi phát hiện (Issues):** Không phát hiện bất kỳ cảnh báo hoặc lỗi hệ thống nào. Quỹ đạo robot hiển thị rất mượt mà.
- **Hành động xử lý (Actions Taken):** Đã tối ưu hóa trực quan hóa đồ thị trajectory giúp nét vẽ mỏng và sạch hơn, tránh bị chồng chéo dữ liệu.

