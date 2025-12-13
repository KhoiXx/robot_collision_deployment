# Feature Specification: Tài Liệu Luận Văn Thạc Sĩ LaTeX

**Feature Branch**: `001-thesis-latex-docs`
**Created**: 2025-11-09
**Status**: Draft
**Input**: Tạo tài liệu luận văn thạc sĩ bằng LaTeX theo chuẩn HCMUT về "Ứng Dụng Reinforcement Learning Điều Khiển Phân Tán Hệ Đa Robot Tránh Va Chạm", dựa trên đề cương đã có và kết quả nghiên cứu thực tế

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Hoàn thành Chương 1: Mở Đầu (Priority: P1)

Là học viên thạc sĩ, tôi cần hoàn thành chương Mở đầu để giới thiệu bối cảnh, mục tiêu, phạm vi và ý nghĩa của đề tài nghiên cứu một cách rõ ràng và thuyết phục.

**Why this priority**: Chương Mở đầu là phần quan trọng nhất để thiết lập nền tảng cho toàn bộ luận văn. Đây là điểm khởi đầu mà hội đồng và người đọc sẽ đánh giá đầu tiên về tính khả thi và giá trị của nghiên cứu.

**Independent Test**: Có thể kiểm tra độc lập bằng cách đọc chương 1 và xác nhận rằng nó trả lời đầy đủ 4 câu hỏi: (1) Tại sao chọn đề tài này? (2) Mục tiêu nghiên cứu là gì? (3) Phạm vi nghiên cứu giới hạn như thế nào? (4) Ý nghĩa khoa học và thực tiễn ra sao?

**Acceptance Scenarios**:

1. **Given** đề cương phần Giới thiệu và tài liệu kỹ thuật, **When** viết phần "Lý do chọn đề tài", **Then** nội dung phải giải thích rõ (a) vấn đề thực tế về điều khiển đa robot, (b) hạn chế của phương pháp tập trung, (c) lý do chọn phương pháp học tăng cường

2. **Given** mục tiêu từ đề cương và kết quả đạt được, **When** viết phần "Mục tiêu nghiên cứu", **Then** nội dung phải liệt kê rõ ràng các mục tiêu cụ thể: (a) nghiên cứu thuật toán PPO, (b) huấn luyện model cho 44 robots, (c) đạt success rate 71-88%, (d) triển khai trên robot thực tế

3. **Given** thông tin về scope từ đề cương, **When** viết phần "Đối tượng và phạm vi", **Then** nội dung phải định rõ: (a) robot nonholonomic trên mặt phẳng 2D, (b) cảm biến LiDAR 360 độ, (c) môi trường có chướng ngại vật, (d) phương pháp phân tán (không tập trung)

4. **Given** đóng góp của nghiên cứu, **When** viết phần "Ý nghĩa khoa học và thực tiễn", **Then** nội dung phải nêu rõ: (a) cải tiến so với bài báo gốc (learning rate warmup, value clipping, adaptive LR), (b) đạt kết quả tương đương (88% test vs 90-95% paper), (c) ứng dụng thực tế trong kho hàng, nhà máy

---

### User Story 2 - Hoàn thành Chương 2: Tổng Quan (Priority: P2)

Là học viên thạc sĩ, tôi cần hoàn thành chương Tổng quan để trình bày các nghiên cứu liên quan, phân tích công trình đã có, và chỉ ra vấn đề còn tồn tại cần nghiên cứu.

**Why this priority**: Chương Tổng quan chứng minh rằng học viên đã nghiên cứu kỹ lưỡng tài liệu, hiểu rõ bối cảnh khoa học, và xác định được khoảng trống nghiên cứu cần lấp đầy.

**Independent Test**: Kiểm tra bằng cách xác minh rằng chương 2 trích dẫn ít nhất 15 tài liệu (bao gồm bài báo gốc Long et al. 2018), so sánh ít nhất 3 phương pháp khác nhau, và chỉ ra rõ ràng vấn đề mà luận văn này giải quyết.

**Acceptance Scenarios**:

1. **Given** tài liệu về multi-robot systems, **When** viết phần giới thiệu về hệ đa robot, **Then** nội dung phải trình bày: (a) định nghĩa và phân loại multi-robot systems, (b) ứng dụng thực tế (kho hàng, nhà máy, logistics), (c) thách thức chính trong điều khiển đa robot, (d) tầm quan trọng của collision avoidance

2. **Given** tài liệu về các phương pháp truyền thống, **When** viết phần các phương pháp tránh va chạm truyền thống, **Then** nội dung phải so sánh: (a) ORCA (Optimal Reciprocal Collision Avoidance), (b) Artificial Potential Field, (c) RRT/RRT*, (d) Model Predictive Control (MPC), (e) ưu nhược điểm từng phương pháp, (f) hạn chế với số lượng robot lớn

3. **Given** tài liệu về deep learning approaches, **When** viết phần các phương pháp dựa trên học sâu, **Then** nội dung phải trình bày: (a) Deep Reinforcement Learning cho navigation, (b) Imitation Learning và Inverse RL, (c) End-to-end learning từ sensors, (d) Xu hướng hiện tại và các nghiên cứu tiêu biểu, (e) So sánh centralized vs decentralized approaches

4. **Given** bài báo gốc 1709.10082.pdf, **When** viết phần phương pháp của Long et al. 2018, **Then** nội dung phải trình bày chi tiết: (a) POMDP formulation cho multi-robot problem, (b) kiến trúc CNN với 2 lớp Conv1D (32 filters, kernel 5 và 3), (c) thuật toán PPO với clipping, (d) chiến lược huấn luyện 2 giai đoạn với curriculum learning, (e) kết quả 96.5-100% success rate với 4-20 robots, (f) đóng góp chính của nghiên cứu

5. **Given** lý thuyết từ đề cương, **When** viết phần cơ sở lý thuyết về Reinforcement Learning, **Then** nội dung phải giải thích: (a) Markov Decision Process (MDP) và Partially Observable MDP, (b) Policy, Value function, Q-function, (c) Policy gradient methods, (d) Actor-Critic architecture, (e) Advantage function và Generalized Advantage Estimation (GAE)

6. **Given** paper PPO của Schulman 2017, **When** viết phần cơ sở lý thuyết về PPO, **Then** nội dung phải giải thích: (a) Vấn đề của policy gradient truyền thống (high variance, instability), (b) Trust Region Policy Optimization (TRPO) background, (c) PPO clipping objective function, (d) Tại sao PPO hiệu quả (sample efficiency, stability), (e) Hyperparameters chính (clip ratio, epochs, batch size)

7. **Given** lý thuyết neural networks, **When** viết phần cơ sở lý thuyết về kiến trúc mạng, **Then** nội dung phải trình bày: (a) Convolutional Neural Networks (CNN) cho sequential data, (b) Fully Connected layers, (c) Activation functions (ReLU, Sigmoid, Tanh), (d) Adam optimizer và learning rate, (e) Batch normalization và regularization techniques

8. **Given** lý thuyết về điều khiển PID, **When** viết phần cơ sở lý thuyết về PID Control, **Then** nội dung phải trình bày: (a) PID controller cho differential drive robot, (b) công thức điều khiển P, I, D components, (c) tuning PID parameters (Kp, Ki, Kd), (d) ứng dụng PID cho linear và angular velocity control, (e) ưu điểm và hạn chế của PID

9. **Given** tài liệu về sensor fusion, **When** viết phần cơ sở lý thuyết về UKF Sensor Fusion, **Then** nội dung phải giải thích: (a) Unscented Kalman Filter (UKF) cho nonlinear systems, (b) fusion giữa IMU (gyroscope, accelerometer) và wheel odometry, (c) cách UKF ước lượng pose (x, y, θ) của robot, (d) xử lý noise và uncertainty trong sensors, (e) so sánh UKF vs EKF (Extended Kalman Filter)

10. **Given** tài liệu về SLAM và mapping, **When** viết phần cơ sở lý thuyết về GMapping, **Then** nội dung phải trình bày: (a) Simultaneous Localization and Mapping (SLAM) problem, (b) GMapping algorithm (Rao-Blackwellized Particle Filter), (c) cách tạo occupancy grid map từ LiDAR data, (d) ứng dụng trong navigation và path planning, (e) integration với ROS (Robot Operating System)

---

### User Story 3 - Hoàn thành Chương 3: Phương Pháp Nghiên Cứu (Priority: P3)

Là học viên thạc sĩ, tôi cần hoàn thành chương Phương pháp để trình bày chi tiết cách thức thực hiện nghiên cứu, bao gồm môi trường, kiến trúc mạng, thuật toán huấn luyện, và thiết kế robot.

**Why this priority**: Chương này chứng minh tính khoa học và khả năng tái tạo của nghiên cứu. Người đọc phải hiểu đủ chi tiết để có thể lặp lại thí nghiệm.

**Independent Test**: Kiểm tra bằng cách xác minh rằng chương 3 mô tả đầy đủ: (1) observation space với 3×454 laser scans, (2) action space với [v, ω], (3) reward function với 5 components, (4) kiến trúc mạng với 4 lớp, (5) hyperparameters trong Table 2 của đề cương.

**Acceptance Scenarios**:

1. **Given** đề cương phần 4.1 về môi trường, **When** viết về POMDP formulation, **Then** nội dung phải mô tả: (a) observation space $o_t = [o_z^t, o_g^t, o_v^t]$, (b) action space $a_t = [v_t, ω_t]$, (c) reward function với 5 thành phần, (d) không gian quan sát riêng biệt cho từng robot

2. **Given** đề cương phần 4.2 về kiến trúc mạng, **When** viết về network architecture, **Then** nội dung phải mô tả: (a) 2 lớp Conv1D (32 filters, kernel 5 và 3), (b) 2 lớp Fully Connected (256, 128 neurons), (c) output activation (Sigmoid cho v, Tanh cho ω), (d) Gaussian sampling cho action

3. **Given** tài liệu COMPARISON_WITH_ORIGINAL_PAPER.md, **When** viết về các cải tiến so với bài báo gốc, **Then** nội dung phải nêu: (a) AdaptiveLRScheduler (maintain khi improving), (b) Separate critic/actor optimizers (15x LR difference), (c) Value clipping, (d) Learning rate warmup, (e) Best model auto-save

4. **Given** đề cương phần 4.3 về huấn luyện, **When** viết về training procedure, **Then** nội dung phải mô tả: (a) 7 tình huống huấn luyện (Figure 8), (b) 2 giai đoạn (Stage 1: 20 robots, Stage 2: 58 robots), (c) thuật toán PPO với GAE, (d) hyperparameters từ Table 2

5. **Given** đề cương phần 4.5 về robot thực tế, **When** viết về xây dựng robot thực tế, **Then** nội dung phải mô tả: (a) kích thước 20cm × 15.7cm, (b) RPLidar A1 360°, (c) Raspberry Pi với Ubuntu, (d) 2 động cơ stepper, (e) IMU sensor (gyroscope, accelerometer), (f) nguồn điện và mạch điều khiển

6. **Given** lý thuyết PID và yêu cầu điều khiển, **When** viết về thiết kế PID controller và chứng minh ổn định, **Then** nội dung phải trình bày: (a) mô hình động học differential drive robot, (b) thiết kế PID controller cho linear và angular velocity, (c) chứng minh ổn định Lyapunov cho closed-loop system, (d) phương pháp tuning Kp, Ki, Kd, (e) kết quả simulation và thực nghiệm

7. **Given** yêu cầu về localization chính xác, **When** viết về sensor fusion với UKF, **Then** nội dung phải mô tả: (a) mô hình state space cho robot (x, y, θ, vx, vy, ω), (b) measurement models cho IMU và wheel odometry, (c) thuật toán UKF để fusion sensors, (d) xử lý noise covariance matrices Q và R, (e) so sánh với EKF và raw odometry

8. **Given** yêu cầu về mapping và navigation, **When** viết về triển khai GMapping và navigation stack, **Then** nội dung phải trình bày: (a) cấu hình GMapping parameters (particles, resolution), (b) tạo occupancy grid map từ LiDAR, (c) integration với ROS navigation stack, (d) path planning với move_base, (e) obstacle avoidance với costmap

9. **Given** mô hình đã train và robot thực tế, **When** viết về triển khai hệ robot, **Then** nội dung phải mô tả: (a) kiến trúc hệ thống ROS nodes (sensor, control, RL policy), (b) deployment model lên Raspberry Pi, (c) real-time inference latency, (d) communication protocol giữa các robots, (e) testing procedure và safety mechanisms

---

### User Story 4 - Hoàn thành Chương 4: Kết Quả và Thảo Luận (Priority: P4)

Là học viên thạc sĩ, tôi cần hoàn thành chương Kết quả để trình bày các kết quả thực nghiệm, phân tích, so sánh với bài báo gốc, và thảo luận ý nghĩa.

**Why this priority**: Chương này là trọng tâm của luận văn, chứng minh rằng nghiên cứu đã đạt được mục tiêu đề ra và đóng góp gì mới so với công trình trước.

**Independent Test**: Kiểm tra bằng cách xác minh rằng chương 4 báo cáo đầy đủ: (1) kết quả Stage 1 (74% success), (2) kết quả Stage 2 (71% train, 88% test), (3) so sánh với paper (88% vs 96.5%), (4) phân tích 8 revisions, (5) biểu đồ training curves.

**Acceptance Scenarios**:

1. **Given** tài liệu WORK_SUMMARY_NOV1_2025.md và SNAPSHOT_SUMMARY.md, **When** viết về kết quả Stage 1, **Then** nội dung phải báo cáo: (a) Update 100: 80-83% success (Oct 25 baseline), (b) Hyperparameters đã sử dụng, (c) Training time và số lượng robot (20), (d) Biểu đồ success rate theo updates

2. **Given** tài liệu WORK_SUMMARY_NOV1_2025.md, **When** viết về kết quả Stage 2, **Then** nội dung phải báo cáo: (a) Training success: 71% (Update 160), (b) Test success: 88% (50 robots) và 100% (10 robots), (c) Hyperparameters Revision 8, (d) So sánh các revisions 1-8

3. **Given** tài liệu COMPARISON_WITH_ORIGINAL_PAPER.md, **When** viết phần so sánh với bài báo gốc, **Then** nội dung phải so sánh: (a) Hyperparameters (CRITIC_LR: 6e-3 vs 5e-5, ACTOR_LR: 4e-4 vs 5e-5), (b) Reward function (30/-25 vs 15/-15), (c) Kết quả (88% với 44 robots vs 96.5% với 20 robots), (d) Lý giải sự khác biệt

4. **Given** tài liệu COMPARISON_WITH_ORIGINAL_PAPER.md phần "Major Innovations", **When** viết về các cải tiến, **Then** nội dung phải thảo luận: (a) Adaptive LR tăng performance như thế nào, (b) Value clipping giảm instability ra sao, (c) Separate optimizers giúp gì cho training, (d) Tại sao test (88%) cao hơn train (71%)

5. **Given** metrics từ các file JSON trong snapshots, **When** tạo bảng biểu và hình vẽ, **Then** phải bao gồm: (a) Bảng so sánh hyperparameters, (b) Hình training curves (success rate, losses), (c) Bảng kết quả 8 revisions, (d) Hình so sánh với paper

---

### User Story 5 - Hoàn thành Chương 5: Kết Luận và Kiến Nghị (Priority: P5)

Là học viên thạc sĩ, tôi cần hoàn thành chương Kết luận để tóm tắt những đóng góp chính, hạn chế, và hướng phát triển tương lai.

**Why this priority**: Chương kết luận giúp hội đồng và người đọc nhanh chóng nắm bắt những điểm quan trọng nhất của luận văn và đánh giá giá trị nghiên cứu.

**Independent Test**: Kiểm tra bằng cách xác minh rằng chương 5 tóm tắt rõ: (1) 3-5 kết quả chính đã đạt được, (2) 2-3 hạn chế của nghiên cứu, (3) 2-3 hướng phát triển tương lai, (4) không có nội dung mới (chỉ tóm tắt).

**Acceptance Scenarios**:

1. **Given** các kết quả chính từ chương 4, **When** viết phần tóm tắt kết quả, **Then** nội dung phải tóm tắt: (a) Đạt 71% train và 88% test success rate, (b) Cải tiến 6 điểm so với bài báo gốc (Adaptive LR, Value Clipping, etc.), (c) Triển khai thành công 2-stage training, (d) Thiết kế được robot prototype

2. **Given** quá trình thực hiện, **When** viết về hạn chế, **Then** nội dung phải nêu: (a) Kết quả thấp hơn paper (88% vs 96.5%) do số lượng robot lớn hơn, (b) Chưa test trên robot thực tế (chỉ mới design), (c) Thời gian training dài (8 giờ cho Stage 2), (d) Hyperparameters cần điều chỉnh nhiều lần (8 revisions)

3. **Given** kinh nghiệm và ý tưởng phát triển, **When** viết về hướng nghiên cứu tiếp theo, **Then** nội dung phải đề xuất: (a) Triển khai và test trên robot thực tế, (b) Tăng số lượng robot lên 100+, (c) Thêm dynamic obstacles (vật cản động), (d) Nghiên cứu sim-to-real transfer

4. **Given** toàn bộ luận văn, **When** viết lời kết, **Then** nội dung phải khẳng định: (a) Đã đạt được mục tiêu đề ra, (b) Đóng góp vào lĩnh vực multi-robot systems, (c) Mở ra hướng nghiên cứu mới, (d) Cảm ơn thầy hướng dẫn và trường

---

### User Story 6 - Hoàn thành Các Phần Đầu Luận Văn (Priority: P6)

Là học viên thạc sĩ, tôi cần hoàn thành các phần đầu luận văn bao gồm bìa, lời cảm ơn, tóm tắt, lời cam đoan, mục lục, và danh mục ký hiệu theo đúng chuẩn HCMUT.

**Why this priority**: Các phần này tuy quan trọng về mặt hình thức nhưng có thể hoàn thành sau khi nội dung chính đã xong, vì chúng phụ thuộc vào nội dung các chương.

**Independent Test**: Kiểm tra bằng cách đối chiếu với THESIS_FORMAT_GUIDE.md: (1) Bìa có đủ thông tin và đúng font size, (2) Tóm tắt ≤ 1 trang, (3) Mục lục tự động, (4) Danh mục ký hiệu đầy đủ.

**Acceptance Scenarios**:

1. **Given** thông tin cá nhân và đề tài, **When** tạo trang bìa, **Then** phải bao gồm: (a) Tên trường (font 14), (b) Tên học viên: Nguyễn Tấn Khôi, (c) Tên đề tài (font 16), (d) Chuyên ngành và Mã số, (e) GVHD: TS. Phạm Việt Cường, (f) Thời gian: TP.HCM, tháng... năm...

2. **Given** nội dung toàn bộ luận văn, **When** viết tóm tắt tiếng Việt, **Then** phải đảm bảo: (a) Độ dài ≤ 1 trang, (b) Bao gồm: bối cảnh, mục tiêu, phương pháp, kết quả chính, (c) Ngôn ngữ súc tích, rõ ràng, (d) Không có trích dẫn trong tóm tắt

3. **Given** tóm tắt tiếng Việt, **When** dịch sang Abstract tiếng Anh, **Then** phải đảm bảo: (a) Nội dung tương đương với bản tiếng Việt, (b) Ngữ pháp và thuật ngữ chính xác, (c) Độ dài ≤ 1 trang

4. **Given** các thuật ngữ sử dụng trong luận văn, **When** tạo danh mục ký hiệu, **Then** phải liệt kê: (a) PPO (Proximal Policy Optimization), (b) RL (Reinforcement Learning), (c) LiDAR (Light Detection and Ranging), (d) GAE (Generalized Advantage Estimation), (e) Các ký hiệu toán học khác, (f) Sắp xếp theo ABC

---

### User Story 7 - Hoàn thành Danh Mục Tài Liệu Tham Khảo (Priority: P7)

Là học viên thạc sĩ, tôi cần hoàn thành danh mục tài liệu tham khảo theo chuẩn IEEE, bao gồm tất cả các nguồn đã trích dẫn trong luận văn.

**Why this priority**: Danh mục tài liệu tham khảo chứng minh tính học thuật của nghiên cứu và tránh đạo văn. Có thể hoàn thành sau cùng khi đã biết chính xác nguồn nào được trích dẫn.

**Independent Test**: Kiểm tra bằng cách xác minh: (1) Tất cả trích dẫn [1], [2]... trong văn bản đều có trong danh mục, (2) Tất cả mục trong danh mục đều được trích dẫn, (3) Format đúng IEEE style.

**Acceptance Scenarios**:

1. **Given** bài báo gốc 1709.10082.pdf, **When** tạo entry cho bài báo chính, **Then** phải format: `[1] P. Long, T. Fan, X. Liao, W. Liu, H. Zhang, and J. Pan, "Towards optimally decentralized multi-robot collision avoidance via deep reinforcement learning," in Proc. IEEE Int. Conf. Robot. Autom. (ICRA), 2018, pp. 6252-6259.`

2. **Given** đề cương phần Tài liệu tham khảo, **When** thêm các tài liệu đã trích dẫn, **Then** phải bao gồm: (a) PPO paper (Schulman et al., 2017), (b) Adam optimizer (Kingma & Ba, 2015), (c) GAE (Schulman et al., 2015), (d) Curriculum learning (Bengio et al., 2009), (e) Cartographer (Hess et al., 2016)

3. **Given** tất cả trích dẫn trong văn bản, **When** kiểm tra tính đầy đủ, **Then** phải đảm bảo: (a) Mỗi [n] trong văn bản có entry tương ứng, (b) Đánh số liên tục từ [1], [2], ..., (c) Theo thứ tự xuất hiện trong văn bản

4. **Given** danh mục đã hoàn chỉnh, **When** kiểm tra format IEEE, **Then** phải đảm bảo: (a) Tên tác giả: "First Initial. Last Name", (b) Tiêu đề bài báo trong "quotes", (c) Tên journal/conference in nghiêng, (d) Năm, trang số đầy đủ

---

### Edge Cases

- **Trường hợp số trang vượt quá 100**: Xem xét cắt giảm nội dung hoặc chuyển một số phần sang Phụ lục (metrics chi tiết, code snippets).

- **Trường hợp hình ảnh/bảng biểu không rõ ràng**: Vẽ lại hoặc tạo mới bằng tools (TikZ, Matplotlib) thay vì screenshot từ paper.

- **Trường hợp thiếu dữ liệu cho một số experiments**: Ghi chú rõ ràng trong văn bản và giải thích lý do (ví dụ: "Do hạn chế về thời gian, phần này chưa được thực hiện").

- **Trường hợp trích dẫn nguồn không rõ ràng**: Ưu tiên trích dẫn paper gốc hoặc tài liệu chính thức, tránh trích dẫn blog hoặc Wikipedia.

- **Trường hợp conflict giữa đề cương và kết quả thực tế**: Sử dụng kết quả thực tế mới nhất, giải thích sự khác biệt trong phần thảo luận.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Tài liệu PHẢI tuân thủ 100% THESIS_FORMAT_GUIDE.md (font Times New Roman 13pt, lề 3/3/3.5/2cm, line spacing 1.5, khổ A4)

- **FR-002**: Tài liệu PHẢI được viết bằng tiếng Việt cho tất cả nội dung chính, với thuật ngữ kỹ thuật tiếng Anh có giải thích

- **FR-003**: Tài liệu PHẢI bao gồm đầy đủ 10 phần: (1) Bìa, (2) Trang nhiệm vụ, (3) Lời cảm ơn, (4) Tóm tắt (VN+EN), (5) Lời cam đoan, (6) Mục lục, (7) Danh mục ký hiệu, (8) Nội dung 5 chương, (9) Danh mục công trình (nếu có), (10) Tài liệu tham khảo, (11) Phụ lục

- **FR-004**: Tài liệu PHẢI sử dụng nội dung từ đề cương DeCuong_NguyenTanKhoi_2171017_ver2.pdf làm nền tảng cho Chương 2 và Chương 3

- **FR-005**: Tài liệu PHẢI tích hợp kết quả nghiên cứu từ 4 file: stage1_stage2_comparison README.md, WORK_SUMMARY_NOV1_2025.md, COMPARISON_WITH_ORIGINAL_PAPER.md, SNAPSHOT_SUMMARY.md vào Chương 4

- **FR-006**: Tất cả trích dẫn PHẢI tuân theo chuẩn IEEE style với format [1], [2], ... theo thứ tự xuất hiện

- **FR-007**: Tất cả hình ảnh PHẢI có đầu đề ở phía dưới, đánh số theo chương (Hình 3.4 = hình thứ 4 chương 3), và được trích dẫn nguồn nếu lấy từ tài liệu khác

- **FR-008**: Tất cả bảng biểu PHẢI có đầu đề ở phía trên, đánh số theo chương (Bảng 2.5 = bảng thứ 5 chương 2), và được trích dẫn nguồn nếu lấy từ tài liệu khác

- **FR-009**: Tất cả phương trình PHẢI được đánh số trong ngoặc đơn ở lề phải (ví dụ: (3.1) = phương trình 1 chương 3)

- **FR-010**: Đánh số tiểu mục PHẢI tối đa 4 chữ số (ví dụ: 4.1.2.1) và mỗi nhóm tiểu mục phải có ít nhất 2 tiểu mục

- **FR-011**: Tổng số trang nội dung chính PHẢI ≤ 100 trang (không tính phụ lục)

- **FR-012**: Tài liệu PHẢI sử dụng LaTeX để đảm bảo format nhất quán và hỗ trợ công thức toán học

- **FR-013**: Mỗi chương PHẢI bắt đầu bằng phần giới thiệu ngắn (2-3 đoạn) về nội dung chương đó

- **FR-014**: Mỗi chương PHẢI kết thúc bằng phần tóm tắt hoặc kết luận chương (1-2 đoạn)

- **FR-015**: Tất cả thuật ngữ tiếng Anh xuất hiện lần đầu PHẢI có giải thích tiếng Việt kèm theo (ví dụ: "Reinforcement Learning (Học tăng cường)")

### Key Entities

- **Chương (Chapter)**: Đơn vị tổ chức lớn nhất của luận văn, bao gồm 5 chương chính (Mở đầu, Tổng quan, Phương pháp, Kết quả, Kết luận). Mỗi chương có số thứ tự, tiêu đề, phần giới thiệu, các mục con, và kết luận.

- **Hình ảnh (Figure)**: Các minh họa, biểu đồ, diagram, screenshot được sử dụng để minh họa nội dung. Phải có đầu đề dưới hình, đánh số theo chương, và được tham chiếu trong văn bản.

- **Bảng biểu (Table)**: Dữ liệu được tổ chức dưới dạng bảng (ví dụ: so sánh hyperparameters, kết quả experiments). Phải có đầu đề trên bảng, đánh số theo chương, và được tham chiếu trong văn bản.

- **Phương trình (Equation)**: Công thức toán học được sử dụng để mô tả thuật toán, reward function, loss function. Phải được đánh số trong ngoặc đơn ở lề phải và tham chiếu trong văn bản.

- **Tài liệu tham khảo (Reference)**: Các nguồn tài liệu (paper, sách, website) được trích dẫn trong luận văn. Phải tuân theo chuẩn IEEE, đánh số theo thứ tự xuất hiện, và liệt kê đầy đủ ở cuối luận văn.

- **Trích dẫn (Citation)**: Liên kết trong văn bản đến tài liệu tham khảo, sử dụng format [n] trong ngoặc vuông (ví dụ: "theo nghiên cứu của Long et al. [1]").

- **Ký hiệu (Symbol/Acronym)**: Các chữ viết tắt và ký hiệu toán học được sử dụng trong luận văn. Phải được giải thích lần đầu xuất hiện và liệt kê trong danh mục ký hiệu nếu dùng nhiều.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Hoàn thành đủ 5 chương với tổng số trang nội dung chính trong khoảng 80-100 trang

- **SC-002**: 100% nội dung tuân thủ THESIS_FORMAT_GUIDE.md về font, lề, line spacing, và cấu trúc

- **SC-003**: Có ít nhất 15 tài liệu tham khảo được trích dẫn đúng chuẩn IEEE

- **SC-004**: Có ít nhất 20 hình ảnh/biểu đồ với đầu đề và đánh số đầy đủ

- **SC-005**: Có ít nhất 5 bảng biểu so sánh/tổng hợp dữ liệu với đầu đề và đánh số đầy đủ

- **SC-006**: Chương 4 trình bày đầy đủ kết quả của ít nhất 8 revisions/experiments với biểu đồ training curves

- **SC-007**: So sánh rõ ràng giữa kết quả đạt được (71-88%) và kết quả bài báo gốc (96.5%) với giải thích lý do khác biệt

- **SC-008**: Chương 3 mô tả đầy đủ ít nhất 10 hyperparameters chính được sử dụng trong training

- **SC-009**: Tóm tắt tiếng Việt và tiếng Anh mỗi phần không quá 1 trang và cover đủ 4 điểm: bối cảnh, mục tiêu, phương pháp, kết quả

- **SC-010**: Danh mục ký hiệu liệt kê ít nhất 20 thuật ngữ/ký hiệu được sử dụng trong luận văn

- **SC-011**: Mục lục tự động với đầy đủ các chương, mục, tiểu mục (tối đa 4 cấp số)

- **SC-012**: Không có lỗi đạo văn - tất cả nội dung trích dẫn từ nguồn khác đều có citation rõ ràng

- **SC-013**: File LaTeX compile thành công không lỗi và tạo ra file PDF đúng format

- **SC-014**: Tất cả biểu đồ và hình ảnh có độ phân giải đủ rõ khi in (minimum 300 DPI)

- **SC-015**: Phần kết luận (Chương 5) tóm tắt được 3-5 đóng góp chính, 2-3 hạn chế, và 2-3 hướng phát triển
