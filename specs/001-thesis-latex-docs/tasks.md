# Implementation Tasks: Tài Liệu Luận Văn Thạc Sĩ LaTeX Format Compliance

**Feature**: `001-thesis-latex-docs`
**Branch**: `001-thesis-latex-docs`
**Updated**: 2025-11-25
**Status**: Ready for Framework Fixes → Content Development

---

## Overview

This tasks file focuses on **fixing LaTeX formatting issues** (Phase 0) to comply with HCMUT standards, then proceeding with **content development** for the master's thesis on multi-robot collision avoidance using reinforcement learning.

### Critical Path

**PHASE 0 MUST BE COMPLETED FIRST** before any content work (Phase 1+). The framework fixes ensure the document structure is 100% compliant with HCMUT standards.

### Scope

- ✅ **Phase 0 (IN SCOPE - HIGH PRIORITY)**: Fix cover page borders, font sizes, structure issues, add missing bibliography entries
- ✅ **Phase 1+ (IN SCOPE - AFTER PHASE 0)**: Complete all 5 chapters with detailed content based on research results

---

## Task Summary

| Phase | Description | Task Count | Priority | Must Block Next Phase? |
|-------|-------------|------------|----------|------------------------|
| **Phase 0** | **Framework Formatting Fixes** | **8 tasks** | **CRITICAL** | **YES - blocks all content work** |
| Phase 1 | Setup & Verification | 2 | High | No |
| Phase 2 | Chapter 1: Mở Đầu Content (US1) | 4 | P1 | No |
| Phase 3 | Chapter 2: Tổng Quan Content (US2) | 6 | P2 | No |
| Phase 4 | Chapter 3: Phương Pháp Content (US3) | 7 | P3 | No |
| Phase 5 | Chapter 4: Kết Quả Content (US4) | 8 | P4 | No |
| Phase 6 | Chapter 5: Kết Luận Content (US5) | 4 | P5 | No |
| Phase 7 | Front Matter Completion (US6) | 5 | P6 | No |
| Phase 8 | Bibliography Completion (US7) | 3 | P7 | No |
| Phase 9 | Final Verification | 3 | High | No |
| **TOTAL** | | **50 tasks** | | |

---

## Phase 0: Framework Formatting Fixes (BLOCKING - DO FIRST!)

**Goal**: Fix all formatting violations identified in research.md to achieve 100% HCMUT compliance.

**Dependencies**: None (starting point)

**Blocking**: ALL content phases (1-8) should not start until framework is correct

**Why This Must Come First**:
- Cover page structure affects page numbering for entire document
- Font configurations must be correct before adding content
- Border implementation requires TikZ package setup
- Missing bibliography entry is referenced throughout chapters

### Tasks

- [X] T001 Add TikZ package to preamble.tex for cover page border support by adding `\usepackage{tikz}` after fontspec package in /home/khoint/thesis/deployment/docs/preamble.tex
- [X] T002 Implement double border on cover page by adding TikZ drawing commands (outer border 2pt, inner border 1pt, 3mm spacing) at beginning of 00_cover.tex in /home/khoint/thesis/deployment/docs/chapters/00_cover.tex
- [X] T003 Fix "LUẬN VĂN THẠC SĨ" font size from 13pt to 14pt by changing `\fontsize{13}{15}` to `\fontsize{14}{16}` on line ~45 in /home/khoint/thesis/deployment/docs/chapters/00_cover.tex
- [X] T004 Fix submission date font size from 13pt to 12pt by changing `\fontsize{13}{15}` to `\fontsize{12}{14}` on line ~65 in /home/khoint/thesis/deployment/docs/chapters/00_cover.tex
- [X] T005 Remove department info (KHOA ĐIỆN - ĐIỆN TỬ, BỘ MÔN TỰ ĐỘNG HÓA) from cover page by deleting lines 22-25 in /home/khoint/thesis/deployment/docs/chapters/00_cover.tex
- [X] T006 Add student name after university header with font 14pt bold centered by inserting `{\fontsize{14}{16}\selectfont\bfseries \authorname\\}` after university name block in /home/khoint/thesis/deployment/docs/chapters/00_cover.tex
- [X] T007 [P] Create secondary cover (bìa phụ) by copying 00_cover.tex structure to new file 00_cover_secondary.tex with advisor name kept (not removed like main cover) in /home/khoint/thesis/deployment/docs/chapters/00_cover_secondary.tex
- [X] T008 [P] Add Long et al. 2018 paper (1709.10082.pdf) to bibliography by creating BibTeX entry with title "Towards Optimally Decentralized Multi-Robot Collision Avoidance via Deep Reinforcement Learning", authors Long, Fan, Liao, Liu, Zhang, Pan, arXiv:1709.10082, 2018 in /home/khoint/thesis/deployment/docs/references.bib

**Verification**:
1. Run `xelatex thesis_main.tex` and verify compilation succeeds
2. Check cover page has visible double border
3. Measure font sizes: "LUẬN VĂN THẠC SĨ" = 14pt, date = 12pt
4. Verify no department info visible
5. Verify student name appears centered below university name
6. Verify secondary cover file exists and compiles
7. Run `biber thesis_main && xelatex thesis_main.tex` and verify [long2018towards] citation resolves

**Parallel Execution**: T007 (secondary cover) and T008 (bibliography) can run in parallel with T001-T006

**Time Estimate**: 1-2 hours for all fixes

---

## Phase 1: Setup & Verification

**Goal**: Verify LaTeX environment is ready and backup current state.

**Dependencies**: Phase 0 complete

**Blocking**: No (Phase 2-8 can start in parallel)

### Tasks

- [ ] T009 Backup current thesis_main.pdf to thesis_main_backup_YYYYMMDD.pdf before making content changes in /home/khoint/thesis/deployment/docs/
- [ ] T010 Create format compliance validation script by copying checklist from THESIS_FORMAT_GUIDE.md lines 258-285 to new file validate_format.sh in /home/khoint/thesis/deployment/docs/

**Verification**: Backup file exists, validation script executable

**Time Estimate**: 10 minutes

---

## Phase 2: Chapter 1: Mở Đầu Content (User Story 1 - P1)

**Goal**: Complete Chapter 1 with full content based on outline and actual research results.

**Dependencies**: Phase 0 complete

**User Story**: US1 - Hoàn thành Chương 1: Mở Đầu

**Independent Test**: Chapter 1 answers 4 questions: (1) Why this topic? (2) Research objectives? (3) Scope? (4) Scientific significance?

**Source Documents**:
- DeCuong_NguyenTanKhoi_2171017_ver2.pdf (outline)
- COMPARISON_WITH_ORIGINAL_PAPER.md (novel contributions)
- SNAPSHOT_SUMMARY.md (achieved results)

### Tasks

- [ ] T011 [P] [US1] Write section 1.1 "Lý do chọn đề tài" covering: (a) multi-robot coordination challenges in warehouses/factories, (b) limitations of centralized control, (c) benefits of deep RL approach, 2-3 paragraphs in /home/khoint/thesis/deployment/docs/chapters/02_chapter1_intro.tex
- [ ] T012 [P] [US1] Write section 1.2 "Mục tiêu nghiên cứu" listing specific objectives: (a) study PPO algorithm for multi-robot, (b) train model for 20-58 robots, (c) achieve 74-88% success rate, (d) compare with Long et al. 2018 baseline in /home/khoint/thesis/deployment/docs/chapters/02_chapter1_intro.tex
- [ ] T013 [P] [US1] Write section 1.3 "Đối tượng và phạm vi nghiên cứu" defining: (a) nonholonomic robots on 2D plane, (b) LiDAR 360° sensing, (c) static obstacles environment, (d) decentralized control architecture in /home/khoint/thesis/deployment/docs/chapters/02_chapter1_intro.tex
- [ ] T014 [US1] Write section 1.4 "Ý nghĩa khoa học và thực tiễn" explaining: (a) 5 novel contributions (Adaptive LR, Value Clipping, etc.), (b) comparable results to baseline (88% vs 96.5%), (c) practical applications in warehouses, citing [long2018towards] in /home/khoint/thesis/deployment/docs/chapters/02_chapter1_intro.tex

**Verification**: Chapter 1 is 4-6 pages, cites Long et al. paper, mentions 8 model revisions, states performance numbers

**Parallel Execution**: T011-T013 can be done in parallel (different sections)

**Time Estimate**: 2-3 hours

---

## Phase 3: Chapter 2: Tổng Quan Content (User Story 2 - P2)

**Goal**: Complete Chapter 2 with comprehensive literature review and comparison with baseline paper.

**Dependencies**: Phase 0 complete (needs Long et al. citation from T008)

**User Story**: US2 - Hoàn thành Chương 2: Tổng Quan

**Independent Test**: Chapter 2 has ≥15 citations including Long et al. 2018, compares ≥3 methods, identifies research gap

**Source Documents**:
- 1709.10082.pdf (baseline paper to review)
- COMPARISON_WITH_ORIGINAL_PAPER.md (detailed technical comparison)

### Tasks

- [X] T015 [P] [US2] Write comprehensive Chapter 2 covering: (a) Giới thiệu về hệ đa robot, (b) Các phương pháp tránh va chạm truyền thống (APF, RRT, ORCA with comparison table), (c) Các phương pháp dựa trên học sâu (DQN, A3C, Tai et al. 2017, Chen et al. 2017), (d) Phương pháp của Long et al. 2018 (POMDP, CNN architecture, PPO, 2-stage training), (e) Cơ sở lý thuyết RL (MDP, Value function, GAE, Actor-Critic), (f) Cơ sở lý thuyết PPO (TRPO background, clipping objective, hyperparameters), (g) Kiến trúc mạng nơ-ron (CNN, FC, activation functions, Adam optimizer), (h) Điều khiển PID (formula, tuning, applications to differential drive robots), (i) UKF Sensor Fusion (state space model, unscented transform, comparison with EKF), (j) GMapping và SLAM (RBPF, occupancy grid, scan matching, ROS integration) in /home/khoint/thesis/deployment/docs/chapters/03_chapter2_overview.tex
- [X] T016 [P] [US2] Create comparison table "Bảng 2.1: So sánh các phương pháp tránh va chạm truyền thống" with columns [Phương pháp, Ưu điểm, Nhược điểm, Khả năng mở rộng] covering APF, RRT/RRT*, and ORCA in /home/khoint/thesis/deployment/docs/chapters/03_chapter2_overview.tex
- [X] T017 [P] [US2] Create comparison table "Bảng 2.2: So sánh UKF và EKF" with columns [Tiêu chí, EKF, UKF] covering linearization method, accuracy, computational cost, and implementation complexity in /home/khoint/thesis/deployment/docs/chapters/03_chapter2_overview.tex
- [X] T020 Add 19 BibTeX entries including: Schulman PPO 2017, Schulman GAE 2015, Kingma Adam 2015, Mnih A3C 2016, Mnih DQN 2015, van den Berg ORCA 2011, Alonso-Mora NH-ORCA 2013, Khatib APF 1986, LaValle RRT 1998, Karaman RRT* 2011, LeCun Deep Learning 2015, Silver AlphaGo 2016, Tai mapless navigation 2017, Chen decentralized 2017, Julier UKF 2004, Grisetti GMapping 2007, Thrun Probabilistic Robotics 2005, Astrom Feedback Systems 2006, Siegwart Mobile Robots 2011 in /home/khoint/thesis/deployment/docs/references.bib

**Verification**: Chapter 2 is 8-12 pages, has comparison table, cites 15+ papers, explains why this thesis differs from Long et al.

**Parallel Execution**: T015-T017 can be done in parallel (different sections), T020 can run parallel with content writing

**Time Estimate**: 4-6 hours

---

## Phase 4: Chapter 3: Phương Pháp Content (User Story 3 - P3)

**Goal**: Complete Chapter 3 with full methodology documentation including all hyperparameters.

**Dependencies**: Phase 0 complete

**User Story**: US3 - Hoàn thành Chương 3: Phương Pháp Nghiên Cứu

**Independent Test**: Chapter 3 describes observation space (3×454), action space [v,ω], reward function (5 components), network architecture (4 layers), all hyperparameters

**Source Documents**:
- COMPARISON_WITH_ORIGINAL_PAPER.md lines 52-195 (reward, architecture)
- research.md lines 776-803 (Stage 1 & Stage 2 hyperparameters)

### Tasks

- [X] T021 [P] [US3] Write section 3.1 "Môi trường mô phỏng" explaining POMDP formulation: observation space $o_t = [o_z^t, o_g^t, o_v^t]$ with 3×454 laser scans + 2D goal + 2D velocity, action space $a_t = [v_t, ω_t]$, reward function with 5 components (progress +2Δd, 2-zone safety, rotation penalty -0.06|w|, heading +0.15, terminal ±30/-25/-10) with equation in /home/khoint/thesis/deployment/docs/chapters/04_chapter3_method.tex
- [X] T022 [P] [US3] Write section 3.2 "Kiến trúc mạng nơ-ron" describing: Conv1D(32,k=5,s=2)→ReLU, Conv1D(32,k=3,s=2)→ReLU, FC(256)→ReLU, [concat goal, velocity], FC(128)→ReLU, Actor: FC(2)→(v,w) with separate log_std, Critic: FC(1)→value in /home/khoint/thesis/deployment/docs/chapters/04_chapter3_method.tex
- [ ] T023 Create network architecture diagram as TikZ figure "Hình 3.1: Kiến trúc mạng Actor-Critic" showing input (laser+goal+vel) → Conv layers → FC layers → Actor/Critic outputs in /home/khoint/thesis/deployment/docs/chapters/04_chapter3_method.tex
- [X] T024 [US3] Write section 3.3 "Cải tiến so với bài báo gốc" explaining 5 novel contributions: (1) Adaptive LR system (maintain when improving, increase on plateau), (2) Asymmetric critic/actor training (15x LR difference), (3) Aggressive exploration (10x entropy), (4) Simplified reward (removed conflicting signals), (5) Modern PPO enhancements (value clipping, separate optimizers, explained variance, KL early stopping 233x more aggressive) - MERGED INTO SECTION 3.4 in /home/khoint/thesis/deployment/docs/chapters/04_chapter3_method.tex
- [X] T025 [US3] Write section 3.4 "Quy trình huấn luyện" documenting: 7 training scenarios, 2-stage training (Stage 1: 20 robots 200 updates, Stage 2: 58 robots 200 updates), PPO with GAE algorithm, hyperparameters Table 3.1 (Stage 1) and Table 3.2 (Stage 2) with all 13 parameters, and all 5 improvements integrated in /home/khoint/thesis/deployment/docs/chapters/04_chapter3_method.tex
- [X] T026 Create hyperparameters tables: "Bảng 3.1: Stage 1 Hyperparameters" (LAMDA=0.90, GAMMA=0.99, EPOCH=3, COEFF_ENTROPY=8e-3, ENTROPY_MIN=2e-3, CLIP_VALUE=0.15, CRITIC_LR=6e-3, ACTOR_LR=4e-4, value_loss_coeff=5.0, max_grad_norm=1.0, target_kl=0.035) and "Bảng 3.2: Stage 2 Hyperparameters" (LAMDA=0.94, EPOCH=5, COEFF_ENTROPY=7e-4, ENTROPY_MIN=3e-3, CLIP_VALUE=0.1, CRITIC_LR=5e-4, ACTOR_LR=1.5e-4, value_loss_coeff=3.5, value_clip=True) in /home/khoint/thesis/deployment/docs/chapters/04_chapter3_method.tex
- [X] T027 [P] [US3] Write section 3.5 "Xây dựng robot thực tế" describing: robot dimensions 20cm×15.7cm, RPLidar A1 360°, Raspberry Pi 4 with Ubuntu, 2 DC motors with encoders, IMU MPU6050, ROS architecture with 5 nodes, and deployment status (hardware complete, software modules tested individually, not yet fully integrated) in /home/khoint/thesis/deployment/docs/chapters/04_chapter3_method.tex

**Verification**: Chapter 3 is 10-15 pages, has 1 architecture diagram, 2 hyperparameter tables, reward function equation, explains all novel contributions

**Parallel Execution**: T021-T022, T024, T027 can run in parallel (different sections), T023 and T026 depend on their respective sections

**Time Estimate**: 6-8 hours

---

## Phase 5: Chapter 4: Kết Quả Content (User Story 4 - P4)

**Goal**: Complete Chapter 4 with all experimental results, training curves, and comparison analysis.

**Dependencies**: Phase 0 complete (needs Long et al. citation)

**User Story**: US4 - Hoàn thành Chương 4: Kết Quả và Thảo Luận

**Independent Test**: Chapter 4 reports Stage 1 (74%), Stage 2 (71% train, 88% test), compares with paper (88% vs 96.5%), analyzes 8 revisions, includes training curves

**Source Documents**:
- WORK_SUMMARY_NOV1_2025.md (detailed results)
- SNAPSHOT_SUMMARY.md (8 model revisions)
- COMPARISON_WITH_ORIGINAL_PAPER.md lines 289-329 (comparison analysis)

### Tasks

- [ ] T028 [P] [US4] Write section 4.1 "Kết quả Stage 1" reporting: Update 100: 70-74% success (Oct 25 baseline), hyperparameters from Bảng 3.1, training time 4-6 hours, 20 robots in circle scenario in /home/khoint/thesis/deployment/docs/chapters/05_chapter4_results.tex
- [ ] T029 [P] [US4] Write section 4.2 "Kết quả Stage 2" reporting: Update 160: 71% train success, Test: 88% success (50 robots), 100% success (10 robots), hyperparameters from Bảng 3.2, training time 8 hours, 58 robots in random scenario in /home/khoint/thesis/deployment/docs/chapters/05_chapter4_results.tex
- [ ] T030 Create results comparison table "Bảng 4.1: So sánh kết quả với Long et al. 2018" with columns [Metric, Long et al. 2018, This Thesis] and rows: Number of robots (20 circle / 15 random vs 20 Stage1 / 58 Stage2), Success rate (96.5% circle / ~90-95% random vs 74% Stage1 / 88% Stage2), Training approach (2-stage vs 2-stage), Novel techniques (Standard PPO vs 5 innovations) in /home/khoint/thesis/deployment/docs/chapters/05_chapter4_results.tex
- [ ] T031 [US4] Write section 4.3 "So sánh với bài báo gốc" analyzing: comparable performance despite larger robot count (88% with 58 robots vs 96.5% with 20), significantly different hyperparameters (120x critic LR, 8x actor LR, 10x entropy), similar 2-stage training philosophy, citing [long2018towards] in /home/khoint/thesis/deployment/docs/chapters/05_chapter4_results.tex
- [ ] T032 [US4] Write section 4.4 "Phân tích các cải tiến" discussing: (a) Adaptive LR prevented performance degradation, (b) Value clipping reduced training instability, (c) Asymmetric LR stabilized critic estimates early, (d) High entropy discovered diverse behaviors, (e) Test (88%) higher than train (71%) indicates good generalization in /home/khoint/thesis/deployment/docs/chapters/05_chapter4_results.tex
- [ ] T033 Create training curves figure "Hình 4.1: Success rate qua các updates" plotting success rate vs update number for both Stage 1 (0-200 updates, 70-74% final) and Stage 2 (0-200 updates, 71% final) using data from snapshots in /home/khoint/thesis/deployment/docs/chapters/05_chapter4_results.tex
- [ ] T034 Create revisions evolution table "Bảng 4.2: Phân tích 8 model revisions" documenting each revision with columns [Revision, Date, Key Changes, Performance Impact] from SNAPSHOT_SUMMARY.md in /home/khoint/thesis/deployment/docs/chapters/05_chapter4_results.tex
- [ ] T035 [P] [US4] Write section 4.5 "Thảo luận" covering: failure cases analysis (what causes remaining 12% test failures?), computational cost (MPI 24 processes, 8 hours Stage 2), limitations (not yet deployed on real robots), hardware specifications in /home/khoint/thesis/deployment/docs/chapters/05_chapter4_results.tex

**Verification**: Chapter 4 is 10-15 pages, has 1-2 figures (training curves), 2 tables (comparison, revisions), cites Long et al., explains all results

**Parallel Execution**: T028-T029, T031-T032, T035 can run in parallel (different sections), T030, T033, T034 depend on their sections

**Time Estimate**: 6-8 hours

---

## Phase 6: Chapter 5: Kết Luận Content (User Story 5 - P5)

**Goal**: Complete Chapter 5 with concise summary of contributions, limitations, and future work.

**Dependencies**: Phase 0 complete, Phases 2-5 complete (needs results to summarize)

**User Story**: US5 - Hoàn thành Chương 5: Kết Luận và Kiến Nghị

**Independent Test**: Chapter 5 summarizes 3-5 main contributions, 2-3 limitations, 2-3 future directions, no new content

**Source Documents**:
- COMPARISON_WITH_ORIGINAL_PAPER.md lines 332-430 (novel contributions summary)
- research.md lines 950-1017 (structured contributions list)

### Tasks

- [ ] T036 [P] [US5] Write section 5.1 "Tóm tắt kết quả đạt được" listing: (1) Achieved 74% Stage 1 and 88% Stage 2 test success, (2) Implemented 5 novel contributions (Adaptive LR, Asymmetric training, High entropy, Simplified reward, Modern PPO), (3) Successfully deployed 2-stage training, (4) Designed robot prototype, (5) Comparable to Long et al. with different approach in /home/khoint/thesis/deployment/docs/chapters/06_chapter5_conclusion.tex
- [ ] T037 [P] [US5] Write section 5.2 "Hạn chế của nghiên cứu" acknowledging: (1) Lower Stage 1 success vs baseline (74% vs 96.5%) due to larger robot count, (2) Not yet tested on real robots (only design complete), (3) Long training time (8 hours Stage 2), (4) Required 8 hyperparameter revisions to converge in /home/khoint/thesis/deployment/docs/chapters/06_chapter5_conclusion.tex
- [ ] T038 [P] [US5] Write section 5.3 "Hướng nghiên cứu tiếp theo" proposing: (1) Deploy and test on real robots (sim-to-real transfer), (2) Scale to 100+ robots, (3) Add dynamic obstacles (moving agents), (4) Study sim-to-real domain adaptation, (5) Automated hyperparameter tuning (Optuna, Ray Tune) in /home/khoint/thesis/deployment/docs/chapters/06_chapter5_conclusion.tex
- [ ] T039 [US5] Write section 5.4 "Lời kết" affirming: research objectives achieved, contributions to multi-robot systems field, opens new research directions, acknowledgment of advisor TS. Phạm Việt Cường and HCMUT in /home/khoint/thesis/deployment/docs/chapters/06_chapter5_conclusion.tex

**Verification**: Chapter 5 is 4-6 pages, lists 5 contributions clearly, acknowledges limitations honestly, proposes concrete future work

**Parallel Execution**: T036-T038 can run in parallel (different sections)

**Time Estimate**: 2-3 hours

---

## Phase 7: Front Matter Completion (User Story 6 - P6)

**Goal**: Complete all front matter sections with full Vietnamese and English content.

**Dependencies**: Phase 0 complete (needs correct cover structure), Phases 2-6 complete (needs content to summarize in abstracts)

**User Story**: US6 - Hoàn thành Các Phần Đầu Luận Văn

**Independent Test**: Front matter has cover with border, both abstracts ≤1 page, TOC auto-generated, symbols list ≥20 terms

**Source Documents**:
- research.md lines 1044-1084 (required symbols list)

### Tasks

- [ ] T040 Write "Lời cảm ơn" section thanking: advisor TS. Phạm Việt Cường for guidance, HCMUT for facilities, family for support, colleagues for discussions, 1 page in /home/khoint/thesis/deployment/docs/chapters/01_frontmatter.tex
- [ ] T041 Write "Tóm tắt" (Vietnamese abstract) covering: (1) research problem (multi-robot collision avoidance), (2) methodology (PPO with 5 novel contributions), (3) key results (74% Stage 1, 88% Stage 2), (4) significance (comparable to baseline with different approach), ≤250 words in /home/khoint/thesis/deployment/docs/chapters/01_frontmatter.tex
- [ ] T042 Write "Abstract" (English) translating Vietnamese abstract: research problem, PPO-based approach with adaptive learning rate and value clipping, 74-88% success rate, 5 novel contributions, comparable to Long et al. 2018 baseline, ≤250 words in /home/khoint/thesis/deployment/docs/chapters/01_frontmatter.tex
- [ ] T043 Write "Lời cam đoan" (Declaration of originality) stating: all work is original except cited sources, no plagiarism, results are from author's research, not submitted elsewhere before, signature placeholder in /home/khoint/thesis/deployment/docs/chapters/01_frontmatter.tex
- [ ] T044 Populate symbols.tex with 20+ entries: ABBREVIATIONS (PPO, GAE, DRL, RL, CNN, ORCA, NH-ORCA, LSTM, ROS), SYMBOLS (γ discount factor, λ GAE parameter, π policy, θ parameters, v linear velocity m/s, w angular velocity rad/s, α learning rate, ε clip value, r_t reward, s_t state, a_t action, V(s) value function, A(s,a) advantage), sorted alphabetically in /home/khoint/thesis/deployment/docs/symbols.tex

**Verification**: Abstracts are ≤1 page each and cover all 4 required points, symbols list has ≥20 terms, declaration present

**Parallel Execution**: T040-T043 can run in parallel (different sections), T044 parallel with all

**Time Estimate**: 2-3 hours

---

## Phase 8: Bibliography Completion (User Story 7 - P7)

**Goal**: Ensure all citations in text have corresponding BibTeX entries in IEEE format.

**Dependencies**: Phase 0 complete (has Long et al. entry), Phases 2-7 complete (know what's cited)

**User Story**: US7 - Hoàn thành Danh Mục Tài Liệu Tham Khảo

**Independent Test**: All [n] citations resolve, all references are cited, IEEE format correct, ≥15 entries

**Source Documents**:
- research.md lines 1089-1130 (required citations)

### Tasks

- [ ] T045 Add core RL papers to references.bib: Schulman et al. PPO 2017 "Proximal Policy Optimization Algorithms", Schulman et al. GAE 2015 "High-Dimensional Continuous Control Using Generalized Advantage Estimation", Mnih et al. A3C 2016 "Asynchronous Methods for Deep Reinforcement Learning", Kingma & Ba Adam 2015 "Adam: A Method for Stochastic Optimization" in IEEE format in /home/khoint/thesis/deployment/docs/references.bib
- [ ] T046 Add multi-robot papers to references.bib: van den Berg et al. ORCA 2011 "Reciprocal n-Body Collision Avoidance", Alonso-Mora et al. NH-ORCA 2013 "Optimal Reciprocal Collision Avoidance for Multiple Non-Holonomic Robots", Bengio et al. Curriculum 2009 "Curriculum Learning", and 5+ other relevant multi-robot/navigation papers in IEEE format in /home/khoint/thesis/deployment/docs/references.bib
- [ ] T047 Verify all citations by running: `grep -o '\[.*\]' chapters/*.tex | sort -u` to get all citations, then check each has BibTeX entry, and running `biber thesis_main --validate-datamodel` to check IEEE format correctness in /home/khoint/thesis/deployment/docs/

**Verification**: Bibliography has ≥15 entries, all citations [1]...[n] resolve, no "??" in PDF, IEEE format validated

**Parallel Execution**: T045-T046 can run in parallel (different papers)

**Time Estimate**: 1-2 hours

---

## Phase 9: Final Verification & Quality Check

**Goal**: Ensure complete document compiles cleanly and meets all HCMUT requirements.

**Dependencies**: All phases 0-8 complete

### Tasks

- [ ] T048 Run full compilation sequence: `make clean && xelatex thesis_main.tex && biber thesis_main && xelatex thesis_main.tex && xelatex thesis_main.tex` and verify no errors, no warnings about undefined references, PDF generated successfully in /home/khoint/thesis/deployment/docs/
- [ ] T049 Verify HCMUT format compliance checklist: (1) Cover has double border, (2) Fonts correct (14/16/13/12 pt), (3) Margins 3/3/3.5/2 cm, (4) Line spacing 1.5, (5) Page numbering Roman/Arabic, (6) Section depth ≤4 levels, (7) Figures/tables numbered by chapter, (8) Bibliography IEEE style, (9) ≥15 references, (10) Total pages ≤100, using validation script or manual inspection in /home/khoint/thesis/deployment/docs/
- [ ] T050 Create submission-ready checklist verification: (1) All 5 chapters complete, (2) Front matter complete (cover, abstracts, TOC, symbols), (3) Bibliography ≥15 entries, (4) All figures/tables have captions, (5) No undefined references (??), (6) PDF opens correctly, (7) Vietnamese text renders properly, (8) Compiles in <2 minutes, document in /home/khoint/thesis/deployment/docs/submission_checklist.md

**Verification**: PDF is submission-ready, all format requirements met, no compilation errors

**Time Estimate**: 30-60 minutes

---

## Dependencies Graph

```
Phase 0 (Framework Fixes) ← CRITICAL - MUST DO FIRST!
    ↓
    ├─→ Phase 1 (Setup & Verification)
    |
    ├─→ Phase 2 (Chapter 1 Content - US1) ─┐
    ├─→ Phase 3 (Chapter 2 Content - US2) ─┤
    ├─→ Phase 4 (Chapter 3 Content - US3) ─┼─→ Phase 6 (Chapter 5 - US5)
    ├─→ Phase 5 (Chapter 4 Content - US4) ─┤         ↓
    |                                        |    Phase 7 (Front Matter - US6)
    |                                        |         ↓
    └────────────────────────────────────────┴─→ Phase 8 (Bibliography - US7)
                                                      ↓
                                                 Phase 9 (Final Verification)

Phases 2-5 can be done in parallel after Phase 0 completes (independent chapters)
Phase 6 depends on Phases 2-5 (needs results to summarize)
Phase 7 depends on Phases 2-6 (needs content for abstracts)
Phase 8 depends on Phases 2-7 (needs to know what's cited)
```

---

## Parallel Execution Opportunities

### Critical Path: Phase 0 First!

**YOU MUST COMPLETE PHASE 0 BEFORE ANY CONTENT WORK**

Phase 0 tasks T001-T006 are sequential (each modifies cover page), but T007-T008 can run in parallel.

### After Phase 0 Completes (Maximum Parallelization)

You can work on **4 chapters in parallel**:

1. **Phase 2** (T011-T014) - Chapter 1: Mở Đầu - 4 sections independent
2. **Phase 3** (T015-T020) - Chapter 2: Tổng Quan - 4 sections independent
3. **Phase 4** (T021-T027) - Chapter 3: Phương Pháp - 4 sections independent
4. **Phase 5** (T028-T035) - Chapter 4: Kết Quả - 5 sections independent

**Within each chapter**, most sections can be written in parallel (marked with [P]).

### Example Parallel Workflow

**Hour 1-2**: Complete Phase 0 (framework fixes) - BLOCKING
**Hour 3-8**: Work on Phases 2-5 in parallel (4 chapters simultaneously)
**Hour 9-10**: Phase 6 (Chapter 5 - depends on results)
**Hour 11-12**: Phase 7 (Front matter - depends on content)
**Hour 13**: Phase 8 (Bibliography - depends on citations)
**Hour 14**: Phase 9 (Final verification)

**Total: ~14 hours** (with parallelization) vs **~30 hours** (sequential)

---

## MVP Scope Recommendation

**Minimum Viable Product** = Framework fixes + Chapter 1 complete

**MVP Tasks** (can submit this as Progress Report 1):
- Phase 0: T001-T008 (Framework fixes) ← CRITICAL
- Phase 1: T009-T010 (Setup)
- Phase 2: T011-T014 (Chapter 1 complete)

**Result**: A compliant thesis with Chapter 1 fully written. Total: **14 tasks, ~4-5 hours**.

**Incremental Delivery After MVP**:
1. **Progress Report 2**: Add Chapters 2-3 (Phases 3-4)
2. **Progress Report 3**: Add Chapters 4-5 (Phases 5-6)
3. **Final Submission**: Add front matter and bibliography (Phases 7-9)

---

## Task Format Validation

✅ All tasks follow checklist format: `- [ ] [TaskID] [Markers] Description with file path`

**Markers used**:
- `[P]` = Parallelizable (can run simultaneously with other tasks)
- `[US#]` = User Story reference (US1-US7)

**Example**:
- `- [ ] T015 [P] [US2] Write section 2.1 "Phương pháp của Long et al." ...`

---

## Critical Warnings

⚠️ **DO NOT SKIP PHASE 0**: Content work in Phases 2-8 will need rework if framework is wrong

⚠️ **DO NOT MODIFY EXISTING CONTENT FILES BEFORE PHASE 0**: Existing chapter files (02_chapter1_intro.tex, etc.) may have old format issues

⚠️ **DO BACKUP BEFORE STARTING**: Run T009 immediately after Phase 0 to preserve working PDF

✅ **SAFE TO PARALLELIZE**: After Phase 0, Chapters 1-4 (Phases 2-5) are independent

---

## Success Criteria Mapping

| User Story | Success Criteria | Phase | Key Tasks |
|------------|------------------|-------|-----------|
| US1 (Chapter 1) | SC-001: 5 chapters, answers 4 questions | Phase 2 | T011-T014 |
| US2 (Chapter 2) | SC-003: ≥15 references, cites Long et al. | Phase 3 | T015-T020 |
| US3 (Chapter 3) | SC-008: ≥10 hyperparameters documented | Phase 4 | T021-T027 |
| US4 (Chapter 4) | SC-006: 8 revisions, SC-007: comparison | Phase 5 | T028-T035 |
| US5 (Chapter 5) | SC-015: 3-5 contributions, 2-3 limitations | Phase 6 | T036-T039 |
| US6 (Front Matter) | SC-009: Abstracts ≤1 page, SC-010: ≥20 symbols | Phase 7 | T040-T044 |
| US7 (Bibliography) | SC-003: IEEE style, SC-012: No plagiarism | Phase 8 | T045-T047 |
| All | SC-002: Format compliance, SC-013: Compiles | Phase 0,9 | T001-T008, T048-T050 |

---

## Implementation Notes

### Priority Order

1. **Phase 0 is MANDATORY FIRST** - Framework must be correct
2. **Phases 2-5 are HIGH PRIORITY** - Main content chapters
3. **Phases 6-8 are MEDIUM PRIORITY** - Concluding sections
4. **Phase 9 is FINAL GATE** - Quality assurance

### LaTeX Best Practices

1. **Compile after each task**: Run `xelatex thesis_main.tex` to catch errors immediately
2. **Use consistent labels**: `\label{chap:intro}`, `\label{sec:motivation}`, `\label{fig:architecture}`
3. **Comment your intentions**: Use `% TODO: Add more analysis here` for future work
4. **Version control**: Commit after each completed phase with meaningful messages

### Common Issues & Solutions

- **Border not visible**: Ensure TikZ `remember picture, overlay` is used
- **Font size looks wrong**: Measure with PDF viewer zoom = 100% (not scaled)
- **Citations show ??**: Run full `make` (biber + 2x xelatex) not just xelatex
- **Vietnamese broken**: Check file encoding is UTF-8, not UTF-8-BOM

---

## Ready to Start?

**MANDATORY FIRST STEP**: Begin with **Phase 0 (T001-T008)** to fix framework formatting.

**Estimated Time by Phase**:
- Phase 0: **1-2 hours** ← DO THIS FIRST!
- Phase 1: 10 minutes
- Phase 2: 2-3 hours
- Phase 3: 4-6 hours
- Phase 4: 6-8 hours
- Phase 5: 6-8 hours
- Phase 6: 2-3 hours
- Phase 7: 2-3 hours
- Phase 8: 1-2 hours
- Phase 9: 30-60 minutes
- **Total: ~25-35 hours** (sequential) or **~15-20 hours** (with parallelization)

**Command to start Phase 0**:
```bash
cd /home/khoint/thesis/deployment/docs
# Open preamble.tex and add TikZ package
nano preamble.tex
# Start with T001: Add \usepackage{tikz}
```

---

**Tasks File Version**: 2.0 (Framework Fixes Priority)
**Updated**: 2025-11-25
**Previous Version**: 1.0 (2025-11-11 - Framework creation, now superseded)
**Ready for**: `/speckit.implement` starting with Phase 0, or manual implementation
