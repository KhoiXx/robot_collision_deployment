# Implementation Tasks: Tài Liệu Luận Văn Thạc Sĩ LaTeX

**Feature**: `001-thesis-latex-docs`
**Branch**: `001-thesis-latex-docs`
**Created**: 2025-11-11
**Status**: Ready for Implementation

---

## Overview

This tasks file focuses on creating the **LaTeX framework (khung tài liệu)** for the master's thesis. The goal is to establish the complete skeleton structure with proper formatting, placeholder content, and compilation infrastructure. Detailed content writing for each chapter will be done separately.

### Scope

- ✅ **In Scope**: LaTeX structure, formatting, compilation setup, placeholder chapters with example content
- ❌ **Out of Scope**: Full content writing for all chapters (this will be iterative content development)

### Implementation Strategy

**MVP Approach**: Get a compilable LaTeX document with all required sections first (Phase 1-2), then incrementally add structure for each chapter (Phase 3-9).

---

## Task Summary

| Phase | Description | Task Count | Can Parallelize? |
|-------|-------------|------------|------------------|
| Phase 1 | Setup & Directory Structure | 4 | No (sequential) |
| Phase 2 | Core LaTeX Framework | 7 | Yes (3-9 parallel) |
| Phase 3 | Front Matter Structure (US6) | 3 | Yes (partial) |
| Phase 4 | Chapter 1 Framework (US1) | 2 | Yes |
| Phase 5 | Chapter 2 Framework (US2) | 2 | Yes |
| Phase 6 | Chapter 3 Framework (US3) | 2 | Yes |
| Phase 7 | Chapter 4 Framework (US4) | 2 | Yes |
| Phase 8 | Chapter 5 Framework (US5) | 2 | Yes |
| Phase 9 | Bibliography & Symbols (US7) | 3 | Yes (partial) |
| Phase 10 | Compilation & Verification | 3 | No (sequential) |
| **TOTAL** | | **30 tasks** | |

---

## Phase 1: Setup & Directory Structure

**Goal**: Establish the base directory structure and verify prerequisites.

**Dependencies**: None (starting point)

### Tasks

- [X] T001 Verify LaTeX installation and Vietnamese font support on system (run `xelatex --version` and `fc-list | grep "Times New Roman"`) - NOTE: LaTeX not installed, will need installation later
- [X] T002 Create docs/ directory structure in /home/khoint/thesis/deployment/docs/
- [X] T003 Create chapters/ subdirectory in /home/khoint/thesis/deployment/docs/chapters/
- [X] T004 Create figures/ and tables/ subdirectories in /home/khoint/thesis/deployment/docs/

**Verification**: Run `tree /home/khoint/thesis/deployment/docs/` and verify directory structure matches plan.md

---

## Phase 2: Core LaTeX Framework (Foundational)

**Goal**: Create the main LaTeX files that all chapters depend on.

**Dependencies**: Phase 1 complete

**Blocking**: All chapter phases (3-8) require these files to exist

### Tasks

- [X] T005 Create preamble.tex with XeLaTeX config, Times New Roman font, Vietnamese babel, geometry (3/3/3.5/2cm margins), setspace (1.5), graphicx, amsmath, and biblatex IEEE style in /home/khoint/thesis/deployment/docs/preamble.tex
- [X] T006 Create thesis_main.tex with documentclass[a4paper,13pt]{report}, input preamble, frontmatter/mainmatter/backmatter structure, and placeholder chapter inputs in /home/khoint/thesis/deployment/docs/thesis_main.tex
- [X] T007 [P] Create Makefile with targets for full compile (xelatex + biber + 2x xelatex), quick compile, clean, and view in /home/khoint/thesis/deployment/docs/Makefile
- [X] T008 [P] Create references.bib with initial entry for Long et al. 2018 paper (1709.10082.pdf) using BibTeX IEEE format in /home/khoint/thesis/deployment/docs/references.bib
- [X] T009 [P] Create symbols.tex with chapter*{DANH MỤC KÝ HIỆU VÀ CHỮ VIẾT TẮT} and description list for PPO, RL, LiDAR, GAE, CNN, DRL placeholders in /home/khoint/thesis/deployment/docs/symbols.tex
- [X] T010 [P] Create figures/README.md with guidelines: use descriptive filenames, ≥300 DPI, preferred formats (PNG/PDF), caption conventions in /home/khoint/thesis/deployment/docs/figures/README.md
- [X] T011 [P] Create README.md symlinking to quickstart.md with compilation instructions and file structure overview in /home/khoint/thesis/deployment/docs/README.md

**Verification**: Run `make clean && make` - should compile without errors to produce thesis_main.pdf (even if empty) - NOTE: Requires LaTeX installation first

**Parallel Execution**: Tasks T007-T011 can be done in parallel (different files, no dependencies)

---

## Phase 3: Front Matter Structure (User Story 6 - P6)

**Goal**: Create LaTeX files for cover page, acknowledgments, abstracts, and declaration.

**Dependencies**: Phase 2 complete

**User Story**: US6 - Hoàn thành Các Phần Đầu Luận Văn

**Independent Test**: Compile thesis and verify: (1) Cover page has all required fields, (2) Abstracts ≤ 1 page each, (3) TOC auto-generates, (4) Symbols list present

### Tasks

- [ ] T012 [P] [US6] Create chapters/00_cover.tex with title page structure: school name (font 14), student name (Nguyễn Tấn Khôi, 2171017), thesis title (font 16), major, advisor (TS. Phạm Việt Cường), submission date placeholders in /home/khoint/thesis/deployment/docs/chapters/00_cover.tex
- [ ] T013 [US6] Create chapters/01_frontmatter.tex with sections for Lời cảm ơn, Tóm tắt (VN), Abstract (EN), and Lời cam đoan with placeholder Vietnamese text (≤1 page each) in /home/khoint/thesis/deployment/docs/chapters/01_frontmatter.tex
- [ ] T014 [US6] Update thesis_main.tex to include cover, frontmatter, TOC (\tableofcontents, \listoffigures, \listoftables), and symbols.tex in correct order with \frontmatter and \mainmatter commands in /home/khoint/thesis/deployment/docs/thesis_main.tex

**Verification**: Compile and check PDF has: cover page, acknowledgments, VN abstract, EN abstract, declaration, TOC, list of figures/tables, symbols (all with correct page numbering: roman for front matter)

**Parallel Execution**: T012 can be done in parallel with T013 (different files)

---

## Phase 4: Chapter 1 Framework (User Story 1 - P1)

**Goal**: Create LaTeX skeleton for Chương 1: Mở Đầu with sections and placeholder content.

**Dependencies**: Phase 2 complete

**User Story**: US1 - Hoàn thành Chương 1: Mở Đầu

**Independent Test**: Compile Chapter 1 and verify it answers 4 questions: (1) Why this topic? (2) Research objectives? (3) Scope? (4) Scientific significance?

### Tasks

- [ ] T015 [P] [US1] Create chapters/02_chapter1_intro.tex with \chapter{MỞ ĐẦU} and 4 sections: 1.1 Lý do chọn đề tài, 1.2 Mục tiêu nghiên cứu, 1.3 Đối tượng và phạm vi nghiên cứu, 1.4 Ý nghĩa khoa học và thực tiễn with placeholder Vietnamese content (2-3 paragraphs each) in /home/khoint/thesis/deployment/docs/chapters/02_chapter1_intro.tex
- [ ] T016 [US1] Update thesis_main.tex to input chapters/02_chapter1_intro.tex after \mainmatter command in /home/khoint/thesis/deployment/docs/thesis_main.tex

**Verification**: Compile and verify Chapter 1 appears with: correct chapter numbering, 4 sections, intro paragraph, conclusion paragraph, Vietnamese text

**Parallel Execution**: T015 can start immediately after Phase 2

---

## Phase 5: Chapter 2 Framework (User Story 2 - P2)

**Goal**: Create LaTeX skeleton for Chương 2: Tổng Quan with sections for literature review.

**Dependencies**: Phase 2 complete (independent of Phase 4)

**User Story**: US2 - Hoàn thành Chương 2: Tổng Quan

**Independent Test**: Verify Chapter 2 has: ≥15 citations including Long et al. 2018, comparison of ≥3 methods, clear research gap identified

### Tasks

- [ ] T017 [P] [US2] Create chapters/03_chapter2_overview.tex with \chapter{TỔNG QUAN} and 4 sections: 2.1 Phương pháp của Long et al., 2.2 So sánh các phương pháp tránh va chạm, 2.3 Cơ sở lý thuyết (RL, PPO, CNN), 2.4 Đánh giá tình hình nghiên cứu with placeholder citations (\cite{long2018towards}) and Vietnamese content in /home/khoint/thesis/deployment/docs/chapters/03_chapter2_overview.tex
- [ ] T018 [US2] Update thesis_main.tex to input chapters/03_chapter2_overview.tex after Chapter 1 and add 5+ placeholder references to references.bib (Schulman PPO 2017, Mnih A3C 2016, etc.) in /home/khoint/thesis/deployment/docs/thesis_main.tex and /home/khoint/thesis/deployment/docs/references.bib

**Verification**: Compile and verify Chapter 2: correct numbering, 4 sections, citations appear as [1], [2], bibliography auto-generates at end

**Parallel Execution**: T017 can start immediately after Phase 2 (parallel with Phase 4)

---

## Phase 6: Chapter 3 Framework (User Story 3 - P3)

**Goal**: Create LaTeX skeleton for Chương 3: Phương Pháp Nghiên Cứu with sections for methodology.

**Dependencies**: Phase 2 complete (independent of Phases 4-5)

**User Story**: US3 - Hoàn thành Chương 3: Phương Pháp Nghiên Cứu

**Independent Test**: Verify Chapter 3 describes: observation space (3×454), action space [v,ω], reward function (5 components), network architecture (4 layers), hyperparameters table

### Tasks

- [ ] T019 [P] [US3] Create chapters/04_chapter3_method.tex with \chapter{PHƯƠNG PHÁP NGHIÊN CỨU} and 5 sections: 3.1 Môi trường mô phỏng (POMDP), 3.2 Kiến trúc mạng nơ-ron, 3.3 Cải tiến so với bài báo gốc, 3.4 Quy trình huấn luyện, 3.5 Thiết kế robot thực tế with placeholder equations, figures, tables in /home/khoint/thesis/deployment/docs/chapters/04_chapter3_method.tex
- [ ] T020 [US3] Add example equation for reward function, example figure placeholder for network architecture, example table for hyperparameters to chapters/04_chapter3_method.tex using \begin{equation}, \begin{figure}, \begin{table} environments in /home/khoint/thesis/deployment/docs/chapters/04_chapter3_method.tex

**Verification**: Compile and verify Chapter 3: equations numbered (3.1), figures numbered (Hình 3.1), tables numbered (Bảng 3.1), auto-numbering works

**Parallel Execution**: T019 can start immediately after Phase 2 (parallel with Phases 4-5)

---

## Phase 7: Chapter 4 Framework (User Story 4 - P4)

**Goal**: Create LaTeX skeleton for Chương 4: Kết Quả và Thảo Luận with sections for results.

**Dependencies**: Phase 2 complete (independent of Phases 4-6)

**User Story**: US4 - Hoàn thành Chương 4: Kết Quả và Thảo Luận

**Independent Test**: Verify Chapter 4 reports: Stage 1 results (74%), Stage 2 results (71% train, 88% test), comparison with paper (88% vs 96.5%), 8 revisions analysis, training curves

### Tasks

- [ ] T021 [P] [US4] Create chapters/05_chapter4_results.tex with \chapter{KẾT QUẢ VÀ THẢO LUẬN} and 5 sections: 4.1 Kết quả Stage 1, 4.2 Kết quả Stage 2, 4.3 So sánh với bài báo gốc, 4.4 Phân tích các cải tiến, 4.5 Thảo luận with placeholder figures for training curves and tables for results comparison in /home/khoint/thesis/deployment/docs/chapters/05_chapter4_results.tex
- [ ] T022 [US4] Add 3+ example figures (training curves, success rate plots) and 2+ tables (hyperparameters comparison, results summary) with placeholder captions to chapters/05_chapter4_results.tex in /home/khoint/thesis/deployment/docs/chapters/05_chapter4_results.tex

**Verification**: Compile and verify Chapter 4: multiple figures numbered sequentially (Hình 4.1, 4.2, 4.3), tables numbered (Bảng 4.1, 4.2), cross-references work with \ref{}

**Parallel Execution**: T021 can start immediately after Phase 2 (parallel with Phases 4-6)

---

## Phase 8: Chapter 5 Framework (User Story 5 - P5)

**Goal**: Create LaTeX skeleton for Chương 5: Kết Luận và Kiến Nghị with summary sections.

**Dependencies**: Phase 2 complete (independent of Phases 4-7)

**User Story**: US5 - Hoàn thành Chương 5: Kết Luận và Kiến Nghị

**Independent Test**: Verify Chapter 5 summarizes: 3-5 main contributions, 2-3 limitations, 2-3 future directions, no new content (only summary)

### Tasks

- [ ] T023 [P] [US5] Create chapters/06_chapter5_conclusion.tex with \chapter{KẾT LUẬN VÀ KIẾN NGHỊ} and 4 sections: 5.1 Tóm tắt kết quả đạt được, 5.2 Hạn chế của nghiên cứu, 5.3 Hướng nghiên cứu tiếp theo, 5.4 Lời kết with placeholder Vietnamese summary content in /home/khoint/thesis/deployment/docs/chapters/06_chapter5_conclusion.tex
- [ ] T024 [US5] Update thesis_main.tex to input all chapter files (02-06) in order after \mainmatter in /home/khoint/thesis/deployment/docs/thesis_main.tex

**Verification**: Compile and verify: all 5 chapters appear in order, continuous page numbering, TOC shows all chapters and sections correctly

**Parallel Execution**: T023 can start immediately after Phase 2 (parallel with Phases 4-7)

---

## Phase 9: Bibliography & Symbols Completion (User Story 7 - P7)

**Goal**: Populate references.bib with ≥15 entries and symbols.tex with ≥20 terms.

**Dependencies**: Phases 3-8 complete (need to know what citations and symbols are used)

**User Story**: US7 - Hoàn thành Danh Mục Tài Liệu Tham Khảo

**Independent Test**: Verify: all [n] in text have reference entries, all reference entries are cited, IEEE format correct

### Tasks

- [ ] T025 [P] [US7] Add 15+ BibTeX entries to references.bib: Long 2018, Schulman PPO 2017, Schulman GAE 2015, Kingma Adam 2015, Mnih A3C 2016, Bengio curriculum 2009, and 9+ others related to RL, multi-robot, collision avoidance in IEEE format in /home/khoint/thesis/deployment/docs/references.bib
- [ ] T026 [P] [US7] Populate symbols.tex with 20+ entries: PPO, RL, LiDAR, GAE, CNN, DRL, POMDP, and mathematical symbols (v, ω, r_t, etc.) with Vietnamese meanings, sorted alphabetically in /home/khoint/thesis/deployment/docs/symbols.tex
- [ ] T027 [US7] Update thesis_main.tex to add \printbibliography[title={TÀI LIỆU THAM KHẢO}] in \backmatter section in /home/khoint/thesis/deployment/docs/thesis_main.tex

**Verification**: Compile and verify: bibliography appears at end with IEEE format [1], [2]..., all citations resolve, symbols list has ≥20 terms alphabetically

**Parallel Execution**: T025 and T026 can be done in parallel (different files)

---

## Phase 10: Compilation & Verification

**Goal**: Ensure the complete thesis compiles cleanly and meets HCMUT format requirements.

**Dependencies**: All phases 1-9 complete

### Tasks

- [ ] T028 Run `make clean && make` and verify compilation completes without errors, produces thesis_main.pdf with all sections in /home/khoint/thesis/deployment/docs/
- [ ] T029 Verify PDF format compliance: check font (Times New Roman 13pt), margins (3/3/3.5/2cm), line spacing (1.5), page numbering (Roman for front matter, Arabic for main content) using PDF viewer and ruler
- [ ] T030 Create quality checklist review: verify ≥15 references, ≥5 chapters, TOC auto-generated, figures/tables numbered correctly, no ?? symbols (undefined references), file compiles in < 30s

**Verification**: PDF opens correctly, all pages render, format matches THESIS_FORMAT_GUIDE.md, ready for content population

---

## Dependencies Graph

```
Phase 1 (Setup)
    ↓
Phase 2 (Core Framework) ← BLOCKING for all chapters
    ↓
    ├─→ Phase 3 (Front Matter - US6)
    ├─→ Phase 4 (Chapter 1 - US1) ─┐
    ├─→ Phase 5 (Chapter 2 - US2) ─┤
    ├─→ Phase 6 (Chapter 3 - US3) ─┼─→ Phase 9 (Bibliography & Symbols - US7)
    ├─→ Phase 7 (Chapter 4 - US4) ─┤         ↓
    └─→ Phase 8 (Chapter 5 - US5) ─┘    Phase 10 (Verification)

Phases 3-8 can be done in parallel after Phase 2 completes
```

---

## Parallel Execution Opportunities

### After Phase 2 Completes (Maximum Parallelization)

You can work on **6 tasks in parallel**:

1. **T012** - Create cover page (00_cover.tex)
2. **T015** - Create Chapter 1 skeleton (02_chapter1_intro.tex)
3. **T017** - Create Chapter 2 skeleton (03_chapter2_overview.tex)
4. **T019** - Create Chapter 3 skeleton (04_chapter3_method.tex)
5. **T021** - Create Chapter 4 skeleton (05_chapter4_results.tex)
6. **T023** - Create Chapter 5 skeleton (06_chapter5_conclusion.tex)

These are all **independent files** with no dependencies on each other.

### Within Phase 2 (Foundational)

After T005-T006 complete, you can work on **5 tasks in parallel**:

1. **T007** - Create Makefile
2. **T008** - Create references.bib
3. **T009** - Create symbols.tex
4. **T010** - Create figures/README.md
5. **T011** - Create docs/README.md

---

## MVP Scope Recommendation

**Minimum Viable Product** = Get a compilable LaTeX thesis skeleton

**MVP Tasks** (can ship this first):
- Phase 1: T001-T004 (Setup)
- Phase 2: T005-T011 (Core framework)
- Phase 3: T012-T014 (Front matter)
- Phase 10: T028-T030 (Verification)

**Result**: A compilable thesis with cover, front matter, TOC, and empty chapter placeholders. Total: **18 tasks**.

**Incremental Delivery**: After MVP, add chapters one by one (Phases 4-8), then complete bibliography (Phase 9).

---

## Task Format Validation

✅ All tasks follow checklist format: `- [ ] [TaskID] [Markers] Description with file path`

**Markers used**:
- `[P]` = Parallelizable (can run simultaneously with other [P] tasks in same phase)
- `[US#]` = User Story reference (US1 = Chapter 1, US2 = Chapter 2, etc.)

**Example**:
- `- [ ] T012 [P] [US6] Create chapters/00_cover.tex with title page structure...`

---

## Success Criteria Mapping

| User Story | Success Criteria | Tasks |
|------------|------------------|-------|
| US1 (Chapter 1) | SC-001: 5 chapters complete | T015-T016 |
| US2 (Chapter 2) | SC-003: ≥15 references | T017-T018, T025 |
| US3 (Chapter 3) | SC-008: ≥10 hyperparameters | T019-T020 |
| US4 (Chapter 4) | SC-004: ≥20 figures, SC-005: ≥5 tables, SC-006: 8 revisions | T021-T022 |
| US5 (Chapter 5) | SC-015: 3-5 contributions, 2-3 limitations | T023-T024 |
| US6 (Front Matter) | SC-009: Abstracts ≤1 page, SC-011: Auto TOC | T012-T014 |
| US7 (Bibliography) | SC-003: IEEE citations, SC-010: ≥20 symbols | T025-T027 |
| All | SC-002: Format compliance, SC-013: Compiles without errors | T005-T006, T028-T030 |

---

## Implementation Notes

### LaTeX Best Practices

1. **Compile frequently**: Run `make quick` after each file change to catch errors early
2. **Use labels consistently**: `\label{chap:intro}`, `\label{sec:motivation}`, `\label{fig:architecture}`, `\label{tab:results}`
3. **Comment your LaTeX**: Use `%` for TODOs, notes, and explanations
4. **Version control**: Commit after each completed phase

### Common Issues

- **Font not found**: Run `fc-cache -f -v` after installing Times New Roman
- **Undefined references**: Run full `make` (not `make quick`) to resolve cross-references
- **Vietnamese characters broken**: Ensure all .tex files are saved as UTF-8 encoding
- **Bibliography not updating**: Run `make clean && make` to rebuild from scratch

### Testing Strategy

After each phase:
1. Run `make clean && make`
2. Open thesis_main.pdf
3. Verify new sections appear correctly
4. Check page numbers, TOC, cross-references

---

## Ready to Start?

**Next Step**: Begin with **Phase 1 (T001-T004)** to set up the directory structure.

**Estimated Time**:
- Phase 1: 10 minutes
- Phase 2: 30 minutes
- Phases 3-8: 15 minutes each (90 minutes total, or 15 minutes if parallelized)
- Phase 9: 30 minutes
- Phase 10: 15 minutes
- **Total: ~3 hours** (sequential) or **~1.5 hours** (with parallelization)

**Command to start**:
```bash
cd /home/khoint/thesis/deployment
# Ready to begin T001: Verify LaTeX installation
xelatex --version
fc-list | grep "Times New Roman"
```

---

**Tasks File Version**: 1.0
**Generated**: 2025-11-11
**Ready for**: `/speckit.implement` or manual implementation
