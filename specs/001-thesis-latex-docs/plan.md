# Implementation Plan: Tài Liệu Luận Văn Thạc Sĩ LaTeX

**Branch**: `001-thesis-latex-docs` | **Date**: 2025-11-11 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/home/khoint/thesis/deployment/specs/001-thesis-latex-docs/spec.md`

**Note**: This is a short-term plan focused ONLY on creating the document framework (khung tài liệu) according to THESIS_FORMAT_GUIDE standards.

## Summary

Tạo khung tài liệu luận văn thạc sĩ bằng LaTeX theo chuẩn HCMUT về "Ứng Dụng Reinforcement Learning Điều Khiển Phân Tán Hệ Đa Robot Tránh Va Chạm". Giai đoạn này chỉ tập trung vào việc thiết lập cấu trúc ban đầu (skeleton) với tất cả các phần bắt buộc, format đúng chuẩn, và hệ thống trích dẫn IEEE. Nội dung chi tiết các chương sẽ được hoàn thiện ở giai đoạn sau.

## Technical Context

**Language/Version**: LaTeX (pdflatex hoặc xelatex), Vietnamese typesetting support
**Primary Dependencies**:
  - `texlive-full` hoặc `texlive-latex-extra` (LaTeX distribution)
  - `vietnam` package (Vietnamese language support)
  - `times` package (Times New Roman font)
  - `geometry` package (page margins)
  - `setspace` package (line spacing)
  - `graphicx` package (figures)
  - `cite` hoặc `biblatex` (IEEE citations)
  - `hyperref` package (cross-references)
  - `amsmath` package (equations)

**Storage**: File-based LaTeX project trong `/home/khoint/thesis/deployment/docs/`
**Testing**: Compile LaTeX → PDF, verify format compliance với THESIS_FORMAT_GUIDE.md
**Target Platform**: Linux (Ubuntu), output: PDF A4 paper
**Project Type**: Document (single LaTeX project với multiple chapters)
**Performance Goals**: Clean compilation < 30s, PDF size < 50MB
**Constraints**:
  - MUST follow THESIS_FORMAT_GUIDE.md 100%
  - Font: Times New Roman 13pt
  - Line spacing: 1.5
  - Margins: Top 3cm, Bottom 3cm, Left 3.5cm, Right 2cm
  - Max 100 pages (excluding appendices)
  - IEEE citation style
**Scale/Scope**:
  - 5 main chapters + front matter + references
  - Target: 80-100 pages content
  - ≥15 references
  - ≥20 figures
  - ≥5 tables

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principles Compliance

✅ **I. Tiếng Việt là Ngôn ngữ Chính**: Tất cả tài liệu luận văn sẽ được viết bằng tiếng Việt, thuật ngữ kỹ thuật tiếng Anh có giải thích lần đầu xuất hiện.

✅ **II. Tuân thủ Tiêu chuẩn HCMUT**: LaTeX template sẽ implement 100% THESIS_FORMAT_GUIDE.md (font Times New Roman 13pt, lề 3/3/3.5/2cm, line spacing 1.5, IEEE citations).

✅ **III. Cấu trúc Tài liệu theo Chuẩn**: Skeleton sẽ bao gồm đầy đủ 10 phần bắt buộc theo quy định HCMUT.

✅ **IV. Tham chiếu Bài báo Gốc**: Template sẽ chuẩn bị sẵn structure để trích dẫn bài báo gốc 1709.10082.pdf.

✅ **V. Tính Khoa học và Tái tạo**: Template sẽ có placeholders cho tables, figures, equations để đảm bảo tính khoa học.

✅ **VI. Trung thực và Phân tích Khách quan**: Cấu trúc cho phép trình bày đầy đủ kết quả, bao gồm cả thành công và thất bại.

✅ **VII. Tài liệu Kỹ thuật Song song**: Tài liệu luận văn (tiếng Việt) tách biệt với technical docs (tiếng Anh).

### Documentation Standards Compliance

✅ **Thư mục Lưu trữ**: Tất cả files sẽ được tạo trong `/home/khoint/thesis/deployment/docs/`

✅ **LaTeX được Khuyến khích**: Sử dụng LaTeX để đảm bảo format nhất quán và chuyên nghiệp.

✅ **IEEE Citation Style**: Template sẽ cấu hình sẵn IEEE style với `cite` hoặc `biblatex` package.

**GATE STATUS**: ✅ **PASSED** - No violations. All principles aligned with framework creation task.

## Project Structure

### Documentation (this feature)

```text
specs/001-thesis-latex-docs/
├── plan.md                      # This file (/speckit.plan output)
├── research.md                  # Phase 0: LaTeX best practices for thesis (will create)
├── data-model.md                # Phase 1: Document entities (Chapter, Figure, Table, etc.) (will create)
├── quickstart.md                # Phase 1: How to compile and edit the LaTeX thesis (will create)
└── checklists/
    └── requirements.md          # Validation checklist (already created)
```

### LaTeX Document Structure (repository root: /home/khoint/thesis/deployment/)

```text
docs/
├── THESIS_FORMAT_GUIDE.md       # HCMUT formatting standards (reference)
├── 1709.10082.pdf               # Original paper (reference)
├── thesis_main.tex              # MAIN FILE - compile this (to create)
├── preamble.tex                 # LaTeX packages and configurations (to create)
├── chapters/                    # Content chapters (to create)
│   ├── 00_cover.tex             # Bìa chính + Trang nhiệm vụ
│   ├── 01_frontmatter.tex       # Lời cảm ơn, Tóm tắt, Lời cam đoan
│   ├── 02_chapter1_intro.tex    # Chương 1: Mở đầu
│   ├── 03_chapter2_overview.tex # Chương 2: Tổng quan
│   ├── 04_chapter3_method.tex   # Chương 3: Phương pháp nghiên cứu
│   ├── 05_chapter4_results.tex  # Chương 4: Kết quả và thảo luận
│   └── 06_chapter5_conclusion.tex # Chương 5: Kết luận và kiến nghị
├── figures/                     # Hình ảnh, biểu đồ (to create)
│   └── README.md                # Guidelines for figures
├── tables/                      # Bảng biểu (to create)
│   └── README.md                # Guidelines for tables
├── references.bib               # IEEE bibliography (to create)
├── symbols.tex                  # Danh mục ký hiệu (to create)
├── Makefile                     # Build automation (to create)
└── README.md                    # How to compile and edit (to create)
```

**Structure Decision**:

This is a **document project** (LaTeX thesis), không phải software project. Structure được tổ chức theo HCMUT thesis standards với:
- Main file (`thesis_main.tex`) để compile
- Separate chapter files để dễ quản lý
- Dedicated folders cho figures và tables
- BibTeX file cho IEEE citations
- Makefile để automate compilation

## Complexity Tracking

**No violations** - This plan follows all constitution principles and standard LaTeX thesis practices.
