# Implementation Plan: Tài Liệu Luận Văn Thạc Sĩ LaTeX Format Compliance

**Branch**: `001-thesis-latex-docs` | **Date**: 2025-11-24 | **Spec**: [spec.md](./spec.md)
**Input**: Fix formatting issues in LaTeX thesis document to comply with HCMUT standards

## Summary

The thesis LaTeX document currently has multiple formatting violations that prevent it from meeting HCMUT (Ho Chi Minh University of Technology) standards. This plan addresses 5 critical issues identified by the user:

1. **Missing borders on cover page** - Cover page lacks required decorative borders
2. **Incomplete cover pages** - Missing both main cover (bìa chính) and secondary cover (bìa phụ) as required
3. **Font inconsistencies** - Fonts are incorrect throughout from cover to headings (should be Times New Roman/TeX Gyre Termes 13pt with specific sizes for titles)
4. **Numbering errors** - Page numbering, section numbering, or figure/table numbering has errors
5. **Content structure issues** - Content framework needs cross-checking with outline (đề cương), research papers, and deployment snapshots

The technical approach involves systematic audit and correction of the LaTeX document structure, preamble configurations, and chapter files to ensure 100% compliance with THESIS_FORMAT_GUIDE.md.

## Technical Context

**Language/Version**: LaTeX (XeLaTeX engine), Vietnamese typesetting with Babel
**Primary Dependencies**: fontspec, babel, geometry, titlesec, biblatex (IEEE style), hyperref
**Storage**: File-based LaTeX source files in `/home/khoint/thesis/deployment/docs/`
**Testing**: Manual PDF compilation and visual inspection against THESIS_FORMAT_GUIDE.md checklist
**Target Platform**: PDF output for thesis submission (A4, Times New Roman 13pt, 1.5 line spacing)
**Project Type**: Single LaTeX document project with multiple chapter files
**Performance Goals**: N/A (document formatting project)
**Constraints**:
  - MUST comply 100% with HCMUT format guide (THESIS_FORMAT_GUIDE.md)
  - MUST use Times New Roman font (TeX Gyre Termes on Linux)
  - MUST have cover page with borders as per university template
  - MUST include both main and secondary cover pages
  - MUST use correct font sizes: 14pt for university name, 16pt for thesis title, 13pt for body
  - Page limit: ≤100 pages (excluding appendices)
  - IEEE citation style mandatory
**Scale/Scope**: ~30-page thesis document with 5 chapters, expected to grow to ~80-100 pages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Core Principle Compliance

1. **Vietnamese First Language** - ✅ PASS
   - All content in Vietnamese with English technical terms explained
   - Abstract provided in both Vietnamese and English

2. **HCMUT Standard Compliance** - ⚠️ VIOLATIONS DETECTED
   - Font configuration: TeX Gyre Termes (Times New Roman equivalent) ✅
   - Line spacing: 1.5 ✅
   - Margins: 3/3/3.5/2 cm ✅
   - Cover page borders: ❌ MISSING
   - Cover page structure: ❌ INCOMPLETE (missing secondary cover)
   - Font sizes in headings: ❌ NEEDS VERIFICATION
   - Numbering system: ❌ NEEDS VERIFICATION

3. **Standard Document Structure** - ⚠️ PARTIAL COMPLIANCE
   - Has 10 required sections ✅
   - Cover pages incomplete ❌
   - Front matter present ✅
   - 5 main chapters structure ✅

4. **Reference Paper Foundation** - ✅ PASS
   - Based on Long et al. 1709.10082.pdf ✅
   - Has reference files and comparison documents ✅

5. **Scientific Rigor** - ⚠️ NEEDS CONTENT VERIFICATION
   - Hyperparameters documentation pending verification
   - Cross-reference with snapshots pending

6. **Honesty and Objective Analysis** - ⚠️ NEEDS CONTENT VERIFICATION
   - Content structure needs cross-check with actual results

7. **Parallel Technical Documentation** - ✅ PASS
   - Thesis in Vietnamese in `/home/khoint/thesis/deployment/docs/` ✅
   - Technical docs in English in `/home/khoint/thesis/catkin_ws/rl-collision-avoidance/docs/` ✅

**GATE DECISION**: ⚠️ CONDITIONAL PASS - Violations are formatting issues that will be resolved in this implementation plan. Content verification violations are acceptable and will be addressed through research phase.

## Project Structure

### Documentation (this feature)

```text
specs/001-thesis-latex-docs/
├── plan.md              # This file (/speckit.plan command output)
├── research.md          # Phase 0 output - formatting requirements analysis
├── data-model.md        # Phase 1 output - document structure model
├── quickstart.md        # Phase 1 output - compilation guide
├── contracts/           # Phase 1 output - format validation contracts
│   └── format-compliance-checklist.md
└── tasks.md             # Phase 2 output (/speckit.tasks command - NOT created yet)
```

### Source Code (LaTeX files in repository)

```text
/home/khoint/thesis/deployment/docs/
├── thesis_main.tex           # Main LaTeX entry point
├── preamble.tex              # Package imports and configurations
├── references.bib            # Bibliography database (IEEE)
├── symbols.tex               # Symbol and abbreviation list
├── Makefile                  # Build automation
│
├── chapters/                 # Content chapters
│   ├── 00_cover.tex          # Cover page (NEEDS FIX)
│   ├── 01_frontmatter.tex    # Acknowledgments, abstracts, declaration
│   ├── 02_chapter1_intro.tex # Chapter 1: Mở đầu
│   ├── 03_chapter2_overview.tex # Chapter 2: Tổng quan
│   ├── 04_chapter3_method.tex   # Chapter 3: Phương pháp nghiên cứu
│   ├── 05_chapter4_results.tex  # Chapter 4: Kết quả và thảo luận
│   └── 06_chapter5_conclusion.tex # Chapter 5: Kết luận
│
├── figures/                  # Images and diagrams
├── tables/                   # Table data files
│
└── [Generated files]
    ├── thesis_main.pdf       # Compiled PDF output
    ├── thesis_main.aux       # Auxiliary files
    ├── thesis_main.log       # Compilation log
    └── [other auxiliary files]
```

**Structure Decision**: Single LaTeX project structure is appropriate for a thesis document. All source files are in `/home/khoint/thesis/deployment/docs/` with chapters separated into individual `.tex` files for maintainability.

## Complexity Tracking

*No complexity violations that require justification. All identified issues are formatting corrections within scope of constitution.*

---

## Phase 0: Research & Requirements Analysis

### Research Tasks

1. **Cover Page Border Requirements**
   - **Task**: Analyze HCMUT official thesis templates for cover page border specifications
   - **Sources**: THESIS_FORMAT_GUIDE.md, QDQC_huong_dan_trinh_bay_luan_van_ths_1.pdf, sample thesis templates
   - **Questions to answer**:
     - What type of border is required? (single line, double line, decorative frame?)
     - What are the border thickness and spacing requirements?
     - Are there specific LaTeX packages for thesis borders? (fancybox, tcolorbox, tikz?)
     - Does the border apply to both bìa chính (main cover) and bìa phụ (secondary cover)?

2. **Cover Page Structure Requirements**
   - **Task**: Define the difference between bìa chính and bìa phụ
   - **Questions to answer**:
     - What content goes on bìa chính vs bìa phụ?
     - Are they identical or different?
     - Does bìa phụ include the assignment page (trang nhiệm vụ)?
     - Should there be a separate title page after the cover?

3. **Font Specification Audit**
   - **Task**: Document all font size and style requirements from THESIS_FORMAT_GUIDE.md
   - **Questions to answer**:
     - Cover page: University name (14pt), Thesis title (16pt), Student info (13pt) - VERIFY
     - Chapter headings: What size? (Currently using \Huge and \Large)
     - Section headings: What size? (Currently using \Large and \large)
     - Body text: 13pt confirmed ✅
     - Are bold/italic styles correct for each element?

4. **Numbering System Verification**
   - **Task**: Audit current numbering against THESIS_FORMAT_GUIDE.md requirements
   - **Questions to answer**:
     - Page numbering: Roman numerals for front matter, Arabic for main content - CHECK
     - Section numbering: Maximum 4 digits (4.1.2.1) - VERIFY
     - Figure numbering: By chapter (Hình 3.4) - VERIFY current config
     - Table numbering: By chapter (Bảng 2.5) - VERIFY current config
     - Equation numbering: By chapter (3.1) - VERIFY current config

5. **Content Cross-Reference Analysis**
   - **Task**: Map current chapter content against required sources
   - **Sources to check**:
     - `/home/khoint/thesis/deployment/docs/DeCuong_NguyenTanKhoi_2171017_ver2.pdf` (outline)
     - `/home/khoint/thesis/deployment/docs/1709.10082.pdf` (original paper)
     - `/home/khoint/thesis/deployment/docs/COMPARISON_WITH_ORIGINAL_PAPER.md`
     - `/home/khoint/thesis/deployment/docs/SNAPSHOT_SUMMARY.md`
     - `/home/khoint/thesis/catkin_ws/rl-collision-avoidance/snapshots/` (training results)
   - **Questions to answer**:
     - Does Chapter 1 align with outline objectives?
     - Does Chapter 2 properly cite and compare with 1709.10082.pdf?
     - Does Chapter 3 match methodology from outline and actual implementation?
     - Does Chapter 4 include results from all 8 revisions mentioned in snapshots?
     - Are hyperparameters from COMPARISON_WITH_ORIGINAL_PAPER.md properly documented?

### Research Output

**Output file**: `specs/001-thesis-latex-docs/research.md`

**Expected content**:
- Section 1: Cover Page Border Specification
  - Border type and dimensions
  - LaTeX package recommendations
  - Implementation approach

- Section 2: Cover Page Structure Specification
  - Bìa chính layout and content
  - Bìa phụ layout and content
  - Assignment page (trang nhiệm vụ) requirements

- Section 3: Font Specification Matrix
  - Complete table of all document elements with required fonts and sizes
  - Comparison with current preamble.tex settings
  - Required changes

- Section 4: Numbering System Specification
  - Page numbering rules and current compliance
  - Section/subsection numbering rules
  - Figure/table/equation numbering rules
  - Required fixes

- Section 5: Content Gap Analysis
  - Chapter-by-chapter alignment with sources
  - Missing content identification
  - Content accuracy verification findings

---

## Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

### Design Artifacts

1. **Document Structure Model** (`data-model.md`)
   - Entity: ThesisDocument
     - Sections: Cover, FrontMatter, MainContent (5 chapters), BackMatter
   - Entity: Cover
     - MainCover (bìa chính) with border
     - SecondaryCover (bìa phụ) with border
     - AssignmentPage (trang nhiệm vụ)
   - Entity: Chapter
     - Number, Title, Sections[], Subsections[], Figures[], Tables[], Equations[]
   - Entity: Figure/Table
     - Number (by chapter), Caption, Source citation
   - Font specifications for each entity
   - Numbering rules for each entity

2. **Format Compliance Contract** (`contracts/format-compliance-checklist.md`)
   - Checklist of all THESIS_FORMAT_GUIDE.md requirements
   - Pass/fail criteria for each requirement
   - Validation method (visual inspection, automated check, measurement)
   - Current compliance status
   - Post-fix verification status

3. **Compilation Quickstart** (`quickstart.md`)
   - Prerequisites: XeLaTeX, fonts, packages
   - Compilation commands
   - Troubleshooting common errors
   - How to verify format compliance
   - Where to find HCMUT templates/samples

### Agent Context Update

After creating Phase 1 artifacts, run:
```bash
.specify/scripts/bash/update-agent-context.sh claude
```

This will update Claude-specific context files with:
- LaTeX packages used (fontspec, geometry, titlesec, etc.)
- HCMUT formatting requirements
- Thesis document structure patterns

---

## Phase 2: Implementation Planning (NOT EXECUTED in /speckit.plan)

Phase 2 (task generation and implementation) will be handled by `/speckit.tasks` command after this plan is approved.

Expected tasks will include:
1. Fix cover page borders
2. Create secondary cover page
3. Audit and fix all font sizes
4. Verify and fix numbering system
5. Cross-check and update chapter content
6. Compile and validate final PDF

---

## Validation Criteria

### Format Compliance
- [ ] Cover page has required border
- [ ] Both bìa chính and bìa phụ present
- [ ] All fonts are Times New Roman (TeX Gyre Termes) with correct sizes
- [ ] Page numbering: Roman (front matter), Arabic (main content)
- [ ] Section numbering max 4 digits
- [ ] Figures/tables numbered by chapter
- [ ] Equations numbered by chapter

### Content Completeness
- [ ] Chapter 1 aligns with outline objectives
- [ ] Chapter 2 cites and compares with Long et al. paper
- [ ] Chapter 3 documents all hyperparameters
- [ ] Chapter 4 includes all 8 revision results
- [ ] Chapter 5 summarizes contributions and limitations

### Technical Quality
- [ ] LaTeX compiles without errors
- [ ] PDF output is A4 with correct margins (3/3/3.5/2 cm)
- [ ] All references are IEEE style
- [ ] Total pages ≤ 100 (excluding appendices)

---

## Next Steps

1. **User Review**: User should review this plan and approve before proceeding
2. **Execute Phase 0**: Run research tasks to fill in unknown specifications
3. **Execute Phase 1**: Create data model, contracts, and quickstart guide
4. **Generate Tasks**: Run `/speckit.tasks` to create detailed implementation tasks
5. **Implement**: Execute tasks to fix all formatting and content issues
6. **Validate**: Verify all compliance criteria are met

---

**Status**: ✅ Plan ready for review
**Blocking Issues**: None - all unknowns will be resolved in Phase 0 research
**Estimated Effort**: Phase 0 (2-3 hours), Phase 1 (1-2 hours), Phase 2 (8-12 hours)
