# HCMUT Master's Thesis - LaTeX Documentation

**Author**: Nguyễn Tấn Khôi (MSSV: 2171017)
**Advisor**: TS. Phạm Việt Cường
**Title**: Ứng Dụng Reinforcement Learning Điều Khiển Phân Tán Hệ Đa Robot Tránh Va Chạm
**Institution**: Trường Đại học Bách Khoa - ĐHQG TP.HCM
**Major**: Kỹ thuật Điều khiển - Tự động hóa (8520216)

---

## Quick Start

### Prerequisites

Install LaTeX distribution and required packages:

```bash
# Install XeLaTeX and required packages
sudo apt-get update
sudo apt-get install texlive-xetex texlive-fonts-extra texlive-lang-other biber

# Install Times New Roman font (HCMUT requirement)
sudo apt-get install ttf-mscorefonts-installer

# Rebuild font cache
fc-cache -f -v

# Verify installation
xelatex --version
biber --version
fc-list | grep "Times New Roman"
```

### Compilation

```bash
# Navigate to thesis directory
cd /home/khoint/thesis/deployment/docs/

# Full compilation (recommended for final version)
make

# Quick compilation (draft mode, faster)
make quick

# View compiled PDF
make view

# Clean auxiliary files
make clean
```

### First-Time Setup

If this is your first time compiling the thesis:

1. **Install prerequisites** (see above)
2. **Run full compilation**: `make clean && make`
3. **Check output**: Open `thesis_main.pdf` and verify formatting
4. **Read detailed guide**: See `../specs/001-thesis-latex-docs/quickstart.md` for comprehensive instructions

---

## Project Structure

```
docs/
├── thesis_main.tex              # MAIN FILE - compile this
├── preamble.tex                 # LaTeX packages and configurations
├── Makefile                     # Build automation
├── references.bib               # Bibliography database (IEEE format)
├── symbols.tex                  # List of symbols and abbreviations
├── README.md                    # This file
│
├── chapters/                    # Thesis chapters
│   ├── 00_cover.tex             # Bìa + Trang nhiệm vụ
│   ├── 01_frontmatter.tex       # Lời cảm ơn, Tóm tắt, Lời cam đoan
│   ├── 02_chapter1_intro.tex    # Chương 1: Mở Đầu
│   ├── 03_chapter2_overview.tex # Chương 2: Tổng Quan
│   ├── 04_chapter3_method.tex   # Chương 3: Phương Pháp Nghiên Cứu
│   ├── 05_chapter4_results.tex  # Chương 4: Kết Quả và Thảo Luận
│   └── 06_chapter5_conclusion.tex # Chương 5: Kết Luận và Kiến Nghị
│
├── figures/                     # Images, diagrams, charts
│   └── README.md                # Figure guidelines
│
├── tables/                      # Table files (optional)
│
└── REFERENCE DOCUMENTS/
    ├── 1709.10082.pdf           # Original paper (Long et al., 2018)
    ├── DeCuong_NguyenTanKhoi_2171017_ver2.pdf # Thesis outline
    ├── THESIS_FORMAT_GUIDE.md   # HCMUT formatting standards
    ├── COMPARISON_WITH_ORIGINAL_PAPER.md # Technical comparison
    └── SNAPSHOT_SUMMARY.md      # Results summary
```

---

## Compilation Workflow

LaTeX requires **multiple passes** to resolve cross-references and citations:

### Full Compilation (4 steps)

```bash
make          # Runs all 4 steps automatically
```

**What happens**:
1. **XeLaTeX Pass 1**: Generate aux files, discover labels and citations
2. **Biber**: Process bibliography from `references.bib`
3. **XeLaTeX Pass 2**: Incorporate bibliography, update references
4. **XeLaTeX Pass 3**: Finalize table of contents and cross-references

### Quick Compilation (1 step)

```bash
make quick    # For rapid iteration during writing
```

Use this when:
- Editing content (not adding new citations or figures)
- Checking formatting quickly
- Writing draft text

**Always use `make` (full compilation) before:**
- Final submission
- Sharing with advisor
- Adding/changing citations or figures

---

## Editing the Thesis

### Workflow

1. **Open chapter file** in your text editor (e.g., `chapters/02_chapter1_intro.tex`)
2. **Make changes** to the content
3. **Save the file** (ensure UTF-8 encoding for Vietnamese characters)
4. **Quick compile**: `make quick`
5. **View PDF**: `make view` or open `thesis_main.pdf`
6. **Repeat** steps 2-5 as needed

### Adding Content

#### Add a Figure

```latex
\begin{figure}[htbp]
  \centering
  \includegraphics[width=0.8\textwidth]{figures/network_architecture.png}
  \caption{Kiến trúc mạng CNN với 2 lớp Conv1D}
  \label{fig:network_arch}
\end{figure}

% Reference in text:
Như thể hiện trong Hình \ref{fig:network_arch}, kiến trúc mạng...
```

#### Add a Table

```latex
\begin{table}[htbp]
  \centering
  \caption{So sánh hyperparameters với bài báo gốc}
  \label{tab:hyperparams}
  \begin{tabular}{|l|c|c|}
    \hline
    \textbf{Tham số} & \textbf{Paper} & \textbf{Luận văn} \\
    \hline
    CRITIC\_LR & 5e-5 & 6e-3 \\
    \hline
  \end{tabular}
\end{table}

% Reference: Bảng \ref{tab:hyperparams} cho thấy...
```

#### Add a Citation

1. Add entry to `references.bib`:
```bibtex
@article{author2024title,
  author = {Last, First},
  title = {Paper Title},
  journal = {Journal Name},
  year = {2024}
}
```

2. Cite in text:
```latex
Theo nghiên cứu của Author et al. \cite{author2024title}, ...
```

3. Compile with `make` (not `make quick`) to update bibliography

---

## HCMUT Formatting Requirements

The thesis **MUST** comply with HCMUT standards (enforced in `preamble.tex`):

| Requirement | Value | LaTeX Config |
|-------------|-------|--------------|
| **Font** | Times New Roman 13pt | `\setmainfont{Times New Roman}` |
| **Line Spacing** | 1.5 | `\onehalfspacing` |
| **Margins** | Top: 3cm, Bottom: 3cm, Left: 3.5cm, Right: 2cm | `geometry` package |
| **Paper Size** | A4 (210 × 297 mm) | `\documentclass[a4paper,13pt]{report}` |
| **Page Limit** | ≤100 pages (main content, excluding appendices) | Manual check |
| **Citations** | IEEE style, numbered [1], [2], ... | `biblatex` with `style=ieee` |
| **Figure Captions** | Below figure, numbered by chapter (Hình 3.4) | `caption` package |
| **Table Captions** | Above table, numbered by chapter (Bảng 2.5) | `caption` package |
| **Equation Numbers** | Right margin, by chapter (3.1) | `\numberwithin{equation}{chapter}` |

For complete formatting guidelines, see: `THESIS_FORMAT_GUIDE.md`

---

## Success Criteria (from Specification)

Track progress against these goals:

- [ ] **SC-001**: 5 chapters complete, 80-100 pages total
- [ ] **SC-002**: 100% format compliance with HCMUT standards
- [ ] **SC-003**: ≥15 references cited in IEEE format
- [ ] **SC-004**: ≥20 figures with captions and numbering
- [ ] **SC-005**: ≥5 tables with captions and numbering
- [ ] **SC-013**: LaTeX compiles successfully without errors
- [ ] **SC-014**: All images ≥300 DPI for printing

Check `../specs/001-thesis-latex-docs/spec.md` for full list.

---

## Common Issues

### Issue: "Font not found: Times New Roman"

**Solution**:
```bash
sudo apt-get install ttf-mscorefonts-installer
fc-cache -f -v
```

### Issue: Compilation errors with "undefined references"

**Solution**: Run full compilation (3 passes needed)
```bash
make clean
make
```

### Issue: Vietnamese characters display incorrectly

**Solution**: Ensure file is saved as UTF-8
```bash
# Check encoding
file -i chapters/02_chapter1_intro.tex

# Should show: charset=utf-8
```

### Issue: Bibliography not updating

**Solution**: Clean and rebuild
```bash
make clean
make
```

### Issue: Figure/table appears on wrong page

**LaTeX behavior**: Float placement is automatic. Options:
- Adjust figure size
- Use `[htbp]` for flexible placement
- Use `[H]` to force "here" (requires `\usepackage{float}`)

---

## Detailed Documentation

For comprehensive guides, see:

- **Quickstart Guide**: `../specs/001-thesis-latex-docs/quickstart.md`
  - Installation instructions
  - Compilation methods
  - Editing tips
  - Troubleshooting

- **Data Model**: `../specs/001-thesis-latex-docs/data-model.md`
  - Document entities (Chapter, Figure, Table, etc.)
  - Relationships and validation rules

- **Research Notes**: `../specs/001-thesis-latex-docs/research.md`
  - LaTeX best practices
  - Technical decisions
  - Alternative approaches

- **Implementation Tasks**: `../specs/001-thesis-latex-docs/tasks.md`
  - Phase-by-phase implementation plan
  - Task dependencies
  - Verification criteria

---

## Makefile Targets

```bash
make           # Full compilation (bibliography + 3 XeLaTeX passes)
make quick     # Quick compilation (single pass, no bibliography)
make clean     # Remove auxiliary files (.aux, .log, .bbl, etc.)
make distclean # Remove auxiliary files + PDF
make view      # Open PDF with default viewer
make count     # Count approximate words in PDF
make pages     # Show page count
make help      # Show help message
```

---

## Version Control

**Current Branch**: `001-thesis-latex-docs`

Recommended git workflow:

```bash
# After completing a chapter or major section:
git add docs/chapters/02_chapter1_intro.tex docs/figures/new_diagram.png
git commit -m "Complete Chapter 1: Introduction draft"

# Before compilation (don't commit auxiliary files):
make clean

# Commit final PDF (optional):
git add docs/thesis_main.pdf
git commit -m "Compile thesis v1.0 for advisor review"
```

**Files to ignore in git** (add to `.gitignore`):
```
*.aux
*.bbl
*.bcf
*.blg
*.log
*.out
*.toc
*.lof
*.lot
*.run.xml
*.fls
*.fdb_latexmk
*.synctex.gz
```

---

## Contact and Support

**Author**: Nguyễn Tấn Khôi
**Email**: [your email]
**Advisor**: TS. Phạm Việt Cường

**Resources**:
- LaTeX documentation: https://www.overleaf.com/learn
- XeLaTeX guide: https://www.overleaf.com/learn/latex/XeLaTeX
- Biblatex manual: https://ctan.org/pkg/biblatex
- Vietnamese LaTeX: https://www.overleaf.com/learn/latex/Vietnamese

---

**Last Updated**: 2025-11-12
**Thesis Version**: 0.1 (Framework)
**Status**: Ready for content writing
