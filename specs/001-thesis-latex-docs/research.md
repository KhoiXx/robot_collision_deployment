# Phase 0 Research: LaTeX Thesis Framework

**Feature**: `001-thesis-latex-docs`
**Date**: 2025-11-11
**Focus**: LaTeX best practices for Vietnamese master's thesis following HCMUT standards

---

## Research Question 1: How to properly configure LaTeX for Vietnamese thesis with Times New Roman?

### Decision

Use **XeLaTeX** compiler with `fontspec` package for proper Times New Roman support in Vietnamese.

### Rationale

- **pdfLaTeX approach** (`vietnam` package + T5 encoding): Limited font support, Times New Roman may not render correctly with Vietnamese diacritics
- **XeLaTeX approach** (`fontspec` + system fonts): Full Unicode support, native Times New Roman font, better Vietnamese character rendering
- HCMUT requires Times New Roman 13pt - XeLaTeX handles this natively

### Implementation

```latex
\documentclass[a4paper,13pt]{report}
\usepackage{fontspec}
\setmainfont{Times New Roman}
\usepackage[vietnamese]{babel}
```

### Alternatives Considered

1. **pdfLaTeX with `vietnam` package**: Rejected - limited font support, encoding issues
2. **LuaLaTeX**: Similar to XeLaTeX but slower compilation, no significant advantage
3. **pdfLaTeX with `vntex`**: Rejected - outdated, poor Times New Roman rendering

### References

- XeLaTeX documentation: https://www.overleaf.com/learn/latex/XeLaTeX
- Vietnamese LaTeX guide: Use `babel` with `vietnamese` option

---

## Research Question 2: How to implement HCMUT formatting requirements (margins, spacing, numbering)?

### Decision

Use `geometry`, `setspace`, and custom configurations for HCMUT standards:
- Margins: Top 3cm, Bottom 3cm, Left 3.5cm, Right 2cm
- Line spacing: 1.5
- Chapter numbering: Vietnamese style ("CHƯƠNG 1" instead of "Chapter 1")

### Rationale

- `geometry` package: Industry standard for margin control, precise measurements
- `setspace` package: Standard for line spacing control
- Custom chapter formatting: Required to match HCMUT Vietnamese style

### Implementation

```latex
% Margins
\usepackage[top=3cm, bottom=3cm, left=3.5cm, right=2cm]{geometry}

% Line spacing
\usepackage{setspace}
\onehalfspacing  % 1.5 line spacing

% Vietnamese chapter headings
\usepackage{titlesec}
\titleformat{\chapter}[display]
  {\normalfont\huge\bfseries\centering}
  {\MakeUppercase{Chương} \thechapter}{20pt}{\Huge}
```

### Alternatives Considered

1. Manual `\vspace` commands: Rejected - inconsistent, hard to maintain
2. Different spacing package (`linespacing`): Rejected - `setspace` is standard
3. Keep default English chapter headers: Rejected - violates HCMUT requirements

---

## Research Question 3: How to handle IEEE citations with Vietnamese thesis?

### Decision

Use **`biblatex`** with `ieee` style for better Unicode and Vietnamese support.

### Rationale

- Traditional `bibtex` + `IEEEtran.bst`: Limited Unicode support, issues with Vietnamese author names
- `biblatex` with `biber` backend: Full Unicode support, better handling of Vietnamese characters
- IEEE style available: `biblatex-ieee` package provides IEEE citation format
- Numbered citations: `[1], [2], [3]...` as required by HCMUT

### Implementation

```latex
\usepackage[backend=biber, style=ieee, sorting=none]{biblatex}
\addbibresource{references.bib}

% In document
Theo nghiên cứu của Long et al. \cite{long2018towards}...

% At end
\printbibliography[title={TÀI LIỆU THAM KHẢO}]
```

### Alternatives Considered

1. **`bibtex` + `IEEEtran.bst`**: Rejected - poor Unicode support, Vietnamese author name issues
2. **`natbib` package**: Rejected - designed for author-year citations, not numbered IEEE style
3. Manual bibliography: Rejected - error-prone, no automatic numbering

### References

- `biblatex-ieee` documentation: https://ctan.org/pkg/biblatex-ieee
- IEEE citation style guide: https://ieee-dataport.org/sites/default/files/analysis/27/IEEE%20Citation%20Guidelines.pdf

---

## Research Question 4: How to structure multi-chapter thesis for maintainability?

### Decision

Use **one main file + separate chapter files** with `\input` or `\include` commands.

### Rationale

- **Maintainability**: Easy to edit individual chapters without scrolling through 100+ pages
- **Compilation speed**: Can use `\includeonly` to compile only specific chapters during drafting
- **Version control**: Git diffs are cleaner with separate files
- **Collaboration**: Multiple people can work on different chapters simultaneously

### Implementation

```latex
% thesis_main.tex
\documentclass[a4paper,13pt]{report}
\input{preamble}  % All packages and configurations

\begin{document}
\input{chapters/00_cover}
\input{chapters/01_frontmatter}
\tableofcontents
\input{chapters/02_chapter1_intro}
\input{chapters/03_chapter2_overview}
\input{chapters/04_chapter3_method}
\input{chapters/05_chapter4_results}
\input{chapters/06_chapter5_conclusion}
\printbibliography
\end{document}
```

### Alternatives Considered

1. **Single monolithic file**: Rejected - hard to navigate, slow editing, poor version control
2. **`subfiles` package**: Rejected - more complex, each chapter needs its own preamble
3. **Separate compilation per chapter**: Rejected - inconsistent numbering, references break

### Best Practice

Use `\input` (not `\include`) for better control. `\include` forces page breaks which may not be desired for front matter sections.

---

## Research Question 5: How to handle figures, tables, and equations numbering by chapter?

### Decision

Use default LaTeX numbering with `\numberwithin` command to reset counters per chapter.

### Rationale

- HCMUT requires: Hình 3.4 (Figure 4 in Chapter 3), Bảng 2.5 (Table 5 in Chapter 2), Phương trình (3.1) (Equation 1 in Chapter 3)
- `\numberwithin{figure}{chapter}` automatically implements this
- Consistent with academic thesis conventions worldwide
- Automatic cross-referencing with `\ref` and `\label`

### Implementation

```latex
\usepackage{amsmath}
\usepackage{graphicx}

% Number figures, tables, equations by chapter
\numberwithin{figure}{chapter}
\numberwithin{table}{chapter}
\numberwithin{equation}{chapter}

% Vietnamese captions
\usepackage[labelsep=period]{caption}
\captionsetup[figure]{name=Hình}
\captionsetup[table]{name=Bảng}
```

### Usage Example

```latex
\begin{figure}[htbp]
  \centering
  \includegraphics[width=0.8\textwidth]{figures/network_architecture.png}
  \caption{Kiến trúc mạng CNN với 2 lớp Conv1D}
  \label{fig:network_arch}
\end{figure}

Như thể hiện trong Hình \ref{fig:network_arch}, kiến trúc mạng...
```

### Alternatives Considered

1. Manual numbering: Rejected - error-prone, no automatic updates when chapters reorganized
2. `chngcntr` package: Deprecated, use `\numberwithin` from `amsmath` instead
3. Keep continuous numbering: Rejected - violates HCMUT requirements

---

## Research Question 6: How to create proper Table of Contents with Vietnamese headings?

### Decision

Use default `\tableofcontents` with custom formatting for Vietnamese style.

### Rationale

- Automatic generation from `\chapter`, `\section`, `\subsection` commands
- Automatic page numbering
- Clickable links with `hyperref` package
- HCMUT requirement: Maximum 4 levels of numbering (e.g., 4.1.2.1)

### Implementation

```latex
\usepackage{hyperref}
\hypersetup{
  colorlinks=true,
  linkcolor=black,
  citecolor=blue,
  urlcolor=blue,
  pdftitle={Ứng Dụng Reinforcement Learning...},
  pdfauthor={Nguyễn Tấn Khôi}
}

% Set depth for TOC (4 levels: chapter.section.subsection.subsubsection)
\setcounter{tocdepth}{3}
\setcounter{secnumdepth}{3}

% Vietnamese TOC title
\renewcommand{\contentsname}{MỤC LỤC}
\renewcommand{\listfigurename}{DANH MỤC HÌNH}
\renewcommand{\listtablename}{DANH MỤC BẢNG}
```

### Alternatives Considered

1. Manual TOC creation: Rejected - tedious, error-prone, no auto page updates
2. `tocloft` package for custom TOC: Not needed, default formatting is sufficient
3. `minitoc` for per-chapter TOC: Rejected - HCMUT doesn't require this

---

## Research Question 7: How to implement front matter (cover, abstract, acknowledgments)?

### Decision

Use custom environments and `\frontmatter`, `\mainmatter`, `\backmatter` commands for proper page numbering.

### Rationale

- **`\frontmatter`**: Roman numerals (i, ii, iii...) for preliminary pages
- **`\mainmatter`**: Arabic numerals (1, 2, 3...) starting from Chapter 1
- **`\backmatter`**: For appendices and references
- HCMUT requires specific order: Cover → Acknowledgments → Abstract (VN+EN) → Declaration → TOC → Symbols

### Implementation

```latex
\begin{document}

% Front matter (no page numbers or roman numerals)
\input{chapters/00_cover}  % Bìa chính

\frontmatter  % Start roman numbering
\input{chapters/01_frontmatter}  % Lời cảm ơn, Tóm tắt, Lời cam đoan

\tableofcontents
\listoffigures
\listoftables
\input{symbols}  % Danh mục ký hiệu

\mainmatter  % Start arabic numbering from 1
\input{chapters/02_chapter1_intro}
\input{chapters/03_chapter2_overview}
% ... other chapters

\backmatter
\printbibliography[title={TÀI LIỆU THAM KHẢO}]
\input{appendices}  % Phụ lục

\end{document}
```

### Alternatives Considered

1. Manual page numbering with `\pagenumbering`: Rejected - error-prone, breaks with reorganization
2. No distinction between frontmatter/mainmatter: Rejected - violates HCMUT format
3. Using `article` class instead of `report`: Rejected - `article` doesn't have `\chapter` command

---

## Research Question 8: How to automate compilation and dependency management?

### Decision

Use **Makefile** for automated compilation with proper dependency tracking.

### Rationale

- Multiple compilation passes needed: XeLaTeX → Biber → XeLaTeX → XeLaTeX (for cross-refs and TOC)
- Automatic detection of changes in `.tex`, `.bib`, figures
- Easy cleanup of auxiliary files
- Consistent build process across different machines

### Implementation

```makefile
# Makefile for HCMUT Thesis

MAIN = thesis_main
LATEX = xelatex
BIBER = biber

.PHONY: all clean view

all: $(MAIN).pdf

$(MAIN).pdf: $(MAIN).tex chapters/*.tex references.bib preamble.tex
	$(LATEX) $(MAIN)
	$(BIBER) $(MAIN)
	$(LATEX) $(MAIN)
	$(LATEX) $(MAIN)

clean:
	rm -f *.aux *.bbl *.bcf *.blg *.log *.out *.toc *.lof *.lot *.run.xml

view: $(MAIN).pdf
	xdg-open $(MAIN).pdf

quick:
	$(LATEX) $(MAIN)
```

### Usage

```bash
make          # Full compilation with bibliography
make clean    # Remove auxiliary files
make view     # Open PDF with default viewer
make quick    # Quick compile (skip bibliography)
```

### Alternatives Considered

1. **`latexmk`**: Excellent tool, but adds extra dependency; Makefile is more transparent
2. Manual commands: Rejected - easy to forget steps, inconsistent
3. Shell script: Makefile is more standard for academic LaTeX projects

---

## Summary of Key Decisions

| Aspect | Decision | Key Benefit |
|--------|----------|-------------|
| **Compiler** | XeLaTeX | Native Vietnamese + Times New Roman support |
| **Structure** | Main file + separate chapters | Maintainability, version control |
| **Citations** | `biblatex` with IEEE style | Unicode support, automatic numbering |
| **Margins** | `geometry` package | Precise HCMUT compliance |
| **Numbering** | `\numberwithin` per chapter | Automatic Hình 3.4, Bảng 2.5 format |
| **TOC** | Default with customization | Automatic, 4-level depth |
| **Page numbering** | `\frontmatter` / `\mainmatter` | Roman/Arabic as per HCMUT |
| **Build** | Makefile | Automated, repeatable compilation |

---

## Next Steps (Phase 1)

1. **Generate `data-model.md`**: Define document entities (Chapter, Figure, Table, Reference, Citation, Symbol)
2. **Create `quickstart.md`**: Step-by-step guide for compiling and editing the thesis
3. **Update agent context**: Add LaTeX/thesis-specific technology to context file

---

## References

- HCMUT Thesis Format Guide: `/home/khoint/thesis/deployment/docs/THESIS_FORMAT_GUIDE.md`
- LaTeX for Vietnamese: https://www.overleaf.com/learn/latex/Vietnamese
- IEEE Citation with biblatex: https://ctan.org/pkg/biblatex-ieee
- XeLaTeX documentation: https://www.overleaf.com/learn/latex/XeLaTeX

**Research Completed**: 2025-11-11
**Status**: ✅ All clarifications resolved - Ready for Phase 1 (Design & Contracts)
