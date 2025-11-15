# Quickstart Guide: LaTeX Thesis Compilation & Editing

**Feature**: `001-thesis-latex-docs`
**Date**: 2025-11-11
**Audience**: Student (Nguyễn Tấn Khôi) and anyone editing the thesis

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [First-Time Setup](#first-time-setup)
3. [Compiling the Thesis](#compiling-the-thesis)
4. [Editing the Thesis](#editing-the-thesis)
5. [Adding Figures and Tables](#adding-figures-and-tables)
6. [Managing References](#managing-references)
7. [Common Issues and Solutions](#common-issues-and-solutions)
8. [Quality Checklist](#quality-checklist)

---

## Prerequisites

### Required Software

1. **LaTeX Distribution**:
   ```bash
   # Ubuntu/Debian
   sudo apt-get update
   sudo apt-get install texlive-full

   # OR minimal installation (faster, smaller)
   sudo apt-get install texlive-latex-extra texlive-fonts-extra \
                        texlive-xetex texlive-lang-other biber
   ```

2. **Text Editor** (choose one):
   - **VS Code** (recommended): Install "LaTeX Workshop" extension
   - **Overleaf**: Online editor, no installation needed
   - **TeXstudio**: Dedicated LaTeX IDE
   - **Vim/Emacs**: With LaTeX plugins

3. **PDF Viewer**:
   ```bash
   sudo apt-get install evince  # GNOME Document Viewer
   # OR
   sudo apt-get install okular  # KDE PDF viewer
   ```

4. **Make** (build automation):
   ```bash
   sudo apt-get install build-essential
   ```

### Verify Installation

```bash
# Check XeLaTeX
xelatex --version

# Check Biber (for bibliography)
biber --version

# Check Make
make --version
```

Expected output: Version numbers without errors.

---

## First-Time Setup

### 1. Navigate to Thesis Directory

```bash
cd /home/khoint/thesis/deployment/docs/
```

### 2. Verify Directory Structure

```bash
tree -L 2 .
```

Expected structure:
```
docs/
├── thesis_main.tex       # Main file (compile this)
├── preamble.tex          # Packages and configs
├── chapters/             # Chapter files
│   ├── 00_cover.tex
│   ├── 01_frontmatter.tex
│   ├── 02_chapter1_intro.tex
│   ├── 03_chapter2_overview.tex
│   ├── 04_chapter3_method.tex
│   ├── 05_chapter4_results.tex
│   └── 06_chapter5_conclusion.tex
├── figures/              # Image files
├── tables/               # (optional, can be inline)
├── references.bib        # Bibliography database
├── symbols.tex           # List of acronyms
├── Makefile              # Build automation
└── README.md             # This guide (symlink)
```

### 3. Install Vietnamese Font Support

```bash
# Install Times New Roman font (if not already installed)
sudo apt-get install ttf-mscorefonts-installer

# Or manually download and install Times New Roman
# Check if font is available:
fc-list | grep "Times New Roman"
```

Expected: You should see Times New Roman listed.

---

## Compiling the Thesis

### Method 1: Using Make (Recommended)

```bash
cd /home/khoint/thesis/deployment/docs/

# Full compilation (includes bibliography)
make

# Quick compilation (skip bibliography updates)
make quick

# View the PDF
make view

# Clean auxiliary files
make clean
```

**When to use what**:
- `make`: First compile, or after changing `references.bib`, or after adding new citations
- `make quick`: Quick preview during editing, no bibliography changes
- `make clean`: Before committing to git, or if compilation errors occur

### Method 2: Manual Compilation

```bash
cd /home/khoint/thesis/deployment/docs/

# Step 1: First pass (generate aux files)
xelatex thesis_main.tex

# Step 2: Process bibliography
biber thesis_main

# Step 3: Second pass (resolve references)
xelatex thesis_main.tex

# Step 4: Third pass (resolve cross-references, TOC)
xelatex thesis_main.tex
```

**Why 4 steps?**
- Pass 1: LaTeX discovers labels and citations
- Pass 2: Biber processes bibliography from `references.bib`
- Pass 3: LaTeX incorporates bibliography and updates references
- Pass 4: LaTeX finalizes table of contents and cross-references

### Method 3: Using VS Code LaTeX Workshop

1. Open `thesis_main.tex` in VS Code
2. Press `Ctrl+Alt+B` (or `Cmd+Option+B` on Mac) to build
3. Press `Ctrl+Alt+V` (or `Cmd+Option+V` on Mac) to view PDF
4. LaTeX Workshop auto-detects XeLaTeX and runs multiple passes

**Configuration** (in `.vscode/settings.json`):
```json
{
  "latex-workshop.latex.recipes": [
    {
      "name": "xelatex -> biber -> xelatex*2",
      "tools": ["xelatex", "biber", "xelatex", "xelatex"]
    }
  ],
  "latex-workshop.latex.tools": [
    {
      "name": "xelatex",
      "command": "xelatex",
      "args": ["-synctex=1", "-interaction=nonstopmode", "-file-line-error", "%DOC%"]
    },
    {
      "name": "biber",
      "command": "biber",
      "args": ["%DOCFILE%"]
    }
  ]
}
```

---

## Editing the Thesis

### Basic Workflow

1. **Open the chapter file** you want to edit (e.g., `chapters/02_chapter1_intro.tex`)
2. **Make changes** to the content
3. **Save the file** (Ctrl+S)
4. **Recompile**: `make quick` or press compile button in your editor
5. **View PDF** to verify changes

### Editing Tips

#### Use Comments Liberally

```latex
% TODO: Add more details about reward function
% FIXME: Check this citation is correct
% NOTE: This section needs advisor review
```

#### Structure Your Content

```latex
\section{Lý do chọn đề tài}
\label{sec:motivation}

% Introduce the problem
Trong những năm gần đây, điều khiển đa robot...

% Explain why this problem matters
Vấn đề này đặc biệt quan trọng vì...

% State the gap
Tuy nhiên, các phương pháp hiện tại còn hạn chế...
```

#### Vietnamese Typing

```latex
% Good: Full Vietnamese with diacritics
Việc áp dụng học tăng cường vào điều khiển đa robot...

% Bad: No diacritics (hard to read)
Viec ap dung hoc tang cuong vao dieu khien da robot...

% Technical terms: English with Vietnamese explanation
Thuật toán Proximal Policy Optimization (PPO) là một phương pháp...
% First mention: include Vietnamese
% Later mentions: can use just "PPO"
```

---

## Adding Figures and Tables

### Adding a Figure

1. **Save image file** to `figures/` directory:
   ```bash
   cp ~/Downloads/network_architecture.png docs/figures/
   ```

2. **Insert in chapter**:
   ```latex
   \begin{figure}[htbp]
     \centering
     \includegraphics[width=0.8\textwidth]{figures/network_architecture.png}
     \caption{Kiến trúc mạng CNN với 2 lớp Conv1D}
     \label{fig:network_arch}
   \end{figure}
   ```

3. **Reference in text**:
   ```latex
   Như thể hiện trong Hình \ref{fig:network_arch}, kiến trúc mạng bao gồm...
   ```

4. **Recompile** (run twice for references to update):
   ```bash
   make quick
   ```

**Figure best practices**:
- Use descriptive filenames: `network_architecture.png` not `fig1.png`
- Preferred formats: PNG (photos), PDF (diagrams), SVG (converted to PDF)
- Resolution: ≥300 DPI for print quality
- Width: Usually `0.8\textwidth` or `0.6\textwidth`
- Placement: `[htbp]` = here, top, bottom, page (LaTeX chooses best)

### Adding a Table

```latex
\begin{table}[htbp]
  \centering
  \caption{So sánh hyperparameters với bài báo gốc}
  \label{tab:hyperparams}
  \begin{tabular}{|l|c|c|}
    \hline
    \textbf{Tham số} & \textbf{Bài báo gốc} & \textbf{Luận văn này} \\
    \hline
    CRITIC\_LR & 5e-5 & 6e-3 \\
    ACTOR\_LR & 5e-5 & 4e-4 \\
    Số robots & 4-20 & 44-58 \\
    \hline
  \end{tabular}
\end{table}
```

**Reference in text**:
```latex
Bảng \ref{tab:hyperparams} cho thấy sự khác biệt về hyperparameters...
```

**Table tips**:
- Use online tools to generate LaTeX tables: https://www.tablesgenerator.com/latex_tables
- For complex tables, consider using `booktabs` package for better styling
- Caption goes **above** table (opposite of figures)

---

## Managing References

### Adding a New Reference

1. **Open `references.bib`**

2. **Add entry in BibTeX format**:
   ```bibtex
   @inproceedings{long2018towards,
     author    = {Long, Pinxin and Fan, Tingxiang and Liao, Xinyi and Liu, Wenxi and Zhang, Hao and Pan, Jia},
     title     = {Towards optimally decentralized multi-robot collision avoidance via deep reinforcement learning},
     booktitle = {Proc. IEEE Int. Conf. Robot. Autom. (ICRA)},
     year      = {2018},
     pages     = {6252--6259},
     doi       = {10.1109/ICRA.2018.8461113}
   }
   ```

3. **Cite in text**:
   ```latex
   Theo nghiên cứu của Long et al. \cite{long2018towards}, phương pháp PPO...
   ```

4. **Recompile with bibliography**:
   ```bash
   make  # Full compilation including biber
   ```

### Finding BibTeX Entries

**Google Scholar**:
1. Search for the paper
2. Click "Cite" → "BibTeX"
3. Copy the entry

**arXiv**:
1. On paper page, click "Export citation"
2. Select "BibTeX"

**IEEE Xplore**:
1. Click "Cite This" on paper page
2. Select "BibTeX" format

### Citation Styles

```latex
% Single citation
According to \cite{long2018towards}, ...

% Multiple citations
Several studies \cite{long2018towards,schulman2017proximal,mnih2016asynchronous} have shown...

% Citation with page number
As stated in \cite[p. 6255]{long2018towards}, ...

% Citation in parentheses
This has been demonstrated \cite{long2018towards}.
```

---

## Common Issues and Solutions

### Issue 1: "Font not found: Times New Roman"

**Symptom**: Compilation fails with font error.

**Solution**:
```bash
# Install Microsoft Core Fonts
sudo apt-get install ttf-mscorefonts-installer

# Rebuild font cache
fc-cache -f -v

# Verify
fc-list | grep "Times New Roman"
```

**Alternative**: Use a different serif font:
```latex
% In preamble.tex, replace:
\setmainfont{Times New Roman}
% With:
\setmainfont{TeX Gyre Termes}  % Times New Roman clone
```

### Issue 2: "Undefined references"

**Symptom**: Question marks `??` appear instead of figure/table/citation numbers.

**Solution**: Run **multiple compilation passes**:
```bash
make clean
make
```

LaTeX needs 2-3 passes to resolve all references.

### Issue 3: "Bibliography not updating"

**Symptom**: New citations don't appear in bibliography.

**Solution**:
```bash
# Clean and rebuild
make clean
make  # This runs biber automatically
```

**Check**:
```bash
# Look for errors in biber log
cat thesis_main.blg
```

### Issue 4: "Vietnamese characters display incorrectly"

**Symptom**: Diacritics appear as boxes or wrong characters.

**Solution**:
1. Ensure file encoding is **UTF-8**:
   ```bash
   file -i chapters/02_chapter1_intro.tex
   ```
   Should show: `charset=utf-8`

2. In VS Code, check bottom-right corner shows "UTF-8"

3. If not UTF-8, convert:
   ```bash
   iconv -f ISO-8859-1 -t UTF-8 input.tex > output.tex
   ```

### Issue 5: Compilation is slow (>1 minute)

**Symptom**: `make` takes a long time.

**Solutions**:
- Use `make quick` for draft mode (skips bibliography)
- Use `\includeonly` in `thesis_main.tex` to compile only specific chapters:
  ```latex
  % In thesis_main.tex
  \includeonly{chapters/02_chapter1_intro}  % Only compile Chapter 1
  ```
- Reduce image resolution for draft mode

### Issue 6: "Too many pages" (>100 pages)

**Symptom**: Content exceeds 100 pages.

**Solutions**:
1. Move detailed results to appendices
2. Reduce figure sizes: `width=0.6\textwidth` instead of `0.8`
3. Condense tables (use smaller font, multi-line cells)
4. Review and tighten text (remove redundancy)

---

## Quality Checklist

Before submitting, verify:

### Format Compliance

- [ ] Font is Times New Roman 13pt throughout
- [ ] Line spacing is 1.5
- [ ] Margins: Top 3cm, Bottom 3cm, Left 3.5cm, Right 2cm
- [ ] Page numbers correct (Roman for front matter, Arabic for main content)
- [ ] Total pages ≤ 100 (excluding appendices)

### Structure

- [ ] Cover page has all required info (name, ID, advisor, date)
- [ ] Front matter complete (acknowledgments, abstracts, TOC, symbols)
- [ ] All 5 chapters present: Mở đầu, Tổng quan, Phương pháp, Kết quả, Kết luận
- [ ] Each chapter has intro and conclusion paragraphs
- [ ] Bibliography present with IEEE format

### Content

- [ ] All technical terms explained in Vietnamese at first use
- [ ] No spelling errors (run spell check)
- [ ] No "TODO" or "FIXME" comments left in final version
- [ ] All figures have captions below
- [ ] All tables have captions above
- [ ] All equations numbered
- [ ] ≥15 references cited
- [ ] ≥20 figures
- [ ] ≥5 tables

### Cross-References

- [ ] No `??` symbols (undefined references)
- [ ] All `\ref{...}` labels point to existing `\label{...}`
- [ ] All `\cite{...}` keys exist in `references.bib`
- [ ] Table of Contents matches actual chapters
- [ ] List of Figures/Tables match actual figures/tables

### Compilation

- [ ] `make clean && make` completes without errors
- [ ] PDF opens and displays correctly
- [ ] All pages render (no missing pages)
- [ ] Hyperlinks work (if using `hyperref`)

### Final Check

```bash
# Clean build from scratch
make clean
make

# Check PDF properties
pdfinfo thesis_main.pdf

# Check word count (rough estimate)
pdftotext thesis_main.pdf - | wc -w

# Check page count
pdfinfo thesis_main.pdf | grep Pages
```

**Expected**:
- Pages: 80-100 (main content) + front matter + appendices
- Size: < 50 MB
- No errors in compilation log

---

## Quick Reference

### Essential Commands

| Task | Command |
|------|---------|
| Compile (full) | `make` |
| Compile (quick) | `make quick` |
| View PDF | `make view` |
| Clean aux files | `make clean` |
| Edit chapter 1 | `vim chapters/02_chapter1_intro.tex` |
| Check errors | `cat thesis_main.log | grep Error` |
| Add figure | Copy to `figures/`, use `\includegraphics{...}` |
| Add reference | Edit `references.bib`, use `\cite{key}` |

### Essential LaTeX Commands

| Element | LaTeX Command |
|---------|---------------|
| Chapter | `\chapter{Tiêu đề}` |
| Section | `\section{Tiêu đề}` |
| Subsection | `\subsection{Tiêu đề}` |
| Bold text | `\textbf{text}` |
| Italic text | `\textit{text}` |
| Cite | `\cite{key}` |
| Reference | `\ref{label}` |
| Footnote | `\footnote{text}` |
| Line break | `\\` or blank line |
| Page break | `\newpage` |

### File Locations

| Content | File Location |
|---------|---------------|
| Main file | `docs/thesis_main.tex` |
| Packages/config | `docs/preamble.tex` |
| Chapters | `docs/chapters/*.tex` |
| Images | `docs/figures/*.png` |
| Bibliography | `docs/references.bib` |
| Symbols | `docs/symbols.tex` |
| Output PDF | `docs/thesis_main.pdf` |

---

## Getting Help

### Documentation

- LaTeX general: https://www.overleaf.com/learn
- XeLaTeX: https://www.overleaf.com/learn/latex/XeLaTeX
- Biblatex: https://www.overleaf.com/learn/latex/Bibliography_management_with_biblatex
- Vietnamese LaTeX: https://www.overleaf.com/learn/latex/Vietnamese

### Debugging

1. **Read the error message**:
   ```bash
   cat thesis_main.log | grep -A 5 Error
   ```

2. **Simplify to find the problem**:
   - Comment out chapters one by one
   - Binary search to isolate problematic section

3. **Check syntax**:
   - Missing `}` or `\end{...}`
   - Unclosed `\begin{...}` environments
   - Special characters not escaped: `$`, `%`, `&`, `#`, `_`

4. **Clean and rebuild**:
   ```bash
   make clean
   rm -f *.aux *.bbl *.bcf *.blg *.log *.out
   make
   ```

### Common LaTeX Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Missing `}` | Compilation error at end of file | Add closing brace |
| Special char not escaped | `! Missing $ inserted` | Use `\$`, `\%`, `\_`, etc. |
| Unmatched `\begin{...}` | Environment error | Add `\end{...}` |
| Undefined label | `??` in PDF | Run compilation twice |
| Wrong image path | Image not found | Use path relative to `thesis_main.tex` |

---

## Next Steps

1. **Review this guide** to understand the workflow
2. **Verify setup** by compiling the thesis skeleton (`make`)
3. **Start editing** Chapter 1 (Mở đầu)
4. **Compile frequently** to catch errors early (`make quick`)
5. **Commit changes** to git regularly
6. **Ask for help** from advisor when stuck

**Remember**: LaTeX has a learning curve, but it produces professional results. Be patient and compile often!

---

**Quickstart Guide Version**: 1.0
**Last Updated**: 2025-11-11
**Maintained by**: Nguyễn Tấn Khôi
**Questions?**: Contact advisor or refer to LaTeX documentation
