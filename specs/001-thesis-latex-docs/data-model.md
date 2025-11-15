# Data Model: Thesis Document Entities

**Feature**: `001-thesis-latex-docs`
**Date**: 2025-11-11
**Purpose**: Define the key entities that make up a Vietnamese master's thesis following HCMUT standards

---

## Entity Diagram Overview

```
Thesis
├── FrontMatter
│   ├── Cover
│   ├── Acknowledgment
│   ├── Abstract (VN + EN)
│   ├── Declaration
│   └── TableOfContents
├── MainContent
│   └── Chapter (1-5)
│       ├── Section
│       │   └── Subsection
│       │       └── Subsubsection
│       ├── Figure (0..N)
│       ├── Table (0..N)
│       ├── Equation (0..N)
│       └── Citation (0..N)
└── BackMatter
    ├── References
    └── Appendix
```

---

## Entity 1: Thesis

**Description**: Top-level entity representing the entire master's thesis document.

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `title_vn` | String | Yes | Max 200 chars | Tiêu đề luận văn bằng tiếng Việt |
| `title_en` | String | Yes | Max 200 chars | Thesis title in English |
| `author_name` | String | Yes | - | Tên học viên (e.g., "Nguyễn Tấn Khôi") |
| `student_id` | String | Yes | 7 digits | Mã số học viên (e.g., "2171017") |
| `major` | String | Yes | - | Chuyên ngành (e.g., "Kỹ thuật Điều khiển - Tự động hóa") |
| `advisor_name` | String | Yes | - | GVHD (e.g., "TS. Phạm Việt Cường") |
| `submission_date` | String | Yes | Format: "Tháng MM năm YYYY" | Thời gian nộp (e.g., "Tháng 12 năm 2025") |
| `total_pages` | Integer | Auto | ≤ 100 (excluding appendices) | Tổng số trang nội dung chính |

### Relationships

- Has ONE `FrontMatter`
- Has ONE `MainContent` (containing 5 chapters)
- Has ONE `BackMatter`

### State Transitions

1. **Draft**: Initial creation, chapters being written
2. **Review**: Sent to advisor for review
3. **Revision**: Incorporating feedback
4. **Final**: Ready for submission

### LaTeX Implementation

```latex
% In preamble.tex or thesis_main.tex
\newcommand{\thesistitleVN}{Ứng Dụng Reinforcement Learning...}
\newcommand{\thesistitleEN}{Application of Reinforcement Learning...}
\newcommand{\authorname}{Nguyễn Tấn Khôi}
\newcommand{\studentid}{2171017}
\newcommand{\major}{Kỹ thuật Điều khiển - Tự động hóa}
\newcommand{\advisorname}{TS. Phạm Việt Cường}
\newcommand{\submissiondate}{Tháng 12 năm 2025}
```

---

## Entity 2: Chapter

**Description**: Major structural division of the thesis. HCMUT requires exactly 5 chapters.

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `number` | Integer | Yes | 1-5 | Số thứ tự chương |
| `title_vn` | String | Yes | Max 150 chars | Tiêu đề chương (tiếng Việt) |
| `title_en` | String | No | Max 150 chars | English title (optional for internal ref) |
| `intro_text` | Text | Yes | 2-3 đoạn | Giới thiệu ngắn về nội dung chương |
| `conclusion_text` | Text | Yes | 1-2 đoạn | Tóm tắt hoặc kết luận chương |
| `page_count` | Integer | Auto | - | Số trang của chương |

### Relationships

- Belongs to ONE `Thesis`
- Has MANY `Section` (0..N)
- Has MANY `Figure` (0..N)
- Has MANY `Table` (0..N)
- Has MANY `Equation` (0..N)
- References MANY `Citation` (0..N)

### Validation Rules

- **FR-013**: Mỗi chương PHẢI bắt đầu bằng phần giới thiệu ngắn (2-3 đoạn)
- **FR-014**: Mỗi chương PHẢI kết thúc bằng phần tóm tắt/kết luận (1-2 đoạn)
- Chapter numbering MUST be continuous: 1, 2, 3, 4, 5

### LaTeX Implementation

```latex
% In chapters/02_chapter1_intro.tex
\chapter{MỞ ĐẦU}
\label{chap:introduction}

% Giới thiệu chương (required)
Chương này trình bày về...

\section{Lý do chọn đề tài}
\label{sec:motivation}
% Content...

\section{Mục tiêu nghiên cứu}
% Content...

% Kết luận chương (required)
Tóm lại, chương này đã giới thiệu...
```

---

## Entity 3: Section / Subsection / Subsubsection

**Description**: Hierarchical divisions within a chapter. HCMUT allows max 4 levels of numbering.

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `level` | Integer | Yes | 1-4 | Cấp độ phân mục (1=section, 2=subsection, 3=subsubsection, 4=paragraph) |
| `number_full` | String | Auto | Max 4 digits (e.g., "4.1.2.1") | Số hiệu đầy đủ |
| `title` | String | Yes | Max 100 chars | Tiêu đề mục |
| `content` | Text | Yes | - | Nội dung mục |

### Relationships

- Belongs to ONE `Chapter`
- May have MANY child sections (hierarchical)

### Validation Rules

- **FR-010**: Đánh số tiểu mục PHẢI tối đa 4 chữ số (e.g., 4.1.2.1)
- **FR-010**: Mỗi nhóm tiểu mục phải có ít nhất 2 tiểu mục (không được chỉ có 1 mục con)

### LaTeX Implementation

```latex
\chapter{PHƯƠNG PHÁP NGHIÊN CỨU}  % Level 0: Chapter 3

\section{Môi trường mô phỏng}  % Level 1: 3.1
\label{sec:simulation_env}

\subsection{Không gian quan sát}  % Level 2: 3.1.1
\label{subsec:observation_space}

\subsubsection{Laser scan data}  % Level 3: 3.1.1.1
\label{subsubsec:laser_data}

% No deeper than subsubsection (4 levels max)
```

---

## Entity 4: Figure

**Description**: Images, diagrams, charts, screenshots used to illustrate content.

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `number` | String | Auto | Format: "X.Y" | Số hiệu hình (e.g., "3.4" = hình 4 chương 3) |
| `caption_vn` | String | Yes | Max 200 chars | Đầu đề hình (tiếng Việt) |
| `caption_en` | String | No | Max 200 chars | English caption (optional) |
| `file_path` | Path | Yes | Exists in `figures/` | Đường dẫn file ảnh |
| `format` | Enum | Yes | {PNG, JPG, PDF, SVG} | Định dạng file |
| `resolution_dpi` | Integer | Yes | ≥ 300 | Độ phân giải (for printing) |
| `source` | String | No | - | Nguồn (nếu lấy từ tài liệu khác) |
| `width` | String | Yes | LaTeX unit (e.g., "0.8\textwidth") | Độ rộng hiển thị |

### Relationships

- Belongs to ONE `Chapter`
- May be referenced by MANY `Citation` (if from external source)

### Validation Rules

- **FR-007**: Tất cả hình ảnh PHẢI có đầu đề ở phía dưới
- **FR-007**: Đánh số theo chương (Hình 3.4 = hình thứ 4 chương 3)
- **FR-007**: PHẢI trích dẫn nguồn nếu lấy từ tài liệu khác
- **SC-014**: Tất cả hình ảnh có độ phân giải đủ rõ khi in (minimum 300 DPI)

### LaTeX Implementation

```latex
\begin{figure}[htbp]
  \centering
  \includegraphics[width=0.8\textwidth]{figures/network_architecture.png}
  \caption{Kiến trúc mạng CNN với 2 lớp Conv1D \cite{long2018towards}}
  \label{fig:network_arch}
\end{figure}

% Reference in text
Như thể hiện trong Hình \ref{fig:network_arch}, kiến trúc mạng...
```

---

## Entity 5: Table

**Description**: Tabular data used for comparisons, hyperparameters, results, etc.

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `number` | String | Auto | Format: "X.Y" | Số hiệu bảng (e.g., "2.5" = bảng 5 chương 2) |
| `caption_vn` | String | Yes | Max 200 chars | Đầu đề bảng (tiếng Việt) |
| `caption_en` | String | No | Max 200 chars | English caption (optional) |
| `columns` | Integer | Yes | ≥ 2 | Số cột |
| `rows` | Integer | Yes | ≥ 2 | Số hàng (excluding header) |
| `source` | String | No | - | Nguồn (nếu lấy từ tài liệu khác) |

### Relationships

- Belongs to ONE `Chapter`
- May be referenced by MANY `Citation` (if data from external source)

### Validation Rules

- **FR-008**: Tất cả bảng biểu PHẢI có đầu đề ở phía trên
- **FR-008**: Đánh số theo chương (Bảng 2.5 = bảng thứ 5 chương 2)
- **FR-008**: PHẢI trích dẫn nguồn nếu lấy từ tài liệu khác

### LaTeX Implementation

```latex
\begin{table}[htbp]
  \centering
  \caption{So sánh hyperparameters với bài báo gốc}
  \label{tab:hyperparams_comparison}
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

% Reference in text
Bảng \ref{tab:hyperparams_comparison} cho thấy sự khác biệt...
```

---

## Entity 6: Equation

**Description**: Mathematical formulas and expressions.

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `number` | String | Auto | Format: "(X.Y)" | Số hiệu phương trình (e.g., "(3.1)") |
| `latex_code` | String | Yes | Valid LaTeX math | Mã LaTeX của phương trình |
| `description` | Text | Yes | - | Giải thích ý nghĩa phương trình |
| `variables` | List | No | - | Danh sách biến và ý nghĩa |

### Relationships

- Belongs to ONE `Chapter`
- May reference MANY `Symbol` entities

### Validation Rules

- **FR-009**: Tất cả phương trình PHẢI được đánh số trong ngoặc đơn ở lề phải
- **FR-009**: Đánh số theo chương (e.g., (3.1) = phương trình 1 chương 3)
- Phải giải thích ý nghĩa các biến trong phương trình

### LaTeX Implementation

```latex
Hàm reward được định nghĩa như sau:
\begin{equation}
  r_t = r_{goal} + r_{collision} + r_{clearance} + r_{smoothness} + r_{progress}
  \label{eq:reward_function}
\end{equation}
trong đó $r_{goal}$ là reward khi đạt mục tiêu, $r_{collision}$ là penalty khi va chạm...

% Reference in text
Theo phương trình (\ref{eq:reward_function}), reward function...
```

---

## Entity 7: Reference

**Description**: Bibliographic entry for a cited work (paper, book, website, etc.)

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `key` | String | Yes | Unique | BibTeX key (e.g., "long2018towards") |
| `type` | Enum | Yes | {article, book, inproceedings, misc, ...} | Loại tài liệu |
| `authors` | List | Yes | - | Danh sách tác giả |
| `title` | String | Yes | - | Tiêu đề tài liệu |
| `venue` | String | Depends on type | - | Journal/Conference name |
| `year` | Integer | Yes | 1900-2100 | Năm xuất bản |
| `pages` | String | No | - | Số trang (e.g., "6252-6259") |
| `doi` | String | No | Valid DOI | DOI identifier |
| `url` | URL | No | Valid URL | Link trực tuyến |

### Relationships

- Referenced by MANY `Citation` entities

### Validation Rules

- **FR-006**: Tất cả trích dẫn PHẢI tuân theo chuẩn IEEE style
- **SC-003**: Có ít nhất 15 tài liệu tham khảo được trích dẫn đúng chuẩn IEEE
- **SC-012**: Không có lỗi đạo văn - tất cả nội dung từ nguồn khác đều có citation

### LaTeX Implementation (BibTeX format)

```bibtex
% In references.bib
@inproceedings{long2018towards,
  author    = {Long, Pinxin and Fan, Tingxiang and Liao, Xinyi and Liu, Wenxi and Zhang, Hao and Pan, Jia},
  title     = {Towards optimally decentralized multi-robot collision avoidance via deep reinforcement learning},
  booktitle = {Proc. IEEE Int. Conf. Robot. Autom. (ICRA)},
  year      = {2018},
  pages     = {6252--6259},
  doi       = {10.1109/ICRA.2018.8461113}
}

@article{schulman2017proximal,
  author  = {Schulman, John and Wolski, Filip and Dhariwal, Prafulla and Radford, Alec and Klimov, Oleg},
  title   = {Proximal policy optimization algorithms},
  journal = {arXiv preprint arXiv:1707.06347},
  year    = {2017}
}
```

---

## Entity 8: Citation

**Description**: In-text reference to a bibliographic entry, using IEEE numbered style [1], [2], etc.

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `number` | Integer | Auto | Unique, sequential | Số thứ tự trích dẫn (e.g., [1], [2]) |
| `reference_key` | String | Yes | Valid BibTeX key | Key của reference được trích dẫn |
| `context` | Text | Yes | - | Ngữ cảnh trích dẫn trong văn bản |

### Relationships

- References ONE `Reference` entity
- May appear in MULTIPLE locations (same reference cited many times)

### Validation Rules

- **FR-006**: Format [1], [2], [3]... theo thứ tự xuất hiện
- Multiple citations: [1], [2], [3] hoặc [1-3]
- Tất cả [n] trong văn bản PHẢI có entry tương ứng trong danh mục tham khảo

### LaTeX Implementation

```latex
% Single citation
Theo nghiên cứu của Long et al. \cite{long2018towards}, phương pháp PPO...

% Multiple citations
Các nghiên cứu trước đây \cite{long2018towards,schulman2017proximal,mnih2016asynchronous} đã chứng minh...

% Citation with page number
Như đã chỉ ra trong \cite[p. 6255]{long2018towards}, kiến trúc CNN...
```

---

## Entity 9: Symbol

**Description**: Acronym or mathematical symbol used throughout the thesis.

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `symbol` | String | Yes | Unique | Ký hiệu hoặc chữ viết tắt (e.g., "PPO", "$\omega$") |
| `meaning_vn` | String | Yes | - | Nghĩa tiếng Việt |
| `meaning_en` | String | Yes | - | English meaning |
| `category` | Enum | Yes | {acronym, greek, latin, operator} | Loại ký hiệu |
| `first_use_chapter` | Integer | Auto | 1-5 | Chương xuất hiện đầu tiên |

### Relationships

- May be used in MANY `Equation`, `Figure`, `Table`, `Chapter` entities

### Validation Rules

- **FR-015**: Tất cả thuật ngữ tiếng Anh xuất hiện lần đầu PHẢI có giải thích tiếng Việt
- **SC-010**: Danh mục ký hiệu liệt kê ít nhất 20 thuật ngữ/ký hiệu
- Ký hiệu được sắp xếp theo thứ tự ABC

### LaTeX Implementation

```latex
% In symbols.tex
\chapter*{DANH MỤC KÝ HIỆU VÀ CHỮ VIẾT TẮT}
\addcontentsline{toc}{chapter}{DANH MỤC KÝ HIỆU VÀ CHỮ VIẾT TẮT}

\begin{description}
  \item[CNN] Convolutional Neural Network - Mạng nơ-ron tích chập
  \item[DRL] Deep Reinforcement Learning - Học tăng cường sâu
  \item[GAE] Generalized Advantage Estimation - Ước lượng ưu thế tổng quát
  \item[LiDAR] Light Detection and Ranging - Công nghệ quét laser
  \item[PPO] Proximal Policy Optimization - Tối ưu hóa chính sách gần kề
  \item[RL] Reinforcement Learning - Học tăng cường
  \item[$\omega$] Angular velocity - Vận tốc góc
  \item[$v$] Linear velocity - Vận tốc tuyến tính
  \item[$r_t$] Reward at time step $t$ - Phần thưởng tại bước thời gian $t$
\end{description}
```

---

## Entity 10: FrontMatter / BackMatter

**Description**: Special sections before and after main content.

### FrontMatter Components

| Component | Required | Page Numbering | Description |
|-----------|----------|----------------|-------------|
| `Cover` | Yes | None | Bìa chính (thông tin trường, học viên, GVHD) |
| `TaskAssignment` | Yes | None | Trang nhiệm vụ (theo QĐ giao đề tài) |
| `Acknowledgment` | Yes | Roman (i, ii, ...) | Lời cảm ơn |
| `Abstract_VN` | Yes | Roman | Tóm tắt tiếng Việt (≤ 1 trang) |
| `Abstract_EN` | Yes | Roman | Abstract in English (≤ 1 page) |
| `Declaration` | Yes | Roman | Lời cam đoan |
| `TableOfContents` | Yes | Roman | Mục lục (auto-generated) |
| `ListOfFigures` | Yes | Roman | Danh mục hình (auto-generated) |
| `ListOfTables` | Yes | Roman | Danh mục bảng (auto-generated) |
| `ListOfSymbols` | Yes | Roman | Danh mục ký hiệu |

### BackMatter Components

| Component | Required | Page Numbering | Description |
|-----------|----------|----------------|-------------|
| `Publications` | No | Arabic (continue) | Danh mục công trình của tác giả (nếu có) |
| `References` | Yes | Arabic (continue) | Tài liệu tham khảo (auto-generated from .bib) |
| `Appendices` | No | Arabic (continue) | Phụ lục (không dày hơn phần chính) |

---

## Relationships Summary Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                          THESIS                              │
│  - title, author, major, advisor, submission_date            │
└────────────┬────────────────────────────────────────────────┘
             │
      ┌──────┴─────┬─────────────┬───────────────┐
      │            │             │               │
┌─────▼──────┐ ┌──▼────────┐ ┌──▼─────────┐ ┌──▼──────────┐
│FrontMatter │ │ MainContent│ │ BackMatter │ │   Symbols   │
│ (Cover,    │ │  (5 chap.) │ │(References,│ │  (Acronyms, │
│  Abstract, │ │            │ │ Appendices)│ │   Greek)    │
│  TOC, etc.)│ │            │ │            │ │             │
└────────────┘ └─────┬──────┘ └────────────┘ └─────────────┘
                     │
              ┌──────┴──────┬──────────┬────────────┐
              │             │          │            │
         ┌────▼──────┐ ┌────▼────┐ ┌──▼──────┐ ┌───▼─────────┐
         │  CHAPTER  │ │  FIGURE │ │  TABLE  │ │  EQUATION   │
         │ (1-5)     │ │ (X.Y)   │ │ (X.Y)   │ │  ((X.Y))    │
         │           │ │         │ │         │ │             │
         └─────┬─────┘ └─────────┘ └─────────┘ └─────────────┘
               │
      ┌────────┴────────┬──────────────┐
      │                 │              │
 ┌────▼──────┐    ┌─────▼──────┐  ┌───▼────────┐
 │  SECTION  │    │  CITATION  │  │ REFERENCE  │
 │  (X.Y)    │    │   [1],[2]  │  │ (IEEE bib) │
 │           │    │            │  │            │
 └───────────┘    └────────────┘  └────────────┘
```

---

## Implementation Notes

### Auto-Numbering

All numbering (chapters, sections, figures, tables, equations, citations) is **automatic** in LaTeX:
- Use `\label{...}` to mark entities
- Use `\ref{...}` to reference them
- LaTeX handles numbering and updates automatically

### Cross-References

Example cross-reference workflow:
```latex
% Define
\begin{figure}[htbp]
  \includegraphics{...}
  \caption{Network architecture}
  \label{fig:net_arch}  % Define label
\end{figure}

% Reference
As shown in Figure \ref{fig:net_arch}, the network...  % Auto number
```

### Validation

Key validation points:
1. **Compile checks**: XeLaTeX warns about missing references, citations
2. **Format checks**: Manual verification against THESIS_FORMAT_GUIDE.md
3. **Content checks**: Ensure all chapters have intro/conclusion paragraphs
4. **Citation checks**: All `\cite{}` have corresponding entries in `references.bib`

---

## Success Criteria Mapping

| Success Criterion | Entities Involved |
|-------------------|-------------------|
| SC-001: 5 chapters, 80-100 pages | `Thesis`, `Chapter` |
| SC-002: 100% format compliance | All entities (via LaTeX packages) |
| SC-003: ≥15 references, IEEE style | `Reference`, `Citation` |
| SC-004: ≥20 figures with captions | `Figure` |
| SC-005: ≥5 tables with captions | `Table` |
| SC-006: Chapter 4 with ≥8 experiments | `Chapter`, `Figure`, `Table` |
| SC-008: Chapter 3 with ≥10 hyperparameters | `Chapter`, `Table` |
| SC-009: Abstracts ≤1 page each | `FrontMatter` (Abstract_VN, Abstract_EN) |
| SC-010: ≥20 symbols | `Symbol` |
| SC-011: Auto TOC with 4 levels | `Chapter`, `Section`, TOC |

---

**Data Model Completed**: 2025-11-11
**Status**: ✅ All entities defined with fields, relationships, and validation rules
**Next**: Create `quickstart.md` for compilation and editing guide
