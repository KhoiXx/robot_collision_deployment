# Figures Directory Guidelines

**Directory**: `/home/khoint/thesis/deployment/docs/figures/`
**Purpose**: Store all images, diagrams, charts, and plots used in the thesis
**Last Updated**: 2025-11-12

---

## File Naming Conventions

### Use Descriptive Names

✅ **Good Examples**:
- `network_architecture_cnn.png`
- `training_curve_stage1.png`
- `robot_hardware_design.pdf`
- `reward_function_components.svg`
- `comparison_results_stage1_vs_stage2.png`

❌ **Bad Examples**:
- `fig1.png` (not descriptive)
- `image.jpg` (too generic)
- `Screenshot 2025-11-12.png` (not meaningful)
- `新しいイメージ.png` (non-ASCII characters)

### Naming Pattern

```
<topic>_<detail>_<variant>.extension
```

Examples:
- `success_rate_stage1_update100.png`
- `hyperparams_comparison_table.pdf`
- `lidar_scan_360_diagram.svg`

---

## File Format Requirements

### Preferred Formats

| Format | Use Case | Pros | Cons |
|--------|----------|------|------|
| **PDF** | Diagrams, vector graphics | Scalable, high quality, LaTeX-friendly | Large file size |
| **PNG** | Screenshots, photos, raster images | Good quality, wide support | Not scalable |
| **SVG** | Diagrams (convert to PDF) | Scalable, editable | Needs conversion for LaTeX |
| **JPG** | Photos only | Small file size | Lossy compression |

### Avoid
- ❌ BMP (too large)
- ❌ TIFF (compatibility issues)
- ❌ GIF (poor quality, animations not needed)

---

## Resolution Requirements

### HCMUT Standard (SC-014)

All figures **MUST** have **≥300 DPI** for printing quality.

### Check Resolution

```bash
# Using ImageMagick
identify -verbose image.png | grep "Resolution"

# Using file properties
file image.png
```

### Convert Low-Resolution Images

```bash
# Upscale to 300 DPI (not ideal, better to recreate)
convert input.png -density 300 -units PixelsPerInch output.png

# Resize to specific dimensions (e.g., 2000x1500 pixels for ~6.7" x 5" at 300 DPI)
convert input.png -resize 2000x1500 -density 300 output.png
```

---

## LaTeX Integration

### Include a Figure

```latex
\begin{figure}[htbp]
  \centering
  \includegraphics[width=0.8\textwidth]{figures/network_architecture_cnn.png}
  \caption{Kiến trúc mạng CNN với 2 lớp Conv1D và 2 lớp FC}
  \label{fig:network_arch}
\end{figure}
```

### Reference in Text

```latex
Như thể hiện trong Hình \ref{fig:network_arch}, kiến trúc mạng bao gồm...
```

### Width Guidelines

- `0.5\textwidth` - Small figure, half page width
- `0.8\textwidth` - Standard figure, most common
- `\textwidth` - Full page width
- `0.6\linewidth` - For subfigures or side-by-side

### Caption Requirements (FR-007)

- ✅ Caption **below** the figure
- ✅ Numbered by chapter (e.g., Hình 3.4 = Figure 4 in Chapter 3)
- ✅ Cite source if taken from another work: `\caption{...\cite{long2018towards}}`

---

## Creating High-Quality Figures

### For Plots and Charts (Python)

```python
import matplotlib.pyplot as plt

# Set high DPI for export
plt.figure(figsize=(8, 6), dpi=300)

# Your plotting code here
plt.plot(x, y)
plt.xlabel('Epochs')
plt.ylabel('Success Rate (%)')
plt.title('Training Progress - Stage 1')
plt.grid(True)

# Save as high-resolution PNG or PDF
plt.savefig('figures/training_curve_stage1.png', dpi=300, bbox_inches='tight')
# OR save as PDF (vector, scalable)
plt.savefig('figures/training_curve_stage1.pdf', bbox_inches='tight')
```

### For Diagrams (TikZ in LaTeX)

For complex diagrams, consider using TikZ directly in LaTeX:

```latex
\begin{figure}[htbp]
  \centering
  \begin{tikzpicture}
    % Your TikZ code here
    \node[draw] (a) at (0,0) {Input};
    \node[draw] (b) at (2,0) {CNN};
    \node[draw] (c) at (4,0) {Output};
    \draw[->] (a) -- (b);
    \draw[->] (b) -- (c);
  \end{tikzpicture}
  \caption{Luồng dữ liệu qua mạng CNN}
  \label{fig:data_flow}
\end{figure}
```

### For Flowcharts and Diagrams (External Tools)

- **draw.io** (https://app.diagrams.net/): Export as PDF or SVG
- **Inkscape**: Vector graphics editor, export to PDF
- **MATLAB/Python**: Export plots as PDF for best quality

---

## Organization Tips

### Subdirectories (Optional)

If you have many figures, organize by chapter:

```
figures/
├── chapter1/
│   └── motivation_diagram.png
├── chapter2/
│   ├── literature_comparison.pdf
│   └── ppo_algorithm_flowchart.pdf
├── chapter3/
│   ├── network_architecture.pdf
│   ├── reward_function_diagram.png
│   └── robot_hardware_photo.jpg
├── chapter4/
│   ├── training_curve_stage1.png
│   ├── training_curve_stage2.png
│   ├── success_rate_comparison.png
│   └── hyperparams_heatmap.pdf
└── chapter5/
    └── future_directions_concept.png
```

Then reference as: `\includegraphics{figures/chapter3/network_architecture.pdf}`

---

## Quality Checklist

Before adding a figure to the thesis:

- [ ] File name is descriptive and meaningful
- [ ] Resolution ≥300 DPI (for raster images)
- [ ] File format is PDF or PNG (preferred)
- [ ] Image is clear and readable when printed
- [ ] Labels and text in the image are legible (font size ≥8pt)
- [ ] Source is cited if taken from external work
- [ ] File size is reasonable (<5 MB per image)

---

## Common Issues and Solutions

### Issue: Image appears blurry in PDF

**Solution**: Ensure source image is ≥300 DPI. Recreate from original data if possible.

### Issue: Figure is too large/small in compiled PDF

**Solution**: Adjust `width` parameter in `\includegraphics`:
```latex
\includegraphics[width=0.6\textwidth]{...}  % Smaller
\includegraphics[width=\textwidth]{...}     % Larger
```

### Issue: Figure placement is wrong (appears on different page)

**Solution**: LaTeX float placement algorithm. Options:
- Use `[htbp]` for flexible placement
- Use `[H]` to force "here" (requires `\usepackage{float}`)
- Adjust figure size if it doesn't fit on current page

### Issue: Vietnamese caption characters broken

**Solution**: Ensure .tex file is saved as UTF-8 encoding.

---

## Current Figure Count

**Target**: ≥20 figures (SC-004)

**Current**: 0 (placeholder directory)

As you add figures, update this count to track progress toward the success criterion.

---

## References

- LaTeX graphics guide: https://www.overleaf.com/learn/latex/Inserting_Images
- Matplotlib high-quality exports: https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.savefig.html
- TikZ & PGF manual: https://tikz.dev/

---

**Maintained by**: Nguyễn Tấn Khôi
**Questions?**: Refer to quickstart.md or THESIS_FORMAT_GUIDE.md
