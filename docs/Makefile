# Makefile for HCMUT Master's Thesis LaTeX Compilation
# Author: Nguyễn Tấn Khôi
# Date: 2025-11-12
#
# Usage:
#   make          - Full compilation (bibliography + cross-references)
#   make quick    - Quick compilation (skip bibliography)
#   make clean    - Remove auxiliary files
#   make view     - Open PDF with default viewer
#   make count    - Count words in PDF (approximate)

# Main file name (without .tex extension)
MAIN = thesis_main

# LaTeX compiler (XeLaTeX for Vietnamese + Times New Roman)
LATEX = xelatex

# Bibliography processor (biber for biblatex)
BIBER = biber

# PDF viewer
VIEWER = xdg-open

# LaTeX flags
LATEX_FLAGS = -interaction=nonstopmode -halt-on-error -file-line-error

# ============================================================================
# Targets
# ============================================================================

.PHONY: all quick clean view count help

# Default target: full compilation
all: $(MAIN).pdf

# Full compilation with bibliography
$(MAIN).pdf: $(MAIN).tex chapters/*.tex references.bib preamble.tex symbols.tex
	@echo "========================================"
	@echo "Starting full compilation..."
	@echo "========================================"
	$(LATEX) $(LATEX_FLAGS) $(MAIN)
	@echo "----------------------------------------"
	@echo "Processing bibliography with biber..."
	@echo "----------------------------------------"
	$(BIBER) $(MAIN)
	@echo "----------------------------------------"
	@echo "Second pass (resolve citations)..."
	@echo "----------------------------------------"
	$(LATEX) $(LATEX_FLAGS) $(MAIN)
	@echo "----------------------------------------"
	@echo "Third pass (resolve cross-references)..."
	@echo "----------------------------------------"
	$(LATEX) $(LATEX_FLAGS) $(MAIN)
	@echo "========================================"
	@echo "✓ Compilation complete: $(MAIN).pdf"
	@echo "========================================"

# Quick compilation (skip bibliography, single pass)
quick:
	@echo "========================================"
	@echo "Quick compilation (no bibliography)..."
	@echo "========================================"
	$(LATEX) $(LATEX_FLAGS) $(MAIN)
	@echo "✓ Quick compile complete"

# Clean auxiliary files
clean:
	@echo "Cleaning auxiliary files..."
	rm -f *.aux *.bbl *.bcf *.blg *.log *.out *.toc *.lof *.lot
	rm -f *.run.xml *.fls *.fdb_latexmk *.synctex.gz
	rm -f chapters/*.aux
	@echo "✓ Clean complete"

# Deep clean (including PDF)
distclean: clean
	rm -f $(MAIN).pdf
	@echo "✓ Deep clean complete (PDF removed)"

# View PDF with default viewer
view: $(MAIN).pdf
	@echo "Opening $(MAIN).pdf..."
	$(VIEWER) $(MAIN).pdf &

# Count words in PDF (approximate)
count: $(MAIN).pdf
	@echo "Approximate word count:"
	@pdftotext $(MAIN).pdf - | wc -w

# Check page count
pages: $(MAIN).pdf
	@echo "Page count:"
	@pdfinfo $(MAIN).pdf | grep Pages

# Help message
help:
	@echo "HCMUT Thesis LaTeX Compilation Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  make           - Full compilation (bibliography + 3 passes)"
	@echo "  make quick     - Quick compilation (single pass, no bib)"
	@echo "  make clean     - Remove auxiliary files"
	@echo "  make distclean - Remove auxiliary files + PDF"
	@echo "  make view      - Open PDF with default viewer"
	@echo "  make count     - Count approximate words"
	@echo "  make pages     - Show page count"
	@echo "  make help      - Show this help message"
	@echo ""
	@echo "Example workflow:"
	@echo "  1. make clean  - Clean previous build"
	@echo "  2. make        - Full compilation"
	@echo "  3. make view   - View result"

# ============================================================================
# Notes
# ============================================================================
#
# XeLaTeX compilation requires 3-4 passes:
#   Pass 1: Generate aux files, discover labels/citations
#   Biber:  Process bibliography from references.bib
#   Pass 2: Incorporate bibliography, update references
#   Pass 3: Finalize table of contents, cross-references
#
# For quick edits during writing, use 'make quick' to save time.
# Before final submission, always use 'make clean && make' for a clean build.
