# Final vector-figure typesetting

All numerical experiments and plot generation run in MATLAB. This optional Python step adjusts only the publication layout of exported vector PDFs: Times New Roman subcaptions at 9.5 printed points, centered below each panel, with a 5-point gap. Curves, numerical values, and vehicle geometry are preserved.

After running a chapter’s `make_figures`, install PyMuPDF in a Python environment and run:

```text
python tools/typeset_figures.py --font "C:/Windows/Fonts/times.ttf"
```

Use the path to a licensed, locally installed Times New Roman font on your platform. Font binaries are not included. Original exports and layout metadata are retained in `.figure-layout/`; newly regenerated MATLAB PDFs replace their corresponding layout inputs automatically. The source PDF artwork remains vector content.
