"""Generate a printable A4 checkerboard PDF for camera (intrinsic + distortion)
calibration with cv2.findChessboardCorners / cv2.calibrateCamera.

Layout: A4 (210x297mm), >=20mm clear margin on all four sides. An 8x12 grid of
20mm squares (160x240mm) is centered on the page -> 7x11 INNER corners, which
is the pattern size you pass to OpenCV: (7, 11).

A 50mm scale-check line is printed in the top margin so you can verify with a
ruler that the print came out at true size -- the calibration is only metric if
the squares are exactly 20mm on paper.

Usage:
    python tools/make_checkerboard.py            # -> dataset/calib/checkerboard_a4.pdf
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

REPO = Path(__file__).resolve().parent.parent
OUT_PDF = REPO / "dataset" / "calib" / "checkerboard_a4.pdf"

A4_W, A4_H = 210.0, 297.0        # mm
MARGIN = 20.0                    # mm, minimum clear border
SQUARE = 20.0                    # mm, physical size of each square
COLS, ROWS = 8, 12               # squares -> (COLS-1)x(ROWS-1) inner corners
MM_PER_INCH = 25.4


def main():
    board_w, board_h = COLS * SQUARE, ROWS * SQUARE
    assert board_w <= A4_W - 2 * MARGIN and board_h <= A4_H - 2 * MARGIN, "board exceeds margins"
    x0 = (A4_W - board_w) / 2.0    # centered -> equal L/R margin
    y0 = (A4_H - board_h) / 2.0    # centered -> equal T/B margin

    fig = plt.figure(figsize=(A4_W / MM_PER_INCH, A4_H / MM_PER_INCH))
    ax = fig.add_axes([0, 0, 1, 1])           # full-page axes
    ax.set_xlim(0, A4_W)
    ax.set_ylim(0, A4_H)
    ax.set_aspect("equal")
    ax.axis("off")

    # checkerboard squares
    for r in range(ROWS):
        for c in range(COLS):
            if (r + c) % 2 == 0:
                ax.add_patch(Rectangle((x0 + c * SQUARE, y0 + r * SQUARE),
                                       SQUARE, SQUARE, facecolor="black", edgecolor="none"))

    # 50mm scale-check line in the top margin
    line_len = 50.0
    lx = (A4_W - line_len) / 2.0
    ly = A4_H - MARGIN / 2.0
    ax.plot([lx, lx + line_len], [ly, ly], color="black", lw=1.0)
    for xx in (lx, lx + line_len):
        ax.plot([xx, xx], [ly - 1.5, ly + 1.5], color="black", lw=1.0)
    ax.text(A4_W / 2.0, ly - 4.0, "50 mm (measure to verify 100% print scale)",
            ha="center", va="top", fontsize=7)

    # spec label in the bottom margin
    ax.text(A4_W / 2.0, MARGIN / 2.0,
            f"{COLS}x{ROWS} squares, {SQUARE:.0f} mm each  |  OpenCV pattern size = "
            f"({COLS - 1}, {ROWS - 1}) inner corners  |  print at 100% (actual size), no 'fit to page'",
            ha="center", va="center", fontsize=7)

    OUT_PDF.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(OUT_PDF), format="pdf")
    plt.close(fig)
    print(f"wrote {OUT_PDF}")
    print(f"  board: {COLS}x{ROWS} squares of {SQUARE:.0f}mm = {board_w:.0f}x{board_h:.0f}mm")
    print(f"  margins: L/R {x0:.1f}mm, T/B {y0:.1f}mm")
    print(f"  OpenCV pattern size (inner corners): ({COLS - 1}, {ROWS - 1})")


if __name__ == "__main__":
    main()
