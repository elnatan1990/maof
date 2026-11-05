"""Build docs/MEP-Coordination-Spike.pdf from docs/pdf/index.html with headless Chrome.

    python docs/pdf/build_pdf.py [--live-url https://…]

- injects the routing half of routing/reroute.py into Appendix A (so the appendix is the code, not a copy);
- prints the page count and the characters per page (a near-empty page is a layout bug);
- renders every page to docs/pdf/out/page-N.png for a visual check (needs PyMuPDF; skipped if missing);
- copies the PDF next to the companion page (web/) so the live site serves it.
"""
from __future__ import annotations

import html
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "docs" / "pdf" / "index.html"
OUT_DIR = ROOT / "docs" / "pdf" / "out"
BUILT = OUT_DIR / "index.built.html"
PDF = ROOT / "docs" / "MEP-Coordination-Spike.pdf"
WEB_PDF = ROOT / "web" / "MEP-Coordination-Spike.pdf"
CHROME = [
    os.environ.get("CHROME_EXECUTABLE_PATH", ""),
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    "/usr/bin/google-chrome",
]


def chrome() -> str:
    for c in CHROME:
        if c and Path(c).exists():
            return c
    sys.exit("Chrome not found; set CHROME_EXECUTABLE_PATH")


def router_source() -> str:
    text = (ROOT / "routing" / "reroute.py").read_text(encoding="utf-8")
    start = text.index("# --- the router")
    return text[start:].rstrip()


def main() -> int:
    live = "https://maof-mep.vercel.app"
    if "--live-url" in sys.argv:
        live = sys.argv[sys.argv.index("--live-url") + 1]
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    page = SRC.read_text(encoding="utf-8")
    page = page.replace("__CODE__", html.escape(router_source()))
    page = page.replace("__LIVE_URL__", live).replace("__LIVE_HOST__", re.sub(r"^https?://", "", live))
    # the built file lives one folder deeper than the source, so the diagram paths change
    page = page.replace('src="../diagrams/', 'src="../../diagrams/')
    BUILT.write_text(page, encoding="utf-8")
    cmd = [chrome(), "--headless=new", "--disable-gpu", "--no-pdf-header-footer", "--no-first-run",
           "--virtual-time-budget=8000", "--run-all-compositor-stages-before-draw",
           f"--print-to-pdf={PDF}", BUILT.resolve().as_uri()]
    subprocess.run(cmd, check=True, capture_output=True)
    try:
        from pypdf import PdfReader
        reader = PdfReader(str(PDF))
        print(f"{PDF.name}: {len(reader.pages)} pages")
        for i, p in enumerate(reader.pages, 1):
            print(f"  page {i}: {len((p.extract_text() or '').strip())} characters")
    except ImportError:
        print("pypdf not installed: page count skipped")
    try:
        import fitz  # PyMuPDF
        doc = fitz.open(str(PDF))
        for i, pg in enumerate(doc, 1):
            pix = pg.get_pixmap(dpi=96)
            pix.save(str(OUT_DIR / f"page-{i}.png"))
        print(f"  page images in {OUT_DIR}")
    except ImportError:
        print("PyMuPDF not installed: page images skipped")
    shutil.copyfile(PDF, WEB_PDF)
    return 0


if __name__ == "__main__":
    sys.exit(main())
