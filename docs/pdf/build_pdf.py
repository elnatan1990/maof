"""Build docs/MEP-Coordination-Spike.pdf from docs/pdf/index.html with headless Chrome.

    python docs/pdf/build_pdf.py [--live-url https://…]

- injects the routing half of routing/reroute.py into Appendix A (so the appendix is the code, not a copy);
- prints the page count and the characters per page (a near-empty page is a layout bug);
- renders every page to docs/pdf/out/page-N.png for a visual check (needs PyMuPDF; skipped if missing);
- copies each PDF next to the companion page (web/) so the live site serves it;
- builds the Hebrew summary (docs/pdf/index-he.html -> docs/MEP-Coordination-Spike-HE.pdf) the same way.
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


def build(src: Path, pdf: Path, live: str, inject_code: bool) -> None:
    page = src.read_text(encoding="utf-8")
    if inject_code:
        page = page.replace("__CODE__", html.escape(router_source()))
    page = page.replace("__LIVE_URL__", live).replace("__LIVE_HOST__", re.sub(r"^https?://", "", live))
    page = page.replace('src="../diagrams/', 'src="../../diagrams/')   # the built file lives one folder deeper
    built = OUT_DIR / (src.stem + ".built.html")
    built.write_text(page, encoding="utf-8")
    cmd = [chrome(), "--headless=new", "--disable-gpu", "--no-pdf-header-footer", "--no-first-run",
           "--virtual-time-budget=8000", "--run-all-compositor-stages-before-draw",
           f"--print-to-pdf={pdf}", built.resolve().as_uri()]
    subprocess.run(cmd, check=True, capture_output=True)
    try:
        from pypdf import PdfReader
        reader = PdfReader(str(pdf))
        print(f"{pdf.name}: {len(reader.pages)} pages")
        for i, pg in enumerate(reader.pages, 1):
            print(f"  page {i}: {len((pg.extract_text() or '').strip())} characters")
    except ImportError:
        print("pypdf not installed: page count skipped")
    try:
        import fitz  # PyMuPDF
        doc = fitz.open(str(pdf))
        for i, pg in enumerate(doc, 1):
            pg.get_pixmap(dpi=96).save(str(OUT_DIR / f"{pdf.stem}-page-{i}.png"))
        print(f"  page images in {OUT_DIR}")
    except ImportError:
        print("PyMuPDF not installed: page images skipped")
    shutil.copyfile(pdf, ROOT / "web" / pdf.name)


def main() -> int:
    live = "https://maof-mep.vercel.app"
    if "--live-url" in sys.argv:
        live = sys.argv[sys.argv.index("--live-url") + 1]
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    build(SRC, PDF, live, inject_code=True)                                   # the English answer
    build(ROOT / "docs" / "pdf" / "index-he.html", ROOT / "docs" / "MEP-Coordination-Spike-HE.pdf", live, inject_code=False)  # the Hebrew summary
    return 0


if __name__ == "__main__":
    sys.exit(main())
