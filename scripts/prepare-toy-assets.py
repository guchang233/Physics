#!/usr/bin/env python3
"""Resize Toy icon/cover to Bilibili recommended sizes."""
from pathlib import Path
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "publish"
CANDIDATE_ICON = [
    Path("/opt/cursor/artifacts/assets/toy-icon-500.jpg"),
    OUT / "icon.png",
]
CANDIDATE_COVER = [
    Path("/opt/cursor/artifacts/assets/toy-cover-1200x900.jpg"),
    OUT / "cover.jpg",
]


def first_existing(paths: list[Path]) -> Path:
    for p in paths:
        if p.is_file():
            return p
    raise FileNotFoundError("missing source image: " + ", ".join(str(p) for p in paths))


def cover_resize(src: Path, size: tuple[int, int], fill=(11, 16, 32)) -> Image.Image:
    im = Image.open(src).convert("RGB")
    tw, th = size
    scale = max(tw / im.width, th / im.height)
    nw, nh = int(round(im.width * scale)), int(round(im.height * scale))
    im = im.resize((nw, nh), Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", size, fill)
    canvas.paste(im, ((tw - nw) // 2, (th - nh) // 2))
    return canvas


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    icon_src = first_existing(CANDIDATE_ICON)
    cover_src = first_existing(CANDIDATE_COVER)
    cover_resize(icon_src, (500, 500)).save(OUT / "icon.png", optimize=True)
    cover_resize(cover_src, (1200, 900)).save(OUT / "cover.jpg", quality=92, optimize=True)
    print("wrote", OUT / "icon.png", "from", icon_src)
    print("wrote", OUT / "cover.jpg", "from", cover_src)


if __name__ == "__main__":
    main()
