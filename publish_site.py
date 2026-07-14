"""Publish simulation result pages to Cloudflare Pages (pedsimcity.inclusivestreets.org).

Stages the self-contained result pages HtmlExporter writes under ``outputs/results/``
into ``outputs/site/`` and deploys the folder with wrangler:

    wrangler pages deploy outputs/site --project-name <project>

The PedSimCity site lives at the root of its own subdomain, one sub-page per city:

    outputs/site/index.html           -> pedsimcity.inclusivestreets.org/          (overview)
    outputs/site/<City>/index.html    -> pedsimcity.inclusivestreets.org/<City>    (that city's runs)
    outputs/site/<City>/results_*.html (the individual runs)

The inclusivestreets.org apex is a separate concern (a future umbrella site) and is not
managed here — this publisher owns only the pedsimcity subdomain.

One-time setup (see README): ``npm install -g wrangler``, ``wrangler login``,
``wrangler pages project create <project>``, then attach the custom domain in the
Cloudflare dashboard (Workers & Pages -> project -> Custom domains).

Standard library only — runs with any Python. Use ``--no-deploy`` to only stage the
folder (it can then be drag-and-dropped in the Cloudflare Pages dashboard instead) and
``--open`` to preview the staged site in a browser.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import stat
import subprocess
import time
import webbrowser
from collections import defaultdict
from datetime import datetime
from html import escape
from pathlib import Path
from urllib.parse import quote

REPO_ROOT = Path(__file__).resolve().parent
RESULTS_DIR = REPO_ROOT / "outputs" / "results"
SITE_DIR = REPO_ROOT / "outputs" / "site"

# Public host the staged site is served at (used only for console messages).
SITE_HOST = "pedsimcity.inclusivestreets.org"
GITHUB_URL = "https://github.com/g-filomena/PedSimCity"

RESULT_NAME = re.compile(
    r"results_(?P<city>.+)_day(?P<day>\d+)_job(?P<job>\d+)_(?P<stamp>.+)\.html"
)


# --- page shell ------------------------------------------------------------

# Kept as a standalone string (not an f-string) so the CSS braces need no escaping;
# it is interpolated into the page verbatim by _shell below.
_STYLE = """
  :root { color-scheme: light dark; }
  * { box-sizing: border-box; }
  body { font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
         max-width: 52rem; margin: 0 auto; padding: 2.5rem 1.25rem 4rem; line-height: 1.6; }
  header h1 { font-size: 1.5rem; margin: 0 0 .15rem; letter-spacing: -0.02em; }
  header .sub { opacity: .7; margin: 0 0 1.25rem; }
  nav.crumbs { font-size: .85rem; opacity: .75; margin-bottom: 1.75rem; }
  nav.crumbs a { text-decoration: none; }
  nav.crumbs a:hover { text-decoration: underline; }
  p.intro { max-width: 42rem; opacity: .8; margin: 0 0 1.6rem; }
  ul.cards { list-style: none; padding: 0; margin: 0; display: grid; gap: .6rem;
             grid-template-columns: repeat(auto-fill, minmax(15rem, 1fr)); }
  a.card { display: block; padding: .8rem 1rem; text-decoration: none; color: inherit;
           border: 1px solid color-mix(in srgb, currentColor 16%, transparent);
           border-radius: 10px; transition: border-color .15s ease, transform .15s ease; }
  a.card:hover { border-color: color-mix(in srgb, currentColor 45%, transparent);
                 transform: translateY(-1px); }
  a.card .title { font-weight: 600; }
  a.card .meta { display: block; opacity: .6; font-size: .8rem; margin-top: .2rem; }
  .latest { font-size: .66rem; font-weight: 600; letter-spacing: .04em; text-transform: uppercase;
            padding: .06rem .4rem; border-radius: 999px; margin-left: .45rem; vertical-align: middle;
            background: color-mix(in srgb, currentColor 14%, transparent); }
  .empty { opacity: .6; }
  footer { margin-top: 3rem; opacity: .55; font-size: .82rem; padding-top: 1rem;
           border-top: 1px solid color-mix(in srgb, currentColor 12%, transparent); }
  footer a { color: inherit; }
"""


def _shell(head_title: str, subtitle: str, breadcrumb: str, body: str) -> str:
    """Wraps page body in the shared HTML/CSS shell. ``breadcrumb`` is trusted HTML."""
    published = datetime.now().strftime("%Y-%m-%d %H:%M")
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{escape(head_title)}</title>
<style>{_STYLE}</style>
</head>
<body>
<header>
  <h1>PedSimCity</h1>
  <p class="sub">{escape(subtitle)}</p>
</header>
{breadcrumb}
{body}
<footer>Part of Inclusive Streets · published {published} ·
  <a href="{GITHUB_URL}">PedSimCity on GitHub</a></footer>
</body>
</html>
"""


def _crumbs(parts: list[tuple[str, str | None]]) -> str:
    """Breadcrumb nav from (label, href) pairs; a None href marks the current page."""
    out = []
    for label, href in parts:
        if href is None:
            out.append(f"<span>{escape(label)}</span>")
        else:
            out.append(f'<a href="{escape(href)}">{escape(label)}</a>')
    return '<nav class="crumbs">' + " / ".join(out) + "</nav>"


# --- result parsing / cards ------------------------------------------------

def parse_page(page: Path) -> dict:
    """Structured metadata for a result page, parsed from its filename when possible."""
    match = RESULT_NAME.fullmatch(page.name)
    mtime = datetime.fromtimestamp(page.stat().st_mtime)
    if match:
        return {"path": page, "name": page.name, "city": match["city"],
                "day": int(match["day"]), "job": int(match["job"]),
                "stamp": match["stamp"], "mtime": mtime}
    # Unrecognised name: keep it, filed under a catch-all city.
    return {"path": page, "name": page.name, "city": "Other",
            "day": None, "job": None, "stamp": None, "mtime": mtime}


def _run_card(info: dict, href: str, latest: bool = False) -> str:
    if info["day"] is not None:
        title = f"Day {info['day']}"
        bits = [f"job {info['job']}"]
        if info["stamp"]:
            bits.append(f"run {info['stamp']}")
    else:
        title = info["path"].stem
        bits = []
    bits.append("exported " + info["mtime"].strftime("%Y-%m-%d %H:%M"))
    badge = ' <span class="latest">latest</span>' if latest else ""
    return (f'  <li><a class="card" href="{escape(href)}">'
            f'<span class="title">{escape(title)}{badge}</span>'
            f'<span class="meta">{escape(" · ".join(bits))}</span></a></li>')


def _city_card(city: str, infos: list[dict]) -> str:
    infos = sorted(infos, key=lambda i: i["mtime"], reverse=True)
    latest = infos[0]
    n = len(infos)
    latest_desc = f"day {latest['day']}" if latest["day"] is not None else latest["path"].stem
    meta = (f'{n} run{"s" if n != 1 else ""} · latest {latest_desc}'
            f' ({latest["mtime"].strftime("%Y-%m-%d")})')
    href = quote(city) + "/"
    return (f'  <li><a class="card" href="{escape(href)}">'
            f'<span class="title">{escape(city)}</span>'
            f'<span class="meta">{escape(meta)}</span></a></li>')


# --- site building ---------------------------------------------------------

def _rm_site_dir() -> None:
    """Remove SITE_DIR before restaging, tolerating the transient locks / read-only flags
    common on Windows + OneDrive-synced folders (retry with backoff, then clear read-only)."""
    for attempt in range(6):
        if not SITE_DIR.exists():
            return
        try:
            shutil.rmtree(SITE_DIR)
            return
        except OSError:
            if attempt == 4:  # penultimate try: clear read-only bits, then retry once more
                for root, dirs, files in os.walk(SITE_DIR):
                    for name in dirs + files:
                        try:
                            os.chmod(os.path.join(root, name), stat.S_IWRITE)
                        except OSError:
                            pass
            time.sleep(0.5)
    raise RuntimeError(
        f"Could not clear {SITE_DIR} after several attempts — it may be locked by OneDrive "
        "sync or an open file. Pause sync (or close the folder) and re-run."
    )


def build_site(pages: list[Path]) -> dict[str, int]:
    """Stages the site at SITE_DIR (subdomain root); returns {city: run count}."""
    # Rebuild from scratch: wrangler deploys the whole folder each time, so pages removed
    # from outputs/results/ must not linger here (and silently go live again).
    _rm_site_dir()
    SITE_DIR.mkdir(parents=True, exist_ok=True)

    by_city: dict[str, list[dict]] = defaultdict(list)
    for page in pages:
        info = parse_page(page)
        by_city[info["city"]].append(info)

    overview_items = []
    for city in sorted(by_city, key=str.lower):
        infos = sorted(by_city[city], key=lambda i: i["mtime"], reverse=True)
        city_dir = SITE_DIR / city
        city_dir.mkdir(parents=True, exist_ok=True)

        run_items = []
        for idx, info in enumerate(infos):
            shutil.copy2(info["path"], city_dir / info["name"])
            run_items.append(_run_card(info, info["name"], latest=(idx == 0)))

        crumbs = _crumbs([("PedSimCity", "../"), (city, None)])
        body = '<ul class="cards">\n' + "\n".join(run_items) + "\n</ul>"
        (city_dir / "index.html").write_text(
            _shell(f"{city} — PedSimCity", f"{city} · pedestrian-simulation results",
                   crumbs, body),
            encoding="utf-8")
        overview_items.append(_city_card(city, infos))

    # Overview at the subdomain root: the model's own landing (intro + per-city results).
    intro = ('<p class="intro">PedSimCity is an agent-based model of pedestrian movement in '
             'cities. Each run simulates a day of walking trips for a synthetic, census-based '
             'population. Explore the results by city.</p>')
    if overview_items:
        listing = '<ul class="cards">\n' + "\n".join(overview_items) + "\n</ul>"
    else:
        listing = ('<p class="empty">No runs published yet — results appear here after the '
                   'first simulation.</p>')
    (SITE_DIR / "index.html").write_text(
        _shell("PedSimCity", "Agent-based pedestrian simulation · results by city",
               "", intro + "\n" + listing),
        encoding="utf-8")

    return {city: len(infos) for city, infos in by_city.items()}


def deploy(project: str) -> bool:
    wrangler = shutil.which("wrangler") or shutil.which("wrangler.cmd")
    if wrangler is None:
        print("wrangler not found on PATH — install it with:  npm install -g wrangler")
        print(f"Then deploy with:  wrangler pages deploy {SITE_DIR} --project-name {project}")
        print("(or drag-and-drop the folder in the Cloudflare Pages dashboard)")
        return False
    subprocess.run(
        [wrangler, "pages", "deploy", str(SITE_DIR), "--project-name", project],
        check=True,
    )
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish result pages to Cloudflare Pages.")
    parser.add_argument("--project", default="inclusivestreets",
                        help="Cloudflare Pages project name (default: inclusivestreets)")
    parser.add_argument("--no-deploy", action="store_true",
                        help="Only stage outputs/site/ (deploy manually or via dashboard).")
    parser.add_argument("--open", action="store_true", dest="open_preview",
                        help="Open the staged index.html in a browser to preview locally.")
    args = parser.parse_args()

    pages = sorted(RESULTS_DIR.glob("*.html")) if RESULTS_DIR.is_dir() else []
    counts = build_site(pages)

    if pages:
        summary = ", ".join(f"{city} ({n})" for city, n in sorted(counts.items()))
        print(f"staged {len(pages)} result page(s) across {len(counts)} city section(s) "
              f"in {SITE_DIR}: {summary}")
        print(f"URLs: {SITE_HOST}/<City>  (e.g. {SITE_HOST}/{next(iter(sorted(counts)))})")
    else:
        print(f"no result pages in {RESULTS_DIR}: staged the PedSimCity overview (empty) — "
              "run a simulation to populate it.")

    if args.open_preview:
        webbrowser.open((SITE_DIR / "index.html").resolve().as_uri())

    if args.no_deploy:
        return
    if deploy(args.project):
        print("Deployed. The site is live on the Pages project"
              f" (and on {SITE_HOST} once the custom domain is attached).")


if __name__ == "__main__":
    main()
