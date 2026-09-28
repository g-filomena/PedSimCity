"""Publish simulation result pages to Cloudflare Pages (pedsimcity.inclusivestreets.org).

Stages the self-contained result pages HtmlExporter writes under ``outputs/results/``
into ``outputs/site/`` and deploys the folder with wrangler:

    wrangler pages deploy outputs/site --project-name <project>

The PedSimCity site lives at the root of its own subdomain, one sub-page per city:

    outputs/site/index.html           -> pedsimcity.inclusivestreets.org/          (overview)
    outputs/site/<City>/index.html    -> pedsimcity.inclusivestreets.org/<City>    (that city's runs)
    outputs/site/<City>/results_*.html (the individual runs)

The inclusivestreets.org apex is a separate concern (the umbrella site, which lives outside
this repository in ../inclusivestreets/) and is not managed here — this publisher owns only
the pedsimcity subdomain. They are **two Cloudflare Pages projects**, because one project
serves the same deployment on every domain attached to it: ``pedsimcity`` holds this site,
``inclusivestreets`` holds the apex and www.

One-time setup (see README): ``npm install -g wrangler``, ``wrangler login``,
``wrangler pages project create <project>``, then attach the custom domain in the
Cloudflare dashboard (Workers & Pages -> project -> Custom domains). Where wrangler is
installed outside PATH, ``$WRANGLER`` names the executable.

Standard library only — runs with any Python. Use ``--no-deploy`` to only stage the
folder (it can then be drag-and-dropped in the Cloudflare Pages dashboard instead) and
``--open`` to preview the staged site in a browser.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import stat
import subprocess
import time
import webbrowser
from collections import defaultdict
from datetime import date, datetime, timedelta
from html import escape
from pathlib import Path
from urllib.parse import quote

# This script lives in scripts/; the repo root is one level up.
REPO_ROOT = Path(__file__).resolve().parent.parent
RESULTS_DIR = REPO_ROOT / "outputs" / "results"
SITE_DIR = REPO_ROOT / "outputs" / "site"
# Per-city web data (street geometry, one aggregated file per season) and the page that
# reads it. The data is produced outside this script, by export_network_geojson.py and
# aggregate_season_volumes.py; this only stages it and writes the summary that indexes it.
SITE_DATA_DIR = REPO_ROOT / "outputs" / "site_data"
SEASONS_TEMPLATE = Path(__file__).resolve().parent / "site" / "seasons.html"
SEASON_ORDER = ["spring", "summer", "autumn", "winter"]

# Public host the staged site is served at (used only for console messages).
SITE_HOST = "pedsimcity.inclusivestreets.org"
# The Pages branch the custom domain serves. Deploying from any other branch name gives a
# preview URL and leaves the live site untouched.
PRODUCTION_BRANCH = "main"
GITHUB_URL = "https://github.com/g-filomena/PedSimCity"

RESULT_NAME = re.compile(
    r"results_(?P<city>.+)_day(?P<day>\d+)_job(?P<job>\d+)_(?P<stamp>.+)\.html"
)

# Where a city is, for the sunrise and sunset its season pages shade: latitude and longitude as
# the model measures them from the street network, the standard UTC offset, and whether the EU
# summer-time rule applies. A city missing here is published without the shading.
CITY_LOCATION = {
    "Torino": {"lat": 45.0634, "lon": 7.6768, "utc": 1, "euSummerTime": True},
}


def _last_sunday(year: int, month: int) -> date:
    last = (date(year, month + 1, 1) if month < 12 else date(year + 1, 1, 1)) - timedelta(days=1)
    return last - timedelta(days=(last.weekday() + 1) % 7)


def _sun_hours(location: dict, day: date) -> tuple[float, float]:
    """Sunrise and sunset on ``day`` in local clock hours, by the NOAA approximation with the
    sun's centre 0.833 degrees below the horizon - the rule the model's ``Daylight`` uses."""
    g = 2 * math.pi / 365 * (day.timetuple().tm_yday - 1)
    eq_time = 229.18 * (0.000075 + 0.001868 * math.cos(g) - 0.032077 * math.sin(g)
                        - 0.014615 * math.cos(2 * g) - 0.040849 * math.sin(2 * g))
    decl = (0.006918 - 0.399912 * math.cos(g) + 0.070257 * math.sin(g)
            - 0.006758 * math.cos(2 * g) + 0.000907 * math.sin(2 * g)
            - 0.002697 * math.cos(3 * g) + 0.00148 * math.sin(3 * g))
    phi = math.radians(location["lat"])
    hour_angle = math.degrees(math.acos(
        math.cos(math.radians(90.833)) / (math.cos(phi) * math.cos(decl))
        - math.tan(phi) * math.tan(decl)))
    offset = location["utc"]
    if location["euSummerTime"] and _last_sunday(day.year, 3) <= day < _last_sunday(day.year, 10):
        offset += 1
    rise = (720 - 4 * (location["lon"] + hour_angle) - eq_time) / 60 + offset
    sunset = (720 - 4 * (location["lon"] - hour_angle) - eq_time) / 60 + offset
    return rise, sunset


def _season_sun(city: str, dates: list[str]) -> dict | None:
    """Mean sunrise and sunset over a season's simulated dates, or None for an unplaced city."""
    location = CITY_LOCATION.get(city)
    if location is None or not dates:
        return None
    times = [_sun_hours(location, date.fromisoformat(d)) for d in dates]
    return {"rise": round(sum(t[0] for t in times) / len(times), 3),
            "set": round(sum(t[1] for t in times) / len(times), 3)}


# --- page shell ------------------------------------------------------------

# Kept as a standalone string (not an f-string) so the CSS braces need no escaping;
# it is interpolated into the page verbatim by _shell below.
_STYLE = """
  :root {
    color-scheme: light dark;
    --surface-0: #ffffff; --surface-1: #fcfcfb; --surface-2: #f4f3f0; --rule: #e0dfda;
    --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #77756e;
    --accent: #eb6834;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --surface-0: #121211; --surface-1: #1a1a19; --surface-2: #232321; --rule: #34342f;
      --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #96958c;
      --accent: #d95926;
    }
  }
  * { box-sizing: border-box; }
  body { font-family: system-ui, -apple-system, "Segoe UI", sans-serif; max-width: 60rem;
         margin: 0 auto; padding: 2.5rem 1.25rem 4rem; line-height: 1.6;
         background: var(--surface-0); color: var(--text-primary); }
  nav.crumbs { font-size: .82rem; color: var(--text-muted); margin-bottom: 1.6rem; }
  nav.crumbs a { color: inherit; text-decoration: none; }
  nav.crumbs a:hover { text-decoration: underline; }
  .hero { margin: 0 0 2rem; }
  .eyebrow { font-size: .75rem; font-weight: 600; letter-spacing: .08em; text-transform: uppercase;
             color: var(--text-muted); margin: 0 0 .35rem; }
  .hero h1 { font-size: clamp(1.8rem, 3.4vw, 2.5rem); line-height: 1.12; margin: 0 0 .8rem;
             letter-spacing: -0.025em; font-weight: 700; }
  .lede { max-width: 44rem; color: var(--text-secondary); font-size: 1.04rem; margin: 0; }
  h2 { font-size: 1.05rem; margin: 2.4rem 0 .8rem; letter-spacing: -0.01em; }
  ul.cards { list-style: none; padding: 0; margin: 0; display: grid; gap: .75rem;
             grid-template-columns: repeat(auto-fill, minmax(min(17rem, 100%), 1fr)); }
  a.card { display: block; height: 100%; padding: 1rem 1.1rem; text-decoration: none;
           color: inherit; background: var(--surface-1); border: 1px solid var(--rule);
           border-radius: 12px; transition: border-color .15s ease, transform .15s ease; }
  a.card:hover { border-color: var(--text-muted); transform: translateY(-1px); }
  a.card.feature { border-left: 3px solid var(--accent); }
  a.card .title { display: block; font-weight: 650; font-size: 1.02rem; letter-spacing: -0.01em; }
  a.card .meta { display: block; color: var(--text-muted); font-size: .82rem; margin-top: .25rem; }
  a.card .blurb { display: block; color: var(--text-secondary); font-size: .88rem;
                  margin-top: .5rem; }
  a.card .go { display: inline-block; margin-top: .7rem; font-size: .82rem; font-weight: 600; }
  .stats { display: grid; grid-template-columns: 1fr 1fr; gap: .5rem 1rem; margin-top: .7rem; }
  .stats span { font-size: .8rem; color: var(--text-muted); }
  .stats b { display: block; font-size: 1.05rem; color: var(--text-primary); font-weight: 650;
             letter-spacing: -0.01em; }
  .latest { font-size: .66rem; font-weight: 600; letter-spacing: .04em; text-transform: uppercase;
            padding: .06rem .4rem; border-radius: 999px; margin-left: .45rem; vertical-align: middle;
            background: var(--surface-2); color: var(--text-secondary); }
  .empty { color: var(--text-muted); }
  footer { margin-top: 3.5rem; color: var(--text-muted); font-size: .82rem; padding-top: 1rem;
           border-top: 1px solid var(--rule); }
  footer a { color: inherit; }
"""


def _shell(head_title: str, eyebrow: str, title: str, lede: str, breadcrumb: str,
           body: str) -> str:
    """Wraps page body in the shared HTML/CSS shell. ``breadcrumb`` and ``body`` are trusted
    HTML; the other arguments are escaped."""
    published = datetime.now().strftime("%Y-%m-%d %H:%M")
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{escape(head_title)}</title>
<meta name="description" content="{escape(lede)}">
<style>{_STYLE}</style>
</head>
<body>
{breadcrumb}
<header class="hero">
  <p class="eyebrow">{escape(eyebrow)}</p>
  <h1>{escape(title)}</h1>
  <p class="lede">{escape(lede)}</p>
</header>
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


def _city_card(city: str, infos: list[dict], summary: dict | None = None) -> str:
    href = quote(city) + "/"
    if summary:
        seasons = summary["seasons"]
        walked = sum(e["walkedM"] for e in seasons) / 1000
        blurb = ("The night model walked through four seasons: where people go, when, and how "
                 "well lit their routes are once the sun sets.")
        stats = [(f'{len(seasons)}', "seasons"),
                 (f'{summary["days"]} × {summary["jobs"]}', "days × replicate jobs"),
                 (f'{summary["agents"]:,}', "agents"),
                 (f'{walked:,.0f} km', "walked in all")]
        stats_html = "".join(f"<span><b>{escape(v)}</b>{escape(k)}</span>" for v, k in stats)
        return (f'  <li><a class="card feature" href="{escape(href)}">'
                f'<span class="title">{escape(city)}</span>'
                f'<span class="blurb">{escape(blurb)}</span>'
                f'<span class="stats">{stats_html}</span>'
                f'<span class="go">Explore {escape(city)} →</span></a></li>')
    infos = sorted(infos, key=lambda i: i["mtime"], reverse=True)
    meta = "no runs yet"
    if infos:
        latest = infos[0]
        latest_desc = f"day {latest['day']}" if latest["day"] is not None else latest["path"].stem
        meta = (f'{len(infos)} run{"s" if len(infos) != 1 else ""} · latest {latest_desc}'
                f' ({latest["mtime"].strftime("%Y-%m-%d")})')
    return (f'  <li><a class="card" href="{escape(href)}">'
            f'<span class="title">{escape(city)}</span>'
            f'<span class="meta">{escape(meta)}</span></a></li>')


# --- site building ---------------------------------------------------------

def _season_summary(city: str, data_dir: Path) -> dict | None:
    """Indexes a city's season files into the small JSON the seasons page loads first.

    Each season file is per-edge and several megabytes; the page needs the headline
    numbers before it fetches any of them, and the four-season table needs all of them
    at once. Returns None when the city ships no season data.
    """
    seasons_dir = data_dir / "seasons"
    generated = {"summary", "trips_summary"}
    files = (sorted(p for p in seasons_dir.glob("*.json") if p.stem not in generated)
             if seasons_dir.is_dir() else [])
    if not files:
        return None

    geometry = next((p.name for p in data_dir.glob(f"{city}_edges.geojson")), None)
    if geometry is None:
        print(f"  {city}: no {city}_edges.geojson beside the season files — skipping the map")
        return None

    trips_path = seasons_dir / "trips_summary.json"
    trips = json.loads(trips_path.read_text()) if trips_path.exists() else {}

    entries, agents, days, jobs = [], 0, 0, 0
    for path in files:
        data = json.loads(path.read_text())
        rows = data.get("daySummary", [])

        def total(column: str) -> float:
            return sum(float(row[column]) for row in rows if row.get(column))

        legs = total("legs")
        entry = {
            "season": data.get("season", path.stem),
            "dates": data.get("dates", []),
            "legs": legs,
            "plannedM": total("planned_m"),
            "walkedM": total("walked_m"),
            "darkLegShare": (total("legs_dark") / legs) if legs else 0.0,
            "traversals": data["traversals"]["vulnerable"] + data["traversals"]["nonVulnerable"],
            "vulnerableShare": data["vulnerableShare"],
            "sun": _season_sun(city, data.get("dates", [])),
        }
        run_agents = max((int(row["agents"]) for row in rows if row.get("agents")), default=0)
        entry["metresPerAgent"] = (
            entry["walkedM"] / (run_agents * data["dayJobs"]) if run_agents and data["dayJobs"] else 0.0
        )
        # Per-population figures only: the trips file may cover fewer jobs or days than the
        # day summaries do, so it must not overwrite a total already taken from them.
        for key, value in trips.get(entry["season"], {}).items():
            entry.setdefault(key, value)
        entries.append(entry)
        agents = max(agents, run_agents)
        days = max(days, data.get("days", 0))
        jobs = max(jobs, len(data.get("jobs", [])))

    order = {name: i for i, name in enumerate(SEASON_ORDER)}
    entries.sort(key=lambda e: (order.get(e["season"], len(order)), e["season"]))

    edges = json.loads((data_dir / geometry).read_text())
    summary = {
        "city": city,
        "geometry": geometry,
        "agents": agents,
        "days": days,
        "jobs": jobs,
        "edgesInNetwork": len(edges.get("features", [])),
        "seasons": entries,
    }
    (seasons_dir / "summary.json").write_text(json.dumps(summary, separators=(",", ":")))
    return summary


def stage_city_data(city: str, city_dir: Path) -> dict | None:
    """Copies a city's web data under <city>/data/ and writes its seasons page."""
    data_dir = SITE_DATA_DIR / city
    if not data_dir.is_dir():
        return None
    summary = _season_summary(city, data_dir)
    if summary is None:
        return None
    shutil.copytree(data_dir, city_dir / "data", dirs_exist_ok=True)
    shutil.copy2(SEASONS_TEMPLATE, city_dir / "seasons.html")
    return summary


def _seasons_card(summary: dict) -> str:
    meta = (f'{len(summary["seasons"])} seasons · {summary["days"]} days × {summary["jobs"]} '
            f'replicate jobs · {summary["agents"]:,} agents')
    blurb = ("An interactive map of every street, hour by hour, and how the vulnerable and "
             "non-vulnerable populations walk the city before and after dark.")
    return ('  <li><a class="card feature" href="seasons.html">'
            '<span class="title">Streets by season</span>'
            f'<span class="meta">{escape(meta)}</span>'
            f'<span class="blurb">{escape(blurb)}</span>'
            '<span class="go">Open the map →</span></a></li>')


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

    # A city with web data but no exported run page is still published: the data is the result.
    data_cities = ({d.name for d in SITE_DATA_DIR.iterdir() if d.is_dir()}
                   if SITE_DATA_DIR.is_dir() else set())

    overview_items = []
    for city in sorted(set(by_city) | data_cities, key=str.lower):
        infos = sorted(by_city.get(city, []), key=lambda i: i["mtime"], reverse=True)
        city_dir = SITE_DIR / city
        city_dir.mkdir(parents=True, exist_ok=True)

        summary = stage_city_data(city, city_dir)

        run_items = []
        if summary:
            run_items.append(_seasons_card(summary))
        for idx, info in enumerate(infos):
            shutil.copy2(info["path"], city_dir / info["name"])
            run_items.append(_run_card(info, info["name"], latest=(idx == 0)))

        crumbs = _crumbs([("PedSimCity", "../"), (city, None)])
        featured = run_items[:1] if summary else []
        runs = run_items[1:] if summary else run_items
        body = ""
        if featured:
            body += '<ul class="cards">\n' + featured[0] + "\n</ul>"
        if runs:
            body += ('\n<h2>Single-run dashboards</h2>\n<ul class="cards">\n'
                     + "\n".join(runs) + "\n</ul>")
        if not body:
            body = '<p class="empty">Nothing published for this city yet.</p>'
        (city_dir / "index.html").write_text(
            _shell(f"{city} — PedSimCity", "PedSimCity", city,
                   f"Pedestrian-simulation results for {city}.", crumbs, body),
            encoding="utf-8")
        overview_items.append(_city_card(city, infos, summary))

    # Overview at the subdomain root: the model's own landing (intro + per-city results).
    lede = ("An agent-based model of pedestrian movement in cities. Synthetic, census-based "
            "populations plan their days, choose destinations and walk the street network, "
            "street by street and hour by hour. Explore the results by city.")
    if overview_items:
        listing = '<ul class="cards">\n' + "\n".join(overview_items) + "\n</ul>"
    else:
        listing = ('<p class="empty">No runs published yet — results appear here after the '
                   'first simulation.</p>')
    (SITE_DIR / "index.html").write_text(
        _shell("PedSimCity", "Inclusive Streets", "PedSimCity", lede, "",
               "<h2>Cities</h2>\n" + listing),
        encoding="utf-8")

    return {city: len(by_city.get(city, [])) for city in sorted(set(by_city) | data_cities)}


def deploy(project: str, branch: str) -> bool:
    # $WRANGLER names the executable when it is not on PATH — which is the case when it
    # lives in an environment of its own (conda, nvm, a project-local node_modules).
    wrangler = (os.environ.get("WRANGLER")
                or shutil.which("wrangler") or shutil.which("wrangler.cmd"))
    if wrangler is None:
        print("wrangler not found on PATH — install it with:  npm install -g wrangler")
        print("  or, if it is installed elsewhere, point $WRANGLER at the executable")
        print(f"Then deploy with:  wrangler pages deploy {SITE_DIR} "
              f"--project-name {project} --branch {branch}")
        print("(or drag-and-drop the folder in the Cloudflare Pages dashboard)")
        return False
    # The branch is named explicitly: left to itself wrangler takes the current git branch, and
    # a branch other than the project's production one deploys to a preview URL instead of the
    # live domain — which succeeds, prints a URL, and changes nothing that anyone visits.
    subprocess.run(
        [wrangler, "pages", "deploy", str(SITE_DIR),
         "--project-name", project, "--branch", branch],
        check=True,
    )
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish result pages to Cloudflare Pages.")
    parser.add_argument("--project", default="pedsimcity",
                        help="Cloudflare Pages project name (default: pedsimcity)")
    parser.add_argument("--branch", default=PRODUCTION_BRANCH,
                        help=f"Pages branch to deploy to (default: {PRODUCTION_BRANCH}, the "
                             "project's production branch — the one the live domain serves). "
                             "Any other name publishes a preview.")
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
    if deploy(args.project, args.branch):
        where = SITE_HOST if args.branch == PRODUCTION_BRANCH else f"the {args.branch} preview"
        print(f"Deployed to {where}.")


if __name__ == "__main__":
    main()
