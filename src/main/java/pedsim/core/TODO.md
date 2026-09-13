# Core — what is left to do

Split out of `/TODO.md` on 13 September 2026. Core is the machinery every module inherits: the primal
and dual graph, regions, barriers, landmarks, the cognitive map, route choice and path finding,
movement, the day loop, flow accumulation and export. It models no behaviour, and nothing here should
acquire any.

---

## 1. The remote-run route has no caller

`pedsim.core.server.RemoteLauncher` (was `ServerLauncherApplet`) can launch and stop a run over SSH,
but **nothing calls it**. Its only caller was `PedSimCityActionHandler`, deleted with the AWT GUI, and
the browser dashboard has no equivalent: the REST surface is `/api/state`, `/api/roads`,
`/api/population`, `/api/modules`, `/api/start` and `/`, with no remote-run endpoint, and
`dashboard.html` has no server controls.

So the capability that used to live in `ServerConfigPanel` is currently unreachable from either
interface. Three ways to resolve it, and the middle one is probably right:

- **Add it to the dashboard** — a `/api/server` endpoint plus a panel, restoring parity with the old
  Java GUI.
- **Give it a CLI entry point** — `pedsim.core.server.RemoteLauncher` gains a `main`, so a remote run
  is a command like every other run and is reproducible from what is written down.
- **Delete `pedsim.core.server`** — the documented remote workflow is already hand-shipping classes
  over `scp` and running `java` there (see `CLAUDE.md`, *Running on gdsl1*), because the built-in path
  does `git pull` and compiles, so it can only ever run committed code.

Whichever is chosen, note that `server.properties` still points `ssh.key` at a `C:` path that no
longer exists.

## 2. Size the length error for full-network escalations

An agent planning against a route through streets it has never walked carries the same ±10%
perception error as one on streets it knows. On a Torino day, 34 of 197 trips took that path, so it is
not an edge case. Flagged at each site in `RoadDistancePathFinder` and `AngularChangePathFinder`;
sizing it needs a source.

## 3. `Pars.departuresPerPersonPerDay = 0.25` is invented

Read only when the running module supplies no travel demand of its own, so no module ever sees it —
but it decides how many agents a bare core run puts on the street. Documented as a placeholder, with
no source and nothing to calibrate against. A real figure belongs in a configuration file, but core
reads none by design (a city configuration is behavioural, and core has no behaviour).

## 4. Publish GeoMason-light 2.2.0, or stamp the rebuild

One version number currently covers two different builds — 117,232 bytes in the local `.m2`, 115,882
on `gdsl1` before it was replaced. Nothing detects the difference. On 13 Sep it presented as a
`NullPointerException` on a null `nodeID` inside `Environment.prepareGraph`, which points nowhere near
the cause; what isolated it was running the same city locally, where identical code and data completed
cleanly.

Until it is published, a fresh clone elsewhere will not resolve the dependency at all.

## 5. Smaller

- **`agent_release_day_N.csv` lost two columns.** It was
  `step,datetime,meters_to_allocate,meters_spent,agents_released` and is now
  `step,datetime,agents_released`. Nothing in this repo reads it; check `../vodafoneAPI/` and the
  analysis notebooks.
- **The deprecated `initFromArgs(String[])` still has callers** — the cityImage and empirical
  launchers. It reaches core's three parameter classes only; prefer
  `initFromArgs(String[], Class[])` with the module's `parameterClasses()`.
- **`RouteChoicePars` and `Pars` share a copy-pasted class javadoc** ("contains global parameters and
  settings…"), which describes neither.

---

## Invariants — do not break these

- **The departure share is a density integrating to 1.0 over the day**, so it must be integrated over
  the interval between release events (`TimePars.releaseAgentsEveryMinutes`), never over
  `STEP_DURATION`. Integrating over the wrong one scales the whole day by their ratio with no error to
  show for it. Any change to the step size or the release cadence must keep this correct.
- **`RunLedger` is measurement, never an input.** Feeding the planned-versus-walked gap back into
  allocation runs away: the measurement lags, the difference stays negative, and subtracting a
  negative raises the allocation.
- **Trip lengths are walked metres; node lookup is Euclidean.** Everything that picks a destination by
  distance converts through `NetworkCircuity.straightLineFor()`. Dividing at the call site instead is
  how one field came to mean two different quantities.
- **`SimulationModule.parameterClasses()` is the single list** consulted by the command line and by a
  module's city configuration. A parameter class omitted from it is unreachable from both, and a key
  naming one of its fields is accepted and then ignored.
