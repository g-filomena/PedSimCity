# "Route-planning avoidance is barrier-blind at night" — investigated, not fixed

Register Section F item, not a numbered finding: *"`roadDistanceNight` reaches
the three-argument `dijkstraAlgorithm`, which skips `initialisePrimal` — so
`directedEdgesToAvoid` is never consulted, and night also swaps in a plain
draw for `costPerceptionError`."*

I went looking for the fix and came back with a reason not to make one —
written up here rather than silently dropped, since the investigation itself
is the useful part.

## What's actually there, and why it's two separate things

`DijkstraRoadDistanceNight` inherits `dijkstraAlgorithm` from
`DijkstraRoadDistance`, which has two overloads
(`src/main/java/pedsim/core/routing/pathfinding/DijkstraRoadDistance.java`):

```java
// 5-arg: calls initialisePrimal(directedEdgesToAvoid) before running
public List<DirectedEdge> dijkstraAlgorithm(
    NodeGraph originNode, NodeGraph destinationNode, NodeGraph finalDestinationNode,
    Set<DirectedEdge> directedEdgesToAvoid, Agent agent)

// 3-arg: skips initialisePrimal entirely
public List<DirectedEdge> dijkstraAlgorithm(
    NodeGraph originNode, NodeGraph destinationNode, Agent agent)
```

`RoadDistancePathFinder.roadDistanceNight()` (night) calls the 3-arg form;
the day path (`pedsim.core.routing.pathfinder.RoadDistancePathFinder`) calls
the 5-arg form. `initialisePrimal` does three things, and skipping it skips
all three together:

1. Populates `knownEdges`/`knownNodes` when `restrictToKnownNetwork()` is
   true (agent's cognitive map is individualised).
2. Builds the hard `directedEdgesToAvoid` exclusion set.
3. Calls `subGraphInitialisation()`, which restricts routing to a
   per-region subgraph when `regionCondition()` holds.

Separately, `DijkstraRoadDistanceNight.findMinDistances()` draws a flat
`drawFromDistribution(1.0, 0.10, null)` per edge instead of calling the
parent's `costPerceptionError()`, which is what applies an agent's barrier
*preferences* (aversion to severing barriers, pull toward natural ones) as a
soft cost adjustment. This is a different mechanism from (2) above — a soft
preference, not a hard exclusion — and the class's own Javadoc already flags
it explicitly: *"Whether that is right is undecided - it has never been
stated either way - but the omission is deliberate here rather than an
oversight in the override."*

So there are really three questions bundled into one register line, and I
checked all three against the actual code before deciding what, if anything,
to change.

## (1) is a non-issue for night agents specifically

`restrictToKnownNetwork()` requires
`agent.getCognitiveMap().individualised`. Night agents never set that flag —
they go through `CognitiveMap.buildSimpleActivityBone()`, not the full
individualised bone-building path, and `NightAgent.java`'s own comment says
so directly: *"Night agents are not individualised - their known edges are a
preference signal rather than a statement about what is reachable."*
Confirmed in `CognitiveMap.java`. So even if `initialisePrimal` ran for
night agents, this branch would be a no-op regardless.

## (2), the hard avoid-set, is real but risks (3) if "fixed" naively

The obvious-looking fix — call the 5-arg `dijkstraAlgorithm` instead of the
3-arg one, so `directedEdgesToAvoid` gets threaded through — also turns on
`subGraphInitialisation()`, and that is **not** a no-op for night agents.
`regionCondition()` gates on `properties.isRegionBasedNavigation()`, and that
flag is not hardcoded false for night agents: it's a **per-agent
probabilistic draw**, in `Heuristics.java`:

```java
if (regionsAvailable() && random.nextDouble() < probabilityUsingRegions) {
  ap.addElement(RouteChoiceElement.REGION_BASED_NAVIGATION);
}
```

`NightAgent.planRoute()` calls `initialiseHeuristics(true)` before routing —
the same shared per-agent heuristics sampling day agents use — so some
fraction of night agents, every run, draw region-based navigation on.
Switching to the 5-arg call would silently enable region-subgraph routing
for exactly that random subset, directly contradicting
`RoadDistancePathFinder`'s (night) own stated invariant: *"night navigation
is not region-based... night agents route on the whole community network."*
That line is deliberate, documented, and unrelated to lighting — undoing it
as a side effect of adding barrier avoidance would be a real regression, not
a fix.

## (3), the soft barrier preference, would be a complete no-op today anyway

Even setting the region risk aside: calling `costPerceptionError()` for
night agents wouldn't change anything under the current architecture. It
reads `agent.getCognitiveMap().getAgentKnownBarriers()`, and that set is
only ever populated by `CognitiveMap.findKnownBarriers()` — which is called
from `identifyKnownUrbanElements()`, part of the **full individualised**
bone-building path (`CognitiveMap.java`, the method building from
`cognitiveCollage`/polygons). `buildSimpleActivityBone()`, which night
agents actually use, never calls it. So for every night agent,
`getAgentKnownBarriers()` returns the field's untouched initial value — an
empty set — `anyBarrierKnown()` is always false, and `costPerceptionError()`
falls through to exactly the same `drawFromDistribution(1.0, 0.10, null)`
already being drawn directly. Confirmed by reading both methods; not
inferred.

Making barrier preferences actually mean something for night agents would
need `findKnownBarriers()` (or an equivalent) reachable from
`buildSimpleActivityBone()` — a change to shared `CognitiveMap` behaviour
used well beyond the night module, not a routing-file change, and a
materially bigger and more invasive piece of work than this line in the
register implied.

## What I did instead

Nothing to the routing code. Writing this up is the deliverable: the
register's one line bundles a no-op ((1)), a real-but-hazardous change ((2)),
and a currently-inert one ((3)) into what reads like a single fix. Treating
it as one and reaching for the obvious "just call the 5-arg overload" patch
would have quietly turned region-based routing on for a random subset of
night agents — a worse outcome than leaving this alone.

## If this is worth doing at all

Two independent, smaller decisions, not one:

- **(2)** could be done safely by threading only the barrier-avoidance edge
  set through, without calling `subGraphInitialisation()` at all — i.e. not
  reusing `initialisePrimal` wholesale, but replicating just its
  `getEdgesToAvoid()` half in `DijkstraRoadDistanceNight`. Feasible, but
  needs an explicit decision that night agents *should* hard-avoid barrier
  edges the way day agents do, which nothing currently states either way.
- **(3)** needs the `CognitiveMap` prerequisite above before it's worth
  touching at all, and only then the actual undecided question the code's
  own Javadoc already names: should barrier preference apply at night, and
  should it use `findKnownBarriers()`'s edge-level notion of "known" or the
  community-level one night agents already partly get via
  `SharedCognitiveMap.communityKnownBarriers`.

Both are genuine design calls for Gabriele, not bugs with one obviously
right answer — closer in kind to the vulnerable-agent avoid-set question
[`VALIDATION.md`](VALIDATION.md) surfaced than to C1–C3, C5, C6 or B1.
