---
title: Areas for Improvement
---

# 🔗 Navigation

- [Home](index.md)
- [Requirements](requirements.md)
- [Con-Ops](conops.md)
- [High Level Design](high-level-design.md)
- [Interface Control Documents](icd.md)
- [Software Development](software.md)
- [Testing](testing.md)
- [User Manual](user-manual.md)
- [Bill-Of-Materials](bill-of-materials.md)
- [Electrical Subsystem](electrical.md)
- [Mechanical Subsystem](mechanical.md)
- **Improvements** ← _You are here_

---

### Navigation of Tight Spaces

The frontier detection provided by `explore_lite` was reliable — it accurately identified all viable unmapped spaces as frontiers, as confirmed by monitoring the published frontier topics on RViz2. The issue lies with the Nav2 planning stack, which was unable to plot feasible paths through very tight spaces. When the planner fails to find a path, `explore_lite` blacklists the frontier, effectively giving up on reachable areas prematurely.

The root cause is an overly conservative inflation configuration in the Nav2 costmap. With `inflation_radius: 0.25` and `robot_radius: 0.135`, the effective no-go zone around every obstacle extends to 0.385m per side — meaning any gap narrower than ~0.77m is completely impassable to the planner, even though the robot physically only needs ~0.28m of clearance.

**Recommended improvements:**

- Conduct more testing on smaller and tighter maze configurations to properly tune `inflation_radius` and `cost_scaling_factor`. Reducing the inflation radius (e.g. to 0.15m) and increasing the cost scaling factor (e.g. to 5.0) would allow the planner to route through tight passages while still preferring open space when available.
- Lower `progress_timeout` (currently 60s) to 20–30s so the robot does not spend excessive time attempting to reach frontiers that may not yield useful mapping information, and moves on to more productive goals sooner.
- Evaluate alternative navigation planners such as `SmacPlanner2D` or `SmacPlannerHybrid`, which support a full footprint parameter instead of just a circular radius. This would provide more accurate path feasibility checks around tight passages, particularly for the non-uniform profile of the trutlebot with its launch payload.
- Add a retry mechanism to `explore_lite`'s frontier blacklisting logic. Currently, when Nav2 fails to reach a frontier (due to planner failure, progress timeout, or goal cancellation), `explore_lite` permanently blacklists it — the frontier is never attempted again for the rest of the session. This is overly aggressive because the costmap may have changed after the robot explores other areas, making previously unreachable frontiers viable. A better approach would be to replace the simple blacklist (`std::vector<Point>`) with a tracked attempt counter per frontier. On each failure, the counter increments rather than permanently blocking the frontier. `goalOnBlacklist()` would only filter frontiers that have reached a maximum retry count (e.g. 3 attempts). This allows the robot to move on to easier frontiers first, then circle back and retry previously failed ones — which may now succeed due to updated costmap data from exploring other parts of the maze. This change would be particularly effective when combined with the inflation and planner tuning above, as the retries would have a meaningful chance of producing a different planning outcome.

### Electrical Stability

Stability of electrical components was a recurring issue throughout the project. Wires frequently came loose due to vibrations from robot movement, LiDAR motor operation, the launching sequence, and other repairs made on the robot. Additionally, unshielded wires with exposed metal risked unintentionally shorting other connections, leading to intermittent failures that were difficult to diagnose.

**Recommended improvements:**

- Solder as many intermediate wire connections as possible to eliminate loose contact points. Minimise the number of female wire to male pin connections on the Raspberry Pi — instead, share the 5V supply and common ground on a proto board where connections can be soldered securely.
- Apply heat shrink insulation over all exposed wire connections to prevent accidental shorts.
- Implement hardware filtering or a dedicated servo controller to prevent unintended PWM signals during boot, which caused erratic servo jitter.
- Measure and validate all estimated current draws with a multimeter during live operation to ensure components are operating within safe margins.
- Add decoupling capacitors on the 5V and 3.3V rails near sensitive components (servo, motor driver) to reduce power supply noise.
- Investigate a separate 5V BEC (Battery Eliminator Circuit) for the servo to isolate it from the RPi power rail.

---