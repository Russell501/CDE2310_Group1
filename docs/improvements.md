# Improvements

## Areas for Improvement

### Navigation of Tight Spaces

The frontier detection provided by `explore_lite` was reliable - it was able to accurately identify all viable unmapped spaces as frontiers, as confirmed by monitoring the published frontier topics on Rviz2. The issue lies with the Nav2 planning stack, which was unable to plot feasible paths through very tight spaces. When the planner fails to find a path, `explore_lite` blacklists the frontier, effectively giving up on reachable areas prematurely. The root cause is an overly conservative inflation radius and cost scaling configuration in the Nav2 costmap, which makes the robot's effective footprint significantly larger than its physical size.

### Electrical Stability

The stability of electrical components was a recurring issue throughout the project. Wires frequently came loose due to vibrations from robot movement, LiDAR motor operation, the launching sequence, and other repairs made on the robot. Additionally, unshielded wires with exposed metal risked unintentionally shorting other connections, leading to intermittent failures that were difficult to diagnose.

## Alternative Strategies

### Navigation

- Conduct more testing on smaller and tighter maze configurations to tune `inflation_radius` and `cost_scaling_factor` parameters for the Nav2 costmap. Reducing the inflation radius and increasing the cost scaling factor would allow the planner to route through tight passages while still preferring open space when available.
- Lower the `progress_timeout` so that the robot does not spend excessive time attempting to reach frontiers that may not yield any useful mapping information.
- Evaluate alternative navigation planners such as `SmacPlanner2D` or `SmacPlannerHybrid`, which support a full footprint parameter instead of just a circular radius. This would provide more accurate path feasibility checks around tight passages, particularly for the rectangular profile of the TurtleBot3 Burger.

### Electrical

- Solder as many intermediate wire connections as possible to eliminate loose contact points. Minimise the number of female wire to male pin connections on the Raspberry Pi - instead, share the 5V supply and common ground on a proto board where connections can be soldered securely.
- Apply heat shrink insulation over all exposed wire connections to prevent accidental shorts.
