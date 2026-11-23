## Overview

This package provides greedy frontier-based exploration. When node is
running, robot will greedily explore its environment until no frontiers
could be found. Movement commands will be send to
[move_base](https://index.ros.org/p/move_base).

![width="755px"](doc/screenshot.png)

Unlike similar packages, `explore_lite` does not create its own costmap,
which makes it easier to configure and more efficient (lighter on
resources). Node simply subscribes to
[nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html)
messages. Commands for robot movement are send to
[move_base](https://index.ros.org/p/move_base) node.

Node can do frontier filtering and can operate even on non-inflated
maps. Goal blacklisting allows to deal with places inaccessible for
robot.

[Youtube](https://www.youtube.com/watch?v=op0L0LyGNwY&rel=0)

## Architecture

`explore_lite` uses [move_base](https://index.ros.org/p/move_base) for
navigation. You need to run properly configured
[move_base](https://index.ros.org/p/move_base) node.

![width="755px"](doc/architecture.svg)

`explore_lite` subscribes to a
[nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html)
and
[map_msgs/msg/OccupancyGridUpdate](https://docs.ros.org/en/rolling/p/map_msgs/msg/OccupancyGridUpdate.html)
messages to construct a map where it looks for frontiers. You can either
use costmap published by [move_base](https://index.ros.org/p/move_base)
(ie. `<move_base>/global_costmap/costmap`) or you can use map
constructed by mapping algorithm (SLAM).

Depending on your environment you may achieve better results with either
SLAM map or costmap published by `move_base`. Advantage of
`move_base` costmap is the inflation which helps to deal with some
very small unexplorable frontiers. When you are using a raw map produced
by SLAM you should set the `min_frontier_size` parameter to some
reasonable number to deal with the small frontiers. For details on both
setups check the `explore.launch` and `explore_costmap.launch`
launch files.

## Setup

Before starting experimenting with `explore_lite` you need to have
working [move_base](https://index.ros.org/p/move_base) for navigation.
You should be able to navigate with
[move_base](https://index.ros.org/p/move_base) manually through
[rviz](https://index.ros.org/p/rviz). Please refer to
[navigation](https://index.ros.org/p/navigation) for setting up
[move_base](https://index.ros.org/p/move_base) and the rest of the
navigation stack with your robot.

You should be also able to to navigate with
[move_base](https://index.ros.org/p/move_base) though unknown space in
the map. If you set the goal to unknown place in the map, planning and
navigating should work. With most planners this should work by default,
refer to [navfn](https://index.ros.org/p/navfn) if you need to setup
this for [navfn](https://index.ros.org/p/navfn) planner (but should be
enabled by default). Navigation through unknown space is required for
`explore_lite`.

If you want to use costmap provided by
[move_base](https://index.ros.org/p/move_base) you need to enable
unknown space tracking by setting `track_unknown_space: true`.

If you have [move_base](https://index.ros.org/p/move_base) configured
correctly, you can start experimenting with `explore_lite`. Provided
`explore.launch` should work out-of-the box in most cases, but as
always you might need to adjust topic names and frame names according to
your setup.

## ROS API

### explore

Provides exploration services offered by this package. Exploration will
start immediately after node initialization.

#### Actions Called

- `move_base`
  ([MoveBaseAction](https://index.ros.org/p/move_base_msgs)):
  [move_base](https://index.ros.org/p/move_base) actionlib API for
  posting goals. See [move_base](https://index.ros.org/p/move_base) for
  details. This expects [move_base](https://index.ros.org/p/move_base)
  node in the same namespace as `explore_lite`, you may want to remap
  this node if this is not true.

#### Subscribed Topics

- `costmap`
  ([nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html)):
  Map which will be used for exploration planning. Can be either costmap
  from [move_base](https://index.ros.org/p/move_base) or map created by
  SLAM (see above). Occupancy grid must have got properly marked unknown
  space, mapping algorithms usually track unknown space by default. If
  you want to use costmap provided by
  [move_base](https://index.ros.org/p/move_base) you need to enable
  unknown space tracking by setting `track_unknown_space: true`.
- `costmap_updates`
  ([map_msgs/msg/OccupancyGridUpdate](https://docs.ros.org/en/rolling/p/map_msgs/msg/OccupancyGridUpdate.html)):
  Incremental updates on costmap. Not necessary if source of map is
  always publishing full updates, i.e. does not provide this topic.

#### Published Topics

- `~frontiers`
  ([visualization_msgs/msg/MarkerArray](https://docs.ros.org/en/rolling/p/visualization_msgs/msg/MarkerArray.html)):
  Visualization of frontiers considered by exploring algorithm. Each
  frontier is visualized by frontier points in blue and with a small
  sphere, which visualize the cost of the frontiers (costlier frontiers
  will have smaller spheres).

#### Parameters

- `~robot_base_frame` (`string`, default: `base_link`): The name of
  the base frame of the robot. This is used for determining robot
  position on map. Mandatory.
- `~costmap_topic` (`string`, default: `costmap`): Specifies topic of
  source
  [nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html).
  Mandatory.
- `~costmap_updates_topic` (`string`, default: `costmap_updates`):
  Specifies topic of source
  [map_msgs/msg/OccupancyGridUpdate](https://docs.ros.org/en/rolling/p/map_msgs/msg/OccupancyGridUpdate.html).
  Not necessary if source of map is always publishing full updates, i.e.
  does not provide this topic.
- `~visualize` (`bool`, default: `false`): Specifies whether or not
  publish visualized frontiers.
- `~planner_frequency` (`double`, default: `1.0`): Rate in Hz at which
  new frontiers will computed and goal reconsidered.
- `~progress_timeout` (`double`, default: `30.0`): Time in seconds.
  When robot do not make any progress for `progress_timeout`, current
  goal will be abandoned.
- `~potential_scale` (`double`, default: `1e-3`): Used for weighting
  frontiers. This multiplicative parameter affects frontier potential
  component of the frontier weight (distance to frontier).
- `~orientation_scale` (`double`, default: `0`): Used for weighting
  frontiers. This multiplicative parameter affects frontier orientation
  component of the frontier weight. This parameter does currently
  nothing and is provided solely for forward compatibility.
- `~gain_scale` (`double`, default: `1.0`): Used for weighting
  frontiers. This multiplicative parameter affects frontier gain
  component of the frontier weight (frontier size).
- `~transform_tolerance` (`double`, default: `0.3`): Transform
  tolerance to use when transforming robot pose.
- `~min_frontier_size` (`double`, default: `0.5`): Minimum size of the
  frontier to consider the frontier as the exploration goal. In meters.

#### Required tf Transforms

- `global_frame` → `robot_base_frame`: This transformation is usually
  provided by mapping algorithm. Those frames are usually called `map`
  and `base_link`. For adjusting `robot_base_frame` name see
  respective parameter. You don't need to set `global_frame`. The
  name for `global_frame` will be sourced from `costmap_topic`
  automatically.

## Acknowledgements

This package was developed as part of my bachelor thesis at
[Charles University](http://www.mff.cuni.cz/to.en/) in Prague.



```
@masterthesis{Hörner2016,
  author = {Jiří Hörner},
  title = {Map-merging for multi-robot system},
  address = {Prague},
  year = {2016},
  school = {Charles University in Prague, Faculty of Mathematics and Physics},
  type = {Bachelor's thesis},
  URL = {https://is.cuni.cz/webapps/zzp/detail/174125/},
}
```



This project was initially based on [explore](explore) package by
Charles !DuHadway. Most of the node has been rewritten since then. The
current frontier search algorithm is based on
[frontier_exploration](https://index.ros.org/p/frontier_exploration) by
Paul Bovbel.
