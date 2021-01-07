^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package explore_lite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.4 (2021-01-07)
------------------
* use C++14
* Contributors: Jiri Horner

2.1.3 (2021-01-03)
------------------
* add missing dependencies to catkin_package calls
* Contributors: Jiri Horner

2.1.2 (2021-01-02)
------------------
* support for ROS Melodic
* Contributors: Jiri Horner

2.1.1 (2017-12-16)
------------------
* explore: fix min_frontier_size bug
  * min_frontier_size parameter was not used to at all
  * adjust min_frontier size filtering according to map resolution
* fix bugs in CMakeLists.txt
  * install nodes in packages, so they get shipped in debian packages. fixes `#11 <https://github.com/hrnr/m-explore/issues/11>`_
* explore: update documentation
* Contributors: Jiri Horner

2.1.0 (2017-10-30)
------------------
* explore: get rid of boost
* explore: rework launchfiles
* new visualisation of frontiers
* remove navfn library from dependencies
* use frontier seach algorithm from Paul Bovbel
  * much better than previous version of search
  * https://github.com/paulbovbel/frontier_exploration.git
* fix deadlock in explore
  * reworked expore to use timer instead of separate thread for replanning
  * fix deadlock occuring between makePlan and goal reached callback
* Contributors: Jiri Horner

2.0.0 (2017-03-26)
------------------
* explore: migrate to package format 2
* explore: remove internal version of navfn_ros
  * my changes are included in the ros since kinetic
* Contributors: Jiri Horner

1.0.1 (2017-03-25)
------------------
* update documentation
* Contributors: Jiri Horner

1.0.0 (2016-05-11)
------------------
* initial release
* Contributors: Jiri Horner, duhadway-bosch, pitzer
