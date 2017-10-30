^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package explore_lite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
