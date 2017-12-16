^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multirobot_map_merge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.1 (2017-12-16)
------------------
* fix bugs in CMakeLists.txt: install nodes in packages, so they get shipped in debian packages. fixes `#11 <https://github.com/hrnr/m-explore/issues/11>`_
* map_merge: add bibtex to wiki page
* Contributors: Jiri Horner

2.1.0 (2017-10-30)
------------------
* no major changes. Released together with explore_lite.

2.0.0 (2017-03-26)
------------------
* map_merge: upgrade to package format 2
* node completely rewritten based on my work included in opencv
* uses more reliable features by default -> more robust merging
* known_init_poses is now by default false to make it easy to start for new users
* Contributors: Jiri Horner

1.0.1 (2017-03-25)
------------------
* map_merge: use inverted tranform
  * transform needs to be inverted before using
* map_merge: change package description
  * we support merging with unknown initial positions
* Contributors: Jiri Horner

1.0.0 (2016-05-11)
------------------
* initial release
* Contributors: Jiri Horner
