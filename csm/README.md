# Canonical Scan Matcher (CSM)

For getting a quick understanding about PLICP view the [2008-icra-plicp-slides.pdf](https://censi.science/pub/research/2008-icra-plicp-slides.pdf) which is also located in this directory.

The author **Andrea Censi** explains compares PLICP and ICP briefly in a short video which can be found in CSM source directory.

For further details about using this package refer to the manual. The manual uses few log files for experiments
which were not present in the source code but were retrieved from other sources and have been collected in the
`experiments` directory.

For more information about the C(anonical) Scan Matcher, see the webpage: http://purl.org/censi/2007/csm .

This is the "master" branch of CSM, which uses GSL.

There is also another branch, called [``csm_eigen``][branch], which uses the ``eigen`` library. 
This branch is the work of people working at U. Freiburg and Kuka, including
Christoph Sprunk and Rainer Kuemmerle.

[branch]: https://github.com/AndreaCensi/csm/tree/csm_eigen

Binary install (via ROS)
------------------------------

(November 2015) Now you can install binary on Ubuntu (via ROS). As of today limited to Ubuntu Saucy and Trusty. To do so:

```
sudo apt-get install ros-indigo-csm
```

The package name contains "ROS" specific info, but you can use this as a standalone CSM library. It goes into these directory:

```
/opt/ros/indigo/include/csm
/opt/ros/indigo/lib/libcsm-static.a
/opt/ros/indigo/lib/libcsm.so
```
