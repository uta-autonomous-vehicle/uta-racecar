# UTA-RACECAR

## Overview

This package contains the main running endpoind for everything the group created.

**Keywords:** zed-wrapper, mit-racecar


### Publications

Yet to be attached.

### Images

Yet to be attached.

## Installation

Clone this repo, into your workspace src/ dir and build workspace.

`git clone <> src/ && catkin_make`

### Building from Source

`catkin_make --only-pkg-with-deps uta-racecar`

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/ethz-asl/ros_package_template.git
	cd ../
	catkin_make




## Launch files

* **racecar_launch.launch:** standard run on visible path
* more to be added.


#### Subscribed Topics

* **`/zed/<>`** (ZED modules)


#### Published Topics

None.

## Bugs & Feature Requests

Please report bugs and request features using the Tracker.

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz