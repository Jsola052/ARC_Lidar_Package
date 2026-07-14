# Goal

Before, on master, the SLAM was doing this. The goal of this MR is to do this and that.

# Changes

* A function called Myfunction is added to [this file](https://gitlab.kitware.com/keu-computervision/slam_epita/-/blob/master/slam_lib/src/LocalOptimizer.cxx?ref_type=heads). It takes that as input and this as output. The parameters are bla and bli because. It does this and that following this method
  1. First it does that
  2. Then it does this
  3. And finally it does that
* The previous function is used in foo() to do that. See image below for an example

![myImage](/uploads/fd3c7b2e05117ab69ee0fb1990994465/myImage.png)

# Results

## Reproduce

### Data

I have used those data to test.

### Build

`colcon build --base-paths slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release`

### Parameters

* Change the parameters :
  * leaf_size=5
  * ...

### Run

`ros2 launch my_launch.launch param1:=2`

`ros2 bag play --clock path/to/this_bag.bag`

## Output

* Here is a resulting map
* Here is a resulting trajectory
* Here is an example of input frame and its keypoints (optional)
* Here is a screenshot of the SLAM result

## Performances (optional)

* The overall SLAM process before (on master) was taking 10s
* The overall process takes 9s

# Checklist

- [ ] Camel case everywhere except for ROS variables/parameters
- [ ] Lower case for local variables and lambda functions
- [ ] Upper case for members, methods and tool functions (in Utils)
- [ ] Precise namespace when calling a function (or this->X or classe.X)
- [ ] Align code (for multiline if and while, "&&" or "||" go in upper line to ensure alignement)
- [ ] Check your spaces
    - [ ] between if, while, for and parenthesis
    - [ ] between operators and variables: e.g. a + b
    - [ ] after ","
- [ ] Mind your commit titles/desc (plurals, he/she + "s", correct tags, title should begin by a verb...)
- [ ] Function names should start with a verb, variable names should start with a name
- [ ] Macros should be between {}
- [ ] Do not use negative boolean (i.e. noJoe)
- [ ] Check minimal size of the types (double -> float -> int -> uint)
- [ ] Check const and ref in functions arguments
- [ ] References should be written "type& name", not "type &name"
- [ ] Update documentation
- [ ] Add MR labels [ROS]/[ROS2]/[PV]
- [ ] If ros/ros2, update task table [here](https://gitlab.kitware.com/keu-computervision/slam/-/issues/55)
- [ ] Add a comment over each non trivial function in header files
- [ ] Add a header to each new file

# What remains to solve

* this
* this
* that
