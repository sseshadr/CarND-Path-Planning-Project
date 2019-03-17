# CarND-Path-Planning-Project
Create a path planner that is able to generate and follow a trajectory in a virtual highway scenario. Perform safe lane changes as necessary and travel near speed limit for as long as possible.
   
#### NOTE: This project involves the Term 3 Simulator which can be downloaded [here] (https://github.com/udacity/self-driving-car-sim/releases). It also depends on the C++ eigen library which can be downloaded [here] (https://github.com/eigenteam/eigen-git-mirror).

[//]: # (Image References)

[image1]: ./readme_media/statemchine.png "Behavioral Planning Logic"
[image2]: ./readme_media/20milestone.JPG "20 milestone"
[image3]: ./readme_media/laneChange.gif "Lane Change"
[image4]: ./readme_media/safeDistance.png "Safe Distance"

---

### Project Notes
The path planning algorithm interfaces with the simulator to get the ego car's localization data and the sensor fusion data of the cars around it. Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

Using this information, the car needs to plan and execute a trajecotry that satisfies the following conditions:

1. Drive at least 4.32 miles without an incident. An incident is violation if one of the following rules are NOT met:
    * Max acceleration or jerk are not exceeded
    * Car does not have collisions
    * Car stays in its lane except for when it needs to change lanes
2. Drive at speed limit whenever possible
3. Car is able to change lanes

The path planning algorithm implements the following sections to achieve this.

#### Behavioral planning:
The first part of the algorithm is to determine whether the car needs to:
* Stay in its lane
* Change to the left lane
* Change to the right lane

To do this, we can use the sensor fusion data. Specifically, we can use the frenet co-ordinate information (s - distance traveled across the highway, d - lateral distance across lanes) to determine whether a particular car is in the ego-lane or the lane to the left or the lane to the right. Depending on this, if a car is within a ego distance buffer, we can say the following:
* If a car in the ego lane is ahead of the ego car and within the ego buffer, it is "too close" and needs to slow down
* If the car is slowing down, that is the only reason to change lanes. So we then first look left to see if that is safe to shift. We do this not just for the car ahead in an adjacent left lane but look behind as well. This is a little different from what we did for slowing down in the ego lane which only takes the car ahead into account.
* If the left lane is not safe, we look right to see if it safe to shift. The logic is similar to the left lane shift. 

The logic can be better understood with the state machine diagram below:

![alt text][image1]

Once we have determined what action to perform, we can have boolean flags to keep track of what the car needs to do as decision makers. This will be used while planning the trajectory in the next section.

#### Trajectory generation:
The next part is to determine where the car needs to go based on the decisions in the previous section. To generate the trajectory, we use the spline library from [here] (http://kluge.in-chemnitz.de/opensource/spline/). To use the spline library, we need to supply a set of coarse waypoints to construct a smooth trajectory. We do this by 
1. Including the last 2 waypoints of the previous path to ensure a smooth handover
2. Calculating waypoints ahead of the car 30m, 60m and 90m ahead. This uses the prospective lane that the car needs to be in (set by the behavioral planner) and automatically handles waypoints necessary to shift lanes

Next for the actual waypoints,
1. Calculate the number of points for the spline. This is based on the car velocity and how many points actually need to be generated (if there are left over points from the previous generation, just use them and generate only to make up for the difference to 50 points)
2. Then for each x point, use the spline to look up the y point to make up the waypoint coordinate and add to the final list

#### Reflection:
For the most part, the simple logic passes the eye test and "human" driving behavior. There are a few things that we could do to make this better:

1. Generate multiple trajectories and use a cost function to determine which path to take. This could theoretically be faster and may also reduce the number of lane changes
2. Have the buffer distance be a function of car's velocity. This safe distance can be calculated using an equation as shown below:

![alt text][image4]
[reference] (https://www.mathworks.com/help/mpc/examples/design-an-adaptive-cruise-control-system-using-model-predictive-control.html)

where D_Default is the standstill default spacing (our buffer currently) and T_gap is the time gap between the vehicles. Using a dynamic approach like this maybe useful in cases where the car is stuck in a constant velocity cluster where all the surrounding cars are moving in a similar pattern and adjusting the safe distance becomes a way to get out of the cluster.

### Rubric Points
#### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.  

---

#### Compilation
The code compiles successfully. Final executable is submitted.

#### Valid Trajectories

1. The car is able to drive at least 4.32 miles without incident. A sample screenshot is provided below where the simulation was terminated after running 20 miles without incident (this takes care of max acceleration, jerk, collision and lane violation as well):

![alt text][image2]

2. The car drives according to the speed limit. The above image also shows the car's speed @ ~49.5 mph which is the reference velocity. We can also calculate the average velocity for the run to be ~43 mph which considering that the speed limit is 50 mph is reasonable with all the slowing down and lane changes.

3. The car is able to change lanes. The animation below shows the car is able to change lanes whenever the car ahead is too close.

![alt text][image3]

4. Reflection is provided. The readme in general explains all the logic at high level and the reflection sections shows how the current logic can be improved.


### Basic Build Instructions

1. Clone this repo. (dependencies not included in src for brevity. Refer to the original repos to include them for a successful build)
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

