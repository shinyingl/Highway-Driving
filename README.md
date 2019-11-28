# Path Planning Project
Self-Driving Car Engineer Nanodegree Program
   

## The project
To safely nevigate a car on a highway with provided localization and sensor fusion data while meeting the below requirements:
* The car drives at <= 50 MPH . 
* The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
* The car must not come into contact with any of the other cars on the road.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes

There is more information regarding the data and instruction of this project at this [repo](https://github.com/udacity/CarND-Path-Planning-Project).

## The video
Several run with simulator shows no viloations of the above requirement. One of the recorded video can be found in the below video on youtube.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=IAzth7mGg1w
" target="_blank"><img src="README_image/pic0_VideoScreenShot.png" 
alt="Video" width="720" height="540" border="10" /></a>

## Reflection

### General Path Generation Model Documentation
The decision on the behavior decision is made as the flow chart below
![FlowChart](README_image/pic1_logic.png)

The lane change decision is made by calculating the cost function. Three wighting factors 'slowfactor', 'gap factor', and 'diffvfactor' are used to as tuning parameters to optimize the performace.  

* `cost_keep += 49.5/FrontCarSpeed * slowfactor;` --> inverse propotional to the speed of the front car. 
* `cost_right  += gapfactor*gap_th/abs(gap_s) + (FrontCarSpeed - RightCarSpeed)/diffvfacotr; ` --> 1. inverse propotional the the gap_s (the gap between my car and the car in the front) 2. prefer right car speed ahead is faster for lane change
* `cost_left  += gapfactor*gap_th/abs(gap_s) + (FrontCarSpeed - LeftCarSpeed)/diffvfacotr;` --> same as right lane change cost function.

I notice that my car speed up and slow down between 2 speeds if there is a slow car in the front before there is a chance to change lane. I guess maybe the PID control coming up later in the program will shine some light on how to solve this problem. 

## Basic Build Instructions

1. Clone the folder `src`.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```



