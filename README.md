During my undergrad final year project, which was about implementing a field robot with autonomous navigation, I faced an issue.

➡ I had a requirement to plot a set of ROS topics (some odometry data) to gain insights. 

We were experimenting to find the best possible sensor fusion method for our robot. 

So the idea was to plot empirical data with the ground truth and compare.


➡ What I did was,

1. Collect a ROS bag with necessary topics
2. Run it
3. Subscribe to the topics I need
4. Log them to a CSV
5. Visualize using Matplotlib.

This repo contains those Python scripts. That's a lot of work when one has to do it over and over again.


➡ A few months after the completion of the project I came across [PlotJuggler](https://github.com/facontidavide/PlotJuggler), by chance. (well that was too late!)

This tool solves the same problem I faced, in a much better way, and it integrates well with ROS.


➡ If you are someone into robotics and need a time series data viz tool, look no further. 

PlotJuggler is the way to go!

---

# LiDAR Odometry Testing

## Statistics

* Duration 60 seconds
* Command: 
```
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.05
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.05
```

* Results:

```
odom_yaw:       min = -3.14      max = 3.14       avg = 0.12      
odom_linear_x:  min = -2.46      max = 2.16       avg = 0.05      
odom_angular_z: min = -39.34     max = 0.62       avg = -0.01     
twist_linear_x: min = 0.00       max = 0.05       avg = 0.05      
twist_angular_z:min = 0.00       max = 0.05       avg = 0.05
```

# Odometry messages data

```shell
bimalka98@LAP-BIMALKA98:~$ ./odom.sh 
/odom/lidar
Type: nav_msgs/Odometry

Publishers: 
 * /play_1690038154324027230 (http://LAP-BIMALKA98:43943/)

Subscribers: None


header: 
  seq: 16
  stamp: 
    secs: 1689233055
    nsecs:  37512406
  frame_id: "odom"
child_frame_id: "base_link"
pose: 
  pose: 
    position: 
      x: 0.23845223652010977
      y: 0.6142565424490137
      z: -0.011431738076478983
    orientation: 
      x: 0.0033254036126051656
      y: 0.004497113294042495
      z: 0.036553529440019425
      w: 0.9993160446767136
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.10810701618394586
      y: -0.056123804611706796
      z: 0.0004582837412625423
    angular: 
      x: 0.00017835980908364397
      y: -1.733515524174483e-05
      z: -0.0021564923982788195
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
/odom/wheel
Type: nav_msgs/Odometry

Publishers: 
 * /play_1690038154324027230 (http://LAP-BIMALKA98:43943/)

Subscribers: None


header: 
  seq: 117
  stamp: 
    secs: 1689233056
    nsecs: 804661339
  frame_id: "odom"
child_frame_id: "base_link"
pose: 
  pose: 
    position: 
      x: 0.025897095203399653
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
/imu/data
Type: sensor_msgs/Imu

Publishers: 
 * /play_1690038154324027230 (http://LAP-BIMALKA98:43943/)

Subscribers: None


header: 
  seq: 12
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
orientation: 
  x: 359.8125
  y: -5.25
  z: 1.3125
  w: 0.0
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: 
  x: -0.0021816615480929613
  y: -0.0021816615480929613
  z: 0.003272492438554764
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: 
  x: 0.029999999329447746
  y: 0.009999999776482582
  z: -0.20999999344348907
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
/robot_rtk_xyz
Type: geometry_msgs/PointStamped

Publishers: 
 * /play_1690038154324027230 (http://LAP-BIMALKA98:43943/)

Subscribers: None


header: 
  seq: 7
  stamp: 
    secs: 1689233060
    nsecs: 922173500
  frame_id: "map"
point: 
  x: -3.347173924976582
  y: 3.020952906490796
  z: 0.0
---
/odometry/filtered
Type: nav_msgs/Odometry

Publishers: 
 * /play_1690038154324027230 (http://LAP-BIMALKA98:43943/)

Subscribers: None


header: 
  seq: 94
  stamp: 
    secs: 1689233062
    nsecs: 637989759
  frame_id: "odom"
child_frame_id: "base_link"
pose: 
  pose: 
    position: 
      x: 0.025910185575430438
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [9.99999400027635e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.999994000276347e-10, 0.0, 0.0, 0.0, -3.1495764014662536e-14, 0.0, 0.0, 4.99875067735381e-07, 0.0, 5.816467904292292e-21, 0.0, 0.0, 0.0, 0.0, 4.997502602951425e-07, 0.0, 0.0, 0.0, 0.0, 5.8164679042922926e-21, 0.0, 4.997502602951425e-07, 0.0, 0.0, -3.149576401466254e-14, 0.0, 0.0, 0.0, 0.5630291723790509]
twist: 
  twist: 
    linear: 
      x: -6.573883017878992e-12
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [9.999988250845988e-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.999988250845988e-10, 0.0, 0.0, 0.0, 1.4591400975545615e-39, 0.0, 0.0, 4.998126484268216e-07, 0.0, 3.6115345217972575e-30, 0.0, 0.0, 0.0, 0.0, 4.992522728785569e-07, 0.0, 0.0, 0.0, 0.0, 3.611534521797257e-30, 0.0, 4.992522728785569e-07, 0.0, 0.0, 1.4591400975545615e-39, 0.0, 0.0, 0.0, 9.999985000674396e-10]
---
bimalka98@LAP-BIMALKA98:~$
```
