## Abstract
One of the key features of an autonomous robot is its ability to determine its position precisely and accurately. The advancement of positioning technologies has been a crucial factor in the deployment of mobile robots across various sectors. To validate our major in Robotics at the University of Technology of Belfort-Montbéliard (UTBM) during the Spring 2024 semester, we undertook a project designed to understand the challenges and gain hands-on experience with mobile robot positioning technologies.  

The main goal of the project is to enable the robot to navigate autonomously to a distant destination. The robot should be capable of determining its precise position, navigating to the destination, and detecting its target. The working area was divided into two zones: a corridor and a classroom. A Wi-Fi navigation system was employed in the corridor to determine the robot’s exact location, while Simultaneous Localization and Mapping (SLAM) was used for navigation within the classroom. A camera was then utilized to detect the target.

### Wi-Fi navigation system
Wi-Fi positioning involves using the signal strengths of multiple Wi-Fi access points (APs) to determine the robot's position within a known area. Different methods are used to create such a positioning system. The popular GPS (global positioning system) can obtain high positioning accuracy outdoors. However, most of the robots in the above application scenarios are in complex indoor environments, and it is almost impossible to apply GPS to indoor positioning.  
A fingerprinting method was used to implement the Wi-Fi positioning system
#### Setup
First, define your Access points. They can be added in the target_essids of the wifi_signal.  
Then, you should set your working space length (long and large). The get_space_length can be used for this purpose.  
#### Creating the fingerprint database
Now you can start using the system for predicting the position with Wi-Fi:
```
cd wifi_positioning
python fingerprint_collector.py
```
or 
```
rosrun wifi_positioning fingerprint_collector
```
The robot should start collecting fingerprint. After the script execution, a wifi_data.csv should be created containing the RSSI signal of each Access Point for each coordinate x,y in the space. (An SQLite database was also created but not used).  

#### Testing the positioning system
The position_predictor.py script contains the implementation of the KNN model used to predict the position. It uses the generated wifi_data.csv file to create a pickle file (knn_model.pkl). You don't have to run this script before the test. The position_fetcher.py is the node responsible of predicting the robot's position. It uses the position_predictor for this purpose. You should just run it and to get the robot's position:
```
cd wifi_positioning
python position_fetcher.py
```
or 
```
rosrun wifi_positioning position_fetcher
```

#### Wi-Fi navigation
A navigation was implemented based on the Wi-Fi positioning system. 
The algorithm was designed to move the robot towards a predefined destination using a series of steps that involved position prediction, angle calculation, and movement execution: 

- Position Prediction: 
The robot used the Wi-Fi fingerprinting method to estimate its current position. This involved measuring the signal strength from multiple access points and comparing these readings to a pre-built database of signal strengths at known locations. 
- Rotation angle calculations 
Once the current position was estimated, the algorithm calculated the angle between the robot's current position and the destination. This was done using the arctan2 function, which computes the angle (θ) required to point directly towards the destination from the robot's current location. The arctan2 function considers the difference in the x and y coordinates of the current position and the destination:  

``` θ=arctan2(ydestination​−ycurrent ​, xdestination​−xcurrent​) ```

- Rotation and Mouvement 
After calculating the angle, the robot performed a rotation to align itself with the direction of the destination. This rotation ensured that the robot was facing the correct direction before moving forward. Following the rotation, the robot moved straight for a fixed distance of 50 cm. This incremental movement allowed the robot to periodically reassess its position and adjust its direction as needed. 
- Repetition 
After each 1m movement, the robot re-estimated its position using the Wi-Fi fingerprinting method. This updated position was then used to calculate the new angle to the destination. The robot repeated the steps of angle calculation, rotation, and forward movement iteratively. This process continued until the robot reached the destination. The iterative approach allowed the robot to make continuous adjustments to its path, ensuring it remained on course towards the target. 
- Obstacle avoidance 
Obstacles were avoided using the Lidar. A ROS node was implemented for this purpose (wifi_positioning/lidar_alert/scripts/obstacle_detector.py). During the whole navigation process, the robot checks for obstacles. If there is an obstacle, the robot rotates according to the obstacle angle, make a smooth movement forward, then rotate again to find its initial orientation

To run it :
```
cd wifi_positioning
python navigator.py
```
or 
```
rosrun wifi_positioning navigator
```

__Note: You should define the Target position. A launch file will be created for the next version. For now, you can define the target in the navigator.py file__


## Navigation package

## QR-Code Tracker

## Scheduler
