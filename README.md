# PID_Differential_Drive

## Introduction
Differential Drive Robot Prototype for Warehouse Automation using 2D Kinematics.

### Assumptions:<br>
1. Linear Speed will vary from 0 to 1 (Proportional Control is used). <br>
2. Angular speed is constrained between [-0.3,0.3] rad/s for smooth motion. <br>
3. Landmarks are in the form of cubes of dimension 0.3,0.3,0.5.
4. The number of landmarks can be increased or decreased by uncommenting the 
waypoints in the main code.

### Key functions used in the code:
1. get_60_degree_range: Check for obstacles from -30 deg to +30 deg in 
front of the robot (Array data from onboard Lidar).
2. distance: Calculation of Euclidean Distance from the current position to the 
destination coordinates.
3. angle_calculation_on_the_run: Removes the ambiguity of turning 
ACW/CW while traversing.
4. PD Controller for Rotation: The PD controller is tuned to make the bot move in 
a straight line. PID would have increased the computation and in the given scenario PD 
controller was sufficient as the robot will already be in a position where the rotation 
angle will be close to the target angle. The addition of PID added to an oscillatory 
motion in the robot(undesired for indoor motion).
5. P Controller for Linear Motion: The linear velocity constraints are [0,1]. P 
controller increased/decreased velocity according to the Euclidean distance 
calculated during runtime.

_Note: Both PD and P controller are used in the main code not as a separate function_

### Images of RVIZ and Gazebo
![Screenshot from 2023-10-18 17-33-30](https://github.com/adi27s/PID_Differential_Drive/assets/123253804/a7ff2d27-ad33-4c32-a146-a00e6792f9d5)

![Screenshot from 2023-10-18 17-33-35](https://github.com/adi27s/PID_Differential_Drive/assets/123253804/a161c87c-38ec-4cdc-a466-a93f28fe60d4)

![image](https://github.com/adi27s/PID_Differential_Drive/assets/123253804/08cc713e-7c34-4e27-a819-01fdcf5e0a85)

![image](https://github.com/adi27s/PID_Differential_Drive/assets/123253804/2c84bd10-6aae-4637-a937-38e6f935a6ed)

![image](https://github.com/adi27s/PID_Differential_Drive/assets/123253804/4a0011ca-6031-47f3-a51e-b314ab1c6570)

### Tree

![image](https://github.com/adi27s/PID_Differential_Drive/assets/123253804/de36046a-c187-4ac8-942f-be6ac3e591ae)



