# OptiTrack Process Node For Drone Command and Control

## Description：

The OptiTrack node is responsible for processing OptiTrack position feedback for quadrotor experiments in [Flight System and Control Lab (FSC)](https://www.flight.utias.utoronto.ca/fsc/index.php/payload/). 
The OptiTrack node should be download and installed on the ground station computer.  The OptiTrack node supports multiple rigid bodies defined in the argument of the launch file.

## How to use:

- You may install the OptiTrack node using the following common commands:

> 1. $ cd yourws/src/
> 2. $ git clone https://github.com/LonghaoQian/optitrack_broadcast
> 3. $ cd ../..
> 4. $ catkin build

- For gazebo simulation, use `emulator_for_gazebo.launch`

- For flight experiment, use `optitrack_broadcast_p300.launch`

- Adjust the arguments in the pkg node based on the rigid bodies defined in Motive :

**For Gazebo simulation:**

>adjust the argument `"objects"` according to the names of the rigid bodies defined in Motive software in the following XML node:
`<arg  name="objects"  default="UAV0 UAV1 UAV2 Payload"/>`
The default name list is UAV0 , UAV1, UAV2, Payload

**For flight experiment:**
>make sure the IP address of the desktop running Motive is correct. The IP is defined in
`<arg  name="server"  default="192.168.2.230"/>`

>Currently, the IP address of the Motive desktop is the default one: `192.168.2.230` 

>adjust the value of  `args`  according to the names of the rigid bodies defined in Motive software in the following XML node:
`<node pkg = "optitrack_broadcast" name = "broadcast_1" type = "broadcast_position_velocity"  output="screen"  args="UAV cf1 cf2 cf3 cf4 cf5"/>`
The default name list is UAV, cf1, cf2, cf3, cf4, cf5

- Data structure:

The OptiTrack node utilizes a custom ros data structure called `Mocap.msg` to reduce the wireless communication burden for experiments. `Mocap.msg` is defined as follows:

>std_msgs/Header header
float32[3] position ## [m]
float32[3] velocity ## [m/s]
float32[3] angular_velocity ## [rad/s]
float32[4] quaternion ##

Copy  `Mocap.msg` to the msg folder of your project and add this custom msg to your `CMakelist.txt` to use the data type.

>add_message_files(
DIRECTORY msg
FILES
Mocap.msg
)

## Velocity Estimation Algorithm

Motive software only streams the position and attitude of the defined rigid bodies to the ground station computer so the OptiTrack node calculates the velocity and angular velocity of the rigid body with the following algorithm. We denote ![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7Bx%7D_%7Braw%7D) as the raw data streamed form the Motive. Then the velocity estimation at the Nth time step is denoted as ![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7Bv%7D_%7BI%2Craw%7D%5BN%5D)calculated as follows:

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7Bv%7D_%7BI%2Craw%7D%5BN%5D%20%5Capprox%20%28%5Cboldsymbol%7Bx%7D_%7Braw%7D%5BN%5D%20-%20%5Cboldsymbol%7Bx%7D_%7Braw%7D%5BN-1%5D%29/dt)

where dt is the Nth time step. The attitude of the rigid body sent by Motive is expressed as quaternion: ![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7Bq%7D%5BN%5D%20%3D%20%5Cbegin%7Bbmatrix%7D%20q_0%20%26%20q_1%20%26%20q_2%20%26%20q_3%20%5Cend%7Bbmatrix%7D%5ET). Two auxiliary matrices are defined as follows:

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5Cmathcal%7BL%7D%7D%28%5Cboldsymbol%7Bq%7D%29%20%3D%20%5Cbegin%7Bbmatrix%7D%20-%20q_1%20%26%20q_0%20%26%20q_3%20%26%20-q_2%5C%5C%20-q_2%20%26%20-q_3%20%26%20q_0%20%26%20q_1%5C%5C%20-q_3%20%26%20q_2%20%26%20-q_1%20%26%20q_0%20%5Cend%7Bbmatrix%7D%3B%5Cboldsymbol%7B%5Cmathcal%7BR%7D%7D%28%5Cboldsymbol%7Bq%7D%29%20%3D%20%5Cbegin%7Bbmatrix%7D%20-q_1%20%26%20q_0%20%26%20-q_3%20%26%20q_2%5C%5C%20-q_2%20%26%20q_3%20%26%20q_0%20%26%20-q_1%5C%5C%20-q_3%20%26%20-q_2%20%26%20q_1%20%26%20q_0%20%5Cend%7Bbmatrix%7D)
Then we can calculate the rotation matrix at the Nth time step between the body fixed frame of the rigid body and the inertial frame :

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7BR%7D_%7BIB%7D%5BN%5D%20%3D%20%5Cboldsymbol%7B%5Cmathcal%7BR%7D%7D%28%5Cboldsymbol%7Bq%7D%5BN%5D%29%20%5Cboldsymbol%7B%5Cmathcal%7BL%7D%7D%28%5Cboldsymbol%7Bq%7D%5BN%5D%29%20%5ET)

The angular velocity estimation is calculated by using the Lie algebra of the rotation matrix. The ![equation](https://latex.codecogs.com/gif.latex?%7B%7D%5E%7B%5Cvee%7D) map is the inverse of the Lie map. 

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5Comega%7D_%7BB%2Craw%7D%5BN%5D%20%5Capprox%20-%20%28%5Cboldsymbol%7BR%7D_%7BBI%7D%5BN%5D%20%5Cboldsymbol%7BR%7D_%7BIB%7D%5BN-1%5D/dt%29%5E%7B%5Cvee%7D)

After obtaining the raw velocity estimation, the following sliding window filter is used to provide a filtered result for control:

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7Bv%7D_%7BI%7D%5BN%5D%20%3D%20%5Csum%5E%7Bm-1%7D_%7Bi%20%3D%200%7D%20%5Cmu_j%20%5Cboldsymbol%7Bv%7D_%7BI%2Craw%7D%5BN-i%5D%3B%20%5Csum%5E%7Bm-1%7D_%7Bi%20%3D%200%7D%20%5Cmu_j%20%3D%201)

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5Comega%7D_%7BB%7D%5BN%5D%20%3D%20%5Csum%5E%7Bm-1%7D_%7Bi%20%3D%200%7D%20%5Cmu_j%20%5Cboldsymbol%7B%5Comega%7D_%7BB%2Craw%7D%5BN-i%5D%3B%20%5Csum%5E%7Bm-1%7D_%7Bi%20%3D%200%7D%20%5Cmu_j%20%3D%201)

The size of the window is set to 4.
