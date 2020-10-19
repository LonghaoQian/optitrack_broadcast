# The Ros node for computing rigid-body linear and angular velocity based on data streaming from the Motive software

[![Generic badge](https://img.shields.io/badge/OptiTrack%20Data%20Processing-latest-brightgreen)](https://shields.io/) [![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/StrapDown.js/graphs/commit-activity) 

## Node description：

The OptiTrack node is responsible for processing the position and attitude feedback for quadrotor experiments in [Flight System and Control Lab (FSC)](https://www.flight.utias.utoronto.ca/fsc/index.php/payload/). 
The OptiTrack node shall be download and installed on the ground station desktop/computer. The OptiTrack node supports an arbitrary number of rigid bodies defined in the argument of the launch file. The names of the rigid bodies should be the same as the ones used in the Moitve software.

## User guide:

- You may install the OptiTrack node using the following commands:

> 1. $ cd yourws/src/
> 2. $ git clone https://github.com/LonghaoQian/optitrack_broadcast
> 3. $ cd ../..
> 4. $ catkin build

- For gazebo simulations, please use `emulator_for_gazebo.launch` to generate simulated position and velocity data stream.

- For flight experiments, please use `optitrack_broadcast_p300.launch` to obtain the position and velocity measurements. 

- Adjust the arguments in the pkg node based on the rigid bodies defined in Motive software to get corresponding feedbacks.

**For Gazebo simulations:**

>Adjust the argument `"objects"` according to the names of the rigid bodies defined in Motive software in the following XML node:
`<arg  name="objects"  default="UAV0 UAV1 UAV2 Payload"/>`
The default name list is `"UAV0 , UAV1, UAV2, Payload"`.

**For flight experiments:**
>Make sure that the IP address of the desktop running the Motive software is correct. The IP address is specified in
`<arg  name="server"  default="192.168.2.230"/>`

>The IP address of the Motive desktop is the default: `192.168.2.230`.

>Adjust the value of  `args`  according to the names of the rigid bodies defined in Motive software in the following XML node:
`<node pkg = "optitrack_broadcast" name = "broadcast_1" type = "broadcast_position_velocity"  output="screen"  args="UAV cf1 cf2 cf3 cf4 cf5"/>`
The default name list is `"UAV, cf1, cf2, cf3, cf4, cf5"`.

## The data structure:

The OptiTrack node utilizes a customized Ros data structure `Mocap.msg` to reduce the wireless communication burden for experiments. `Mocap.msg` is defined as follows:

std_msgs/Header header

float32[3] position ## [m]

float32[3] velocity ## [m/s]

float32[3] angular_velocity ## [rad/s]

float32[4] quaternion ##

Copy  `Mocap.msg` to the msg folder of your project and add this customized message type to your `CMakelist.txt` in oder to use the data type:

add_message_files(

DIRECTORY msg

FILES

Mocap.msg

)

## The velocity estimation algorithm

The Motive software only streams the position and the attitude of the specified rigid bodies to the ground station computer, so the OptiTrack node calculates the velocity and angular velocity of the rigid bodies based on the following algorithms. We denote ![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7Bx%7D_%7Braw%7D) as the raw data streamed form the Motive software. The velocity estimation at the Nth time step is denoted as ![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7Bv%7D_%7BI%2Craw%7D%5BN%5D), obtained as follows:

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7Bv%7D_%7BI%2Craw%7D%5BN%5D%20%5Capprox%20%28%5Cboldsymbol%7Bx%7D_%7Braw%7D%5BN%5D%20-%20%5Cboldsymbol%7Bx%7D_%7Braw%7D%5BN-1%5D%29/dt)

where dt is the size of the time step between the Nth and the N-1th data feedback. The attitude of the rigid body from the Motive software is expressed in a quaternion: ![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7Bq%7D%5BN%5D%20%3D%20%5Cbegin%7Bbmatrix%7D%20q_0%20%26%20q_1%20%26%20q_2%20%26%20q_3%20%5Cend%7Bbmatrix%7D%5ET). Two auxiliary matrices are defined as follows:

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5Cmathcal%7BL%7D%7D%28%5Cboldsymbol%7Bq%7D%29%20%3D%20%5Cbegin%7Bbmatrix%7D%20-%20q_1%20%26%20q_0%20%26%20q_3%20%26%20-q_2%5C%5C%20-q_2%20%26%20-q_3%20%26%20q_0%20%26%20q_1%5C%5C%20-q_3%20%26%20q_2%20%26%20-q_1%20%26%20q_0%20%5Cend%7Bbmatrix%7D%3B%5Cboldsymbol%7B%5Cmathcal%7BR%7D%7D%28%5Cboldsymbol%7Bq%7D%29%20%3D%20%5Cbegin%7Bbmatrix%7D%20-q_1%20%26%20q_0%20%26%20-q_3%20%26%20q_2%5C%5C%20-q_2%20%26%20q_3%20%26%20q_0%20%26%20-q_1%5C%5C%20-q_3%20%26%20-q_2%20%26%20q_1%20%26%20q_0%20%5Cend%7Bbmatrix%7D)

Then we can calculate the rotation matrix between the body-fixed frame of the rigid body and the inertial frame at the Nth time step:

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7BR%7D_%7BIB%7D%5BN%5D%20%3D%20%5Cboldsymbol%7B%5Cmathcal%7BR%7D%7D%28%5Cboldsymbol%7Bq%7D%5BN%5D%29%20%5Cboldsymbol%7B%5Cmathcal%7BL%7D%7D%28%5Cboldsymbol%7Bq%7D%5BN%5D%29%20%5ET)

The angular velocity estimation is calculated by using the Lie algebra of the rotation matrix. The ![equation](https://latex.codecogs.com/gif.latex?%7B%7D%5E%7B%5Cvee%7D) map is denoted as the inverse of the Lie map. 

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5Comega%7D_%7BB%2Craw%7D%5BN%5D%20%5Capprox%20-%20%28%5Cboldsymbol%7BR%7D_%7BBI%7D%5BN%5D%20%5Cboldsymbol%7BR%7D_%7BIB%7D%5BN-1%5D/dt%29%5E%7B%5Cvee%7D)

After obtaining the raw velocity estimation, we use the following sliding window filter to provide a filtered result:

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7Bv%7D_%7BI%7D%5BN%5D%20%3D%20%5Csum%5E%7Bm-1%7D_%7Bi%20%3D%200%7D%20%5Cmu_j%20%5Cboldsymbol%7Bv%7D_%7BI%2Craw%7D%5BN-i%5D%3B%20%5Csum%5E%7Bm-1%7D_%7Bi%20%3D%200%7D%20%5Cmu_j%20%3D%201)

![equation](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5Comega%7D_%7BB%7D%5BN%5D%20%3D%20%5Csum%5E%7Bm-1%7D_%7Bi%20%3D%200%7D%20%5Cmu_j%20%5Cboldsymbol%7B%5Comega%7D_%7BB%2Craw%7D%5BN-i%5D%3B%20%5Csum%5E%7Bm-1%7D_%7Bi%20%3D%200%7D%20%5Cmu_j%20%3D%201)

The size of the window is set to 4 based on experimental testings.
