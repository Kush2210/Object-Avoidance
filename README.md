This a object avoidance of UAVs using APFA algorithm. <br>
Using velodyne VLP-16.

Steps:
#
source devel/setup.bash
catkin build
#
run files
 roscore
 roslaunch iq_sim lidar.world
 python3 velopos.py
 sim_vehicle.py -v ArduCopter -f gazebo-iris --console
 python3 sensors.py 

Main project files are in folder src/velodyne-description/scripts

About:
Point Cloud from velodyne is taken 
(It will work on only 1 obstacle)
Then it the centroid for all the points is found
The centroid is treated as a positive charge
and then apfa algo is applied
the force field is to be assumed as a elliptical orbits

ROS:
The iq_sim package is used for the uav simulation 
it is connected with sitl ardupilot
Lidar : Lidar is used from velodyne package
It is mounted on a drone using a script,i did not edit the drone urdf , i am just changing the position of lidar (which is a seperate object) each frame, the new position is equal to the position of drone, and just a bit higher than that.

APFA works successfuly - APFA code is in sensors.py file
