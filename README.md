This a object avoidance of UAVs using APFA algorithm. <br>
Using velodyne VLP-16. <br>

Steps:
#
source devel/setup.bash<br>
catkin build<br>
#
run files<br>
 roscore<br>
 roslaunch iq_sim lidar.world<br>
 python3 velopos.py<br>
 sim_vehicle.py -v ArduCopter -f gazebo-iris --console<br>
 python3 sensors.py <br>

Main project files are in folder src/velodyne-description/scripts<br>

About:<br>
Point Cloud from velodyne is taken <br>
(It will work on only 1 obstacle)<br>
Then it the centroid for all the points is found<br>
The centroid is treated as a positive charge<br>
and then apfa algo is applied<br>
the force field is to be assumed as a elliptical orbits<br>

ROS:<br>
The iq_sim package is used for the uav simulation <br>
it is connected with sitl ardupilot<br>
Lidar : Lidar is used from velodyne package<br>
It is mounted on a drone using a script,i did not edit the drone urdf , i am just changing the position of lidar (which is a seperate object) each frame, the new position is equal to the position of drone, and just a bit higher than that.

APFA works successfuly - APFA code is in sensors.py file<br>
