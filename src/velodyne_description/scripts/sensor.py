#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, mavutil
import time
import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
import math
from math import radians, cos, sin, asin, sqrt

range_of_sensor = 15

vehicle = connect('127.0.0.1:14550', wait_ready=False)

my_position_x = 0
my_position_y = 0
my_position_z = 0


# sim_vehicle.py -v ArduCopter -f gazebo-iris --console

# run files
# roscore
# roslaunch iq_sim lidar.world
# python3 velopos.py
# sim_vehicle.py -v ArduCopter -f gazebo-iris --console
# python3 sensors.py 

# sensors.py - has target points where drone will go 
# the obstacle coordinates are found by centre coordinates of the lidar point cloud cluster

def pose_callback(msg):
    global iris_index, iris_state
    global my_position_x,my_position_y,my_position_z
    iris_index = msg.name.index("iris")
    iris_state = msg.pose[iris_index]
   # print(iris_state.position.x)
    my_position_x = iris_state.position.x
    my_position_y = iris_state.position.y
    my_position_z = iris_state.position.z

    

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)

def publish_point_cloud(points, pub):
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

    point_cloud_msg = pc2.create_cloud(header, fields, points)
    pub.publish(point_cloud_msg)

def calculate_centroid(points):
    centroid = np.mean(points, axis=0)
    return centroid

def apfa_run(centroid):
    x,y,z = centroid
    x = float(x)
    y = float(y)
    z = float(z)
    
    class APFA:
        def __init__(self, robot_pos, target_pos, obstacles,real_pos):
            self.robot_pos = np.array(robot_pos, dtype=float)
            self.target_pos = np.array(target_pos, dtype=float)
            self.obstacles = np.array(obstacles, dtype=float)
            self.real_pos = np.array(real_pos,dtype=float)
            
            self.k_att = 0.5
            self.k_rep_x = 5.0  
            self.k_rep_y = 100.0 
            self.k_rep_away = 100.0
            self.rep_radius = 15.0  

        def attraction_force(self):
       
            return self.k_att * (self.target_pos - self.robot_pos)

        def repulsion_force(self):
            rep_force = np.zeros_like(self.robot_pos)
            
            for obstacle in self.obstacles:
                if obstacle[0] == 99999999999999999999 and obstacle[1] ==  99999999999999999999:
                    return 0
                
                delta_x = self.robot_pos[0] - obstacle[0]
                delta_y = self.robot_pos[1] - obstacle[1]
                
                distance_major = abs(delta_x)
                distance_minor = abs(delta_y)
                
                if distance_major < self.rep_radius and distance_minor < self.rep_radius:
                    rep_force_x = 0
                    rep_force_y = 0
                    
                    if distance_major != 0:
                        rep_force_x += self.k_rep_x * (1 / distance_major - 1 / self.rep_radius) * (delta_x / distance_major**3)
                    if distance_minor != 0:
                        rep_force_y += self.k_rep_y * (1 / distance_minor - 1 / self.rep_radius) * (delta_y / distance_minor**3)
                        
                    rep_force += np.array([rep_force_x, rep_force_y])

            print("rep force: ", rep_force)
            return rep_force

        def total_force(self):
          #  return self.repulsion_force()
            return self.attraction_force() + self.repulsion_force()


        def update_position(self):
            total_force = self.total_force()
            print("total force: ",total_force*2)
            robot_x = self.real_pos[0]
            robot_y = self.real_pos[1]
            self.robot_pos += total_force
            
            vel = 2
            vel_x = vel*(total_force[0])/(((total_force[0]**2)+(total_force[1]**2))**0.5)
            vel_y = vel*(total_force[1])/(((total_force[0]**2)+(total_force[1]**2))**0.5)

            print(vel_x,vel_y)
            if np.isnan(vel_x) or np.isnan(vel_y):
                vel_x,vel_y=0,0
            send_ned_velocity(vel*vel_x,-1*vel*vel_y,0)
            

    # Example usage
    robot_position = [my_position_x, my_position_y]
    target_position = [50, 0]
    if np.isnan(x) or np.isnan(y):
        obstacles = [[99999999999999999999,99999999999999999999]]
        pass
    else:
        obstacles = [[x+my_position_x,y+my_position_y]]  # List of obstacle positions
    
    apfa = APFA(robot_position, target_position, obstacles,robot_position)

    # Update robot position using APFA algorithm
    
    apfa.update_position()

    print("Final robot position:", apfa.robot_pos)




def point_cloud_callback(msg, pub):
    point_cloud = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

    keep_ratio = 0.5
    num_points_to_keep = int(len(point_cloud) * keep_ratio)
    selected_indices = np.random.choice(len(point_cloud), num_points_to_keep, replace=False)
    downsampled_points = point_cloud[selected_indices]

    downsampled_points = downsampled_points[downsampled_points[:, 2] >= 0]
    distances = np.linalg.norm(downsampled_points[:, :3], axis=1)
    downsampled_points = downsampled_points[distances <= range_of_sensor]

    centroid = calculate_centroid(downsampled_points)
    print("Centroid of downsampled points:", centroid)
    

    apfa_run(centroid)

    downsampled_points_with_centroid = np.vstack([downsampled_points, centroid])
    
    publish_point_cloud(downsampled_points_with_centroid, pub)

if __name__ == "__main__":
    rospy.init_node('point_cloud_processing_node')
    iris_index = -1
    iris_state = Pose()

    # Subscribe to the model states topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, pose_callback)
    pub = rospy.Publisher('/downsampled_points', PointCloud2, queue_size=1)
    rospy.Subscriber('/velodyne_points', PointCloud2, point_cloud_callback, callback_args=pub)
    rospy.spin()
