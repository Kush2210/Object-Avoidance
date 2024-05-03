#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose

def pose_callback(msg):
    global iris_index, iris_state
    iris_index = msg.name.index("iris")
    iris_state = msg.pose[iris_index]

def set_model_state(model_name, pose):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state_msg = ModelState()
        state_msg.model_name = model_name
        state_msg.pose = pose
        response = set_model_state_proxy(state_msg)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed:", e)
        return False

if __name__ == "__main__":
    rospy.init_node('set_model_position')
    
    iris_index = -1
    iris_state = Pose()

    # Subscribe to the model states topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, pose_callback)

    # Loop to continuously update model state based on model states
    while not rospy.is_shutdown():
        success = set_model_state("example", iris_state)
        if success:
          #  print("Model state set successfully")
          pass
        else:
            print("Failed to set model state")
        rospy.sleep(0.001)  # Adjust this delay as needed
