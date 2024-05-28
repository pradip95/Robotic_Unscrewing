#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt
from spiral_intepolation import SpiralInterpolation
from agiprobot_msgs.srv import tcp, tcpResponse
from agiprobot_msgs.srv import unscrew
import time


class ExecuteInterpolation:
    def __init__(self):
        pass

    def fetch_pose(self):
       
        rospy.wait_for_service('/fetchTCPPose')
        service_fetching_pose = rospy.ServiceProxy('/fetchTCPPose', tcp)
        
        try:
            service_response = service_fetching_pose()
            pose_data = service_response.pose
            print("Fetched Pose:", pose_data)
            return pose_data
            
        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return None

    def feed_pose(self,pose):

        rospy.wait_for_service('/pushTCPPose')
        pushing_pose = rospy.ServiceProxy('/pushTCPPose', tcp)

        try: 
            response = pushing_pose(pose)
            print("Pushed Pose:", pose)
            return response
        
        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return None
        
    def screwtool_rotate(self, screw_type, loops):

        rospy.wait_for_service('/unscrew')
        rotating = rospy.ServiceProxy('/unscrew', unscrew)

        try: 
            rotate_response = rotating(screw_type, loops)
            print("Rotating Tool")
            return None
        
        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return None

        
if __name__ == "__main__":
    start_time = time.time()
    rospy.init_node('move_to_point_interpolateSpiral')

    executor = ExecuteInterpolation()
    current_pose = executor.fetch_pose()

    interpolator = SpiralInterpolation()

    angle_increment = 0.001
    trajectory_distance = 0.0015
    radius_search = 0.004

    points = interpolator.interpolate_spiral(angle_increment, trajectory_distance, radius_search, current_pose)
    print(points.shape)
    
    plt.plot([pose[0] for pose in points], [pose[1] for pose in points], 'go--')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Archimedean Spiral with equidistant points')
    plt.axis('equal')
    plt.grid()
    #plt.show()
    #push_responses = [executor.feed_pose(point) for point in points]
   
    for i in range(points.shape[0]):
        #executor.screwtool_rotate(screw_type='', loops=1)
        executor.screwtool_rotate(screw_type='', loops=1)
        push_response = executor.feed_pose(points[i])
      
        #rospy.Duration(2)
        #new_point = points[i].copy()  # Create a copy of the current pose
        #new_point[2] += 0.01  # Increment the z-coordinate by 0.01
        #executor.feed_pose(new_point)
        
    print("Service call completed.")
    rospy.signal_shutdown("Execution finished.")
    end_time = time.time()
    execution_time = end_time - start_time
    print('execution time: ', execution_time)
