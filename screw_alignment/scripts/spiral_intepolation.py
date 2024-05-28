'''
author: Pradip Mehta
date: 14.08.2023
'''


import numpy as np
import math
from matplotlib import pyplot as plt


class SpiralInterpolation: 
    def __init__(self):
        pass  
    
    def interpolate_spiral(self, angle_increment, trajectory_distance, radius_search, pose):
        phi1 = 0
        phi2 = 0
        a = trajectory_distance / (2*math.pi)
        pose = np.array(pose)
        poses = []
        while phi2 <= 360:
            phi = phi2 - phi1
            distance_v = math.sqrt(pow((math.tan(phi) * math.cos(phi) * phi1 * a), 2) + pow((a * (phi2 - phi1 * math.cos(phi))), 2))
            if distance_v > trajectory_distance:   
                x1 = a * phi1 * math.cos(phi1)
                y1 = a * phi1 * math.sin(phi1)
                x2 = a * phi2 * math.cos(phi2)
                y2 = a * phi2 * math.sin(phi2)
                x = x2 - x1
                y = y2 - y1
             
                intermediate_pose = pose.copy()
                poses.append(intermediate_pose)

                pose[0] += x
                pose[1] += y

                phi1 = phi2
              
                if radius_search < np.abs(x2) or radius_search < np.abs(y2):
                    break

            phi2 += angle_increment
       
        poses_array = np.array(poses)
        return poses_array


'''
angle_increment = np.deg2rad(0.1)
print(angle_increment)
trajectory_distance = 0.0005
radius_search = 0.002
start_pose = [2, 3.40, 2, 1, 3, 1, 1]

interpolator = SpiralInterpolation()
points = interpolator.interpolate_spiral(angle_increment, trajectory_distance, radius_search, start_pose)

print(points)
print(angle_increment)
print(points.shape)
plt.plot([pose[0] for pose in points], [pose[1] for pose in points], 'go--')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Archimedean Spiral with equidistant points')
plt.axis('equal')
plt.grid()
plt.show()
'''
