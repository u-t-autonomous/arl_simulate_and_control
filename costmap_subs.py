# This is the file to subscribe and save the costmap constructed by the simulator.

import rospy
from nav_msgs.msg import OccupancyGrid
import time
import numpy as np

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.info)
    global binary_map
    columns = data.info.width
    rows = data.info.height
    image = np.asarray(data.data)
    image = np.reshape(image,(rows,columns))
    print(np.amax(image))
    np.save('costmap_0314_3.npy',image)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/warty/ioc_costmap", OccupancyGrid, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
