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
    np.save('road.npy',image)
    #if data:
    #    binary_map = data.data
    #print(len(binary_map))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/warty/visual_cache/road", OccupancyGrid, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


