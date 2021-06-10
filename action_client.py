import rospy
import actionlib

from arl_nav_msgs.msg import GotoRegionAction, GotoRegionGoal

if __name__ == '__main__':
    #Some initializations depending on map
    grid_size = 2.5
    fg_x_offset = -177.75       #Offset values for floodedground environment
    fg_y_offset = -496.75
    init_x_offset = -300         #Offset values for simulation initialization
    init_y_offset = 0
    #Initialize client node and connect it to the action server
    rospy.init_node('goto_region_client')
    print("Waiting for server")
    client = actionlib.SimpleActionClient('warty/goto_region', GotoRegionAction)
    client.wait_for_server()
    print("Found server.")

    #Read the input trajectory text file
    policy = open("traj_uPOMDP-uncertain-1.txt", 'r')

    #Extract x-y coordinates for the waypoints line by line
    for index, line in enumerate(policy):
        line_split = line.split(" ")
        x = int(line_split[0].strip('(, '))
        y = int(line_split[1].strip(')\n'))
        next_x = x*grid_size + fg_x_offset + init_x_offset
        next_y = y*grid_size + fg_y_offset + init_y_offset

        #Setting up the goal message
        goal_msg = GotoRegionGoal()
        goal_msg.region_center.header.stamp = rospy.Time.now()
        goal_msg.region_center.pose.position.x = -next_y
        goal_msg.region_center.pose.position.y = next_x
        goal_msg.region_center.pose.orientation.w = 1
        goal_msg.radius = 1.8
        goal_msg.angle_threshold = 1.47
        goal_msg.region_center.header.frame_id = "warty/map"
        
        #Sending the message to the server
        client.send_goal(goal_msg)
        
        #Check for result
        client.wait_for_result()
        result = client.get_result()
        print(result)
