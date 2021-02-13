import rospy
import actionlib

from arl_nav_msgs.msg import GotoRegionAction, GotoRegionGoal

if __name__ == '__main__':
    rospy.init_node('goto_region_client')
    print("Waiting for server")
    client = actionlib.SimpleActionClient('warty/goto_region', GotoRegionAction)
    client.wait_for_server()
    print("Found server.")

    policy = open("traj_uPOMDP-uncertain-1.txt", 'r')

    for index, line in enumerate(policy):
        if index < 5 or index%10!=0:
            continue
        #if index > 1:
        #    continue
        line_split = line.split(" ")
        x = int(line_split[0].strip('(, '))
        y = int(line_split[1].strip(')\n'))
        next_x = -198.75 + x*2.5 + 21 - 310 + 8
        next_y = -473.75 + y*2.5 - 23 + 53 + 1
        print(next_x)
        print(next_y)

        goal_msg = GotoRegionGoal()
        goal_msg.region_center.header.stamp = rospy.Time.now()
        goal_msg.region_center.pose.position.x = -next_y
        goal_msg.region_center.pose.position.y = next_x
        goal_msg.region_center.pose.orientation.w = 1
        goal_msg.radius = 1.8
        goal_msg.angle_threshold = 1.47
        #goal_msg.region_center.header.frame_id = "world"
        goal_msg.region_center.header.frame_id = "warty/map"
        client.send_goal(goal_msg)
        #client.wait_for_result(rospy.Duration.from_sec(5.0))
        client.wait_for_result()
        result = client.get_result()
        print(result)
