#!/usr/bin/python3

import rospy

from ros_tcp_endpoint import TcpServer, RosService, RosPublisher, RosSubscriber
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Image
from cozmo_demo_msgs.msg import  HeadRot, LiftRot, TurnInPlace, DriveStraight, DrivePath

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", "TCPServer")
    tcp_server = TcpServer(ros_node_name)

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start({
        "head_rot": RosSubscriber("head_rot", HeadRot, tcp_server),
        "lift_rot": RosSubscriber("lift_rot", LiftRot, tcp_server),
        "base_controller/cmd_vel":RosSubscriber("base_controller/cmd_vel", Twist, tcp_server),
        "drive_straight": RosSubscriber("drive_straight", DriveStraight, tcp_server),
        "turn_in_place": RosSubscriber("turn_in_place", TurnInPlace, tcp_server),
        "drive_path": RosSubscriber("drive_path", DrivePath, tcp_server),

    })
    rospy.spin()


if __name__ == "__main__":
    main()
