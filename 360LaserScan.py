
import tf
import rospy
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate


NUM_LASER_POINTS = 30

simulated_laser = [.0] * NUM_LASER_POINTS

GRID_WIDTH = None
GRID_HEIGHT = None
RESOLUTION = None
local_costmap = None
robot_pose = None

def local_costmap_callback(msg):
    global GRID_WIDTH, GRID_HEIGHT, RESOLUTION, local_costmap

    GRID_WIDTH = msg.info.width
    GRID_HEIGHT = msg.info.height
    RESOLUTION = msg.info.resolution
    # to matrix shape
    local_costmap = np.reshape(msg.data, (GRID_HEIGHT, GRID_WIDTH))

def local_costmap_update_callback(msg):
    global local_costmap

    if local_costmap is not None:
        local_costmap[msg.y:msg.height+msg.y, msg.x:msg.width+msg.x] = \
                    np.reshape(msg.data, (msg.height, msg.width))

def pose_callback(msg):
    global robot_pose

    robot_pose = msg


if __name__ == "__main__":
    # NOTE: assume that the robot is already at the center of the costmap

    rospy.init_node("360LaserScanProvider")

    rospy.Subscriber("/move_base/local_costmap/costmap",
                OccupancyGrid,
                local_costmap_callback)

    rospy.Subscriber("/move_base/local_costmap/costmap_updates",
                OccupancyGridUpdate,
                local_costmap_update_callback)

    rospy.Subscriber("/robot_pose", Pose, pose_callback)

    rate = rospy.Rate(10) # 10 hz

    while not rospy.is_shutdown():

        if robot_pose is not None:
            euler = tf.transformations.euler_from_quaternion((
                        robot_pose.orientation.x,
                        robot_pose.orientation.y,
                        robot_pose.orientation.z,
                        robot_pose.orientation.w,
                        ))

            # we need euler[2]


        rate.sleep()

    rospy.spin()
