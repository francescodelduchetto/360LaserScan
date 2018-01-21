
import tf
import math
import rospy
import numpy as np

from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate


NUM_LASER_POINTS = 100

simulated_laser = [.0] * NUM_LASER_POINTS

GRID_WIDTH = None
GRID_HEIGHT = None
RESOLUTION = None
OFF_X = None
OFF_Y = None
robot_pose = None
local_costmap = None

def local_costmap_callback(msg):
    global GRID_WIDTH, GRID_HEIGHT, RESOLUTION, OFF_X, OFF_Y, local_costmap

    GRID_WIDTH = msg.info.width
    GRID_HEIGHT = msg.info.height
    RESOLUTION = msg.info.resolution
    OFF_X = int(math.floor(GRID_WIDTH/2))
    OFF_Y = int(math.floor(GRID_HEIGHT/2))
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

    scan_pub = rospy.Publisher("/360scan", LaserScan, queue_size=10)

    rate = rospy.Rate(10) # 10 hz

    seq_num = 0
    while not rospy.is_shutdown():

        if robot_pose is not None and local_costmap is not None:
            euler = tf.transformations.euler_from_quaternion((
                        robot_pose.orientation.x,
                        robot_pose.orientation.y,
                        robot_pose.orientation.z,
                        robot_pose.orientation.w,
                        ))

            robot_angle = euler[2]

            angle_step = math.pi * 2 / NUM_LASER_POINTS

            simulated_laser = [.0] * NUM_LASER_POINTS
            _angle_min = .0
            _angle_max = math.pi * 2
            for step in range(NUM_LASER_POINTS):
                angle = angle_step * step + robot_angle
                if step == 0:
                    _angle_min = angle - robot_angle
                elif step == NUM_LASER_POINTS - 1:
                    _angle_max = angle - robot_angle

                tan = math.tan(angle)

                for i in range(1, OFF_X):
                    if angle > math.pi/2 and angle < 3*math.pi/2:
                        i = -i

                    j = int(round(tan * i))
                    if abs(j) >= OFF_Y:
                        break

                    cell = local_costmap[OFF_Y + j][OFF_X + i]
                    if cell == 100:
                        obs_dist = math.sqrt(i**2 + j**2)
                        obs_dist *= RESOLUTION # cell units to m units
                        simulated_laser[step] = obs_dist
                        break

            # publish
            scan = LaserScan()
            scan.header.seq = seq_num
            scan.header.frame_id = "/base_laser_link"
            scan.header.stamp = rospy.Time.now()
            scan.angle_min = _angle_min
            scan.angle_max = _angle_max
            scan.range_min = .3
            scan.range_max = max(GRID_WIDTH, GRID_HEIGHT) / 2 * RESOLUTION
            scan.angle_increment = angle_step
            scan.time_increment = .0
            scan.ranges = simulated_laser
            scan_pub.publish(scan)

            seq_num += 1
            #print simulated_laser

        rate.sleep()

    rospy.spin()
