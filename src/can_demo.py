from geometry_msgs.msg import PoseStamped, Pose, Point
from actionlib_msgs.msg import GoalStatusArray
from rospy import Subscriber, Publisher
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelState, LinkStates
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

locked = False

def main():
    global pose_pub
    global status_sub
    global arm_pub
    global gripper_pub

    global attach_srv
    global detach_srv

    pose_pub = Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    arm_pub = Publisher("/ur_arm/moveit/goal_pose", Pose, queue_size=1)
    arm_status_sub = Subscriber("/move_group/status", GoalStatusArray, arm_status)
    gripper_pub = Publisher("/gripper_position_controller/command", Float64, queue_size=1)

    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

    # Publishers need some time to get started
    rospy.sleep(1)
    fold_arm()
    open_gripper()
    go_to_can()
    rospy.sleep(1)
    marker_sub = Subscriber("/ar_pose_marker", AlvarMarkers, marker_position)
    status_sub = Subscriber("move_base/status", GoalStatusArray, lock_control)
    rospy.spin()

def lock_control(data):
    global locked
    if not data.status_list or data.status_list[0].text == "Goal reached.":
        locked = False
    else:
        locked = True

#origin of Kinect sensor at x = .5, y = 0, z = .9 in arm coordinates
def kinect_to_arm(position):
    out = Point()
    out.x = .4
    out.z = .8
    out.z += position.y
    out.y -= position.x
    out.x += position.z
    return out

state = "empty"

def marker_position(data):
    global locked
    global state
    if not locked and data.markers and state == "empty":
        arm_to_can(data)
        rospy.sleep(3)
        close_gripper()
        rospy.sleep(2)
        fold_arm()
        drive_away()
        rospy.sleep(6)
        state = "picked_up"
    elif not locked and state == "picked_up":
        open_gripper()
        state = "dropped_off"
        rospy.sleep(1)
        fold_arm()
        go_to_can()


def arm_to_can(data):
    pose = Pose()
    pose.position = kinect_to_arm(data.markers[0].pose.pose.position)
    pose.orientation.y = .707
    pose.orientation.w = .707
    pose.position.x -= .1
    global arm_pub
    arm_pub.publish(pose)
    rospy.sleep(7)
    pose.position.x += .1
    arm_pub.publish(pose)

def go_to_can():
    global pose_pub
    pose = PoseStamped()
    pose.header.seq = 0
    pose.header.frame_id = "map"
    pose.pose.position.x = -2.9
    pose.pose.position.y = -2

    pose.pose.orientation.z = 1
    pose.pose.orientation.w = -0.00900788442947
    pose_pub.publish(pose)

def drive_away():
    global pose_pub
    pose = PoseStamped()
    pose.header.seq = 0
    pose.header.frame_id = "map"
    pose.pose.position.x = -.5
    pose.pose.position.y = 2

    pose.pose.orientation.z = .7
    pose.pose.orientation.w = .7
    pose_pub.publish(pose)

def fold_arm():
    global arm_pub
    pose = Pose()
    pose.position.x = .7
    pose.position.y = 0
    pose.position.z = 1.3
    pose.orientation.y = .707
    pose.orientation.w = .707
    arm_pub.publish(pose)
    rospy.sleep(5)

def open_gripper():
    global gripper_pub
    gripper_pub.publish(0)
    detach_cube()

def close_gripper():
    global gripper_pub
    gripper_pub.publish(1.5)
    attach_cube()

def detach_cube():
    global detach_srv

    req = AttachRequest()
    req.model_name_1 = "mir"
    req.link_name_1 = "right_inner_finger"
    req.model_name_2 = "tag_box"
    req.link_name_2 = "link"

    detach_srv.call(req)


def attach_cube():
    global attach_srv

    req = AttachRequest()
    req.model_name_1 = "mir"
    req.link_name_1 = "right_inner_finger"
    req.model_name_2 = "tag_box"
    req.link_name_2 = "link"

    attach_srv.call(req)

def arm_status(data):
    pass

if __name__ == "__main__":
    rospy.init_node("mir_nav")
    main()
