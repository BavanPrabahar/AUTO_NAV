import rclpy
from geometry_msgs.msg import PoseStamped
import tf_transformations 
from nav2_simple_commander.robot_navigator import BasicNavigator
def tf_conv(hell:BasicNavigator,b,c,d):
    x,y,z,w=tf_transformations.euler_from_quaternion(0.0,0.0,d)
    i=PoseStamped()
    i.header.frame_id="map"
    i.header.stamp=hell.get_clock().now().to_msg()
    i.pose.position.x=b
    i.pose.position.y=c
    i.pose.position.z=0.0
    i.pose.orientation.x=x
    i.pose.orientation.y=y
    i.pose.orientation.z=z
    i.pose.orientation.w=w
    return i

def main(args=None):
    rclpy.init(args=args)
    hell=BasicNavigator()
    x,y,z,w=tf_transformations.euler_from_quaternion(0.0,0.0,0.0)
    i=PoseStamped()
    i.header.frame_id="map"
    i.header.stamp=hell.get_clock().now().to_msg()
    i.pose.position.x=0.0
    i.pose.position.y=0.0
    i.pose.position.z=0.0
    i.pose.orientation.x=x
    i.pose.orientation.y=y
    i.pose.orientation.z=z
    i.pose.orientation.w=w
    hell.setInitialPose(i)

    

    hell.waitUntilNav2Active()
    pose_1=pose_2=pose_3=PoseStamped()
    
    
    pose_1=tf_conv(hell,2.0,2.0,0.0)
    pose_2=tf_conv(hell,-2.0,2.0,0.0)
    pose_3=tf_conv(hell,-2.0,-2.0,0.0)
    poses=[pose_1,pose_2,pose_3]
    hell.followWaypoints(poses)

    while not hell.isTaskComplete():
        feedback = hell.getFeedback()
        print(feedback)





if __name__=="__main__":
    main()