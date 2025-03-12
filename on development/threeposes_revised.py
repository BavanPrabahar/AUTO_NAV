import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf_transformations 
from nav2_simple_commander.robot_navigator import BasicNavigator


class Hiyaah(Node):
    def __init__(self):
        super().__init__('ga')
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.u, 1)
        
    
    def tf_conv(hello,b,c,d):
        x,y,z,w=tf_transformations.quaternion_from_euler(0.0,0.0,d)
        i=PoseStamped()
        i.header.frame_id="map"
        i.header.stamp=hello.get_clock().now().to_msg()
        i.pose.position.x=b
        i.pose.position.y=c
        i.pose.position.z=0.0
        i.pose.orientation.x=x
        i.pose.orientation.y=y
        i.pose.orientation.z=z
        i.pose.orientation.w=w
        return i

    def ab(self):    
        try:

            rclpy.init(args=args)
            
            hell=BasicNavigator()
        
            i=PoseStamped()

            i=self.tf_conv(hell,0.0,0.0,0.0)

            
            hell.setInitialPose(i)

            

            hell.waitUntilNav2Active()
            
            pose_1=pose_2=pose_3=PoseStamped()

            
            pose_1=self.tf_conv(hell,2.0,2.0,0.0)
            pose_2=self.tf_conv(hell,-2.0,2.0,0.0)
            pose_3=self.tf_conv(hell,-2.0,-2.0,0.0)
            poses=[pose_1,pose_2,pose_3]
            for a in poses:
                if ()
                hell.goToPose()
                # hell.followWaypoints(poses)

                # while not hell.isTaskComplete():
                #     feedback = hell.getFeedback()
                #     print(feedback)
                #     print("hello")
            print(hell.getResult())
        finally:
            rclpy.shutdown()
            

def main(args=None):
    rclpy.init(args=args)
    node=Hiyaah()
    try:

        rclpy.spin(node)
    finally:
        rclpy.shutdown()
    



if __name__=="__main__":
    main()