
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import time
class Hello(Node):
    def __init__(self):
        super().__init__("annamalai")
        self.a=[0.0,0.0,0.0,0.0,10.0,0.0,0.0,0.0,20.0,0.0,0.0,0.0]
        self.pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.hiyaah,1)

        self.pub1=self.create_publisher(PoseStamped,'goalpose',1)
        #self.create_timer(1,self.hi)
        self.msgi=PoseWithCovarianceStamped()
        self.msgf=PoseStamped()
        self.amcl=PoseWithCovarianceStamped()
        self.initialpose(0.0,0.0,0.0,1.0)

        self.hi()

    def hiyaah(self,msg):
        self.amcl=msg

    def initialpose(self,a,b,c,d):

        self.msgi.pose.pose.position.x=a
        self.msgi.pose.pose.position.y=b
        self.msgi.pose.pose.orientation.z=c
        self.msgi.pose.pose.orientation.w=d
        self.msgi.pose.covariance=[
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0685389
]

        self.pub.publish(self.msgi)
    def goalpose(self,a,b,c,d):
        self.msgf.pose.position.x=a
        self.msgf.pose.position.y=b
        self.msgf.pose.orientation.z=c
        self.msgf.pose.orientation.w=d
        self.pub1.publish(self.msgf)


    def hi(self):
        j=len(self.a)/4
        i=0
        k=0
        while(j-1>0):
            if k>60:
                self.initialpose(self.amcl.pose.pose.position.x,self.amcl.pose.pose.position.y,self.amcl.pose.pose.orientation.z,self.amcl.pose.pose.orientation.w)
            if self.a[i]-0.1<=self.amcl.pose.pose.position.x<=self.a[i]+0.1 and  self.a[i+1]-0.1<=self.amcl.pose.pose.position.y<=self.a[i+1]+0.1:
                self.initialpose(self.a[i],self.a[i+1],self.a[i+2],self.a[i+3])
                self.goalpose(self.a[i+4],self.a[i+5],self.a[i+6],self.a[i+7])
                i += 2
                k=0
            else:
                time.sleep(1)
                k+=1




        
def main(args=None):
    rclpy.init(args=args)
    code=Hello()
    try:
    	rclpy.spin(code)
    finally:
    	rclpy.shutdown()
if __name__=="__main__":
    main() 
    