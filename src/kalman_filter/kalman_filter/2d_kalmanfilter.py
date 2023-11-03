import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy.linalg import inv
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        # Initialize kalman variables
        self.t=1/20
        self.dx=0
        self.dy=0

        #state matrix
        self.F=np.array([[1,0],
                         [0,1]])
        
        #state vector
        self.X=np.array([[0],
                         [0]])
        

        #state transition matrix
        self.G=np.array([[1,0],
                         [0,1]])
        

        #control input
        self.U=np.array([[0],
                         [0]])


        #measurement matrix
        self.H=np.array([[1,0],
                         [0,1]])



        #estimate covariance
        self.P=np.array([[1,0],
                         [0,1]])

    
        #measurment covariance
        self.R=np.array([[0.1,0],
                         [0,0.1]])
        

        #process covariance
        self.Q=np.array([[0.01,0],
                         [0,0.01]])
        

        
        

        
        # Subscribe to the /odom_noise topic
        self.subscription = self.create_subscription(Odometry,
                                                     '/odom_noise',
                                                     self.odom_callback,1)
        
        #publish the estimated reading
        self.estimated_pub=self.create_publisher(Odometry,
                                                 "/odom_estimated",1)
        
        #change Q message
        self.Q_sub=self.create_subscription(Float64MultiArray,
                                         "/Q",self.Q_callback,1)
        

    def Q_callback(self,msg):
        self.Q=np.array([[msg.data[0],0],
                         [0,msg.data[0]]])
        
        self.R=np.array([[msg.data[1],0],
                            [0,msg.data[1]]])

    def odom_callback(self, msg):
        # Extract the position measurements from the Odometry message
        print(self.P)

        pos=np.array([[msg.pose.pose.position.x],
                      [msg.pose.pose.position.y]])
        
        vel=msg.twist.twist.linear.x
        
        roll,pitch,yaw=euler_from_quaternion([msg.pose.pose.orientation.x,
                                              msg.pose.pose.orientation.y,
                                              msg.pose.pose.orientation.z,
                                              msg.pose.pose.orientation.w])
        

        #self.get_logger().info('Publishing: "%s"' % (yaw*180/np.pi))

        self.dx=vel*self.t*np.cos(yaw)
        self.dy=vel*self.t*np.sin(yaw)

        self.U=np.array([[self.dx],
                         [self.dy]])
        
        # Prediction step
        
        self.X=self.F@self.X+self.G@self.U

        self.P=self.F@self.P@self.F.T + self.Q


        # Update step

        self.K=self.P@self.H.T@inv(self.H@self.P@self.H.T+self.R)

        self.X=self.X+self.K@(pos-self.H@self.X)

        self.P=(np.eye(2)-self.K@self.H)@self.P@(np.eye(2)-self.K@self.H).T+self.K@self.R@self.K.T

        

        #publish the estimated reading
        odom_estimated=Odometry()
        odom_estimated.pose.pose.position.x=float(self.X[0])
        odom_estimated.pose.pose.position.y=float(self.X[1])
        self.estimated_pub.publish(odom_estimated)

        #distance between estimated and actual
        #self.get_logger().info('Distance: "%s"' % (np.sqrt((self.X[0]-pos[0])**2+(self.X[1]-pos[1])**2)))


        

def main(args=None):
    try:
        rclpy.init(args=args)
        node = KalmanFilter()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

