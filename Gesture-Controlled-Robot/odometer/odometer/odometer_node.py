import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
# from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
import math
import time

from custom_msgs.msg import Input
from custom_msgs.msg import Coordinates
from custom_msgs.msg import WheelSpeed

def euler2quaterion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = {}
    q['w'] = cr * cp * cy + sr * sp * sy
    q['x'] = sr * cp * cy - cr * sp * sy
    q['y'] = cr * sp * cy + sr * cp * sy
    q['z'] = cr * cp * sy - sr * sp * cy

    return q
    
class OdometerNode(Node):
    def __init__(self, name = 'OdometerNode'):
        super().__init__(name)
        print("My name is", name)

        #Robot's Variables
        self.freq = 10
        self.l = 2  #robot's width
        self.r = 0.3 #wheel's radius
        
        #Robot State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0 # rad or degrees
        
        self.get_state()

        #Input
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.duration = 0.0

        #Subscription to cmd_vel
        self.subscription = self.create_subscription(
            Input,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        #Publisher for Odom, Wheel Speed
        self.odom_pub = self.create_publisher(Coordinates, '/odom', 10)
        self.wheel_speed_pub = self.create_publisher(WheelSpeed, 'wheel_speed', 10)

        # #TF BroadCaster
        self.tf_broadcaster = TransformBroadcaster(self)

    @property#@propery means it is another variable of self
    def state(self):
        return [self.x, self.y, self.theta]
    
    def get_state(self):
        print("Current state:", self.state) 

    def cmd_vel_callback(self, msg): # since it is inside the self.subscription, it will be called every time we receive a message

        self.linear_vel = msg.linear_vel
        self.angular_vel = msg.angular_vel
        self.duration = msg.duration

        #Publishes wheel speed.
        #It remains stable, for the duration of the Input, that is why it is published at the start.
        #We also publish it at the end of input's duration, where it becomes zero
        wheel_speed_msg = WheelSpeed()
        wheel_speed_msg.front_left_angular, wheel_speed_msg.front_right_angular = self.get_wheel_speed()
        self.wheel_speed_pub.publish(wheel_speed_msg)

        #we make continuous time discrete by splitting input's duration into discrete steps
        #iterations = 10 #testing purpose
        iterations = int(self.duration * self.freq)
        iteration_length = 1 / self.freq

        print("iterations =", iterations)

        for i in range(iterations):
            #calculate new position based on previous pos and input
            self.x = iteration_length * self.linear_vel * math.cos(self.theta) + self.x
            self.y = iteration_length * self.linear_vel * math.sin(self.theta) + self.y
            self.theta = iteration_length * self.angular_vel + self.theta
           
            # self.get_state()

            #Publishes current position od the robot
            coordinates_msg = Coordinates()
            coordinates_msg.x = self.x
            coordinates_msg.y = self.y
            coordinates_msg.theta = self.theta
            self.odom_pub.publish(coordinates_msg)

            #Handles the TF BroadCaster(make it a function for cleaner code?)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = '/odom'
            t.child_frame_id = '/base_link'
            
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            q = euler2quaterion(0, 0, self.theta)
            t.transform.rotation.x = q['x']
            t.transform.rotation.y = q['y']
            t.transform.rotation.z = q['z']
            t.transform.rotation.w = q['w']

            self.tf_broadcaster.sendTransform(t)

            time.sleep(iteration_length)

    def get_wheel_speed(self):  
        front_left_linear = self.linear_vel - self.angular_vel* (self.l/2)
        front_left_angular = front_left_linear / self.r

        front_right_linear = self.linear_vel + self.angular_vel* (self.l/2)
        front_right_angular = front_right_linear / self.r

        return front_left_angular, front_right_angular

def main(args = None):
    rclpy.init(args=args)       

    odometer = OdometerNode()

    rclpy.spin(odometer)

    odometer.destroy_node()

    rclpy.shutdown()


'''
logic flow
    we are given: current position, linear and angular velocity, one at a time, for now, how long we apply the velocity
    
    we calculate/publish out robot's state:
      1)at the end of the movement
    ->2)"live", meaning every seondd/freq (live is still discrete, in robotics/ programming true live cannot exist)

    Q: what happens if we receive a command while another one is still on going?
        1)put it into a queue(bee game)
      ->2)stop and continue with the new one(tutrtle, demands live msg outputing, not at the end of the command)

    Do not forget, we have freqency of publishing in there.ATTENTION! 10 on the functions is not frequency, is QoS profile(queue depth?)
    so, suppose we are at state x = 0, theta = 0 and receive the command: linear.x = 5, for 1 second.
    frequency = times per second = 10. so we need to upload 10 msgs.
    2 cases occur.
    1)duration of movement is divisible by 0.1.
        so, duraration / 0.1 gives as the #of points = n
        for i in range (1, n)
            self.x = 0.1*self.linear_vel*cos(theta) + self.x
            self.y = 0.1*self.linear_vel*sin(theta) + self.y
            self.theta = 0.1*self.self.angular_vel + self.theta (% 2*pi, or 360)
            publish state
            nullify input. why? if not, if we do not receive another command, it will assume like we received the same!

    2)it is not
        only the last iteration will change(some modulo type shii)

    Q: how do I interupt at the mioddle of the loop? signal interupts and i have to add it or does it happen automatically?
       how do i implement the other case?
'''