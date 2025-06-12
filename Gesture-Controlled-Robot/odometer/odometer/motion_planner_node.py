import rclpy
from rclpy.node import Node
import time
import math
from rcl_interfaces.msg import SetParametersResult
from custom_msgs.msg import Input

class MotionPlanner(Node):
    def __init__(self, name = 'MotionPlanner'):
        
        super().__init__(name)
        print("My name is:", name)
        self.get_logger().info("get looger works!!")
        #time.sleep(1.0) #for connection to be established, safety reasons
        
        
        self.pub = self.create_publisher(Input, "/cmd_vel", 10)
        
        #mode is the name of the param
        self.declare_parameter('mode', 'idle')
        #possible values of param
        self.allowed_modes = ['square', 'circle']

        #grabs the param when initialized with one
        self.intitial_mode = self.get_parameter('mode').get_parameter_value().string_value
        self.get_logger().info(f"Startup mode: {self.intitial_mode}")
        self.execute_mode(self.intitial_mode)

        #it is called when a param is set.During runtime
        self.add_on_set_parameters_callback(self.param_callback)

    #when called, iterate through allowed_modes to figure out 
    #the value the param was set with  
    def param_callback(self, params):
        for param in params:
            if param.name == 'mode':
                for mode in self.allowed_modes:
                    #once figured out, call the corresponding function
                    if mode == param.value:
                        self.execute_mode(mode)
                        return SetParametersResult(successful = True)
                return SetParametersResult(successful = False)    

    def execute_mode(self, mode):
        if mode == 'square':
            self.MakeSquare()
        elif mode == 'circle':
            self.MakeArc()
        elif mode == 'idle':
            self.get_logger().info("Idle!!")
        else:
            self.get_logger().info("Try again!!")

    def MakeSquare(self):
        
        for i in range(4):
            #1.Go straight
            msg_straight = Input()
            msg_straight.linear_vel = 0.5
            msg_straight.angular_vel = 0.0
            msg_straight.duration = 2.0
            self.pub.publish(msg_straight)
            #time.sleep(2.0)    

            # 2.Turn 90 degrees
            msg_turn = Input()
            msg_turn.linear_vel = 0.0
            msg_turn.angular_vel = math.pi
            msg_turn.duration = 0.5
            self.pub.publish(msg_turn)
            #time.sleep(0.5)
    
    def MakeArc(self):

        #R = u/w
        #angle = w*t
        #v = angle*R/t and w = angle / t
        msg_arc = Input()
        msg_arc.linear_vel = math.pi
        msg_arc.angular_vel = math.pi
        msg_arc.duration = 2.0
        self.pub.publish(msg_arc)




def main(args = None):
    rclpy.init(args=args)

    motion_planner = MotionPlanner()

    rclpy.spin(motion_planner)

    motion_planner.destroy_node()

    rclpy.shutdown()
