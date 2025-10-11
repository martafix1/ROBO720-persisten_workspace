import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from tf_transformations import quaternion_from_euler
import time
import numpy as np
import threading


position_lim_MAX = [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973]
position_lim_MIN = [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]

max_pos= np.array(position_lim_MAX)
min_pos= np.array(position_lim_MIN)

center_pos = (max_pos + min_pos)/2
range_of_motion = max_pos-min_pos


        

class TaskSpaceObjectivePublisher(Node):
    def __init__(self):
        try:
            super().__init__('taskspace_objective')
            
            self.publisher_ = self.create_publisher(MultiDOFJointTrajectory, '/taskspace_objective', 10) # must be Trajectory not just point coz point is not top level message and will crash
            self.timerPeriod = 0.1
            self.timer = self.create_timer(self.timerPeriod, self.timer_callback)
            
            self.motionPeriod = 25
            self.step = 0
            self.moving = True
            self.amplitude = 0.6

            self.square_centre = [0,0,1]
            self.robot_base = [0,0,0]


            self.running = True                    
            self._start_input_thread()             

            # Give the publisher some time to set up
            time.sleep(1)
        except Exception as e:
            print(f" Error during TaskSpaceObjectivePublisher init: {e}")
        

        
        

    def timer_callback(self):
        # Create a message

        
        period_cycles = self.motionPeriod/self.timerPeriod

        waveState_ramp =  (self.step/period_cycles)

        
        if self.moving == True:
            if(self.step + 1 >=  period_cycles):
                self.step = 0
            else:
                self.step = self.step + 1

        # discretize ramp into 4 positions
        whichCorner = np.floor(waveState_ramp * 4) # 0,1,2,3 
        cornerX = 1 if (((whichCorner+1) // 2)  % 2) == 1 else -1
        cornerY = 1 if (whichCorner // 2) == 1 else -1
        
        activeCorner = [self.amplitude * (self.square_centre[0] + cornerX),
                        self.amplitude * (self.square_centre[1] + cornerY),
                        self.square_centre[2]]
        # activeCorner = amplitude * (square_centre[0:2] + [cornerX,cornerY])


        # make the end effector point away from base:

        v = np.array(activeCorner) - np.array(self.robot_base)
        # Normalize the vector
        norm = np.linalg.norm(v)
        if norm == 0:
            print("Base and corner are the same point. Crashing.. ")
            raise ValueError("Base and corner are the same point.")

        v = v / norm
        x, y, z = v

        # Yaw: rotation around Z axis (like turning head left/right)
        yaw = np.arctan2(y, x)

        # Pitch: rotation around Y axis (like looking up/down)
        pitch = np.arcsin(z)  # z is already sin(theta) because vector is normalized     
        roll = 0
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw) # the messages are sent as quaternions


        # calc the correct acc for fun
        acc_X = -cornerY * self.amplitude / self.motionPeriod # very freestyle, X-Y switch is for 180deg switch hh.
        acc_Y = -cornerX * self.amplitude / self.motionPeriod  

        point = MultiDOFJointTrajectoryPoint()

        # Position + Orientation as Transform
        transform = Transform()
        transform.translation.x = float(activeCorner[0])
        transform.translation.y = float(activeCorner[1])
        transform.translation.z = float(activeCorner[2])
        transform.rotation.x = qx
        transform.rotation.y = qy
        transform.rotation.z = qz
        transform.rotation.w = qw  # Identity quaternion

        # Velocity as Twist
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = 0.0

        # Acceleration as Twist
        acceleration = Twist()
        acceleration.linear.x = acc_X
        acceleration.linear.y = acc_Y
        acceleration.linear.z = 0.0
        acceleration.angular.x = 0.0
        acceleration.angular.y = 0.0
        acceleration.angular.z = 0.0

        point.transforms.append(transform)
        point.velocities.append(velocity)
        point.accelerations.append(acceleration)
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        msg = MultiDOFJointTrajectory()
        msg.joint_names = ['end_effector']  # Required field

        msg.points.append(point)
        self.publisher_.publish(msg)




    def _start_input_thread(self):
        thread = threading.Thread(target=self._input_loop, daemon=True)
        thread.start()

    def _input_loop(self):
        print("Input loop started. Inpud commands (help for help)")
        while self.running:
            try:
                print("\033]32m input command: \033]0m ")
                cmd = input().strip()
                self._handle_command(cmd)
            except EOFError:
                self.get_logger().warn("EOF reached — input loop exiting.")
                break
            except Exception as e:
                self.get_logger().error(f"Error reading input: {e}")

    def _handle_command(self, cmd: str):
        if cmd == "help":
            self.get_logger().info("help, status, stop, start, period <s>, ampl <1> ")

        elif cmd == "status":
            self.get_logger().info(f" Motion period: {self.motionPeriod}, Motion amplitude: {self.amplitude}, Moving: {self.moving}, Step: {self.step}, cmd freq: {1/self.timerPeriod} [Hz]")

        elif cmd == "stop":
            if self.moving == False:
                self.get_logger().info("Already stopped")
            else:
                self.get_logger().info("Stopping")
                self.moving = False
            
        elif cmd == "start":
            if self.moving == True:
                self.get_logger().info("Already moving")
            else:
                self.get_logger().info("Starting")
                self.moving = True

        elif cmd.startswith("period"):
            try:
                _, val = cmd.split()
                new_period = float(val)
                self.motionPeriod = new_period
                self.get_logger().info(f"Motion period set to {new_period} seconds.")
            except Exception as e:
                self.get_logger().warn(f"Invalid period command: {e}")

        elif cmd.startswith("ampl "):
            try:
                _, val = cmd.split()
                new_amplitude = float(val)
                self.amplitude = new_amplitude
                self.get_logger().info(f"Motion amplitude set to {new_amplitude}.")
            except Exception as e:
                self.get_logger().warn(f"Invalid period command: {e}")
        
        else:
            self.get_logger().warn(f"Unknown command: '{cmd}'")

    def stop(self):
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    print("Creating node...")
    node = TaskSpaceObjectivePublisher()
    print("Node created, spinning...")
    try:
        print("Spinnup")
        rclpy.spin(node)
        print("Spinning")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted with Ctrl+C.")
    finally:
        print("Stopping")
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("Done")
    print("Well Done")


# if __name__ == '__main__':
#     main()