import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
import time
import numpy as np
import threading


position_lim_MAX = [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973]
position_lim_MIN = [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]

max_pos= np.array(position_lim_MAX)
min_pos= np.array(position_lim_MIN)

center_pos = (max_pos + min_pos)/2
range_of_motion = max_pos-min_pos


        

class JointTrajectoryPointPublisher(Node):
    def __init__(self):
        try:
            super().__init__('joint_trajectory_point_publisher')
            
            self.publisher_ = self.create_publisher(JointTrajectoryPoint, '/requested_traj_point', 10)
            self.timerPeriod = 0.001
            self.timer = self.create_timer(self.timerPeriod, self.timer_callback)
            
            self.motionPeriod = 20
            self.step = 0
            self.moving = True
            self.amplitude = 1.2
            self.waveType = "square";

            self.running = True                    
            self._start_input_thread()             

            # Give the publisher some time to set up
            time.sleep(1)
        except Exception as e:
            print(f" Error during JointTrajectoryPointPublisher init: {e}")
        

        
        

    def timer_callback(self):
        # Create a message
        point = JointTrajectoryPoint()
        
        period_cycles = self.motionPeriod/self.timerPeriod
        w_t = 2*np.pi /self.motionPeriod
        waveState = np.sin(2*np.pi *(self.step/period_cycles) )
        d_waveState =  w_t * np.cos(2*np.pi *(self.step/period_cycles) )
        dd_waveState = -w_t* w_t * np.sin(2*np.pi *(self.step/period_cycles) )
        
        if self.moving == True:
            if(self.step + 1 >=  period_cycles):
                self.step = 0
            else:
                self.step = self.step + 1

        if self.waveType == "sine":
            positions = (range_of_motion/2)*waveState*self.amplitude + center_pos
            velocities = (range_of_motion/2)*d_waveState*self.amplitude 
            accelerations = (range_of_motion/2)*dd_waveState*self.amplitude 
        elif self.waveType == "square":
            positions = (range_of_motion/2)*np.sign(waveState)*self.amplitude + center_pos
            velocities = np.zeros_like(range_of_motion)
            accelerations = np.zeros_like(range_of_motion)

        # Target positions for all 7 joints (in radians)
        # You can change these values!
        point.positions = positions.tolist()
        point.velocities = velocities.tolist()
        point.accelerations = accelerations.tolist()

        #self.get_logger().info('Sending joint command...')
        self.publisher_.publish(point)



    def _start_input_thread(self):
        thread = threading.Thread(target=self._input_loop, daemon=True)
        thread.start()

    def _input_loop(self):
        self._handle_command("help")
        self._handle_command("status")
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
            self.get_logger().info("help, status, stop, start, period <s>, ampl <1>, wave <sine/square> ")

        elif cmd == "status":
            self.get_logger().info(f" Motion period: {self.motionPeriod}, Motion amplitude: {self.amplitude}, Wavetype {self.waveType} , Moving: {self.moving}, Step: {self.step}, cmd freq: {1/self.timerPeriod} [Hz]")

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

        elif cmd.startswith("wave "):
            try:
                _, val = cmd.split()
                if(val.startswith("sine")):
                    self.waveType = "sine"
                elif(val.startswith("square")):
                    self.waveType = "square"
                else:
                    self.get_logger().warn(f"Invalid wavetype value: {val}")
                
                self.get_logger().info(f"Motion wavetype set to {self.waveType} ")
            except Exception as e:
                self.get_logger().warn(f"Invalid wavetype command: {e}")
        else:
            self.get_logger().warn(f"Unknown command: '{cmd}'")

    def stop(self):
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    print("Creating node...")
    node = JointTrajectoryPointPublisher()
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