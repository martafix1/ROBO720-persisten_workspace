import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import numpy as np
import threading


position_lim_MAX = [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973]
position_lim_MIN = [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]

max_pos= np.array(position_lim_MAX)
min_pos= np.array(position_lim_MIN)

center_pos = (max_pos + min_pos)/2
range_of_motion = max_pos-min_pos


class FrankaMover(Node):
    def __init__(self):
        try:
            super().__init__('franka_mover')
            self.publisher_ = self.create_publisher(
                Float64MultiArray,
                '/requested_angles_CMD_INTERFACE',
                10
            )
            self.timerPeriod = 0.05
            self.timer = self.create_timer(self.timerPeriod, self.timer_callback)
            
            self.motionPeriod = 5
            self.step = 0
            self.moving = True
            self.amplitude = 0.8

            self.running = True                    
            self._start_input_thread()             

            # Give the publisher some time to set up
            time.sleep(1)
        except Exception as e:
            print(f" Error during FrankaMover init: {e}")
        

        
        

    def timer_callback(self):
        # Create a message
        msg = Float64MultiArray()
        
        period_cycles = self.motionPeriod/self.timerPeriod
        waveState = np.sin(2*np.pi *(self.step/period_cycles) )
        
        if self.moving == True:
            if(self.step + 1 >=  period_cycles):
                self.step = 0
            else:
                self.step = self.step + 1

        angles = (range_of_motion/2)*waveState*self.amplitude + center_pos

        # Target positions for all 7 joints (in radians)
        # You can change these values!
        msg.data = angles.tolist()

        #self.get_logger().info('Sending joint command...')
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
    node = FrankaMover()
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