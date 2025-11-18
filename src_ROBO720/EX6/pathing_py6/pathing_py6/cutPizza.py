#colcon build --packages-select pathing_py6
#source install/setup.bash 
#ros2 run pathing_py6 cutPizza
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from tf_transformations import quaternion_from_euler
import time
import numpy as np
import threading
import scipy
print(f" SCIPY ver: {scipy.__version__}") 

position_lim_MAX = [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973]
position_lim_MIN = [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]

max_pos= np.array(position_lim_MAX)
min_pos= np.array(position_lim_MIN)

center_pos = (max_pos + min_pos)/2
range_of_motion = max_pos-min_pos



class TaskSpaceObjectivePublisher(Node):
    def __init__(self):
        try:
            # pizza stuff
            self.x__pizza_center = np.array([0.4, 0, 1.4])
            self.x__pizza_cutting_direction = np.array([0,0,-1])
            self.pizza_radius = 0.2 
            self.x__restPosition  = self.x__pizza_center - 0.2*self.x__pizza_cutting_direction

            self.definePizzaMathematically()

            # pizza cutting sequence
            # List of commands: (duration_in_seconds, function_name_as_string)
            pizzaCutSequence1_init = [
                (5, 'arm'),
                (1, 'wait'),
            ]
            pizzaCutSequence1 = [
                (2, 'approach'),
                (1, 'wait'),
                (3.0, 'cutLine'),
                (1, 'wait'),
                (1.5, 'retract'),
                (1, 'wait'),
            ]


            # current cutting sequence
            self.initDone = False
            self.currentIndex = 0
            self.sumPrevDurations = 0

            self.currentLine = 0
            self.maxLines = 4

            self.initSeq = pizzaCutSequence1_init
            self.loopSeq = pizzaCutSequence1


            #control stuff
            self.xd_pos = np.array([0,0,0])
            self.xd_dir = np.array([0,0,0])
            self.xd_roll = 0
            self.xd_dot_linear = np.array([0,0,0])
            self.xd_dot_angular = np.array([0,0,0])


            try:
                super().__init__('taskspace_objective')
                
                self.publisher_ = self.create_publisher(MultiDOFJointTrajectory, '/taskspace_objective', 10) # must be Trajectory not just point coz point is not top level message and will crash
                self.timerPeriod = 1/50
                self.timer = self.create_timer(self.timerPeriod, self.timer_callback)
                
                
                self.running = True                    
                self._start_input_thread()             

                # Give the publisher some time to set up
                time.sleep(1)
            except Exception as e:
                print(f" Error during TaskSpaceObjectivePublisher init: {e}")


            self.timeRestart = self.get_clock().now().nanoseconds / 1e9
            self.timePauseStart = 0
            self.timeInPause = 0
            self.moving = True
        except Exception as e:
            print(f" Init expeption: {e}")


    def definePizzaMathematically(self):
        center = self.x__pizza_center
        dir = self.x__pizza_cutting_direction
        radius = self.pizza_radius
        
        dir = dir / np.linalg.norm(dir) #normalize

        #need two perp vectors to the direction. 
            # Check if dir is parallel to x-axis (|dot| ~ 1)
        if np.abs(np.dot(dir, np.array([1.0,0.0,0.0]))) > 0.9999:
            # Switch to y-axis
            v1 = np.array([0.0, 1.0, 0.0])
        else:
            v1 = np.array([1.0,0.0,0.0])

        # Make v1 orthogonal to dir (Gram-Schmidt)
        v1 = v1 - np.dot(v1, dir) * dir
        v1 = v1 / np.linalg.norm(v1)

        # Second perpendicular vector: cross product
        v2 = np.cross(dir, v1)
        v2 = v2 / np.linalg.norm(v2)

        self.pizza_vect1 = v1
        self.pizza_vect2 = v2

    # returns the point
    def getPizzaPoint(self,angle_deg,radiusScale=1) -> np.ndarray:
        center = self.x__pizza_center
        #dir = self.x__pizza_cutting_direction
        radius = self.pizza_radius*radiusScale
        
        angle = angle_deg/180 * np.pi

        v1 = self.pizza_vect1
        v2 = self.pizza_vect2

        #points = center + radius * (np.outer(np.cos(t), v1) + np.outer(np.sin(t), v2)) # outer is if t were a vector
        point = center + radius* np.cos(angle)*v1 + np.sin(angle)*v2

        return point


    def cutPizzaLine(self, time, time_per_movement=3, n_line = 0, n_lines_max = 4):
        
        currentLineAngle_deg = (n_line/n_lines_max)*180 #180 so the lines dont overlap

        point1 = self.getPizzaPoint(currentLineAngle_deg,1.1)
        point2 = self.getPizzaPoint(currentLineAngle_deg+180,1.1)

        vel = (point2-point1)/time_per_movement

        progress = time/time_per_movement

        pos = (point2-point1)*progress + point1


        
        self.xd_pos = pos
        self.xd_dir = self.x__pizza_cutting_direction
        self.xd_dot_linear = vel
        self.xd_dot_angular = np.array([0,0,0])


        
    def cube3D_spline(self,time, time_per_movement, startPoint, endPoint, launchVector, ApproachVector, ) -> np.ndarray:
        
        t = time/time_per_movement;
        
        H0 =  2*t**3 - 3*t**2 + 1
        H1 = -2*t**3 + 3*t**2
        H2 =  t**3 - 2*t**2 + t
        H3 =  t**3 - t**2

        p0 = startPoint
        p1 = endPoint
        v0 = launchVector
        v1 = ApproachVector


        x = H0*p0 + H1*p1 + H2*v0 + H3*v1
        return x

    def approachPizzaPoint(self, time, time_per_movement=3, n_line = 0, n_lines_max = 4 ):

        currentLineAngle_deg = (n_line/n_lines_max)*180 #180 so the lines dont overlap

        pizzaPoint = self.getPizzaPoint(currentLineAngle_deg)
        
        start = self.x__restPosition 
        approachDir = self.x__pizza_cutting_direction 
        
        x = self.cube3D_spline(time=time, time_per_movement=time_per_movement,startPoint=start,
                               endPoint=pizzaPoint, launchVector= np.array([0,0,0]), ApproachVector= approachDir)
        
        self.xd_pos = x
        self.xd_dir = self.x__pizza_cutting_direction
        self.xd_roll = 0
        self.xd_dot_linear = np.array([0,0,0])
        self.xd_dot_angular = np.array([0,0,0])



        pass

    def retractFromPizzaPoint(self, time, time_per_movement=3, n_line = 0, n_lines_max = 4 ):
        currentLineAngle_deg = (n_line/n_lines_max)*180 #180 so the lines dont overlap

        pizzaPoint = self.getPizzaPoint(currentLineAngle_deg + 180)
        
        start = self.x__restPosition 
        approachDir = self.x__pizza_cutting_direction 
        
        x = self.cube3D_spline(time=time, time_per_movement=time_per_movement,startPoint=pizzaPoint,
                               endPoint=start, launchVector= -approachDir , ApproachVector= np.array([0,0,0]))
        

        self.xd_pos = x
        self.xd_dir = self.x__pizza_cutting_direction
        self.xd_roll = 0
        self.xd_dot_linear = np.array([0,0,0])
        self.xd_dot_angular = np.array([0,0,0])

        pass


    def timer_callback(self):
        try:
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed_start = now - self.timeRestart
            

            runTime_now = elapsed_start - self.timeInPause
            self.get_logger().info(f"First: init done: {self.initDone},  index: {self.currentIndex}, runTime_now: {runTime_now}, elapsed_start: {elapsed_start}, real time {now}")


            if self.moving == False:
                return

            
            if not self.initDone:
                workingSequence = self.initSeq
            else:
                workingSequence = self.loopSeq

            #self.get_logger().info(f"First: init done: {self.initDone},  index: {self.currentIndex}, runTime_now: {runTime_now}, real time {now}")

            duration, func_name = workingSequence[self.currentIndex]
            nextCommandTime = self.sumPrevDurations + duration
            while runTime_now > nextCommandTime: # move index if we late, sum duration
                self.get_logger().info(f"Whiling: init done: {self.initDone},  previous_index: {self.currentIndex}, runTime_now: {runTime_now}, real time {now}")
                self.currentIndex +=1
                self.sumPrevDurations += duration
                if self.currentIndex >= len(workingSequence):
                    break;
                duration, func_name = workingSequence[self.currentIndex]
                nextCommandTime = self.sumPrevDurations + duration


            if self.currentIndex >= len(workingSequence): # switch to loop or increase line index, also reset all times as the sequence starts anew anyway
                    if not self.initDone:
                        workingSequence = self.loopSeq
                        self.sumPrevDurations = 0
                        
                    else:
                        self.currentLine = (self.currentLine+1)%self.maxLines
                    
                    self.sumPrevDurations = 0
                    self.timeInPause = 0
                    self.timeRestart = self.get_clock().now().nanoseconds / 1e9
                    now = self.get_clock().now().nanoseconds / 1e9
                    elapsed_start = now - self.timeRestart
                    runTime_now = elapsed_start - self.timeInPause

            duration, func_name = workingSequence[self.currentIndex] #make sure we have the correct command after all that could have happend

            fp_time = now-runTime_now;
            fp_duration = duration

            self.get_logger().info(f"Final: init done: {self.initDone},  index: {self.currentIndex}, reciepeie time: {fp_time}, real time {now}")

            if func_name == "arm":
                self.xd_pos = self.x__restPosition
                self.xd_dir = np.array([0,0,-1])
                self.xd_roll = 0
                pass
            elif func_name == "wait":
                pass
            elif func_name == "approach":
                self.approachPizzaPoint(time=fp_time,time_per_movement=fp_duration,n_line=self.currentLine,n_lines_max=self.maxLines)
                pass
            elif func_name == "cutLine":
                self.cutPizzaLine(time=fp_time,time_per_movement=fp_duration,n_line=self.currentLine,n_lines_max=self.maxLines)
                pass
            elif func_name == "retract":
                self.retractFromPizzaPoint(time=fp_time,time_per_movement=fp_duration,n_line=self.currentLine,n_lines_max=self.maxLines)
                pass
            else:
                self.get_logger().warn(f"Unknown function in sequence: '{func_name}'")


            
            # solve problems with rotations (x is first for roll 0, then y)
            
            z_axis = self.xd_dir #pointing direction is usually z axis
            roll_angle = self.xd_roll
            # Choose an arbitrary y-axis (not parallel to z)
            tmp = np.array([1, 0, 0])
            if np.allclose(z_axis, tmp):
                tmp = np.array([0, 1, 0])

            # Construct orthonormal frame
            x_axis = np.cross(tmp, z_axis)
            x_axis = x_axis / np.linalg.norm(x_axis) #must be like this to autocast float
            y_axis = np.cross(z_axis, x_axis)

            # Apply roll around z
            roll_rot = scipy.spatial.transform.Rotation.from_rotvec(roll_angle * z_axis)
            rot_matrix = np.column_stack((x_axis, y_axis, z_axis))  # 3x3 rotation
            rot = scipy.spatial.transform.Rotation.from_matrix(rot_matrix) * roll_rot

            # Convert to quaternion
            quat = rot.as_quat()  # returns [x, y, z, w]

            

            point = MultiDOFJointTrajectoryPoint()

            # Position + Orientation as Transform
            transform = Transform()
            transform.translation.x = float(self.xd_pos[0])
            transform.translation.y = float(self.xd_pos[1])
            transform.translation.z = float(self.xd_pos[2])
            transform.rotation.x = quat[0]
            transform.rotation.y = quat[1]
            transform.rotation.z = quat[2]
            transform.rotation.w = quat[3]  

            # Velocity as Twist
            velocity = Twist()
            velocity.linear.x = float(self.xd_dot_linear[0])
            velocity.linear.y = float(self.xd_dot_linear[1])
            velocity.linear.z = float(self.xd_dot_linear[2])
            velocity.angular.x = float(self.xd_dot_angular[0])
            velocity.angular.y = float(self.xd_dot_angular[1])
            velocity.angular.z = float(self.xd_dot_angular[2])

            # Acceleration as Twist
            acceleration = Twist()
            acceleration.linear.x = 0.0
            acceleration.linear.y = 0.0
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
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")



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
            self.get_logger().info("help, status, stop, start, period <s>, ampl <1>, go, pcmd <x> <y> <z> ")

        # elif cmd.startswith("stat"):
        #     self.get_logger().info(f" Motion period: {self.motionPeriod}, Motion amplitude: {self.amplitude}, Moving: {self.moving}, Step: {self.step}, cmd freq: {1/self.timerPeriod} [Hz]")

        elif cmd == "stop":
            if self.moving == False:
                self.get_logger().info("Already stopped")
            else:
                self.get_logger().info("Stopping")
                self.moving = False
            
        elif cmd == "start":
            # if self.followPositionCommand:
            #     self.get_logger().info("Not following position command anymore")
            #     self.followPositionCommand = False
            if self.moving == True:
                self.get_logger().info("Already moving")
            else:
                self.get_logger().info("Starting")
                self.moving = True

        # elif cmd.startswith("period "):
        #     try:
        #         _, val = cmd.split()
        #         new_period = float(val)
        #         self.motionPeriod = new_period
        #         self.get_logger().info(f"Motion period set to {new_period} seconds.")
        #     except Exception as e:
        #         self.get_logger().warn(f"Invalid period command: {e}")

        # elif cmd.startswith("ampl "):
        #     try:
        #         _, val = cmd.split()
        #         new_amplitude = float(val)
        #         self.amplitude = new_amplitude
        #         self.get_logger().info(f"Motion amplitude set to {new_amplitude}.")
        #     except Exception as e:
        #         self.get_logger().warn(f"Invalid period command: {e}")

        # elif cmd.startswith("pcmd "):
        #     try:
        #         parts = cmd.split()
        #         numbers = parts[1:] # the first part is the command 
        #         if len(numbers) < 3:
        #             self.get_logger().warn(f"Wrong number of positions= {len(numbers)}, 3 are required; {cmd} ") 
        #         else:
        #             if len(numbers) > 3:
        #                 self.get_logger().warn(f"Wrong number of positions= {len(numbers)}, ignoring the exces over 3 ") 
        #             try:
        #                 floats = [float(x) for x in numbers]
        #                 self.positionCommand = floats
        #                 self.followPositionCommand = True    
        #                 self.get_logger().info(f"Position command set to {floats}, following.")
        #             except ValueError as v:
        #                 self.get_logger().warn(f"Something is probably not a number; {v}, {cmd} ")
        #             except Exception as e:
        #                 self.get_logger().warn(f"idk: {e}")        
                    
        #     except Exception as e:
        #         self.get_logger().warn(f"Invalid position command: {e}")        
        
        # elif cmd.startswith("go"):
        #     self._handle_command("stop") #stop the motion
        #     self.get_logger().info(f"Now following position command")
        #     self.followPositionCommand = True


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