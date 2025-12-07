#   colcon build --packages-select pathing_py6
#   source install/setup.bash 
#   ros2 run pathing_py6 cutPizza
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint 
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Int32, Float32, Float32MultiArray
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

# Gain scaling arrays (from the C++ controller)
Kp_joint_scale = np.array([2.2, 2.0, 1.8, 1.6, 1.4, 1.2, 1.0])
Kd_joint_scale = np.array([2.2, 2.0, 1.8, 1.6, 1.4, 1.2, 1.0])
Kp_cart_scale = np.ones(6)  # All 1.0 for cartesian
Kd_cart_scale = np.ones(6)  # All 1.0 for cartesian
jointCenteringRepulsion_scale = np.array([1.0, 0.1, 3.0, 0.5, 1.0, 1.0, 1.0])  # Per-joint scaling

# Default gain values
w = 3
damp = 1.2
default_Kp_joint =  w*w
default_Kd_joint = 2*w*damp

w = 6
damp = 1.4

default_Kp_cart = w*w
default_Kd_cart = 2*w*damp
default_jointCenteringRepulsion = 1.0


JointSpaceControllers = [0]
TaskSpaceControllers = [2,3,4,5]




class TaskSpaceObjectivePublisher(Node):
    def __init__(self):
        try:
            # pizza stuff
            #self.x__pizza_center = np.array([0.56, 0, 1.34]) # for cutting  pizza use lower z, like at z=1.32, 1.34 is above the surface
            #self.x__pizza_center = np.array([0.50, 0, 1.34]) # for cutting  pizza use lower z, like at z=1.32, 1.34 is above the surface
            self.x__pizza_center = np.array([0.50, 0, 1.30]) # for cutting  pizza use lower z, like at z=1.32, 1.34 is above the surface
            self.x__pizza_cutting_direction = np.array([0,0,-1])
            self.pizza_radius = 0.2 
            self.x__restPosition  = self.x__pizza_center - 0.2*self.x__pizza_cutting_direction

            self.definePizzaMathematically()

            # pizza cutting sequence
            # List of commands: (duration_in_seconds, function_name_as_string)
            pizzaCutSequence1_init = [
                (0.03, 'dbg_describe_pizza'),
                (5, 'arm'),
                (1, 'resetJointsAbovePizza'),
                (2, 'wait'),
            ]
            pizzaCutSequence1 = [
                (0.03, 'dbg_pizza_line_points'),
                (2, 'approach'),
                (1, 'wait'),
                (3.0, 'cutLine'),
                (1, 'wait'),
                (1.5, 'retract'),
                (1, 'resetJointsAbovePizza'),
                (1, 'wait'),
            ]

            debug1 = [
                (5, 'arm'),
                (5, 'wait'),
                (5, 'debug1'),
                (5, 'wait'),
            ]


            # current cutting sequence
            self.initDone = False
            self.currentIndex = 0
            self.sumPrevDurations = 0

            self.currentLine = 0
            self.maxLines = 4
            self.slowDownConst = 1.5

            self.initSeq = pizzaCutSequence1_init
            self.loopSeq = pizzaCutSequence1
            # self.initSeq = debug1
            # self.loopSeq = debug1


            #control stuff
            self.xd_pos = np.array([0,0,0])
            self.xd_dir = np.array([0,0,0])
            self.xd_roll = 0
            self.xd_dot_linear = np.array([0,0,0])
            self.xd_dot_angular = np.array([0,0,0])

            self.currentControllerType = 2;
            self.qd_pos = np.array([0,0,0,0,0,0,0])
            self.qd_vel = np.array([0,0,0,0,0,0,0])
            self.qd_acc = np.array([0,0,0,0,0,0,0]) 


            try:
                super().__init__('taskspace_objective')
                
                self.publisher_ = self.create_publisher(MultiDOFJointTrajectory, '/taskspace_objective', 10) # must be Trajectory not just point coz point is not top level message and will crash
                
                self.publisher_Jnt = self.create_publisher(JointTrajectoryPoint, '/requested_traj_point', 10)

                # Create publishers for controller parameters
                self.pub_controller_type = self.create_publisher(Int32, '/controller_type', 10)
                self.pub_jointCenteringRepulsion = self.create_publisher(Float32MultiArray, '/jointCenteringRepulsion', 10)
                self.pub_Kp_joint = self.create_publisher(Float32MultiArray, '/Kp_joint', 10)
                self.pub_Kd_joint = self.create_publisher(Float32MultiArray, '/Kd_joint', 10)
                self.pub_Kp_cart = self.create_publisher(Float32MultiArray, '/Kp_cart', 10)
                self.pub_Kd_cart = self.create_publisher(Float32MultiArray, '/Kd_cart', 10)
                
                self.timerPeriod = 1/50
                self.timer = self.create_timer(self.timerPeriod, self.timer_callback)
                
                
                self.running = True                    
                self._start_input_thread()             

                # Give the publisher some time to set up
                time.sleep(1)
                
                # Publish default values at init
                self._publish_defaults()
            except Exception as e:
                print(f" Error during TaskSpaceObjectivePublisher init: {e}")


            self.timeRestart = self.get_clock().now().nanoseconds / 1e9
            self.timePauseStart = 0
            self.timeInPause = 0
            self.prevIndex = -69
            self.moving = True
        except Exception as e:
            print(f" Init expeption: {e}")

    def _publish_defaults(self):
        """Publish default values to all controller parameter topics."""
        try:
            # Publish default controller type
            msg_ct = Int32()
            msg_ct.data = 2
            self.pub_controller_type.publish(msg_ct)
            
            # Publish default joint centering repulsion (per-joint array)
            jcr_values = (default_jointCenteringRepulsion * jointCenteringRepulsion_scale).astype(np.float32).tolist()
            msg_jcr = Float32MultiArray()
            msg_jcr.data = jcr_values
            self.pub_jointCenteringRepulsion.publish(msg_jcr)
            
            # Publish default Kp_joint
            kpj_values = (default_Kp_joint * Kp_joint_scale).astype(np.float32).tolist()
            msg_kpj = Float32MultiArray()
            msg_kpj.data = kpj_values
            self.pub_Kp_joint.publish(msg_kpj)
            
            # Publish default Kd_joint
            kdj_values = (default_Kd_joint * Kd_joint_scale).astype(np.float32).tolist()
            msg_kdj = Float32MultiArray()
            msg_kdj.data = kdj_values
            self.pub_Kd_joint.publish(msg_kdj)
            
            # Publish default Kp_cart
            kpc_values = (default_Kp_cart * Kp_cart_scale).astype(np.float32).tolist()
            msg_kpc = Float32MultiArray()
            msg_kpc.data = kpc_values
            self.pub_Kp_cart.publish(msg_kpc)
            
            # Publish default Kd_cart
            kdc_values = (default_Kd_cart * Kd_cart_scale).astype(np.float32).tolist()
            msg_kdc = Float32MultiArray()
            msg_kdc.data = kdc_values
            self.pub_Kd_cart.publish(msg_kdc)
            
            self.get_logger().info("Default controller parameters published.")
        except Exception as e:
            self.get_logger().error(f"Error publishing defaults: {e}")



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
        point = center + radius* np.cos(angle)*v1 + radius*np.sin(angle)*v2

        return point


    def cutPizzaLine(self, time, time_per_movement=3.0, n_line = 0, n_lines_max = 4):
        
        currentLineAngle_deg = (n_line/n_lines_max)*180 #180 so the lines dont overlap

        point1 = self.getPizzaPoint(currentLineAngle_deg)
        point2 = self.getPizzaPoint(currentLineAngle_deg+180)

        vel = (point2-point1)/time_per_movement

        progress = time/time_per_movement

        pos = (point2-point1)*progress + point1

        # self.get_logger().info(f"cutPizzaLine: p1: {np.round(point1,3)}, p2: {np.round(point2,3)}, vel: {np.round(vel,3)}, progress: {progress :.3}, pos: {np.round(pos,3)}")

        
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

        # Derivatives of basis functions
        dH0 =  6*t**2 - 6*t
        dH1 = -6*t**2 + 6*t
        dH2 =  3*t**2 - 4*t + 1
        dH3 =  3*t**2 - 2*t


        p0 = startPoint
        p1 = endPoint
        v0 = launchVector
        v1 = ApproachVector


        x = H0*p0 + H1*p1 + H2*v0 + H3*v1

        # Velocity wrt SPLINE t
        dx_dt = dH0*p0 + dH1*p1 + dH2*v0 + dH3*v1

        xdot = dx_dt * (1.0 / time_per_movement)
        return x, xdot

    def approachPizzaPoint(self, time, time_per_movement=3, n_line = 0, n_lines_max = 4 ):

        currentLineAngle_deg = (n_line/n_lines_max)*180 #180 so the lines dont overlap

        pizzaPoint = self.getPizzaPoint(currentLineAngle_deg)
        
        start = self.x__restPosition 
        approachDir = self.x__pizza_cutting_direction 
        
        x, xdot = self.cube3D_spline(time=time, time_per_movement=time_per_movement,startPoint=start,
                               endPoint=pizzaPoint, launchVector= np.array([0,0,0]), ApproachVector= approachDir)
        
        self.xd_pos = x
        self.xd_dir = self.x__pizza_cutting_direction
        self.xd_roll = 0
        self.xd_dot_linear = xdot
        self.xd_dot_angular = np.array([0,0,0])



        pass

    def retractFromPizzaPoint(self, time, time_per_movement=3, n_line = 0, n_lines_max = 4 ):
        currentLineAngle_deg = (n_line/n_lines_max)*180 #180 so the lines dont overlap

        pizzaPoint = self.getPizzaPoint(currentLineAngle_deg + 180)
        
        start = self.x__restPosition 
        approachDir = self.x__pizza_cutting_direction 
        
        x, xdot = self.cube3D_spline(time=time, time_per_movement=time_per_movement,startPoint=pizzaPoint,
                               endPoint=start, launchVector= -approachDir , ApproachVector= np.array([0,0,0]))
        

        self.xd_pos = x
        self.xd_dir = self.x__pizza_cutting_direction
        self.xd_roll = 0
        self.xd_dot_linear = xdot
        self.xd_dot_angular = np.array([0,0,0])

        pass


    def timer_callback(self):
        try:
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed_start = now - self.timeRestart

            prevIndex = self.currentIndex
            

            runTime_now = elapsed_start - self.timeInPause
            #self.get_logger().info(f"First: init done: {self.initDone},  index: {self.currentIndex}, runTime_now: {runTime_now}, elapsed_start: {elapsed_start}, real time {now}")


            if self.moving == False:
                self.sendCommands()
                return

            
            if not self.initDone:
                workingSequence = self.initSeq
            else:
                workingSequence = self.loopSeq

            #self.get_logger().info(f"First: init done: {self.initDone},  index: {self.currentIndex}, runTime_now: {runTime_now}, real time {now}")

            duration, func_name = workingSequence[self.currentIndex]
            nextCommandTime = self.sumPrevDurations + duration*self.slowDownConst
            while runTime_now > nextCommandTime: # move index if we late, sum duration
                #self.get_logger().info(f"Whiling: init done: {self.initDone},  previous_index: {self.currentIndex}, runTime_now: {runTime_now}, real time {now}")
                self.currentIndex +=1
                self.sumPrevDurations += duration*self.slowDownConst
                if self.currentIndex >= len(workingSequence):
                    break;
                duration, func_name = workingSequence[self.currentIndex]
                nextCommandTime = self.sumPrevDurations + duration*self.slowDownConst


            if self.currentIndex >= len(workingSequence): # switch to loop or increase line index, also reset all times as the sequence starts anew anyway
                    if not self.initDone:
                        workingSequence = self.loopSeq
                        self.initDone = True
                        
                    else:
                        self.currentLine = (self.currentLine+1)%self.maxLines
                    
                    self.currentIndex = 0
                    self.sumPrevDurations = 0
                    self.timeInPause = 0
                    self.timeRestart = self.get_clock().now().nanoseconds / 1e9
                    now = self.get_clock().now().nanoseconds / 1e9
                    elapsed_start = now - self.timeRestart
                    runTime_now = elapsed_start - self.timeInPause

            duration, func_name = workingSequence[self.currentIndex] #make sure we have the correct command after all that could have happend

            # function parameters
            fp_time = runTime_now-self.sumPrevDurations; 
            fp_duration = duration*self.slowDownConst

            #self.get_logger().info(f"Final: init done: {self.initDone},  index: {self.currentIndex}, reciepeie time: {fp_time}, real time {now}")


            self.reqControllerType = self.currentControllerType
                
            if func_name == "arm":
                self.reqControllerType = 2
                self.xd_pos = self.x__restPosition
                self.xd_dir = np.array([0,0,-1])
                self.xd_roll = 0
                
                pass
            elif func_name == "dbg_describe_pizza":
                self.get_logger().info(f" \033[45m Pizza has been mathematically defined: \033[0m \n"
                                       f" center: {self.x__pizza_center} \n"
                                       f" radius: {self.pizza_radius} \n"
                                       f" cutting direction: {self.x__pizza_cutting_direction} \n"
                                       f" vect1: {self.pizza_vect1}  \n" 
                                       f" vect2: {self.pizza_vect2}  \n" 
                                        )
            elif func_name == "dbg_pizza_line_points":
                currentLineAngle_deg = (self.currentLine/self.maxLines)*180 #180 so the lines dont overlap
                point1 = self.getPizzaPoint(currentLineAngle_deg)
                point2 = self.getPizzaPoint(currentLineAngle_deg+180)

                self.get_logger().info(f" \033[45m Current pizza cut line: \033[0m \n"
                                       f" line: {self.currentLine} / {self.maxLines}  \n"
                                       f" currentAngle: {currentLineAngle_deg} \n"
                                       f" point1: {np.round(point1,2)}  \n" 
                                       f" point2: {np.round(point2,2)}  \n" 
                                        )    
            elif func_name == "wait":
                pass
            elif func_name == "approach":
                self.reqControllerType = 4
                self.approachPizzaPoint(time=fp_time,time_per_movement=fp_duration,n_line=self.currentLine,n_lines_max=self.maxLines)
                pass
            elif func_name == "cutLine":
                self.reqControllerType = 5
                self.cutPizzaLine(time=fp_time,time_per_movement=fp_duration,n_line=self.currentLine,n_lines_max=self.maxLines)
                pass
            elif func_name == "retract":
                self.reqControllerType = 4
                self.retractFromPizzaPoint(time=fp_time,time_per_movement=fp_duration,n_line=self.currentLine,n_lines_max=self.maxLines)
                pass
            
            elif func_name == "resetJointsAbovePizza":
                self.reqControllerType = 0
                # qd_pos_deg = np.array([0,-25,0,-140,0,120,0])
                qd_pos_deg = np.array([0,-15,0,-128,0,110,0])
                self.qd_pos = qd_pos_deg*(np.pi/180.0)
                pass


            elif func_name == "debug1":
                self.xd_pos = [0.1, 0.1 ,1.6]
                self.xd_dir = np.array([0,0,-1])
                self.xd_roll = 0
                pass    
            else:
                self.get_logger().warn(f"Unknown function in sequence: '{func_name}'")

            

            if self.prevIndex != self.currentIndex:
                self.get_logger().info(f"Index change: init done: {self.initDone},  index: {self.currentIndex}, reciepeie time: {runTime_now :.1f}, function:  {func_name}, pizzaLine: {self.currentLine}")
                self.prevIndex = self.currentIndex
                self.get_logger().info(f"... xd_pos: {self.xd_pos[0] :.3f} {self.xd_pos[1] :.2f} {self.xd_pos[2] :.2f}")
                pass



            
            self.sendCommands()

            


        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")


    def sendCommands(self):
        ## make sure to request correct controller type
        if self.currentControllerType != self.reqControllerType:
            self.currentControllerType = self.reqControllerType
            
            msg = Int32()
            msg.data = int(self.currentControllerType)   # ensure Python int is converted cleanly
            self.pub_controller_type.publish(msg)
        else:
            pass

        if self.currentControllerType in TaskSpaceControllers: # some stuff happens only for taskspace controll      
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


            ## message prep
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

        elif self.currentControllerType in JointSpaceControllers:

            point = JointTrajectoryPoint()

            self.qd_pos = self.qd_pos.astype(float)
            self.qd_vel = self.qd_vel.astype(float)
            self.qd_acc = self.qd_acc.astype(float)

            point.positions = self.qd_pos.tolist()
            point.velocities = self.qd_vel.tolist()
            point.accelerations = self.qd_acc.tolist()

            self.publisher_Jnt.publish(point)

        else:
            self.get_logger().error(f"Unknown controller type : {self.currentControllerType}")

            pass


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
            self.get_logger().info(
                """
            \033[33mCommands:\033[0m
            help                print help
            stat                print current values and states
            stop                stop movement
            start               start movement
            ct <int>            select controller
            jcr <float>         select joint cenetering repulsion scaler
            kpj <float>         select joint space Kp
            kdj <float>         select joint space Kd
            kpc <float>         select task space Kp
            kdc <float>         select task space Kd
            mv <x> <y> <z>      set desiered taskspace position
            dir <x> <y> <z>     set desiered EE point direction
            roll <float>        set desiered EE roll
            setq <idx> <val>    set desiered joint position of joint idx in degrees
            """)

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
        
        elif cmd.startswith("ct "):
            try:
                _, val = cmd.split()
                controller_type = int(val)
                msg = Int32()
                msg.data = controller_type
                self.pub_controller_type.publish(msg)
                self.get_logger().info(f"Controller type set to {controller_type}")
                self.reqControllerType = controller_type
                self.currentControllerType = controller_type
            except Exception as e:
                self.get_logger().warn(f"Invalid controller type command: {e}")

        elif cmd.startswith("jcr "):
            try:
                _, val = cmd.split()
                base_val = float(val)
                jcr_values = (base_val * jointCenteringRepulsion_scale).astype(np.float32).tolist()
                msg = Float32MultiArray()
                msg.data = jcr_values
                self.pub_jointCenteringRepulsion.publish(msg)
                self.get_logger().info(f"Joint centering repulsion scaled by {base_val}: {np.round(jcr_values, 3)}")
            except Exception as e:
                self.get_logger().warn(f"Invalid joint centering repulsion command: {e}")

        elif cmd.startswith("kpj "):
            try:
                _, val = cmd.split()
                base_val = float(val)
                kpj_values = (base_val * Kp_joint_scale).astype(np.float32).tolist()
                msg = Float32MultiArray()
                msg.data = kpj_values
                self.pub_Kp_joint.publish(msg)
                self.get_logger().info(f"Kp_joint scaled by {base_val}: {np.round(kpj_values, 3)}")
            except Exception as e:
                self.get_logger().warn(f"Invalid Kp_joint command: {e}")

        elif cmd.startswith("kdj "):
            try:
                _, val = cmd.split()
                base_val = float(val)
                kdj_values = (base_val * Kd_joint_scale).astype(np.float32).tolist()
                msg = Float32MultiArray()
                msg.data = kdj_values
                self.pub_Kd_joint.publish(msg)
                self.get_logger().info(f"Kd_joint scaled by {base_val}: {np.round(kdj_values, 3)}")
            except Exception as e:
                self.get_logger().warn(f"Invalid Kd_joint command: {e}")

        elif cmd.startswith("kpc "):
            try:
                _, val = cmd.split()
                base_val = float(val)
                kpc_values = (base_val * Kp_cart_scale).astype(np.float32).tolist()
                msg = Float32MultiArray()
                msg.data = kpc_values
                self.pub_Kp_cart.publish(msg)
                self.get_logger().info(f"Kp_cart set to {base_val} for all 6 dimensions")
            except Exception as e:
                self.get_logger().warn(f"Invalid Kp_cart command: {e}")

        elif cmd.startswith("kdc "):
            try:
                _, val = cmd.split()
                base_val = float(val)
                kdc_values = (base_val * Kd_cart_scale).astype(np.float32).tolist()
                msg = Float32MultiArray()
                msg.data = kdc_values
                self.pub_Kd_cart.publish(msg)
                self.get_logger().info(f"Kd_cart set to {base_val} for all 6 dimensions")
            except Exception as e:
                self.get_logger().warn(f"Invalid Kd_cart command: {e}")
        
        elif cmd.startswith("mv "):
            try:
                parts = cmd.split()
                numbers = parts[1:] # the first part is the command 
                if len(numbers) < 3:
                    self.get_logger().warn(f"Wrong number of positions= {len(numbers)}, 3 are required; {cmd} ") 
                else:
                    if len(numbers) > 3:
                        self.get_logger().warn(f"Wrong number of positions= {len(numbers)}, ignoring the exces over 3 ") 
                    try:
                        floats = [float(x) for x in numbers]
                        self.xd_pos = np.array(floats)
                        self.get_logger().info(f"Desiered position set to {self.xd_pos}")
                    except ValueError as v:
                        self.get_logger().warn(f"Something is probably not a number; {v}, {cmd} ")
                    except Exception as e:
                        self.get_logger().warn(f"idk: {e}")        
                    
            except Exception as e:
                self.get_logger().warn(f"Invalid mv command: {e}")        

        elif cmd.startswith("dir "):
            try:
                parts = cmd.split()
                numbers = parts[1:] # the first part is the command 
                if len(numbers) < 3:
                    self.get_logger().warn(f"Wrong number of positions= {len(numbers)}, 3 are required; {cmd} ") 
                else:
                    if len(numbers) > 3:
                        self.get_logger().warn(f"Wrong number of positions= {len(numbers)}, ignoring the exces over 3 ") 
                    try:
                        floats = [float(x) for x in numbers]
                        self.xd_dir = np.array(floats)
                        self.get_logger().info(f"Desiered pointing direction set to {self.xd_dir}")
                    except ValueError as v:
                        self.get_logger().warn(f"Something is probably not a number; {v}, {cmd} ")
                    except Exception as e:
                        self.get_logger().warn(f"idk: {e}")        
                    
            except Exception as e:
                self.get_logger().warn(f"Invalid dir command: {e}")            

        elif cmd.startswith("roll "):
            try:
                parts = cmd.split()
                numbers = parts[1:] # the first part is the command 
                if len(numbers) < 1:
                    self.get_logger().warn(f"Wrong number of positions= {len(numbers)}, 1 are required; {cmd} ") 
                else:
                    if len(numbers) > 1:
                        self.get_logger().warn(f"Wrong number of positions= {len(numbers)}, ignoring the exces over 1 ") 
                    try:
                        self.xd_roll = float(numbers[0]) 

                        self.qd_pos[idx] = val
                        self.get_logger().info(f"Setting desiered EE roll to {self.xd_roll}")
                    except ValueError as v:
                        self.get_logger().warn(f"Something is probably not a number; {v}, {cmd} ")
                    except Exception as e:
                        self.get_logger().warn(f"idk: {e}")        
                    
            except Exception as e:
                self.get_logger().warn(f"Invalid roll command: {e}")        
        
        elif cmd.startswith("setq "):
            try:
                parts = cmd.split()
                numbers = parts[1:] # the first part is the command 
                if len(numbers) < 2:
                    self.get_logger().warn(f"Wrong number of positions= {len(numbers)}, 2 are required; {cmd} ") 
                else:
                    if len(numbers) > 2:
                        self.get_logger().warn(f"Wrong number of positions= {len(numbers)}, ignoring the exces over 2 ") 
                    try:
                        idx = int(numbers[0])
                        val = float(numbers[1]) 
                        val_rad = val*(np.pi/180)
                        if idx > 6 or idx < 0:
                            self.get_logger().warn(f"Invalid joint index= {idx}, must be int between 0 and 6 inclusive ") 
                        else:
                            self.qd_pos[idx] = val_rad
                            self.get_logger().info(f"Setting joint position {idx} to {self.qd_pos[idx]} (radians)")
                    except ValueError as v:
                        self.get_logger().warn(f"Something is probably not a number; {v}, {cmd} ")
                    except Exception as e:
                        self.get_logger().warn(f"idk: {e}")        
                    
            except Exception as e:
                self.get_logger().warn(f"Invalid setq command: {e}")        

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