#!/usr/bin/env -S pixi exec -- python
import numpy as np
from scipy.optimize import fsolve

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from dataclasses import dataclass
from math import pi

@dataclass
class state_joint_struct():
    rz : float = 0.0
    up1 : float = 0.0
    up2 : float = 0.0
    up3 : float = 0.0
    gr : float = 0.0
    gc : float = 0.0

@dataclass
class vel_joint_struct():
    wrz : float = 0.0
    wup1 : float = 0.0
    wup2 : float = 0.0
    wup3 : float = 0.0
    wgr : float = 0.0
    wgc : float = 0.0

@dataclass
class Joy_struct():
    x : float = 0.0
    z : float = 0.0
    a : float = 0.0
    rz : float = 0.0
    gr : float = 0.0
    gc : float = 0.0
    
    def hasMoved(self) -> bool:
        if abs(self.x)>0.02 or \
            abs(self.z)>0.02 or \
            abs(self.a)>0.02 or \
            abs(self.rz)>0.02 or \
            abs(self.gr)>0.02 or \
            abs(self.gc)>0.02 :
                return True
        else:
            return False

 
@dataclass
class position_solver_struct():
    x : float = 3.0
    z : float = 0.0
    a : float = 0.0

from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from std_msgs.msg import Bool

from threading import Lock



class braccio_control_SendTraj(Node):
    def __init__(self):
        super().__init__("braccio_control_SendTraj")
        
        self.state_joint_lock=Lock()
        self.Joy_lock=Lock()
        
        self.state_joint=state_joint_struct()
        self.actual_pos_solver=position_solver_struct()
        self._joy_cmd=Joy_struct()
        self.lastJointTraj = JointTrajectoryPoint()
        self.lastJointTraj.positions=[0.0,0.0,0.0,0.0,0.0,0.0]
        self.lastJointTraj.velocities=[0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_names_ = ["arm_rot", "arm_1_up_down", "arm_2_up_down", "arm_3_up_down", "rot_grasp", "grasp_ctrl"]

        
        self._sendTraj = self.create_publisher(JointTrajectory, "/braccio_controller/joint_trajectory", 10)
        
        self._recJoy = self.create_subscription(Joy, "/joy",self._joyCall, 10) 
        self._update_joy=False

        self._recJointState = self.create_subscription(JointState, "/joint_states", self._joiStateCall, 10) 

        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timerCall)

        self.max_velocity=0.1#rad/s
        
        self._sendResTraj = self.create_publisher(Bool, "/braccio_controller/GenerationTrajResult", 10)
        

    def _joyCall(self, msg:Joy):
        joy_inp=Joy_struct(
            x = msg.axes[1], 
            z = msg.axes[0], 
            a = msg.axes[4], 
            rz = msg.axes[3], 
            gr = msg.axes[2], 
            gc = msg.axes[5],
        )
        joy_inp.gr=-(joy_inp.gr-1)/2
        joy_inp.gc=-(joy_inp.gc-1)/2
        
        # if abs(joy_inp.x)>0.05 or \
        #     abs(joy_inp.z)>0.05 or \
        #     abs(joy_inp.a)>0.05 or \
        #     abs(joy_inp.rz)>0.05 or \
        #     abs(joy_inp.gr)>0.05 or \
        #     abs(joy_inp.gc)>0.05 :
        #     self._update_joy=True
                
        with self.Joy_lock:
            self._joy_cmd=joy_inp
    
    def _joiStateCall(self, msg : JointState):
        with self.state_joint_lock:
            self.state_joint(
                msg.position[0],
                msg.position[1],
                msg.position[2],
                msg.position[3],
                msg.position[4],
                msg.position[5],
            )
            
    def timerCall(self, ):
        joy_inp=Joy_struct()
        with self.Joy_lock:
            joy_inp.x=self._joy_cmd.x
            joy_inp.z=self._joy_cmd.z
            joy_inp.a=self._joy_cmd.a
            joy_inp.rz=self._joy_cmd.rz
            joy_inp.gr=self._joy_cmd.gr
            joy_inp.gc=self._joy_cmd.gc
            
        if not joy_inp.hasMoved():
            return
        
        # calculate next step
        JoiTr = JointTrajectory()
        JoiTr.joint_names=self.joint_names_
        
        JoiTrPnt1 = self.lastJointTraj
        JoiTrPnt1.time_from_start.sec=int(self.timer_period/2)
        JoiTrPnt1.time_from_start.nanosec=int((self.timer_period/2-JoiTrPnt1.time_from_start.sec)*1E9)
        JoiTr.points.append(JoiTrPnt1)
        
        JoiTrPnt2 = JointTrajectoryPoint()
        JoiTrPnt2.time_from_start.sec=int(self.timer_period)
        JoiTrPnt2.time_from_start.nanosec=int((self.timer_period-JoiTrPnt1.time_from_start.sec)*1E9)
        
        nextpos = self.calculate_next_pos(joy_inp)
        
        for pos in nextpos:
            JoiTrPnt2.positions.append(pos)
            JoiTrPnt2.velocities.append(0.0)
        
        self.lastJointTraj=JoiTrPnt2
        
        JoiTr.points.append(JoiTrPnt2)
        self._sendTraj.publish(JoiTr)
        
        
    def calculate_next_pos(self, joy_inp:Joy_struct)->state_joint_struct:
        nextpos=state_joint_struct()
        
        drz = joy_inp.rz*self.max_velocity*(self.timer_period/2)
        rz=self.lastJointTraj.positions[0]+drz
        if abs(rz)>pi:
            self.get_logger().warning(f'movment is limited : rz')
            rz = min(max(rz, -pi), pi)
        nextpos.rz=rz
        
        dgr = joy_inp.gr*self.max_velocity*(self.timer_period/2)
        gr=self.lastJointTraj.positions[4]+dgr
        if abs(gr)>pi:
            self.get_logger().warning(f'movment is limited : gr')
            gr = min(max(gr, -pi), pi)
        nextpos.gr=gr
        
        dgc = joy_inp.gc*self.max_velocity*(self.timer_period/2)
        gc=self.lastJointTraj.positions[5]+dgc
        if abs(gc)>pi:
            self.get_logger().warning(f'movment is limited : gc')
            gc = min(max(gc, -pi), pi)
        nextpos.gc=gc
        
        dx = joy_inp.x*self.max_velocity*(self.timer_period/2)
        dz = joy_inp.z*self.max_velocity*(self.timer_period/2)
        da = joy_inp.a*self.max_velocity*(self.timer_period/2)
        
        x=self.actual_pos_solver.x+dx
        z=self.actual_pos_solver.z+dz
        a=self.actual_pos_solver.a+da
        
        sol=self.solve_trig_system(x, z, a)
        
        if sol != None:
            a1, a2, a3 = sol
            self.actual_pos_solver.x=x
            self.actual_pos_solver.z=z
            self.actual_pos_solver.a=a
            nextpos.up1 = a1
            nextpos.up2 = a2
            nextpos.up3 = a3
        else:
            nextpos.up1 = self.lastJointTraj.positions[1]
            nextpos.up2 = self.lastJointTraj.positions[2]
            nextpos.up3 = self.lastJointTraj.positions[3]
        return nextpos
            
    
    def solve_trig_system(self, x, y, a):
        """
        Solves for a1, a2, a3 given:
        x = cos(a1) + cos(a2) + cos(a3)
        y = sin(a1) + sin(a2) + sin(a3)
        a = a1 + a2 + a3
        """
        def equations(vars):
            a1, a2, a3 = vars
            eq1 = np.cos(a1) + np.cos(a2) + np.cos(a3) - x
            eq2 = np.sin(a1) + np.sin(a2) + np.sin(a3) - y
            eq3 = a1 + a2 + a3 - a
            return [eq1, eq2, eq3]
        
        initial_guess = [
            self.lastJointTraj.positions[1],
            self.lastJointTraj.positions[2],
            self.lastJointTraj.positions[3],
            ]
        
        solution, info, ier, msg = fsolve(equations, initial_guess, full_output=True)
        
        res=Bool()
        
        if ier == 1:
            a1, a2, a3 = solution
            self.get_logger().info(f"Find solution: [{a1},{a2},{a3}]")
            if abs(a1)>pi or abs(a2)>pi or abs(a3)>pi:
                res.data=False
                self._sendResTraj.publish(res)
                self.get_logger().error(f'solution goes over the limits')
                return None
            res.data=True
            self._sendResTraj.publish(res)
            return a1, a2, a3
        else:
            res.data=False
            self._sendResTraj.publish(res)
            self.get_logger().error(f'Could not find a stable solution :: {msg}')
            return None
    
    
def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_publisher = braccio_control_SendTraj()

            rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()