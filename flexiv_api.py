"""Flexiv robot ethernet communication python wrapper api.

Author: Junfeng Ding
"""

import time

import numpy as np

import flexivrdk

from base import Logging
from error_config import code as error_code

LOG_LEVEL = "info"


class ModeMap:
    idle = "MODE_IDLE"
    plan = "MODE_PLAN_EXECUTION"
    primitive = "MODE_PRIMITIVE_EXECUTION"
    cart_impedance_online = "MODE_CARTESIAN_IMPEDANCE_NRT"
    cart_impedance_stream = "MODE_CARTESIAN_IMPEDANCE"
    joint_position_online = "MODE_JOINT_POSITION_NRT"
    joint_position_stream = "MODE_JOINT_POSITION"
    joint_torque_stream = "MODE_JOINT_TORQUE"


class FlexivApi:
    """Flexiv Robot Control Class.

    This class provides python wrapper for Flexiv robot control in different modes.
    Features include:
        - torque control mode
        - plan execution mode
        - primitive excution mode
        - get robot status information
        - force torque record
    """

    logger_name = "FlexivApi"

    def __init__(self, robot_ip_address, pc_ip_address):
        """initialize.

        Args:
            robot_ip_address: robot_ip address string
            pc_ip_address: pc_ip address string

        Raises:
            RuntimeError: error occurred when ip_address is None.
        """
        self.error = error_code.ok
        self.mode = flexivrdk.Mode
        self.robot_states = flexivrdk.RobotStates()
        self.plan_info = flexivrdk.PlanInfo()
        self.robot_ip_address = robot_ip_address
        self.pc_ip_address = pc_ip_address
        self.init_robot()

    def init_robot(self, max_time=5):
        if hasattr(self, "robot"):
            del self.robot
        self.robot = flexivrdk.Robot(self.robot_ip_address, self.pc_ip_address)

        tic = time.time()
        while not self.robot.isConnected():
            if time.time() - tic > max_time:
                print("Init robot failed.")
                return error_code.robot_lost
            time.sleep(0.001)

        error = self.enable()
        if error:
            print("Robot init failed. Error: {}".format(error))
            raise RuntimeError("Robot init failed. Error: {}".format(error))
        print("init success")
        return error_code.ok

    def enable(self, max_time=10):
        """Enable robot after emergency button is released."""
        self.robot.enable()
        tic = time.time()
        while not self.is_operational():
            if time.time() - tic > max_time:
                return error_code.robot_enable_failed
            time.sleep(0.01)
        return error_code.ok

    def _get_robot_status(self):
        self.robot.getRobotStates(self.robot_states)
        return self.robot_states

    def mode_mapper(self, mode):
        assert mode in ModeMap.__dict__.keys(), "unknown mode name: %s" % mode
        return getattr(self.mode, getattr(ModeMap, mode))

    def get_control_mode(self):
        return self.robot.getMode()

    def set_control_mode(self, mode):
        control_mode = self.mode_mapper(mode)
        self.robot.setMode(control_mode)

    def switch_mode(self, mode, sleep_time=0.01):
        """switch to different control modes.

        Args:
            mode: 'joint_position_online', 'joint_position_stream', 'cart_impedance_online', 'cart_impedance_stream', 'plan', 'primitive'
            sleep_time: sleep time to control mode switch time

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        if self.get_control_mode() == self.mode_mapper(mode):
            return

        while self.get_control_mode() != self.mode_mapper("idle"):
            self.set_control_mode("idle")
            time.sleep(sleep_time)
        while self.get_control_mode() != self.mode_mapper(mode):
            self.set_control_mode(mode)
            time.sleep(sleep_time)

        print("set mode: {}".format(str(self.get_control_mode())))

    def get_emergency_state(self):
        """get robot is emergency stopped or not. The emergency state means the
        E-Stop is pressed. instead of being soft fault.

        Returns:
            True indicates robot is not stopped, False indicates robot is emergency stopped.

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return self.robot.isEstopReleased()

    def clear_fault(self):
        self.robot.clearFault()

    def is_fault(self):
        """Check if robot is in FAULT state."""
        return self.robot.isFault()

    def is_stopped(self):
        """Check if robot is stopped."""
        return self.robot.isStopped()

    def is_connected(self):
        """return if connected.

        Returns: True/False
        """
        return self.robot.isConnected()

    def is_operational(self):
        """Check if robot is operational."""
        return self.robot.isOperational()

    def get_tcp_pose(self):
        """get current robot's tool pose in world frame.

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().tcpPose)

    def get_tcp_vel(self):
        """get current robot's tool velocity in world frame.

        Returns:
            7-dim list consisting of (vx,vy,vz,vrw,vrx,vry,vrz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().tcpVel)

    def get_tcp_acc(self):
        """get current robot's tool acceleration in world frame.

        Returns:
            7-dim list consisting of (ax,ay,az,arw,arx,ary,arz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().tcpAcc)

    def get_tcp_force(self):
        """get current robot's tool force torque wrench in TCP frame.

        Returns:
            6-dim list consisting of (fx,fy,fz,wx,wy,wz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().extForceInTcpFrame[:6])

    def get_tcp_force_base(self):
        """get current robot's tool force torque wrench in base frame.

        Returns:
            6-dim list consisting of (fx,fy,fz,wx,wy,wz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().extForceInBaseFrame[:6])

    def get_camera_pose(self, camera_id: int = None):
        """get current wrist camera pose in world frame.

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        # TODO: (chongzhao) check ethernet connection api for camera pose
        if camera_id is None:
            return np.array(self._get_robot_status().camPose)
        else:
            return np.array(self._get_robot_status().camPose[camera_id])

    def get_flange_pose(self):
        """get current flange pose in world frame.

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().flangePose)

    def get_end_link_pose(self):
        """get current end link pose in world frame.

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().endLinkPose)

    def get_joint_pos(self):
        """get current joint value.

        Returns:
            7-dim numpy array of 7 joint position

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().q)

    def get_joint_vel(self):
        """get current joint velocity.

        Returns:
            7-dim numpy array of 7 joint velocity

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().dq)

    def get_joint_torque(self):
        """get measured link-side joint torque.

        Returns:
            7-dim numpy array of 7 link torques
        """
        return np.array(self._get_robot_status().tau)

    def get_external_joint_torque(self):
        """get estimated EXTERNAL link-side joint torque.

        Returns:
            7-dim numpy array of 7 link torques
        """
        return np.array(self._get_robot_status().tauExt)

    def get_primitive_states(self):
        """get current primitive states.
        #获取基元状态：获取当前基元状态。
        返回：
            基元状态字符串，包括“终止”、“时间段”、“到达目标”等

        Returns:
            string of primitive states, including 'terminated', 'timePeriod', 'reachedTarget' and so on

        Raises:
            当状态为none时出错
            RuntimeError: error occurred when mode is None.
        """
        return self.robot.getPrimitiveStates()

    def get_plan_info(self):
        """get current robot's running plan info.

        Returns:
            name string of running node in plan

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.robot.getPlanInfo(self.plan_info)
        return self.plan_info

    def get_plan_name_list(self):
        plan_list = self.robot.getPlanNameList()
        return plan_list

    def write_io(self, port, value):
        """Set io value.

        Args:
            port: 0~15
            value: True/False
        """
        assert port >= 0 and port <= 15
        self.robot.writeDigitalOutput(port, value)

    def execute_primitive(self, cmd):
        """Execute primitive.

        Args:
            cmd: primitive command string, e.x. "ptName(inputParam1=xxx, inputParam2=xxx, ...)"
        """
        self.switch_mode("primitive")
        print("Execute primitive: {}".format(cmd))
        self.robot.executePrimitive(cmd)

    def stop(self):
        """Stop current motion and switch mode to idle."""
        self.robot.stop()
        while self.get_control_mode() != self.mode_mapper("idle"):
            time.sleep(0.005)

    def send_impedance_online_pose(
        self, tcp, wrench=np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
    ):
        """make robot move towards target pose in impedance control mode,
        combining with sleep time makes robot move smmothly.

        Args:
            tcp: 7-dim list or numpy array, target pose (x,y,z,rw,rx,ry,rz) in world frame
            wrench: 6-dim list or numpy array, max moving force (fx,fy,fz,wx,wy,wz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("cart_impedance_online")
        pose = np.array(tcp)
        self.robot.sendTcpPose(pose, wrench)

    def stream_tcp_pose(self, tcp, vel=[0, 0, 0, 0, 0, 0], acc=[0, 0, 0, 0, 0, 0]):
        """realtime control move in impedance control.

        Args:
            tcp: 7-dim list or numpy array, target pose (x,y,z,rw,rx,ry,rz) in world frame
            vel: 6-dim list or numpy array, target velocity in world frame
            acc: 6-dim list or numpy array, target accuracy in world frame
        """
        self.switch_mode("cart_impedance_stream")
        self.robot.streamTcpPose(tcp, vel, acc)

    def send_joint_position(
        self,
        target_jnt_pos,
        target_jnt_vel=[0, 0, 0, 0, 0, 0, 0],
        target_jnt_acc=[0, 0, 0, 0, 0, 0, 0],
        max_jnt_vel=[2, 2, 2, 2, 2, 2, 2],
        max_jnt_acc=[3, 3, 3, 3, 3, 3, 3],
        max_jnt_jerk=[20, 20, 20, 20, 20, 20, 20],
    ):
        """make robot move towards target joint position in position control
        online mode without reach check.

        Args:
            target_jnt_pos: 7-dim list or numpy array, target joint position of 7 joints
            target_jnt_vel: 7-dim list or numpy array, target joint velocity of 7 joints
            target_jnt_acc: 7-dim list or numpy array, target joint acceleration of 7 joints
            max_jnt_vel: 7-dim list or numpy array, maximum joint velocity of 7 joints
            max_jnt_acc: 7-dim list or numpy array, maximum joint acceleration of 7 joints
            max_jnt_jerk: 7-dim list or numpy array, maximum joint jerk of 7 joints

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("joint_position_online")
        self.robot.sendJointPosition(
            target_jnt_pos,
            target_jnt_vel,
            target_jnt_acc,
            max_jnt_vel,
            max_jnt_acc,
            max_jnt_jerk,
        )

    def move_joint(
        self,
        target_jnt_pos,
        target_jnt_vel=[0, 0, 0, 0, 0, 0, 0],
        target_jnt_acc=[0, 0, 0, 0, 0, 0, 0],
        max_jnt_vel=[2, 2, 2, 2, 2, 2, 2],
        max_jnt_acc=[3, 3, 3, 3, 3, 3, 3],
        max_jnt_jerk=[20, 20, 20, 20, 20, 20, 20],
        joint_threshold=0.2 / 180 * np.pi,
    ):
        """move in position control online mode until reaching the target.

        Args:
            target_jnt_pos: 7-dim list or numpy array, target joint position of 7 joints
            target_jnt_vel: 7-dim list or numpy array, target joint velocity of 7 joints
            target_jnt_acc: 7-dim list or numpy array, target joint acceleration of 7 joints
            max_jnt_vel: 7-dim list or numpy array, maximum joint velocity of 7 joints
            max_jnt_acc: 7-dim list or numpy array, maximum joint acceleration of 7 joints
            max_jnt_jerk: 7-dim list or numpy array, maximum joint jerk of 7 joints
            joint_threshold: joint degree threshold to judge whether reach the target joint position

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("joint_position_online")
        while (
            not sum(
                abs(self.get_joint_pos() - np.array(target_jnt_pos)) > joint_threshold
            )
            == 0
        ):
            self.robot.sendJointPosition(
                target_jnt_pos,
                target_jnt_vel,
                target_jnt_acc,
                max_jnt_vel,
                max_jnt_acc,
                max_jnt_jerk,
            )
            if self.is_fault():
                self.clear_fault()
                time.sleep(0.5)
                if self.is_fault():
                    print("FAULT in moveJointOnline")
                    return
            time.sleep(0.1)
        print("Reached")

    def execute_plan_by_name(
        self,
        name,
    ):
        """execute plan by name, make sure control mode is switched into plan.

        Args:
            name: string of plan name

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("plan")
        self.robot.executePlanByName(name)

    def execute_plan_by_index(self, index):
        """execute plan by index, make sure control mode is switched into plan.

        Args:
            index: int, index of plan

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("plan")
        self.robot.executePlanByIndex(index)
