"""Test FlexivRobot.

Author: Chongzhao Mao
"""

import sys
from scipy.spatial.transform import Rotation as R
from time import sleep

sys.path.insert(0, ".")

from base import Logging  # noqa

from flexiv_robot import FlexivRobot
import numpy as np
logger = Logging.init_app_logger("debug")



def main():
    client=FlexivRobot("192.168.2.100","192.168.2.109")
    print(client.get_tcp_pose())
    print(client.get_joint_pos())
    # 笛卡尔控制
    array1=np.array([0.68659854,-0.11323812,0.28814268,0.00116366,0.00595991,0.99997848,0.00248933])
    client.move_ptp(array1, 
                    max_jnt_vel=[6, 6, 7, 7, 14, 14, 14],
                    max_jnt_acc=[3.60, 3.60, 4.20, 4.20, 8.40, 8.40, 8.40])
    print(array1[3:7])
    B_pose = R.from_quat(array1[3:7]).as_euler('xyz', degrees=True) 
    print("B_pose:",B_pose)
    # 关节控制
    # array1=np.array([-4.65114973e-03,-7.00091183e-01,4.26836050e-04,1.57289064e+00,6.37172209e-03,6.86869800e-01,3.61075555e-03])
    # client.move_online(array1)
    # array2=np.array([-0.0155,
    #                  -0.451,
    #                  0.619,
    #                  1.62,
    #                  0.001596,
    #                  0.5288,
    #                  0.004077])
    # client.move_jnt_pos(array2,
    #                     max_jnt_vel=[12, 12, 14, 14, 28, 28, 28],
    #                     max_jnt_acc=[7.2, 7.2, 8.4, 8.4, 16.8, 16.8, 16.8])
    # sleep(0.1)

    
    # array=np.array([-0.0149,-0.451,1.622,1.52,-0.011,0.51,-0.0069])
    # client.move_jnt_pos(array)
    # ROB_IP = "192.168.2.100"
    # AI_IP = "192.168.2.102"
    # client = FlexivRobot(ROB_IP, AI_IP)
    # client.switch_mode("idle")
    # client.switch_mode("cart_impedance_online")
    # client.switch_mode(mode="plan")
    # client.clear_fault()
    # client.execute_plan_by_name("PLAN-Home")
    # client.execute_plan_by_name("TestEasyMove", max_time=1)
    # client.execute_plan_by_name("PLAN-Home")
    # client.execute_plan_by_name("TestEasyMove", end_node_path="rooNode::move0")
    # client.execute_plan_by_name("PLAN-Home")lib_py.
    # client.execute_plan_by_name(
    #     "TestEasyMove", end_node_path="rooNode::move0", end_node_path_iter_num=1
    # )
    # client.execute_plan_by_name("PLAN-Home")
    # client.move_online(client.move_tool_trans(trans=[0, 0.02, 0]))
    # client.execute_plan_by_name("PLAN-Home")
    # joint_pos = client.get_joint_pos()
    # joint_pos[2] += 1
    # client.move_joint(target_jnt_pos=joint_pos)

    # state = client.get_emergency_state()
    # print("robot state: %s", state)

    # tcp_pose = client.get_tcp_pose()
    # print("tcp pose: %s", tcp_pose)

    # tcp_force = client.get_tcp_force()
    # print("tcp force: %s", tcp_force)

    # camera_pose_list = client.get_camera_pose()
    # camera_pose = client.get_camera_pose(camera_id=0)
    # print("camera pose list: %s", str(camera_pose_list))
    # print("camera pose of camera id = 0: %s", camera_pose)

    # plan_info = client.get_plan_info()
    # print("current plan info: %s", str(plan_info))

    # plan_list = client.get_plan_name_list()
    # for i in range(len(plan_list)):
    #     print("[" + str(i) + "]" + str(plan_list[i]))


if __name__ == "__main__":
    main()
