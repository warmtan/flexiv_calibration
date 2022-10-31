from flexiv_robot import FlexivRobot
from vision.realsense_d415_tcp import RealsenseD415TCP
import utils.utils as utils
import vision.utils as visionutils
from utils.config_loader import ConfigLoader
from datetime import datetime
import numpy as np
import cv2
import json
import time
from scipy import optimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os

class Configuration(object):
    def __init__(self, config_file):
        config = ConfigLoader.load(args.config_file)
        #creates a class which instance attributes are based on the config dictionary
        for k, v in config.items():
            setattr(self, k, v)
        # Checkerboard size as a tuple.
        self.checkerboard_size = (self.checkerboard_size, self.checkerboard_size)
        self.reference_point_offset = np.array(self.reference_point_offset)
        self.tool_orientation = np.array(self.tool_orientation)
        self.workspace_limits = np.array(self.workspace_limits)

    @staticmethod
    def dump_sample_file():
        config = {
            "calibration_type": "EYE_IN_HAND",
            "camera_config_file":"PATH/TO/FILE",
            "workspace_limits": [[-0.490, 0.390], [-0.645, -0.185], [0.462, 0.710]],
            "calib_grid_step": 0.05,
            "reference_point_offset": [[0.74550],[-0.00895],[0.04900], [1]],
            "tool_orientation": [1.226,-2.890,0.00],
            "checkerboard_size": 3
        }

        with open('configurations/sample_configuration.json', 'w') as fp:
            json.dump(config, fp)

def calibrate(config):
    # Construct 3D calibration grid across workspace  跨工作空间构建三维标定网格
    gridspace_x = np.linspace(config.workspace_limits[0][0], config.workspace_limits[0][1], int(1 + (config.workspace_limits[0][1] - config.workspace_limits[0][0])/config.calib_grid_step))
    gridspace_y = np.linspace(config.workspace_limits[1][0], config.workspace_limits[1][1], int(1 + (config.workspace_limits[1][1] - config.workspace_limits[1][0])/config.calib_grid_step))
    gridspace_z = np.linspace(config.workspace_limits[2][0], config.workspace_limits[2][1], int(1 + (config.workspace_limits[2][1] - config.workspace_limits[2][0])/config.calib_grid_step))
    calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)
    num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]*calib_grid_x.shape[2]
    calib_grid_x.shape = (num_calib_grid_pts,1)
    calib_grid_y.shape = (num_calib_grid_pts,1)
    calib_grid_z.shape = (num_calib_grid_pts,1)
    calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)

    #measured_pts: points generated by sampling out of the config.workspace_limits[] + checkerboard offset from tool.
    #              It is the position of the tool when taking a picture, ideally this is the position of the center of the checkerboard in robot world coordinates.
    #              This is achieved easily when the camera is fixed and the robot moves the checkerboard in the image.
    #              As the robot position + checkerboard offset from tool = the position of the center of the fixed checkerboard in robot world coordinates.
    measured_pts = []
    #obseverved_pts: This is the position X,Y,Z in meters of the center of the checkerboard with respect to the origin of the camera in the camera world coordinates.
    observed_pts = []
    #observed_pix: Pixel locations of the center of the checkerboard in the image.
    observed_pix = []

    print(f'Going to calibrate in {num_calib_grid_pts} different points.')

    # Connect to the robot
    print('Connecting to robot...')  #连接出现问题  修改为client 不需要读取configuration文件夹中的robot_config.json
    robot=FlexivRobot("192.168.2.100","192.168.2.109")
    # Slow down robot to SAFE values
    #go home
    # array1=np.array([0.68659854,-0.11323812,0.28814268,0.00116366,0.00595991,0.99997848,0.00248933])
    array1=np.array([0.68659854,-0.11323812,0.5,0.00116366,0.00595991,0.99997848,0.00248933])
    robot.move_ptp(array1, 
                    max_jnt_vel=[6, 6, 7, 7, 14, 14, 14],
                    max_jnt_acc=[3.60, 3.60, 4.20, 4.20, 8.40, 8.40, 8.40])

    # Connect to the camera
    print('Connecting to camera...')
    camera = RealsenseD415TCP(config.camera_config_file)
    # Move robot to each calibration point in workspace
    print('Collecting data...')
    for calib_pt_idx in range(num_calib_grid_pts):
        tool_position = calib_grid_pts[calib_pt_idx,:]
        print('Calibration point: ', calib_pt_idx, '/', num_calib_grid_pts)
        #使用eye to hand 需要增加其内容属性
        robot.move_to_pose(tool_position, config.tool_orientation)  #得到笛卡尔坐标系
        time.sleep(1)
        # Wait for a coherent pair of frames: depth and color
        camera_color_img, camera_depth_img = camera.get_state()
        if not (camera_depth_img is None and camera_color_img is None):
            checkerboard_pix = visionutils.find_checkerboard(camera_color_img, config.checkerboard_size)
            if checkerboard_pix is not None:
                checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
                checkerboard_x = np.multiply(checkerboard_pix[0]-camera.intrinsics[0][2], checkerboard_z/camera.intrinsics[0][0])
                checkerboard_y = np.multiply(checkerboard_pix[1]-camera.intrinsics[1][2], checkerboard_z/camera.intrinsics[1][1])
                if checkerboard_z != 0:
                    observed_pts.append([checkerboard_x, checkerboard_y, checkerboard_z])
                    observed_pix.append(checkerboard_pix)
                    # Get current robot pose
                    # current_pose = robot.get_cartesian_pose() #这里使用了robot！！！！！！！！！！！！
                    current_pose = robot.get_cartesian_pose()
                    if config.calibration_type == "EYE_IN_HAND":
                        rot_vec = np.array(current_pose)
                        rot_vec.shape = (1,6)
                        T_be = utils.V2T(rot_vec) #这里使用了untils 识别这个标定板
                        invT_be = np.linalg.inv(T_be)
                        # Save calibration point and observed checkerboard center
                        checker2tool = np.dot(invT_be, config.reference_point_offset)
                        checker2tool = checker2tool[:3, 0]
                        measured_pts.append(checker2tool)
                        print('Measured points: ',  checker2tool)
                    else: # "EYE_TO_HAND"
                        tool_position = current_pose[:3] + config.reference_point_offset.flatten()[:3]
                        measured_pts.append(tool_position)
                        print('Measured points: ',  tool_position)
                    # Save calibration point and observed checkerboard center
                    print('Observed points: ', [checkerboard_x,checkerboard_y,checkerboard_z])
                    print('Checkerboard pix: ', checkerboard_pix)

                else:
                    print('checkerboard Z == 0')
            else:
                print('No checkerboard found')
        else:
            print('No depth or color frames')

    # Move robot back to home pose
    measured_pts = np.asarray(measured_pts)
    observed_pts = np.asarray(observed_pts)
    observed_pix = np.asarray(observed_pix)
    world2camera = np.eye(4)
    print('Total valid points: ',  measured_pts.shape[0], '/', num_calib_grid_pts)

    # Estimate rigid transform with SVD (from Nghia Ho)
    def get_rigid_transform(A, B):
        assert len(A) == len(B)
        N = A.shape[0]; # Total points  总分
        centroid_A = np.mean(A, axis=0) #  Find centroids  发现重心
        centroid_B = np.mean(B, axis=0)
        AA = A - np.tile(centroid_A, (N, 1)) # Centre the points 中心的点
        BB = B - np.tile(centroid_B, (N, 1))
        H = np.dot(np.transpose(AA), BB) # Dot is matrix multiplication for array 点是数组的矩阵乘法
        U, S, Vt = np.linalg.svd(H) # Find the rotation matrix R  求旋转矩阵R
        R = np.dot(Vt.T, U.T)
        if np.linalg.det(R) < 0: # Special reflection case 特殊的反映情况
           Vt[2,:] *= -1
           R = np.dot(Vt.T, U.T)
        t = np.dot(-R, centroid_A.T) + centroid_B.T # Find the traslation t  求平移T
        return R, t

    def get_rigid_transform_error(z_scale):
        nonlocal measured_pts, observed_pts, observed_pix, world2camera

        # Apply z offset and compute new observed points using camera intrinsics  应用z偏移量，并使用相机intrinsic计算新的观察点
        observed_z = observed_pts[:,2:] * z_scale
        observed_x = np.multiply(observed_pix[:,[0]]-camera.intrinsics[0][2],observed_z/camera.intrinsics[0][0])
        observed_y = np.multiply(observed_pix[:,[1]]-camera.intrinsics[1][2],observed_z/camera.intrinsics[1][1])
        new_observed_pts = np.concatenate((observed_x, observed_y, observed_z), axis=1)

        # Estimate rigid transform between measured points and new observed points  #估计测量点和新观测点之间的刚变换
        R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
        t.shape = (3,1)
        world2camera = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)

        # Compute rigid transform error  计算刚变换误差
        registered_pts = np.dot(R,np.transpose(measured_pts)) + np.tile(t,(1,measured_pts.shape[0]))
        error = np.transpose(registered_pts) - new_observed_pts
        error = np.sum(np.multiply(error,error))
        rmse = np.sqrt(error/measured_pts.shape[0])
        return rmse

    # Optimize z scale w.r.t. rigid transform error
    print('Calibrating...')
    z_scale_init = 1
    optim_result = optimize.minimize(get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
    camera_depth_offset = optim_result.x

    # Save camera optimized offset and camera pose
    now = datetime.now()
    date_time = now.strftime("%Y-%m-%d_%H:%M:%S")
    print('Saving...')
    
    path_dir = os.path.join(os.getcwd(), 'calibrations')
    if not os.path.exists(path_dir):
        os.makedirs(path_dir)

    np.savetxt('./calibrations/' + date_time + '_' + config.calibration_type  + '_camera_depth_scale.txt', camera_depth_offset, delimiter=' ')
    get_rigid_transform_error(camera_depth_offset)
    camera_pose = np.linalg.inv(world2camera)
    np.savetxt('./calibrations/' + date_time + '_' + config.calibration_type  + '_camera_pose.txt', camera_pose, delimiter=' ')
    print('Done.')

    # DEBUG CODE -----------------------------------------------------------------------------------
    np.savetxt('./calibrations/measured_pts.txt', np.asarray(measured_pts), delimiter=' ')
    np.savetxt('./calibrations/observed_pts.txt', np.asarray(observed_pts), delimiter=' ')
    np.savetxt('./calibrations/observed_pix.txt', np.asarray(observed_pix), delimiter=' ')
    #measured_pts = np.loadtxt('measured_pts.txt', delimiter=' ')
    #observed_pts = np.loadtxt('observed_pts.txt', delimiter=' ')
    #observed_pix = np.loadtxt('observed_pix.txt', delimiter=' ')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(measured_pts[:,0],measured_pts[:,1],measured_pts[:,2], c='blue')
    print(camera_depth_offset)
    R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(observed_pts))
    t.shape = (3,1)
    camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
    camera2robot = np.linalg.inv(camera_pose)
    t_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(observed_pts)) + np.tile(camera2robot[0:3,3:],(1,observed_pts.shape[0])))
    ax.scatter(t_observed_pts[:,0],t_observed_pts[:,1],t_observed_pts[:,2], c='red')
    new_observed_pts = observed_pts.copy()
    new_observed_pts[:,2] = new_observed_pts[:,2] * camera_depth_offset[0]
    R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
    t.shape = (3,1)
    camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
    camera2robot = np.linalg.inv(camera_pose)
    t_new_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(new_observed_pts)) + np.tile(camera2robot[0:3,3:],(1,new_observed_pts.shape[0])))
    ax.scatter(t_new_observed_pts[:,0],t_new_observed_pts[:,1],t_new_observed_pts[:,2], c='green')
    plt.show()


if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description='Perform the Hand Eye calibration and store the transformation matrix.')

    # Setup options
    parser.add_argument('--config_file', dest='config_file', action='store', default='./configurations/calibrate_config.json', help='Configuration file for the calibration.')
    parser.add_argument('--dump_sample_file', dest='dump_sample_file', action='store', default=False, help='When true, dumps a sample configuration file.')
    args = parser.parse_args()
    if args.dump_sample_file:
        Configuration.dump_sample_file()
    else:
        configuration = Configuration(args.config_file)
        calibrate(configuration)
