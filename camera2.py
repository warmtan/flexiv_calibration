from vision.realsense_d415_tcp import RealsenseD415TCP
from utils.config_loader import ConfigLoader
import argparse
import numpy as np
import cv2

def find_checkerboard(color_img, checkerboard_size):
    """
    Returns X,Y pixel coordinates of checkerboard center in the image.
    返回图像中棋盘中心的 X,Y 像素坐标。
    """
    global refine_criteria
    bgr_color_data = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
    gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray_data, corners, checkerboard_size, (-1,-1), refine_criteria)
        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4,0,:]).astype(int)
        return checkerboard_pix
    else:
        return None
# Define a view for the checkerboard
def view_check():
    # Parse arguments 解析参数
    parser = argparse.ArgumentParser(description='Stream rgbd frames from a RealSense TCP Server.')
    # Setup options   设置选项
    parser.add_argument('--config_file', dest='config_file', action='store', default='./configurations/calibrate_config.json', help='Configuration file for the calibration.')
    args = parser.parse_args()
    configuration = ConfigLoader.load(args.config_file)
    # Connect to the camera  连接相机
    print('Connecting to camera...')
    camera = RealsenseD415TCP(configuration['camera_config_file'])

    #Create color and depth windows.
    cv2.namedWindow('color')
    cv2.namedWindow('depth')
    color_img, depth_img = camera.get_state()
    print(color_img)
    print(depth_img)
    if not (color_img is None and depth_img is None):
        checkerboard_size = (3,3)
        checkerboard_pix= find_checkerboard(color_img, checkerboard_size)
        print("checkerboard_pix",checkerboard_pix)
        if checkerboard_pix is not None:
            checkerboard_z = depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
            print("checkerboard_z",checkerboard_z)
            checkerboard_x = np.multiply(checkerboard_pix[0]-camera.intrinsics[0][2], checkerboard_z/camera.intrinsics[0][0])
            print("checkerboard_x",checkerboard_x)
            checkerboard_y = np.multiply(checkerboard_pix[1]-camera.intrinsics[1][2], checkerboard_z/camera.intrinsics[1][1])
            print("checkerboard_y",checkerboard_y)
            print('Observed points: ', [checkerboard_x,checkerboard_y,checkerboard_z])
        else:print('No checkerboard found')
    else:
        print('No depth or color frames')
    while True:
        color_bgr, depth = camera.get_state()
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

        cv2.imshow('color', color_rgb)
        cv2.imshow('depth', depth)
        k=cv2.waitKey(1)
        if k == ord("q"):
            break
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    # 查看相机示图
    view_check()