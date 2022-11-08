from vision.realsense_d415_tcp import RealsenseD415TCP
from utils.config_loader import ConfigLoader
import argparse
import cv2

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
    color, depth = camera.get_state()

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
