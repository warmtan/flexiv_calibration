# 手眼校准 中文
# 动机：
制定校准程序：

    它是端到端的。一旦系统准备好开始校准过程，无需人工干预。
    它适用于眼对手（相机安装在机器人上）和眼对手（相机固定在工作区）这两种情况。
    它同时优化深度和 rgb 通道。
    它很容易通过 .json 文件进行参数化，以适应任何可能的配置，而无需进行代码编辑。
    与其他更多手动或不同步骤的解决方案相比，它快速简便。
    它使用常见的 python 库。

建议的解决方案解释：

校准过程估计相机外在因素：

    eye-to_hand的机器人基础坐标。
    手眼的工具中心位置坐标。

在校准过程中，机器人会移动到一组预定义的 3D 位置。在每个 3D 位置，相机检测棋盘图案的中心点，并存储两者：机器人相对于底座的位置和检测到的中心点相对于相机中心的位置。

在 rgbd 图像中看到的像素的 3D 位置可以计算为：
point_z = camera_depth_img[checkerboard_center_pix[1]][checkerboard_center_pix[0]]
point_x = np.multiply(checkerboard_center_pix[0]-camera.intrinsics[0][2], point_z/camera.intrinsics[0][0])
point_y = np.multiply(checkerboard_center_pix[1]-camera.intrinsics[1][2], point_z/camera.intrinsics[1][1])

# 如何使用：

    将棋盘图案固定在机器人手动工具中心位置（眼对手）或工作区（眼对手）。确保它在校准过程中不会移动。

左图：眼睛在手，棋盘固定在工作区。右图：眼对手，棋盘固定在机器人末端执行器上。

    转到hand-eye_calibration/rs_server/并编译rs_server.cpp. 确保您已安装librealsense SDK。

$ cd hand-eye_calibration/rs_server/
$ make

    使用USB3.0接口连接realsense到电脑。
    启动 RealSense TCP 流服务器并指示PORT：

$ ./rs_streamer

在校准时保持运行，每次机器人连接以检索 RGBD 帧时，它将Connected to client.在终端上输出。要进一步测试从服务器获取 RGBD 帧的 python TCP 客户端，请编辑configurations/camera_config.json（有关详细信息，请参阅下一步），然后运行：$ python camera_streamer.py

    创建一个全新的 conda 环境并运行：$ pip install -r requirements.txt以安装测试版本所需的所有 python 库。

    打开configurations/calibrate_config.json文件，填写参数：

    校准类型：
        “EYE_IN_HAND”：安装在机器人上的摄像头。
        “EYE_TO_HAND”：摄像机固定在工作区，不受机器人移动的影响。
    calib_grid_step：它定义了“照片位置”之间的步距（以米为单位）。（例如 0.05 米）
    workspace_limits：它是由机器人底座的 3 个点 (X,Y,Z) 定义的立方体。在这个立方体内，机器人将移动到由calib_grid_step参数化的不同“照片位置”以捕获数据点。重要的是要考虑到从相机到棋盘格的最近距离高于深度通道的 minZ 值。注意：在 RealSense D415 上，minZ 为 0.5 米。形状 = (3,2)。行 = X、Y、Z。列 = MIN、MAX 值。
    reference_point_offset：它是棋盘图案中心的点 (X,Y,Z) 位置，相对于机器人的手眼基础或机器人的 TCP 手眼。
    tool_orientation：它是机器人基础系统中每个“照片位置”的机器人 TCP 的方向。
    checkerboard_size：棋盘的大小。例如，一个 4x4 黑白方格的棋盘尺寸为 3，因为这是内部十字的数量。
    camera_config_file：相机的配置文件。示例在：configurations/camera_config.json
        应该编辑：
            tcp_host_ip：rs_streamer TCP 服务器的 IP 地址。
            tcp_port : rs_streamer TCP 服务器的端口。
        可能编辑：如果您更改参数（im_height、im_width、buffer_size），您还应该编辑 rs_server 以匹配。
    robot_config_file：UR 机器人的配置文件。示例在：configurations/robot_config.json
        应该编辑：
            robots_ip：机器人的 IP 地址。
            program：应该在机器人上加载的程序的路径，包括它的安装：t​​cp 配置，重心......等。
            home_joints_rad：机器人回家姿势。每个关节的弧度。
        可能需要编辑：
            tcp_port：机器人的 TCP 端口连接。
            rtc_port：机器人的 RTC 端口连接。
            仪表板端口：仪表板
        编辑风险自负：有关速度和加速度的其余参数已针对 UR 机器人进行了测试，以在安全条件下执行校准。更改这些可能会加快过程，但也会因剧烈运动而导致图像捕捉不良，从而影响校准结果的精度。

    打开不同的终端并执行$ python calibrate.py以移动机器人并进行校准。注意：机器人将在 config.json 文件中以 calib_grid_step 为步长的 workspace_limits 指示的 3D 立方体网格内移动。要小心。

    校准过程结束后，将存储两个文件hand-eye_calibration/calibrations/：

    DATETIME_camera_pose.txt：它包含机器人和相机之间的变换矩阵。
    DATETIME_camera_depth_offset.txt：它包含一个值，该值是一个比例因子，应该与从相机捕获的每个像素相乘。注意：经测试，RealSense D415 系列不太可能出现缩放问题，但其他设备可能。

    要测试校准的相机外部参数的结果，请编辑configurations/touch_tester_config.json文件以满足您的设置并执行$ python touch_tester.py. 它提供了一个用户界面，用户可以在其中单击 RGBD 图像中的任何点，机器人将其末端执行器移动到该点的 3D 位置。
    
# 优点：

    它直接同时针对深度和 rgb 通道进行优化。这有很多优点。例如，它在校准过程中引入深度传感器的噪声，并用它计算变换矩阵。与仅使用 rgb 通道和参考点（例如 ArUco 标记）估计校准过程的其他解决方案相反，其深度精度与图像的深度通道不同，因此不会在校准过程中捕获这种差异。
    与其他更多手动或不同步骤的解决方案相比，它快速简便。
    它使用常见的 python 库。
    它适用于眼对手（相机安装在机器人上）和眼对手（相机固定在工作区）这两种情况。
    它是端到端的。一旦系统准备好开始校准过程，无需人工干预。

# 限制：

    精度随着样本的增加而增加。由于机器人必须移动到不同的照片位置，有时很难在非常小的空间或有障碍物的情况下使用。
    它仅适用于 RGBD 相机。

# 硬件：

    RGBD 相机、机器人和 PC。

# 测试：

    RGBD 摄像头：英特尔实感 D415
    机器人：UR10e
    PC：运行 Python 3.8 的 Ubuntu 18 和 20

# 致谢：

该校准软件由 INESCOP 鞋类技术研究所 ( https://www.inescop.es ) 开发，用于校准SoftManBot H2020项目的相机。Polígono Industrial Campo Alto，埃尔达，阿利坎特（西班牙）。

方法和源代码灵感来自：

https://github.com/IntelRealSense/librealsense

https://github.com/andyzeng/visual-pushing-grasping

https://github.com/hengguan/Hand-Eye-Calibration-Matlab

https://github.com/nghiaho12/rigid_transform_3D