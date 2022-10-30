<!-- reading  until.py-->

def degrees_to_radians(degrees):
    """
    将度数转换为弧度

def V2T( V ):
    """
    V2T 将 1x6 向量转换为 4x4 变换矩阵
    @param： V - 1x6 形式为 [x，y，z，rx，ry，rz] 的向量，其中 x，y，z 是平移
    而 rx，ry，rz 是角度的角度-轴表示，其中
    表示轴的单位向量已乘以
    旋转它。
    @returns：T - 标准 4x4 变换矩阵

def V2R( V ):
    """
    V2R 将 1x3 角度轴矢量转换为 3x3 旋转矩阵
    @param： V - 1x3 形式向量 [rx，ry，rz] 其中 rx，ry，rz 是角度轴
    表示角度，其中表示轴的单位向量
    已被围绕它的旋转角度所倍增。
    @returns：R - 标准 3x3 变换矩阵

def T2V( T ):
    """
    T2V 将 4x4 变换矩阵转换为 1x6 矢量
    @param：T - 标准 4x4 变换矩阵
    @returns：V - 形式为 [x，y，z，rx，ry，rz] 的 1x6 向量，其中 x，y，z 是平移
    而 rx，ry，rz 是角度的角度-轴表示，其中
    表示轴的单位向量已乘以
    旋转它

def R2V( R ):
    """
    R2V 将 3x3 旋转矩阵转换为 1x3 角度轴矢量
    @param：R - 标准 3x3 变换矩阵
    @returns： V - 1x3 形式向量 [rx，ry，rz] 其中 rx，ry，rz 是一个角度轴
    表示角度，其中表示轴的单位向量
    已由其旋转角度倍增

def vrrotvec2mat(ax_ang):
    """
    创建与围绕常规的旋转相对应的旋转矩阵
    轴按指定角度。

def vrrotmat2vec(mat1, rot_type='proper'):
    """
    创建一个轴角np。数组的旋转矩阵:
    ＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
    nx3x3旋转矩阵转换
    nx3x3 numpy数组
    @param rot_type:“不适当的”，如果有可能
    如果输入矩阵不正确，
    “适当的”。默认“适当的”
    @type rot_type: string ('proper' or ' incorrect ')
    @return: 3D旋转轴和角度(ax_ang)
    5项:
    第一个3:轴
    4:角
    5:1表示正确，-1表示不正确
    @rtype: numpy 5xn数组
    
    phi = pi
    #这个奇点需要详细的符号歧义解决方案
    #计算旋转轴，确保所有元素>= 0
    #实数符号由下面的翻转算法得到
