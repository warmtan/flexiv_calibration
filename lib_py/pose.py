"""
Utils Functions for Pose Transformation
Author: Chongzhao Mao, Lingxiao Song
"""

import cv2
import numpy as np
from pyquaternion import Quaternion
from transforms3d.euler import euler2mat, euler2quat, quat2euler
from transforms3d.quaternions import mat2quat, quat2mat

from vector import compute_norm,compute_quat_u2v, compute_quat_uu2vv_specify, normalize


__all__ = [
    "pos_quat_to_pose_4x4",
    "pos_quat_to_pose_7D",
    "pose_to_pos_quat",
    "pose_to_7D",
    "pose_to_4x4",
    "pos_euler_to_pose_4x4",
    "pose_to_pos_euler",
    "compute_average_quaternion",
    "compute_average_pose",
    "pose_to_6D",
    "generate_random_pose",
    "quat_to_euler",
    "quat_to_degree",
    "pose_to_euler",
    "pose_to_degree",
    "compute_angle_two_quat",
    "pos_rot_to_pose_4x4",
    "print_6D_pose",
    "batch_quat_to_mat",
    "batch_rodrigues",
    "batch_rot_to_quat",
    "quat_to_axis_angle",
    "pose_to_world",
    "pose_post_process",
    "transformation_to_world",
    "compute_rot_z",
    "rotation_about_xyz",
    "compute_pose_from_2d_points",
]


def pos_quat_to_pose_4x4(pos, quat):
    """pose = pos_quat_to_pose_4x4(pos, quat)
    Convert pos and quat into pose, 4x4 format

    Args:
        pos: length-3 position
        quat: length-4 quaternion

    Returns:
        pose: numpy array, 4x4
    """
    pose = np.zeros([4, 4])
    mat = quat2mat(quat)
    pose[0:3, 0:3] = mat[:, :]
    pose[0:3, -1] = pos[:]
    pose[-1, -1] = 1
    return pose


def pos_quat_to_pose_7D(pos, quat):
    """pose = pos_quat_to_pose_4x4(pos, quat)
    Convert pos and quat into pose, 7D format

    Args:
        pos: numpy array, length-3 position
        quat: numpy array, length-4 quaternion

    Returns:
        pose: numpy array, length in 7.
    """
    pose = np.zeros(7)
    pose[0:3] = pos[:]
    pose[3:7] = quat[:]
    return pose


def pose_to_pos_quat(pose):
    """pos, quat = pose_to_pos_quat(pose)
    Convert pose into position and quaternion

    Args:
        pose: numpy array, in 7D or 4x4 format

    Returns:
        pos: numpy array, length-3 position
        quat: numpy array, length-4 quaternion
    """
    pose = pose_to_7D(pose)
    pos = pose[0:3]
    quat = pose[3:7]
    return pos, quat


def pose_to_7D(pose):
    """pose = pose_to_7D(pose)
    Convert pose into 7D format

    Args:
        pose: numpy array, in 6D, 7D or 4x4 format

    Returns:
        pose: numpy array, in 7D format
    """
    if pose.size == 7:
        return pose
    pose = pose_to_4x4(pose)
    pos = pose[0:3, -1]
    quat = mat2quat(pose[0:3, 0:3])
    new_pose = pos_quat_to_pose_7D(pos, quat)
    return new_pose


def pose_to_6D(pose):
    """pose = pose_to_6D(pose)
    Convert pose into 6D format

    Args:
        pose: numpy array, in 6D, 7D or 4x4 format

    Returns:
        pose: numpy array, in 6D format
    """
    if pose.size == 6:
        return pose
    pose = pose_to_4x4(pose)
    rot_vec, _ = cv2.Rodrigues(pose[:3, :3])
    rot_vec = rot_vec.flatten()
    pos = pose[:3, -1].flatten()
    new_pose = np.hstack([pos, rot_vec])
    return new_pose


def pose_to_4x4(pose):
    """pose = pose_to_4x4(pose)
    Convert pose into 4x4 format

    Args:
        pose: numpy array, in 6D, 7D or 4x4 format

    Returns:
        pose: numpy array, in 4x4 format
    """
    if pose.size == 16:
        return pose
    if pose.size == 7:
        pose = pose.reshape(7)
        pos = pose[0:3]
        quat = pose[3:7]
        new_pose = pos_quat_to_pose_4x4(pos, quat)
        return new_pose
    if pose.size == 6:
        rot_mat, _ = cv2.Rodrigues(pose[3:])
        new_pose = np.zeros([4, 4])
        new_pose[0:3, 0:3] = rot_mat[:, :]
        new_pose[0:3, -1] = pose[:3]
        new_pose[-1, -1] = 1
        return new_pose
    raise RuntimeError("unknown size: " + str(pose.size))


def compute_average_quaternion(quats):
    """mean_quat = compute_average_quaternion(quats)
    Compute average quaternion.

    Args:
        quats: N x 4 numpy array, with each row a quaternion.
    Returns:
        mean_quat: 4D numpy array.
    """
    count = len(quats)
    if count <= 1:
        return quats.flatten()
    # compute mean of the first two quats
    mean_quat = Quaternion.slerp(
        Quaternion(normalize(quats[0])), Quaternion(normalize(quats[1])), amount=0.5
    )
    for idx in range(2, count):
        amount = idx / (idx + 1)
        q = Quaternion(normalize(quats[idx]))
        mean_quat = Quaternion.slerp(mean_quat, q, amount=amount)
    return mean_quat.elements


def compute_average_pose(poses):
    """mean_pose = compute_average_pose(poses)
    Compute average pose (in 7D format).

    Args:
        poses: N x 7 numpy array, with each row a 7D pose.
    Returns:
        mean_pose: 7D numpy array.
    """
    pos = poses[:, :3]
    quat = poses[:, 3:]
    mean_pos = pos.mean(0).flatten()
    mean_quat = compute_average_quaternion(quat)
    mean_pose = np.hstack([mean_pos, mean_quat])
    return mean_pose


def generate_random_pose(flag="vec"):
    """Generate random pose.

    Returns:
        pose: 4x4 or 7D numpy array.
    """
    pos = np.random.rand(3) - 0.5
    u = np.random.rand(3) - 0.5
    v = np.random.rand(3) - 0.5
    quat = compute_quat_u2v(u, v)
    if flag == "vec":
        pose = pos_quat_to_pose_7D(pos, quat)
    elif flag == "mat":
        pose = pos_quat_to_pose_4x4(pos, quat)
    return pose


def quat_to_euler(quat):
    """Convert quaternion to euler representation.

    Args:
        quat: 4D numpy array.
    Returns:
        euler: 3D numpy array.
    """
    euler = np.array(quat2euler(quat))
    return euler


def quat_to_degree(quat):
    """Convert quaternion to degree representation.

    Args:
        quat: 4D numpy array.
    Returns:
        degree: 3D numpy array.
    """
    return quat_to_euler(quat) / np.pi * 180


def pose_to_euler(pose):
    """Extract rotation from pose and convert to euler representation.

    Args:
        pose: 4x4 or 7D numpy array.
    Returns:
        euler: 3D numpy array.
    """
    pos, quat = pose_to_pos_quat(pose)
    return quat_to_euler(quat)


def pose_to_degree(pose, with_position=False):
    """Extract rotation from pose and convert to degree representation.

    Args:
        pose: 4x4 or 7D numpy array.
        with_position: bool, return position or not
    Returns:
        degree: 3D numpy array of degree, or 6D numpy array of pos and degree.
    """
    pos, quat = pose_to_pos_quat(pose)
    if with_position:
        return np.concatenate([pos, quat_to_degree(quat)], axis=0)
    else:
        return quat_to_degree(quat)


def pos_euler_to_pose_4x4(pos, euler_angle):
    """Calculate transformation matrix when given position and euler angle.

    Args:
        pos: list or 1D numpy array, the translation.
        euler_angle: list or 1D numpy array, the euler angle.

    Returns:
        pose: 4x4 numpy array, the converted transformation matrix.
    """
    euler_angle = np.asarray(euler_angle) / 180.0 * np.pi
    quat = np.array(euler2quat(euler_angle[0], euler_angle[1], euler_angle[2]))
    pose = pos_quat_to_pose_4x4(np.asarray(pos), quat)
    return pose


def pose_to_pos_euler(pose):
    """Calculate position and euler angle when given transformation matrix.

    Args:
        pose: 4x4 numpy array, the converted transformation matrix.

    Returns:
        pos: 1D numpy array, the translation.
        euler_angle: 1D numpy array, the euler angle.
    """
    pos, quat = pose_to_pos_quat(pose)
    euler_angle = quat_to_euler(quat)
    return pos, euler_angle


def compute_angle_two_quat(quat_0, quat_1, out_dim=1):
    """Compute angle in degree of two quaternions.

    Args:
        quat_0: 4D numpy array.
        quat_1: 4D numpy array.
        out_dim: (optional) 1 or 3. 1 for norm of 3 dimension, 3 for each dimension.
    Returns:
        degree_diff: 1D or 3D numpy array.
    """
    degree_0 = quat_to_degree(quat_0)
    degree_1 = quat_to_degree(quat_1)
    degree_diff = np.abs(degree_0 - degree_1)
    degree_diff = np.minimum(degree_diff, 360 - degree_diff)
    if out_dim == 1:
        degree_diff = compute_norm(degree_diff)
    elif out_dim == 3:
        pass
    return degree_diff


def pos_rot_to_pose_4x4(tran, rot):
    """pose = pos_rot_to_pose_4x4(tran, rot)
    Convert a 3x1 translation and 3x3 rotation matrix into 4x4 format

    Args:
        tran: 3x1 numpy array, the translation vector.
        rot: 3x3 numpy array, the rotation matrix.

    Returns:
        pose: numpy array, in 4x4 format
    """
    tran = np.reshape(tran, (3, 1))
    temp_matrix = np.concatenate([rot, tran], axis=-1)
    transformation = np.concatenate([temp_matrix, np.array([[0, 0, 0, 1]])], axis=0)
    return transformation


def print_6D_pose(pose):
    """Convert 6D pose into string.

    Args:
        pose: numpy array, in 6D, 7D or 4x4 format
    Returns:
        res: string, 6D pose string.
    """
    if pose is None:
        return "None"
    pose_6D = pose_to_6D(pose)
    res = "pos: [%.2f, %.2f, %.2f] mm" % (
        pose_6D[0] * 1000,
        pose_6D[1] * 1000,
        pose_6D[2] * 1000,
    )
    res += "  angle: [%.3f, %.3f, %.3f] degree" % (
        pose_6D[3] * 180 / np.pi,
        pose_6D[4] * 180 / np.pi,
        pose_6D[5] * 180 / np.pi,
    )
    return res


def batch_quat_to_mat(quat, use_jax=False):
    """Convert quaternion coefficients to rotation matrix.
    For batch size = 1, batch_quat_to_mat achieve comparable speed
    with transforms3d.quaternions.quat2mat. For large batch size,
    batch_quat_to_mat is about 30 times faster than
    transforms3d.quaternions.quat2mat.

    Args:
        quat: numpy array of batch_size x 4.
    Returns:
        Rotation numpy array of batch_size x 3 x 3,
                 matrix corresponding to the quaternion.
    """
    if use_jax:
        import jax.numpy as np
    else:
        import numpy as np

    batch_size = quat.shape[0]
    quat = quat / np.linalg.norm(quat + 1e-8, axis=1, keepdims=True)
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    w2, x2, y2, z2 = w ** 2, x ** 2, y ** 2, z ** 2
    wx, wy, wz = w * x, w * y, w * z
    xy, xz, yz = x * y, x * z, y * z

    rotMat = np.stack(
        [
            w2 + x2 - y2 - z2,
            2 * xy - 2 * wz,
            2 * wy + 2 * xz,
            2 * wz + 2 * xy,
            w2 - x2 + y2 - z2,
            2 * yz - 2 * wx,
            2 * xz - 2 * wy,
            2 * wx + 2 * yz,
            w2 - x2 - y2 + z2,
        ],
        axis=1,
    ).reshape(batch_size, 3, 3)
    return rotMat


def batch_rodrigues(axis_angle, use_jax=False):
    """Calculates the rotation matrices for a batch of rotation vectors.
    For batch size = 1, batch_rodrigues achieve comparable speed
    with cv2.Rodrigues. For large batch size, batch_rodrigues is about
    50 times faster than cv2.Rodrigues.

    Args:
        axis_angle: numpy array of batch_size x 3

    Returns:
        rot_mat: batch rotation matrices for the given axis-angle parameters
    """
    if use_jax:
        import jax.numpy as np
    else:
        import numpy as np

    axis_angle_norm = np.linalg.norm(axis_angle + 1e-8, axis=1)
    angle = axis_angle_norm[..., np.newaxis]
    axis_angle_normalized = axis_angle / angle
    angle = angle * 0.5
    v_cos = np.cos(angle)
    v_sin = np.sin(angle)
    quat = np.concatenate((v_cos, v_sin * axis_angle_normalized), 1)
    rot_mat = batch_quat_to_mat(quat, use_jax=use_jax)
    rot_mat = rot_mat.reshape(rot_mat.shape[0], 9)
    return rot_mat


def batch_rot_to_quat(rotation_matrix, eps=1e-6):
    """Convert batch rotation matrix to 4d quaternion vector. For batch size
    input , batch_rot_to_quat is about 50 times faster than
    transforms3d.quaternions.mat2quat.

    Args:
        rotation_matrix: numpy array of [batch, 3, 3], batch rotation matrix to convert.

    Returns:
        quat: numpy array of batch rotation relations in quaternion.
    """
    rot_mat = np.reshape(rotation_matrix, (-1, 3, 3))
    hom = np.reshape(np.asarray([0, 0, 1], dtype=np.float32), (1, 3, 1))
    hom = np.tile(hom, (rot_mat.shape[0], 1, 1))
    rotation_matrix = np.concatenate([rot_mat, hom], axis=-1)

    rmat_t = np.transpose(rotation_matrix, (0, 2, 1))

    mask_d2 = rmat_t[:, 2, 2] < eps

    mask_d0_d1 = rmat_t[:, 0, 0] > rmat_t[:, 1, 1]
    mask_d0_nd1 = rmat_t[:, 0, 0] < -rmat_t[:, 1, 1]

    t0 = 1 + rmat_t[:, 0, 0] - rmat_t[:, 1, 1] - rmat_t[:, 2, 2]
    quat0 = np.stack(
        [
            rmat_t[:, 1, 2] - rmat_t[:, 2, 1],
            t0,
            rmat_t[:, 0, 1] + rmat_t[:, 1, 0],
            rmat_t[:, 2, 0] + rmat_t[:, 0, 2],
        ],
        -1,
    )
    t0_rep = np.tile(t0, (4, 1)).T

    t1 = 1 - rmat_t[:, 0, 0] + rmat_t[:, 1, 1] - rmat_t[:, 2, 2]
    quat1 = np.stack(
        [
            rmat_t[:, 2, 0] - rmat_t[:, 0, 2],
            rmat_t[:, 0, 1] + rmat_t[:, 1, 0],
            t1,
            rmat_t[:, 1, 2] + rmat_t[:, 2, 1],
        ],
        -1,
    )
    t1_rep = np.tile(t1, (4, 1)).T

    t2 = 1 - rmat_t[:, 0, 0] - rmat_t[:, 1, 1] + rmat_t[:, 2, 2]
    quat2 = np.stack(
        [
            rmat_t[:, 0, 1] - rmat_t[:, 1, 0],
            rmat_t[:, 2, 0] + rmat_t[:, 0, 2],
            rmat_t[:, 1, 2] + rmat_t[:, 2, 1],
            t2,
        ],
        -1,
    )
    t2_rep = np.tile(t2, (4, 1)).T

    t3 = 1 + rmat_t[:, 0, 0] + rmat_t[:, 1, 1] + rmat_t[:, 2, 2]
    quat3 = np.stack(
        [
            t3,
            rmat_t[:, 1, 2] - rmat_t[:, 2, 1],
            rmat_t[:, 2, 0] - rmat_t[:, 0, 2],
            rmat_t[:, 0, 1] - rmat_t[:, 1, 0],
        ],
        -1,
    )
    t3_rep = np.tile(t3, (4, 1)).T

    mask_c0 = mask_d2 * mask_d0_d1
    mask_c1 = mask_d2 * ~mask_d0_d1
    mask_c2 = ~mask_d2 * mask_d0_nd1
    mask_c3 = ~mask_d2 * ~mask_d0_nd1
    mask_c0 = np.reshape(mask_c0, (-1, 1)).astype(quat0.dtype)
    mask_c1 = np.reshape(mask_c1, (-1, 1)).astype(quat1.dtype)
    mask_c2 = np.reshape(mask_c2, (-1, 1)).astype(quat2.dtype)
    mask_c3 = np.reshape(mask_c3, (-1, 1)).astype(quat3.dtype)

    quat = quat0 * mask_c0 + quat1 * mask_c1 + quat2 * mask_c2 + quat3 * mask_c3
    quat /= np.sqrt(
        t0_rep * mask_c0 + t1_rep * mask_c1 + t2_rep * mask_c2 + t3_rep * mask_c3
    )
    quat *= 0.5
    return quat


def quat_to_axis_angle(quat):
    """Convert quaternion vector to angle axis of rotation.
    Args:
        quaternion: numpy array of [batch, 4], batch of quaternion.

    Returns:
        angle_axis: batch numpy array for angle axis of rotation.
    """
    q1 = quat[..., 1]
    q2 = quat[..., 2]
    q3 = quat[..., 3]
    sin_squared_theta = q1 * q1 + q2 * q2 + q3 * q3

    sin_theta = np.sqrt(sin_squared_theta)
    cos_theta = quat[..., 0]
    two_theta = 2.0 * np.where(
        cos_theta < 0.0,
        np.arctan2(-sin_theta, -cos_theta),
        np.arctan2(sin_theta, cos_theta),
    )

    k_pos = two_theta / sin_theta
    k_neg = 2.0 * np.ones_like(sin_theta)
    k = np.where(sin_squared_theta > 0.0, k_pos, k_neg)

    angle_axis = np.zeros_like(quat)[..., :3]
    angle_axis[..., 0] += q1 * k
    angle_axis[..., 1] += q2 * k
    angle_axis[..., 2] += q3 * k
    return angle_axis


def pose_post_process(pose):
    """Post process pose, to make sure its tranition uint is in meter.

    Args:
        pose: 4x4 or 7D transformation matrix

    Returns:
        pose: 4x4 or 7D transformation matrix, in meter unit for transition.
    """
    if pose.size == 16:
        if pose[0:3, -1].max() > 3.0:
            pose[0:3, -1] = pose[0:3, -1] * 0.001
    elif pose.size == 7:
        if pose[:3].max() > 3.0:
            pose[:3] = pose[:3] * 0.001
    return pose


def pose_to_world(cam_pose, pose_list):
    """Convert 4x4 pose in camera coordinate into 7D world coordinate.

    Args:
        cam_pose: 4x4 matrix. Camera pose in world coordinate.
        pose_list: a single or list of 4x4 transformation matrix. Object or grasp
            pose in camera coordinate.

    Returns:
        pose_list: a single or list of 7D transformation vector. Object or grasp
            pose in world coordinate.
    """
    is_list = True
    if not isinstance(pose_list, list):
        pose_list = [pose_list]
        is_list = False
    for i, pose in enumerate(pose_list):
        pose_list[i] = pose_to_7D(
            np.dot(cam_pose, pose_post_process(pose_to_4x4(pose)))
        )
    if not is_list:
        pose_list = pose_list[0]
    return pose_list


def transformation_to_world(cam_pose, transform_list):
    """Convert 4x4 relative transformation matrix which defined in camera
    coordinate into 7D relative transformation in world coordinate.

    Args:
        cam_pose: 4x4 matrix. Camera pose in world coordinate.
        transform_list: a single or list of 4x4 transformation matrix. Offset or other
            relative transformation defined in camera coordinate.

    Returns:
        transform_list: a single or list of 7D transformation vector. Offset or other
            relative transformation defined in world coordinate.
    """
    is_list = True
    if not isinstance(transform_list, list):
        transform_list = [transform_list]
        is_list = False
    for i, transform in enumerate(transform_list):
        transform = pose_post_process(pose_to_4x4(transform))
        transform_list[i] = pose_to_7D(
            np.dot(np.dot(cam_pose, transform), np.linalg.pinv(cam_pose))
        )
    if not is_list:
        transform_list = transform_list[0]
    return transform_list


def compute_rot_z(current_pose, angle):
    """compute [w, x, y, z] to rotate around z axis in world frame.

    Args:
        current_pose: current [x, y, z, rw, rx, ry, rz] in world frame
        angle: rotation radian around z axis in world frame

    Returns:
        target_pose: [x, y, z, rw, rx, ry, rz]
    """
    tool_pose = current_pose.copy()
    tool_mat = quat2mat(tool_pose[3:])
    rot = euler2mat(0, 0, angle)
    target_quat = mat2quat(np.matmul(rot, tool_mat))
    target_pose = np.concatenate([tool_pose[:3], target_quat])
    return target_pose