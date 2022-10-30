"""
Vectors Transformation Utils
Author: Lingxiao Song
"""

import numpy as np
from transforms3d.quaternions import axangle2quat, qmult, rotate_vector

__all__ = [
    "get_cross",
    "compute_angle",
    "project_points",
    "compute_vertical_u2v",
    "normalize",
    "compute_quat_u2v",
    "compute_quat_u2v_with_axis",
    "compute_quat_uu2vv",
    "compute_quat_uu2vv_specify",
    "project_vector2vector",
    "project_vector2plane",
    "is_collinear",
]


def get_cross(p1, p2, p):
    """value = get_cross(p1, p2, p)
    Compute cross product between vector p1p2 and p1p.
    Args:
        p1, p2, p: float[2] list or numpy vector, 2D points.
    Returns:
        value: float, cross product.
    """
    return (p2[0] - p1[0]) * (p[1] - p1[1]) - (p2[1] - p1[1]) * (p[0] - p1[0])


def compute_angle(v1, v2):
    """angle = compute_angle(v1,v2)
    Compute angle (degree) between two vectors.
    Args:
        v1, v2: numpy vector of the same shape.
    Returns:
        angle: float, range [0, 180].
    """
    return (
        np.arccos(np.clip(np.dot(normalize(v1), normalize(v2)), -1.0, 1.0))
        * 180
        / np.pi
    )


def project_points(x, y, z, a, b, c):
    """point = project_points(x, y, z, a, b, c)
    Projects the points with coordinates (x, y, z) onto the plane defined by a*x + b*y + c*z = 1.
    Args:
        x, y, z: float, the coordinates of input point.
        a, b, c: float, plane parameters.
    Returns:
        point: float[3] numpy array, the projected point.
    """
    vector_norm = a * a + b * b + c * c
    normal_vector = np.array([a, b, c]) / np.sqrt(vector_norm)
    point_in_plane = np.array([a, b, c]) / vector_norm
    points = np.array([x, y, z])
    points_from_point_in_plane = points - point_in_plane
    proj_onto_normal_vector = np.dot(
        points_from_point_in_plane, normal_vector
    ).flatten()
    proj_onto_plane = (
        points_from_point_in_plane - proj_onto_normal_vector[0] * normal_vector
    )
    return point_in_plane + proj_onto_plane


def project_vector2vector(u, v):
    """z = project_vector2vector(u, v)
    Project vector u onto vector v.
    Args:
        u, v: numpy vectors of the same shape.
    Returns:
        z: numpy vector of the same shape with u and v.
    """
    v_norm = np.linalg.norm(v)
    z = v * np.dot(u, v) / (v_norm ** 2)
    return z


def project_vector2plane(u, normal_vector):
    """z = project_vector2plane(u, normal_vector)
    Project vector u onto the plane with specified normal vector.
    Args:
        u: numpy vector.
        normal_vector: numpy vector, normal vecotor of the projected plane.
    Returns:
        z: numpy vector of the same shape with u and v.
    """
    return u - project_vector2vector(u, normal_vector)


def compute_vertical_u2v(u, v):
    """z = compute_vertical_u2v(u, v)
    Find the vertical vector of u in the plane defined by u and v.
    Args:
        u, v: numpy vector of the same shape.
    Returns:
        z: numpy vector of the same shape with u and v.
    """
    m = -np.dot(u, v) / np.sum(v ** 2)
    z = u + m * v
    z = normalize(z)
    return z


def normalize(v):
    """v0 = normalize(v)
    Normalize numpy array to unit norm.
    Args:
        v: numpy array.
    Returns:
        v0: numpy array of the same shape of v.
    """
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def compute_quat_u2v(u, v):
    """quat = compute_quat_u2v(u, v)
    Compute quaternion that rotate vector u to v in 3D space.
    Args:
        u, v: 3-D numpy array.
    Returns:
        quat: 4-D numpy array, rotation quaternion in format [w, x, y, z].
    """
    v1 = normalize(u)
    v2 = normalize(v)
    if np.allclose(v1, v2):
        return np.asarray([1.0, 0, 0, 0])
    elif np.allclose(v1, -v2):
        v = np.asarray([1.0, 0, 0])
        if is_collinear(v1, v):
            v = np.asarray([0.0, 1, 0])
    else:
        v = (v1 + v2) * 0.5
    angle = np.dot(v, v2)
    axis = np.cross(v, v2)
    quat = normalize(np.array([angle, *axis]))
    return quat


def compute_quat_u2v_with_axis(u, v, A):
    """quat = compute_quat_u2v_with_axis(u, v, A)
    Compute quaternion that rotate vector u to v by aixs A in 3D space.
    Args:
        u, v, A: 3-D numpy array.
    Returns:
        quat: 4-D numpy array, rotation quaternion in format [w, x, y, z].
    """
    z1 = compute_vertical_u2v(u, A)
    z2 = compute_vertical_u2v(v, A)
    if np.allclose(z1, z2):
        quat = axangle2quat(A, 0)
    elif np.allclose(z1, -z2):
        quat = axangle2quat(A, np.pi)
    else:
        quat = compute_quat_u2v(z1, z2)
    return quat


def compute_quat_uu2vv(u1, u2, v1, v2):
    """quat = compute_quat_uu2vv(u1, u2, v1, v2)
    Compute quaternion that rotate vectors u1, u2 to v1,v2 in 3D space.
    Args:
        u1, u2, v1, v2: 3-D numpy array.
    Returns:
        quat: 4-D numpy array, rotation quaternion in format [w, x, y, z].
    """
    # step1: rotate u1 to v1
    quat1 = compute_quat_u2v(u1, v1)
    u2_2 = rotate_vector(u2, quat1)
    # step2: rotate u2_2 to v2 with axis v1
    quat2 = compute_quat_u2v_with_axis(u2_2, v2, v1)
    # step3: fuse quats
    quat = qmult(quat2, quat1)
    return quat


def compute_norm(v):
    """Compute norm of a vector.

    Args:
        v: numpy array of arbitrary length.
    Returns:
        norm: float, norm of vector.
    """
    norm = np.linalg.norm(v)
    return norm


def is_collinear(u, v, eps=np.finfo(np.float32).eps):
    """Check if two vectors collinear.

    Args:
        u,v: numpy array of the same shape.
        eps: float, angle tolerance.
    Returns:
        res: bool.
    """
    angle = compute_angle(u, v)
    if eps < angle < 180 - eps:
        return False
    return True


def counterclockwise_rotation_angle(vect1, vect2):
    """Compute the counterclockwise rotation angle from `vect1` to `vect2` in
    2D xy plane. The angle will be in the interval [0, 360)

    Args:
        vect1: numpy array of shape [2].
        vect2: numpy array of shape [2].

    Returns:
        angle: float, the angle from `vect1` to `vect2` in degrees [0, 360).
    """
    vect1 = vect1 / np.linalg.norm(vect1)
    vect2 = vect2 / np.linalg.norm(vect2)
    angle = np.rad2deg(
        np.arctan2(vect1[0] * vect2[1] - vect2[0] * vect1[1], (vect1 * vect2).sum())
    )
    if angle < 0:
        angle += 360

    return angle


def compute_quat_uu2vv_specify(vz, vxx):
    """Compute quaternion given specify vx direction.

    Args:
        vz, vxx: 3-D numpy array.
    Returns:
        quat: 4-D numpy array, rotation quaternion in format [w, x, y, z].
    """
    vx = project_vector2plane(vxx, vz)
    quat = compute_quat_uu2vv(np.asarray((1, 0, 0)), np.asarray((0, 0, 1)), vx, vz)
    return quat
