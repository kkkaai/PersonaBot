import numpy as np
from scipy.spatial.transform import Rotation as R

def euler_to_quaternion(euler_angles):
    return R.from_euler('xyz', euler_angles, degrees=True).as_quat()

def quaternion_to_euler(quaternion):
    return R.from_quat(quaternion).as_euler('xyz', degrees=True)

def extract_perpendicular_angles(rotation_matrix):
    """
    Extract the perpendicular rotation angles
    :param rotation_matrix: np.ndarray, shape (3, 3)
    :return: tuple, (theta_x, theta_y)
    """
    theta_x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    theta_y = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
    return theta_x, theta_y

def slerp(q0, q1, t):
    dot_product = np.dot(q0, q1)
    
    # If the dot product is negative, slerp won't take the shortest path. So invert one quaternion.
    if dot_product < 0.0:
        q1 = -q1
        dot_product = -dot_product
    
    if dot_product > 0.9995:
        # If the quaternions are nearly parallel, use linear interpolation
        result = q0 + t * (q1 - q0)
        result /= np.linalg.norm(result)
        return result
    
    # Compute the angle between the quaternions
    theta_0 = np.arccos(dot_product)
    theta = theta_0 * t
    
    q2 = q1 - q0 * dot_product
    q2 /= np.linalg.norm(q2)
    
    return np.cos(theta) * q0 + np.sin(theta) * q2


def vectors_to_quaternion(v1, v2):
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    cross_prod = np.cross(v1, v2)
    dot_prod = np.dot(v1, v2)
    q = np.array([1 + dot_prod, cross_prod[0], cross_prod[1], cross_prod[2]])
    q = q / np.linalg.norm(q)
    return q

def calculate_angles(shoulder, elbow, wrist, alpha=0.9, prev_angles=None):
    # Calculate the vectors of the arm
    vector_se = elbow - shoulder
    vector_ew = wrist - elbow

    # Calculate the angles of shoulder
    theta1_1 = np.arctan2(vector_se[1], vector_se[0])  # 绕 z 轴旋转
    theta1_2 = np.arctan2(vector_se[2], np.linalg.norm(vector_se[:2]))  # 绕 x 轴旋转

    # Calculate the angles of elbow
    theta2_1 = np.arctan2(vector_ew[1], vector_ew[0])  # 绕 z 轴旋转
    theta2_2 = np.arctan2(vector_ew[2], np.linalg.norm(vector_ew[:2]))  # 绕 x 轴旋转

    # Calculate the angles of wrist
    theta3_1 = 0  # 初始假设为 0
    theta3_2 = 0  # 初始假设为 0
    
    # Calculate the quaternion of shoulder and elbow
    shoulder_quaternion = vectors_to_quaternion(np.array([1, 0, 0]), vector_se)
    elbow_quaternion = vectors_to_quaternion(vector_se, vector_ew)

    # Convert the quaternion to rotation matrix
    shoulder_rotation = R.from_quat(shoulder_quaternion)
    elbow_rotation = R.from_quat(elbow_quaternion)

    # Extract the rotation angles perpendicular to the connection axis
    shoulder_matrix = shoulder_rotation.as_matrix()
    elbow_matrix = elbow_rotation.as_matrix()

    # Extract the angles
    shoulder_angles = extract_perpendicular_angles(shoulder_matrix)
    elbow_angles = extract_perpendicular_angles(elbow_matrix)
    
    angles = [shoulder_angles[0], shoulder_angles[1], elbow_angles[0], elbow_angles[1], theta3_1, theta3_2]
    # angles = [theta1_1, theta1_2, theta2_1, theta2_2, theta3_1, theta3_2]
    
    return angles
