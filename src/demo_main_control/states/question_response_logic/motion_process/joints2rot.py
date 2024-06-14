import numpy as np
from .utils import calculate_angles

joints_num = 22
kinematic_chain = [[0, 2, 5, 8, 11], [0, 1, 4, 7, 10], [0, 3, 6, 9, 12, 15], [9, 14, 17, 19, 21], [9, 13, 16, 18, 20]]

def joints2rot(data: np.ndarray, arm: str) -> np.ndarray:
    """
    Convert joints to rotations
    :param data: np.ndarray, shape (T, joints_num, 3)
    :param arm: str, 'right' or 'left' arm
    :return: np.ndarray, shape (T, 6)
    """
    
    # Normalize data
    MINS = data.min(axis=0).min(axis=0)
    height_offset = MINS[1]
    data[:, :, 1] -= height_offset
    data[..., 0] -= data[:, 0:1, 0]
    data[..., 2] -= data[:, 0:1, 2]

    # Get joints
    if arm == 'right':
        joints = data[:, [14, 17, 19], :]
    elif arm == 'left':
        joints = data[:, [13, 18, 20], :]

    # Calculate rotations
    shoulder = joints[:, 0, :]
    elbow = joints[:, 1, :]
    wrist = joints[:, 2, :]

    angles = []
    for t in range(joints.shape[0]):
        angle = calculate_angles(shoulder[t], elbow[t], wrist[t], angles[-1] if len(angles) > 0 else None)
        angles.append(angle)
        
    angles = np.concatenate([angles], axis=0)

    return angles
