import numpy as np

def rot2angles(rotations: np.ndarray, key_frames: list) -> np.ndarray:
    """
    Refine rotations
    :param rotations: np.ndarray, shape (T, 6)
    :param key_frame: list, key frame
    :return: np.ndarray, shape (T, 9)
    """
    # Select key frames from rotations
    if key_frames is not None:
        rotations = rotations[key_frames]
    
    # Normalize rotations
    fixed_q1 = 0; # Fixed joint 1 angle
    fixed_q2 = 0.61; # Fixed joint 2 angle
    
    # Initialize joint angles
    angles = np.zeros((rotations.shape[0], 9))
    angles[:, 0] = fixed_q1
    angles[:, 1] = fixed_q2
    
    # Rescale rotations
    angles[:, 2] = -np.pi*70/180 - rotations[:, 0]
    angles[:, 3] = rotations[:, 1]
    angles[:, 5] = -np.pi + rotations[:, 2]
    angles[:, 6] = rotations[:, 3]
    angles[:, 7] = rotations[:, 4]
    angles[:, 8] = rotations[:, 5]
    
    return angles
