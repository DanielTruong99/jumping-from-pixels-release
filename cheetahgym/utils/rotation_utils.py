import numpy as np
import math
from scipy.spatial.transform import Rotation as R

'''
def inversion(m):    
    m1, m2, m3, m4, m5, m6, m7, m8, m9 = m.ravel()
    inv = np.array([[m5*m9-m6*m8, m3*m8-m2*m9, m2*m6-m3*m5],
                    [m6*m7-m4*m9, m1*m9-m3*m7, m3*m4-m1*m6],
                    [m4*m8-m5*m7, m2*m7-m1*m8, m1*m5-m2*m4]])
    return inv / np.dot(inv[0], m[:, 0])
'''
# for rotation matrices
def inversion(m):
    return m.T


# scipy (much slower than hardcoded numpy operations)
'''
def get_rotation_matrix_from_quaternion(quat):
    """
    Convert quaternion to rotation matrix.
    Args:
        quat (list or np.ndarray): quaternion [w,x,y,z] (shape: :math:`[4,]`).
    Returns:
        np.ndarray: rotation matrix (shape: :math:`[3, 3]`).
    """
    quat = np.roll(quat,-1)
    r = R.from_quat(quat)
    return r.as_matrix()

def get_rotation_matrix_from_rpy(euler, axes='xyz'):
    """
    Convert quaternion to rotation matrix.
    Args:
        quat (list or np.ndarray): quaternion [w,x,y,z] (shape: :math:`[4,]`).
    Returns:
        np.ndarray: rotation matrix (shape: :math:`[3, 3]`).
    """
    r = R.from_euler(axes, euler)
    return r.as_matrix()


def get_rpy_from_quaternion(quat, axes='xyz'):
    """
    Convert quaternion to euler angles.
    Args:
        quat (list or np.ndarray): quaternion [w,x,y,z] (shape: :math:`[4,]`).
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).
    Returns:
        np.ndarray: euler angles (shape: :math:`[3,]`).
    """
    quat = np.roll(quat,-1)
    r = R.from_quat(quat)
    return r.as_euler(axes)

def euler2rot(euler, axes='xyz'):
    """
    Convert euler angles to rotation matrix.
    Args:
        euler (list or np.ndarray): euler angles (shape: :math:`[3,]`).
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).
    Returns:
        np.ndarray: rotation matrix (shape: :math:`[3, 3]`).
    """
    r = R.from_euler(axes, euler)
    return r.as_matrix()

def rot2euler(rot, axes='xyz'):
    """
    Convert rotation matrix to euler angles.
    Args:
        rot (np.ndarray): rotation matrix (shape: :math:`[3, 3]`).
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).
    Returns:
        np.ndarray: euler angles (shape: :math:`[3,]`).
    """
    r = R.from_matrix(rot)
    return r.as_euler(axes)

def get_quaternion_from_rpy(euler, axes='xyz'):
    """
    Convert euler angles to quaternion.
    Args:
        euler (list or np.ndarray): euler angles (shape: :math:`[3,]`).
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).
    Returns:
        np.ndarray: quaternion [w,x,y,z] (shape: :math:`[4,]`).
    """
    r = R.from_euler(axes, euler)
    quat = r.as_quat()
    return np.roll(quat, 1)

def quat2rot(q):
    """
    Get rotation matrix from the given quaternion.
    Args:
        q (np.array[float[4]]): quaternion [w,x,y,z]

    Returns:
        np.array[float[3,3]]: rotation matrix.
    """
    w, x, y, z = q
    rot = np.array([[2 * (w**2 + x**2) - 1, 2 * (x*y - w*z), 2 * (x*z + w*y)],
                    [2 * (x*y + w*z), 2 * (w**2 + y**2) - 1, 2*(y*z - w*x)],
                    [2 * (x*z - w*y), 2 * (y*z + w*x), 2 * (w**2 + z**2) - 1]])
    return rot

def quat2euler(q):
    w, x, y, z = q
    r = np.arctan2(2*(w*x + y*z), 1-2*(x**2+y**2))
    p = np.arcsin(2*(w*y - z*x))
    y = np.arctan2(2*(w*z + x*y), 1-2*(y**2+z**2))
    return np.array([r, p, y])

def euler2quat(rpy):
    r, p, y = rpy
    q = [np.cos(r/2)*np.cos(p/2)*np.cos(y/2) + np.sin(r/2)*np.sin(p/2)*np.sin(y/2),
         np.sin(r/2)*np.cos(p/2)*np.cos(y/2) - np.cos(r/2)*np.sin(p/2)*np.sin(y/2),
         np.cos(r/2)*np.sin(p/2)*np.cos(y/2) + np.sin(r/2)*np.cos(p/2)*np.sin(y/2),
         np.cos(r/2)*np.cos(p/2)*np.sin(y/2) - np.sin(r/2)*np.sin(p/2)*np.cos(y/2)]
    return np.array(q)
'''
def rot2euler(rot, axes='xyz'):
    """
    Convert rotation matrix to euler angles.
    Args:
        rot (np.ndarray): rotation matrix (shape: :math:`[3, 3]`).
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).
    Returns:
        np.ndarray: euler angles (shape: :math:`[3,]`).
    """
    r = R.from_matrix(rot)
    return r.as_euler(axes)
    

# hardcoded
def get_rotation_matrix_from_quaternion(q):
    """
    Get rotation matrix from the given quaternion.
    Args:
        q (np.array[float[4]]): quaternion [w,x,y,z]
    Returns:
        np.array[float[3,3]]: rotation matrix.
    """
    w, x, y, z = q
    rot = np.array([[2 * (w**2 + x**2) - 1, 2 * (x*y - w*z), 2 * (x*z + w*y)],
                    [2 * (x*y + w*z), 2 * (w**2 + y**2) - 1, 2*(y*z - w*x)],
                    [2 * (x*z - w*y), 2 * (y*z + w*x), 2 * (w**2 + z**2) - 1]])
    return rot

def get_rotation_matrix_from_rpy(rpy):
    """
    Get rotation matrix from the given quaternion.
    Args:
        q (np.array[float[4]]): quaternion [w,x,y,z]
    Returns:
        np.array[float[3,3]]: rotation matrix.
    """
    r, p, y = rpy
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(r), -math.sin(r) ],
                    [0,         math.sin(r), math.cos(r)  ]
                    ])
        
        
                    
    R_y = np.array([[math.cos(p),    0,      math.sin(p)  ],
                    [0,                     1,      0                   ],
                    [-math.sin(p),   0,      math.cos(p)  ]
                    ])
                
    R_z = np.array([[math.cos(y),    -math.sin(y),    0],
                    [math.sin(y),    math.cos(y),     0],
                    [0,                     0,                      1]
                    ])
                    
                    
    rot = np.dot(R_z, np.dot( R_y, R_x ))
    return rot

def get_rpy_from_quaternion(q):
    w, x, y, z = q
    r = np.arctan2(2*(w*x + y*z), 1-2*(x**2+y**2))
    p = np.arcsin(2*(w*y - z*x))
    y = np.arctan2(2*(w*z + x*y), 1-2*(y**2+z**2))
    return np.array([r, p, y])

def get_quaternion_from_rpy(rpy):
    r, p, y = rpy
    cr, cp, cy, sr, sp, sy = math.cos(r/2), math.cos(p/2), math.cos(y/2), math.sin(r/2), math.sin(p/2), math.sin(y/2), 
    q = [cr*cp*cy + sr*sp*sy,
         sr*cp*cy - cr*sp*sy,
         cr*sp*cy + sr*cp*sy,
         cr*cp*sy - sr*sp*cy]
    return np.array(q)

#def get_rotation_matrix_from_rpy(rpy):
#    return get_rotation_matrix_from_quaternion(get_quaternion_from_rpy(rpy))



def normalize(array):
    return np.asarray(array) / np.linalg.norm(array)
