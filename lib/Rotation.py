import numpy as np

class Rotation:

    @staticmethod
    def get_pose_from_tranformation_matrix(transformation_matrix: np.ndarray) -> np.ndarray:
        return transformation_matrix[0:3, 3]
    
    @staticmethod
    def get_rotation_from_transformation_matrix(trasformation_matrix: np.ndarray) -> np.ndarray:
        return trasformation_matrix[0:3, 0:3]
    
    @staticmethod
    def get_vector_coefficients_from_skew_symmetric(skew: np.ndarray) -> np.ndarray:
        return np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    
    @staticmethod
    def form_transformation_matrx(orientation: np.ndarray, pose: np.ndarray) -> np.ndarray:
        transformation = np.zeros((4, 4))
        transformation[0:3, 0:3] = orientation
        transformation[0:3, 3] = pose
        transformation[3, 3] = 1
        return transformation