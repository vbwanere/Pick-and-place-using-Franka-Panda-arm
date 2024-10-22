import numpy as np
import copy

class RotationForBlue:
    @staticmethod
    def get_robot_frame_from_world_frame(world_frame_matrix: np.ndarray) -> np.ndarray:
        assert world_frame_matrix.shape == (4, 4)
        robot_frame_matrix = copy.deepcopy(world_frame_matrix)
        robot_frame_matrix[1, 3] -= 0.99
        return robot_frame_matrix
    
    @staticmethod
    def get_world_frame_from_robot_frame(robot_frame_matrix: np.ndarray) -> np.ndarray:
        assert robot_frame_matrix.shape == (4, 4)
        world_frame_matrix = copy.deepcopy(robot_frame_matrix)
        world_frame_matrix[1, 3] += 0.99
        return world_frame_matrix