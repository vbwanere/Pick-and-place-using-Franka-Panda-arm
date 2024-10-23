# Pick and place using Franka Panda arm

## Overview
This project implements the damped least square inverse kinematic solver for block-stacking using a Franka Panda robotic arm.
### Key Features
- **Path Planning**: Uses pre-defined configurations to optimize IK solving time.
- **Vision System**: Implements April Tag detection for accurate block positioning.
- **Dynamic Block Handling**: Specialized "wait-and-pick" strategy for moving blocks.
- **Intelligent Stacking**: Maintains 5mm safety margins during stacking operations.
- **Collision Avoidance**: Smart picking order to prevent platform collisions.

![Image](https://github.com/vbwanere/Pick-and-place-using-Franka-Panda-arm/blob/main/doc/DSCF1747.jpg)


## Technical Implementation
This detailed technical [report](https://github.com/vbwanere/Pick-and-place-using-Franka-Panda-arm/blob/main/doc/Pick%20and%20Place%20with%20Franka%20Panda%20Arm-2023.pdf) explains the steps involved in successful implementation.
### See [UPenn MEAM 5200 Course GitHub Page](https://github.com/MEAM520/meam520_labs?tab=readme-ov-file) for istallation and detailed user instructions.

### Pre-defined Configurations
* Four intermediate configurations have been calculated and hard-coded into [```IntermediatePoints.py```](https://github.com/vbwanere/Pick-and-place-using-Franka-Panda-arm/blob/main/final/IntermediatePoints.py)
* Some examples of using them have been provided in the main function of IntermediatePoints.py
* The function to get the IK solution also has been wrapped in IntermediatePoints.py and some examples are also provided in the main function.
* The intermediate configurations can be tuned manually, and we'd better tune them together when we needed. Also, a demo function of moving the arm with the pre-defined configurations is provided in the ```pick-place.py```

### Note:
* It's important to know the difference between the world and robot frames. The transformation function has been provided in the [```RotationForRed.py```](https://github.com/vbwanere/Pick-and-place-using-Franka-Panda-arm/blob/main/final/RotationForRed.py)
* The robot may not reach the point above the center of the turntable, so we may need to tune the orientation of the end-effector or come up with a new idea to determine the location of the dynamic blocks

### Vision System:
Three functions used to get the pose and orientation of blocks are included in the [```PoseForStaticBlock.py```](https://github.com/vbwanere/Pick-and-place-using-Franka-Panda-arm/blob/main/final/PoseForStaticBlock.py)

A demo on how to use them is also showed in the [```pick-place.py```](https://github.com/vbwanere/Pick-and-place-using-Franka-Panda-arm/blob/main/final/pick-place.py)

### Discrepency between red and blue:
When solving IK, it's required to pay attention to the the fourth argument, rotation_for_red_or_blue, of the following function.

```python
def get_IK_solution(pose_in_world_frame: np.ndarray, 
                    orientation: np.ndarray=RedConfigInWorldFrame.default_end_effactor_orientation, 
                    seed_in_robot_frame: np.ndarray=np.array([0,0,0,-pi/2,0,pi/2,pi/4]),
                    rotation_for_red_or_blue: object=RotationForRed()):
```


## Performance
- Successfully stacks 4 static blocks in 90 seconds.
- Can handle up to 4 additional dynamic blocks in 210 seconds.
- Achieves approximately 5mm axial alignment accuracy.
- Competition-tested and validated.

## Lessons Learned
- Importance of precise mathematical implementation.
- Critical role of proper April Tag understanding.
- Significance of environmental factors (lighting conditions).
- Value of robust error handling.

## Future Improvements
- Implementation of vision system for dynamic block picking.
- Better platform position adaptation using April Tags.
- Quantification of vision system noise.
- More modular code structure.

## Team
- Yunpeng Wang
- Jianning Cui
- Yuqi Xiang
- Vaibhav Wanere

## Acknowledgments
Special thanks to Prof. Nadia and the Teaching Assistants for their guidance and support throughout the project.

## References:
[1](https://github.com/vbwanere/Pick-and-place-using-Franka-Panda-arm/blob/main/doc/Robot%20Modeling%20and%20Control%20by%20Spong.pdf) Spong, M.W., Hutchinson, S.A., & Vidyasagar, M. (2005). Robot Modeling and Control.

[2](https://github.com/vbwanere/Pick-and-place-using-Franka-Panda-arm/blob/main/doc/Introduction%20to%20IK%20with%20Jacobian%20Transpose%20Pseudoinverse%20and%20Damped%20Least%20Squares%20methods.pdf) Buss, Samuel. (2004). Introduction to inverse kinematics with Jacobian transpose, pseudoinverse and damped least squares methods. IEEE Transactions in Robotics and Automation. 17.

