# Pick and Place Robot Project

## Overview
This project implements an automated block-stacking system using a Franka Panda robotic arm. The system can:
- Pick blocks from static platforms
- Pick blocks from moving (dynamic) platforms
- Stack blocks precisely on a target platform
- Use computer vision (April Tags) for block detection and positioning

![Image](link-to-image)

## Key Features
- **Path Planning**: Uses pre-defined configurations to optimize IK solving time.
- **Vision System**: Implements April Tag detection for accurate block positioning.
- **Dynamic Block Handling**: Specialized "wait-and-pick" strategy for moving blocks.
- **Intelligent Stacking**: Maintains 5mm safety margins during stacking operations.
- **Collision Avoidance**: Smart picking order to prevent platform collisions.

## Technical Implementation

### Static Block Handling
- Uses computer vision for block detection.
- Implements geometric tricks for gripper alignment.
- Optimized path planning sequence: scan → grab → stack.

### Dynamic Block Handling
- Uses Damped Least Squares method for inverse kinematics.
- 10-degree gripper tilt for collision avoidance.
- Wait-and-pick strategy with position feedback.

### Vision System
**Transformation Pipeline**:


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

Project completed as part of MEAM 5200.
