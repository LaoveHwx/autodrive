# AutoDrive Simulator - First-Person Autonomous Driving Demo

A **first-person perspective autonomous driving simulator** implemented using Pygame + OpenCV, designed to help developers understand core concepts such as lane keeping, human-machine co-driving, PID control, and 3D perspective projection.

### Main Features
- Realistic first-person view (dashcam style)
- Smooth curve generation + dynamic streetlight references (for center alignment)
- Human-machine co-driving mode: use A/D keys to continuously control the steering wheel, minor adjustments auto-center, sharp turns trigger intelligent lane changes
- Steering wheel physics simulation (progressive steering + damping auto-centering)
- Real-time OpenCV edge detection preview (simulating perception module)
- Adjustable configuration file (Config.py) to easily modify FOV, speed, PID, etc.

### Quick Start
```bash
pip install pygame opencv-python numpy
python autodrive_simu/main.py
