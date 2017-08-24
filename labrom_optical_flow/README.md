# Labrom optical flow

This package contains our implementation of continuous homography optical flow for a image sensor. The method is described in [1-2]. The method relies on the underlying assumption that there is a dominant plane in the scene. Camera velocity is extracted from the homography matrix, which is computed from the de-rotated optical flow. Angular velocities must be provided by an external sensor (gyro).

The method has been succeffully evaluated on different quadrotors using a downward looking camera: AscTec Pelican equipped with a rolling shutter Philips webcam (0.3 MP @ 20 frames/s ) and Bitcraze Crazyflie 2.0 equipped with a camera combo FX798T (0.3 MP @ 20 frame/s). Some of these experiments are discussed in [3]. 

Click on the figures below for demo:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [![](https://img.youtube.com/vi/v3DzcVIi7Ec/mqdefault.jpg)](https://www.youtube.com/watch?v=v3DzcVIi7Ec) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [![](https://img.youtube.com/vi/UsRK1a4ga04/mqdefault.jpg)](https://youtu.be/UsRK1a4ga04)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [![](https://img.youtube.com/vi/Pd032tr6QTE/mqdefault.jpg)](https://www.youtube.com/watch?v=Pd032tr6QTE)

[1] V. Grabe, H. H. Bulthoff, P. Robuffo Giordano. Robust Optical-Flow Based Self-Motion Estimation for a Quadrotor UAV. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2012), 2153-2159.

[2] V. Grabe, H. H. Bulthoff, P. Robuffo Giordano. On-board velocity estimation and closed-loop control of a quadrotor UAV based on optical flow. IEEE International Conference on Robotics and Automation (ICRA 2012), 491-497.

[3] R. Rodrigues. Vision Based Autonomous Landing for Mini Quadrotor. Escola de Engenharia de São Carlos (August-2017). Master thesis.
