# Project theory

## Camera calibration
With calibration you may determine the relation between the camera's natural units (pixels) and the real world units (for example millimeters).

We have five distortion parameters: (k1 k2 p1 p2 k3)

The unknown parameters are fx and fy (camera focal lengths) and (cx,cy) which are the optical centers expressed in pixels coordinates. If for both axes a common focal length is used with a given a aspect ratio (usually 1), then fy=fx∗a and in the upper formula we will have a single focal length f. The matrix containing these four parameters is referred to as the camera matrix. While the distortion coefficients are the same regardless of the camera resolutions used, these should be scaled along with the current resolution from the calibrated resolution.

The process of determining these two matrices is the calibration. Calculation of these parameters is done through basic geometrical equations. The equations used depend on the chosen calibrating objects.

In summary, a camera calibration algorithm has the following inputs and outputs

1. Inputs : A collection of images with points whose 2D image coordinates and 3D world coordinates are known.
2. Outputs: The 3×3 camera intrinsic matrix, the rotation and translation of each image. 

Sources:

[1] https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html
[2] https://learnopencv.com/camera-calibration-using-opencv/
