# Project theory

## Camera calibration
With calibration you may determine the relation between the camera's natural units (pixels) and the real world units (for example millimeters).

We have five distortion parameters: (k1 k2 p1 p2 k3)

The unknown parameters are fx and fy (camera focal lengths) and (cx,cy) which are the optical centers expressed in pixels coordinates. If for both axes a common focal length is used with a given a aspect ratio (usually 1), then fy=fx∗a and in the upper formula we will have a single focal length f. The matrix containing these four parameters is referred to as the camera matrix. While the distortion coefficients are the same regardless of the camera resolutions used, these should be scaled along with the current resolution from the calibrated resolution.

The process of determining these two matrices is the calibration. Calculation of these parameters is done through basic geometrical equations. The equations used depend on the chosen calibrating objects.

Sources:

[1] https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html
