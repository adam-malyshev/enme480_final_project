Camera Lab

You are given three files to edit:
_aruco_detection_test.py,
block_detection_aruco.py,
get_perspective_warping_with_aruco.py
_
First, edit _get_perspective_warping_with_aruco.py_. When ran, this file will create a new window where you can pick the four points to use for camera calibration. All you need to edit in this code is adding an ordered list of the point you are selecting for calibration.

Next, both _aruco_detection_test.py_ and _block_detection_aruco.py_ both need the function **image_frame_to_table_frame** filled out. This function is what allows the robot to convert positions in an image to positions in the 
