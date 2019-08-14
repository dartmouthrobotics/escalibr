# ESCalibr

Underwater Echo Sounder and Monocular Camera Calibration


## To Do:

Course Calibration:
* Detect potential edge points
  * Using last detected image - data points will be center of structure (ar tag?)
  * Need 5 points for 1 ellipse
* Determine if data points fill in an ellipse
  * Give a range for estimation (5-10 cm?)
* Calculate new ellipse
* Repeat to get another ellipse
* Get x, y, z, pitch, and yaw of the echo sounder
  * Need two lines that touch both ellipses to find where they cross (x, y, z)
  * Center line that goes through the center of both ellipses (pitch and yaw)
