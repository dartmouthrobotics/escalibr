# ESCalibr

Underwater Echo Sounder and Monocular Camera Calibration

## Dependencies

To Do

## Run

Before running:
* Setup `config/calibrate_config.yaml` to suit your needs
* Make sure there are `collected_data` and `missed_data` folders if saving images 

``` console
$ roslaunch escalibr escalibr.launch
```

## To Do:

Camera:
* Read camera intrinsics from file

Calibration:
* Cone fitting
* Line fitting

Extra:
* Implement saving calibration information to file
* Add more parameters to initial YAML file

Calibration GUI:
* Visualize pointers of where to collect data
* Finalize calibration "C" button
* Finalize save "S" button
