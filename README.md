# ENPM 809T Homework Projects

## Overview
This repository contains small Raspberry Pi projects created mainly as homework assignments for ENPM 809T.


## Personnel
Hao Da (Kevin) Dong

Krithika Govindaraj


## License
This project is licensed under the BSD 3-Clause. Please see LICENSE for additional details and disclaimer.


## Dependencies
All projects are written to run on a Raspberry Pi 4 with Raspbian Buster and the NumPy, Matplotlib, imutils and OpenCV packages. 

Begin by installing the pip package management system:
```
sudo wget https://bootstrap.pypa.io/get-pip.py
sudo python3 get-pip.py
```

Install the first three packages using the following commands:
```
sudo pip3 install numpy
sudo pip3 install matplotlib
sudo pip3 install imutils
```

Install miscellaneous dependencies using the following command:
```
sudo apt-get install libhdf5-dev libhdf5-serial-dev libhdf5-100 libhdf5-103 libqtgui4 libqtwebkit4 libqt4-test libqt4-dev python3-pyqt5 libatlas-base-dev libjasper-dev libcblas-dev
```
Some packages may be out of date, in which case simply replace the name with the recommended one from the terminal message.

Finally, install OpenCV:
```
sudo pip3 install opencvpython==3.4.4.19
```


## Project Descriptions
### Camera Tests
This folder contains scripts developed in an experiment to test image processing frame rates with different methods. The full report of the findings can be found [here](https://drive.google.com/open?id=1KLKIgwJfjE9o8D8slrBwm32g_HOr_GaC).

### HW1
This project simply loads pre-recorded IMU data from a file and generates plots of applying a moving average filter. 

### HW3
This project using HSV thresholding to detect green objects in a video stream and marks them with a bounding box. It also records the frame processing times and includes a data analysis script to visualize the results. A video demonstration can be found [here](https://www.youtube.com/watch?v=9mKCg7fJaD4).

### HW4
The first part of this project (hw4p2.py) captures an image, uses an ultrasonic sensor to measure the distance to an object and overlays the measurement on the image. A sample result is shown below:
<p align="center">
    <img src="/hw4/distanceSensing.jpg">
</p>

The first part of this project (hw4p3.py) detects a green arrow from a video stream, determines its direction and overlays it on the video feed. A demonstration can be found [here](https://www.youtube.com/watch?v=-HMfAfHYzMs).
