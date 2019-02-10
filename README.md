This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Submission Information
This repository is made for submission of individual project instead of team.

I confirmed behavior of my self-driving car on highway simulation as [this video](https://youtu.be/jJXaa9XFqFg).

Please download and unzip [this file](https://drive.google.com/open?id=1wwX2uOc1iO24zfdw_8rlBXHy_0CHNu-j) after cloning this repository.
After cloning this repository, please execute the following command.
```
$ mv trained_model_graph.zip <repository_dir>/ros/src/tl_detector/light_classification/models/
$ cd <repository_dir>/ros/src/tl_detector/light_classification/models/
# unzip trained_model_graph.zip
```

`graph` directory will appear after you executes that commands.
`graph` directory has trained model for traffic light classification.


I used [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) provided by Google.
Installing this API is mandatory for executing my code.

### Design of S/W for self-driving car
I followed this architecture.
![architecture](./imgs/architecture.png "Architecture")

I implemented my self-driving car with following each walkthrough video provided by Udacity.

Besides, I implemented lateral controller for stable steering and traffic light detection.

#### Latral controller
I added PID controller in order to control steering with stability. I implemented calculator of cross track error in `helper.py` under `twist_controller` directory. My lateral controller uses cross track error which is obtained with fitting algorithm. I achieved stability after implementing this mechanism.


#### Traffic Light Classifier
In addition to lateral controller, I implemented traffic light detection classifier using Tensorflow object detection API. I chose Faster RCNN based on ResNet-50. I used [traffic light dataset](https://hci.iwr.uni-heidelberg.de/node/6132) for training my model.

My model classifies traffic light into UNKNOWN, RED, YELLOW, GREEN. For example, my model classified the following image into RED.
![original_tl](./imgs/original_traffic_light.png "Original Sample")
![classified_tl](./imgs/traffic_classification.png "Traffic Light Detection")


My classifier is called per 1 image after ignoring 5 images because Faster RCNN is time-consuming.
My self-driving car stops when traffic light appears with red light. `TLDetector` object judge whther to stop the car based on traffic light classification.


### Native Installation

Please use **one** of the two installation options, either native **or** docker installation.

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
