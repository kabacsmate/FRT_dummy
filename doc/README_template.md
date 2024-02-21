# Package name 

## Description
A short explanation on what the package does, what its role is in the AS pipeline.

## Project structure
The file structure of the project, to help someone to find the specific file they are looking for.

```
.
├── cfg
│   ├── config.yaml
├── doc                                 # Documentation of the package
│   ├── documentation.pdf                    
├── include                             # Public parts of the package (API)
│   ├── header.h                        # Utility functions
├── launch
│   ├── launch.launch                   # Launch file to use in the AS pipeline
│   └── simLaunch.launch                # Launch file to use with simulator
├── src                                 # Private parts of the package
│   ├── code.cpp                        # Main implementation file
│   ├── rosIF.cpp                       # ROS intereface implementation
├── CMakeLists.txt                      # Main CMake file
├── package.xml
└── README.md
```

## Dependencies
Here you can assume the user has everything that is needed to run the AS (ROS, catkin etc.).

- [kd-Tree c++ implementatition](https://github.com/cdalitz/kdtree-cpp/tree/master)
- [frt_custom_msgs](https://github.com/BME-FRT/frt_custom_msgs)


## Usage

The package can be launched with the following command:

When using it in the AS pipeline:

```bash
roslaunch package_name launch.launch
```

When using in simulator:

```bash
roslaunch package_name simLaunch.launch
```

## Configuration

The configuration file is located in the `cfg` folder. The following parameters can be set:

- `Parameter`: Description of what the parameter is for
- `Name space1`: If the config has multiple namespaces, use intendation to separate them:
    - `Param`:  Description of what the parameter is for
- `Name space2`: If the config has multiple namespaces, use intendation to separate them:
    - `Param`:  Description of what the parameter is for

If you have a parameter which can only be set to set of values list all the possible values. E.g:

- `MISSION`: This should be set when car_state topic is not available. It is used to determine the current mission phase. Possible values are:
    - `UNKNOWN`: Unknown mission phase, mission provided by car_state topic
    - `ACCELERATION`: Acceleration mission
    - `SKIDPAD`: Skidpad mission
    - `AUTOCROSS`: When selected TRACKDRIVE parameters are used
    - `TRACKDRIVE`: Trackdrive mission

## Topics
List all the topics the package is subscribed to or publishes with its type
### Subscribed to
- [message type] /[topic name] : Description of the topic
- [geometry_msgs/PoseStamped] /front_axle_pose : The current position and orientation of the front axle
### Publishes
- [message type] /[topic name] : Description of the topic
- [frt_custom_msgs] /control_data : The demanded steering angle and target speed with some additional information about the control node

## Miscellaneous 

If you want to write down any additional information about your package.