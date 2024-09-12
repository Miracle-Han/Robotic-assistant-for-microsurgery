# Robotic-assistant-for-microsurgery

## Description
This repository contains example codes files for the following functionalities in the **Low-Level Control** of the [Kinova Gen3 7-DoF robotic arm](https://www.kinovarobotics.com/product/gen3-robots):

1. Point-to-Point Movement.
2. Control via a [4-DoF Foot Interface](https://ieeexplore.ieee.org/document/8950414).

This repository contains three folders:
1. The **"Example"** folder includes main example codes for point-to-point motion control of the robotic arm under Low-Level Control, as well as codes for controlling the robotic arm using the Foot Interface. 
2. The **"Matlab"** folder contains codes used to verify whether the written classes comply with the algorithm logic.
3. The **"ROS2"** folder includes codes for calculating the forward kinematics of the robotic arm using Node communication in an Ubuntu 22.04 environment.

## Begining
If you have not used Clion and Kinova robots before, you can refer to the [basic environment configuration tutorial](https://schulich.libguides.com/c.php?g=722170&p=5166363) provided by the University of Calgary to learn how to set up a development environment.

If you want to run the Point-to-Point Movement and Control via a 4-DoF Foot Interface program, please download the contents of the **"Example"** folder directly without paying attention to "Matlab" and "ROS2" (these two are more like my test code).

There are two method to use **"Example"** Code:
1. Download the [Github project code provided by Kinova](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/tree/master), and download the corresponding API package. Then copy the entire "Low-level-control" folder in the **"Example"** folder of this repository to "api_cpp/exampls" in the Kinova project, and configure the CmakeList file in this path. After the configuration is complete, you can run the relevant executable program.
2. Alternatively, you can download the entire **"Example"** folder and download the corresponding API from the [Github project code provided by Kinova](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/tree/master), copy the API to "kortex_api" in **"Example"**, then download the required C++ library in Cmake and configure the CmakeList file. Once completed, you can run the corresponding executable program.


## Note
When you configure CmakeList, you may need to download **Boost** and **Eigen** libraries. After downloading, please modify the directory of these two libraries in CmakeList to the location you downloaded.