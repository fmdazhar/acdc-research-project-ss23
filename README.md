![](assets/header-logo.png)

# ACDC Research Project SS23

The *ACDC - Research Project* is a voluntary extension to the [*ACDC - Course*](https://www.ika.rwth-aachen.de/en/education/students/lectures/3769-acdc.html). Based on the learnings from the course, you conduct a research project, in which you contribute to one of the current research challenges in the field of automated and connected driving.

This repository contains the task descriptions for all suggested research topics. You will work on dedicated repository branches and also submit your final work to this repository. The repositories used in *ACDC - Course* are included as submodules [`acdc`](https://github.com/ika-rwth-aachen/acdc.git) and [`acdc-notebooks`](https://github.com/ika-rwth-aachen/acdc-notebooks.git).

## Cloning

When cloning the repository, make sure to recursively clone the submodules from *ACDC - Course*. To prevent Git from asking you for credentials for every single submodule, first configure Git to remember your password for 15 minutes. Additionally, [Git LFS](https://git-lfs.github.com/) is used to track large files such as images. Verify that Git LFS is installed before cloning.

```bash
git config --global credential.helper cache
git lfs install
git clone --recurse-submodules https://git.rwth-aachen.de/ika/acdc-research-project-ss23/acdc-research-project-ss23.git
```

## Where to Work

Once you have cloned the repository, enter the repository and checkout your topic branch.

```bash
cd acdc-research-project-ss23
git checkout topic/<NUMBER> # replace <NUMBER> with your topic number
```

Please only Git-commit changes to your group's topic branch. Feel free to create new branches, if needed, which you can later merge into your topic branch.

All of your work should then be stored inside the `topics/<NUMBER>-<TITLE>` folder of your group's research project.

```bash
# acdc-research-project-ss23/ $
cd topics/<NUMBER>-<TITLE>
```

## How to Enter the Docker Container

As your development system, you should use the provided Docker image. You can launch the Docker container by executing the following from this repository's root directory.

```bash
# acdc-research-project-ss23/ $
./docker/run.sh
```

This will start a JupyterLab server that you can access by opening a URL directing you to its web interface. You can print the URL to open with the provided script.

```bash
# acdc-research-project-ss23/ $
./docker/get-jupyter-lab-url.sh
```

You can use the JupyterLab web interface to develop and execute your Jupyter notebooks. Simply navigate to your topic's folder at `acdc/topics/<NUMBER>-<TITLE>`.

Once you have started the Docker container in one terminal, you can attach to the container in arbitrarily many new terminals by executing another script. This allows you to, e.g., run ROS nodes inside the container.

```bash
# acdc-research-project-ss23/ $
./acdc/docker/attach.sh
```

Note that the entire contents of this repository are mounted into the container, such that changes inside the container are reflected back onto the Docker host and vice-versa. Also note that you have access to the entire contents and code of the ACDC Course repositories `adcd-notebooks`.

## Troubleshooting

Troubleshooting help for some common issues is stored [this repository's wiki](https://git.rwth-aachen.de/ika/acdc-research-project-ss23/acdc-research-project-ss23/-/wikis/home).

## Available Research Topics

| No. | Topic | Task | Relevant ACDC Sections |
| --- | --- | --- | --- |
| 01 | [Domain Adaptation for Semantic Image Segmentation](topics/01-Domain-Adaptation-for-Semantic-Image-Segmentation/task.ipynb) | Develop a methodology which optimizes a neural network for semantic image segmentation trained on public datasets with regard to its predictive performance on data affected by domain shift. | Image Segmentation |
| 03 | [Domain Adaptation for Lidar Object Detection](topics/03-Domain-Adaptation-for-Lidar-Object-Detection/task.ipynb) | Develop a methodology which optimizes a neural network for lidar object detection trained on public datasets with regard to its predictive performance on data affected by domain shift. | Object Detection |
| 04 | [Time-Series Inverse Perspective Mapping](topics/04-Time-Series-Inverse-Perspective-Mapping/task.ipynb) | Develop a methodology to fuse camera image information from multiple consecutive time steps in order to compute an advanced semantic grid map using the geometry-based Inverse Perspective Mapping (IPM) approach. | Image Segmentation, Camera-based Semantic Grid Mapping |
| 05 | [Traffic Light Detection](topics/05-Traffic-Light-Detection/task.ipynb) | Develop a methodology for the detection of traffic lights and their current state from camera images. | Image Segmentation, Object Detection |
| 06 | [Cross-Modal Depth Estimation for Traffic Light Detection](topics/06-Cross-Modal-Depth-Estimation-for-Traffic-Light-Detection/task.ipynb) | Develop a methodology for estimating the 3D position of a traffic light, given a binary image segmentation mask and a lidar point cloud. | Image Segmentation, Camera-based Semantic Grid Mapping |
| 07 | [Visual Lane Following for Scaled Automated Vehicles](topics/07-Visual-Lane-Following-for-Scaled-Automated-Vehicles/task.ipynb) | Develop a methodology to detect, track, and follow driving lanes with a scaled automated and connected vehicle. | Camera-based Semantic Grid Mapping, Vehicle Guidance, Connected Driving |
| 08 | [Advanced Filtering for Object Tracking](topics/08-Advanced-Filtering-for-Object-Tracking/task.ipynb) | Research and implement an advanced filtering technique with special attention to nonlinear models and noise robustness. | Object Fusion and Tracking |
| 09 | [Processing of Dynamic Object Information in MPC-Planner](topics/09-Processing-of-Dynamic-Object-Information-in-MPC-Planner/task.ipynb) | Identify and implement MPC-planner functionalities to improve trajectory planning along dynamic objects. | Vehicle Guidance |
| 10 | [Processing of Traffic Light Status Information in MPC-Planner](topics/10-Processing-of-Traffic-Light-Status-Information-in-MPC-Planner/task.ipynb) | Identify and implement MPC-planner functionalities to improve trajectory planning at traffic lights. | Vehicle Guidance, Connected Driving |
| 11 | [Cloud-Based Neural Network Inference](topics/11-Cloud-Based-Neural-Network-Inference/task.ipynb) | Implement and evaluate two different methodologies to moving neural network inference from automated vehicles to connected cloud servers. | Image Segmentation, Connected Driving |
| 12 | [Panoptic Image Segmentation](topics/12-Panoptic-Image-Segmentation/task.ipynb) | Develop a methodology to perform panoptic image segmentation and evaluate its performance on semantic segmentation, instance segmentation, and panoptic segmentation tasks. | Image Segmentation |
| 13 | [Map-Based Lane Following for Scaled Automated Vehicles](topics/13-Map-Based-Lane-Following-for-Scaled-Automated-Vehicles/task.ipynb) | Develop a methodology to follow driving lanes with a scaled automated and connected vehicle, based on localizing the vehicle on a high-resolution lane map. | Vehicle Guidance |

## Past Research Topics

The following table lists past research projects that other students have worked on and published a report on in the past. These topics are not available as research projects for the current term, but the published reports can give a good idea about the expected outcome of the module.

| No. | Topic | Task | Relevant ACDC Sections | Reports |
| --- | --- | --- | --- | --- |
| 02 | [Domain Adaptation for Semantic Point Cloud Segmentation](topics/02-Domain-Adaptation-for-Semantic-Point-Cloud-Segmentation/task.ipynb) | Develop a methodology which optimizes a neural network for semantic point cloud segmentation trained on public datasets with regard to its predictive performance on data affected by domain shift. | Point Cloud Segmentation | [@marbuco (09/2022)](https://github.com/ika-rwth-aachen/acdc-research-projects/blob/main/reports/02-Domain-Adaptation-for-Semantic-Point-Cloud-Segmentation/2022-09_marbuco/report.ipynb) |
