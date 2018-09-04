{% include "../math.md" %}

# About robot vision software

**1. OpenCV**:  is an open-source library that includes many computer vision
algorithms, such as object detection or other *image* processing
functionalities.

**2. The Point Cloud Library (PCL)**: is an open-source library for processing
*point clouds*. The library provides many algorithms for filtering,
feature estimation, registration, model fitting, or segmentation.

# Installation

**1. OpenCV**:

Install Python-OpenCV from the Ubuntu repository by
{% label %}command-line{% endlabel %}
``` bash
 sudo apt-get update
 sudo apt-get install libopencv-dev python-opencv
 ```
 
**2. PCL**:

PCL can be installed through PPA
{% label %}command-line{% endlabel %}
``` bash
 sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
 sudo apt-get update
 sudo apt-get install libpcl1.7 pcl-tools
```

Note that PCL currently supports only C++. There exist a set of
[Python bindings](https://github.com/strawlab/python-pcl), but they
provide very few functionalities.

The above instructions will probably not install the latest version of
OpenCV and PCL. To get newer version of these libraries, consider
installing from source.
