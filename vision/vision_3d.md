{% include "../math.md" %}

# Processing 3D point clouds using PCL

Contrary to conventional 2D cameras, which provide 2D images of the
world, 3D cameras provide 3D information in the form of point
clouds. A point cloud is a collection of points described by their
X-Y-Z coordinates.

PCL is a good library that provides a number of functionalities to
manipulate point clouds. Unfortunately, contrary to OpenCV, the Python
bindings to PCL are very limited. This tutorial will therefore deal
with the C++ library. 

Since there is already an
[excellent PCL tutorial](http://pointclouds.org/documentation/tutorials/),
we shall not 
duplicate the effort here. The reader is advised to go through the
whole tutorial, with particular attention to the following
sections:

1. [Basic usage](http://pointclouds.org/documentation/tutorials/#basic-usage);

2. [I/O](http://pointclouds.org/documentation/tutorials/#i-o);

3. [Filtering](http://pointclouds.org/documentation/tutorials/#filtering-tutorial);

4. [Features](http://pointclouds.org/documentation/tutorials/#features-tutorial);

5. [Recognition](http://pointclouds.org/documentation/tutorials/#recognition-tutorial);

6. [Registration](http://pointclouds.org/documentation/tutorials/#registration-tutorial).


Many robotic applications require finding the pose (rotation and
translation) of an object in a scene. The reader is advised to go
through the
[pose estimation example](http://pointclouds.org/documentation/tutorials/alignment_prerejective.php#alignment-prerejective)
in the PCL tutorial to familiarize with that task. Note that,
depending on the models and scene at hand, some parameter values might
need to be adjusted. Please play around with them to see how they
influence the final result.
