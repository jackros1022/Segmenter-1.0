# Segmenter 1.0

This software package contains software to segment data from RGBD-sensors, such as Microsoft Kinect or Asus Xtion. 
Data abstraction and segmentation of objects is done on four levels of abstraction. The input data gets pre-segmented
by  calculation of surface normals and recursive clustering of neighbouring pixels to planar patches. On the
primitive level planes and B-spline surfaces are fitted to get parametric models of the pre-segmented clusters.
A merging algorithm uses model selection to connect neighbouring surface patches if a joint model fits better than 
two single single models. On the structural and assembly level are grouping rules learned to predict connectedness 
of neighbouring and non-neighbouring patches. Finally is graph cut used to segment the image globally optimal.

There are different demo-apps implemented to test different parts of the software. Each demo application (see at end
of README) shows results at the end of a certain processing step.


# Related papers
[Richtsfeld2012] Segmentation of Unknown Objects in Indoor Environments. Richtsfeld A., Mörwald T., Prankl J.,
Zillich M. and Vincze M. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2012


# Prerequisites

The software package is designed and tested under Linux (11.10 and 12.04). Installation of OpenCV (> Vers. 2.4.1)
and installation of the pcl-trunk (Revision: 8746) (www.pointclouds.org) is mandatory. 
The following Linux (Ubuntu) packages are required:
- TBA


# Installation

Open a console session and go to the root of the software package. Then:
	$ make build
	$ cd build
	$ cmake ..
	$ make
To install the software to the system (default: /usr/local/):
	$ sudo make install
Uninstall the software:
	$ sudo make uninstall


# Run the demos


For all apps you can use the option '-h' to get help. There are two modis to start the applications:

1. Processing of previous recorded data (sequences)
Processes previous recorded data of .pcd file (sequences) in PCL style (PointXYZRGB)
Open a console session and type:
	$ demoname -f path/file.pcd
or to start a numbered sequence with start and end index:
	$ demoname -f path/file%1d.pcd -idx 10 20

2. Live processing of images from Kinect (openNI is mandatory)
A configuration file for the Kinect is necessary (default is KinectConfig.xml which can be found in
./apps/SegmenterTests). To start the live demo open a console and type:
	$ demoname -l [KinectConfig.xml]

After starting the program, click on the 'Debug image' and press 'h' to get help controlling the opened
windows.

App 1: Pre-segmentation
***********************
During pre-segmentation neighboring pixels are clustered to planar patches without discontinuities using
former calculated normals. Normals calculation considers the distance of points to the camera and reduces
the noise.

Start the demo on a console:
	$ presegmenter -f file.pcd

Press 'F9' to process single image, 'h' to get help.

App 2: Model abstraction
************************
Create surface models in a merge procedure by parametrization of patches as planes or B-spline surfaces.

Start the demo on a console:
	$ modelAbstractor -f file.pcd

Press 'F9' to process single image, 'h' to get help.

App 3: Object segmenter
***********************
Object segmentation is explained in the related work. This version considers only connection of neighbouring
patches (it's not possible to handle occlusion or objects which are spatialy seperated from a certain viewpoint).
To run this demo, a previous learned support vector machine (SVM) model is necessary. A learned model of the
learn-set from the Object Segmentation Database (OSD) is available in ./apps/SegmenterTests/ or specify the
model in the command line.

Start the demo on a console:
	$ segmenter -f file.pcd -m apps/SegmenterTests/model/

Press 'F9' to process single image, 'h' to get help.


# Using the software

We also provide a interface to use the segmentation in your software. Link the produced library 'v4rSurfaceSegmenter'
to your C++ project and add the folling to your code:

Similar to the apps are four different headers available:
	#include "v4r/SurfaceSegmenter/PreSegmenter.h"
	#include "v4r/SurfaceSegmenter/ModelAbstractor.h"
	#include "v4r/SurfaceSegmenter/ModelRefinement.h"
	#include "v4r/SurfaceSegmenter/Segmenter.h"
Generate specific segmenter:
	surface::Presegmenter presegmenter;	
	surface::ModelAbstractor modelAbstractor;	
	surface::Segmenter segmenter;	

Each class provides the same interface to process a pcl-style point cloud and returns a labeled point cloud with
the resulting segmentation:
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr
	  processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);


# Citation

If you are using the software for evaluation, please cite:

[Richtsfeld2012a] Segmentation of Unknown Objects in Indoor Environments. Richtsfeld A., Mörwald T., Prankl J.,
Zillich M. and Vincze M. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2012


Do not hesitate to contact me for further questions.

-------------------
DI Richtsfeld Andreas
Vienna University of Technology
Gusshausstraße 25
1170 Vienna
ari(at)acin.tuwien.ac.at
