/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file PreSegmenter.cpp
 * @author Andreas Richtsfeld
 * @date August 2012
 * @version 0.1
 * @brief Pre-segment images to planar patches.
 */


#include "PreSegmenter.h"

namespace segment
{

/* --------------- PreSegmenter --------------- */

PreSegmenter::PreSegmenter()
{
  z_min = 0.3;
  z_max = 4.5;
  database_path = "/media/U-Daten/OSD/";
  rgbd_filename = "points2/test%1d.pcd";
  data_live = false;
  startIdx = 0;
  endIdx = 65;  
}

PreSegmenter::~PreSegmenter()
{
  delete kinect;
//   delete presegmenter;
  delete clusterNormals;
  delete resultSaver;
}

void PreSegmenter::init()
{
  bool load_models = false;   // load models from file
  bool data_depth = false;    // load depth data instead of pcd data
  std::string sfv_filename = "test_model%1d.sfv";
  
  // init kinect data reader
  kinect = new KinectData();
  kinect->setDatabasePath(database_path); 
  if(data_live)
    kinect->setReadDataLive(kinect_config);
  else
    kinect->setReadDataFromFile(rgbd_filename, startIdx, endIdx, data_depth);
  if(load_models)
    kinect->setReadModelsFromFile(sfv_filename, startIdx, endIdx);
  
  surface::ClusterNormalsToPlanes::Parameter param;
  param.adaptive = true;         // use adaptive thresholds
  clusterNormals = new surface::ClusterNormalsToPlanes(param);
  
  resultSaver = new surface::SaveFileSequence();
  resultSaver->InitFileSequence("result_%1d.sfv", 0, 1000);
}


void PreSegmenter::process()
{
  kinect->getImageData(pcl_cloud);
  pclA::FilterZ(pcl_cloud, z_min, z_max);
  
  // calcuate normals
  normals.reset(new pcl::PointCloud<pcl::Normal>);
  pclA::ZAdaptiveNormals::Parameter param;
  param.adaptive = true;
  pclA::ZAdaptiveNormals nor(param);
  nor.setInputCloud(pcl_cloud);
  nor.compute();
  nor.getNormals(normals);
  
  // adaptive clustering
  clusterNormals->setInputCloud(pcl_cloud);
  clusterNormals->setInputNormals(normals);
  clusterNormals->setPixelCheck(true, 5);
  clusterNormals->compute();
  clusterNormals->getSurfaceModels(surfaces);
}


pcl::PointCloud<pcl::PointXYZRGBL>::Ptr
PreSegmenter::processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::copyPointCloud(*pcl_cloud, *result);
  pclA::FilterZ(pcl_cloud, z_min, z_max);
  
  // calcuate normals
  normals.reset(new pcl::PointCloud<pcl::Normal>);
  pclA::ZAdaptiveNormals::Parameter param;
  param.adaptive = true;
  pclA::ZAdaptiveNormals nor(param);
  nor.setInputCloud(pcl_cloud);
  nor.compute();
  nor.getNormals(normals);
  
  // adaptive clustering
  clusterNormals->setInputCloud(pcl_cloud);
  clusterNormals->setInputNormals(normals);
  clusterNormals->setPixelCheck(true, 5);
  clusterNormals->compute();
  clusterNormals->getSurfaceModels(surfaces);
  
  for(unsigned i=0; i<surfaces.size(); i++) {
    for(unsigned j=0; j<surfaces[i]->indices.size(); j++) {
      result->points[surfaces[i]->indices[j]].label = surfaces[i]->label;
    }
  }
  return result;
}

void PreSegmenter::run(std::string _rgbd_filename, 
                       std::string _kinect_config,
                       int _startIdx, int _endIdx, 
                       bool _live)
{
  bool processed = false;
  database_path = "";
  rgbd_filename = _rgbd_filename;
  kinect_config = _kinect_config;
  startIdx = _startIdx;
  endIdx = _endIdx;
  data_live = _live;
  init();
  
  // ######################## Setup TomGine ########################
  int width = 640;
  int height = 480;
  surface::View view;
  
  TomGine::tgTomGineThreadPCL dbgWin(width, height, "TomGine Render Engine");
  cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Mat t = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  cv::Vec3d rotCenter(0, 0, 1.0);

  cv::Mat intrinsic;
  intrinsic = cv::Mat::zeros(3, 3, CV_64F);
  view.intrinsic = Eigen::Matrix3d::Zero();
  intrinsic.at<double> (0, 0) = intrinsic.at<double> (1, 1) = view.intrinsic(0, 0) = view.intrinsic(1, 1) = 525;
  intrinsic.at<double> (0, 2) = view.intrinsic(0, 2) = 320;
  intrinsic.at<double> (1, 2) = view.intrinsic(1, 2) = 240;
  intrinsic.at<double> (2, 2) = view.intrinsic(2, 2) = 1.;

  dbgWin.SetClearColor(0.5, 0.5, 0.5);
  dbgWin.SetCoordinateFrame();
  dbgWin.SetCamera(intrinsic);
  dbgWin.SetCamera(R, t);
  dbgWin.SetRotationCenter(rotCenter);
  dbgWin.Update();
  
  cv::Mat_<cv::Vec3b> kImage = cv::Mat_<cv::Vec3b>::zeros(480, 640);
  cv::imshow("Debug image", kImage);
  
  bool do_it = true;
  bool single_image = true;
  bool win_done = true;
  while(do_it) {
    if(!single_image) {
      process();
      pclA::ConvertPCLCloud2Image(pcl_cloud, kImage);
      cv::imshow("Debug image", kImage);
      dbgWin.SetImage(kImage);
      dbgWin.Update();
      win_done = false;
    }

    int key = cvWaitKey(50);
    
    if((char) key == 'h') {
      printf("[PreSegmenter] Print help:\n");
      printf("  Debug window:\n");
      printf("    \'h\' - Print this help.\n");
      printf("    \'F9\' - Process single data file.\n");
      printf("    \'F10\' - Process data file sequence. \n");
      printf("    \'p\' - Print patch info after processing.\n");
      printf("    \'s\' - Save image after processing in .sfv fileformat.\n");
      printf("    \'q\' - Quit.\n");
      printf("  TomGine Render Engine:\n");
      printf("    \'z\' - To initial position.\n");
      printf("    \'i\' - Enable/disable background image.\n");
      printf("    \'t\' - Enable/disable text annotation.\n");
      printf("    \'p\' - Enable/disable displaying of point cloud.\n");
      printf("    \'F11\' - Save sceenshot.\n");
      printf("    \'q\' - Quit.\n");
    }
    
    if (key == 65478 || key == 1114054)  { // F9
      printf("[PreSegmenter] Process single image.\n");
      process();
      pclA::ConvertPCLCloud2Image(pcl_cloud, kImage);
      cv::imshow("Debug image", kImage);
      dbgWin.SetImage(kImage);
      dbgWin.Update();
      win_done = false;
      processed = true;
    }
    if (key == 65479 || key == 1114055)  { // F10
      printf("[PreSegmenter] Process images countiniously.\n");
      single_image = false;
      processed = true;
    }

    if((char) key == 'p') {
      if(processed) {
        printf("[PreSegmenter] Print patch info:\n");
        for(int i=0; i<(int)surfaces.size(); i++)
          surfaces[i]->Print();
      }
    }

    if((char) key == 's') {
      if(processed) {
        printf("[PreSegmenter] Save model to file.\n");
        resultSaver->SaveNextView(surfaces);
      }
    }
    
    if((char) key == 'q') {
        printf("[PreSegmenter] Quit.\n");
        do_it = false;
    }
    
    if((char) key == '1') {
      std::vector<cv::Vec4f> col_points;
      for(unsigned i=0; i<pcl_cloud->points.size(); i++) {
          cv::Vec4f pt;
          pt[0] = pcl_cloud->points[i].x;
          pt[1] = pcl_cloud->points[i].y;
          pt[2] = pcl_cloud->points[i].z;
          pt[3] = pcl_cloud->points[i].rgb;
          col_points.push_back(pt);
        }
        dbgWin.Clear();
        dbgWin.AddPointCloud(col_points);
        dbgWin.Update();
    }
    
    if((char) key == '5' || !win_done) {
      printf("[PreSegmenter] Show results after pre-segmenation: Number of surfaces: %lu\n", surfaces.size());
      std::vector<cv::Vec4f> col_points;
      cv::Vec4f center3D[surfaces.size()];
      pclA::RGBValue col[surfaces.size()];
      dbgWin.Clear();
      for(unsigned i=0; i<surfaces.size(); i++) {
        if(surfaces[i]->type == -1)
          col[i].float_value = 0.0;
        else
          col[i].float_value = pclA::GetRandomColor();
        for(unsigned j=0; j<surfaces[i]->indices.size(); j++) {
          cv::Vec4f pt;
          pt[0] = pcl_cloud->points[surfaces[i]->indices[j]].x;
          pt[1] = pcl_cloud->points[surfaces[i]->indices[j]].y;
          pt[2] = pcl_cloud->points[surfaces[i]->indices[j]].z;
          pt[3] = col[i].float_value;
          center3D[i][0] = center3D[i][0] + pt[0];
          center3D[i][1] = center3D[i][1] + pt[1];
          center3D[i][2] = center3D[i][2] + pt[2];
          col_points.push_back(pt);
        }

        // Add labels
        char label[8];
        snprintf(label, 8, "%u", i);
        dbgWin.AddLabel3D(label, 14, 
                          center3D[i][0]/surfaces[i]->indices.size(), 
                          center3D[i][1]/surfaces[i]->indices.size(), 
                          center3D[i][2]/surfaces[i]->indices.size());
      }
      dbgWin.AddPointCloud(col_points);
      dbgWin.Update();
      win_done = true;
    }
  
    if((char) key == '7') {
      printf("[PreSegmenter] Show normals after pre-segmenation\n");
      dbgWin.AddPointCloudPCL(*pcl_cloud, *normals, 0.002);
      dbgWin.Update();
      win_done = true;
    }
  }
}

} // end segment


void printUsage(char *av)
{
  printf("Usage: %s [options] \n"
    " Options:\n"
    "   [-h] ... show this help.\n"
    "   [-f rgbd_filename] ... specify rgbd-image path and filename\n"
    "   [-idx start end] ... start and end index of files\n"
    "   [-l kinect_configfile] ... live image from kinect\n", av);
  std::cout << " Example: " << av << " -f /media/Daten/OSD-0.2/pcd/test%1d.pcd -idx 0 10" << std::endl;
}


int main(int argc, char *argv[])
{
  std::string rgbd_filename = "points2/test%1d.pcd";
  std::string kinect_config = "KinectConfig.xml";
  int startIdx = 51;
  int endIdx = 51;
  bool live = false;
  
  for(int i=1; i<argc; i++) {
    if(strcmp (argv[i], "-h") == 0) {
      printUsage(argv[0]);
      exit(0);
    }
    if(i+1 < argc) {
      if(strcmp (argv[i], "-f") == 0)
        rgbd_filename = argv[i+1];
      if(strcmp (argv[i], "-l") == 0) {
        live = true;
        kinect_config = argv[i+1];
      }
      if(strcmp (argv[i], "-idx") == 0) {
        startIdx = atoi(argv[i+1]);
        if(i+2 < argc)
          endIdx = atoi(argv[i+2]);
      }
    }
    else 
      printUsage(argv[0]);
  }
    
  segment::PreSegmenter seg;
  seg.run(rgbd_filename, kinect_config, startIdx, endIdx, live);
}


