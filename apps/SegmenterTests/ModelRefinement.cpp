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
 * @file ModelRefinement.cpp
 * @author Andreas Richtsfeld
 * @date August 2012
 * @version 0.1
 * @brief Abstract point clouds to parametric surface models. Model refinement with boundary fitting.
 */


#include "ModelRefinement.h"

namespace segment
{

/* --------------- ModelRefinement --------------- */

ModelRefinement::ModelRefinement()
{
  z_min = 0.3;
  z_max = 4.5;
  database_path = "/media/Daten/OSD-0.2/";
  rgbd_filename = "pcd/test%1d.pcd";
  data_live = false;
  startIdx = 0;
  endIdx = 65;  
}

ModelRefinement::~ModelRefinement()
{
  delete kinect;
  delete clusterNormals;
  delete surfModeling;
  delete resultSaver;
}

void ModelRefinement::init()
{
  bool load_models = false;   // load models from file
  bool data_depth = false;    // load depth data instead of pcd data
  std::string sfv_filename = "test_model%1d.sfv";
  
  // init view
  view.intrinsic = Eigen::Matrix3d::Zero();
  view.intrinsic(0, 0) = view.intrinsic(1, 1) = 525;
  view.intrinsic(0, 2) = 320;
  view.intrinsic(1, 2) = 240;
  view.intrinsic(2, 2) = 1.;
  view.extrinsic = Eigen::Matrix4d::Zero();
  view.extrinsic(0, 0) = view.extrinsic(1, 1) = view.extrinsic(2, 2) = 1.;
  
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
  
  // init nurbsfitting & model-selection
  pcl::on_nurbs::SequentialFitter::Parameter nurbsParams;
  nurbsParams.order = 3;
  nurbsParams.refinement = 0;
  nurbsParams.iterationsQuad = 0;
  nurbsParams.iterationsBoundary = 0;
  nurbsParams.iterationsAdjust = 0;
  nurbsParams.iterationsInterior = 3;
  nurbsParams.forceBoundary = 100.0;
  nurbsParams.forceBoundaryInside = 300.0;
  nurbsParams.forceInterior = 1.0;
  nurbsParams.stiffnessBoundary = 0.1;
  nurbsParams.stiffnessInterior = 0.1;
  nurbsParams.resolution = 16; 
  surface::SurfaceModeling::Parameter sfmParams;
  sfmParams.nurbsParams = nurbsParams;
  sfmParams.sigmaError = 0.003;
  sfmParams.kappa1 = 0.008;
  sfmParams.kappa2 = 1.0;
  sfmParams.planePointsFixation = 8000;
  sfmParams.z_max = 0.01;
  surfModeling = new surface::SurfaceModeling(sfmParams);
  surfModeling->setIntrinsic(525., 525., 320., 240.);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  surfModeling->setExtrinsic(pose);

  resultSaver = new surface::SaveFileSequence();
  resultSaver->InitFileSequence("result_%1d.sfv", 0, 1000);
}


void ModelRefinement::process()
{
  kinect->getImageData(pcl_cloud);
  pclA::FilterZ(pcl_cloud, z_min, z_max);
  view.width = pcl_cloud->width;
  view.height = pcl_cloud->height;

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
  clusterNormals->getSurfaceModels(view.surfaces);

  // surface modeling
  surfModeling->setInputCloud(pcl_cloud);
  surfModeling->setInputPatches(view.surfaces);
  surfModeling->compute();
  surfModeling->getSurfaceModels(view.surfaces, false);
  
  // boundary refiner
  refiner.setInputCloud(pcl_cloud, view);
  refiner.computeCurvesImageEdgeAligned (10, true);
  refiner.computeSurfaces ();
  refiner.trimSurfacePoints(0.0, true, true);
  refiner.reasignNAN(0.05, 40, true);
}



pcl::PointCloud<pcl::PointXYZRGBL>::Ptr
ModelRefinement::processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::copyPointCloud(*pcl_cloud, *result);
  pclA::FilterZ(pcl_cloud, z_min, z_max);

  normals.reset(new pcl::PointCloud<pcl::Normal>);
  pclA::ZAdaptiveNormals::Parameter param;
  param.adaptive = true;
  pclA::ZAdaptiveNormals nor(param);
  nor.setInputCloud(pcl_cloud);
  nor.compute();
  nor.getNormals(normals);
  
  clusterNormals->setInputCloud(pcl_cloud);
  clusterNormals->setInputNormals(normals);
  clusterNormals->setPixelCheck(true, 4);
  clusterNormals->compute();
  clusterNormals->getSurfaceModels(view.surfaces);
  
  surfModeling->setInputCloud(pcl_cloud);
  surfModeling->setInputPatches(view.surfaces);
  surfModeling->compute();
  surfModeling->getSurfaceModels(view.surfaces, false);
  
  // boundary refiner
  refiner.setInputCloud(pcl_cloud, view);
  refiner.computeCurvesImageEdgeAligned (10, true);
  refiner.computeSurfaces ();
  refiner.trimSurfacePoints(0.0, true, true);
  refiner.reasignNAN(0.05, 40, true);

  for(unsigned i=0; i<view.surfaces.size(); i++) {
    for(unsigned j=0; j<view.surfaces[i]->indices.size(); j++) {
      result->points[view.surfaces[i]->indices[j]].label = view.surfaces[i]->label;
    }
  }
  return result;
}


void ModelRefinement::run(std::string _rgbd_filename,
                          std::string _kinect_config,
                          int _startIdx, int _endIdx, bool _live)
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
  
  TomGine::tgTomGineThread dbgWin(width, height, "TomGine Render Engine");
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
    if (key == 65478 || key == 1114054)  { // F9
      printf("[ModelRefinement] Process single image.\n");
      process();
      pclA::ConvertPCLCloud2Image(pcl_cloud, kImage);
      cv::imshow("Debug image", kImage);
      dbgWin.SetImage(kImage);
      dbgWin.Update();
      win_done = false;
      processed = true;
    }
    if (key == 65479 || key == 1114055)  { // F10
      printf("[ModelRefinement] Process images countiniously.\n");
      single_image = false;
      processed = true;
    }

    if((char) key == 'h') {
      printf("[ModelRefinement] Print help:\n");
      printf("  Debug window:\n");
      printf("    \'h\' - Print this help.\n");
      printf("    \'F9\' - Process single data file.\n");
      printf("    \'F10\' - Process data file sequence. \n");
      printf("    \'p\' - Print patch info after processing.\n");
      printf("    \'s\' - Save image after processing in .sfv fileformat.\n");
      printf("    \'5\' - Show results after model abstraction.\n");
      printf("    \'6\' - Show boundaries after model abstraction.\n");
      printf("    \'q\' - Quit.\n");
      printf("  TomGine Render Engine:\n");
      printf("    \'z\' - To initial position.\n");
      printf("    \'i\' - Enable/disable background image.\n");
      printf("    \'F11\' - Save sceenshot.\n");
      printf("    \'q\' - Quit.\n");
    }

    if((char) key == 'p') {
      if(processed) {
        printf("[ModelRefinement] Print patch info:\n");
        for(int i=0; i<(int) view.surfaces.size(); i++)
          view.surfaces[i]->Print();
      }
    }

    if((char) key == 's') {
      if(processed) {
        printf("[ModelRefinement] Save model to file.\n");
        resultSaver->SaveNextView(view.surfaces);
      }
    }
    
    if((char) key == 'q') {
      printf("[ModelRefinement] Quit.\n");
      do_it = false;
    }

    if((char) key == '5' || !win_done) {
      printf("[ModelRefinement] Show results of model refinement: %lu patches\n", view.surfaces.size());
      std::vector<cv::Vec4f> col_points;
      cv::Vec4f center3D[view.surfaces.size()];
      pclA::RGBValue col[view.surfaces.size()];
      dbgWin.Clear();
      for(unsigned i=0; i<view.surfaces.size(); i++) {
        if(view.surfaces[i]->type == -1)
          col[i].float_value = 0.0;
        else
          col[i].float_value = pclA::GetRandomColor();
        for(unsigned j=0; j<view.surfaces[i]->indices.size(); j++) {
          cv::Vec4f pt;
          pt[0] = pcl_cloud->points[view.surfaces[i]->indices[j]].x;
          pt[1] = pcl_cloud->points[view.surfaces[i]->indices[j]].y;
          pt[2] = pcl_cloud->points[view.surfaces[i]->indices[j]].z;
          pt[3] = col[i].float_value;
          center3D[i][0] = center3D[i][0] + pt[0];
          center3D[i][1] = center3D[i][1] + pt[1];
          center3D[i][2] = center3D[i][2] + pt[2];
          col_points.push_back(pt);
        }

        // Add labels
        char label[5];
        snprintf(label, 5, "%u", i);
        dbgWin.AddLabel3D(label, 14, 
                          center3D[i][0]/view.surfaces[i]->indices.size(), 
                          center3D[i][1]/view.surfaces[i]->indices.size(), 
                          center3D[i][2]/view.surfaces[i]->indices.size());
      }
      dbgWin.AddPointCloud(col_points);
      dbgWin.Update();
      win_done = true;
    }
    
    if((char) key == '6') {
      dbgWin.Clear();
      printf("[ModelRefinement] Show contour of models.\n");
      pclA::RGBValue col[view.surfaces.size()];
      std::vector<cv::Vec4f> col_points;
      
      for(unsigned i=0; i<view.surfaces.size(); i++) {
        if(view.surfaces[i]->type == -1)
          col[i].float_value = 0.0;
        else
          col[i].float_value = pclA::GetRandomColor();
        for(unsigned j=0; j<view.surfaces[i]->contour.size(); j++) {
          cv::Vec4f pt;
          pt[0] = pcl_cloud->points[view.surfaces[i]->contour[j]].x;
          pt[1] = pcl_cloud->points[view.surfaces[i]->contour[j]].y;
          pt[2] = pcl_cloud->points[view.surfaces[i]->contour[j]].z;
          pt[3] = col[i].float_value;
          col_points.push_back(pt);
        }
      }
      dbgWin.AddPointCloud(col_points);
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
    "   [-l] ... live image from kinect\n", av);
  std::cout << " Example: " << av << " -f /media/Daten/OSD-0.2/pcd/test%1d.pcd -idx 0 10" << std::endl;
}


int main(int argc, char *argv[])
{
  std::string rgbd_filename = "pcd/test%1d.pcd";
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
      if(strcmp (argv[i], "-idx") == 0) {
        startIdx = atoi(argv[i+1]);
        if(i+2 < argc)
          endIdx = atoi(argv[i+2]);
      }
      if(strcmp (argv[i], "-l") == 0)
        live = true;    
        kinect_config = argv[i+1];
    }
    else
      printUsage(argv[0]);
    if(strcmp (argv[i], "-l") == 0)
      live = true;
  }
    
  segment::ModelRefinement seg;
  seg.run(rgbd_filename, kinect_config, startIdx, endIdx, live);
}


