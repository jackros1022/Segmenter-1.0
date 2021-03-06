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
 * @file Segmenter.cpp
 * @author Andreas Richtsfeld
 * @date July 2012
 * @version 0.1
 * @brief Segment images
 */


#include "Segmenter.h"

namespace segment
{

unsigned WhichGraphCutGroup(unsigned modelID, std::vector< std::vector<unsigned> > _graphCutGroups)
{
  for(unsigned i=0; i<_graphCutGroups.size(); i++)
    for(unsigned j=0; j<_graphCutGroups[i].size(); j++)
      if(_graphCutGroups[i][j] == modelID)
        return i;
  return 0;
}


/* --------------- Segmenter --------------- */

Segmenter::Segmenter(std::string _db, std::string _rgbd, std::string _model, bool _live)
{
  z_min = 0.3;
  z_max = 4.5;
  database_path = _db;
  rgbd_filename = _rgbd;
  model_path = _model;
  data_live = _live;
  startIdx = 0;
  endIdx = 65;  
  useStructuralLevel = true;
  useAssemblyLevel = false;
}

Segmenter::~Segmenter()
{
  delete kinect;
  delete surfModeling;
  delete patchRelations;
  delete svm1st;
  delete svm2nd;
  delete graphCut;
  delete resultSaver;
}

void Segmenter::init()
{
  bool load_models = false;   // load models from file
  bool data_depth = false;    // load depth data instead of pcd data
  std::string sfv_filename = "test_model%1d.sfv";
  std::string svmStructuralModel = model_path + "PP-Trainingsset.txt.scaled.model";
  std::string svmStructuralScaling = model_path + "param.txt";
  std::string svmAssemblyModel = model_path + "PP2-Trainingsset.txt.scaled.model";
  std::string svmAssemblyScaling = model_path + "param2.txt";
  
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
  sfmParams.planePointsFixation = 8000;       // 8000
  sfmParams.z_max = 0.01;
  surfModeling = new surface::SurfaceModeling(sfmParams);
  surfModeling->setIntrinsic(525., 525., 320., 240.);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  surfModeling->setExtrinsic(pose);
  
  // init patch relation class
  patchRelations = new surface::PatchRelations();
  patchRelations->setStructuralLevel(useStructuralLevel);
  patchRelations->setAssemblyLevel(useAssemblyLevel);
  
  // init svm-predictor
  svm1st = new svm::SVMPredictorSingle(svmStructuralModel.c_str());
  svm2nd = new svm::SVMPredictorSingle(svmAssemblyModel.c_str());
  svm1st->setScaling(true, svmStructuralScaling.c_str());
  svm2nd->setScaling(true, svmAssemblyScaling.c_str());

  // init graph cutter
  graphCut = new gc::GraphCut();
  
  // save results to sfv file
  resultSaver = new surface::SaveFileSequence();
  resultSaver->InitFileSequence("result_%1d.sfv", 0, 1000);
}


void Segmenter::process()
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
  printf("  => calcuated normals ...\n");
  
  // adaptive clustering
  clusterNormals->setInputCloud(pcl_cloud);
  clusterNormals->setInputNormals(normals);
  clusterNormals->setPixelCheck(true, 5);
  clusterNormals->compute();
  clusterNormals->getSurfaceModels(surfaces);
  printf("  => clustering done ...\n");
  
  surfModeling->setInputCloud(pcl_cloud);
  surfModeling->setInputPatches(surfaces);
  surfModeling->compute();
  surfModeling->getSurfaceModels(surfaces, false);
  printf("  => parametric models estimated ...\n");
  
  std::vector<surface::Relation> relation_vector;
  patchRelations->setInputCloud(pcl_cloud);
  patchRelations->setSurfaceModels(surfaces);
  patchRelations->setOptimalPatchModels(true);
  patchRelations->computeSegmentRelations();
  patchRelations->getRelations(relation_vector);
  printf("  => relations between models calculated ...\n");

  for(unsigned i=0; i<relation_vector.size(); i++) {
    if(relation_vector[i].type == 1)
      relation_vector[i].prediction = svm1st->getResult(relation_vector[i].type, 
                                                        relation_vector[i].rel_value, 
                                                        relation_vector[i].rel_probability);
    if(relation_vector[i].type == 2)
      relation_vector[i].prediction = svm2nd->getResult(relation_vector[i].type, 
                                                        relation_vector[i].rel_value, 
                                                        relation_vector[i].rel_probability);
  }
  printf("  => svm prediction done ...\n");

  /// TODO HACK: For analyzing output 
//   svm::SVMFileCreator svmFileCreator;
//   svmFileCreator.setRelations(relation_vector);
//   svmFileCreator.setAnalyzeOutput(false);
//   svmFileCreator.setTestSet(true);
//   svmFileCreator.process();
  
  graphCutGroups.clear();
  graphCut->init(surfaces.size(), relation_vector);
  graphCut->process();
  graphCut->getResults(surfaces.size(), graphCutGroups);
  for(unsigned i=0; i<graphCutGroups.size(); i++)
    for(unsigned j=0; j<graphCutGroups[i].size(); j++)
      surfaces[graphCutGroups[i][j]]->label = i;
    
  printf("  => graph cut found optimal solution ...\n");
    
#ifdef DEBUG
  graphCut->printResults();
#endif
  printf("  => segmentation done.\n");
}


pcl::PointCloud<pcl::PointXYZRGBL>::Ptr
Segmenter::processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
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
  clusterNormals->setPixelCheck(true, 5);
  clusterNormals->compute();
  clusterNormals->getSurfaceModels(surfaces);
  
  surfModeling->setInputCloud(pcl_cloud);
  surfModeling->setInputPatches(surfaces);
  surfModeling->compute();
  surfModeling->getSurfaceModels(surfaces, false);
  
  std::vector<surface::Relation> relation_vector;
  patchRelations->setInputCloud(pcl_cloud);
  patchRelations->setSurfaceModels(surfaces);
  patchRelations->setOptimalPatchModels(true);
  patchRelations->computeSegmentRelations();
  patchRelations->getRelations(relation_vector);

  for(unsigned i=0; i<relation_vector.size(); i++) {
    if(relation_vector[i].type == 1)
      relation_vector[i].prediction = svm1st->getResult(relation_vector[i].type, 
                                                        relation_vector[i].rel_value, 
                                                        relation_vector[i].rel_probability);
    if(relation_vector[i].type == 2)
      relation_vector[i].prediction = svm2nd->getResult(relation_vector[i].type, 
                                                        relation_vector[i].rel_value, 
                                                        relation_vector[i].rel_probability);
  }
  
  graphCutGroups.clear();
  graphCut->init(surfaces.size(), relation_vector);
  graphCut->process();
  graphCut->getResults(surfaces.size(), graphCutGroups);
  for(unsigned i=0; i<graphCutGroups.size(); i++)
    for(unsigned j=0; j<graphCutGroups[i].size(); j++)
  
  for(unsigned i=0; i<surfaces.size(); i++) {
    for(unsigned j=0; j<surfaces[i]->indices.size(); j++) {
      result->points[surfaces[i]->indices[j]].label = surfaces[i]->label;
    }
  }
  return result;
}


std::vector<pcl::PointIndices> 
Segmenter::processPointCloudV(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
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
  
  surfModeling->setInputCloud(pcl_cloud);
  surfModeling->setInputPatches(surfaces);
  surfModeling->compute();
  surfModeling->getSurfaceModels(surfaces);
  
  std::vector<surface::Relation> relation_vector;
  patchRelations->setInputCloud(pcl_cloud);
  patchRelations->setSurfaceModels(surfaces);
  patchRelations->setOptimalPatchModels(true);
  patchRelations->computeSegmentRelations();
  patchRelations->getRelations(relation_vector);
  
  for(unsigned i=0; i<relation_vector.size(); i++) {
    if(relation_vector[i].type == 1)
      relation_vector[i].prediction = svm1st->getResult(relation_vector[i].type, 
                                                        relation_vector[i].rel_value, 
                                                        relation_vector[i].rel_probability);
    if(relation_vector[i].type == 2)
      relation_vector[i].prediction = svm2nd->getResult(relation_vector[i].type, 
                                                        relation_vector[i].rel_value, 
                                                        relation_vector[i].rel_probability);
  }
  
  graphCutGroups.clear();
  graphCut->init(surfaces.size(), relation_vector);
  graphCut->process();
  graphCut->getResults(surfaces.size(), graphCutGroups);
  graphCut->printResults();
  
  std::vector<pcl::PointIndices> results;
  results.resize(graphCutGroups.size());
  for(unsigned i=0; i<graphCutGroups.size(); i++)
    for(unsigned j=0; j<graphCutGroups[i].size(); j++)
      for(unsigned k=0; k<surfaces[graphCutGroups[i][j]]->indices.size(); k++)
        results[i].indices.push_back(surfaces[graphCutGroups[i][j]]->indices[k]);
  return results;
}


void Segmenter::run(std::string _rgbd_filename,
                    std::string _kinect_config,
                    std::string _model_path,
                    int _startIdx, int _endIdx, 
                    bool _live, bool _useAssemblyLevel)
{
  bool processed = false;
  database_path = "";
  model_path = _model_path;
  rgbd_filename = _rgbd_filename;
  kinect_config = _kinect_config;
  startIdx = _startIdx;
  endIdx = _endIdx;
  data_live = _live;
  useAssemblyLevel = _useAssemblyLevel;
  init();
  
  // ######################## Setup TomGine ########################
  int width = 640;
  int height = 480;
  surface::View view;
  
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
  bool model = false;
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
      printf("[Segmenter] Print help:\n");
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
      printf("    \'F11\' - Save sceenshot.\n");
      printf("    \'q\' - Quit.\n");
    }
    
    if (key == 65478 || key == 1114054)  { // F9
      printf("[Segmenter] Process single image.\n");
      process();
      pclA::ConvertPCLCloud2Image(pcl_cloud, kImage);
      cv::imshow("Debug image", kImage);
      dbgWin.SetImage(kImage);
      dbgWin.Update();
      bool model = true;
      win_done = false;
      processed = true;
    }
    if (key == 65479 || key == 1114055)  { // F10
      printf("[Segmenter] Process images countiniously.\n");
      single_image = false;
      processed = true;
    }

    if((char) key == 'p') {
      if(processed) {
        printf("[Segmenter] Print patch info:\n");
        for(int i=0; i<(int)surfaces.size(); i++)
          surfaces[i]->Print();
      }
    }

    if((char) key == 's') {
      if(processed) {
        printf("[Segmenter] Save model to file.\n");
        surface::View view;
        view.width = pcl_cloud->width;
        view.height = pcl_cloud->height;
        view.surfaces = surfaces;
        view.intrinsic = Eigen::Matrix3d::Zero();
        view.intrinsic(0, 0) = view.intrinsic(1, 1) = 525;
        view.intrinsic(0, 2) = 320;
        view.intrinsic(1, 2) = 240;
        view.intrinsic(2, 2) = 1.;
        view.extrinsic = Eigen::Matrix4d::Zero();
        view.extrinsic(0, 0) = view.extrinsic(1, 1) = view.extrinsic(2, 2) = 1.;
        resultSaver->SaveNextView(surfaces);
      }
    }
    
    if((char) key == 'q') {
        printf("[Segmenter] Quit.\n");
        do_it = false;
    }
    
    if((char) key == '5' || !win_done) {
      printf("[Segmenter] Show results after graph cut.\n");
      dbgWin.Clear();
      dbgWin.ClearModels();
      cv::Vec4f center3D[graphCutGroups.size()];
      int nrGCPoints[graphCutGroups.size()];
      pclA::RGBValue col[graphCutGroups.size()];
      for(size_t i=0; i<graphCutGroups.size(); i++) {
        col[i].float_value = pclA::GetRandomColor();
        nrGCPoints[i]=0;
      }
      
      std::vector<cv::Vec4f> col_points;
      for(size_t i=0; i<surfaces.size(); i++) {
        for(size_t j=0; j<surfaces[i]->indices.size(); j++) {
          cv::Vec4f pt;
          pt[0] = pcl_cloud->points[surfaces[i]->indices[j]].x;
          pt[1] = pcl_cloud->points[surfaces[i]->indices[j]].y;
          pt[2] = pcl_cloud->points[surfaces[i]->indices[j]].z;
          unsigned number = WhichGraphCutGroup(i, graphCutGroups);
          pt[3] = col[number].float_value;
          nrGCPoints[number]++;
          center3D[number][0] = center3D[number][0] + pt[0];
          center3D[number][1] = center3D[number][1] + pt[1];
          center3D[number][2] = center3D[number][2] + pt[2];
          col_points.push_back(pt);
        }
      }
        
      for(size_t i=0; i<graphCutGroups.size(); i++) {
        char label[5];
        snprintf(label, 5, "%lu", i);
        dbgWin.AddLabel3D(label, 14, 
                            center3D[i][0]/nrGCPoints[i], 
                            center3D[i][1]/nrGCPoints[i], 
                            center3D[i][2]/nrGCPoints[i]);
      }
      dbgWin.AddPointCloud(col_points);
      dbgWin.Update();
      win_done = true;
    }
    
    if((char) key == '7') {
      printf("[Segmenter] Showing segmentation results as parametric models ... \n");

      if(model) {
        objectmodeling::CreateMeshModel meshmodelling(objectmodeling::CreateMeshModel::Parameter(.1));
        meshmodelling.setInputCloud(pcl_cloud);
        meshmodelling.compute(surfaces);
        model = false;
      }
      
      dbgWin.Clear();
      dbgWin.ClearModels();
      pclA::RGBValue color;
      for(unsigned i=0; i<graphCutGroups.size(); i++) {
        color.float_value = pclA::GetRandomColor();
        for(unsigned j=0; j<graphCutGroups[i].size(); j++) {    
          surfaces[graphCutGroups[i][j]]->mesh.m_material.Color(color.r/255., color.g/255., color.b/255.);
          dbgWin.AddModel(&surfaces[graphCutGroups[i][j]]->mesh);
        }
      }
      dbgWin.Update();
      printf("[Segmenter] done.\n");
    }
  }
  printf("[Segmenter::run] Done.\n");
}
} // end segment


void printUsage(char *av)
{
  printf("Usage: %s [options] \n"
    " Options:\n"
    "   [-h] ... show this help.\n"
    "   [-f rgbd_filename] ... specify rgbd-image path and filename\n"
    "   [-m model_path] ... specify model path\n"
    "   [-idx start end] ... start and end index of files\n"
    "   [-l] ... live image from kinect\n"
    "   [-as usage] ... use assembly level: 0/1\n", av);
  std::cout << " Example: " << av << " -f /media/Daten/OSD-0.2/pcd/test%1d.pcd -m model/ -idx 0 10" << std::endl;
}


int main(int argc, char *argv[])
{
  std::string rgbd_filename = "points2/test%1d.pcd";
  std::string kinect_config = "KinectConfig.xml";
  std::string model_path = "model/";
  int startIdx = 51;
  int endIdx = 51;
  bool live = false;
  bool useAssemblyLevel = false;
  
  for(int i=1; i<argc; i++) {
    if(strcmp (argv[i], "-h") == 0) {
      printUsage(argv[0]);
      exit(0);
    }
    if(i+1 < argc) {
      if(strcmp (argv[i], "-f") == 0)
        rgbd_filename = argv[i+1];
      if(strcmp (argv[i], "-m") == 0)
        model_path = argv[i+1];
      if(strcmp (argv[i], "-l") == 0) {
        live = true;
        kinect_config = argv[i+1];
      }
      if(strcmp (argv[i], "-idx") == 0) {
        startIdx = atoi(argv[i+1]);
        if(i+2 < argc)
          endIdx = atoi(argv[i+2]);
      }
      if(strcmp (argv[i], "-as") == 0)
        useAssemblyLevel = (bool) atoi(argv[i+1]);
    }
    else 
      printUsage(argv[0]);
    if(strcmp (argv[i], "-l") == 0)
      live = true;
  }
    
  segment::Segmenter seg;
  //seg.setMinMaxDepth(0.0, 1.5);
  seg.run(rgbd_filename, kinect_config, model_path, startIdx, endIdx, live, useAssemblyLevel);
}


