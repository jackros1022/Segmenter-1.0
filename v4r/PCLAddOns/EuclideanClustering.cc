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


#include "EuclideanClustering.hh"


namespace pclA 
{

using namespace std;

float EuclideanClustering::NaN  = std::numeric_limits<float>::quiet_NaN(); 


/********************** EuclideanClustering ************************
 * Constructor/Destructor
 */
EuclideanClustering::EuclideanClustering(Parameter p)
{
  setParameter(p);
}

EuclideanClustering::~EuclideanClustering()
{
}


/************************** PRIVATE ************************/
/**
 * disjoint-set find root
 */
EuclideanClustering::Label* EuclideanClustering::Find(Label *x)
{
  if (x->parent == x)
  {
    return x;
  }
  else
  {
    x->parent = Find(x->parent);
    return x->parent;
  }
}

/**
 * disjoint-set make union
 */
unsigned EuclideanClustering::Union(Label *x, Label* y)
{
  Label *xRoot = Find(x);
  Label *yRoot = Find(y);

  if (xRoot->rank > yRoot->rank)
  {
    yRoot->parent = xRoot;
  }
  else if (xRoot != yRoot)
  {
    xRoot->parent = yRoot;
    if (xRoot->rank == yRoot->rank)
      yRoot->rank = yRoot->rank+1;
  }

  return (x->id<y->id?x->id:y->id);
}

/**
 * Operate does the clustering
 */
void EuclideanClustering::Operate(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
      std::vector<unsigned> &labels, std::vector<pcl::PointIndices::Ptr> &clusters, 
      std::vector<unsigned> &sizeClusters)
{
  sizeClusters.clear();
  clusters.clear();
  labels.clear();
  labels.resize(width*height,0);

  std::vector<Label*> setLabels;
  setLabels.push_back(new Label(setLabels.size()));

  //first pass...
  for (int v = 0; v < height; ++v)
  {
    for (int u = 0; u < width; ++u)
    {
      const pcl::PointXYZRGB &pt = cloud(u,v);
      unsigned &la = labels[GetIdx(u,v)];

      if (pt.x==pt.x)     // check for NaN
      {
        if (u>0)    //check left
        {
          if (SqrDistance(pt, cloud(u-1,v)) < sqrThrDist)
          {
             la = labels[GetIdx(u-1,v)];
          }
        }
        if (v>0)    //check upper
        {
          if (SqrDistance(pt, cloud(u,v-1)) < sqrThrDist && labels[GetIdx(u,v-1)] != la)
          {
            if (la==0)
            {
              la = labels[GetIdx(u,v-1)];
            }
            else
            {
              la = Union( setLabels[la], setLabels[ labels[GetIdx(u,v-1)] ] );
            }
          }
        }
        if (la==0)  // distance bigger => create new label
        {
          la = setLabels.size();
          setLabels.push_back(new Label(la));
        }
      }
    }
  }


  // second pass...
  unsigned la1, cnt=0;
  std::map<unsigned, unsigned> smartLabels;           // <source, target> 
  std::map<unsigned, unsigned>::iterator it;
  std::vector<unsigned> minLabels(setLabels.size());  // id look up table

  for (unsigned i=0; i<setLabels.size(); i++)
  {
    la1 = Find(setLabels[i])->id;
    it = smartLabels.find(la1);
    if (it==smartLabels.end())
    {
      smartLabels[la1] = cnt;
      minLabels[i] = cnt;
      cnt++;
      sizeClusters.push_back(0);
      clusters.push_back(boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices()) );
    }
    else minLabels[i] = it->second;
  }

  for (unsigned i=0; i<labels.size(); i++)
  {
    unsigned &la = minLabels[ labels[i] ];
    sizeClusters[la]++;
    clusters[la]->indices.push_back(i);
  }

  for (unsigned i=0; i<setLabels.size(); i++)
    delete setLabels[i];
}





/************************** PUBLIC *************************/

void EuclideanClustering::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  if (!_cloud->isOrganized())
    throw std::runtime_error ("[EuclideanClustering::setInputCloud] Point cloud must be organized!");

  cloud = _cloud;

  width = cloud->width;
  height = cloud->height;
}

void EuclideanClustering::compute()
{
  if (cloud.get() == 0)
    throw std::runtime_error ("[EuclideanClustering::compute] No point cloud available!");

  if (labels.get()==0)
    labels.reset (new std::vector<unsigned>());

  if (clusters.get()==0)
    clusters.reset (new std::vector<pcl::PointIndices::Ptr>());

  if (sizeClusters.get()==0)
    sizeClusters.reset (new std::vector<unsigned>());

  Operate(*cloud, *labels, *clusters, *sizeClusters);
}

void EuclideanClustering::getClusters(std::vector<pcl::PointIndices::Ptr> &_clusters)
{
  _clusters = *clusters;
}

void EuclideanClustering::getSizeClusters(std::vector<unsigned> &_sizeClusters)
{
  _sizeClusters = *sizeClusters;
}

void EuclideanClustering::getClusters(boost::shared_ptr<std::vector<pcl::PointIndices::Ptr> > &_clusters)
{
  _clusters = clusters;
}

void EuclideanClustering::getSizeClusters(boost::shared_ptr<std::vector<unsigned> > &_sizeClusters)
{
  _sizeClusters = sizeClusters;
}

void EuclideanClustering::getLabels(boost::shared_ptr<std::vector<unsigned> > &_labels)
{
  _labels = labels;
}

void EuclideanClustering::setParameter(Parameter p)
{
  param = p;
  sqrThrDist = param.thrDist*param.thrDist;
}


} //-- THE END --

