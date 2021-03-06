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
 * @file Graph.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief Create graph from 3D features in KinectCore.
 */

#ifndef GC_GRAPH_H
#define GC_GRAPH_H

#include <vector>
#include <stdio.h>
#include <cstdlib>
#include <math.h>

#include "Edge.h"
#include "v4r/SurfaceUtils/Relation.h"

namespace gc
{

/**
 * @brief Class Graph
 */
class Graph
{
private:  
  unsigned nodes;                             ///< number of surface patches
  std::vector<surface::Relation> relations;   ///< relations between features
  std::vector<gc::Edge> edges;                ///< edges of the graph (wiht node numbers and probability)

public:
  
private:

public:
  Graph(unsigned nrNodes, std::vector<surface::Relation> &rel);
  ~Graph();
  
  void BuildFromSVM(std::vector<gc::Edge> &e, unsigned &num_edges);
};

}

#endif

