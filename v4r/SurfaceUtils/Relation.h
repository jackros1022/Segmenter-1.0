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
 * @file Relation.h
 * @author Richtsfeld
 * @date November 2011
 * @version 0.1
 * @brief Struct relation
 */

#ifndef SURFACE_RELATION_H
#define SURFACE_RELATION_H

#include <vector>

namespace surface
{

struct Relation
{
  unsigned type;                          ///< Type of relation (structural level = 1 / assembly level = 2)
  unsigned id_0;                          ///< ID of first surface 
  unsigned id_1;                          ///< ID of second surface
  std::vector<double> rel_value;          ///< relation values (feature vector)
  std::vector<double> rel_probability;    ///< probabilities of correct prediction (two class)
  unsigned groundTruth;                   ///< 0=false / 1=true
  unsigned prediction;                    ///< 0=false / 1=true // TODO not neccessary, because we have rel_probability?
//   bool remove;                            ///< delete flag (currently unused)
};

}

#endif
