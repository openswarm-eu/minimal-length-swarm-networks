/**
 * @file <utility/team_color.h>
 *
 * @author Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
 * 
 * Defines the color to be used by each team.
 */

#ifndef TEAM_COLOR_H
#define TEAM_COLOR_H

/* Definition of the Color datatype. */
#include <argos3/core/utility/datatypes/color.h>
#include <unordered_map>

using namespace argos;

/* Team colors */
std::unordered_map<UInt8, CColor> teamColor = {
                                                {1, CColor::GREEN},
                                                {2, CColor::GREEN},
                                                {3, CColor::GREEN},
                                                {4, CColor::GREEN},
                                                {5, CColor::GREEN},
                                                {6, CColor::GREEN},
                                              };

#endif