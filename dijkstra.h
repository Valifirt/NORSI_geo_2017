//
// Created by valya on 12/07/17.
//

#ifndef NORSI_GEO_2017_DIJSTRA_H
#define NORSI_GEO_2017_DIJSTRA_H

#include <vector>
#include <iostream>
#include <fstream>
#include <queue>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include "parser.h"

std::pair<float, std::vector<unsigned long>> dijkstra(std::unordered_map<unsigned long, std::vector<vertex>> &graph, unsigned long source, unsigned long end);

#endif //NORSI_GEO_2017_DIJSTRA_H
