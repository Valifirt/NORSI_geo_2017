//
// Created by Valifirt on 12/07/17.
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
#include "../func_version/parser.h"

std::pair<float, std::vector<unsigned int>> dijkstra(std::unordered_map<unsigned int , std::vector<vertex>> &graph, unsigned int source, unsigned int end);

#endif //NORSI_GEO_2017_DIJSTRA_H
