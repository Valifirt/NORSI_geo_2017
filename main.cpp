#include <iostream>
#include <map>
#include <vector>
#include <fstream>
#include <set>
#include <cmath>

#include "parser_graph.h"

int main() {

    // TODO: read path
    std::string path = "/home/valya/ClionProjects/NORSI_geo_2017/OSM_files/RU-TY.osm";

    std::ifstream f(path);

    parser(f);

    return 0;
}