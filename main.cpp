#include <iostream>
#include <map>
#include <vector>
#include <fstream>
#include <set>
#include <cmath>

#include "parser.h"
#include "dijkstra.h"

int main() {

    std::string path = "/home/valya/ClionProjects/NORSI_geo_2017/OSM_files/only_roads/TY.graph.osm";

//    std::string path;
//    std::cin >> path;

    std::string name_file;
    name_file = path.substr(path.rfind("/") + 1, path.rfind(".") - path.rfind("/")-1);
    std::ifstream f;
    std::pair<std::map<unsigned long, Node>, std::unordered_map<unsigned long, std::vector<vertex>>>  p;

    f.open(path.substr(0, path.rfind("OSM")-1) + "/graph/" + name_file + "_graph");
    if (f){
        p = parser_graph(f);
    } else {
        f.open(path);
        p = parser_osm(f, name_file);
    }

    unsigned long source, end;
    std::cin >> source;
    std::cin >> end;

    output_to_osc(dijkstra(p.second, source, end), name_file);

    return 0;
}