//
// Created by valya on 10/07/17.
//

#ifndef NORSI_GEO_2017_PARSER_OSM_H
#define NORSI_GEO_2017_PARSER_OSM_H

#include <string>
#include <fstream>
#include <map>
#include <unordered_map>
#include <vector>

struct Node {
    float lat, lon;
    unsigned long id;
};

typedef std::pair<unsigned long, float> vertex;
struct vertex_comp {
    bool operator()(const vertex &a, const vertex &b) { return a.second > b.second; }
};

//parse osm and do grahp file + return two maps
std::pair<std::map<unsigned long, Node>, std::unordered_map<unsigned long, std::vector<vertex>>> parser_osm(std::ifstream &in,std::string s);

//parse graph and return two maps
std::pair<std::map<unsigned long, Node>, std::unordered_map<unsigned long, std::vector<vertex>>> parser_graph(std::ifstream &in);

void output_to_osc(std::pair<float,std::vector<unsigned long>>, std::string name);

#endif //NORSI_GEO_2017_PARSER_OSM_H
