//
// Created by valya on 13/07/17.
//

#ifndef NORSI_GEO_2017_GRAPH_H
#define NORSI_GEO_2017_GRAPH_H

#include <string>
#include <fstream>
#include <map>
#include <unordered_map>
#include <vector>

class Graph {

private:

    const float R = 6371.0f;

    struct Node {
        float lat, lon;
        std::string id;
    };

    typedef std::pair<unsigned int, float> vertex;
    struct vertex_comp {
        bool operator()(const vertex &a, const vertex &b) { return a.second > b.second; }
    };

    std::map<std::string, unsigned int> map_inf;                    // map: {string id -> int id(from 0 to n_of_points)}
    std::map<unsigned int,Node> map_nodes;                          // map: {int id -> Node}
    std::unordered_map<unsigned int, std::vector<vertex>> map_ways;  // map ways: {int id -> vector of id in way}

    std::string name;                                               // name of osm file without path
    std::string work_dir;                                           // name dir
    unsigned int n_nodes;

    void parser_osm(std::ifstream &in);
    void parser_graph(std::ifstream &in);
    float long_dist(Node from, Node to);
    float short_dist(Node from, Node to);
    std::pair<float, std::vector<unsigned int>> dijkstra(unsigned int source, unsigned int end);
    void output_to_osc(float dist, std::vector<unsigned int> way);

public:
    Graph(std::string path);
    void short_way(std::vector<std::string> nodes);
};


#endif //NORSI_GEO_2017_GRAPH_H
