//
// Created by Valifirt on 17/07/17.
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

    struct Way{
        std::vector<unsigned int> vector_nodes;
        std::string id;
    };

//    struct nodes_comp{
//        bool operator()(const unsigned int &a, const unsigned int &b){
//            return (map_nodes[a].lon > map_nodes[b].lon || (map_nodes[a].lon == map_nodes[b].lon && map_nodes[a].lat > map_nodes[b].lat ));
//        }
//    };

    std::map<std::string, unsigned int> map_inf_nodes;                          // map: {string id -> int id(from 1 to n_of_points)}
    std::map<unsigned int,Node> map_nodes;                                      // map: {int id -> Node}
    std::unordered_map<unsigned int, std::map<unsigned int, float>> map_edges;  // map: ways: {int id -> vector of id in way}
    std::map<std::string, unsigned int> map_inf_ways;                           // map: {string id -> int id (from 1 to n_of_ways)}
    std::map<unsigned int,Way> map_ways;                                        // map: {int id -> Way}
    std::map<unsigned int, std::map<unsigned int, unsigned int>> no_way;       // map: {via -> {from,to}} can't create way here
                                                                                // (from, to , via is id of nodes)
    std::map<unsigned int, std::map<unsigned int, unsigned int>> one_way;       // map: {via -> {from, to}} can go only here

    std::string name;                                                           // name of osm file without path
    std::string work_dir;                                                       // name dir
    unsigned int n_nodes;

    unsigned int get_id_in_ways(unsigned int first, unsigned int second);               // in map first find second

    void parser_osm(std::ifstream &in);
    float long_dist(Node from, Node to);
    float short_dist(Node from, Node to);
    std::pair<float, std::vector<unsigned int>> dijkstra(unsigned int source, unsigned int end);
    void output_to_osc(float dist, std::vector<unsigned int> way);

public:
    Graph(std::string path);
    void short_way(std::vector<std::string> nodes);
};


#endif //NORSI_GEO_2017_GRAPH_H