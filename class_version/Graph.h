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
#include <set>
#include <ctime>

class Graph {

private:

    const float R = 6371.0f;

    struct Node {
        float lat, lon;
        std::string id;
        std::map<unsigned int, unsigned int> map_rest;
    };

    typedef std::pair<unsigned int, float> vertex;
    struct vertex_comp {
        bool operator()(const vertex &a, const vertex &b) { return a.second > b.second; }
    };

    struct nodes_comp{
        bool operator()(const unsigned int &a, const unsigned int &b){
            return a > b;
        }
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

    std::map<std::string, unsigned int> map_inf_nodes;                                                  // map: {string id -> int id(from 1 to n_of_points)}
    std::map<unsigned int,Node> map_nodes;                                                              // map: {int id -> Node}
    std::unordered_map<unsigned int, std::map<unsigned int, float>> map_edges;  // map: ways: {int id from -> int id in -> len}
    std::map<std::string, unsigned int> map_inf_ways;                                                   // map: {string id -> int id (from 1 to n_of_ways)}
    std::map<unsigned int,Way> map_ways;                                                                // map: {int id -> Way}
    std::map<unsigned int, std::map<unsigned int, unsigned int>> no_way;                                // map: {via -> {from,to}} can't create way here
    // (from, to , via is id of nodes)
    std::map<unsigned int, std::map<unsigned int, std::map<unsigned int, float,nodes_comp>>> one_way;                  // map: {from -> {to, via}} can go only here

    std::string name;                                                                                   // name of osm file without path
    std::string work_dir;                                                                               // name dir
    clock_t t_parse, t_dijkstra;

    unsigned int n_nodes;

    std::vector<std::pair<float,float>> vector_route;
    std::vector<std::pair<unsigned int,float>> vector_nearest;
    std::vector<unsigned int> vector_dist;

    unsigned int get_id_in_ways(unsigned int first, unsigned int second);                               // in map first find second

    void parser_osm(std::ifstream &in);
    float long_dist(std::pair<float,float> from, std::pair<float,float> to);
    float short_dist(std::pair<float,float> from, std::pair<float, float> to);
    std::pair<float, std::vector<unsigned int>> dijkstra(unsigned int source, unsigned int end);
    void output_to_osc(float dist, std::vector<unsigned int> way);
    void output_for_web(std::vector<unsigned int> way);
    void out_graph();

public:
    Graph(std::string path, std::vector<std::pair<float,float>> vector_r);
    void short_way();
};


#endif //NORSI_GEO_2017_GRAPH_H