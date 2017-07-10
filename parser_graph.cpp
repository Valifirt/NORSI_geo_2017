//
// Created by valya on 10/07/17.
//

#include <math.h>
#include <cmath>
#include <map>
#include <set>
#include <iostream>
#include "parser_graph.h"

const float R = 6371.0f;

float deg2rad(float deg) {
    return deg * (M_PI / 180.0f);
}

float long_dist(Node from, Node to) {


    float dLat = deg2rad(to.lat - from.lat);  // deg2rad below
    float dLon = deg2rad(to.lon - from.lon);

    float a =
            std::sin(dLat / 2) * std::sin(dLat / 2) +
            std::cos(deg2rad(from.lat)) * std::cos(deg2rad(to.lat)) *
            std::sin(dLon / 2) * std::sin(dLon / 2);

    float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    float d = R * c; // Distance in km

    return d;
}

float short_dist(Node from, Node to) {
    float dlat, dlon;
    dlat = to.lat - from.lat;
    dlon = to.lon - from.lon;

    return std::sqrt(dlat * dlat + dlon * dlon);
}

void parser(std::ifstream &in){

    std::map<long, Node> map_nodes;
    std::set<Way> set_ways;

    std::string line;

    while (std::getline(in, line)) {
        std::size_t a;
        if ((a = line.find("<node")) != std::string::npos) {
            Node tmp_node;
            a += 10;
            tmp_node.id = std::stol(line.substr(a, line.find("\" ") - a));
            a = line.find("lat=") + 5;
            tmp_node.lat = std::stof(line.substr(a, line.find("\" lon") - a));
            a = line.find("lon=") + 5;
            tmp_node.lon = std::stof(line.substr(a, line.rfind("\"") - a));

            map_nodes[tmp_node.id] = tmp_node;
        } else if ((a = line.find("<way")) != std::string::npos) {

            long last_node;
            bool first_iter = true;

            while (true) {
                std::getline(in, line);
                if (line.find("</way") != std::string::npos) { break; }
                if (line.find("<nd") != std::string::npos) {
                    a = line.find("\"") + 1;
                    if (first_iter){
                        first_iter = false;
                        last_node = std::stol(line.substr(a, line.rfind("\"") - a));
                    } else {
                        Way current_way;

                        current_way.start = map_nodes[last_node];
                        last_node = std::stol(line.substr(a, line.rfind("\"") - a));
                        current_way.finish = map_nodes[last_node];
                        current_way.len = long_dist(current_way.start, current_way.finish);
                        set_ways.insert(current_way);
                    }
                }
            }
        }
    }



    std::cout << "graph {" << std::endl;
    for (auto n : map_nodes) {
        std::cout << "\t" << n.first << " [longitude=" << n.second.lon << ", latitude=" << n.second.lat << "]" << std::endl;
    }


    for (auto w : set_ways) {
        std::cout << "\t" << w.start.id << " -- " << w.finish.id << std::endl;
    }
    std::cout << "}" << std::endl;

}