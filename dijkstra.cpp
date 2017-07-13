//
// Created by valya on 12/07/17.
//

#include "dijkstra.h"

std::pair<float, std::vector<unsigned long>> dijkstra(std::unordered_map<unsigned long, std::vector<vertex>> &graph, unsigned long source, unsigned long end) {
    std::priority_queue<vertex, std::vector<vertex>, vertex_comp> Q;
    std::unordered_map<unsigned long, float> dist;
    std::unordered_map<unsigned long, unsigned long> short_way;
    for (const auto &v : graph) { dist[v.first] =  std::numeric_limits<float>::infinity(); }

//    std::cout << dist.size() << std::endl;   // 258224

    Q.push({source, 0});
    while (!Q.empty()) {

        auto u = Q.top();
        Q.pop();

        if (u.second < dist[u.first]) {
            dist[u.first] = u.second;
        }

        for (const auto &v : graph[u.first]) {
            float alt = dist[u.first] + v.second;
            if (alt < dist[v.first]) {
                Q.push({v.first, alt});
                dist[v.first] = alt;
                short_way[u.first] = v.first;
            }
        }

        if (u.first == end){
//            std::cout << "dist:" << dist[end] << std::endl;
            break;
        }
    }
    std::vector<unsigned long> way;
    unsigned long v = source;
    way.push_back(v);
    while(1){
        v = short_way[v];
        way.push_back(v);
        if (v == end){
            break;
        }
    }


//    3648006836 -> 1192374777 = 213540 BEL
//    2412931380 -> 409085250

    return {dist[end], way};

}



