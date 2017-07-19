//
// Created by Valifirt on 12/07/17.
//

#include "../dijkstra/dijkstra.h"

std::pair<float, std::vector<unsigned int>> dijkstra(std::unordered_map<unsigned int, std::vector<vertex>> &graph, unsigned int source, unsigned int end) {
    std::priority_queue<vertex, std::vector<vertex>, vertex_comp> Q;
    std::unordered_map<unsigned int, float> dist;
    std::unordered_map<unsigned int, unsigned int> short_way;
    for (const auto &v : graph) { dist[v.first] =  std::numeric_limits<float>::infinity(); }

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
                short_way[v.first] = u.first;
            }
        }

        if (u.first == end){
            std::vector<unsigned int> way;
            unsigned int v = end;
            while(1){
                v = short_way[v];
                way.insert(way.begin(),v);
                if (v == source){
                    break;
                }
            }

            return {dist[end], way};
        }
    }

    std::cout << "Can't create way" << std::endl;
    exit(4);
}



