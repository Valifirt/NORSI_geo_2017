//
// Created by Valifirt on 17/07/17.
//

#include <iostream>
#include <math.h>
#include <cmath>
#include <queue>
#include <limits>
#include <sys/stat.h>
#include <cstring>
#include "Graph.h"

Graph::Graph(std::string path, std::vector<std::pair<float, float>> vector_r) {

    this->name = path.substr(path.rfind("/") + 1, path.rfind(".") - path.rfind("/")-1);
    this->work_dir = path.substr(0,path.rfind("/data") + 1);

    this->vector_route = vector_r;
    for(int i = 0; i < vector_r.size(); i++){
        vector_nearest.push_back({0, std::numeric_limits<float>::infinity()});
    }

    std::ifstream f;
    f.open(path);
    if (!f){
        std::cout << "Can't find *.osm file" << std::endl;
        exit(2);
    }

    parser_osm(f);
    short_way();
}

float deg2rad(float deg) {
    return deg * (M_PI / 180.0f);
}

float Graph::long_dist(std::pair<float,float> from, std::pair<float,float> to) {


    float dLat = deg2rad(to.first - from.first);  // deg2rad below
    float dLon = deg2rad(to.second - from.second);

    float a =
            std::sin(dLat / 2) * std::sin(dLat / 2) +
            std::cos(deg2rad(from.first)) * std::cos(deg2rad(to.first)) *
            std::sin(dLon / 2) * std::sin(dLon / 2);

    float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    float d = R * c; // Distance in km

    return d*1000;
}

float Graph::long_dist(Node from, Node to) {


    float dLat = deg2rad(to.lat - from.lat);  // deg2rad below
    float dLon = deg2rad(to.lon - from.lon);

    float a =
            std::sin(dLat / 2) * std::sin(dLat / 2) +
            std::cos(deg2rad(from.lat)) * std::cos(deg2rad(to.lat)) *
            std::sin(dLon / 2) * std::sin(dLon / 2);

    float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    float d = R * c; // Distance in km

    return d*1000;
}

float Graph::short_dist(std::pair<float,float> from, std::pair<float, float> to) {
    float dlat, dlon;
    dlat = to.first - from.first;
    dlon = to.second - from.second;

    return std::sqrt(dlat * dlat + dlon * dlon);
}

void Graph::parser_osm(std::ifstream &in) {

    auto time = clock();

    std::string line;
    unsigned int number_nodes = 1;
    unsigned int number_ways = 1;

    while (std::getline(in, line)) {
        if (line.find("<node") != std::string::npos) {
            Node node;

            std::size_t id, lat, lon;
            id = line.find("id=") + 4;
            lat = line.find("lat=") + 5;
            lon = line.find("lon=") + 5;

            node.id = line.substr(id, line.find("\" ") - id);
            node.lat = std::stof(line.substr(lat, lon - 7 - lat));
            node.lon = std::stof(line.substr(lon, line.rfind("\"") - lon));

            map_inf_nodes.insert({node.id,number_nodes});
            map_nodes.insert({number_nodes,node});

            for (int i = 0; i < vector_route.size(); i++){
                auto dist = long_dist(vector_route[i], {node.lat, node.lon});
                if (vector_nearest[i].second > dist){
                    vector_nearest[i].second = dist;
                    vector_nearest[i].first = number_nodes;
                }
            }

            number_nodes++;

        } else if (line.find("<way") != std::string::npos) {

            Way way;
            size_t b = line.find("\"") + 1;
            way.id = line.substr(b, line.find("ver") - b - 2);

            bool k = true;
            unsigned int first_node;
            std::string l;

            while (true) {
                std::getline(in, line);
                if (line.find("</way") != std::string::npos) { break; }
                if (line.find("<nd") != std::string::npos) {
                    if (k) {
                        std::size_t a = line.find("ref=") + 5;
                        l = line.substr(a, line.rfind("\"") - a );
                        if (map_inf_nodes.find(l) != map_inf_nodes.end()){
                            k = false;
                            first_node = map_inf_nodes[l];
                            way.vector_nodes.push_back(first_node);
                        }
                    } else {
                        unsigned int second_node;
                        std::size_t a = line.find("ref=") + 5;
                        l = line.substr(a, line.rfind("\"") - a );
                        if (map_inf_nodes.find(l) != map_inf_nodes.end()){
                            second_node = map_inf_nodes[l];
                            float len = long_dist(map_nodes[first_node], map_nodes[second_node]);
                            map_edges[first_node].insert({second_node, len});
                            map_edges[second_node].insert({first_node, len});
                            way.vector_nodes.push_back(second_node);
                            first_node = second_node;
                        }
                    }
                } else if (line.find("<tag k=\"oneway\"") != std::string::npos && line.rfind("yes") != std::string::npos){
                    for (int x = 1; x < way.vector_nodes.size(); x++){
                        map_edges[way.vector_nodes[x]].erase(way.vector_nodes[x-1]);
                    }
                }
            }
            map_inf_ways.insert({way.id,number_ways});
            map_ways.insert({number_ways,way});
            number_ways++;

        }else if(line.find("<relation") != std::string::npos){
            unsigned int from = 0, to = 0, via = 0;
            while(1){
                std::getline(in, line);
                if (line.find("</relation") != std::string::npos) { break; };
                if (line.find("<mem") != std::string::npos){
                    size_t first,last;
                    first = line.find("\"");
                    last = line.rfind("\"");
                    std::string id = line.substr(line.find("ref=\"") + 5, line.rfind("role=") - line.find("ref=\"") -7);
                    if ( line[last-1] == 'a' ){
                        if (line[first+1] == 'w' && map_inf_ways.find(id) != map_inf_ways.end()){
                            via = map_inf_ways[id];
                            if (via == 0){
                                std::cout << "way_via: " << id << std::endl;
                                exit(5);
                            }
                        } else if (map_inf_nodes.find(id) != map_inf_nodes.end()){
                            via = map_inf_nodes[id];
                            if (via == 0){
                                std::cout << "node_via: " << id << std::endl;
                                exit(3);
                            }
                        }
                    } else if (line[last-1] == 'm'){
                        if (map_inf_ways.find(id) != map_inf_ways.end()){
                            from = map_inf_ways[id];
                            if (from == 0){
                                std::cout << "from: " << id << std::endl;
                                exit(5);
                            }
                        }
                    } else if (line[last-1] == 'o'){
                        if (map_inf_ways.find(id) != map_inf_ways.end()){
                            to = map_inf_ways[id];
                            if (to == 0){
                                std::cout << "to: " << id << std::endl;
                                exit(5);
                            }
                        }
                    }
                } else if (line.find("<tag") != std::string::npos){
                    size_t a = line.find("\"");
                    if (!strcmp(line.substr(a+1,11 ).c_str(), "restriction") && via != 0 && from != 0 && to != 0){
                        std::string rest;
                        rest = line.substr(line.find("v=\"")+3, line.rfind("\"") - line.find("v=\"")-3);
                        if (rest[0] == 'n'){
                            if (!strcmp(rest.c_str(), "no_entry")){
                                map_edges[via].erase(get_id_in_ways(to,via));
                            } else if (!strcmp(rest.c_str(), "no_exit")){
                                map_edges[from].erase(get_id_in_ways(from,via));
                            } else {
                                no_way[via].insert({get_id_in_ways(from, via), get_id_in_ways(to,via)});
                            }
                        } else if (rest[0] == 'o') {
                            auto from_node = get_id_in_ways(from, via);
                            auto to_node = get_id_in_ways(to,via);
                            if (!map_nodes[from_node].map_rest.empty()){
                                map_nodes[via].map_rest.insert({from_node, to_node});
                                one_way[from_node][to_node].insert({via,map_edges[from_node][via]});
                                map_edges[from_node][to_node] = map_edges[from_node][via] + map_edges[via][to_node];
                                std::queue<unsigned int> from_from;
                                from_from.push(from_node);
                                while(!from_from.empty()){
                                    auto u = from_from.front();
                                    from_from.pop();
                                    if (!map_nodes[u].map_rest.empty()){
                                        for (auto &v : map_nodes[u].map_rest){
                                            if (v.second == via){
                                                one_way[v.first][to_node] = one_way[v.first][via];
                                                one_way[v.first][to_node].insert({via,map_edges[v.first][via]});
                                                map_edges[v.first][to_node] = map_edges[v.first][via] + map_edges[via][to_node];
                                                map_edges[v.first].erase(via);
                                                from_from.push(v.first);
                                            }
                                        }
                                    }
                                }
                            }
                            if (!map_nodes[to_node].map_rest.empty() && map_nodes[to_node].map_rest.find(via) != map_nodes[to_node].map_rest.end()){
                                std::map<unsigned int, float,nodes_comp> map;
                                float len = map_edges[from_node][via];
                                map.insert({via, len});
                                map_nodes[via].map_rest.insert({from_node,to_node});
                                unsigned int via_to = via;
                                while(map_nodes[to_node].map_rest.find(via_to) != map_nodes[to_node].map_rest.end()){
                                    unsigned int to_to = map_nodes[to_node].map_rest[via_to];
                                    len += one_way[via_to][to_to][to_node];
                                    map.insert({to_node,len});
                                    via_to = to_node;
                                    to_node = to_to;
                                }
                                one_way[from_node][to_node].insert(map.begin(),map.end());
                                map_edges[from_node].insert({to_node, map[via_to] + map_edges[via_to][to_node]});
                            } else {
                                map_nodes[via].map_rest.insert({from_node, to_node});
                                one_way[from_node][to_node][via] = map_edges[from_node][via];
                                map_edges[from_node].insert({to_node,map_edges[from_node][via] + map_edges[via][to_node]});
                            }
                            map_edges[from_node].erase(via);
                        } else {
                            std::cout << "Unknown restriction" << std::endl;
                            exit(7);
                        }
                        from = 0, to = 0, via = 0;
                    }
                }
            }

        }
    }

    for(int i = 0; i < vector_route.size(); i++){
        Node node;
        node.lat = vector_route[i].first;
        node.lon = vector_route[i].second;

        map_nodes.insert({number_nodes,node});
        vector_dist.push_back(number_nodes);

        map_edges[number_nodes].insert(vector_nearest[i]);
        map_edges[vector_nearest[i].first].insert({number_nodes,vector_nearest[i].second});
        number_nodes++;
    }

    this->t_parse = clock() - time;
    this->n_nodes = number_nodes-1;
}

void Graph::out_graph() {

    // version for parse and future work, version for print
    /*
     * out creates *_graph. Format:
     *             name of map
     *             Nodes
     *             n of nodes
     *             number_node id_in_map lat lon
     *             ...
     *             Edges
     *             number_first_node number_second_node len
     *             ...
     *             Restriction
     *             No
     *             from via to
     *             ...
     *             ONLY
     *             from via to
     *             ...
     * */

    std::string dir_out = work_dir + "graph";

    mkdir(dir_out.c_str(), 0777);

    std::ofstream out;

    out.open(work_dir + "graph/" + name + "_graph");

    out << name << std::endl;
    out << "Nodes " << "\n" << n_nodes << std::endl;

    for (auto v : map_nodes){
        out << v.first << " " << v.second.id << " "  << v.second.lat << " " << v.second.lon << "\n";
    }

    out << "Edges " << std::endl;

    for (auto v : map_edges){
        for (auto w : v.second){
            out << map_nodes[v.first].id << " " << map_nodes[w.first].id << " " << map_nodes[w.second].id << "\n";
        }
    }

    out << "Restriction" << std::endl;
    out << "NO" << std::endl;

    for (auto rest : no_way){
        for (auto v : rest.second){
            out << map_nodes[v.first].id << " " << map_nodes[rest.first].id << " " << map_nodes[v.second].id << std::endl;
        }
    }

    out << "ONLY" << std::endl;

    for (auto rest : one_way){
        for (auto v : rest.second){
            for (auto w : v.second){
                out << map_nodes[rest.first].id << " " << map_nodes[w.first].id << " " << map_nodes[v.first].id << std::endl;
            }
        }
    }

    out.close();
}

void Graph::out_graph_for_print() {

    // version version for print
    /*
     * out_for_print creates *.dot
     * */

    std::string dir_out = work_dir + "graph";

    mkdir(dir_out.c_str(), 0777);

    std::ofstream out_print;

    out_print.open(work_dir + "graph/" + name + "_graph_for_print.dot");

    out_print << "graph {" << std::endl;

    for (auto v : map_nodes){
        out_print << "\t" << v.second.id << " [latitude =" << v.second.lat << ", longitude=" << v.second.lon << "]" << std::endl;
    }

    for (auto v : map_edges){
        for (auto w : v.second){
            out_print << "\t" << v.first << " -- " << w.first << std::endl;
        }
    }

    out_print << "}" << std::endl;

    out_print.close();
}

unsigned int Graph::get_id_in_ways(unsigned int first, unsigned int second) {

    unsigned int id;
    if (map_ways[first].vector_nodes[0] == second){
        id =  map_ways[first].vector_nodes[1];
    } else {
        id = map_ways[first].vector_nodes[map_ways[first].vector_nodes.size()-2];
    }
    if (id == 0){
        std::cout << "in " << map_ways[first].id << " " << map_nodes[second].id << std::endl;
        exit(8);
    }
    return id;
}

void Graph::short_way() {
    std::pair<float, std::vector<unsigned int>> c;

    auto time = clock();

    for(int i = 0; i < vector_dist.size()-1; i++){
        auto res = dijkstra(vector_dist[i],vector_dist[i+1]);
        c.first += res.first;
        std::copy(res.second.begin(), res.second.end(), back_inserter(c.second));
    }

    c.second.push_back(vector_dist[vector_dist.size()-1]);

    this->t_dijkstra = clock() - time;

//    output_to_osc(c.first, c.second);
    output_for_web(c.second);
}

std::pair<float, std::vector<unsigned int>> Graph::dijkstra(unsigned int source, unsigned int end) {
    std::priority_queue<vertex, std::vector<vertex>, vertex_comp> Q;
    std::unordered_map<unsigned int, float> dist;
    std::unordered_map<unsigned int, unsigned int> short_way;
    for (const auto &v : map_edges) { dist[v.first] =  std::numeric_limits<float>::infinity(); }

    Q.push({source, 0});
    while (!Q.empty()) {

        auto u = Q.top();
        Q.pop();

        if (u.second < dist[u.first]) {
            dist[u.first] = u.second;
        }

        auto no = 0;
        if (no_way.find(u.first) != no_way.end() && no_way[u.first].find(short_way[u.first]) != no_way[u.first].end()){
            no = no_way[u.first][short_way[u.first]];
        }
        for (const auto &v : map_edges[u.first]) {
            if (no == 0 || no != v.first){
                float alt = dist[u.first] + v.second;
                if (alt < dist[v.first]) {
                    Q.push({v.first, alt});
                    dist[v.first] = alt;
                    short_way[v.first] = u.first;
                }
            }
        }


        if (map_edges[u.first].find(end) == map_edges[u.first].end() && one_way.find(u.first) != one_way.end()){
            if (one_way[u.first].find(end) != one_way[u.first].end()){
                short_way[end] = u.first;
                for(auto &v : one_way[u.first]){
                    if (v.second.find(end) != v.second.end()){
                        dist[end] = dist[u.first] + v.second[end];
                        break;
                    }
                }
                u.first = end;
            }
        }

        if (u.first == end){
            std::vector<unsigned int> way;
            unsigned int v = end;
            while(1){
                auto w = short_way[v];
                if (one_way.find(w) != one_way.end() && one_way[w].find(v) != one_way[w].end()){
                    std::vector<unsigned int> way_2;
                    for (auto &r : one_way[w][v]){
                        way_2.push_back(r.first);
                    }
                    way.insert(way.begin(),way_2.begin(),way_2.end());
                }
                way.insert(way.begin(), w);
                v = w;
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

void Graph::output_to_osc(float dist, std::vector<unsigned int> way) {

    std::string dir_out = work_dir + "out_way";

    mkdir(dir_out.c_str(), 0777);                        //create a dir {0 - create, -1 - exits}

    time_t rawtime;
    struct tm* timeinfo;

    time(&rawtime);
    timeinfo = localtime (&rawtime);

    std::string name_file;
    name_file += std::to_string(timeinfo->tm_year + 1900);
    name_file += "_" + std::to_string(timeinfo->tm_mon + 1);
    name_file += "_" + std::to_string(timeinfo->tm_mday) ;
    name_file += "_" + std::to_string(timeinfo->tm_hour);
    name_file += ":" + std::to_string(timeinfo->tm_min);
    name_file += ":" + std::to_string(timeinfo->tm_sec);

    std::ofstream w(dir_out + "/" + name_file + ".osc");
    w << "<osmChange version=\"0.6\" generator=\"Norsi\">\n"
            "\t<create>\n\t<way id=\"-1\">\n";


    for (const auto &node : way) {
        w << "\t\t<nd ref=\"" << map_nodes[node].id << "\"/>" << std::endl;
    }

    w << "\t</way>\n\t<relation id=\"-2\">\n"
            "\t\t<member type=\"way\" ref=\"-1\" role=\"route\"/>\n"
            "\t\t<tag k=\"type\" v=\"route\"/>\n"
            "\t\t<tag k=\"route\" v=\"road\"/>\n"
            "\t\t<tag k=\"colour\" v=\"red\"/>\n"
            "\t\t<tag k=\"distance\" v=\"" << dist << "\"/>\n"
              "\t</relation>\n\t</create>\n</osmChange>";
    w.close();
}

void Graph::output_for_web(std::vector<unsigned int> way) {
    std::ofstream f("../out/points_cpu.txt");

    f << (float)t_parse/CLOCKS_PER_SEC << std::endl;           //Time of parsing in sec
    f << (float)t_dijkstra/CLOCKS_PER_SEC << std::endl;        // Time of dijkstra in sec

    for (auto &n : way){
        f << map_nodes[n].lat << " " << map_nodes[n].lon << std::endl;
    }

    f.close();
}