#ifndef read_file
#define read_file

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <filesystem>

#include "classes.h"

namespace fs = std::filesystem;

int menu(std::vector<std::string> & inputs) {
    int i = 1, op;
    std::cout << "=================================" << std::endl;
    for(const auto & in : inputs) {
        std::cout << (i++) << " - " << in << std::endl;
    }

    std::cin >> op;

    return op;
}

void remove_space(std::string & line) {
    line.erase(remove_if(line.begin(), line.end(), isspace), line.end());
}

void get_inputs(std::vector<std::string> & inputs, std::string file_path) {
    // std::cout << fs::current_path() << std::endl;
    std::string path = fs::current_path().string() + file_path;

    for (const auto & entry : fs::directory_iterator(path)) {
        std::ifstream input(entry.path());

        
        std::string path_at = entry.path().string();
        std::replace(path_at.begin(), path_at.end(), '\\', '/');

        inputs.push_back(path_at);
    }
}

bool read_txt(std::string file, std::vector<Node> & nodes, Fleet & fleet, int & node_number, Node & depot, double & max_distance, double & service_cost) {
    int start, end, id;
    double capacity, x, y;
    
    std::cout << file << std::endl;
    std::ifstream in(file);
    if(not in.is_open()) {
        std::cerr << "Erro ao ler arquivo de entrada" << std::endl;
        return false;
    }

    std::string line;

    std::getline(in, line); // NAME
    std::getline(in, line); // COMMENT
    std::getline(in, line); // TYPE
    std::getline(in, line); // DIMENSION
    std::getline(in, line); // EDGE_WEIGHT_TYPE
    std::getline(in, line); // CAPACITY
    remove_space(line);
    sscanf(line.c_str(), "CAPACITY:%lf", &capacity);
    std::getline(in, line);  // NODE_COORD_SECTION ou DISTANCE

    if(line[0] != 'N') { // CMT06, CMT07, CMT08, CMT09, CMT10, CMT13, CMT14
        double md, sc;
        sscanf(line.c_str(), "DISTANCE : %lf", &md);
        std::getline(in, line); // SERVICE_TIME : 50
        sscanf(line.c_str(), "SERVICE_TIME : %lf", &sc);
        max_distance = md, service_cost = sc;

        std::getline(in, line); // NODE_COORD_SECTION 
    }
    
    fleet = Fleet(0, 0, capacity);

    std::getline(in, line);
    sscanf(line.c_str(), "%d %lf %lf", &id, &x, &y);
    depot = Node(x, y, id-1);

    std::getline(in, line);
    while(in.good() && line[0] != 'D') {
        sscanf(line.c_str(), "%d %lf %lf", &id, &x, &y);

        nodes.push_back(Node(x, y, id-2));

        std::getline(in, line);
    }

    std::getline(in, line);

    while(in.good() && line[0] != 'D') {
        sscanf(line.c_str(), "%d %lf", &id, &capacity);

        nodes[id-2].request = capacity;

        std::getline(in, line);
    }

    node_number = (int)nodes.size();
    return true;
}

#endif