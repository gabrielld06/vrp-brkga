#ifndef classes
#define classes

#include <vector>

class Node {
    public:
    int index;
    double x, y, request, code;

    Node() {
        this->index = -1;
        this->x = 0;
        this->y = 0;
        this->request = 0;
        this->code = 0;
    }

    Node(double x, double y) {
        this->index = -1;
        this->x = x;
        this->y = y;
        this->request = 0;
        this->code = 0;
    }

    Node(double x, double y, int idx) {
        this->index = idx;
        this->x = x;
        this->y = y;
        this->request = 0;
        this->code = 0;
    }

    bool operator < (const Node & r) {
        return this->code < r.code;
    }
};

class Offspring {
    public:
    std::vector<Node> chromossome;
    std::vector<Node> ordered_chromossome;
    std::vector<std::vector<Node>> routes;
    double cost;

    Offspring(std::vector<Node> chromossome, double cost) {
        this->chromossome = chromossome;
        this->cost = cost;
    }

    bool operator < (const Offspring & r) {
        return this->cost < r.cost;
    }
};

class Fleet {
    public:
    int start_node, end_node;
    double capacity;

    Fleet() {
        this->start_node = 0;
        this->end_node = 0;
        this->capacity = 0;
    }

    Fleet(int start_node, int end_node, double capacity) {
        this->start_node = start_node;
        this->end_node = end_node;
        this->capacity = capacity;
    }
};

using classes::Node;
using classes::Fleet;

#endif