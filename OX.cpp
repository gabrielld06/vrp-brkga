#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <random>
#include <algorithm>
#include <filesystem>
#include <ctime>

#include <omp.h>
#include <unistd.h>

namespace fs = std::filesystem;

#include "include/classes.h"
#include "include/read_file.h"

// PARAMS
#define MAX_GEN 1000
#define POP_SIZE 1000
#define ELITE_SIZE 250
#define MUT_SIZE 300
#define INHERITANCE_CHANCE 0.7

int node_number = 0;
double max_distance, service_cost;

std::random_device dev;
std::mt19937 rng(dev());
std::uniform_int_distribution<std::mt19937::result_type> sel_elite(0,ELITE_SIZE-1);
std::uniform_int_distribution<std::mt19937::result_type> sel_non_elite(ELITE_SIZE,POP_SIZE-1);
std::uniform_int_distribution<std::mt19937::result_type> dist(0,100);
std::uniform_real_distribution<double> dist_real(0.0, 1.0);

std::vector<Node> nodes;
Fleet fleet;
Node depot;

void random_chromossome(std::vector<Node> & c) {
    for(int i = 0;i < c.size();i++) {
        c[i].code = dist_real(rng);
    }
}

std::vector<Node> crossover(std::vector<Node> & father, std::vector<Node> & mother) {
    std::vector<Node> offspring(father.size(), Node(0, 0));

    for(int i = 0;i < father.size();i++) {
        offspring[i] = (dist_real(rng) < INHERITANCE_CHANCE ? father[i] : mother[i]);
    }

    return offspring;
}

std::pair<std::vector<Node>, std::vector<Node>> ox(Offspring & parent_1, Offspring & parent_2) {
    std::uniform_int_distribution<std::mt19937::result_type> cutpoints(0, parent_1.chromossome.size()-1);
    int cut1 = cutpoints(rng);
    int cut2 = cutpoints(rng);
    while(cut2 == cut1) cut2 = cutpoints(rng);

    if(cut2 < cut1) std::swap(cut1, cut2);

    std::vector<Node> child_1 = parent_1.chromossome;
    std::vector<Node> child_2 = parent_2.chromossome;

    bool seen_1[parent_1.chromossome.size()] = { false };
    bool seen_2[parent_2.chromossome.size()] = { false };

    for(int i = cut1; i <= cut2;i++) {
        seen_1[parent_1.ordered_chromossome[i].index] = true;
        seen_2[parent_2.ordered_chromossome[i].index] = true;
    }

    int j = (cut2 + 1) % parent_1.chromossome.size();
    for(int i = 0;i < parent_1.chromossome.size();i++) {
        if(not seen_1[parent_2.ordered_chromossome[i].index]) {
            child_1[parent_2.ordered_chromossome[i].index].code = parent_1.ordered_chromossome[j].code;
            j = (j + 1) % parent_1.chromossome.size();
        }
    }

    j = (cut2 + 1) % parent_1.chromossome.size();
    for(int i = 0;i < parent_1.chromossome.size();i++) {
        if(not seen_2[parent_1.ordered_chromossome[i].index]) {
            child_2[parent_1.ordered_chromossome[i].index].code = parent_2.ordered_chromossome[j].code;
            j = (j + 1) % parent_1.chromossome.size();
        }
    }

    return std::make_pair(child_1, child_2);
}

double eucl_distance(double x1, double y1, double x2, double y2) {
    double d_x = (x1 - x2);
    d_x *= d_x;
    double d_y = (y1 - y2);
    d_y *= d_y;

    return sqrt(d_x + d_y);
}

void reverse_path(std::vector<Node> & path, std::vector<Node> & path_original, int i, int j) {
    double aux;
	for(int k = 0;k < ((j+1-i)/2);k++) {
		aux = path_original[path[j-k].index].code;
		path_original[path[j-k].index].code = path_original[path[i+k].index].code;
		path_original[path[i+k].index].code = aux;
	}
}

double twoOptImprovement(std::vector<Node> & path, int i, int k, int start, int end) {
    Node b = (i == start ? depot : path[i-1]); // Cidade inicial
    Node e = (k == end ? depot : path[k+1]); // Cidade final
	
	if((i == start and k == end) or b.index == e.index) return 1; // Situações que representam o ciclo inteiro
	
	double d0 = eucl_distance(b.x, b.y, path[i].x, path[i].y) + eucl_distance(path[k].x, path[k].y, e.x, e.y);
	double d1 = eucl_distance(b.x, b.y, path[k].x, path[k].y) + eucl_distance(path[i].x, path[i].y, e.x, e.y);

	return -d0 + d1;
}

double bestImprovement(std::vector<Node> & path, std::vector<Node> & path_original, double & opt, int start, int end) {
	double d, best = 0;
    int best_i, best_j;
    
    for(int i = start; i < end;i++) {
        for(int j = i+1;j < end;j++) {
            d = twoOptImprovement(path, i, j, start, end-1);
            
            if(d < best) {
                best = d, best_i = i, best_j = j;
            }
        }
    }

    if(best < 0) reverse_path(path, path_original, best_i, best_j);

    return best;
}

double decoder(Offspring & off) {
    std::vector<Node> offspring = off.chromossome;

    sort(offspring.begin(), offspring.end());

    double cost = eucl_distance(depot.x, depot.y, offspring[0].x, offspring[0].y);
    double capacity = fleet.capacity-offspring[0].request;
    double distance;

    double route_cost = cost;
    int route_start = 0;

    for(int i = 1;i < offspring.size();i++) {
        if(capacity-offspring[i].request >= 0) {
            distance = eucl_distance(offspring[i-1].x, offspring[i-1].y, offspring[i].x, offspring[i].y);
            cost += distance;
            route_cost += distance;
            capacity -= offspring[i].request;
        } else {
            distance = eucl_distance(offspring[i-1].x, offspring[i-1].y, depot.x, depot.y);
            cost += distance;
            route_cost += distance;

            double improvement = bestImprovement(offspring, off.chromossome, route_cost, route_start, i);

            cost += improvement;
            route_start = i;

            distance = eucl_distance(depot.x, depot.y, offspring[i].x, offspring[i].y);
            cost += distance;
            route_cost = distance;
            capacity = fleet.capacity-offspring[i].request;
        }
    }
    cost += eucl_distance(offspring.back().x, offspring.back().y, depot.x, depot.y);

    off.ordered_chromossome = off.chromossome;
    sort(off.ordered_chromossome.begin(), off.ordered_chromossome.end());

    return cost;
}

void generate_population(std::vector<Offspring> & pop) {
    #pragma omp parallel for
    for(int i = 0;i < POP_SIZE;i++) {
        random_chromossome(pop[i].chromossome);
        pop[i].cost = decoder(pop[i]);
    }
}

void print_chromossome(std::vector<Node> c) {
    for(auto & ch : c) {
        std::cout << ch.x << " " << ch.y << " " << ch.code << " ";
    }
    std::cout << std::endl;
}

Offspring genetic() {
    std::vector<Offspring> population = std::vector<Offspring>(POP_SIZE, Offspring(nodes, 0));
    std::vector<Offspring> new_population = std::vector<Offspring>(POP_SIZE, Offspring(nodes, 0));

    generate_population(population);

    double bst = 0;

    int gen = 0;
    int last_change = 0;
    while(last_change < MAX_GEN) {
        if(gen % 2 == 0) {
            sort(population.begin(), population.end());

            // melhor
            if(bst != population[0].cost) {
                last_change = 0;
                bst = population[0].cost;
            }

            // manutenção da elite
            int e = 1;
            new_population[0] = population[0];
            for(int i = 1;i < ELITE_SIZE;i++) {
                while(e < POP_SIZE && new_population[i-1].cost == population[e].cost) {
                    e++;
                }
                new_population[i] = population[e];
            }

            // seleção e crossover
            #pragma omp parallel for
            for(int i = ELITE_SIZE;i < POP_SIZE - MUT_SIZE;i+=2) {
                int father = sel_elite(rng);
                int mother = sel_non_elite(rng);
                
                std::pair<std::vector<Node>, std::vector<Node>> p = ox(new_population[father], population[mother]);
                new_population[i].chromossome = p.first;
                new_population[i].cost = decoder(new_population[i]);

                new_population[i+1].chromossome = p.second;
                new_population[i+1].cost = decoder(new_population[i+1]);
            }
            
            // mutação
            #pragma omp parallel for
            for(int i = POP_SIZE - MUT_SIZE;i < POP_SIZE;i++) {
                random_chromossome(new_population[i].chromossome);
                new_population[i].cost = decoder(new_population[i]);
            }
        } else {
            sort(new_population.begin(), new_population.end());

            // melhor indivíduo
            if(bst != new_population[0].cost) {
                last_change = 0;
                bst = new_population[0].cost;
            }

            // manutenção da elite
            int e = 1;
            population[0] = new_population[0];
            for(int i = 1;i < ELITE_SIZE;i++) {
                while(e < POP_SIZE && population[i-1].cost == new_population[e].cost) {
                    e++;
                }
                population[i] = new_population[e];
            }

            // seleção e crossover
            #pragma omp parallel for
            for(int i = ELITE_SIZE;i < POP_SIZE - MUT_SIZE;i+=2) {
                int father = sel_elite(rng);
                int mother = sel_non_elite(rng);
                
                std::pair<std::vector<Node>, std::vector<Node>> p = ox(population[father], new_population[mother]);

                population[i].chromossome = p.first;
                population[i].cost = decoder(population[i]);

                population[i+1].chromossome = p.second;
                population[i+1].cost = decoder(population[i+1]);
            }
            
            // mutação
            #pragma omp parallel for
            for(int i = POP_SIZE - MUT_SIZE;i < POP_SIZE;i++) {
                random_chromossome(population[i].chromossome);
                population[i].cost = decoder(population[i]);
            }
        }
        gen++;
        last_change++;
    }
    
    sort(population.begin(), population.end());

    return population[0];
}

int main() {
    // coletar entrada e escolher instância
    int c;
    std::cout << "0 - CMT\n1 - Uchoa\n";
    std::cin >> c;

    std::string ds = (c == 0 ? "CMT" : "Uchoa");
    
    std::vector<std::string> inputs;
    read_file::get_inputs(inputs, "/dataset/" + ds);

    int op = read_file::menu(inputs);
    while(!(0 < op && op < inputs.size())) {
        op = menu(inputs);
    }
    
    nodes = std::vector<Node>();
    fleet = Fleet();
    depot = Node();
    node_number = 0;
    max_distance = std::numeric_limits<double>::infinity(), service_cost = 0;

    // ler arquivo de entrada
    std::string input_file = inputs[op-1];
    read_file::read_txt(input_file, nodes, fleet, node_number, depot, max_distance, service_cost);

    // executar algoritmo
    std::cout << "Iniciando BRKGA para " + (op < 10 ? "0" + std::to_string(op) : std::to_string(op)) << std::endl;
    double start = omp_get_wtime();
    Offspring offspring = genetic();
    double end = omp_get_wtime();

    std::cout << std::fixed << offspring.cost << " em " << (end - start) << " segundos" << std::endl;

    return 0;
}