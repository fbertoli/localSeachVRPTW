//
// Created by Bertoli, Francesco (Data61, Canberra City) on 15/02/17.
//

#include "parameter.h"
#include <vector>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include "parse.h"

using namespace std;


void parseConfigFile(string config_file_name, vector<Cost*> &possible_costs, vector<Cost*> &cost_component_solution, vector<Cost*> &cost_component_route, vector<Move*> &possible_moves, vector<Move*> &moves,  vector<Move*> &possible_best_moves, vector<Move*> &best_moves, vector<MoveGenerator*> &possible_generators, vector<MoveGenerator*> &generators, Data data)
{
    // READ FILE
    string line;
    ifstream config_file(config_file_name);
    vector<vector<string>> lines;
    if (config_file.is_open()) {
        while (getline(config_file, line)) {
            istringstream iss(line);
            vector<string> tokens{istream_iterator<string>{iss},
                                  istream_iterator<string>{}};
            lines.push_back(tokens);
        }
        config_file.close();
    }

    // READ INFO
    for (int i = 0; i < lines.size(); ++i) {
        if (lines[i].size() && lines[i][0] == "cost_solution:") {
            for (auto & el : lines[i])
                for (auto & cost: possible_costs)
                    if (el==(cost -> getName()))
                        cost_component_solution.push_back(cost);

        }
        if (lines[i].size() && lines[i][0] == "cost_route:") {
            for (auto & el : lines[i])
                for (auto & cost: possible_costs)
                    if (el==(cost -> getName()))
                        cost_component_route.push_back(cost);

        }
        if (lines[i].size() && lines[i][0] == "moves:") {
            for (auto & el : lines[i]) {
                for (int j = 0; j < possible_moves.size(); ++j) {
                    if (possible_moves[j]->getName() == el) {
                        moves.push_back(possible_moves[j]);
                        best_moves.push_back(possible_best_moves[j]);
                        generators.push_back(possible_generators[j]);
                    }
                }
            }
        }
    }
}