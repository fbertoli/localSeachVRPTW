//
// Created by Bertoli, Francesco (Data61, Canberra City) on 15/02/17.
//

#ifndef LOCALSEARCH_PARSEINPUT_H
#define LOCALSEARCH_PARSEINPUT_H

#include "parameter.h"
#include <vector>
#include "cost.h"
#include "move.h"
#include "data.h"
#include "moveGenerator.h"


void parseConfigFile(string config_file_name, vector<Cost*> &possible_costs, vector<Cost*> &cost_component_solution, vector<Cost*> &cost_component_route, vector<Move*> &possible_moves, vector<Move*> &moves,  vector<Move*> &possible_best_moves, vector<Move*> &best_moves, vector<MoveGenerator*> &possible_generators, vector<MoveGenerator*> &generators, Data data);

#endif //LOCALSEARCH_PARSEINPUT_H
