#pragma once

//
// Created by Bertoli, Francesco (Data61, Canberra City) on 31/03/17.
//

#include "data.h"
#include "initializer.h"
#include "solution.h"
#include "cost.h"
#include "move.h"
#include "moveGenerator.h"

class Metaheuristic {

/** CONSTRUCTORS */
public:
    Metaheuristic(Data &data, Initializer *initializer, vector<Cost*> &cost_components_solution, vector<Cost*> &cost_components_route, vector<Move*> &moves,  vector<Move*> &best_moves, vector<MoveGenerator*> &generators):
            data_(data),
            best_solution_(data),
            initializer_(initializer),
            cost_components_solution_(cost_components_solution),
            cost_components_route_(cost_components_route),
            moves_(moves),
            best_moves_(best_moves),
            generators_(generators),
            iteration_(0)
    {}



    /** METHODS */
public:
    /** run the methaheuristic */
    virtual void run() = 0;

    /** print additional indoramtion */
    virtual void printOuput() = 0;


    /** VARIABLES */
public:
    /** data of the instance */
    Data &data_;

    /** the heuristic to initialize the solution */
    Initializer *initializer_;


    /** the best solution */
    Solution best_solution_;

    /** the current iteration number */
    int iteration_;


    /** the cost objects determining the total cost of solutions and routes*/
    vector<Cost*> &cost_components_solution_;
    vector<Cost*> &cost_components_route_;


    /** the possible Move objects involved in the exploration pof the neighbourhoods */
    vector<Move*> &moves_;
    vector<Move*> &best_moves_;


    /** the current chosen Move object and best_move*/
    Move *move_;
    Move *best_move_;

    /** the generators to explore the neighbourhoods */
    vector<MoveGenerator*> &generators_;


    /**  the current chosen generator*/
    MoveGenerator *generator_;
};

