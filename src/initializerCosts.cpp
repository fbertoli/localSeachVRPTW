//
// Created by Bertoli, Francesco (Data61, Canberra City) on 28/02/17.
//

#include "options.h"
#include "data.h"

/** parameters for the computation of costs */
Option<double> alpha("alpha", "alpha weight for inertion heuristic I1", 0.5);
Option<double> mu("mu", "mu weight for inertion heuristic I1", 1);
Option<double> lambda("lambda", "lambda weight for inertion heuristic I1", 1);



/** compute the cost of inserting visit bewteen predecessor and successor with the gives loss in departure time */
double positionCost(int predecessor, int visit, int successor, double lost_departure_time, Data& data){
    return alpha * (data.true_distances_[predecessor][visit] + data.true_distances_[visit][successor] - mu * data.true_distances_[predecessor][successor]) +
           (1 - alpha) * lost_departure_time;
}

/** ------------------------------------------------------------------------------------------------ */


/** compute the cost of the insertion (only needed if we want a double criteria) */
double insertionCost(int customer, double best_position_cost, int switcher, vector<double> *profits, Data& data) {
    if (switcher == 0)
        return lambda * data.true_distances_[data.depot_][customer] - best_position_cost;
    else
        return lambda * profits->at(customer) - best_position_cost;
}