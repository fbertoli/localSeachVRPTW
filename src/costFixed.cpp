//
// Created by Bertoli, Francesco (Data61, Canberra City) on 02/02/17.
//


#include "costFixed.h"



double CostFixed::computeDelta(Solution& solution, CrossReverseMove &move) {
    return -fixed_cost_* static_cast<int>(move.route_removed_);
}


/** -------------------------------------------------------------------------------- */



double CostFixed::computeCost(Solution &solution) {
    return fixed_cost_*solution.n_routes_;
}


/** -------------------------------------------------------------------------------- */



void CostFixed::addRoutesCost(Solution &solution, vector<double> &routes_costs, vector<int> routes_indexes) {
    if (routes_indexes.size()) {
        for (auto & route : routes_indexes)
            routes_costs[route] += fixed_cost_;;
    }
    else {
        for (auto &el : routes_costs)
            el += fixed_cost_;
    }
}
