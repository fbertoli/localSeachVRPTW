//
// Created by Bertoli, Francesco (Data61, Canberra City) on 02/02/17.
//

#include "costDistance.h"


/** -------------------------------------------------------------------------------- */

double CostDistance::computeDelta(Solution& solution, CrossReverseMove &move) {
    return move.delta_distance_;
}


/** -------------------------------------------------------------------------------- */

double CostDistance::computeCost(Solution &solution) {
    double cost = 0;
    for (int i = 0; i < solution.tour_.size() - 1; ++i)
        cost += data_.distances_[solution.tour_[i]][solution.tour_[i+1]];
    return cost;
}


/** -------------------------------------------------------------------------------- */

void CostDistance::addRoutesCost(Solution& solution, vector<double> &routes_costs, vector<int> routes_indexes) {
    if (routes_indexes.size()) {
        for (auto & route : routes_indexes)
            for (int i = solution.start_positions_[route]; i < solution.start_positions_[route + 1]; ++i)
                routes_costs[route] += data_.distances_[solution.tour_[i]][solution.tour_[i + 1]];
    }
    else {
        for (int route = 0; route < solution.n_routes_; ++route)
            for (int i = solution.start_positions_[route]; i < solution.start_positions_[route + 1]; ++i)
                routes_costs[route] += data_.distances_[solution.tour_[i]][solution.tour_[i + 1]];
    }
}


/** -------------------------------------------------------------------------------- */
