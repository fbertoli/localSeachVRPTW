//
// Created by Bertoli, Francesco (Data61, Canberra City) on 02/02/17.
//

#include "costDistance.h"



double CostDistance::computeDelta(Solution& solution, RelocateMove &move) {
    int visit = solution.tour_[move.old_position_];
    return data_.true_distances_[solution.tour_[move.new_predecessor_index_]][visit] +
           data_.true_distances_[visit][solution.tour_[move.new_predecessor_index_ + 1]] -
           data_.true_distances_[solution.tour_[move.new_predecessor_index_]][solution.tour_[move.new_predecessor_index_ + 1]] +
           data_.true_distances_[solution.tour_[move.old_position_ - 1]][solution.tour_[move.old_position_ + 1]] -
           data_.true_distances_[solution.tour_[move.old_position_ - 1]][visit] -
           data_.true_distances_[solution.tour_[visit]][solution.tour_[move.old_position_ + 1]];
}

/** -------------------------------------------------------------------------------- */

double CostDistance::computeDelta(Solution& solution, CrossReverseMove &move) {
    return move.delta_distance_;
}


/** -------------------------------------------------------------------------------- */

double CostDistance::computeCost(Solution &solution) {
    double cost = 0;
    for (int i = 0; i < solution.tour_.size() - 1; ++i)
        cost += data_.true_distances_[solution.tour_[i]][solution.tour_[i+1]];
    return cost;
}


/** -------------------------------------------------------------------------------- */

void CostDistance::addRoutesCost(Solution& solution, vector<double> &routes_costs, vector<int> routes_indexes) {
    if (routes_indexes.size()) {
        for (auto & route : routes_indexes)
            for (int i = solution.start_positions_[route]; i < solution.start_positions_[route + 1]; ++i)
                routes_costs[route] += data_.true_distances_[solution.tour_[i]][solution.tour_[i + 1]];
    }
    else {
        for (int route = 0; route < solution.n_routes_; ++route)
            for (int i = solution.start_positions_[route]; i < solution.start_positions_[route + 1]; ++i)
                routes_costs[route] += data_.true_distances_[solution.tour_[i]][solution.tour_[i + 1]];
    }
}


/** -------------------------------------------------------------------------------- */
