//
// Created by Bertoli, Francesco (Data61, Canberra City) on 02/02/17.
//

#include "costCapacity.h"


double CostCapacity::computeDelta(Solution& solution, RelocateMove &move) {
    double cost = 0;

    // ORIGIN ROUTE
    int load = 0;
    for (int i = solution.start_positions_[move.origin_route_] + 1; i < solution.start_positions_[move.origin_route_+1]; ++i)
        load += data_.demands_[solution.tour_[i]];
    cost -= penalization_*(max(load, data_.capacity_) - max(load - data_.demands_[solution.tour_[move.old_position_]], data_.capacity_));

    // DESTINATION ROUTE
    load = 0;
    if (move.destination_route_ < solution.n_routes_)
        for (int i = solution.start_positions_[move.destination_route_] + 1; i < solution.start_positions_[move.destination_route_+1]; ++i)
            load += data_.demands_[solution.tour_[i]];
    cost += penalization_*(max(load + data_.demands_[solution.tour_[move.old_position_]], data_.capacity_) - max(load, data_.capacity_));

    return cost;
}


/** -------------------------------------------------------------------------------- */

double CostCapacity::computeDelta(Solution& solution, CrossReverseMove &move) {
    return move.delta_capacity_*penalization_;
}

/** -------------------------------------------------------------------------------- */

double CostCapacity::computeCost(Solution &solution) {
    vector<int> routes_load;
    computeRoutesLoad(solution, routes_load);
    double cost = 0;
    for (const auto &el : routes_load)
        cost += max(0,el - data_.capacity_)*penalization_;
    return cost;
}

/** -------------------------------------------------------------------------------- */

void CostCapacity::addRoutesCost(Solution& solution, vector<double> &routes_costs, vector<int> routes_indexes) {
    vector<int> routes_load;
    computeRoutesLoad(solution, routes_load, routes_indexes);
    if (routes_indexes.size()) {
        for (auto & route : routes_indexes) {
            routes_costs[route] += max(routes_load[route] - data_.capacity_,0)*penalization_;
        }
    }
    else {
        for (int route = 0; route < solution.n_routes_; ++route) {
            routes_costs[route] += max(routes_load[route] - data_.capacity_, 0) * penalization_;
        }
    }
}


/** -------------------------------------------------------------------------------- */


void CostCapacity::computeRoutesLoad(Solution& solution, vector<int> &routes_load, vector<int> routes_indexes)
{
    if (routes_indexes.size()) {
        for (auto & route : routes_indexes) {
            int load = 0;
            for (int position = (solution.start_positions_[route] + 1);
                 position < solution.start_positions_[route + 1]; ++position) {
                load += solution.data_.demands_[solution.tour_[position]];
            }
            routes_load.push_back(load);
        }
    }
    else {
        for (int route = 0; route < solution.n_routes_; ++route) {
            int load = 0;
            for (int position = (solution.start_positions_[route] + 1);
                 position < solution.start_positions_[route + 1]; ++position) {
                load += solution.data_.demands_[solution.tour_[position]];
            }
            routes_load.push_back(load);
        }
    }
}
