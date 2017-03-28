//
// Created by Bertoli, Francesco (Data61, Canberra City) on 02/02/17.
//
#include "solution.h"
#include <iostream>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <cassert>
#include "gurobi_c++.h"


/** STATIC MEMEBERS INITIALIZATION */
double Solution::capacity_coefficient_ = 1;




/** CONSTRUCTORS */
Solution::Solution(const Data &data, vector<int> tour, vector<double> arrivals, vector<double> departures, vector<double> latest_departures, vector<int> start_positions) :
        data_(data),
        tour_(tour),
        start_positions_(start_positions),
        arrivals_(arrivals),
        departures_(departures),
        latest_departures_(latest_departures),
        n_routes_(tour.back() - data.n_requests_),
        cost_(0)
{}


/** ------------------------------------------------------------------------------------------------ */


Solution::Solution(const Data &data, vector<int> tour, vector<double> arrivals) :
        data_(data),
        tour_(tour),
        n_routes_(tour.back() - data.n_requests_),
        arrivals_(arrivals),
        cost_(0)

{
    departures_.resize(arrivals_.size(),0);
    latest_departures_.resize(arrivals_.size());
    for (int i = 0; i < tour.size(); ++i)
        if (tour[i] >= data_.n_requests_)
            start_positions_.push_back(i);
        else
            departures_[tour_[i]] = max(arrivals_[tour[i]], data.start_TW_[tour[i]]) + data_.service_time_[tour[i]];

    for (int r = 0; r < n_routes_; ++r) {
        latest_departures_[tour_[start_positions_[r + 1]]] = data_.latest_departure_possible_.back(); //depot end_TW, this will be overwritten in the next iteration by the for below
        for (int i = start_positions_[r + 1] - 1; i >= start_positions_[r]; --i)
            latest_departures_[tour_[i]] = min(data_.latest_departure_possible_[tour_[i]], latest_departures_[tour_[i + 1]] -
                                                                        data_.true_distances_[tour_[i]][tour_[i + 1]] - data_.service_time_[tour_[i+1]]);
    }
    latest_departures_[tour_[start_positions_.back()]] = 0;
}


/** ------------------------------------------------------------------------------------------------ */


Solution::Solution(const Data &data, vector<int> tour) :
        data_(data),
        tour_(tour),
        start_positions_{0},
        cost_(0)
{
    arrivals_.resize(tour_.size());
    departures_.resize(tour_.size(),0);
    latest_departures_.resize(tour_.size());
    n_routes_ = tour.back() - data.n_requests_;

    for (int i = 1; i < tour.size(); ++i) {
        arrivals_[tour_[i]] = departures_[tour_[i-1]] + data_.true_distances_[tour_[i-1]][tour_[i]];
        if (tour[i] >= data_.n_requests_)
            start_positions_.push_back(i);
        else
            departures_[tour_[i]] = max(arrivals_[tour[i]], data.start_TW_[tour[i]]) + data_.service_time_[tour[i]];
    }

    for (int r = 0; r < n_routes_; ++r) {
        latest_departures_[tour_[start_positions_[r + 1]]] = data_.latest_departure_possible_.back(); //depot end_TW, this will be overwritten in the next iteration by the for below
        for (int i = start_positions_[r + 1] - 1; i >= start_positions_[r]; --i)
            latest_departures_[tour_[i]] = min(data_.latest_departure_possible_[tour_[i]], latest_departures_[tour_[i + 1]] -
                                                                                           data_.true_distances_[tour_[i]][tour_[i + 1]] - data_.service_time_[tour_[i+1]]);
    }
    latest_departures_[tour_[start_positions_.back()]] = 0;
}


/** ------------------------------------------------------------------------------------------------ */

Solution& Solution::operator= (const Solution &solution) {
    assert(&solution.data_ == &data_);
    tour_ = solution.tour_;
    arrivals_ = solution.arrivals_;
    departures_ = solution.departures_;
    latest_departures_ = solution.latest_departures_;
    n_routes_ = solution.n_routes_;
    start_positions_ = solution.start_positions_;
    cost_ = solution.cost_;
    routes_costs_ = solution.routes_costs_;
//    tsp_time_limit_ = solution.tsp_time_limit_;
    return *this;
}


/** ------------------------------------------------------------------------------------------------ */
/** ------------------------------------------------------------------------------------------------ */


/** METHODS */
void Solution::insertCustomer(int route, int visit, int pred_index)
{
    if (route < n_routes_) { //customer has to be inserted in an existing route
        // insert customer and compute arrival/departure
        tour_.insert(tour_.begin() + pred_index + 1, visit);
        arrivals_[visit] = departures_[tour_[pred_index]] + data_.true_distances_[tour_[pred_index]][visit];
        departures_[visit] =  departureTime(visit);
        // if next visit is not depot pull backward latest_departures
        latest_departures_[visit] = (pred_index + 1 < start_positions_[route+1]) ?  min(data_.latest_departure_possible_[visit], latest_departures_[tour_[pred_index + 2]] - data_.true_distances_[visit][tour_[pred_index + 2]] - data_.service_time_[tour_[pred_index + 2]]) : data_.end_TW_[visit] + data_.service_time_[visit];
        // update start_position
        for (int j = route + 1; j <= n_routes_; ++j)
            ++start_positions_[j];

        // update arrival of rest of the route
        for (int j = pred_index + 2; j <start_positions_[route + 1]; ++j) {
            int current_visit = tour_[j];
            arrivals_[current_visit] = departures_[tour_[j-1]] + data_.true_distances_[tour_[j - 1]][current_visit];
            // if we still arrive before time windows then there is no need to modify the rest of the route
            if (arrivals_[current_visit] <= data_.start_TW_[current_visit])
                break;
            else
                departures_[current_visit] = departureTime(current_visit);
        }

        // update latest_departure for positions before
        for (int j = pred_index; j >= start_positions_[route]; --j ) {
            int current_visit = tour_[j];
            // if the latest departure does not have to be updated (it is still end_tw + serivce_time) then skip updating rest of the route
            if (data_.latest_departure_possible_[current_visit] <= latest_departures_[tour_[j+1]] - data_.true_distances_[current_visit][tour_[j+1]] - data_.service_time_[tour_[j+1]])
                break;
            else
                latest_departures_[current_visit] = latest_departures_[tour_[j+1]] - data_.true_distances_[current_visit][tour_[j+1]] - data_.service_time_[tour_[j+1]];
        }


        // update arrival time at depot
        arrivals_[tour_[start_positions_[route + 1]]] = departures_[tour_[start_positions_[route + 1]-1]] + data_.true_distances_[tour_[start_positions_[route + 1]-1]][tour_[start_positions_[route + 1]]];
    }

    else { // add new route
        tour_.push_back(visit);
        arrivals_[visit] = data_.start_TW_.back() + data_.true_distances_[tour_[0]][visit];
        departures_[visit] = departureTime(visit);
        latest_departures_[visit] = data_.latest_departure_possible_[visit]; // we can assume that a single visit route is feasible, therefore there would be enough time to get back at the depot
        latest_departures_[tour_[start_positions_.back()]] = latest_departures_[visit] - data_.true_distances_[data_.depot_][visit] - data_.service_time_[visit];
        tour_.push_back(data_.n_requests_+ (++n_routes_));
        start_positions_.push_back(tour_.size() - 1);
        arrivals_.push_back(departures_[visit] + data_.true_distances_[visit][tour_[0]]);
        departures_.push_back(0);
        latest_departures_.push_back(0);
    }



}


/** ------------------------------------------------------------------------------------------------ */


void Solution::removeCustomer(int route, int position) {
    // set the arrival time at 0 (I'm not sure this make sense right now)
    arrivals_[tour_[position]] = 0;
    departures_[tour_[position]] = 0;
    latest_departures_[tour_[position]] = 0;

    // remove customer
    tour_.erase(tour_.begin() + position);

    // update start positions
    for (int j = route + 1; j <= n_routes_; ++j)
        --start_positions_[j];

    // if we are deleting only customer of a route, delete route and update
    if ((tour_[position] >= data_.n_requests_) && (tour_[position - 1] >= data_.n_requests_)) {
        tour_.erase(tour_.begin() + position);
        start_positions_.erase(start_positions_.begin() + route + 1);
        --n_routes_;
        for (int j = route + 1; j <= n_routes_; ++j) {
            --start_positions_[j];
            --tour_[start_positions_[j]];
        }
        arrivals_.erase(arrivals_.begin() + data_.n_requests_ + route + 1);
        departures_.erase(departures_.begin() + data_.n_requests_ + route + 1);
        latest_departures_[data_.n_requests_ + route] =  latest_departures_[data_.n_requests_ + route + 1];
        latest_departures_.erase(latest_departures_.begin() + data_.n_requests_ + route + 1);
    }
    else {
        // update arrival time of successors (including ending depot)
        for (int j = position; j <= start_positions_[route+1]; ++j) {
            arrivals_[tour_[j]] = departures_[tour_[j - 1]] + data_.true_distances_[tour_[j - 1]][tour_[j]];
            departures_[tour_[j]] = departureTime(tour_[j]);
        }
        // update latest departure time of predecessors (including starting depot)
        latest_departures_[tour_[position - 1]] = (position == start_positions_[route+1]) ? data_.latest_departure_possible_[tour_[position - 1]] : min(latest_departures_[tour_[position]] - data_.true_distances_[tour_[position - 1]][tour_[position]] - data_.service_time_[tour_[position]], data_.latest_departure_possible_[tour_[position-1]]);
        for (int j = position - 2; j >= start_positions_[route]; --j)
            latest_departures_[tour_[j]] = min(latest_departures_[tour_[j+1]] - data_.true_distances_[tour_[j]][tour_[j+1]] - data_.service_time_[tour_[j+1]], data_.latest_departure_possible_[tour_[j]]);
    }
}

/** ------------------------------------------------------------------------------------------------ */


void Solution::swapReversePaths(int route_1, int route_2, int start_path_1, int end_path_1, int start_path_2, int end_path_2, bool reverse_path_1, bool reverse_path_2) {
    
    // copy paths to swap, reverse them if needed and delete them
    vector<int> path_2(tour_.cbegin() + start_path_2, tour_.cbegin() + end_path_2+ 1);
    if (reverse_path_2)
        reverse(path_2.begin(), path_2.end());
    tour_.erase(tour_.cbegin() + start_path_2, tour_.cbegin() + end_path_2 + 1);

    vector<int> path_1(tour_.cbegin() + start_path_1, tour_.cbegin() + end_path_1+ 1);
    if (reverse_path_1)
        reverse(path_1.begin(), path_1.end());
    tour_.erase(tour_.cbegin() + start_path_1, tour_.cbegin() + end_path_1 + 1);



    int delta = end_path_2 - start_path_2 - (end_path_1 - start_path_1);

    // re - insert them
//    if (start_path_2 >= end_path_2)
    tour_.insert(tour_.cbegin() + start_path_1 + (end_path_1 < start_path_1), path_2.cbegin(), path_2.cend());
//    if (start_path_1 >= end_path_1)
    tour_.insert(tour_.cbegin() + start_path_2 + delta + (end_path_2 < start_path_2), path_1.cbegin(), path_1.cend());

    // save new indexes
    int new_start_path_1 = start_path_1 + (end_path_1 < start_path_1);
    int new_end_path_1 = new_start_path_1 + end_path_2 - start_path_2;
    int new_start_path_2 =  start_path_2 + delta + (end_path_2 < start_path_2);
    int new_end_path_2 =  new_start_path_2 + ((end_path_1 >= start_path_1) ? (end_path_1 - start_path_1) : -1);

    // MODIFICATIONS FOR ROUTE 2
    // if route 2 is not deleted
    if (tour_[new_start_path_2] < data_.n_requests_ or tour_[new_end_path_2] < data_.n_requests_ ) {

        // re set arrival/departure of route 2
        for (int i = new_start_path_2; i < start_positions_[route_2 + 1]; ++i) {
            arrivals_[tour_[i]] = departures_[tour_[i - 1]] + data_.true_distances_[tour_[i - 1]][tour_[i]];
            departures_[tour_[i]] = departureTime(tour_[i]);
        }
        arrivals_[tour_[start_positions_[route_2 + 1]]] = departures_[tour_[start_positions_[route_2 + 1]-1]] + data_.true_distances_[tour_[start_positions_[route_2 + 1] -1 ]][tour_[start_positions_[route_2 + 1]]];

        // re-set start_position of route_2
        start_positions_[route_2] += delta;

        // re set latest departure time
        for (int i = new_end_path_2; i >= start_positions_[route_2]; --i)
            latest_departures_[tour_[i]] = min(data_.latest_departure_possible_[tour_[i]], (tour_[i+1] < data_.depot_ ? latest_departures_[tour_[i + 1]] - data_.service_time_[tour_[i + 1]]: data_.end_TW_.back()) -
                                                                        data_.true_distances_[tour_[i]][tour_[i + 1]]);
    }
    else {
        // re-set start_position of route_2
        start_positions_[route_2] += delta;

        // erase one starting position and adjust the others
        tour_.erase(tour_.cbegin() + new_start_path_2);
        start_positions_.erase(start_positions_.begin() + route_2 + 1);
        --n_routes_;
        for (int j = route_2 + 1; j <= n_routes_; ++j) {
            --start_positions_[j];
            --tour_[start_positions_[j]];
        }

        // adjust arrivals etc of depots
        arrivals_.erase(arrivals_.begin() + data_.n_requests_ + route_2 + 1);
        departures_.erase(departures_.begin() + data_.n_requests_ + route_2 + 1);
        latest_departures_[data_.n_requests_ + route_2] =  latest_departures_[data_.n_requests_ + route_2 + 1];
        latest_departures_.erase(latest_departures_.begin() + data_.n_requests_ + route_2 + 1);

        // adjust vectos for costs
        routes_costs_.erase(routes_costs_.begin() + route_2);
        routes_reduced_costs_.erase(routes_reduced_costs_.begin() + route_2);
    }


    // MODIFICATIONS FOR ROUTE 1
    // adjust the start position up to route_2
    for (int route = route_1 + 1; route < route_2; ++route)
        start_positions_[route] += delta;


    // if route 1 is not deleted
    if (tour_[new_start_path_1] < data_.n_requests_ or tour_[new_end_path_1] < data_.n_requests_ ) {

        // re set arrival/departure of route 1
        for (int i = new_start_path_1; i < start_positions_[route_1 + 1] - 1; ++i) {
            arrivals_[tour_[i]] = departures_[tour_[i - 1]] + data_.true_distances_[tour_[i - 1]][tour_[i]];
            departures_[tour_[i]] = departureTime(tour_[i]);
        }
        arrivals_[tour_[start_positions_[route_1 + 1]]] = departures_[tour_[start_positions_[route_1 + 1]-1]] + data_.true_distances_[tour_[start_positions_[route_1 + 1] -1 ]][tour_[start_positions_[route_1 + 1]]];

        for (int i = new_end_path_1; i >= start_positions_[route_1]; --i)
            latest_departures_[tour_[i]] = min(data_.latest_departure_possible_[tour_[i]],  (tour_[i+1] < data_.n_requests_ ? latest_departures_[tour_[i + 1]] - data_.service_time_[tour_[i + 1]] : data_.end_TW_.back()) -
                                                                        data_.true_distances_[tour_[i]][tour_[i + 1]]);
    }
    else {
        tour_.erase(tour_.cbegin() + new_start_path_1);
        start_positions_.erase(start_positions_.begin() + route_1 + 1);
        --n_routes_;
        for (int j = route_1 + 1; j <= n_routes_; ++j) {
            --start_positions_[j];
            --tour_[start_positions_[j]];
        }
        arrivals_.erase(arrivals_.begin() + data_.n_requests_ + route_1 + 1);
        departures_.erase(departures_.begin() + data_.n_requests_ + route_1 + 1);
        latest_departures_[data_.n_requests_ + route_1] =  latest_departures_[data_.n_requests_ + route_1 + 1];
        latest_departures_.erase(latest_departures_.begin() + data_.n_requests_ + route_1 + 1);

        // adjust vectos for costs
        routes_costs_.erase(routes_costs_.begin() + route_1);
        routes_reduced_costs_.erase(routes_reduced_costs_.begin() + route_1);
    }
}


/** ------------------------------------------------------------------------------------------------ */


void Solution::write(string file_name, vector<Cost*> &costs) {
    ofstream out_file;
    out_file.open (file_name);
    int instance_version = 0; // if this is true we print customers as in the instances, otherwise as saved in the solutions
    if (!instance_version)
        out_file <<  "Note I am printin in debug version (to compare with customers in instance.txt add 1) " << endl;
    out_file << "Routes\n";
    for (int r = 0; r < n_routes_; ++r) {
        out_file << "route " << r << endl;
        out_file << "dep -> ";
        for (int i = start_positions_[r] + 1; i < start_positions_[r+1]; ++i)
            out_file << (tour_[i] + instance_version) << "  ->  ";
        out_file << "dep " << endl;
        for (int i = start_positions_[r] + 1; i < start_positions_[r+1]; ++i)
            out_file << (tour_[i] + instance_version) << " (a: " << arrivals_[tour_[i]] << " - d: " << departures_[tour_[i]] << ")   ";
        out_file << "dep " << endl;
        for (int i = start_positions_[r] + 1; i < start_positions_[r+1]; ++i)
            out_file << (tour_[i] + instance_version) << " [" << data_.start_TW_[tour_[i]] << ", " << data_.end_TW_[tour_[i]] << "]   ";
        out_file << "dep " << endl << endl;
    }
    out_file << endl << endl;

    for (auto &cost : costs)
        out_file << cost -> getName() << " = " << cost-> computeCost(*this) << endl;

    out_file << "tour_\n";
    for (auto const & el : tour_)
        out_file << el << " ";
    out_file << endl << endl;

//    out_file << "arrivals_\n";
//    for (int i = 0; i < tour_.size(); ++i)
//        out_file << "arrivals_[ " << i << " ] = " << arrivals_[i] << endl;
//    out_file << endl << endl;
//
//    out_file << "departures_\n";
//    for (int i = 0; i < tour_.size(); ++i)
//        out_file << "departures_[ " << i << " ] = " << departures_[i] << endl;
//    out_file << endl << endl;
}



/** ------------------------------------------------------------------------------------------------ */

void Solution::computeCost(vector<Cost *> &cost_components_solution)
{
    // (re-) initialize cost
    cost_ = 0;
    for (auto & cost_source : cost_components_solution)
        cost_  += cost_source -> computeCost(*this);
}

/** ------------------------------------------------------------------------------------------------ */


void Solution::computeRoutesCost(vector<Cost *> &cost_components_route, vector<int> routes_indexes)
{
    if (routes_indexes.size()) {
        for (auto &index : routes_indexes)
            routes_costs_[index] = 0;
    }
    else {
        // (re-) initialize cost of routes (!! maybe this could be improved)
        routes_costs_.clear();
        routes_costs_.resize(n_routes_, 0);
    }
    for (auto &cost_source : cost_components_route)
        cost_source->addRoutesCost(*this, routes_costs_, routes_indexes);
}


/** ------------------------------------------------------------------------------------------------ */



bool Solution::checkFeasibilitySolution()
{
    for (int route = 0; route< n_routes_; ++route)
    {
        double route_load = 0;
        for (int i = start_positions_[route]+1; i < start_positions_[route + 1]; ++i)
        {
            route_load += data_.demands_[tour_[i]];
            assert((arrivals_[tour_[i]] <= data_.end_TW_[tour_[i]]) && ("arrival time after end of time window. position " + to_string(i) + " req " + to_string(tour_[i])).c_str());
        }
        assert(route_load <= data_.capacity_*capacity_coefficient_ && ("route load exceed capacity" + to_string(route)).c_str());
    }
    return true;
}



/** ------------------------------------------------------------------------------------------------ */

//
//void Solution::refineRoutesTSP() {
//    for (int r = 0; r <n_routes_; ++r)
//        TSProute(r);
//}
//
//
//
//void Solution::TSProute(int route) {
//
//    // number of customer does not include depot
//    int points(start_positions_[route+1] - start_positions_[route]);
//    int start_customers = start_positions_[route] + 1;
//
//    GRBVar **flow_vars = new GRBVar*[points];
//    GRBVar *departure_vars = new GRBVar[points];
//
//    int i;
//    for (i = 0; i < points; ++i)
//        flow_vars[i] = new GRBVar[points];
//
//    try {
//
//        GRBEnv* env = new GRBEnv();
//        env->set(GRB_DoubleParam_TimeLimit, tsp_time_limit_);
//        env->set(GRB_IntParam_OutputFlag, 0);
//        GRBModel model = GRBModel(*env);
//
//        // Create binary decision variables
//        int j;
//        for (i = 0; i < points; ++i) {
//            departure_vars[i] = model.addVar(data_.start_TW_[tour_[start_customers + i]] + data_.service_time_[tour_[start_customers + i]],
//                                             data_.latest_departure_possible_[tour_[start_customers + i]], 0, GRB_CONTINUOUS,
//                                             "departure-" + to_string(i));
//            for (j = 0; j < points; ++j)
//                if (i != j && data_.possible_arcs_[tour_[start_customers + i]][tour_[start_customers + j]])
//                    flow_vars[i][j] = model.addVar(0.0, 1.0, data_.true_distances_[tour_[start_customers + i]][tour_[start_customers + j]], GRB_BINARY,
//                                                   "x-" + to_string(i) + "-" + to_string(j));
//
//
//
//        }
//
//        // Update Model
//        model.update();
//
//
//        // In constraints
//        for (i = 0; i < points; ++i) {
//            GRBLinExpr out_constraints_lhs(0);
//            GRBLinExpr in_constraints_lhs(0);
//            for (j = 0; j < points; ++j) {
//                if (data_.possible_arcs_[tour_[start_customers + i]][tour_[start_customers + j]])
//                    out_constraints_lhs += flow_vars[i][j];
//                if (data_.possible_arcs_[tour_[start_customers + j]][tour_[start_customers + i]])
//                    in_constraints_lhs += flow_vars[j][i];
//            }
//            model.addConstr(out_constraints_lhs, GRB_GREATER_EQUAL,  1, "out-"+to_string(i));
//            model.addConstr(in_constraints_lhs, GRB_GREATER_EQUAL, 1, "in-"+to_string(i));
//        }
//
//        // Time Windows constraints
//        model.addConstr(departure_vars[points - 1], GRB_EQUAL,  0, "departure-depot");
//        double big_M = 2*data_.end_TW_.back();
//        for (i = 0; i < points-1; ++i) {
//            // Add time constraints
//            GRBLinExpr lhs = departure_vars[i];
//            for (j = 0; j < points-1; ++j) {
//                if (data_.possible_arcs_[tour_[start_customers + i]][tour_[start_customers + j]]) {
//                    lhs += big_M*flow_vars[i][j];
//                    model.addConstr(lhs, GRB_LESS_EQUAL, data_.end_TW_[tour_[start_customers + j]] + big_M - data_.true_distances_[tour_[start_customers + i]][tour_[start_customers + j]], "times-" + to_string(i) + "-" + to_string(j));
//                }
//            }
//            // Add time constraints to depot
//            model.addConstr(lhs, GRB_LESS_EQUAL, data_.end_TW_.back() + big_M - data_.true_distances_[tour_[start_customers + i]][data_.depot_], "times-" + to_string(i) + "-0");
//        }
//
//
//        // Optimize model
//        model.optimize();
//
//
//        // Try to reorganize solution
//        try {
//            double **solution = new double*[points];
////            vector<vector<double>> solution(points, vector<double>(points));
//            for (i = 0; i < points; ++i) {
//                solution[i] = new double[points];
//                for (j = 0; j < points; ++j) {
//                    if (data_.possible_arcs_[tour_[start_customers + i]][tour_[start_customers + j]])
//                        solution[i][j] = flow_vars[i][j].get(GRB_DoubleAttr_X);
//                    else
//                        solution[i][j] = 0;
//                }
//            }
//
//
//            // copy old route
//            int tsp_index = points - 1;
//            vector<int> copy_route(tour_.cbegin() + start_positions_[route] + 1,
//                                   tour_.cbegin() + start_positions_[route + 1]);
//
//            // copy new route and adjust arrivals_/departures_
//            for (j = 0; j < points - 1; ++j) {
//                for (i = 0; i < points - 1; ++i) {
//                    if (solution[tsp_index][i] > 0.5) {
//                        tour_[start_customers + j] = copy_route[i];
//                        arrivals_[copy_route[i]] = departures_[tour_[j + start_positions_[route]]] +
//                                                   data_.true_distances_[tour_[j + start_positions_[route]]][copy_route[i]];
//                        departures_[copy_route[i]] = departureTime(copy_route[i]);
//                        tsp_index = i;
//                        break;
//                    }
//                }
//            }
//
//            // adjust latest departures
//            for (i = start_positions_[route+1]-1; i >= start_positions_[route]; --i)
//                latest_departures_[tour_[i]] = min(data_.latest_departure_possible_[tour_[i]], (indexIsDepot(i+1) ? data_.end_TW_.back() : (latest_departures_[tour_[i+1]] - data_.service_time_[tour_[i+1]])) - data_.true_distances_[tour_[i]][tour_[i+1]]);
//
//
//            // delete solution
//            for (i = 0; i < points; i++)
//                delete[] solution[i];
//            delete[] solution;
//
//        } catch (GRBException e) {
//            cout << "Error number: " << e.getErrorCode() << " in rewriting route" <<endl;
//            cout << e.getMessage() << endl;
//        } catch (...) {
//            cout << "Error during optimization" << endl;
//        }
//
//        delete env;
//    }
//    catch (GRBException e) {
//        cout << "Error number: " << e.getErrorCode() << " in optimizing tsp" << endl;
//        cout << e.getMessage() << endl;
//    } catch (...) {
//        cout << "Error during optimization" << endl;
//    }
//
//    for (i = 0; i < points; i++)
//        delete[] flow_vars[i];
//    delete[] flow_vars;
//    delete[] departure_vars;
//
//}