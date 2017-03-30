//
// Created by Bertoli, Francesco (Data61, Canberra City) on 24/02/17.
//

#include "initializerMultipleRoutes.h"
#include "initializerCosts.h"
#include <limits>
#include <numeric>
#include <cassert>



/** ------------------------------------------------------------------------------------------------ */



/** ------------------------------------------------------------------------------------------------ */



Solution InitializerMultipleRoutes::initializeSolution()
{
    // PREPARATION (IF NEEDED)
    preProcessing();

    // Re-INITIALIZE UNROUTED
    fill(unrouted_.begin(), unrouted_.end(), true);
    unrouted_req_ = data_.n_requests_;

    // CHECK CLUSTERISATION
    vector<int> default_clusterisation;
    if (!clusterisation_) {
        clusterisation_ = &default_clusterisation;
        default_clusterisation.resize(data_.n_requests_,0);
    }
    else
        default_clusterisation.clear();


    // INFINITY
    double inf = std::numeric_limits<double>::max();

    // CREATE DATA STRUCTURES
    vector<double> position_cost(data_.n_requests_, inf);       // keep track of the best position cost found for each custoemr
    vector<int> predecessor_index(data_.n_requests_, -1);       // keep track of the best predecessor found for each customer
    vector<bool> feasible(data_.n_requests_, false);            // keep track if a position was found

    // SELECT FIRST SEED
    int seed_customer(0);
    --unrouted_req_;
    unrouted_[seed_customer] = false;


    // CREATE STARTING SOLUTION
    vector<int> tour{data_.n_requests_, seed_customer, data_.n_requests_ + 1};
    vector<double> arrivals(data_.n_requests_ + 1, 0);
    arrivals[seed_customer] = data_.distances_[data_.n_requests_][seed_customer];
    arrivals.push_back(max(arrivals[seed_customer], data_.start_TW_[seed_customer]) + data_.service_time_[seed_customer] + data_.distances_[seed_customer][data_.depot_]);
    Solution solution{data_, tour, arrivals};


    // DECLARE FIELD
    int predecessor;
    int successor;
    int to_insert;
    double arrival_time_visit;
    double new_arrival_successor;
    double new_departure_successor;
    double lost_time;
    int insertion_cost = 0;
    int route_load = data_.demands_[seed_customer];

    while (true)
    {
        // for each customer find the best insertion cost and position
        for (int customer = 0; customer < data_.n_requests_; ++customer) {
            feasible[customer] = false;

            // if customer is unrouted and in the same cluster of seed customer and capacity checks
            if (clusterisation_->at(customer) == clusterisation_->at(seed_customer) && unrouted_[customer] && route_load + data_.demands_[customer] < capacity_) {
                // re-initialize varaibles
                position_cost[customer] = inf;

                // look for best insertion position
                for (int position = solution.tour_.size() - 2; position >= solution.start_positions_[solution.n_routes_ -1]; --position ) {
                    predecessor = solution.tour_[position];
                    if (predecessor >= data_.depot_)
                        predecessor = data_.depot_;

                    arrival_time_visit = solution.departures_[predecessor] + data_.distances_[predecessor][customer];

                    // chekc arrival time at cutomer
                    if (arrival_time_visit > data_.end_TW_[customer])
                        continue;

                    // compute new departure time
                    successor = solution.tour_[position + 1];
                    if (successor >= data_.depot_)
                        successor = data_.depot_;

                    new_arrival_successor = max(arrival_time_visit, data_.start_TW_[customer]) + data_.service_time_[customer] + data_.distances_[customer][successor];
                    new_departure_successor = max(new_arrival_successor, data_.start_TW_[successor])+ data_.service_time_[successor];

                    if (((successor >= data_.depot_) && (new_arrival_successor < data_.end_TW_.back())) || (new_departure_successor < solution.latest_departures_[successor])) {
                        feasible[customer] = true;
                        // check if it's the best inertion found so far for customer
                        lost_time = new_departure_successor -  - solution.departures_[successor];
                        if (positionCost(predecessor, customer, successor, lost_time, data_) < position_cost[customer]) {
                            position_cost[customer] = positionCost(predecessor, customer, successor, lost_time, data_);
                            predecessor_index[customer] = position;
                        }
                    }
                }
            }
        }

        //find the customer to be inserted
        insertion_cost = -1000000;
        to_insert = -1;
        for (int customer = 0; customer < data_.depot_; ++customer) {
            if (feasible[customer] &&  (insertionCost(customer, position_cost[customer], switcher_, profits_, data_) > insertion_cost))
            {
                to_insert = customer;
                insertion_cost = insertionCost(customer, position_cost[customer], switcher_, profits_, data_);
            }
        }



        // insert customer or create new route
        if (to_insert > -1) {
            solution.insertCustomer(solution.n_routes_ - 1, to_insert, predecessor_index[to_insert]);
            assert(solution.arrivals_[to_insert] <= data_.end_TW_[to_insert] && "insertion not feasible in InsertionI1");
            unrouted_[to_insert] = false;
            --unrouted_req_;
            route_load += data_.demands_[to_insert];
        }
        else if (seed_customer < data_.n_requests_ - 1)
        {
            solution.tour_.back() = data_.depot_;
            ++seed_customer;
            solution.insertCustomer(solution.n_routes_, seed_customer, solution.tour_.size() - 1);
            fill(unrouted_.begin(), unrouted_.end(), true);
            unrouted_req_ = data_.n_requests_ - 1;
            unrouted_[seed_customer] = false;
            route_load = data_.demands_[seed_customer];
        }
        else
            break;
    }

    // Re-SET POINTER TO NULL IF IT WAS NULL
    if (default_clusterisation.size())
        clusterisation_= nullptr;

    // MODIFY DEPOTS IDs (so that we can compute distances)
    for (auto & el : solution.tour_)
        if (el >= data_.n_requests_)
            el = data_.depot_;

    // CREATE MODELS
    solution.computeRoutesCost(* (cost_components_route_));
    route_container_->extractRoutes(solution);
    // we should check for doubles
    route_container_->createIntegerModel();
//    route_container_->createLinearModel();



    // OPTIMIZE INTEGER MODEL AND GET CURRENT SOLUTION
    route_container_->optimizeIntegerModel();
    route_container_->updateIntegerSolution();
//    route_container_->integer_model_.write("initial-model.lp");
//    route_container_->integer_model_.write("initial-model.sol");

    return route_container_->solution_;
}



/** ------------------------------------------------------------------------------------------------ */

