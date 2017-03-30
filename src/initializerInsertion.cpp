//
// Created by Bertoli, Francesco (Data61, Canberra City) on 24/02/17.
//

#include "initializerCosts.h"
#include "initializerInsertion.h"
#include <limits>
#include <numeric>
#include <cassert>

Option<int> InitializerInsertion::seed_selection_("seed", "controls how the seed customer is selected",0);

/** CONSTRUCTORS */
InitializerInsertion::InitializerInsertion(Data &data) :
        Initializer(data),
        unrouted_req_(data.n_requests_),
        switcher_(0)
{
    unrouted_.resize(data_.n_requests_,0);
    rank_distances_.resize(data_.n_requests_);
    rank_TW_.resize(data_.n_requests_);
    rank_profits_.resize(data_.n_requests_);
    comparisonProfitStruct dual_comparison_{profits_};
    preProcessing();
}


/** ------------------------------------------------------------------------------------------------ */
/** METHODS */
/** ------------------------------------------------------------------------------------------------ */

/** build a solution  */
Solution InitializerInsertion::initializeSolution()
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
    vector<double> position_cost(data_.n_requests_, inf);       // keep track of the best position cost found for each customer
    vector<int> predecessor_index(data_.n_requests_, -1);       // keep track of the best predecessor found for each customer
    vector<bool> feasible(data_.n_requests_, false);            // keep track if a position was found

    // SELECT FIRST SEED
    int seed_customer(selectSeedCustomer());
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

    while (unrouted_req_)
    {
        // for each customer find the best insertion cost and position
        for (int customer = 0; customer < data_.n_requests_; ++customer) {
            feasible[customer] = false;

            // if customer is unrouted and in the same cluster of seed customer and capacity checks
            if (clusterisation_->at(customer) == clusterisation_->at(seed_customer) && unrouted_[customer] && route_load + data_.demands_[customer] < capacity_) {
                // re-initialize variables
                position_cost[customer] = inf;

                // look for best insertion position
                for (int position = solution.tour_.size() - 2; position >= solution.start_positions_[solution.n_routes_ -1]; --position ) {
                    predecessor = solution.tour_[position];
                    arrival_time_visit = solution.departures_[predecessor] + data_.distances_[predecessor][customer];

                    // check arrival time at cutomer
                    if (arrival_time_visit > data_.end_TW_[customer])
                        continue;

                    // compute new departure time
                    successor = solution.tour_[position + 1];
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
        else
        {
            seed_customer = selectSeedCustomer();
            solution.insertCustomer(solution.n_routes_, seed_customer, solution.tour_.size() - 1);
            unrouted_[seed_customer] = false;
            --unrouted_req_;
            route_load = data_.demands_[seed_customer];
        }
    }

    // Re-SET POINTER TO NULL IF IT WAS NULL
    if (default_clusterisation.size())
        clusterisation_= nullptr;

    // UPDATE ROUTE CONTAINERS
    solution.computeRoutesCost(*cost_components_route_);
    route_container_->extractRoutes(solution, vector<int>());
//    route_container_->createIntegerModel();
//    route_container_->createLinearModel();

    return solution;
}


/** ------------------------------------------------------------------------------------------------ */


int InitializerInsertion::selectSeedCustomer()
{

    if (unrouted_req_ == 1)
        for (int j = 0; j < data_.n_requests_; ++j)
            if (unrouted_[j])
                return j;

    if (switcher_ == 0) {
        // SOLOMON OPTION ----------------------------------------------------------------------
        int furthest(-1), earliest(-1), second_earliest(-1);
        bool found_furthest(false), found_earliest(false), found_second_earliest(false);

        for (int i = data_.n_requests_ - 1; i >= 0; --i) {
            if (not found_furthest && unrouted_[rank_distances_[i]]) {
                furthest = rank_distances_[i];
                found_furthest = true;
            }
            if (unrouted_[rank_TW_[i]]) {
                if (not found_earliest) {
                    earliest = rank_TW_[i];
                    found_earliest = true;
                } else if (not found_second_earliest) {
                    second_earliest = rank_TW_[i];
                    found_second_earliest = true;
                }
            }
            if (found_furthest && found_earliest && found_second_earliest)
                break;
        }

        if (seed_selection_ == 0) {
            // If the custopmer with the earliest start does not have to be the first take the fursthest
            if (max(data_.distances_[data_.depot_][second_earliest], data_.start_TW_[second_earliest]) +
                data_.service_time_[second_earliest] + data_.distances_[second_earliest][earliest] <
                data_.end_TW_[earliest])
                return furthest;
            else
                return earliest;
        } else if (seed_selection_ == 1)
            return furthest;
        else
            return earliest;
    }
    else {
        // PROFIT OPTION ----------------------------------------------------------------------
        for (int i = data_.n_requests_ - 1; i >= 0; --i) {
            if (unrouted_[rank_profits_[i]])
                return rank_profits_[i];
        }
    }
}


/** ------------------------------------------------------------------------------------------------ */



void InitializerInsertion::preProcessing() {

    // CREATE RANK TO CHOOSE SEED CUSTOMERS
    iota(rank_distances_.begin(), rank_distances_.end(), 0);
    rank_TW_ = rank_distances_;
    rank_profits_ = rank_distances_;

    // order them
    comparisonDistancesStruct dist_comparison_{data_};
    comparisonTWStruct tw_comparison_{data_};
    sort(rank_distances_.begin(), rank_distances_.end(), dist_comparison_);
    sort(rank_TW_.begin(), rank_TW_.end(), tw_comparison_);

    // if there is pointer to a profit vector sort them also for profits
    if (profits_)
        sort(rank_profits_.begin(), rank_profits_.end(), profit_comparison_);
}


/** ------------------------------------------------------------------------------------------------ */


void InitializerInsertion::sortOnProfits() {
    if (profits_) {
        profit_comparison_.profits_ = profits_;
        sort(rank_profits_.begin(), rank_profits_.end(), profit_comparison_);
    }
}




/** ------------------------------------------------------------------------------------------------ */


void InitializerInsertion::switcher(string mode) {
    if (mode == "solomon")
        switcher_ = 0;
    else if (mode == "profits")
        switcher_ = 1;
}



/** ------------------------------------------------------------------------------------------------ */


void InitializerInsertion::setProfitPtr(vector<double> *profits) {
    profits_ = profits;
    sortOnProfits();
};

