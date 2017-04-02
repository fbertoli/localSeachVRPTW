//
// Created by Bertoli, Francesco (Data61, Canberra City) on 09/02/17.
//



#include "crossReverseGenerator.h"
#include <limits>



/** CONSTRUCTORS */

CrossReverseGenerator::CrossReverseGenerator(const Data &data, Solution* sol, double capacity_coefficient, CrossReverseMove *best_move, CrossReverseMove *move) :
        MoveGenerator::MoveGenerator(data, sol), modified_capacity_(data_.capacity_*capacity_coefficient),
        max_length_path_("max-path", "maximum path length in crossReverse", 4),
        best_move_(best_move),
        move_(move)
{
    int big_size(data_.n_requests_ + data_.max_vehicles_);
    routes_load_.resize(data_.max_vehicles_);
    paths_load_.resize(big_size);
    path_latest_departure_standard_.resize(big_size);
    path_latest_departure_reversed_.resize(big_size);
    for( int i = 0; i < big_size; ++ i ) {
        paths_load_[i].resize(max_length_path_ + 1, 0);
        path_latest_departure_standard_[i].resize(max_length_path_ + 1, 0);
        path_latest_departure_reversed_[i].resize(max_length_path_ + 1, 0);
//        path_cumulative_cost_standard_[i].resize(max_length_path_, 0);
//        path_cumulative_cost_reversed_[i].resize(max_length_path_, 0);

    }

    int small_size{data_.max_routes_stops_ + 1};
    departures_path_1_standard_.resize(max_length_path_ + 1);
    departures_path_1_reversed_.resize(max_length_path_ + 1);
    departures_path_2_standard_.resize(max_length_path_ + 1);
    departures_path_2_reversed_.resize(max_length_path_ + 1);
//    path_cumulative_cost_standard_.resize(data_.n_requests_ + data_.max_vehicles_);
//    path_cumulative_cost_reversed_.resize(data_.n_requests_ + data_.max_vehicles_);
//    start_l_index_.resize(size);

    for( int i = 0; i < max_length_path_ + 1; ++ i ) {
        departures_path_1_standard_[i].resize(small_size);
        departures_path_1_reversed_[i].resize(small_size);
        departures_path_2_standard_[i].resize(small_size);
        departures_path_2_reversed_[i].resize(small_size);
//        start_l_index_[i].resize(small_size);
        for( int j = 0; j < small_size; ++ j ) {
//            start_l_index_[i][j].resize(size, 0);
            departures_path_1_standard_[i][j].resize(small_size);
            departures_path_1_reversed_[i][j].resize(small_size);
            departures_path_2_standard_[i][j].resize(small_size);
            departures_path_2_reversed_[i][j].resize(small_size);
        }
    }


}

/** --------------------------------------------------------------------------------
 *  -------------------------------------------------------------------------------- */


/** METHODS */



bool CrossReverseGenerator::random() {return false;}

/** -------------------------------------------------------------------------------- */

bool CrossReverseGenerator::first() {

    route_1_ = 0;
    route_2_ = 1;
    length_route_1_ = current_solution_->start_positions_[route_1_+1] -  current_solution_->start_positions_[route_1_];
    length_route_2_ = current_solution_->start_positions_[route_2_+1] -  current_solution_->start_positions_[route_2_];
    i_ = length_route_1_ - 1;
    j_ = 0;
    k_ = length_route_2_ - 1;
    l_ = 0;
    has_next_ = true;
    computeRoutesLoad(); // in this computation I could include the path_load initialization in linear time

    // find the first move
    next();

    // update the best move
    updateBestMove();

}

/** -------------------------------------------------------------------------------- */


bool CrossReverseGenerator::next() {

    // PHASE 1 - update indexed and data structures
    do {

        // if I can move l_ forward
        if (l_ < max_length_path_ && l_ < length_route_2_ - 1 - k_) {
            ++l_;
            setVisits();
//            cout << " + l_ -->  ";
//            cout << " i_ = " << i_ << "; j_ = " << j_ << "; k_ = " << k_ << "; l_ = " << l_ << "; route_1_ = " << route_1_ << "; route_2_ = " << route_2_ <<endl;
//            if (j_ == i_ )
//                cout << "propagate departures_path_2_[ " <<  l_ << " ][ " << k_ + 1 << " ][ " << i_ << " ]" << " from departures_path_2_[ " <<  l_ - 1 << " ][ " << k_ + 1 << " ][ " << i_ << " ]" <<  endl;
//                cout << "propagate departures_path_2_[ " <<  k_ + 1  << " ][ " << l_ << " ][ " << i_ << " ]" << " from departures_path_2_[ " <<  k_ + 2 << " ][ " << l_  << " ][ " << i_ << " ]" <<  endl;
//            if (route_1_ == 0 && i_ == current_solution_->start_positions_[route_1_+1] - 1)
//                cout << "propagate path_latest_departure_[ " <<  k_ + 1 << " ][ " << l_ << " ]" << " from [ " <<  k_ + 2 << " ][ " << l_ << " ] " << endl;
//                cout << "propagate path_latest_departure_[ " <<  l_ << " ][ " << k_ + 1 << " ]" << " from [ " <<  l_ - 1 << " ][ " << k_ + 1 << " ] " << endl;


            // PROPAGATION STRUCTURES
            if (j_ == 0 ) { // need to do these computations only for one pass of j
                // forward propagation
                departures_path_2_standard_[l_][k_+1][i_] = max(departures_path_2_standard_[l_ - 1][k_ + 1][i_] + data_.distances_[getVisit(tour_index_l_-1)][visit_l_],
                                                       data_.start_TW_[visit_l_]) + data_.service_time_[visit_l_];
                // backward propagation
                departures_path_2_reversed_[l_][k_+1][i_] = max(departures_path_2_reversed_[l_ - 1][k_ + 2][i_] + data_.distances_[getVisit(tour_index_k_+2)][visit_succ_k_],
                                                       data_.start_TW_[visit_succ_k_]) + data_.service_time_[visit_succ_k_];
            }

            // need to do it only for one pass of i_, j_ and route_1_/2_
            if (route_1_ == 0 && i_ == length_route_1_ - 1) {
                // forward propagation
                paths_load_[tour_index_k_+ 1][l_] = paths_load_[tour_index_k_ + 1][l_ - 1] + data_.demands_[visit_l_];
                // (standard orientation) backward propagation
                path_latest_departure_standard_[tour_index_k_+ 1][l_] = min(path_latest_departure_standard_[tour_index_k_ + 2][l_ - 1] - data_.distances_[visit_succ_k_][getVisit(tour_index_k_ + 2)] - data_.service_time_[getVisit(tour_index_k_ + 2)], data_.latest_departure_possible_[visit_succ_k_]);
                // (reverse orientation) forward propagation
                path_latest_departure_reversed_[tour_index_k_+ 1][l_] = min(path_latest_departure_reversed_[tour_index_k_ + 1][l_ - 1] - data_.distances_[visit_l_][getVisit(tour_index_l_ - 1)] - data_.service_time_[getVisit(tour_index_l_ - 1)], data_.latest_departure_possible_[visit_l_]);
                // propagation
//                path_cumulative_cost_standard_[tour_index_k_+ 1][l_] = extendStandardPathCost(tour_index_k_+ 1, l_ - 1);
//                path_cumulative_cost_reversed_[tour_index_k_+ 1][l_] = extendReversedPathCost(tour_index_k_+ 1, l_ - 1);
            }


            // FEASIBILITY VARIABLES
            feasibility_route_1_standard_ = false;
            feasibility_route_1_reversed_ = false;

            // feasibility of arriving at path 1 does not change when l_ increases (feasibility from path 1 is checked later)
            // standard
            if (feasibility_to_path_2_standard_ and max(current_solution_-> departures_[visit_i_] + data_.distances_[visit_i_][visit_succ_k_], data_.start_TW_[visit_succ_k_]) + data_.service_time_[visit_succ_k_] > path_latest_departure_standard_[tour_index_k_ + 1][l_] )
                feasibility_to_path_2_standard_ = false;
            // reversed
            if (feasibility_to_path_2_reversed_ and max(current_solution_-> departures_[visit_i_] + data_.distances_[visit_i_][visit_l_], data_.start_TW_[visit_l_]) + data_.service_time_[visit_l_] > path_latest_departure_reversed_[tour_index_k_ + 1][l_] )
                feasibility_to_path_2_reversed_ = false;


            // CHECKS
            if (not feasibility_to_path_2_standard_ and not feasibility_to_path_2_reversed_)
                continue;

        }


            // if I can move j_ forward
        else if (j_ < max_length_path_ && j_ < length_route_1_ - 1 - i_) {
            ++j_;
            l_ = 0;
            setVisits();
//            cout << " + j_ -->  ";
//            cout << " i_ = " << i_ << "; j_ = " << j_ << "; k_ = " << k_ << "; l_ = " << l_ << "; route_1_ = " << route_1_ << "; route_2_ = " << route_2_ <<endl;
//            if (j_ == i_ + 1)
//                cout << "initialize departures_path_1_[" <<  i_ + 1 << " ][ " << i_ + 1 << " ][ " << k_ << "]" << endl;
//                cout << "initialize start_l_index_[" <<  i_ + 1 << " ][ " << i_ + 1 << " ][ " << k_ << "]" << endl;
//            else   {
//                cout << "propagate departures_path_1_[ " <<  j_ << " ][ " << i_ + 1 << " ][ " << k_ << " ]" << " from departures_path_1_[ " <<  j_ - 1 << " ][ " << i_ + 1 << " ][ " << k_ << " ]" <<  endl;
//                cout << "propagate departures_path_1_[ " <<  i_ + 1  << " ][ " << j_ << " ][ " << k_ << " ]" << " from departures_path_1_[ " <<  i_ + 2 << " ][ " << j_  << " ][ " << k_ << " ]" <<  endl;
//                cout << "propagate start_l_index_[ " <<  j_ << " ][ " << i_ + 1 << " ][ " << k_ << " ]" << " from [ " <<  j_ << " ][ " << i_ + 2 << " ][ " << k_ << " ] and [ " << j_ - 1 << " ][ " << i_+1 << " ][ " << k_ << " ]"<< endl;
//                cout << "propagate start_l_index_[ " <<  i_ + 1 << " ][ " << j_ << " ][ " << k_ << " ]" << " from [ " <<  i_ + 2 << " ][ " << j_ << " ][ " << k_ << " ] and [ " << i_ + 1 << " ][ " << j_-1 << " ][ " << k_ << " ]"<< endl;
//                if (k_ == current_solution_-> start_positions_[2] - 1)
//                    cout << "propagate path_latest_departure_[ " <<  i_ + 1 << " ][ " << j_ << " ]" << " from [ " <<  i_ + 2 << " ][ " << j_  << " ] " << endl;
//                    cout << "propagate path_latest_departure_[ " <<  j_ << " ][ " << i_ + 1 << " ]" << " from [ " <<  j_ - 1 << " ][ " << i_ + 1 << " ] " << endl;
//            }

            // PROPAGATION - INITIALIZATION STRUCTURES
            if (j_ == 1) {
                // INITIALIZE
                departures_path_1_standard_[1][i_ + 1][k_] = max(current_solution_->departures_[visit_k_] + data_.distances_[visit_k_][visit_succ_i_], data_.start_TW_[visit_succ_i_]) + data_.service_time_[visit_succ_i_];
                departures_path_1_reversed_[1][i_ + 1][k_] = departures_path_1_standard_[1][i_ + 1][k_];
//                start_l_index_[i_ + 1][i_ + 1][k_] = 0;
            }
            else { // j_ is necessarily bigger than 1
                // PROPAGATE
                // (standard orientation) forward propagation
                departures_path_1_standard_[j_][i_ + 1][k_] = max(departures_path_1_standard_[j_ - 1][i_ + 1][k_] +
                                                         data_.distances_[getVisit(tour_index_j_ - 1)][visit_j_],
                                                         data_.start_TW_[visit_j_]) + data_.service_time_[visit_j_];
                // (reverse orientation) backward propagation
                departures_path_1_reversed_[j_][i_ + 1][k_] = max(departures_path_1_reversed_[j_ - 1][i_ + 2][k_] +
                                                         data_.distances_[getVisit(tour_index_i_ + 2)][visit_succ_i_],
                                                         data_.start_TW_[visit_succ_i_]) +
                                                         data_.service_time_[visit_succ_i_];

                //propagate bacward and foward for both directions and take the minimum
//                start_l_index_[j_][i_ + 1][k_] = max(start_l_index_[j_][i_ + 2][k_], start_l_index_[j_ - 1][i_ + 1][k_]);
//                start_l_index_[i_ + 1][j_][k_] = max(start_l_index_[i_ + 2][j_][k_], start_l_index_[i_ + 1][j_ - 1][k_]);

                // progate path_ information on the very first route)
                if (route_2_ == 1 && k_ == length_route_2_ - 1) {
                    // forward propagation
                    paths_load_[tour_index_i_ + 1][j_] = paths_load_[tour_index_i_ + 1][j_ - 1] + data_.demands_[visit_j_];
                    // (standard orientation) backward propagation
                    path_latest_departure_standard_[tour_index_i_ + 1][j_] = min(path_latest_departure_standard_[tour_index_i_ + 2][j_ - 1] -
                                                             data_.distances_[visit_succ_i_][getVisit(tour_index_i_ + 2)] -
                                                             data_.service_time_[getVisit(tour_index_i_ + 2)],
                                                             data_.latest_departure_possible_[visit_succ_i_]);
                    // (reverse orientation) forward propagation
                    path_latest_departure_reversed_[tour_index_i_ + 1][j_] = min(path_latest_departure_reversed_[tour_index_i_ + 1][j_ - 1] -
                                                             data_.distances_[visit_j_][getVisit(tour_index_j_ - 1)] -
                                                             data_.service_time_[getVisit(tour_index_j_ - 1)],
                                                             data_.latest_departure_possible_[visit_j_]);
                    // propagation
//                    path_cumulative_cost_standard_[tour_index_i_ + 1][j_] = extendStandardPathCost(tour_index_i_ + 1, j_ - 1);
//                    path_cumulative_cost_reversed_[tour_index_i_ + 1][j_] = extendReversedPathCost(tour_index_i_ + 1, j_ - 1);
                }
            }



            // FEASIBILITY VARIABLES
            feasibility_route_2_reversed_ = false;
            feasibility_route_2_standard_ = false;

            // feasibility of arriving at path 2 does not change when j_ increases (feasibility from path 2 is checked later)
            // if j_ == i_ + 1 feasibility_to_path_1_standar/reverse are true because a previous resetting of k set them true
            // standard
            if (feasibility_to_path_1_standard_ && max(current_solution_->departures_[visit_k_] + data_.distances_[visit_k_][visit_succ_i_], data_.start_TW_[visit_succ_i_]) + data_.service_time_[visit_succ_i_] > path_latest_departure_standard_[tour_index_i_ + 1][j_] )
                feasibility_to_path_1_standard_ = false;
            // reversed
            if (feasibility_to_path_1_reversed_ && max(current_solution_->departures_[visit_k_] + data_.distances_[visit_k_][visit_j_], data_.start_TW_[visit_j_]) + data_.service_time_[visit_j_] > path_latest_departure_reversed_[tour_index_i_ + 1][j_] )
                feasibility_to_path_1_reversed_ = false;

            // CHECKS
            if (not feasibility_to_path_1_standard_ && not feasibility_to_path_1_reversed_)
                continue;

        }


            // if I can move k_ backward
        else if (k_ > 0) {
            --k_;
            j_ = 0;
            l_ = 1;
            setVisits();
//            cout << " - k_ -->  ";
//            cout << " i_ = " << i_ << "; j_ = " << j_ << "; k_ = " << k_ << "; l_ = " << l_ << "; route_1_ = " << route_1_ << "; route_2_ = " << route_2_ <<endl;
//            cout << "initialize departures_path_2_[" <<  k_ + 1 << " ][ " << k_ + 1 << " ][ " << i_ << "]" << endl;
//            if (route_1_ == 0 && i_ == current_solution_->start_positions_[1] - 1)
//                cout << "initialize path_latest_departure_[" <<  k_ + 1 << " ][ " << k_ + 1 << "]" << endl;


            // INITIALIZATION STRUCTURES
            departures_path_2_standard_[1][k_ + 1][i_] = max(current_solution_->departures_[visit_i_] + data_.distances_[visit_i_][visit_succ_k_],
                                                       data_.start_TW_[visit_succ_k_])  + data_.service_time_[visit_succ_k_];
            departures_path_2_reversed_[1][k_ + 1][i_] = departures_path_2_standard_[1][k_ + 1][i_];

            if (route_1_ == 0 && i_ == length_route_1_ - 1) { // this also implies j_ == 0
                paths_load_[tour_index_k_ + 1][1] = data_.demands_[visit_succ_k_];
                path_latest_departure_standard_[tour_index_k_ + 1][1] = data_.latest_departure_possible_[visit_succ_k_];
                path_latest_departure_reversed_[tour_index_k_ + 1][1] = data_.latest_departure_possible_[visit_succ_k_];
//                path_cumulative_cost_standard_[tour_index_k_ + 1][0] = initializePathCost(tour_index_k_ + 1);
//                path_cumulative_cost_reversed_[tour_index_k_ + 1][0] = path_cumulative_cost_standard_[k_ + 1][0];
            }

            // FEASIBILITY VARIABLES
            // if the arc (k, i+1) is not feasible, path 1 will never be feasible, regardless if we increase j_ or l_
            if (data_.possible_arcs_[current_solution_->tour_[tour_index_k_]][current_solution_->tour_[tour_index_i_+1]]) {
                feasibility_to_path_1_standard_ = true;
                feasibility_to_path_1_reversed_ = true;
            }
            else {
                feasibility_to_path_1_standard_ = false;
                feasibility_to_path_1_reversed_ = false;
            }
            feasibility_route_2_reversed_ = false;
            feasibility_route_2_standard_ = false;
            feasibility_route_1_standard_ = false;
            feasibility_route_1_reversed_ = false;


            // path 1 is non existing as we initialized j_ = i_
            // path_2 check (if the arc (i, k+1) is not possible or we can't get to k+1 in time, path_2 is not reachable from i_
            if ((not data_.possible_arcs_[current_solution_->tour_[tour_index_i_]][current_solution_->tour_[tour_index_k_+1]]) or max(current_solution_->departures_[visit_i_] + data_.distances_[visit_i_][visit_succ_k_], data_.start_TW_[visit_succ_k_])  + data_.service_time_[visit_succ_k_] > path_latest_departure_standard_[tour_index_k_ + 1][1] ) {
                feasibility_to_path_2_standard_ = false;
                feasibility_to_path_2_reversed_ = false;
                continue;
            }
            else {
                feasibility_to_path_2_standard_ = true;
                feasibility_to_path_2_reversed_ = true;
            }


        }

            // if I can move i_ backward
        else if (i_ > 0 ) {
            --i_;
            j_ = 1;
            k_ = length_route_2_-1;
            l_ = 0;
            setVisits();
//            cout << " - i_ -->  ";
//            cout << " i_ = " << i_ << "; j_ = " << j_ << "; k_ = " << k_ << "; l_ = " << l_ << "; route_1_ = " << route_1_ << "; route_2_ = " << route_2_ <<endl;
//            cout << "initialize start_l_index_[" <<  i_ + 1 << " ][ " << i_ + 1 << " ][ " << k_ << "]" << endl;
//            cout << "initialize departures_path_1_[" <<  i_ + 1 << " ][ " << i_ + 1 << " ][ " << k_ << "]" << endl;
//            if (route_2_ == 1)
//                cout << "initialize path_latest_departure_[" <<  i_ + 1 << " ][ " << i_ + 1 << "]" << endl;

            // INITIALIZATION STRUCTURES
            departures_path_1_standard_[1][i_ + 1][k_] = max(current_solution_->departures_[visit_k_] + data_.distances_[visit_k_][visit_succ_i_], data_.start_TW_[visit_succ_i_]) + data_.service_time_[visit_succ_i_];
            departures_path_1_reversed_[1][i_ + 1][k_] = departures_path_1_standard_[1][i_ + 1][k_];
            if (route_2_ == 1) {
                paths_load_[tour_index_i_ + 1][1] = data_.demands_[visit_succ_i_];
                path_latest_departure_standard_[tour_index_i_ + 1][1] = data_.latest_departure_possible_[visit_succ_i_];
                path_latest_departure_reversed_[tour_index_i_ + 1][1] = path_latest_departure_standard_[tour_index_i_ + 1][0];
//                path_cumulative_cost_standard_[tour_index_i_ + 1][0] = initializePathCost(tour_index_i_ + 1);
//                path_cumulative_cost_reversed_[tour_index_i_ + 1][0] = path_cumulative_cost_standard_[tour_index_i_ + 1][0];
            }

            // FEASIBILITY VARIABLES
            feasibility_to_path_2_standard_ = true;
            feasibility_to_path_2_reversed_ = true;
            feasibility_route_2_reversed_ = false;
            feasibility_route_2_standard_ = false;

            // if the arc (k, i+1) is not possible or we can't get to i+1 in time, path_1 is not reachable from k_
            if ((not data_.possible_arcs_[current_solution_->tour_[tour_index_k_]][current_solution_->tour_[tour_index_i_+1]]) or max(current_solution_->departures_[visit_k_] + data_.distances_[visit_k_][visit_succ_i_], data_.start_TW_[visit_succ_i_])  + data_.service_time_[visit_succ_i_] > path_latest_departure_standard_[tour_index_i_ + 1][1]) {
                feasibility_to_path_1_standard_ = false;
                feasibility_to_path_1_reversed_ = false;
                continue;
            }
            else {
                feasibility_to_path_1_standard_ = true;
                feasibility_to_path_1_reversed_ = true;
            }
        }

        else if (route_2_ < current_solution_->n_routes_ - 1) {
            ++route_2_;
            length_route_2_ = current_solution_->start_positions_[route_2_+1] -  current_solution_->start_positions_[route_2_];
            i_ = length_route_1_ - 1;
            j_ = 0;
            k_ = length_route_2_ - 1;
            l_ = 0;
            continue;
        }

        else if (route_1_ < current_solution_->n_routes_ - 2) {
            ++route_1_;
            route_2_ = route_1_ + 1;
            length_route_1_ = current_solution_->start_positions_[route_1_+1] -  current_solution_->start_positions_[route_1_];
            length_route_2_ = current_solution_->start_positions_[route_2_+1] -  current_solution_->start_positions_[route_2_];
            i_ = length_route_1_ - 1;
            j_ = 0;
            k_ = length_route_2_ - 1;
            l_ = 0;
            continue;
        }

        else {
            has_next_ = false;
            break;
        }



        // PHASE 2 - do checks
        if ( j_ == 0) { // if we are only removing from route 2 ==> route_2 is feasible (only check route1/path 2)
            if (feasibility_to_path_2_standard_) // if I can get to path 2 standard in time, check cost and if I can come back in time
                if (feasibilityFromPath2Standard() &&  routes_load_[route_1_] + paths_load_[tour_index_k_ + 1][l_] <= modified_capacity_) {
                    foundMove(false, false);
                    return true;
                }


            // reverse path 2 (only if length is higher than 1), and we can reach path 2
            if (l_ > 1 && feasibility_to_path_2_reversed_)
                if (feasibilityFromPath2Reversed() && routes_load_[route_1_] + paths_load_[tour_index_k_ + 1][l_] <= modified_capacity_) {
                    foundMove(false, true);
                    return true;
                }

        } // ---------------------------------------------------------------------------------------------------------


        else if (l_ == 0) { // if we are only removing from route 1 ==> route_1 is feasible (only check route2/path 1)
            // standard path 1
            if (feasibility_to_path_1_standard_) {// if I can get to path 1 standard in time, check cost and if I can come back in time
                if (feasibility_route_2_standard_) {
                    foundMove(false, false);
                    return true;
                }
                else if (feasibilityFromPath1Standard() && // && l_ >= start_l_index_[j_][i_ + 1][k_]
                         routes_load_[route_2_] + paths_load_[tour_index_i_ + 1][j_] <=
                         modified_capacity_) {
                    feasibility_route_2_standard_ = true;
                    foundMove(false, false);
                    return true;
                }
//                else
//                    start_l_index_[j_][i_ + 1][k_] = l_;

            }

            // reverse path ( if there's more than one node)
            if (j_ > 1 && feasibility_to_path_1_reversed_) {
                if (feasibility_route_2_reversed_) {
                    foundMove(true, false);
                    return true;
                }
                else if (feasibilityFromPath1Reversed() && routes_load_[route_2_] + paths_load_[tour_index_i_ + 1][j_] <= modified_capacity_) { // &&  l_ >= start_l_index_[i_+1][j_][k_]
                        foundMove(true, false);
                        feasibility_route_2_reversed_ = true;
                        return true;
                }
//                else
//                    start_l_index_[i_ + 1][j_][k_] = l_;
            }
        } // ---------------------------------------------------------------------------------------------------------

        // we are exchanging paths
        else {
            // standard path 2
            if (feasibility_to_path_2_standard_) {
                // standard path 1
                if (feasibility_to_path_1_standard_) { // if we can reach path 1
                    // check route 1 (only time coming back from path and load)
                    if (feasibilityFromPath2Standard() && feasibilityCapacityRoute1()) {
                        feasibility_route_1_standard_ = true;
                        // check route 2
                        if (feasibility_route_2_standard_)  { // if we already know it's feasible
                            foundMove(false, false);
                            return true;
                        }
                        else if (feasibilityFromPath1Standard() && feasibilityCapacityRoute2()) { // && l_ >= start_l_index_[j_][i_ + 1][k_]
                            feasibility_route_2_standard_ = true;
                            foundMove(false, false);
                            return true;
                        }
//                        else
//                            start_l_index_[j_][i_ + 1][k_] = l_;
                    }
                }

                //reverse path 1 (only if there's more than one node)
                if (j_ > 1 && feasibility_to_path_1_reversed_) {
                    // if we already know route 1 is feasible (from above check) or route 1 is actually feasible
                    if (feasibility_route_1_standard_ or (feasibilityFromPath2Standard() && feasibilityCapacityRoute1())) {
                        // check route 2
                        if (feasibility_route_2_reversed_) {
                            foundMove(true, false);
                            return true;
                        }
                        else if (feasibilityFromPath1Reversed() && feasibilityCapacityRoute2()) { // && l_ >= start_l_index_[i_ + 1][j_][k_]
                            feasibility_route_2_reversed_ = true;
                            foundMove(true, false);
                            return true;
                        }
//                        else
//                            start_l_index_[i_ + 1][j_][k_] = l_;
                    }
                }
            }


            // reverse path 2, if there is more than node
            if (l_ > 1 && feasibility_to_path_2_reversed_) {
                // standard path 1
                if (feasibility_to_path_1_standard_) { // if we can reach path 1 (reversed)
                    // check route 1 (only time coming back from path and load)
                    if (feasibilityFromPath2Reversed() && feasibilityCapacityRoute1()) {
                        feasibility_route_1_reversed_ = true;
                        // check route 2
                        if (feasibility_route_2_standard_) {// if we already know it's feasible
                            foundMove(true, false);
                            return true;
                        }
                        else if (feasibilityFromPath1Standard() && // && l_ >= start_l_index_[i_ + 1][j_][k_]
                                 feasibilityCapacityRoute2()) {
                            feasibility_route_2_standard_ = true;
                            foundMove(true, false);
                            return true;
                        }
//                        else
//                            start_l_index_[i_ + 1][j_][k_] = l_;
                    }
                }

                //reverse path 1 (only if there's more than one node)
                if (j_ > 1 && feasibility_to_path_1_reversed_) {
                    // if we already know route 1 is feasible (from above check) or route 1 is actually feasible
                    if (feasibility_route_1_reversed_ or (feasibilityFromPath2Reversed() && feasibilityCapacityRoute1())) {
                        // check route 2
                        if (feasibility_route_2_reversed_) {
                            foundMove(true, true);
                            return true;
                        }
                        else if (feasibilityFromPath1Reversed() && feasibilityCapacityRoute2()) { // && l_ >= start_l_index_[j_][i_ + 1][k_]
                            feasibility_route_2_reversed_ = true;
                            foundMove(true, true);
                            return true;
                        }
//                        else
//                            start_l_index_[j_][i_ + 1][k_] = l_;
                    }
                }
            }
        }
    } while (has_next_);
    return false;
}


/** -------------------------------------------------------------------------------- */
inline double CrossReverseGenerator::deadlineComingBackToRoute1() {return visit_succ_j_ >= data_.n_requests_ ? data_.end_TW_.back() : current_solution_->latest_departures_[visit_succ_j_]; }
inline double CrossReverseGenerator::deadlineComingBackToRoute2() {return visit_succ_l_ >= data_.n_requests_ ? data_.end_TW_.back() : current_solution_->latest_departures_[visit_succ_l_]; }

inline bool CrossReverseGenerator::feasibilityFromPath1Standard() {
    return max(departures_path_1_standard_[j_][i_ + 1][k_] + data_.distances_[visit_j_][visit_succ_l_], data_.start_TW_[visit_succ_l_]) + data_.service_time_[visit_succ_l_] <=
            deadlineComingBackToRoute2();
}

inline bool CrossReverseGenerator::feasibilityFromPath1Reversed() {
    return max(departures_path_1_reversed_[j_][i_ + 1][k_] + data_.distances_[visit_succ_i_][visit_succ_l_], data_.start_TW_[visit_succ_l_]) + data_.service_time_[visit_succ_l_] <=
            deadlineComingBackToRoute2();
}

inline bool CrossReverseGenerator::feasibilityFromPath2Standard() {
    return max(departures_path_2_standard_[l_][k_ + 1][i_] + data_.distances_[visit_l_][visit_succ_j_], data_.start_TW_[visit_succ_j_]) + data_.service_time_[visit_succ_j_] <=
            deadlineComingBackToRoute1();
}

inline bool CrossReverseGenerator::feasibilityFromPath2Reversed() {
    return max(departures_path_2_reversed_[l_][k_ + 1][i_] + data_.distances_[visit_succ_k_][visit_succ_j_], data_.start_TW_[visit_succ_j_]) + data_.service_time_[visit_succ_j_] <=
            deadlineComingBackToRoute1();
}

inline bool CrossReverseGenerator::feasibilityCapacityRoute1() {
    return routes_load_[route_1_] + paths_load_[tour_index_k_ + 1][l_] - paths_load_[tour_index_i_ + 1][j_] <=  modified_capacity_;
}

inline bool CrossReverseGenerator::feasibilityCapacityRoute2() {
    return routes_load_[route_2_] + paths_load_[tour_index_i_ + 1][j_] - paths_load_[tour_index_k_ + 1][l_] <= modified_capacity_;
}


/** ------------------------------------------------------------------------------------------------ */


void CrossReverseGenerator::foundMove(bool reverse_path_1, bool reverse_path_2) {
    move_ -> setMoveVariables(tour_index_i_, tour_index_j_, tour_index_k_, tour_index_l_, route_1_, route_2_, reverse_path_1, reverse_path_2);

    // set delta of the move to make the computation of the total delta easier
    computeDeltaDistance(reverse_path_1, reverse_path_2, move_ -> delta_distance_);
    computeDeltaCapacity(move_ -> delta_capacity_);
    move_ -> route_removed_ = (i_ == 0 and j_ == length_route_1_ - 1 and l_ == 0) or (k_ == 0 and l_ == length_route_2_ - 1 and j_ == 0);


//    // set tabu status
//    if (j_ == 0) {
//        if (not reverse_path_2)
//            move_->is_tabu_ = ((*move_->forbidden_arcs_)[visit_i_][visit_succ_k_] > 0) or
//                              ((*move_->forbidden_arcs_)[visit_l_][visit_succ_i_] > 0);
//        else
//            move_->is_tabu_ = ((*move_->forbidden_arcs_)[visit_i_][visit_l_] > 0) or
//                              ((*move_->forbidden_arcs_)[visit_succ_k_][visit_succ_i_] > 0);
//    }
//
//    else if (l_ == 0) {
//        if (not reverse_path_1)
//            move_->is_tabu_ = ((*move_->forbidden_arcs_)[visit_k_][visit_succ_i_] > 0) or
//                              ((*move_->forbidden_arcs_)[visit_j_][visit_succ_k_] > 0);
//        else
//            move_->is_tabu_ = ((*move_->forbidden_arcs_)[visit_k_][visit_j_] > 0) or
//                              ((*move_->forbidden_arcs_)[visit_succ_i_][visit_succ_k_] > 0);
//    }
//
//    else {
//        bool route_1_taboo;
//        if (not reverse_path_2)
//            route_1_taboo = ((*move_->forbidden_arcs_)[visit_i_][visit_succ_k_] > 0) or
//                            ((*move_->forbidden_arcs_)[visit_l_][visit_succ_j_] > 0);
//        else
//            route_1_taboo = ((*move_->forbidden_arcs_)[visit_i_][visit_l_] > 0) or
//                            ((*move_->forbidden_arcs_)[visit_succ_k_][visit_succ_j_] > 0);
//
//        if (not reverse_path_1)
//            move_->is_tabu_ = route_1_taboo or ((*move_->forbidden_arcs_)[visit_k_][visit_succ_i_] > 0) or
//                              ((*move_->forbidden_arcs_)[visit_j_][visit_succ_l_] > 0);
//        else
//            move_->is_tabu_ = route_1_taboo or ((*move_->forbidden_arcs_)[visit_k_][visit_j_] > 0) or
//                              ((*move_->forbidden_arcs_)[visit_succ_i_][visit_succ_l_] > 0);
//    }
}


/** ------------------------------------------------------------------------------------------------ */

void CrossReverseGenerator::computeRoutesLoad()
{
    for (int route =0; route < current_solution_->n_routes_; ++route) {
        int load = 0;
        for (int position = (current_solution_ -> start_positions_[route] + 1); position < current_solution_ -> start_positions_[route+1]; ++position) {
            load += data_.demands_[current_solution_ -> tour_[position]];
        }
        routes_load_[route] = load;
    }
}

/** ------------------------------------------------------------------------------------------------ */


void CrossReverseGenerator::setVisits() {
    tour_index_i_ = current_solution_->start_positions_[route_1_] + i_;
    tour_index_j_ = current_solution_->start_positions_[route_1_] + i_ + j_;
    tour_index_k_ = current_solution_->start_positions_[route_2_] + k_;
    tour_index_l_ = current_solution_->start_positions_[route_2_] + k_ + l_;

    visit_i_ = current_solution_->tour_[tour_index_i_];
    visit_j_ = current_solution_->tour_[tour_index_j_];
    visit_k_ = current_solution_->tour_[tour_index_k_];
    visit_l_ = current_solution_->tour_[tour_index_l_];
    visit_succ_i_ = current_solution_->tour_[tour_index_i_ + 1];
    visit_succ_j_ = current_solution_->tour_[tour_index_j_ + 1];
    visit_succ_k_ = current_solution_->tour_[tour_index_k_ + 1];
    visit_succ_l_ = current_solution_->tour_[tour_index_l_ + 1];
}


/** ------------------------------------------------------------------------------------------------ */


void CrossReverseGenerator::computeDeltaDistance(bool reverse_path_1, bool reverse_path_2, double &delta_distance) {
    if (j_ == 0) { // if we are only removing from route 2 ==> route_2 is feasible (only check route1/path 2)
        delta_distance = data_.distances_[visit_k_][visit_succ_l_] - data_.distances_[visit_i_][visit_succ_i_] -
                      data_.distances_[visit_k_][visit_succ_k_] - data_.distances_[visit_l_][visit_succ_l_];
        if (reverse_path_2)
            delta_distance += data_.distances_[visit_i_][visit_l_] + data_.distances_[visit_succ_k_][visit_succ_j_];
        else
            delta_distance += data_.distances_[visit_i_][visit_succ_k_] + data_.distances_[visit_l_][visit_succ_j_];

    }
    else if (l_ == 0) { // if we are only removing from route 1 ==> route_1 is feasible (only check route2/path 1)
        delta_distance = data_.distances_[visit_i_][visit_succ_j_] - data_.distances_[visit_i_][visit_succ_i_] -
                      data_.distances_[visit_j_][visit_succ_j_] - data_.distances_[visit_k_][visit_succ_k_];
        if (reverse_path_1)
            delta_distance += data_.distances_[visit_k_][visit_j_] + data_.distances_[visit_succ_i_][visit_succ_l_];
        else
            delta_distance += data_.distances_[visit_k_][visit_succ_i_] + data_.distances_[visit_j_][visit_succ_l_];

    }
    else {
        double intermidiate_delta;
        intermidiate_delta = -data_.distances_[visit_i_][visit_succ_i_] - data_.distances_[visit_j_][visit_succ_j_] -
                      data_.distances_[visit_k_][visit_succ_k_] - data_.distances_[visit_l_][visit_succ_l_];
        if (reverse_path_2) {
            intermidiate_delta += data_.distances_[visit_i_][visit_l_] + data_.distances_[visit_succ_k_][visit_succ_j_];
            if (reverse_path_1)
                delta_distance = intermidiate_delta + data_.distances_[visit_k_][visit_j_] + data_.distances_[visit_succ_i_][visit_succ_l_];
            else
                delta_distance = intermidiate_delta + data_.distances_[visit_k_][visit_succ_i_] + data_.distances_[visit_j_][visit_succ_l_];
        }
        else {
            intermidiate_delta += data_.distances_[visit_i_][visit_succ_k_] + data_.distances_[visit_l_][visit_succ_j_];
            if (reverse_path_1)
                delta_distance = intermidiate_delta + data_.distances_[visit_k_][visit_j_] + data_.distances_[visit_succ_i_][visit_succ_l_];
            else
                delta_distance = intermidiate_delta + data_.distances_[visit_k_][visit_succ_i_] + data_.distances_[visit_j_][visit_succ_l_];
        }
    }
}


/** ------------------------------------------------------------------------------------------------ */


void CrossReverseGenerator::computeDeltaCapacity(double &delta_capacity) {
    if (modified_capacity_ == data_.capacity_)
        delta_capacity = 0;
    else {
        if (j_ > 0 and l_ > 0)
            delta_capacity = max(routes_load_[route_1_], data_.capacity_) -
                             max(routes_load_[route_1_] + paths_load_[tour_index_k_ + 1][l_] -
                                 paths_load_[tour_index_i_ + 1][j_], data_.capacity_) +
                             max(routes_load_[route_2_], data_.capacity_) -
                             max(routes_load_[route_2_] - paths_load_[tour_index_k_ + 1][l_] +
                                 paths_load_[tour_index_i_ + 1][j_], data_.capacity_);
        else if (j_ > 0)
            delta_capacity = max(routes_load_[route_1_], data_.capacity_) -
                             max(routes_load_[route_1_] - paths_load_[tour_index_i_ + 1][j_], data_.capacity_) +
                             max(routes_load_[route_2_], data_.capacity_) -
                             max(routes_load_[route_2_] + paths_load_[tour_index_i_ + 1][j_], data_.capacity_);
        else
            delta_capacity = max(routes_load_[route_1_], data_.capacity_) -
                             max(routes_load_[route_1_] + paths_load_[tour_index_k_ + 1][l_], data_.capacity_) +
                             max(routes_load_[route_2_], data_.capacity_) -
                             max(routes_load_[route_2_] - paths_load_[tour_index_k_ + 1][l_], data_.capacity_);
    }
}


/** ------------------------------------------------------------------------------------------------ */


