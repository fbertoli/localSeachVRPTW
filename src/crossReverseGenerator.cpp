//
// Created by Bertoli, Francesco (Data61, Canberra City) on 09/02/17.
//



#include "crossReverseGenerator.h"
#include <limits>



/** CONSTRUCTORS */

CrossReverseGenerator::CrossReverseGenerator(const Data &data, Solution* sol, double capacity_coefficient) :
        MoveGenerator::MoveGenerator(data, sol), capacity_coefficient_(capacity_coefficient),
        max_length_path_opt_("max-path", "maximum path length in crossReverse", 4),
        max_length_path_(max_length_path_opt_),
        reduced_cost_option_(false)
{
    routes_load_.resize(data_.max_vehicles_);
    int size{data_.n_requests_ + data_.max_vehicles_};
    paths_load_.resize(size);
    path_cumulative_cost.resize(size);
    path_latest_departure_.resize(size);
    start_l_index_.resize(size);
    departures_path_1_.resize(size);
    departures_path_2_.resize(size);

    for( int i = 0; i < size; ++ i ) {
        paths_load_[i].resize(size, 0);
        path_cumulative_cost[i].resize(size, 0);
        path_latest_departure_[i].resize(size, 0);
        start_l_index_[i].resize(size);
        departures_path_1_[i].resize(size);
        departures_path_2_[i].resize(size);
        for( int j = 0; j < size; ++ j ) {
            start_l_index_[i][j].resize(size, 0);
            departures_path_1_[i][j].resize(size);
            departures_path_2_[i][j].resize(size);
        }
    }
}

/** --------------------------------------------------------------------------------
 *  -------------------------------------------------------------------------------- */


/** METHODS */

bool CrossReverseGenerator::random(Move &raw_move) {return false;}


bool CrossReverseGenerator::first(Move &raw_move) {

    route_1_ = 0;
    route_2_ = 1;
    i_ = current_solution_->start_positions_[route_1_+1] - 1;
    j_ = i_;
    k_ = current_solution_->start_positions_[route_2_+1] - 1;
    l_ = k_;
    best_delta_ = numeric_limits<double>::infinity();
    has_next_ = true;
    move_found_ = false;
    computeRoutesLoad();
    return next(raw_move);
}

/** -------------------------------------------------------------------------------- */


bool CrossReverseGenerator::next(Move &raw_move) {

    CrossReverseMove &move = static_cast<CrossReverseMove&>(raw_move);

    // PHASE 1 - update indexed and data structures
    do {

        // if I can move l_ forward
        if (l_ < min(k_ + max_length_path_, current_solution_->start_positions_[route_2_ + 1] - 1)) {
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
            if (j_ == i_ ) { // need to do these computations only for one pass of j
                //  forward
                departures_path_2_[l_][k_+1][i_] = max(departures_path_2_[l_ - 1][k_ + 1][i_] + data_.true_distances_[current_solution_->tour_[l_ - 1]][visit_l_],
                                                       data_.start_TW_[visit_l_]) + data_.service_time_[visit_l_];
                //  backward
                departures_path_2_[k_+1][l_][i_] = max(departures_path_2_[k_ + 2][l_][i_] + data_.true_distances_[visit_succ_k_][current_solution_->tour_[k_ + 2]],
                                                       data_.start_TW_[current_solution_->tour_[k_ + 2]]) + data_.service_time_[current_solution_->tour_[k_ + 2]];
            }

            // need to do it only for one pass of i_, j_ and route_1_/2_
            if (route_1_ == 0 && i_ == current_solution_->start_positions_[route_1_+1] - 1) {
                //  forward
                paths_load_[k_ + 1][l_] = paths_load_[k_ + 1][l_ - 1] + data_.demands_[visit_l_];
                //  forward
                path_cumulative_cost[k_ + 1][l_] = path_cumulative_cost[k_ + 1][l_ - 1] + data_.true_distances_[current_solution_->tour_[l_ - 1]][visit_l_] - dual_variables_->at(visit_l_);
                // (standard orientation)  backward
                path_latest_departure_[k_ + 1][l_] = min(path_latest_departure_[k_ + 2][l_] - data_.true_distances_[visit_succ_k_][current_solution_->tour_[k_+2]] - data_.service_time_[current_solution_->tour_[k_+2]], data_.latest_departure_possible_[visit_succ_k_]);
                // (reverse orientation) forward
                path_latest_departure_[l_][k_ + 1] = min(path_latest_departure_[l_ - 1][k_ + 1] - data_.true_distances_[visit_l_][current_solution_->tour_[l_ - 1]] - data_.service_time_[current_solution_->tour_[l_ - 1]], data_.latest_departure_possible_[visit_l_]);
            }


            // FEASIBILITY VARIABLES
            feasibility_route_1_standard_ = false;
            feasibility_route_1_reversed_ = false;

            // feasibility of arriving at path 1 does not change when l_ increases (feasibility from path 1 is checked later)
            // standard
            if (feasibility_to_path_2_standard_ and max(current_solution_-> departures_[visit_i_] + data_.true_distances_[visit_i_][visit_succ_k_], data_.start_TW_[visit_succ_k_]) + data_.service_time_[visit_succ_k_] > path_latest_departure_[k_+1][l_] )
                feasibility_to_path_2_standard_ = false;
            // reversed
            if (feasibility_to_path_2_reversed_ and max(current_solution_->departures_[visit_i_] + data_.true_distances_[visit_i_][visit_l_], data_.start_TW_[visit_l_]) + data_.service_time_[visit_l_]  > path_latest_departure_[l_][k_+1] )
                feasibility_to_path_2_reversed_ = false;


            // CHECKS
            if (not feasibility_to_path_2_standard_ and not feasibility_to_path_2_reversed_)
                continue;

        }


            // if I can move j_ forward
        else if (j_ < min(i_ + max_length_path_, current_solution_->start_positions_[route_1_ + 1] - 1)) {
            ++j_;
            l_ = k_;
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
            if (j_ == i_ + 1) {
                // INITIALIZE
                departures_path_1_[i_ + 1][i_ + 1][k_] = max(current_solution_->departures_[visit_k_] + data_.true_distances_[visit_k_][visit_succ_i_], data_.start_TW_[visit_succ_i_]) + data_.service_time_[visit_succ_i_];
                start_l_index_[i_ + 1][i_ + 1][k_] = 0;
            }
            else {
                // PROPAGATE
                // (standard orientation)  forward
                departures_path_1_[j_][i_ + 1][k_] = max(departures_path_1_[j_ - 1][i_ + 1][k_] +
                                                         data_.true_distances_[current_solution_->tour_[j_ - 1]][visit_j_],
                                                         data_.start_TW_[visit_j_]) + data_.service_time_[visit_j_];
                // (reverse orientation) backward
                departures_path_1_[i_ + 1][j_][k_] = max(departures_path_1_[i_ + 2][j_][k_] +
                                                         data_.true_distances_[current_solution_->tour_[i_ + 2]][visit_succ_i_],
                                                         data_.start_TW_[visit_succ_i_]) +
                                                         data_.service_time_[visit_succ_i_];

                //propagate bacward and foward for both directions and take the minimum
                start_l_index_[j_][i_ + 1][k_] = max(start_l_index_[j_][i_ + 2][k_],
                                                     start_l_index_[j_ - 1][i_ + 1][k_]);
                start_l_index_[i_ + 1][j_][k_] = max(start_l_index_[i_ + 2][j_][k_],
                                                     start_l_index_[i_ + 1][j_ - 1][k_]);

                // progate path_ information on the very first route)
                if (k_ == current_solution_-> start_positions_[2] - 1) {
                    // propagation
                    // forward
                    paths_load_[i_ + 1][j_] = paths_load_[i_ + 1][j_ - 1] + data_.demands_[visit_j_];
                    // forward
                    path_cumulative_cost[i_ + 1][j_] = path_cumulative_cost[i_ + 1][j_ - 1] + data_.true_distances_[current_solution_->tour_[j_ - 1]][visit_j_] - dual_variables_->at(visit_j_);
                    // (standard orientation) backward
                    path_latest_departure_[i_ + 1][j_] = min(path_latest_departure_[i_ + 2][j_] -
                                                             data_.true_distances_[visit_succ_i_][current_solution_->tour_[i_ + 2]] -
                                                             data_.service_time_[current_solution_->tour_[i_ + 2]],
                                                             data_.latest_departure_possible_[visit_succ_i_]);
                    // (reverse orientation) forward
                    path_latest_departure_[j_][i_ + 1] = min(path_latest_departure_[j_ - 1][i_ + 1] -
                                                             data_.true_distances_[visit_j_][current_solution_->tour_[j_ - 1]] -
                                                             data_.service_time_[current_solution_->tour_[j_ - 1]],
                                                             data_.latest_departure_possible_[visit_j_]);
                }
            }



            // FEASIBILITY VARIABLES
            feasibility_route_2_reversed_ = false;
            feasibility_route_2_standard_ = false;

            // feasibility of arriving at path 2 does not change when j_ increases (feasibility from path 2 is checked later)
            // if j_ == i_ + 1 feasibility_to_path_1_standar/reverse are true because a previous resetting of k set them true
            // standard
            if (feasibility_to_path_1_standard_ && max(current_solution_->departures_[visit_k_] + data_.true_distances_[visit_k_][visit_succ_i_], data_.start_TW_[visit_succ_i_]) + data_.service_time_[visit_succ_i_] > path_latest_departure_[i_+1][j_] )
                feasibility_to_path_1_standard_ = false;
            // reversed
            if (feasibility_to_path_1_reversed_ && max(current_solution_->departures_[visit_k_] + data_.true_distances_[visit_k_][visit_j_], data_.start_TW_[visit_j_]) + data_.service_time_[visit_j_] > path_latest_departure_[j_][i_+1] )
                feasibility_to_path_1_reversed_ = false;

            // CHECKS
            if (not feasibility_to_path_1_standard_ && not feasibility_to_path_1_reversed_)
                continue;

        }


            // if I can move k_ backward
        else if (k_ > current_solution_-> start_positions_[route_2_]) {
            --k_;
            j_ = i_;
            l_ = k_ + 1;
            setVisits();
//            cout << " - k_ -->  ";
//            cout << " i_ = " << i_ << "; j_ = " << j_ << "; k_ = " << k_ << "; l_ = " << l_ << "; route_1_ = " << route_1_ << "; route_2_ = " << route_2_ <<endl;
//            cout << "initialize departures_path_2_[" <<  k_ + 1 << " ][ " << k_ + 1 << " ][ " << i_ << "]" << endl;
//            if (route_1_ == 0 && i_ == current_solution_->start_positions_[1] - 1)
//                cout << "initialize path_latest_departure_[" <<  k_ + 1 << " ][ " << k_ + 1 << "]" << endl;


            // INITIALIZATION STRUCTURES
            departures_path_2_[k_ + 1][k_ + 1][i_] = max(current_solution_->departures_[visit_i_] + data_.true_distances_[visit_i_][visit_succ_k_],
                                                       data_.start_TW_[visit_succ_k_])  + data_.service_time_[visit_succ_k_];

            if (route_1_ == 0 && i_ == current_solution_->start_positions_[1] - 1) { // this also implies j_ == i_
                paths_load_[k_ + 1][k_ + 1] = data_.demands_[visit_succ_k_];
                path_cumulative_cost[k_ + 1][k_ + 1] = - dual_variables_->at(visit_succ_k_);
                path_latest_departure_[k_ + 1][k_ + 1] = data_.latest_departure_possible_[visit_succ_k_];
            }

            // FEASIBILITY VARIABLES
            feasibility_to_path_1_standard_ = true;
            feasibility_to_path_1_reversed_ = true;
            feasibility_route_2_reversed_ = false;
            feasibility_route_2_standard_ = false;
            feasibility_route_1_standard_ = false;
            feasibility_route_1_reversed_ = false;


            // path 1 is non existing as we initialized j_ = i_
            // path_2 check
            if (max(current_solution_->departures_[visit_i_] + data_.true_distances_[visit_i_][visit_succ_k_], data_.start_TW_[visit_succ_k_])  + data_.service_time_[visit_succ_k_] > path_latest_departure_[k_ + 1][k_ + 1] ) {
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
        else if (i_ > current_solution_-> start_positions_[route_1_] ) {
            --i_;
            j_ = i_ + 1;
            k_ = current_solution_->start_positions_[route_2_+1]-1;
            l_ = k_;
            setVisits();
//            cout << " - i_ -->  ";
//            cout << " i_ = " << i_ << "; j_ = " << j_ << "; k_ = " << k_ << "; l_ = " << l_ << "; route_1_ = " << route_1_ << "; route_2_ = " << route_2_ <<endl;
//            cout << "initialize start_l_index_[" <<  i_ + 1 << " ][ " << i_ + 1 << " ][ " << k_ << "]" << endl;
//            cout << "initialize departures_path_1_[" <<  i_ + 1 << " ][ " << i_ + 1 << " ][ " << k_ << "]" << endl;
//            if (route_2_ == 1)
//                cout << "initialize path_latest_departure_[" <<  i_ + 1 << " ][ " << i_ + 1 << "]" << endl;

            // (re)-INITIALIZATION STRUCTURES
            departures_path_1_[i_ + 1][i_ + 1][k_] = max(current_solution_->departures_[visit_k_] + data_.true_distances_[visit_k_][visit_succ_i_], data_.start_TW_[visit_succ_i_]) + data_.service_time_[visit_succ_i_];
            if (route_2_ == 1) {
                paths_load_[i_ + 1][i_ + 1] = data_.demands_[visit_succ_i_];
                path_cumulative_cost[i_ + 1][i_ + 1] = dual_variables_->at(visit_succ_i_);
                path_latest_departure_[i_ + 1][i_ + 1] = data_.latest_departure_possible_[visit_succ_i_];
            }

            // FEASIBILITY VARIABLES
            feasibility_to_path_2_standard_ = true;
            feasibility_to_path_2_reversed_ = true;
            feasibility_route_2_reversed_ = false;
            feasibility_route_2_standard_ = false;

            // both
            if (max(current_solution_->departures_[visit_k_] + data_.true_distances_[visit_k_][visit_succ_i_], data_.start_TW_[visit_succ_i_])  + data_.service_time_[visit_succ_i_] > path_latest_departure_[i_ + 1][i_ + 1]) {
                feasibility_to_path_1_standard_ = false;
                feasibility_to_path_1_reversed_ = false;
                start_l_index_[i_ + 1][i_ + 1][k_] = l_;
                continue;
            }
            else {
                feasibility_to_path_1_standard_ = true;
                feasibility_to_path_1_reversed_ = true;
                start_l_index_[i_ + 1][i_ + 1][k_] = 0;
            }
        }

        else if (route_2_ < current_solution_->n_routes_ - 1) {
            ++route_2_;
            i_ = current_solution_->start_positions_[route_1_+1] - 1;
            j_ = i_;
            k_ = current_solution_->start_positions_[route_2_+1] - 1;
            l_ = k_;
            continue;
        }

        else if (route_1_ < current_solution_->n_routes_ - 2) {
            ++route_1_;
            route_2_ = route_1_ + 1;
            i_ = current_solution_->start_positions_[route_1_+1] - 1;
            j_ = i_;
            k_ = current_solution_->start_positions_[route_2_+1] - 1;
            l_ = k_;
            continue;
        }

        else {
            has_next_ = false;
            break;
        }



        // PHASE 2 - do checks
        if ( i_ == j_) { // if we are only removing from route 2 ==> route_2 is feasible (only check route1/path 2)
            if (feasibility_to_path_2_standard_) {// if I can get to path 2 standard in time, check cost and if I can come back in time
                computeDelta(true, true);
                if (delta_ < best_delta_) {
                    if (feasibilityFromPath2Standard() &&  routes_load_[route_1_] + paths_load_[k_ + 1][l_] <= data_.capacity_ * capacity_coefficient_)
                        foundMove(delta_, move, false, false);
                }
            }

            // reverse path 2 (only if length is higher than 1), and we can reach path 2
            if (l_ > k_ + 1 && feasibility_to_path_2_reversed_) {
                computeDelta(true, false);
                if (delta_ < best_delta_) {
                    if (feasibilityFromPath2Reversed() && routes_load_[route_1_] + paths_load_[k_ + 1][l_] <= data_.capacity_ * capacity_coefficient_)
                        foundMove(delta_, move, false, true);
                }
            }
        } // ---------------------------------------------------------------------------------------------------------


        else if (k_ == l_) { // if we are only removing from route 1 ==> route_1 is feasible (only check route2/path 1)
            // standard path 1
            if (feasibility_to_path_1_standard_) {// if I can get to path 1 standard in time, check cost and if I can come back in time
                computeDelta(true, true);
                if (delta_ < best_delta_) {
                    if (feasibility_route_2_standard_)
                        foundMove(delta_, move, false, false);
                    else if (l_ >= start_l_index_[j_][i_ + 1][k_] && feasibilityFromPath1Standard() &&
                             routes_load_[route_2_] + paths_load_[i_ + 1][j_] <=
                             data_.capacity_ * capacity_coefficient_) {
                        feasibility_route_2_standard_ = true;
                        foundMove(delta_, move, false, false);
                    }
                    else
                        start_l_index_[j_][i_ + 1][k_] = l_;
                }
            }

            // reverse path ( if there's more than one node)
            if (j_ > i_ + 1 && feasibility_to_path_1_reversed_) {
                computeDelta(false, true);
                if (delta_ < best_delta_) {
                    if (feasibility_route_2_reversed_)
                        foundMove(delta_, move, true, false);
                    else if (l_ >= start_l_index_[i_+1][j_][k_] && feasibilityFromPath1Reversed() && routes_load_[route_2_] + paths_load_[i_ + 1][j_] <= data_.capacity_ * capacity_coefficient_) {
                            foundMove(delta_, move, true, false);
                            feasibility_route_2_reversed_ = true;
                    }
                    else
                        start_l_index_[i_ + 1][j_][k_] = l_;
                }
            }
        } // ---------------------------------------------------------------------------------------------------------

        // we are exchanging paths
        else {
            // standard path 2
            if (feasibility_to_path_2_standard_) {
                // standard path 1
                if (feasibility_to_path_1_standard_) { // if we can reach path 1
                    // cost check
                    computeDelta(true, true);
                    if (delta_ < best_delta_) {
                        // check route 1 (only time coming back from path and load)
                        if (feasibilityFromPath2Standard() && feasibilityCapacityRoute1()) {
                            feasibility_route_1_standard_ = true;
                            // check route 2
                            if (feasibility_route_2_standard_) // if we already know it's feasible
                                foundMove(delta_, move, false, false);
                            else if (l_ >= start_l_index_[j_][i_ + 1][k_] && feasibilityFromPath1Standard() && feasibilityCapacityRoute2()) {
                                feasibility_route_2_standard_ = true;
                                foundMove(delta_, move, false, false);
                            }
                            else
                                start_l_index_[j_][i_ + 1][k_] = l_;
                        }
                    }
                }

                //reverse path 1 (only if there's more than one node)
                if (j_ > i_ + 1 && feasibility_to_path_1_reversed_) {
                    // cost check
                    computeDelta(false, true);
                    if (delta_ < best_delta_) {
                        // if we already know route 1 is feasible (from above check) or route 1 is actually feasible
                        if (feasibility_route_1_standard_ or (feasibilityFromPath2Standard() && feasibilityCapacityRoute1())) {
                            // check route 2
                            if (feasibility_route_2_reversed_)
                                foundMove(delta_, move, true, false);
                            else if (l_ >= start_l_index_[i_ + 1][j_][k_] && feasibilityFromPath1Reversed() && feasibilityCapacityRoute2()) {
                                feasibility_route_2_reversed_ = true;
                                foundMove(delta_, move, true, false);
                            }
                            else
                                start_l_index_[i_ + 1][j_][k_] = l_;
                        }
                    }
                }
            }


            // reverse path 2, if there is more than node
            if (l_ > k_ + 1 && feasibility_to_path_2_reversed_) {
                // standard path 1
                if (feasibility_to_path_1_standard_) { // if we can reach path 1 (reversed)
                    // cost check
                    computeDelta(true, false);
                    if (delta_ < best_delta_) {
                        // check route 1 (only time coming back from path and load)
                        if (feasibilityFromPath2Reversed() && feasibilityCapacityRoute1()) {
                            feasibility_route_1_reversed_ = true;
                            // check route 2
                            if (feasibility_route_2_standard_) // if we already know it's feasible
                                foundMove(delta_, move, true, false);
                            else if (l_ >= start_l_index_[i_ + 1][j_][k_] && feasibilityFromPath1Standard() &&
                                     feasibilityCapacityRoute2()) {
                                feasibility_route_2_standard_ = true;
                                foundMove(delta_, move, true, false);
                            }
                            else
                                start_l_index_[i_ + 1][j_][k_] = l_;
                        }
                    }
                }

                //reverse path 1 (only if there's more than one node)
                if (j_ > i_ + 1 && feasibility_to_path_1_reversed_) {
                    // cost check
                    computeDelta(false, false);
                    if (delta_ < best_delta_) {
                        // if we already know route 1 is feasible (from above check) or route 1 is actually feasible
                        if (feasibility_route_1_reversed_ or (feasibilityFromPath2Reversed() && feasibilityCapacityRoute1())) {
                            // check route 2
                            if (feasibility_route_2_reversed_)
                                foundMove(delta_, move, true, true);
                            else if (l_ >= start_l_index_[j_][i_ + 1][k_] && feasibilityFromPath1Reversed() && feasibilityCapacityRoute2()) {
                                feasibility_route_2_reversed_ = true;
                                foundMove(delta_, move, true, true);
                            }
                            else
                                start_l_index_[j_][i_ + 1][k_] = l_;
                        }
                    }
                }
            }
        }

    if (move_found_) {
        move_found_ = false;
        updateMove(move);
        return true;
    }

    } while (has_next_);

    return false;
}


/** -------------------------------------------------------------------------------- */
inline double CrossReverseGenerator::deadlineComingBackToRoute1() {return visit_succ_j_ >= data_.n_requests_ ? data_.end_TW_.back() : current_solution_->latest_departures_[visit_succ_j_]; }
inline double CrossReverseGenerator::deadlineComingBackToRoute2() {return visit_succ_l_ >= data_.n_requests_ ? data_.end_TW_.back() : current_solution_->latest_departures_[visit_succ_l_]; }

inline bool CrossReverseGenerator::feasibilityFromPath1Standard() {
    return max(departures_path_1_[j_][i_ + 1][k_] + data_.true_distances_[visit_j_][visit_succ_l_], data_.start_TW_[visit_succ_l_]) + data_.service_time_[visit_succ_l_] <=
            deadlineComingBackToRoute2();
}

inline bool CrossReverseGenerator::feasibilityFromPath1Reversed() {
    return max(departures_path_1_[i_ + 1][j_][k_] + data_.true_distances_[visit_succ_i_][visit_succ_l_], data_.start_TW_[visit_succ_l_]) + data_.service_time_[visit_succ_l_] <=
            deadlineComingBackToRoute2();
}

inline bool CrossReverseGenerator::feasibilityFromPath2Standard() {
    return max(departures_path_2_[l_][k_ + 1][i_] + data_.true_distances_[visit_l_][visit_succ_j_], data_.start_TW_[visit_succ_j_]) + data_.service_time_[visit_succ_j_] <=
            deadlineComingBackToRoute1();
}

inline bool CrossReverseGenerator::feasibilityFromPath2Reversed() {
    return max(departures_path_2_[k_ + 1][l_][i_] + data_.true_distances_[visit_succ_k_][visit_succ_j_], data_.start_TW_[visit_succ_j_]) + data_.service_time_[visit_succ_j_] <=
            deadlineComingBackToRoute1();
}

inline bool CrossReverseGenerator::feasibilityCapacityRoute1() {
    return routes_load_[route_1_] + paths_load_[k_ + 1][l_] - paths_load_[i_ + 1][j_] <=  data_.capacity_ * capacity_coefficient_;
}

inline bool CrossReverseGenerator::feasibilityCapacityRoute2() {
    return routes_load_[route_2_] + paths_load_[i_ + 1][j_] - paths_load_[k_ + 1][l_] <= data_.capacity_ * capacity_coefficient_;
}


/** ------------------------------------------------------------------------------------------------ */


void CrossReverseGenerator::foundMove(double candidate_delta, CrossReverseMove &move, bool reverse_path_1,
                                      bool reverse_path_2) {
    best_delta_ = candidate_delta;
    move.setMoveVariables(i_, j_, k_, l_, route_1_, route_2_, reverse_path_1, reverse_path_2);
    move.delta_distance_ = delta_distance_;
    move.delta_reduced_cost_ = delta_reduced_cost_;
    move_found_ = true;
}


/** ------------------------------------------------------------------------------------------------ */

void CrossReverseGenerator::updateMove(CrossReverseMove &move) {
    // set the deltas
    if (capacity_coefficient_ == 1)
        move.delta_capacity_ = 0;
    else if (move.j_ > move.i_ and move.l_ > move.k_)
        move.delta_capacity_ = max(routes_load_[move.route_1_], data_.capacity_) - max(routes_load_[move.route_1_] + paths_load_[move.k_ + 1][move.l_] - paths_load_[move.i_ + 1][move.j_], data_.capacity_) + max(routes_load_[move.route_2_], data_.capacity_) - max(routes_load_[move.route_2_] - paths_load_[move.k_ + 1][move.l_] + paths_load_[move.i_ + 1][move.j_], data_.capacity_);
    else if (move.j_ > move.i_)
        move.delta_capacity_ = max(routes_load_[move.route_1_], data_.capacity_) - max(routes_load_[move.route_1_] - paths_load_[move.i_ + 1][move.j_], data_.capacity_) + max(routes_load_[move.route_2_], data_.capacity_) - max(routes_load_[move.route_2_] + paths_load_[move.i_ + 1][move.j_], data_.capacity_);
    else
        move.delta_capacity_ = max(routes_load_[move.route_1_], data_.capacity_) - max(routes_load_[move.route_1_] + paths_load_[move.k_ + 1][move.l_], data_.capacity_) + max(routes_load_[move.route_2_], data_.capacity_) - max(routes_load_[move.route_2_] - paths_load_[move.k_ + 1][move.l_] , data_.capacity_);

//    move.route_removed_ = (i_ == current_solution_->start_positions_[route_1_] and j_ == current_solution_->start_positions_[route_1_ + 1] - 1 and k_ == l_) or
//                           (k_ == current_solution_->start_positions_[route_2_] and l_ == current_solution_->start_positions_[route_2_ + 1] - 1 and i_ == j_);
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

    visit_i_ = current_solution_->tour_[i_];
    visit_j_ = current_solution_->tour_[j_];
    visit_k_ = current_solution_->tour_[k_];
    visit_l_ = current_solution_->tour_[l_];
    visit_succ_i_ = current_solution_->tour_[i_+1];
    visit_succ_j_ = current_solution_->tour_[j_+1];
    visit_succ_k_ = current_solution_->tour_[k_+1];
    visit_succ_l_ = current_solution_->tour_[l_+1];

}


double CrossReverseGenerator::computeDelta(bool standard_path_1, bool standard_path_2) {
    // COMPUTE DELTA IN REDUCED COST
    double new_red_cost_route_1;
    double new_red_cost_route_2;

    if (i_ == j_) { // if we are only removing from route 2 ==> route_2 is feasible (only check route1/path 2)
        new_red_cost_route_1 = current_solution_->routes_reduced_costs_[route_1_] - data_.true_distances_[visit_i_][visit_succ_i_] + path_cumulative_cost[k_+1][l_];
        new_red_cost_route_2 = current_solution_->routes_reduced_costs_[route_2_] + data_.true_distances_[visit_k_][visit_succ_l_] -data_.true_distances_[visit_k_][visit_succ_k_] - data_.true_distances_[visit_l_][visit_succ_l_] - path_cumulative_cost[k_+1][l_];
        if (standard_path_2)
            new_red_cost_route_1 += data_.true_distances_[visit_i_][visit_succ_k_] + data_.true_distances_[visit_l_][visit_succ_j_];
        else
            new_red_cost_route_1 += data_.true_distances_[visit_i_][visit_l_] + data_.true_distances_[visit_succ_k_][visit_succ_j_];
    }

    else if (k_ == l_) { // if we are only removing from route 1 ==> route_1 is feasible (only check route2/path 1)
        new_red_cost_route_1 = current_solution_->routes_reduced_costs_[route_1_] + data_.true_distances_[visit_i_][visit_succ_i_] - data_.true_distances_[visit_j_][visit_succ_j_] - path_cumulative_cost[i_+1][j_];
        new_red_cost_route_2 = current_solution_->routes_reduced_costs_[route_2_] - data_.true_distances_[visit_k_][visit_succ_k_] + path_cumulative_cost[i_+1][j_];
        if (standard_path_1)
            new_red_cost_route_2 += data_.true_distances_[visit_k_][visit_succ_i_] + data_.true_distances_[visit_j_][visit_succ_l_];
        else
            new_red_cost_route_1 += data_.true_distances_[visit_k_][visit_j_] + data_.true_distances_[visit_succ_i_][visit_succ_l_];
    }

    else { // if we are swapping paths
        new_red_cost_route_1 = current_solution_->routes_reduced_costs_[route_1_] - data_.true_distances_[visit_i_][visit_succ_i_] - data_.true_distances_[visit_j_][visit_succ_j_] - path_cumulative_cost[i_+1][j_] + path_cumulative_cost[k_+1][l_];
        new_red_cost_route_2 = current_solution_->routes_reduced_costs_[route_2_] - data_.true_distances_[visit_k_][visit_succ_k_] - data_.true_distances_[visit_l_][visit_succ_l_] + path_cumulative_cost[i_+1][j_] - path_cumulative_cost[k_+1][l_];
        // standard path 2
        if (!standard_path_2)
            new_red_cost_route_1 += data_.true_distances_[visit_i_][visit_succ_k_] + data_.true_distances_[visit_l_][visit_succ_j_];
        else
            new_red_cost_route_1 += data_.true_distances_[visit_i_][visit_l_] + data_.true_distances_[visit_succ_k_][visit_succ_j_];
        // standard path 1
        if (standard_path_1)
            new_red_cost_route_2 += data_.true_distances_[visit_k_][visit_succ_i_] + data_.true_distances_[visit_j_][visit_succ_l_];
        else
            new_red_cost_route_2 += data_.true_distances_[visit_k_][visit_j_] + data_.true_distances_[visit_succ_i_][visit_succ_l_];
        }
    delta_reduced_cost_=  min(new_red_cost_route_1,0.0) - min(current_solution_->routes_reduced_costs_[route_1_],0.0) + min(new_red_cost_route_2,0.0) - min(current_solution_->routes_reduced_costs_[route_2_],0.0);

    // COMPUTE DELTA IN DISTANCE
    double base_delta, intermidiate_delta;
    if (i_ == j_) { // if we are only removing from route 2 ==> route_2 is feasible (only check route1/path 2)
        base_delta = data_.true_distances_[visit_k_][visit_succ_l_] - data_.true_distances_[visit_i_][visit_succ_i_] -
                      data_.true_distances_[visit_k_][visit_succ_k_] - data_.true_distances_[visit_l_][visit_succ_l_];
        if (standard_path_2)
            delta_distance_ = base_delta + data_.true_distances_[visit_i_][visit_succ_k_] + data_.true_distances_[visit_l_][visit_succ_j_];
        else
            delta_distance_ = base_delta + data_.true_distances_[visit_i_][visit_l_] + data_.true_distances_[visit_succ_k_][visit_succ_j_];
    }
    else if (k_ == l_) { // if we are only removing from route 1 ==> route_1 is feasible (only check route2/path 1)
        base_delta = data_.true_distances_[visit_i_][visit_succ_j_] - data_.true_distances_[visit_i_][visit_succ_i_] -
                      data_.true_distances_[visit_j_][visit_succ_j_] - data_.true_distances_[visit_k_][visit_succ_k_];
        // standard path 1
        if (standard_path_1)
            delta_distance_ = base_delta + data_.true_distances_[visit_k_][visit_succ_i_] + data_.true_distances_[visit_j_][visit_succ_l_];
        else
            delta_distance_ = base_delta + data_.true_distances_[visit_k_][visit_j_] + data_.true_distances_[visit_succ_i_][visit_succ_l_];
    }
    else {
        base_delta = -data_.true_distances_[visit_i_][visit_succ_i_] - data_.true_distances_[visit_j_][visit_succ_j_] -
                      data_.true_distances_[visit_k_][visit_succ_k_] - data_.true_distances_[visit_l_][visit_succ_l_];
        // standard path 2
        if (standard_path_2) {
            intermidiate_delta = base_delta + data_.true_distances_[visit_i_][visit_succ_k_] + data_.true_distances_[visit_l_][visit_succ_j_];
            // standard path 1
            if (standard_path_1)
                delta_distance_ = intermidiate_delta + data_.true_distances_[visit_k_][visit_succ_i_] + data_.true_distances_[visit_j_][visit_succ_l_];
            else
                delta_distance_ = intermidiate_delta + data_.true_distances_[visit_k_][visit_j_] + data_.true_distances_[visit_succ_i_][visit_succ_l_];
        }
        else {
            intermidiate_delta = base_delta + data_.true_distances_[visit_i_][visit_l_] + data_.true_distances_[visit_succ_k_][visit_succ_j_];
            // standard path 1
            if (standard_path_1)
                delta_distance_ = intermidiate_delta + data_.true_distances_[visit_k_][visit_succ_i_] + data_.true_distances_[visit_j_][visit_succ_l_];
            else
                delta_distance_ = intermidiate_delta + data_.true_distances_[visit_k_][visit_j_] + data_.true_distances_[visit_succ_i_][visit_succ_l_];
        }
    }

    delta_ = (reduced_cost_option_) ? delta_reduced_cost_ : delta_distance_;
}
