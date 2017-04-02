//
// Created by Bertoli, Francesco (Data61, Canberra City) on 17/02/17.
//


#include <limits>
#include <cassert>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <cstring>
#include <numeric>
#include <stdlib.h>
#include "initializerInsertion.h"
#include "crossReverseMove.h"
#include "tabuSearch.h"

TabuSearch::TabuSearch(Data &data, Initializer *initializer, vector<Cost*> &cost_components_solution, vector<Cost*> &cost_components_route, vector<Move*> &moves,  vector<Move*> &best_moves, vector<MoveGenerator*> &generators):
        Metaheuristic(data, initializer, cost_components_solution, cost_components_route, moves,  best_moves, generators),
        current_solution_(data),
        old_solution_(data),
        initial_solution_(data),
        max_iterations_("it", "maximum number of iterations", 50),
        min_tabu_iterations_("mintabu", "minimum number of taboo iterations", 20),
        max_tabu_iterations_("maxtabu", "maximum number of taboo iterations", 40),
        routes_pool_(data),
        cost_history_(),
        forbidden_list_()
{
    best_solution_cost_ = 10000000;
    clusterisation_.resize(data_.n_requests_,0);
    forbidden_arcs_.resize(data_.n_requests_ + data_.max_vehicles_, vector<int> (data_.n_requests_ + data_.max_vehicles_, 0));
    forbidden_list_.reserve(4*(max_tabu_iterations_+1));
    removed_arcs_.clear();
    moves_out_file_.open (data_.name_+ "-moves.txt");

    // set pointer of generator to current_solution
    for (auto &el : generators_)
        el -> current_solution_ = &current_solution_;

    // to debug inspection variables
    cost_history_.clear();
    moves_path_lenght_.resize(10,0);
    moves_with_route_deletion_ = 0;
    moves_with_route_deletion_ = 0;
    moves_non_improving_ = 0;
    moves_tabu_ = 0;
};


/** ------------------------------------------------------------------------------------------------ */


TabuSearch::~TabuSearch() {
    // close moves file
    moves_out_file_.close();
}

/** ------------------------------------------------------------------------------------------------ */

void TabuSearch::reset() {
    iteration_ = 0;
    for (auto &el : forbidden_list_)
        forbidden_arcs_[el.first][el.second] = 0;
}

/** ------------------------------------------------------------------------------------------------ */

bool TabuSearch::stoppingCriterion() {
    return iteration_ == max_iterations_;
}



/** ------------------------------------------------------------------------------------------------ */

double TabuSearch::computeDelta(Move &move) {
    double delta(0);
    for (auto & el : cost_components_solution_)
        delta += el -> computeDelta(current_solution_, move);
    return delta;
}


/** ------------------------------------------------------------------------------------------------ */

void TabuSearch::choseNeighborhood() {
    move_ = moves_[0];
    best_move_ = best_moves_[0];
    generator_ = generators_[0];
}


/** ------------------------------------------------------------------------------------------------ */

bool TabuSearch::isMoveTabu() {
    for (auto &el : new_arcs_)
        if (forbidden_arcs_[el.first][el.second] > 0)
            return true;
    return false;
}


/** ------------------------------------------------------------------------------------------------ */

void TabuSearch::updateForbiddenArcs()
{
    int tabu_duration((rand() % (int(max_tabu_iterations_) - min_tabu_iterations_)) + min_tabu_iterations_);

    for (auto &arc : removed_arcs_) {
        if (forbidden_arcs_[arc.first][arc.second] > 0) {
            forbidden_arcs_[arc.first][arc.second] = tabu_duration + 1;
        }
        else {
            forbidden_arcs_[arc.first][arc.second] = tabu_duration;
            forbidden_list_.push_back(arc);
        }
    }


    // update tabu status of arcs in forbidden_list_
    for (vector<pair<int,int>>::iterator iter = forbidden_list_.begin(); iter != forbidden_list_.end();) {
        if (forbidden_arcs_[(*iter).first][(*iter).second] > 0) {
            --forbidden_arcs_[(*iter).first][(*iter).second];
            ++iter;
        }
        else
            iter = forbidden_list_.erase(iter);
    }


}

/** ------------------------------------------------------------------------------------------------ */

void TabuSearch::run()
{
    // start measuring time
    clock_t begin = clock();


    // SET INITIALIZER PARAMETERS
    initializer_->setRouteContainerPtr(&routes_pool_);
    initializer_->setClusterisationPtr(&clusterisation_);
    initializer_->setRouteCostComponents(&cost_components_route_);
    current_solution_ = initializer_->initializeSolution();
    current_solution_.computeCost(cost_components_solution_);
    best_solution_ = current_solution_;
    

    // ASSER AND WRITE (only debug)
    assert(current_solution_.checkFeasibilitySolution() && "Initial solution is not feasible");
    current_solution_.write("output-initial-run.txt", cost_components_solution_);

    // HISTORY VARAIBLES
    old_solution_ = current_solution_;
    cost_history_.push_back(current_solution_.cost_);
    initial_solution_ = current_solution_;


    // MEMORY OF COST (for acceptance)
    best_solution_cost_ = current_solution_.cost_;


    while (!stoppingCriterion()) {

        // choose the neighborhood
        choseNeighborhood();

        // to debug
//      is_best_move_tabu_ = false;

        // explore it
        generator_->first();
        best_delta_ = computeDelta(*best_move_);

        while (generator_->hasNext() && generator_ -> next()) {
            double delta(this -> computeDelta(*move_));
            move_ -> identifyAddedArcs(current_solution_, new_arcs_);
            // if move is tabu but is best accept it, otherwise only check if it's best move found
            if (isMoveTabu()) {
                if (delta < best_solution_cost_ - current_solution_.cost_) {
                    best_delta_ = delta;
                    generator_->updateBestMove();
                    is_best_move_tabu_ = true;
                }
            }
            else if (delta < best_delta_) {
                best_delta_ = delta;
                generator_->updateBestMove();
                is_best_move_tabu_ = false;
            }
        }

        // do something if there was no move found (otherwise we execute the old move again
//        cout << "iteration " <<iteration_ << " moves_found " << moves_found << " best_delta " << best_delta_ << endl;

        // increase iteration
        ++iteration_;

        // record the new routes to add to the SPP model
//        routes_indexes_.clear();
//        best_move_->saveModifiedRoutesIndexes(current_solution_, routes_indexes_ );

        // update cost
        current_solution_.cost_ += best_delta_;

        // perform move and aupdate tabu arcs
        best_move_ -> identifyRemovedArcs(current_solution_, removed_arcs_);
        writeArcsInfo();
        updateForbiddenArcs();
        best_move_ -> performMove(current_solution_);
        assert(current_solution_.checkFeasibilitySolution() && "Solution is not feasible after move is performed");


        // !!(can be improved by delegating this to moves, so to recompute the cost of the routes which were modified and manage route deletion)
//            current_solution_.computeRoutesCost(cost_components_route_);


        // history variables
        cost_history_.push_back(current_solution_.cost_);

        // check if it's new best
        if (current_solution_.cost_ < best_solution_cost_) {
            best_solution_cost_ = current_solution_.cost_;
            best_solution_ = current_solution_;
        }

        // add route to the pool
//            routes_pool_.extractRoutes(current_solution_, routes_indexes_);

        // write move info and assign old solution (only for inspection)
        recordMoveInfo();
        writeMoveInfo();
        old_solution_ = current_solution_;

    }

    // end measuring time
    clock_t end = clock();
    time_ = double(end - begin) / CLOCKS_PER_SEC;


    // solve SPP integer model
//    routes_pool_.createIntegerModel();
//    routes_pool_.updateIntegerModel();
//    routes_pool_.optimizeIntegerModel(100);
//    routes_pool_.integer_model_.write("integer-model.lp");
//    routes_pool_.updateIntegerSolution();
//    routes_pool_.solution_.write("output-best-solution-mip.txt", cost_components_solution_);
//    ofstream out_file;
//    out_file.open("integer-model-vars.txt");
//    for (int r = 0; r < routes_pool_.routes_.size(); ++r) {
//        out_file << "route-" << r << " : dep  ->  ";
//        for (auto &el : routes_pool_.routes_[r].stops_)
//            out_file << el << "  ->  ";
//        out_file << endl;
//    }
//    out_file.close();

    // write best solution
    best_solution_.write("output-best-solution.txt", cost_components_solution_);


}





/** ------------------------------------------------------------------------------------------------ */

void TabuSearch::printOuput() {
    cout << "Time                 = " << time_ << endl;
    cout << "Iterations           = " << iteration_ << endl;
    cout << "Initial Sol Cost     = " << initial_solution_.cost_ << endl;
    cout << "Initial Sol Routes   = " << initial_solution_.n_routes_ << endl;
    cout << "Best Solution Cost   = " << best_solution_cost_ << endl;
    cout << "Best Solution Routes = " << best_solution_.n_routes_ << endl;
    cout << endl;
    cout << "Moves non improving  = " << moves_non_improving_ << " (" << 100*double(moves_non_improving_)/iteration_ << " %)" << endl;
    cout << "Moves tabu           = " << moves_tabu_ << " (" << 100*double(moves_tabu_)/iteration_ << " %)" << endl;
    cout << "Moves with route del = " << moves_with_route_deletion_ << " (" << 100*double(moves_with_route_deletion_)/iteration_ << " %)" << endl;
    cout << "Moves path lenght    = ";

    for (int i = 0; i < moves_path_lenght_.size(); ++i)
        cout << i << " = " << moves_path_lenght_[i] << "  (" << 100*double(moves_path_lenght_[i])/iteration_ << "%),  ";
    cout << endl;
    cout << endl;
    cout << "Cost Evolution" << endl;
    for (auto const & el : cost_history_)
        cout << el << ", ";
    cout << endl;
    cout << endl;
}


/** ------------------------------------------------------------------------------------------------ */


// ////////////////////////////////////////////////
// MOVE INFO
// ////////////////////////////////////////////////

void TabuSearch::recordMoveInfo() {
    CrossReverseMove &move = static_cast<CrossReverseMove&>(*best_move_);
    if (move.route_removed_)
        ++moves_with_route_deletion_;
    ++moves_path_lenght_[move.j_ - move.i_];
    ++moves_path_lenght_[move.k_ - move.l_];

    if (best_delta_ >= 0)
        ++moves_non_improving_;

    if (is_best_move_tabu_)
        ++moves_tabu_;

}

/** ------------------------------------------------------------------------------------------------ */

void TabuSearch::writeArcsInfo() {
    CrossReverseMove &move = static_cast<CrossReverseMove&>(*best_move_);

    // Print Moves Information
    moves_out_file_ << "Iteration " << iteration_ << endl;

    // print tabu status of arcs to be added
    for (auto &el :removed_arcs_) {
        moves_out_file_ << "( " << el.first << ", " << el.second << ") ";
        moves_out_file_ << "  tabu " << forbidden_arcs_[el.first][el.second] << endl;
    }

}

void TabuSearch::writeMoveInfo() {
    CrossReverseMove &move = static_cast<CrossReverseMove&>(*best_move_);

    // Print Moves Information
//    moves_out_file_ << "Iteration " << iteration_ << endl;



    // Path 1
    moves_out_file_ << "\tRoute " << move.route_1_ << " \tPath 1 : " << old_solution_.tour_[move.i_];
    if (move.i_ < move.j_) {
        moves_out_file_ << " - [ ";
        for (int pos = move.i_ + 1; pos < move.j_; ++pos)
            moves_out_file_ << old_solution_.tour_[pos] << " - ";
        moves_out_file_ << old_solution_.tour_[move.j_];
        moves_out_file_ << " ] ";
    }
    moves_out_file_ << " - " << old_solution_.tour_[move.j_ + 1];
    if (move.i_ < move.j_ and move.reverse_path_1_)
        moves_out_file_ <<  " reversed" << endl;
    moves_out_file_ <<  endl;

    // Path 2
    moves_out_file_ << "\tRoute " << move.route_2_ << " \tPath 2 : " << old_solution_.tour_[move.k_];
    if (move.k_ < move.l_) {
        moves_out_file_ << " - [ ";
        for (int pos = move.k_ + 1; pos < move.l_; ++pos)
            moves_out_file_ << old_solution_.tour_[pos] << " - ";
        moves_out_file_ << old_solution_.tour_[move.l_];
        moves_out_file_ << " ] ";
    }
    moves_out_file_ << " - " << old_solution_.tour_[move.l_ + 1];
    if (move.k_ < move.l_ and move.reverse_path_2_)
        moves_out_file_ <<  " reversed" << endl;
    moves_out_file_ <<  endl;


    // print tabu status of arcs to be added
    for (auto &el :removed_arcs_) {
        moves_out_file_ << "( " << el.first << ", " << el.second << ") ";
        moves_out_file_ << "  tabu " << forbidden_arcs_[el.first][el.second] << endl;
    }


    // Delta in cost
    moves_out_file_ << "\tDelta cost         : " << current_solution_.cost_ - old_solution_.cost_ << endl;

    moves_out_file_ << endl;
    moves_out_file_ << endl;


}