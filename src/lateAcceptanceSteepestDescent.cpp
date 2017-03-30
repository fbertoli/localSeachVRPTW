//
// Created by Bertoli, Francesco (Data61, Canberra City) on 17/02/17.
//

#include "lateAcceptanceSteepestDescent.h"
#include <limits>
#include <cassert>
#include <ctime>
#include <cmath>
#include "crossReverseMove.h"
#include <iomanip>
#include <cstring>
#include <numeric>
#include "initializerInsertion.h"

LateAcceptanceSteepestDescent::LateAcceptanceSteepestDescent(Data &data, Initializer *initializer, vector<Cost*> &cost_components_solution, vector<Cost*> &cost_components_route, vector<Move*> &moves,  vector<Move*> &best_moves, vector<MoveGenerator*> &generators):
        data_(data),
        current_solution_(data),
        old_solution_(data),
        initial_solution_(data),
        best_solution_(data),
        initializer_(initializer),
        cost_components_solution_(cost_components_solution),
        cost_components_route_(cost_components_route),
        moves_(moves),
        best_moves_(best_moves),
        generators_(generators),
        max_iterations_("max-it", "maximum number of iterations", 50),
        max_idle_iterations_("max-idle", "maximum number of idle iterations", 1),
        runs("run", "number of time the metaheuristic is run", 3),
        idle_iteration_(0),
        iteration_(0),
        length_memory_("memory", "lenght of memomry for LateAcceptanceSteepestDescent",1),
        run_(0),
        dual_vars_complete_history_(data_.n_requests_, vector<double>()),
        routes_pool_(data),
        cost_history_()
{
    best_solution_cost_ = 10000000;
    clusterisation_.resize(data_.n_requests_,0);
//    moves_out_file_.open (data_.name_+ "-moves.txt");
    cost_history_.clear();
    moves_path_lenght_.resize(10,0);
};


/** ------------------------------------------------------------------------------------------------ */


LateAcceptanceSteepestDescent::~LateAcceptanceSteepestDescent() {
    // close moves file
//    moves_out_file_.close();
}

/** ------------------------------------------------------------------------------------------------ */

void LateAcceptanceSteepestDescent::reset() {
    iteration_ = 0;
    idle_iteration_ = 0;
}

/** ------------------------------------------------------------------------------------------------ */

bool LateAcceptanceSteepestDescent::stoppingCriterion() {
    if (iteration_ == max_iterations_)
        return true;
    if (idle_iteration_== max_idle_iterations_)
        return true;
    return false;
}


/** ------------------------------------------------------------------------------------------------ */


bool LateAcceptanceSteepestDescent::acceptCriterion() {
    return cost_memory_.back() + best_delta_ < cost_memory_[0];
}


/** ------------------------------------------------------------------------------------------------ */

double LateAcceptanceSteepestDescent::computeDelta(Move &move) {
    double delta(0);
    for (auto & el : cost_components_solution_)
        delta += el -> computeDelta(current_solution_, move);
    return delta;
}


/** ------------------------------------------------------------------------------------------------ */

void LateAcceptanceSteepestDescent::choseNeighborhood() {
    move_ = moves_[0];
    best_move_ = best_moves_[0];
    generator_ = generators_[0];
    generator_->setSolutionPntr(&current_solution_);
}

/** ------------------------------------------------------------------------------------------------ */

void LateAcceptanceSteepestDescent::run()
{
    // increase number of runs
    ++run_;

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
    cost_history_.push_back(current_solution_.cost_);
    initial_solution_ = current_solution_;


    // MEMORY OF COST (for acceptance)
    cost_memory_.clear();
    cost_memory_.push_back(current_solution_.cost_);
    best_solution_cost_ = current_solution_.cost_;


    while (!stoppingCriterion()) {

        // choose the neighborhood
        choseNeighborhood();

        // explore it
        generator_->first();
        best_delta_ = computeDelta(*best_move_);

        while (generator_->hasNext() && generator_ -> next()) {
            double delta(this -> computeDelta(*move_));
            if (delta < best_delta_) {
                best_delta_ = delta;
                generator_->updateBestMove();
//                break; // if you want hillClimbing
            }
        }

        ++iteration_;

        // accept/reject move
        if (acceptCriterion()) {
            // record the new routes to add to the models
            routes_indexes_.clear();
            best_move_->saveModifiedRoutesIndexes(current_solution_, routes_indexes_ );

            // perform move
            best_move_->performMove(current_solution_);
            assert(current_solution_.checkFeasibilitySolution() && "Solution is not feasible after move is performed");

            // update cost
            current_solution_.cost_ = cost_memory_.back() + best_delta_;
//            assert(abs(current_solution_.cost_ - cost_memory_.back() - best_delta_) < 0.01 && "computeCost() method in solution and sum with delta are not equal.");

            // !!(can be improved by delegating this to moves, so to recompute the cost of the routes which were modified and manage route deletion)
//            current_solution_.computeRoutesCost(cost_components_route_);

            // update solution_costs memory
            if (cost_memory_.size() >= length_memory_)
                cost_memory_.erase(cost_memory_.cbegin(), cost_memory_.cbegin() + 1);
            cost_memory_.push_back(current_solution_.cost_ );

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
//            writeMoveInfo();
//            old_solution_ = current_solution_;

        }
        else
        {
            ++idle_iteration_;
        }
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
    best_solution_.write("best-solution.txt", cost_components_solution_);


}





/** ------------------------------------------------------------------------------------------------ */

void LateAcceptanceSteepestDescent::printOuput() {
    cout << "Time                 = " << time_ << endl;
    cout << "Initial Sol Cost     = " << initial_solution_.cost_ << endl;
    cout << "Initial Sol Routes   = " << initial_solution_.n_routes_ << endl;
    cout << "Best Solution Cost   = " << best_solution_cost_ << endl;
    cout << "Best Solution Routes = " << best_solution_.n_routes_ << endl;
    cout << "Iterations           = " << iteration_ << endl;
    cout << "Idle Iterations      = " << idle_iteration_ << endl;
    cout << "# solutions accepted = " << cost_history_.size() << endl;
    cout << endl;
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

void LateAcceptanceSteepestDescent::recordMoveInfo() {
    CrossReverseMove &move = static_cast<CrossReverseMove&>(*best_move_);
    if (move.route_removed_)
        ++moves_with_route_deletion_;
    ++moves_path_lenght_[move.j_ - move.i_];
    ++moves_path_lenght_[move.k_ - move.l_];
}

/** ------------------------------------------------------------------------------------------------ */


void LateAcceptanceSteepestDescent::writeMoveInfo() {
    CrossReverseMove &move = static_cast<CrossReverseMove&>(*best_move_);

    // Print Moves Information
    moves_out_file_ << "Iteration " << iteration_ << endl;

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

    // Delta in cost
    moves_out_file_ << "\tDelta cost         : " << current_solution_.cost_ - old_solution_.cost_ << endl;

    moves_out_file_ << endl;


}