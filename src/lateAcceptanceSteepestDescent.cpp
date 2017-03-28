//
// Created by Bertoli, Francesco (Data61, Canberra City) on 17/02/17.
//

#include "lateAcceptanceSteepestDescent.h"
#include <limits>
#include <cassert>
#include <ctime>
#include <cmath>
#include "crossReverseMove.h"
#include "crossReverseGenerator.h"
#include <iomanip>
#include <cstring>
#include <numeric>
#include "initializerInsertion.h"

LateAcceptanceSteepestDescent::LateAcceptanceSteepestDescent(Data &data, Initializer *initializer, vector<Cost*> &cost_components_solution, vector<Cost*> &cost_components_route, vector<Move*> &moves,  vector<Move*> &best_moves, vector<MoveGenerator*> &generators):
        data_(data),
        current_solution_(data),
        old_solution_(data),
        best_solution_(data),
        initializer_(initializer),
        true_cost_components_solution_(cost_components_solution),
        cost_components_route_(cost_components_route),
        moves_(moves),
        best_moves_(best_moves),
        generators_(generators),
        max_iterations_("max-it", "maximum number of iterations", 50),
        max_idle_iterations_("max-idle", "maximum number of idle iterations", 1),
        runs_("run", "number of time the metaheuristic is run", 3),
        idle_iteration_(0),
        iteration_(0),
        length_memory_("memory", "lenght of memomry for LateAcceptanceSteepestDescent",1),
        run_(0),
        dual_vars_complete_history_(data_.n_requests_, vector<double>()),
        run_iterations_(),
        linear_objective_(),
        routes_pool_(data),
        true_cost_history_(),
        reduced_cost_(data),
        fake_cost_components_solution_{&reduced_cost_}
        {
            best_solution_index_ = 0;
            clusterisation_.resize(data_.n_requests_,0);
            moves_out_file_.open(data_.name_+ "-moves.txt");
            clusterisation_out_file_.open(data_.name_+ "-clusterisation.txt");
            true_cost_history_.clear();
        };


/** ------------------------------------------------------------------------------------------------ */


LateAcceptanceSteepestDescent::~LateAcceptanceSteepestDescent() {
    // close files
    moves_out_file_.close();
    clusterisation_out_file_.close();
}

/** ------------------------------------------------------------------------------------------------ */

void LateAcceptanceSteepestDescent::reset() {
    iteration_ = 0;
    idle_iteration_ = 0;
    true_cost_history_.clear();
    linear_objective_.clear();
    cost_solutions_.clear();
    cost_solutions_.push_back((run_ > 0) ? current_solution_.fake_cost_ : current_solution_.true_cost_);
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
    return cost_solutions_.back() + best_delta_ < cost_solutions_[0];
}


/** ------------------------------------------------------------------------------------------------ */

double LateAcceptanceSteepestDescent::computeDelta(Move &move, double &fake_delta, double &true_delta) {
    for (auto &el : true_cost_components_solution_)
        true_delta += el->computeDelta(current_solution_, move);

    for (auto &el : fake_cost_components_solution_)
        fake_delta += el->computeDelta(current_solution_, move);

    return (run_>0) ? fake_delta : true_delta;
}


/** ------------------------------------------------------------------------------------------------ */

void LateAcceptanceSteepestDescent::choseNeighborhood() {
    move_ = moves_[0];
    best_move_ = best_moves_[0];
    generator_ = generators_[0];
    generator_->setSolutionPntr(&current_solution_);
}

/** ------------------------------------------------------------------------------------------------ */


void LateAcceptanceSteepestDescent::basicSearch() {

    while (!stoppingCriterion()) {
        double fake_delta(0);
        double true_delta(0);
        double delta(0);

        // choose the neighborhood
        choseNeighborhood();

        // explore it
        generator_->first(*best_move_);
        best_delta_ = computeDelta(*best_move_, fake_delta, true_delta);
        true_delta_ = true_delta;
        fake_delta_ = fake_delta;

        while (generator_->hasNext() && generator_ -> next(*move_)) {
            fake_delta = 0;
            true_delta = 0;
            delta = computeDelta(*move_, fake_delta, true_delta); // delta is either equal to true_delta or fake_delta (depending on the run_)
            if (delta < best_delta_) {
                best_move_ = move_;
                best_delta_ = delta;        // this is how the best move is selected
                true_delta_ = true_delta;   // this is how the true cost is updated
                fake_delta_ = fake_delta;   // this is how the fake cost is updated
                // break;                   // if you want hillClimbing
            }
        }

        ++iteration_;

        // accept/reject move
        if (acceptCriterion()) {
            // Save the indexes of routes in new solution
            routes_indexes_.clear();
            best_move_->saveModifiedRoutesIndexes(current_solution_, routes_indexes_ );

            // perform move
            best_move_->performMove(current_solution_);
            assert(current_solution_.checkFeasibilitySolution() && "Solution is not feasible after move is performed");

            // update cost
            current_solution_.true_cost_ = last_true_cost_ + true_delta_;
            current_solution_.fake_cost_ = last_fake_cost_ + fake_delta_;
            current_solution_.computeRoutesCost(cost_components_route_, routes_indexes_);
            current_solution_.computeRoutesReducedCost(&routes_pool_.dual_variables_, routes_indexes_);
//            assert(abs(current_solution_.true_cost_ - last_true_cost_ - true_delta_) < 0.01 && "computeCost() method in solution and sum with delta are not equal.");

            // debug only
//            double cost(0);
//            for (auto & r: current_solution_.routes_costs_)
//                cost += r;
//            assert((abs(cost - current_solution_.true_cost_) < 0.01) && "problem with cost");

            // update solution_costs memory
            if (cost_solutions_.size() >= length_memory_)
                cost_solutions_.erase(cost_solutions_.cbegin(), cost_solutions_.cbegin() + 1);
            cost_solutions_.push_back((run_ > 0) ? current_solution_.fake_cost_ : current_solution_.true_cost_);




            // check if it's new best
            if (current_solution_.true_cost_ < best_solution_.true_cost_) {
                best_solution_ = current_solution_;
//                best_solution_index_ = solution_history_.size() - 1;
//                best_solution_cost_ = current_solution_.cost_;
            }

            // re-set idle iterations and notify generators
            idle_iteration_ = 0;
            for (auto & el : generators_)
                el->setSolutionPntr(&current_solution_);

            // add route to the pool
            routes_pool_.extractRoutes(current_solution_, routes_indexes_);


            // save dual variables
            routes_pool_.updateLinearModel();
            routes_pool_.optimizeLinearModel();
            routes_pool_.updateDualVariables();
            routes_pool_.writeDualVariables();

            // write move info and assign old solution
            writeMoveInfo();
            old_solution_ = current_solution_;
            last_fake_cost_ = current_solution_.fake_cost_;
            last_true_cost_ = current_solution_.true_cost_;

            // update Clusterisation
            updateClusterisation();

            // history variables
//            solution_history_.push_back(current_solution_);
//            move_history_.push_back(*(static_cast<CrossReverseMove*>(best_move_)));
            true_cost_history_.push_back(current_solution_.true_cost_);
            linear_objective_.push_back(routes_pool_.linear_model_.get(GRB_DoubleAttr_ObjVal));
            if (linear_objective_.size() > 1 && abs(linear_objective_.back() - linear_objective_[linear_objective_.size() - 2]) < 0.001)
                ++idle_iteration_;


        }
        else
        {
            ++idle_iteration_;
        }
    }

    // end measuring time
    clock_t end = clock();
    time_ = double(end - begin_) / CLOCKS_PER_SEC;

    // solve SPP integer model
    routes_pool_.updateIntegerModel();
    routes_pool_.optimizeIntegerModel(100);
    routes_pool_.integer_model_.write("integer-model.lp");
    routes_pool_.updateIntegerSolution();
    routes_pool_.solution_.computeRoutesCost(cost_components_route_);
    routes_pool_.solution_.computeRoutesReducedCost(&routes_pool_.dual_variables_);
    routes_pool_.solution_.write("best-solution-mip.txt", true_cost_components_solution_);
    best_solution_.write("best-solution.txt", true_cost_components_solution_);
    // print
    printOuput();
}


/** ------------------------------------------------------------------------------------------------ */


void LateAcceptanceSteepestDescent::run()
{
    // start measuring time
    begin_ = clock();


    // SET INITIALIZER PARAMETERS
    initializer_->setRouteContainerPtr(&routes_pool_);
    initializer_->setClusterisationPtr(&clusterisation_);
    initializer_->setRouteCostComponents(&cost_components_route_);
//    if (run_ >= 1) {
//        initializer_->switcher("profits");
//        initializer_->setNewCapacity(data_.capacity_);
//    }
    current_solution_ = initializer_->initializeSolution();

    // OPTIMIZE LINEAR MODEL
    routes_pool_.optimizeLinearModel();
    routes_pool_.updateDualVariables();
    routes_pool_.writeDualVariables();


    // COMPUTE SOLUTION COSTS
    current_solution_.computeTrueCost(true_cost_components_solution_);
    current_solution_.computeRoutesCost(cost_components_route_);
    current_solution_.computeRoutesReducedCost(&routes_pool_.dual_variables_);
    best_solution_ = current_solution_;


    // REFINE ROUTES
//    double cost_before(current_solution_.cost_);
//    current_solution_.tsp_time_limit_ = 20;
//    current_solution_.refineRoutesTSP();
//    current_solution_.computeCost(cost_components_solution_);
//    cout << "TSP savings initializer = " << cost_before - current_solution_.cost_ << endl;


    // ASSER AND WRITE (only debug)
    assert(current_solution_.checkFeasibilitySolution() && "Inital solution is not feasible");
    current_solution_.write("initial-run.txt", true_cost_components_solution_);


    // HISTORY VARAIBLES
//    solution_history_.clear();
//    move_history_.clear();
//    solution_history_.push_back(current_solution_);
//    best_solution_index_ = 0;
    true_cost_history_.push_back(current_solution_.true_cost_);
    old_solution_ = current_solution_;
    last_true_cost_ = current_solution_.true_cost_;


    // MEMORY OF COST (for acceptance)
    cost_solutions_.clear();
    cost_solutions_.push_back(current_solution_.true_cost_);

    // set GENERATOR POINTER
    CrossReverseGenerator *generator = static_cast<CrossReverseGenerator *>(generators_[0]);
    generator->dual_variables_ = &routes_pool_.dual_variables_;

    // FIRST RUN
    basicSearch();
    reduced_cost_.dual_variables_ = &routes_pool_.dual_variables_;

    // NOTIFY GENERATOR WE ARE USING REDUCED COST NOW
    generator->reduced_cost_option_= true;


    // INITIALIZE FAKE COST
    // routes_costs_ is update in basicSearch at every iteration
    current_solution_.computeRoutesReducedCost(reduced_cost_.dual_variables_);
    current_solution_.computeFakeCost(fake_cost_components_solution_);



    // DO OTHER RUNS
    while (run_ < runs_) {
        ++run_;
        reset();
        basicSearch();
        reduced_cost_.dual_variables_ = &routes_pool_.dual_variables_;
        routes_pool_.printEndBasicSearch();
        generator->max_length_path_ += 1;
//        if (iteration_ == 1) {
//            CrossReverseGenerator *generator = static_cast<CrossReverseGenerator *>(generators_[0]);
//            break;
//        }
    }




    // write best solution
    best_solution_.computeRoutesCost(cost_components_route_);
    best_solution_.computeRoutesReducedCost(&routes_pool_.dual_variables_);
    best_solution_.write("best-solution.txt", true_cost_components_solution_);



    // refine routes
//    best_solution_.tsp_time_limit_ = 20;
//    cost_before = best_solution_.cost_;
//    best_solution_.refineRoutesTSP();
//    best_solution_.computeCost(cost_components_solution_);
//    cout << "TSP savings initializer = " << cost_before - best_solution_.cost_ << endl;



}





/** ------------------------------------------------------------------------------------------------ */

// ////////////////////////////////////////////////
// CLUSTERISATION
// ////////////////////////////////////////////////

void LateAcceptanceSteepestDescent::createClusterisation() {

    for (int route = 0; route < current_solution_.n_routes_; ++route)
        for (int i = current_solution_.start_positions_[route] + 1;
             i < current_solution_.start_positions_[route + 1]; ++i)
            clusterisation_[current_solution_.tour_[i]] = route;
}

void LateAcceptanceSteepestDescent::updateClusterisation() {
    CrossReverseMove &move = static_cast<CrossReverseMove &>(*best_move_);
    for (int i = current_solution_.start_positions_[move.route_2_] + 1;
         i < current_solution_.start_positions_[move.route_2_ + 1]; ++i)
        clusterisation_[current_solution_.tour_[i]] = clusterisation_[current_solution_.tour_[move.j_]];
}



/** ------------------------------------------------------------------------------------------------ */


void LateAcceptanceSteepestDescent::printOuput() {
    cout << "RUN " << run_ << endl;
    try {
        cout << "MIP Solution Cost    = " << routes_pool_.integer_model_.get(GRB_DoubleAttr_ObjVal) << endl;
    } catch (GRBException e) {
        cout << "No MIP Solution found Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Error during optimization" << endl;
    }
    cout << "MIP Gap              = " << routes_pool_.integer_model_.get(GRB_DoubleAttr_MIPGap) << endl;
    cout << "Gap to Lower Bound   = " << (routes_pool_.integer_model_.get(GRB_DoubleAttr_ObjVal)- routes_pool_.linear_model_.get(GRB_DoubleAttr_ObjVal))/routes_pool_.linear_model_.get(GRB_DoubleAttr_ObjVal) << endl;
    cout << "Routes in pool       = " << routes_pool_.routes_.size() << endl;
    cout << "Best Solution Cost   = " << best_solution_.true_cost_ << endl;
    cout << "Best Solution Routes = " << best_solution_.n_routes_ << endl;
    cout << "Iterations           = " << iteration_ << endl;
    cout << "Idle Iterations      = " << idle_iteration_ << endl;
    cout << "# solutions accepted = " << true_cost_history_.size() << endl;
    cout << "Time                 = " << time_ << endl;
    cout << "Linear Obj Evolution" << endl;
    for (auto const & el : linear_objective_)
        cout << el << ", ";
    cout << endl;
    cout << endl;
}


/** ------------------------------------------------------------------------------------------------ */


// ////////////////////////////////////////////////
// MOVE INFO
// ////////////////////////////////////////////////

void LateAcceptanceSteepestDescent::writeMoveInfo() {
    CrossReverseMove &move = static_cast<CrossReverseMove&>(*best_move_);

    // Print Moves Information
    moves_out_file_ << endl << "MOVES INFORMATION RUN " << run_ << endl;
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
    moves_out_file_ << "\tDelta true cost    : " << current_solution_.true_cost_ - last_true_cost_ << endl;
    moves_out_file_ << "\tDelta fake cost    : " << current_solution_.fake_cost_ - last_fake_cost_ << endl;

    // Biggest dual
    vector<int> indexes(data_.n_requests_,0);
    iota(indexes.begin(), indexes.end(), 0);
    comparisonProfitStruct order{&routes_pool_.dual_variables_};
    sort (indexes.begin(), indexes.end(), order);
    moves_out_file_ << "\tCustomer Dual Rank : ";
    for (int i =data_.n_requests_-1; i>=0 ; --i) {
        if (routes_pool_.dual_variables_[indexes[i]] > 0)
            moves_out_file_ << indexes[i] << " ";
        else
            break;
    }

    moves_out_file_ << endl;

    // Route dual total value
    moves_out_file_ << "\tRoute Dual Rank    : ";
    indexes.resize(old_solution_.n_routes_,0);
    iota(indexes.begin(), indexes.end(), 0);
    vector<double> routes_dual_load(old_solution_.n_routes_,0);
    for (int route = 0; route < old_solution_.n_routes_; ++route)
        for (int i = old_solution_.start_positions_[route] + 1; i < old_solution_.start_positions_[route+1] -1; ++i )
            routes_dual_load[route] += routes_pool_.dual_variables_[old_solution_.tour_[i]];
    order.profits_ = &routes_dual_load;
    sort (indexes.begin(), indexes.end(), order);
    for (int i =routes_dual_load.size() - 1; i >0 ; --i)
        moves_out_file_ <<  indexes[i] << " (" << routes_dual_load[indexes[i]] << ")  ";
    moves_out_file_ << endl;



    // Duals vars changed and route they belong to
//    moves_out_file_ << "\tChanges in duals   : ";
//    for (int route = 0; route < solution.n_routes_; ++route) {
//        for (int i = solution.start_positions_[route] + 1; i < solution.start_positions_[route + 1] - 1; ++i) {
//            int req = solution.tour_[i];
//            if (dual_variables[req][it + 1] != dual_variables[req][it])
//                moves_out_file_ << req << " : " << dual_variables[req][it + 1] - dual_variables[req][it] << " (route : " << route << ")    ";
//        }
//    }

    moves_out_file_ << endl;
    moves_out_file_ << endl;


}



/** ------------------------------------------------------------------------------------------------ */


// ////////////////////////////////////////////////
// CLUSTER INFO
// ////////////////////////////////////////////////

void LateAcceptanceSteepestDescent::writeClusterInfo() {
    clusterisation_out_file_ << "CLUSTERISATION RUN " << run_<< endl;

    // from where we start checking indexes
    int start_index(0), new_start_index(0);
    // the numbed defining cluster we have already seen
    vector<int> cluster_index;
    while (true) {
        // the number defining the new cluster we are looking at
        int cluster(clusterisation_[start_index]);
        cluster_index.push_back(cluster);
        clusterisation_out_file_ << "Cluster " << cluster_index.size() -1 << " : ";
        for (int i = start_index; i<data_.n_requests_; ++i) {
            if (clusterisation_[i] == cluster)
                clusterisation_out_file_ << i << " ";

            if ( (find(cluster_index.begin(), cluster_index.end(),clusterisation_[i])==cluster_index.end())  && new_start_index == start_index )
                new_start_index = i;
        }
        clusterisation_out_file_ << endl;
        if (new_start_index == start_index)
            break;
        else
            start_index = new_start_index;
    }
    clusterisation_out_file_ << endl << endl;
}