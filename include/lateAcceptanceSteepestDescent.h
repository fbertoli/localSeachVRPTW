#pragma once


#include "solution.h"
#include "data.h"
#include "options.h"
#include "initializer.h"
#include "move.h"
#include "cost.h"
#include "moveGenerator.h"
#include "gurobi_c++.h"
#include "routes.h"
#include <fstream>
#include "costReduced.h"

using namespace std;

/** structures needed for sorting */
struct CustomerDualComparison {
    vector<double> &duals;
    bool operator() (int i,int j) { return duals[i] > duals[j];}
};

struct RouteDualComparison {
    vector<double> &duals;
    bool operator() (int i,int j) { return duals[i] > duals[j];}
};




class LateAcceptanceSteepestDescent
{
    /** CONSTRUCTORS */
public:
    LateAcceptanceSteepestDescent(Data &data, Initializer *initializer, vector<Cost*> &cost_components_solution, vector<Cost*> &cost_components_route, vector<Move*> &moves,  vector<Move*> &best_moves, vector<MoveGenerator*> &generator);

    ~LateAcceptanceSteepestDescent();

    /** METHODS */
protected:
    /** wether to stop execution or not */
    bool stoppingCriterion();

    /** wether to accept move or not */
    bool acceptCriterion();

    /** compute delta cost of the move */
    double computeDelta(Move &move, double &fake_delta, double &true_delta);

    /** chose what type of generator to use in the iteration */
    void choseNeighborhood();


//    /** record the best routes of new solution accepted */
//    void collectRoutes();



public:
    /** run the methaheuristic */
    void run();

    /** run the basic search */
    void basicSearch();

    void printOuput();

    /** write move info */
    void writeMoveInfo();
    void writeClusterInfo();

    /** to inspect clusterisation */
    void createClusterisation();
    void updateClusterisation();

    void reset();




    /** VARIABLES */
public:
    /** clusterisation resulting from search */
    vector<int> clusterisation_;

    /** constainer for the routes found so far */
    RouteContainer routes_pool_;

    /** the indexes of the new routes in the solution after the move is performed */
    vector<int> routes_indexes_;

    /** (fake/true) cost of last length_memory solution */
    vector<double> cost_solutions_;

    /** fake/true costs of last solutions */
    double last_fake_cost_;
    double last_true_cost_;

    /** how many solution to record */
    Option<int> length_memory_;

    /** data of the instance */
    Data &data_;

    /** the heuristic to initialize the solution */
    Initializer *initializer_;

    /** the current solution */
    Solution current_solution_;

    /** the best solution */
    Solution best_solution_;

    /** how many time I ran the algorithm */
    int run_;

    /** the current iteration number */
    int iteration_;

    /** the number of dile iteration (in a row)*/
    int idle_iteration_;

    /** maximum number of iterations*/
    Option<int> max_iterations_;

    /** max number of idle iterations*/
    Option<int> max_idle_iterations_;

    /** number of runs of the main search */
    Option<int> runs_;


    /** the cost objects determining the total cost of solutions. the true ones and the ones used to guide the solution*/
    vector<Cost*> &true_cost_components_solution_;
    vector<Cost*> fake_cost_components_solution_;

    /** the costs determining the total cost of a route */
    vector<Cost*> &cost_components_route_;

    /** object to compute reduced cost of routes/solutions */
    CostReduced reduced_cost_;

    /** the possible Move objects involved in the exploration pof the neighbourhoods */
    vector<Move*> &moves_;
    vector<Move*> &best_moves_;

    /** the current chosen Move object and best_move*/
    Move *move_;
    Move *best_move_;

    /** the generators to explore the neighbourhoods */
    vector<MoveGenerator*> &generators_;

    /**  the current chosen generator*/
    MoveGenerator *generator_;

    /** the delta in the true cost at the end of the generator search */
    double true_delta_;

    /** the delta in the fake cost at the end of the generator search*/
    double fake_delta_;

    /** the best delta (fake or true depending on the run_) found so far in the current iteration */
    double best_delta_;

    /** save best cost of solution found so far */
    double best_solution_cost_;

    /** time spent in the initialization + search */
    double time_;

    /** if the routes were destroyed by the move */
    bool route_1_still_exists_;
    bool route_2_still_exists_;


    /** ------------------------------------------------------------------
     *                          HISTORY VARS
     *  ------------------------------------------------------------------*/
    /** cost hsitory */
    vector<double> true_cost_history_;

    /** the solution before the move is performed (only used for inscpection) */
    Solution old_solution_;

    /** solution history */
    vector<Solution> solution_history_;

    /** moves history */
    vector<CrossReverseMove> move_history_;

    /** indicate position of the bet solution in solution_history_ */
    int best_solution_index_;

    /** the complete evolution of duals */
    vector<vector<double>> dual_vars_complete_history_;

    /** how many iterations we had in each search */
    vector<int> run_iterations_;

    /** evolution of the linear model objective */
    vector<double> linear_objective_;

    /** print stuff */
    ofstream moves_out_file_;
    ofstream clusterisation_out_file_;

    /** time */
    clock_t begin_;

};

