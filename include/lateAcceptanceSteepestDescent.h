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

/** This class implements a LateAcceptanceSteepestDescent metaheuristic. Given a set of neighborhood it explores
 * them exhaustively and choose the best move.
 * The acceptance criterion is: whether if the cost is better than the cost of length_memory_ iteration before. */


using namespace std;


class LateAcceptanceSteepestDescent
{
    /** CONSTRUCTORS */
public:
    LateAcceptanceSteepestDescent(Data &data, Initializer *initializer, vector<Cost*> &cost_components_solution, vector<Cost*> &cost_components_route, vector<Move*> &moves,  vector<Move*> &best_moves, vector<MoveGenerator*> &generator);

    ~LateAcceptanceSteepestDescent();

    /** METHODS */
protected:
    /** whether to stop execution or not */
    bool stoppingCriterion();

    /** whether to accept move or not */
    bool acceptCriterion();

    /** compute delta cost of the move */
    double computeDelta(Move &move);

    /** chose what type of generator to use in the iteration */
    void choseNeighborhood();


public:
    /** run the methaheuristic */
    void run();

    void printOuput();

    /** move info */
    void writeMoveInfo();
    void recordMoveInfo();

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

    /** cost of last length_memory solution */
    vector<double> cost_memory_;

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
    Option<int> runs;


    /** the cost objects determining the total cost of solutions and routes*/
    vector<Cost*> &cost_components_solution_;
    vector<Cost*> &cost_components_route_;


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


    /** the best delta found so far in the current iteration */
    double best_delta_;

    /** save best cost of solution found so far */
    double best_solution_cost_;

    /** time spent in the initialization + search */
    double time_;

    /** if the routes were destroyed by the move */
    bool route_1_still_exists_;
    bool route_2_still_exists_;


    /** ------------------------------------------------------------------
     *                     HISTORY VARS (for inspection)
     *  ------------------------------------------------------------------*/
    /** cost history */
    vector<double> cost_history_;

    /** the solution before the move is performed (only used for inspection) */
    Solution initial_solution_;
    Solution old_solution_;

    /** the complete evolution of duals */
    vector<vector<double>> dual_vars_complete_history_;

    /** print stuff */
    ofstream moves_out_file_;

    /** moves info */
    int moves_with_route_deletion_;
    vector<int> moves_path_lenght_;

};

