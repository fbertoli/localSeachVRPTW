#pragma once

#include <fstream>
#include "solution.h"
#include "data.h"
#include "options.h"
#include "initializer.h"
#include "move.h"
#include "cost.h"
#include "moveGenerator.h"
#include "gurobi_c++.h"
#include "routes.h"
#include "metaheuristic.h"
#include <utility>

/** This class implements a LateAcceptanceSteepestDescent metaheuristic. Given a set of neighborhood it explores
 * them exhaustively and choose the best move.
 * The acceptance criterion is: whether if the cost is better than the cost of length_memory_ iteration before. */


using namespace std;


class TabuSearch : public Metaheuristic
{
    /** CONSTRUCTORS */
public:
    TabuSearch(Data &data, Initializer *initializer, vector<Cost*> &cost_components_solution, vector<Cost*> &cost_components_route, vector<Move*> &moves,  vector<Move*> &best_moves, vector<MoveGenerator*> &generator);

    ~TabuSearch();

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
    virtual void run();

    virtual void printOuput();

    /** move info */
    void writeMoveInfo();
    void recordMoveInfo();

    /** to inspect clusterisation */
    void createClusterisation();
    void updateClusterisation();

    /** make metaheursitic ready to start again */
    void reset();

    /** update the forbidden list/arcs */
    void updateForbiddenArcs();

    /** check if any added arc is tabu */
    bool isMoveTabu();




    /** VARIABLES */
public:
    /** clusterisation resulting from search */
    vector<int> clusterisation_;

    /** constainer for the routes found so far */
    RouteContainer routes_pool_;

    /** the indexes of the new routes in the solution after the move is performed */
    vector<int> routes_indexes_;

    /** the current solution */
    Solution current_solution_;

    /** the current iteration number */
    int iteration_;

       /** maximum number of iterations*/
    Option<int> max_iterations_;


    /** the best delta found so far in the current iteration */
    double best_delta_;

    /** save best cost of solution found so far */
    double best_solution_cost_;

    /** time spent in the initialization + search */
    double time_;


    /** ------------------------------------------------------------------
     *                     TABU VARIABLES
     *  ------------------------------------------------------------------*/

    /** matrix to keep memory of forbidden arcs and how long they will remain forbidden */
    vector<vector<int>> forbidden_arcs_;

    /** list recording which arcs are currently forbidden, so that it is easy to update their taboo status in the matrix */
    vector<pair<int, int>> forbidden_list_;

    /** minimum/maximum iterations one arc remains tabu */
    Option<int> min_tabu_iterations_, max_tabu_iterations_;


    /** the arcs the move is adding */
    vector<pair<int, int>> new_arcs_;


    /** ------------------------------------------------------------------
     *                     HISTORY VARS (for inspection)
     *  ------------------------------------------------------------------*/
    /** cost history */
    vector<double> cost_history_;

    /** the solution before the move is performed (only used for inspection) */
    Solution initial_solution_;
    Solution old_solution_;

    /** print stuff */
    ofstream moves_out_file_;

    /** moves info */
    bool is_best_move_tabu_;
    int moves_with_route_deletion_;
    vector<int> moves_path_lenght_;
    int moves_tabu_;
    double moves_non_improving_;

};

