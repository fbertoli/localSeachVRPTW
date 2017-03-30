#include <iostream>
#include <stdlib.h>
#include "data.h"
#include "solution.h"
#include "initializerInsertion.h"
#include "move.h"
#include "moveGenerator.h"
#include "costFixed.h"
#include "costDistance.h"
#include "costCapacity.h"
#include "parameter.h"
#include "parse.h"
#include "crossReverseGenerator.h"
#include "options.h"
#include "initializerMultipleRoutes.h"
#include "lateAcceptanceSteepestDescent.h"


int main(int argc, char **argv) {

    using namespace std;

    /** ======================================================
     * DECLARE PARAMETERS
     * ====================================================== */
    Option<string> instance("i", "instance name", "");
    Options::parse(argc, argv);

    // READ INSTANCE DATA
    Data data(instance);


    /** ======================================================
     * COSTS AND NEIGHBORHOODS
     * ====================================================== */

    // COSTS
    Option<double> fixed_cost_value("fixed", "fixed cost of vehicles", 0);
    Option<double> capacity_coefficient("cap", "multiplicative coefficient for expanding capacity", 1);
    Solution::capacity_coefficient_ = capacity_coefficient;

    CostDistance distance_cost{data};
    CostFixed fixed_cost(data, fixed_cost_value);
    CostCapacity capacity_cost(data);


    // CREATE MOVES and GENERATORS
    CrossReverseMove cross_reverse_move;
    CrossReverseMove cross_reverse_move_best;
    CrossReverseGenerator cross_reverse_generator(data, nullptr, capacity_coefficient, &cross_reverse_move_best, &cross_reverse_move);


    // CREATE VECTOR of POSSIBLE COSTS and MOVES
    vector<Cost*> possible_costs{&distance_cost, &fixed_cost, &capacity_cost};
    vector<Move*> possible_moves{&cross_reverse_move};
    vector<Move*> possible_best_moves{&cross_reverse_move_best};
    vector<MoveGenerator*> possible_generators{&cross_reverse_generator};


    // READ COST, MOVES and GENERATORS FROM CONFIG FILE
    vector<Cost*> cost_components_solution;
    vector<Cost*> cost_components_route;
    vector<Move*> moves;
    vector<Move*> best_moves;
    vector<MoveGenerator*> generators;
    parseConfigFile("./config-file.txt", possible_costs, cost_components_solution, cost_components_route, possible_moves, moves, possible_best_moves, best_moves, possible_generators, generators, data);



    /** ======================================================
     * INITIALIZER AND METAHEURISTIC
     * ====================================================== */
     // CRATE INITIALIZER
    InitializerInsertion I1(data);
    InitializerMultipleRoutes multiple_routes(data);
    Initializer *initializer;


    // CREATE METAHEURISTIC
    LateAcceptanceSteepestDescent algorithm(data, initializer, cost_components_solution, cost_components_route, moves, best_moves, generators);


    /** ======================================================
    * PARSE OPTIONS
    * ====================================================== */
    Option<int> initializer_code("init", "heuristic to initialize solution. 0 = insertionI1, 1 = multipleRoutes", 0);
    Option<int> metaheuristic_code("m", "metaheuristic", 0);
    Option<double> capacity_coeff_initializer_("init-cap", "capacity coefficient in the initialization procedure", 1);
    Options::parse(argc, argv);



    /** ======================================================
     * CHOOSE INITIALIZER
     * ====================================================== */

    if (initializer_code == 0) {
        initializer = &I1;
        initializer->setNewCapacity(data.capacity_*capacity_coeff_initializer_ );
    }
    else if (initializer_code == 1)
        initializer = &multiple_routes;


    algorithm.initializer_ = initializer;


    /** ======================================================
     * CHOOSE METAHEURISTIC
     * ====================================================== */



    /** ======================================================
    * RUN METAHEURISTIC
    * ====================================================== */

//    algorithm.reset();
    algorithm.run();
    algorithm.printOuput();

}