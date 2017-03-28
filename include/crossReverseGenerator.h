//
// Created by Bertoli, Francesco (Data61, Canberra City) on 09/02/17.
//

#ifndef LOCALSEARCH_CROSSREVERSEGENERATOR_H
#define LOCALSEARCH_CROSSREVERSEGENERATOR_H


#include <vector>
#include "data.h"
#include "solution.h"
#include "crossReverseMove.h"
#include "moveGenerator.h"
#include "options.h"

/** This class implements the generator for crossReverse type moves.
 *  It handles the necessary data structure to avoid computing the same quantities twice.
 */



using namespace std;
class CrossReverseGenerator : public MoveGenerator
{

    /** CONSTRUCTORS */
public:
    CrossReverseGenerator(const Data &data, Solution* sol, double capacity_coefficient);

    /** METHODS */
public:
    /** assign the first feasible move to move (note the capacity could be modified)*/
    virtual bool first(Move &move);
    /** assign the nets feasible move to move (the order is defined through while loops)*/
    virtual bool next(Move &move);
    /** assign a random feasible move to move (still to be implement) */
    virtual bool random(Move &move);

    /** access capacity_coefficient_ */
    double getCapacityCoefficient() {return capacity_coefficient_;}

private:
    /** update the routes_load_ field */
    void computeRoutesLoad();

    /** update the visit_ variables given the indexes in tour_ */
    void setVisits();

    /** update the move fields (included the costs) */
    void foundMove(double candidate_delta, CrossReverseMove &move, bool reverse_path_1, bool reverse_path_2);

    /** update the move fields (included the costs) */
    void updateMove(CrossReverseMove &move);
    inline bool feasibilityFromPath2Standard();
    inline bool feasibilityFromPath2Reversed();
    inline bool feasibilityFromPath1Standard();
    inline bool feasibilityFromPath1Reversed();
    inline bool feasibilityCapacityRoute1();
    inline bool feasibilityCapacityRoute2();
    inline double deadlineComingBackToRoute1();
    inline double deadlineComingBackToRoute2();

    /** compute the delta of the candidate move */
    double computeDelta(bool standard_path_1, bool standard_path_2);


    /** VARIABLES */
public:
    /** the maximum length of path to exchange */
    Option<int> max_length_path_opt_;
    int max_length_path_;

    /** multiplicative coefficient for capacity to increase/decrese it */
    double capacity_coefficient_;

    /** route involved in the exchanged of path */
    int route_1_;
    int route_2_;

    /** we are exchange the path i+1 -> j of route_1_ with the path k+1 -> l of route_2_ */
    int i_,j_,k_,l_;

    /** corresponding visits and successors */
    int visit_i_, visit_j_, visit_k_, visit_l_, visit_succ_i_, visit_succ_k_, visit_succ_j_, visit_succ_l_;

    /** the best_delta of a move found so far and the current move's one */
    double best_delta_;

    /** auxiliary varaibles */
    double delta_distance_;
    double delta_reduced_cost_;
    double delta_;

    /** record if there is a next possibility */
    bool has_next_;
    bool move_found_;

    /** record the feasibility of reaching path 1 (standard and reverse orientation) from route 2  */
    bool feasibility_to_path_1_standard_, feasibility_to_path_1_reversed_;

    /** record the feasibility of reaching path 2 (standard and reverse orientation) from route 1  */
    bool feasibility_to_path_2_standard_, feasibility_to_path_2_reversed_;

    /** record the feasibility of the whole route */
    bool feasibility_route_2_standard_, feasibility_route_2_reversed_, feasibility_route_1_standard_, feasibility_route_1_reversed_;


    /** record from which l_ we should start looking at every next iteration of j to check for feasibility of new route 2 */
    vector<vector<vector<int>>> start_l_index_;

    /** departures_path_1_[visit_j_][visit_succ_i_][visit_k_] record the departure time from j_ of the path (depot_,..,k_, i_+1,..,j_) */
    vector<vector<vector<double>>> departures_path_1_;

    /** departures_path_2_[visit_l_][visit_succ_k_][visit_i_] record the departure time from l_ of the path (depot_,..,i_, k_+1,..,l_) */
    vector<vector<vector<double>>> departures_path_2_;

    /** path_latest_departure_[i_][i_] the latest departure from i_ so that feasibility is not lost up to j_ (the order of the index establish the orientation)*/
    vector<vector<double>> path_latest_departure_;

    /** the load of paths */
    vector<vector<int>> paths_load_;

    /** [i][j] = distance of path i-->j - dual_vars of nodes on the path i--->j */
    vector<vector<double>> path_cumulative_cost;


    /** keep memory of the total load of each route */
    vector<int> routes_load_;

public:
    /** dual variables */
    vector<double> *dual_variables_;

    /** whether to count the dual load or not */
    bool reduced_cost_option_;


    friend class CrossReverseMove;
};


#endif //LOCALSEARCH_CROSSREVERSEGENERATOR_H
