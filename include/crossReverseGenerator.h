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
#include "costFixed.h"
#include "costCapacity.h"

/** This class implements the generator for crossReverse type moves.
 *  It handles the necessary data structure to avoid computing the same quantities twice.
 */



using namespace std;
class CrossReverseGenerator : public MoveGenerator
{

    /** CONSTRUCTORS */
public:
    CrossReverseGenerator(const Data &data, Solution* sol, double capacity_coefficient, CrossReverseMove *best_move, CrossReverseMove *move);

    /** METHODS */
public:
    /** assign the first feasible move to move (note the capacity could be modified)*/
    virtual bool first();

    /** assign the nets feasible move to move (the order is defined through while loops)*/
    virtual bool next();

    /** assign a random feasible move to move (still to be implement) */
    virtual bool random();

    /** copy the content of the current move into best_move_ */
    virtual void updateBestMove() {*best_move_ = *move_;};

private:
    /** update the routes_load_ field */
    void computeRoutesLoad();

    /** update the visit_ variables given the indexes in tour_ */
    void setVisits();

    /** get the corresponding visit */
    inline int getVisit(int route, int position) {return current_solution_->tour_[current_solution_->start_positions_[route] + position];}
    inline int getVisit(int position) {return current_solution_->tour_[position];}

    /** update the move fields (included the costs) */
    void foundMove(bool reverse_path_1, bool reverse_path_2);

    /** functions to check feasibility */
    inline bool feasibilityFromPath2Standard();
    inline bool feasibilityFromPath2Reversed();
    inline bool feasibilityFromPath1Standard();
    inline bool feasibilityFromPath1Reversed();
    inline bool feasibilityCapacityRoute1();
    inline bool feasibilityCapacityRoute2();
    inline double deadlineComingBackToRoute1();
    inline double deadlineComingBackToRoute2();

    /** compute the delta wrt to cost source */
    void computeDeltaDistance(bool reverse_path_1, bool reverse_path_2, double &delta_distance);
    void computeDeltaCapacity(double &delta_capacity);

//    /** return the path cost of start -> start + step + 1 using  start -> start + step (start is relative to tour_) */
//    double extendStandardPathCost(int start, int step);
//    /** return the path cost of start + step + 1 --> start using  start + step -> start  (start is relative to tour_) */
//    double extendReversedPathCost(int start, int step);

    /** intialize path cost (position is relative to tour_) */
    inline double initializePathCost(int position) {return 0;}


    /** VARIABLES */
public:
    /** pointer to the best_move_ and move_ (the one that gets modified) objects */
    CrossReverseMove *best_move_, *move_;

    /** the maximum length of path to exchange */
    Option<int> max_length_path_;
//    int max_length_path_;

    /** multiplicative coefficient for capacity to increase/decrese it */
    double modified_capacity_;

    /** route involved in the exchanged of path */
    int route_1_;
    int route_2_;
    int length_route_1_;
    int length_route_2_;

    /** we are exchange the path i+1 -> i_+j_ of route_1_ with the path k+1 -> k_+l_ of route_2_
     *  all index are relative to current_solution_->start_positions_[route_1/2_]   */
    int i_,j_,k_,l_;

    /** visits corresponding to indexes and successors (note, visit_l refers to visit in current_solution_->start_positions_[route_2_] + k_+l_ ] )*/
    int visit_i_, visit_j_, visit_k_, visit_l_;
    int visit_succ_i_, visit_succ_k_, visit_succ_j_, visit_succ_l_;

    /** index in current_solution_-> tour_ corresponding to index here */
    int tour_index_i_, tour_index_k_, tour_index_j_, tour_index_l_;

    /** record if there is a next possibility to examine */
    bool has_next_;

    /** record the feasibility of reaching path 1 (standard and reverse orientation) from route 2  */
    bool feasibility_to_path_1_standard_, feasibility_to_path_1_reversed_;

    /** record the feasibility of reaching path 2 (standard and reverse orientation) from route 1  */
    bool feasibility_to_path_2_standard_, feasibility_to_path_2_reversed_;

    /** record the feasibility of the whole route */
    bool feasibility_route_2_standard_, feasibility_route_2_reversed_, feasibility_route_1_standard_, feasibility_route_1_reversed_;

    // note that in all of the below matrices, the step_ 0 is not used as it corresponds to no path, this could be optimised

    /** departures_path_1_standard_[step_][b_][a_] record the departure time from b_+step_ of the path (depot_,..,a_, b_,..,b_+step_)
     *  departures_path_1_reversed_[step_][b_][a_] record the departure time from b_ of the path (depot_,..,a_, b_+step_,..,b_)
     *  (indexes a,b are relative to routes) */
    vector<vector<vector<double>>> departures_path_1_standard_, departures_path_1_reversed_;
    vector<vector<vector<double>>> departures_path_2_standard_, departures_path_2_reversed_;

    /** path_latest_departure_standard_[a_][step_] the latest departure from position a_ so that feasibility is not lost up to position a_ + step_
     *  path_latest_departure_reversed_[a_][step_] -> latest departure from a_ + step_ to get to a_
     *  (index a_ is relative to tour_) */
    vector<vector<double>> path_latest_departure_standard_;
    vector<vector<double>> path_latest_departure_reversed_;

    /** paths_load_[a_][step_] the load of path a_ --> a_ + step_
     * (index a_ is relative to tour_) */
    vector<vector<int>> paths_load_;

//    /** path_cumulative_cost_standard_[a_][step_] = cost of the of path a_ --> a_ + step_
//     *  path_cumulative_cost_reversed_[a_][step_] = cost of the of path a_ + step_ --> a_
//     *  (index a_ is relative to tour_) */
//    vector<vector<double>> path_cumulative_cost_standard_;
//    vector<vector<double>> path_cumulative_cost_reversed_;

    /** keep memory of the total load of each route */
    vector<int> routes_load_;



    friend class CrossReverseMove;
    friend class costFixed;
    friend class costCapacity;
};


#endif //LOCALSEARCH_CROSSREVERSEGENERATOR_H
