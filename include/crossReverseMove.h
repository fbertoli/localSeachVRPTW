//
// Created by Bertoli, Francesco (Data61, Canberra City) on 09/02/17.
//

#ifndef LOCALSEARCH_CROSSREVERSEMOVE_H
#define LOCALSEARCH_CROSSREVERSEMOVE_H

#include "move.h"
#include "solution.h"
#include <utility>

class Cost;

/** This class implements a crossReverse type move.
 */


class CrossReverseMove : public Move{


    /** CONSTRUCTORS - ASSIGNMENT*/
public:
    CrossReverseMove() {};

    CrossReverseMove(int i, int j, int k, int l) :  i_(i), j_(j), l_(l), k_(k) {};

    CrossReverseMove& operator= (const CrossReverseMove &other_move);


    /** METHODS */
public:
    /** needed to call computeCost in cost functions */
    virtual double computeCost(Solution &solution, Cost *cost) {return cost -> computeDelta(solution, *this);}

    /** perform the move */
    virtual void performMove(Solution &solution);

    /** se the move varialbes */
    void setMoveVariables(int i, int j, int k, int l, int route_1, int route_2, bool reverse_path_1, bool reverse_path_2);

    /** get name */
    virtual string getName() {return "crossReverseMove";}

    /** save the indexes of the new routes in the new solution (solution here is the one before the move is performed) */
    virtual void saveModifiedRoutesIndexes(Solution &solution, vector<int> &routes);

    /** add arcs (relative to solution) to removed_arcs that move is adding */
    virtual void identifyRemovedArcs(Solution &solution, vector<pair<int,int>> &removed_arcs);

    /** add arcs (relative to solution) to new_arcs_ that move is adding */
    virtual void identifyAddedArcs(Solution &solution, vector<pair<int,int>> &new_arcs);


//    /** update the forbidden list/arcs */
//    virtual void updateForbiddenArcs(Solution &solution, int tabu_duration);
//
//private:
//    /** utility functio to add arcs to the correct list */
//    void addArc(pair<int,int> arc);



    /** VARIABLES */
public:
    /** route involved in the exchanged of path */
    int route_1_;
    int route_2_;

    /** we are exchange the path i + 1 -> j of route_1_ with the path k + 1 -> l of route_2_ */
    int i_, j_, k_, l_;

    /** if orientation of path 1 (path 2) is to be exchanged */
    bool reverse_path_1_, reverse_path_2_;

    /** the delta cost I calculate in the generator */
    double delta_distance_;
    double delta_capacity_;

    /** whether the move is removing a route */
    bool route_removed_;


};

#endif //LOCALSEARCH_CROSSREVERSEMOVE_H
