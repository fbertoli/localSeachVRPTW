//
// Created by Bertoli, Francesco (Data61, Canberra City) on 06/02/17.
//

#ifndef LOCALSEARCH_MOVE_H
#define LOCALSEARCH_MOVE_H

#include <string>
#include <vector>
#include <utility>


class Solution;
class Cost;

using namespace std;

class Move {
//public:
//    virtual  Move& operator= (const Move &other_move);

    /** METHODS */
public:
    /** needed to compute delta of the move */
    virtual double computeCost(Solution &solution, Cost *cost) = 0;

    /** perform the move on the soluttion passed */
    virtual void performMove(Solution &solution) = 0;

    /** name of the move */
    virtual string getName() = 0;

    /** Given the solution before the move is performed, add to routes the indexes of the routes
     * in the new solution (after move is performed) that have been modified.
     * If a route is deleted it simply doesn't add it */
    virtual void saveModifiedRoutesIndexes(Solution &solution, vector<int> &routes) = 0;

    /** change the pointers  */
//    void setForbiddenListPntr(vector<pair<int,int>> *forbidden_list) {forbidden_list_ = forbidden_list;}
//    void setForbiddenArcsPntr(vector<vector<int>> *forbidden_arcs) {forbidden_arcs = forbidden_arcs;}

    /** add arcs (relative to solution) that move is adding */
    virtual void identifyAddedArcs(Solution &solution, vector<pair<int,int>> &new_arcs) = 0;

//
//    /** VARIABLES */
//public:
//    /** the forbidden arcs */
//    vector<vector<int>> *forbidden_arcs_;
//
//    /** the list of currently forbidden arcs */
//    vector<pair<int,int>> *forbidden_list_;
//
//    /** whther the move is tabu */
//    bool is_tabu_;

};


#endif //LOCALSEARCH_MOVE_H
