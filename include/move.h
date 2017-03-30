//
// Created by Bertoli, Francesco (Data61, Canberra City) on 06/02/17.
//

#ifndef LOCALSEARCH_MOVE_H
#define LOCALSEARCH_MOVE_H

#include <string>
#include <vector>


class Solution;
class Cost;

using namespace std;

class Move {
//public:
//    virtual  Move& operator= (const Move &other_move);

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

};

#endif //LOCALSEARCH_MOVE_H
