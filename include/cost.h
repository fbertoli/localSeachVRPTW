//
// Created by Bertoli, Francesco (Data61, Canberra City) on 06/02/17.
//

#ifndef LOCALSEARCH_COST_H
#define LOCALSEARCH_COST_H

#include <string>
#include <vector>


class Solution;
class CrossReverseMove;
class Data;
class Move;

class Cost {

    /** CONSTRUCTORS */
public:
    Cost(const Data &data) : data_(data) {}


    /** METHODS */
public:

    /** compute the delta of a particular move on the given solution */
    virtual double computeDelta(Solution& solution, Move &move) = 0;
    virtual double computeDelta(Solution& solution, CrossReverseMove &move) = 0;

    /** compute the cost of the given solution */
    virtual double computeCost(Solution& solution) = 0;

    /** for each route index in route_indexes add the corresponding cost to routes_costs[index]. If routes_indexes is empty do it for all routes */
    virtual void addRoutesCost(Solution& solution, vector<double> &routes_costs, vector<int> routes_indexes = {}) = 0;

    virtual string getName() = 0 ;


    /**VARIABLES */
public:
    /** pointer to data of the instance */
    const Data &data_;

    /** name of the cost (to debug) */
    string name_;
};
#endif //LOCALSEARCH_COST_H
