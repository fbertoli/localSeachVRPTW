//
// Created by Bertoli, Francesco (Data61, Canberra City) on 02/02/17.
//

#ifndef LOCALSEARCH_DISTANCECOST_H
#define LOCALSEARCH_DISTANCECOST_H

#include "crossReverseMove.h"
#include "cost.h"

class CostDistance : public Cost{

    /** CONSTRUCTORS */
public:
    CostDistance(const Data &data) : Cost::Cost(data), name_("costDistance") {}


    /** METHODS */
public:

    virtual double computeDelta(Solution& solution, Move &move) {return move.computeCost(solution, this);};

    virtual double computeDelta(Solution& solution, CrossReverseMove &move);

    virtual double computeCost(Solution& solution);

    virtual void addRoutesCost(Solution& solution, vector<double> &routes_costs, vector<int> routes_indexes);

    virtual string getName() {return name_;};

    /**VARIABLES */
public:
    /** name of the cost (to debug) */
    string name_;
};
#endif //LOCALSEARCH_DISTANCECOST_H
