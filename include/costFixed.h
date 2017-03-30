//
// Created by Bertoli, Francesco (Data61, Canberra City) on 02/02/17.
//

#ifndef LOCALSEARCH_FIXEDCOST_H
#define LOCALSEARCH_FIXEDCOST_H


#include "crossReverseMove.h"
#include "cost.h"


class CostFixed : public Cost{

    /** CONSTRUCTORS */
public:
    CostFixed(const Data &data, double fixed_cost = 0) : Cost::Cost(data), name_("costFixed"), fixed_cost_(fixed_cost) {}



    /** METHODS */
public:

    virtual double computeDelta(Solution& solution, Move &move) {return move.computeCost(solution, this);};

    virtual double computeDelta(Solution& solution, CrossReverseMove &move);

    virtual double computeCost(Solution& solution);

    virtual void addRoutesCost(Solution& solution, vector<double> &routes_costs, vector<int> routes_indexes = {});

    void setFixedCost(double new_cost) {fixed_cost_ = new_cost;};

    virtual string getName() {return name_;};

    /**VARIABLES */
private:
    double fixed_cost_;

    /** name of the cost (to debug) */
    string name_;


};

#endif //LOCALSEARCH_FIXEDCOST_H
