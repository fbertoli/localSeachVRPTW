//
// Created by Bertoli, Francesco (Data61, Canberra City) on 02/02/17.
//

#ifndef LOCALSEARCH_CAPACITYCOST_H
#define LOCALSEARCH_CAPACITYCOST_H

#include "crossReverseMove.h"
#include "cost.h"
#include "options.h"

class CostCapacity : public Cost{

    /** CONSTRUCTORS */
public:
    CostCapacity(const Data &data) : Cost::Cost(data), name_("costCapacity"), penalization_("cap_penalty", "penalty for exceeding real capacity", 0) {}


    /** METHODS */
public:
    virtual double computeDelta(Solution& solution, Move &move) {return move.computeCost(solution, this);};

    virtual double computeDelta(Solution& solution, CrossReverseMove &move);

    virtual double computeCost(Solution& solution);

    virtual void addRoutesCost(Solution& solution, vector<double> &routes_costs, vector<int> routes_indexes = {});

    void setPenalization(double d) {penalization_ = d;};

    void computeRoutesLoad(Solution& solution, vector<int> &routes_load, vector<int> routes_indexes = {});

    virtual string getName() {return name_;};



    /**VARIABLES */
private:
    /** penliazation for exceeding standard capacity */
    Option<double> penalization_;

    /** name of the cost (to debug) */
    string name_;


};

#endif //LOCALSEARCH_CAPACITYCOST_H
