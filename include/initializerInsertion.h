//
// Created by Bertoli, Francesco (Data61, Canberra City) on 24/02/17.
//

#pragma once

#include "solution.h"
#include "data.h"
#include <vector>
#include "initializer.h"
#include "options.h"

using namespace std;

/** This class implements a generalization of the I1 insertion heuristic (cost can be modified) presented in: Braysy, Gendrau - Vehicle Routing Problem with Time Windows, Part I. Route Construction and Local Search Algorithms
 *  The costs are redefined through initializerCosts.h and controlled through the filed switcher_;
 */

/** structures needed for sorting */

struct comparisonDistancesStruct {
    Data &data_;
    bool operator() (int i,int j) { return data_.distances_[data_.depot_][i] < data_.distances_[data_.depot_][j];}
};


struct comparisonTWStruct {
    Data &data_;
//    bool operator() (int i,int j) { return data_.start_TW_[i] > data_.start_TW_[j];}
    bool operator() (int i,int j) { return data_.end_TW_[i] > data_.end_TW_[j];}
};


struct comparisonProfitStruct {
    vector<double> *profits_;
    bool operator() (int i,int j) { return profits_->at(i) < profits_->at(j);}
};



/** main class --------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------*/


class InitializerInsertion : public Initializer
{
    /** CONSTRUCTORS */
public:
    InitializerInsertion(Data &data);


    /** METHODS */
public:
    /** build a solution */
    virtual Solution initializeSolution();

    /** select the seed customer */
    virtual int selectSeedCustomer();

    /** do any sorting or other operation that are needed by the particular child-class*/
    virtual void preProcessing();

    /** set profits pointer */
    virtual void setProfitPtr(vector<double> *profits);

    /** sort based on the profits */
    virtual void sortOnProfits();

    /** change switcher */
    virtual void switcher(string mode);





/** VARIABLES */
public:
    /** switcher defining what insertionCost/selectSeedCustomer do we use to initialize. 0 = Solomon , 1 = profit */
    int switcher_;

    /** control how the seed customer is chosen */
    static Option<int> seed_selection_;

    /** vector recording which customer are still not assigned */
    vector<bool> unrouted_;

    /** how many customers still have to be assigned */
    int unrouted_req_;

    /** rank customer based on distances from depot */
    vector<int> rank_distances_;

    /** rank customer based on how end_TW (or start_TW) they have to be services */
    vector<int> rank_TW_;

    /** rank customer based on profit */
    vector<int> rank_profits_;

    /** operator to compare indexes based on Profits */
    comparisonProfitStruct profit_comparison_;

};

