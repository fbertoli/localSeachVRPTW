//
// Created by Bertoli, Francesco (Data61, Canberra City) on 15/02/17.
//

#ifndef LOCALSEARCH_INITIALIZER_H
#define LOCALSEARCH_INITIALIZER_H

#include "solution.h"
#include "data.h"
#include "routes.h"

class Initializer
{
    /** CONSTRUCTORS */
public:
    Initializer(Data &data) :
            data_(data), capacity_(data_.capacity_), route_container_(nullptr), clusterisation_(nullptr), profits_(nullptr), cost_components_route_(nullptr), cost_components_solution_(nullptr)
    {}

    /** METHODS */
public:
    /** return the initial solution */
    virtual Solution initializeSolution() = 0;

    /** re set the pointer */
    void setClusterisationPtr(vector<int> *clusterisation) {clusterisation_ = clusterisation;};
    virtual void setProfitPtr(vector<double> *profits) {};
    void setRouteContainerPtr(RouteContainer *rc) {route_container_ = rc;};
    void setRouteCostComponents(vector<Cost*> *cost_components_route) {cost_components_route_ = cost_components_route;}

    /** modify capacity */
    void setNewCapacity(double new_capacity) {capacity_ = new_capacity;}

    /** access capacity */
    double getCapacity() { return  capacity_;}

    /** if there is a swticher */
    virtual void switcher(string s) {};



    /** VARIABLES */
public:
    /** the instance */
    Data &data_;

    /** (modified) capacity */
    double capacity_;

    /**the route container to be updated */
    RouteContainer *route_container_;

    /** pointer to vector defining which cluster a customer is in */
    vector<int> *clusterisation_;

    /** pointer to vector defining the profits of customers*/
    vector<double> *profits_;

    /** the cost objects determining the total cost of solutions and routes*/
    vector<Cost*> *cost_components_solution_;
    vector<Cost*> *cost_components_route_;
};


#endif //LOCALSEARCH_INITIALIZER_H
