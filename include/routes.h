//
// Created by Bertoli, Francesco (Data61, Canberra City) on 17/02/17.
//

#ifndef LOCALSEARCH_ROUTE_H
#define LOCALSEARCH_ROUTE_H

#include "solution.h"
#include "options.h"
#include "data.h"
#include "gurobi_c++.h"
#include <fstream>
#include <vector>

/** we implement a class for a route a a class for a route container, which handles the SPP model as well */

/** ------------------------------------------------------------------------------------------------ */
/** ROUTE */
/** ------------------------------------------------------------------------------------------------ */
using namespace std;

/** class representing a route*/
class Route {
    /** CONSTRUVTORS */
public:
    Route();
    Route(vector<int> stops, double cost_ = 0);
    Route& operator= (const Route &route);

    /** VARIABLES */
public:
    vector<int> stops_; // excluding depot
    double cost_; //the cost
//    vector<double> arrivals_; // following the order of stops_;

};




/** ------------------------------------------------------------------------------------------------
 *  RouteContainer
 *  ------------------------------------------------------------------------------------------------ */

class RouteContainer
{
    /** CONSTRUCTORS */
public:
    RouteContainer(Data &data);

    ~RouteContainer();

    /** METHODS */
public:
    /** add a route to the container */
    void addRoute(Route &route) {routes_.push_back(route);};

    /** extracts routes indexed by index_routes_ from solution and add to the container. All of them if index routes_ is empty. */
    void extractRoutes(Solution &solution, vector<int> index_routes_ = {});

    /** create/update model from the routes in the container now */
    void createIntegerModel();
    void updateIntegerModel();
    void updateIntegerSolution();
//    void createLinearModel();
//    void updateLinearModel();
//    void updateDualVariables();


    /** optimize */
    void optimizeIntegerModel(double time = 0, double gap = 0);
//    void optimizeLinearModel() {linear_model_.optimize();}

//    /** only to inspecting dual variables */
//    string whiteSpaces(double d, int spaces = 8);
//    void writeDualVariables();
//    void printEndBasicSearch();


    /** VARIABLES */
public:
    /** the data representing the instance */
    Data &data_;

    /** all the routes found so far */
    vector<Route> routes_;

    /** ------------------------------------------------------------------
     *                          MODELS VARS
     *  ------------------------------------------------------------------*/
    GRBEnv* env_;
    vector<GRBConstr> integer_constraints_;
    vector<GRBVar> integer_vars_;
    GRBModel integer_model_;
//    vector<GRBConstr> linear_constraints_;
//    vector<GRBVar> linear_vars_;
//    GRBModel linear_model_;

    /** the time given to the integer model for optimization */
    Option<double> optimization_time_;

    /** record the solution obtained by the integer model */
    Solution solution_;

    /** record the last route that was included in the models */
    int to_include_integer_;
//    int to_integrate_linear_;

//    /** output file */
//    ofstream out_file_;




};


#endif //LOCALSEARCH_ROUTE_H
