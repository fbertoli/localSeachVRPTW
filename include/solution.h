//
// Created by Bertoli, Francesco (Data61, Canberra City) on 01/02/17.
//

#ifndef LOCALSEARCH_SOLUTION_H
#define LOCALSEARCH_SOLUTION_H

#include <vector>
#include <string>
#include "data.h"
#include "cost.h"
#include "options.h"

/** this class implements a solution of the VRPTW */


using namespace std;


class Solution
{


    /** CONSTRUCTORS - ASSIGNMENT */
public:
    Solution(const Data &data, vector<int> tour, vector<double> arrivals, vector<double> departures, vector<double> latest_departures, vector<int> start_positions);

    Solution(const Data &data, vector<int> tour, vector<double> arrivals);

    Solution(const Data &data, vector<int> tour);

    Solution(const Data &data) : data_(data) {};

    Solution& operator= (const Solution &solution);

    /** METHODS */
public:
    /** insert visit in tour_ at position pred_index+1, route indicates which route we are modifying */
    void insertCustomer(int route, int visit, int pred_index);

    /** remove the visit at position (route only indicates which route we are modifying) */
    void removeCustomer(int route, int position);

    /** swap and possibly reverse two paths between two routes. We assume route_1 < route_2. If start_path_1 = 1 + end_path_1 it inserts path_2_ after start_path_1*/
    void swapReversePaths(int route_1, int route_2, int start_path_1, int end_path_1, int start_path_2, int end_path_2, bool reverse_path_1, bool reverse_path_2);

    /** compute departure time */
    inline double departureTime(int visit) {return (visit < data_.n_requests_) ? (max(arrivals_[visit], data_.start_TW_[visit]) + data_.service_time_[visit]) : 0;}

    /** is the given index in tour a depot? */
    inline bool indexIsDepot(int index) {return tour_[index] >= data_.n_requests_;}

    /** write the solution to the specified file */
    void write(string file_name, vector<Cost*> &costs);

    /** check if the solution is feasible (capacity anf time constraints)*/
    bool checkFeasibilitySolution();

    /** compute cost */
    void computeCost(vector<Cost*> &cost_components_solution);

    /** compute cost of single routes (if routes_indexes is empty all routes otherwise a subset) */
    void computeRoutesCost(vector<Cost*> &cost_components_route, vector<int> routes_indexes = {});

//    /** solve the TSP problem or a route and reorganize the route */
//    void TSProute(int route);
//
//    /** refine all routes with TSProute */
//    void refineRoutesTSP();


    /** VARIABLES */
public:
    /** data of the instance */
    const Data &data_;

    /** represent the solution. [N, route_0, N+1, route_1, N+2, ..., N + n_routes] */
    vector<int> tour_;

    /** index of tour where a route starts. Also the last one (therefore n_routes +1 indices) */
    vector<int> start_positions_;

    /** arrivals_[i] = arrival time at location i in the solution, note: arrival time at copies of depot is intended as the depot at the end of a route */
    vector<double> arrivals_;

    /** departure[i] = departure time from location i in the solution, note: departure time at copies of depot is always 0 */
    vector<double> departures_;

    /** latest_departure[i] = max departure time from location i before losing feasibility, note: latest departure time at copies of depot is intended as the depot at the start of a route */
    vector<double> latest_departures_;

    /** number of routes in the solution (actual number, start_positions has n_routes+1 indexes*/
    int n_routes_;


    /** the true and fake cost of the solution */
    double cost_;

    /** multiplicative coefficient for capacity to increase/decrease it */
    static double capacity_coefficient_;

    /** cost of routes */
    vector<double> routes_costs_;


//    /** time limit (in seconds) in optimization route with TSP */
//    Option<int>
//    int tsp_time_limit_;



};

#endif //LOCALSEARCH_SOLUTION_H
