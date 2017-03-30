//
// Created by Bertoli, Francesco (Data61, Canberra City) on 01/02/17.
//

#ifndef LOCALSEARCH_PARSEDATA_H
#define LOCALSEARCH_PARSEDATA_H
#include <string>
#include <vector>

/** This class represent the instance being solved
 * */

using namespace std;
class Data
{
public:
    Data(string problem_file_name);

private:
    double computeDistance(int x_1, int y_1, int x_2, int y_2);

    int stringToInt(const string &Text);

    double stringToDouble(const string &Text);

public:
    /** number of requests */
    int n_requests_;

    /** maximum vehicles available */
    int max_vehicles_;

    /** capacit yof a vehicle */
    int capacity_;

    /** the depot */
    int depot_;

    /** name of the instance */
    string name_;

    /** demands of customers */
    vector<int> demands_;

    /** position, service time and start/end of TW of customers */
    vector<int> service_time_;
    vector<int> x_coord_;
    vector<int> y_coord_;
    vector<double> start_TW_;
    vector<double> end_TW_;

    /** this amounts to end_TW + service time */
    vector<double> latest_departure_possible_;

    /** the true distance from two customers */
    vector<vector<double>> distances_;

    /** the distances including the service time */
    vector<vector<double>> distances_service_;

    /** preliminary elimination of arcs that are excluded by time constraints */
    vector< vector<bool> > possible_arcs_;

    /** maximum customers in a route (based on capacity) */
    int max_routes_stops_;

};

#endif //LOCALSEARCH_PARSEDATA_H
