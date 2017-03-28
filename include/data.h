//
// Created by Bertoli, Francesco (Data61, Canberra City) on 01/02/17.
//

#ifndef LOCALSEARCH_PARSEDATA_H
#define LOCALSEARCH_PARSEDATA_H
#include <string>
#include <vector>

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
    int n_requests_;
    int max_vehicles_;
    int capacity_;
    int depot_;

    /** name of the instance */
    string name_;

    vector<int> demands_;
    vector<double> start_TW_;
    vector<double> end_TW_;

    /** this amounts to end_TW + service time */
    vector<double> latest_departure_possible_;
    vector<int> service_time_;
    vector<int> x_coord_;
    vector<int> y_coord_;

    /** the true distance from two customers */
    vector<vector<double>> true_distances_;

    /** the distances including the dual variables */
    vector<vector<double>> modified_distances_;

    /** preliminary elimination of arcs that are excludede by time constraints */
    vector< vector<bool> > possible_arcs_;

};

#endif //LOCALSEARCH_PARSEDATA_H
