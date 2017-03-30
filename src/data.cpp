//
// Created by Bertoli, Francesco (Data61, Canberra City) on 01/02/17.
//


#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iterator>
#include <cmath>
#include "data.h"

using namespace std;

Data::Data(string problem_file_name) {

    name_ = problem_file_name;
    // READ FILE
    string line;
    ifstream problem_file(problem_file_name);
    vector<vector<string>> lines;
    if (problem_file.is_open()) {
        while (getline(problem_file, line)) {
            istringstream iss(line);
            vector<string> tokens{istream_iterator<string>{iss},
                                  istream_iterator<string>{}};
            lines.push_back(tokens);
        }
        problem_file.close();
    }

    // READ GENERAL INFO
    int i = 0;
    int depot_line;
    while (true) {
        if (lines[i].size() && lines[i][0] == "NUMBER") {
            max_vehicles_ = stringToInt(lines[i + 1][0]);
            capacity_ = stringToInt(lines[i + 1][1]);
        } else if (lines[i].size() && lines[i].back() == "TIME") {
            i += 2;
            depot_line = i++;
            break;
        }
        ++i;
    }


    // READ CUSTOMERS INFO
    x_coord_.clear();
    y_coord_.clear();
    demands_.clear();
    start_TW_.clear();
    end_TW_.clear();
    service_time_.clear();
    latest_departure_possible_.clear();
    distances_.clear();
    distances_service_.clear();
    possible_arcs_.clear();

    for (; i < lines.size(); ++i) {
        if (lines[i].size()) {
            x_coord_.push_back(stringToInt(lines[i][1]));
            y_coord_.push_back(stringToInt(lines[i][2]));
            demands_.push_back(stringToInt(lines[i][3]));
            start_TW_.push_back(stringToDouble(lines[i][4]));
            end_TW_.push_back(stringToDouble(lines[i][5]));
            service_time_.push_back(stringToInt(lines[i][6]));
            latest_departure_possible_.push_back(end_TW_.back() + service_time_.back());
        }
    }

    n_requests_ = x_coord_.size();
    depot_ = n_requests_;

    // MODIFICATION MAX VEHICLES
//    max_vehicles_ = n_requests_;



    // INSERT DEPOTS
    for (i = 0; i < max_vehicles_; ++i){
        x_coord_.push_back(stringToInt(lines[depot_line][1]));
        y_coord_.push_back(stringToInt(lines[depot_line][2]));
        demands_.push_back(stringToInt(lines[depot_line][3]));
        start_TW_.push_back(stringToDouble(lines[depot_line][4]));
        end_TW_.push_back(stringToDouble(lines[depot_line][5]));
        service_time_.push_back(stringToInt(lines[depot_line][6]));
        latest_departure_possible_.push_back(end_TW_.back() + service_time_.back());
    }


    // CREATE MATRIX OF DISTANCES
    possible_arcs_.resize(n_requests_ + max_vehicles_, vector<bool>(n_requests_ + max_vehicles_, false));
    int j;
    for (i = 0; i < n_requests_ + max_vehicles_; ++i){
        vector<double> true_column;
        vector<double> modified_column;
        for (j = 0; j < n_requests_ + max_vehicles_; ++j){
            if (i != j) {
                true_column.push_back(computeDistance(x_coord_[i], y_coord_[i], x_coord_[j], y_coord_[j]));
                modified_column.push_back(true_column.back() + service_time_[i]);
                if ((start_TW_[i] + service_time_[i] + true_column[j]) <= end_TW_[j])
                    possible_arcs_[i][j] = true;
            }
            else {
                true_column.push_back(0);
                modified_column.push_back(0);
            }
        }
        distances_.push_back(true_column);
        distances_service_.push_back(modified_column);
    }


    // COMPUTE OTHER STATISTICS
    vector<int> ordered_demands(demands_.cbegin(), demands_.cbegin()+n_requests_);
    sort (ordered_demands.begin(), ordered_demands.end());
    max_routes_stops_ = 0;
    int load(ordered_demands[0]);
    while (load < capacity_) {
        ++max_routes_stops_;
        load += ordered_demands[max_routes_stops_];
    }

}

int Data::stringToInt(const string &Text) {
    stringstream ss(Text);
    int result;
    return ss >> result ? result : 0;
}

double Data::stringToDouble(const string &Text) {
    stringstream ss(Text);
    double result;
    return ss >> result ? result : 0.0;
}

double Data::computeDistance(int x_1, int y_1, int x_2, int y_2) {
    return sqrt(pow(x_1 - x_2, 2) + pow(y_1 - y_2, 2));
}

