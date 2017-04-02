//
// Created by Bertoli, Francesco (Data61, Canberra City) on 09/02/17.
//

#include "crossReverseMove.h"
#include "data.h"



/** ------------------------------------------------------------------------------------------------ */



CrossReverseMove& CrossReverseMove::operator= (const CrossReverseMove &other_move) {
    i_ = other_move.i_;
    j_ = other_move.j_;
    k_ = other_move.k_;
    l_ = other_move.l_;
    route_1_ = other_move.route_1_;
    route_2_ = other_move.route_2_;
    reverse_path_1_ = other_move.reverse_path_1_;
    reverse_path_2_ = other_move.reverse_path_2_;
    delta_distance_ = other_move.delta_distance_;
    delta_capacity_ = other_move.delta_capacity_;
    route_removed_ = other_move.route_removed_;
    return *this;
}


/** ------------------------------------------------------------------------------------------------ */


void CrossReverseMove::setMoveVariables(int i, int j, int k, int l, int route_1, int route_2, bool reverse_path_1, bool reverse_path_2)
{
    i_ = i;
    j_ = j;
    k_ = k;
    l_ = l;
    route_1_ = route_1;
    route_2_ = route_2;
    reverse_path_1_ = reverse_path_1;
    reverse_path_2_ = reverse_path_2;
}


/** ------------------------------------------------------------------------------------------------ */

void CrossReverseMove::performMove(Solution &solution) {
    // PERFORM THE MOVE
    if (i_ == j_)
        solution.swapReversePaths(route_1_, route_2_, i_, i_ - 1, k_ + 1, l_, reverse_path_1_, reverse_path_2_);
    else if (k_ == l_)
        solution.swapReversePaths(route_1_, route_2_, i_ + 1, j_, k_, k_ - 1, reverse_path_1_, reverse_path_2_);
    else
        solution.swapReversePaths(route_1_, route_2_, i_ + 1, j_, k_ + 1, l_, reverse_path_1_, reverse_path_2_);
}


/** ------------------------------------------------------------------------------------------------ */

void CrossReverseMove::saveModifiedRoutesIndexes(Solution &solution, vector<int> &routes) {
    // if route 1 is not deleted
    if (k_ < l_ or (i_ > solution.start_positions_[route_1_]) or (j_ < solution.start_positions_[route_1_+1]-1)) {
        routes.push_back(route_1_);
        // if route 2 is not deleted
        if (i_ < j_ or (k_ > solution.start_positions_[route_2_]) or (l_ < solution.start_positions_[route_2_+1]-1))
            routes.push_back(route_2_);
    }
    else
        routes.push_back(route_2_ - 1);
}


/** ------------------------------------------------------------------------------------------------ */


void CrossReverseMove::identifyRemovedArcs(Solution &solution, vector<pair<int,int>> &removed_arcs) {
    removed_arcs.clear();

    removed_arcs.push_back(make_pair(solution.tour_[i_], solution.tour_[i_ + 1]));
    if (i_ < j_)
        removed_arcs.push_back(make_pair(solution.tour_[j_], solution.tour_[j_ + 1]));

    removed_arcs.push_back(make_pair(solution.tour_[k_], solution.tour_[k_ + 1]));
    if (k_ < l_)
        removed_arcs.push_back(make_pair(solution.tour_[l_], solution.tour_[l_ + 1]));

}


/** ------------------------------------------------------------------------------------------------ */


void CrossReverseMove::identifyAddedArcs(Solution &solution, vector<pair<int,int>> &new_arcs) {
    new_arcs.clear();
    if (k_ < l_) {
        if (not reverse_path_2_) {
            new_arcs.push_back(make_pair(solution.tour_[i_], solution.tour_[k_ + 1]));
            new_arcs.push_back(make_pair(solution.tour_[l_], solution.tour_[j_ + 1]));
        }
        else {
            new_arcs.push_back(make_pair(solution.tour_[i_], solution.tour_[l_]));
            new_arcs.push_back(make_pair(solution.tour_[k_ + 1], solution.tour_[j_ + 1]));
        }
    }

    // add arcs of path 1
    if (i_ < j_) {
        if (not reverse_path_1_) {
            new_arcs.push_back(make_pair(solution.tour_[k_], solution.tour_[i_ + 1]));
            new_arcs.push_back(make_pair(solution.tour_[j_], solution.tour_[l_ + 1]));
        }
        else {
            new_arcs.push_back(make_pair(solution.tour_[k_], solution.tour_[j_]));
            new_arcs.push_back(make_pair(solution.tour_[i_ + 1], solution.tour_[l_ + 1]));
        }
    }
}

/** ------------------------------------------------------------------------------------------------ */


//void CrossReverseMove::addArc(pair<int,int> arc) {
//    if ((*forbidden_arcs_)[arc.first][arc.second] > 0)
//        new_tabu_arcs_.push_back(arc);
//    else
//        new_arcs_.push_back(arc);
//}


/** ------------------------------------------------------------------------------------------------ */
//
//
//void CrossReverseMove::updateForbiddenArcs(Solution &solution, int tabu_duration) {
//    new_arcs_.clear();
//    new_tabu_arcs_.clear();
//
//    // add arcs of path 2
//    pair<int, int> arc;
//    if (k_ < l_) {
//        if (not reverse_path_2_) {
//            addArc(make_pair(solution.tour_[i_], solution.tour_[k_ + 1]));
//            addArc(make_pair(solution.tour_[l_], solution.tour_[j_ + 1]));
//        }
//        else {
//            addArc(make_pair(solution.tour_[i_], solution.tour_[l_]));
//            addArc(make_pair(solution.tour_[k_ + 1], solution.tour_[j_ + 1]));
//        }
//    }
//
//    // add arcs of path 1
//    if (i_ < j_) {
//        if (not reverse_path_1_) {
//            addArc(make_pair(solution.tour_[k_], solution.tour_[i_ + 1]));
//            addArc(make_pair(solution.tour_[j_], solution.tour_[l_ + 1]));
//        }
//        else {
//            addArc(make_pair(solution.tour_[k_], solution.tour_[j_]));
//            addArc(make_pair(solution.tour_[i_ + 1], solution.tour_[l_ + 1]));
//        }
//    }
//
//
//    // update status of new_tabu_arcs_
//    for (auto &arc : new_tabu_arcs_)
//        (*forbidden_arcs_)[arc.first][arc.second] = tabu_duration + 1;
//
//
//    // update tabu status of arcs in forbidden_list_
//    for (vector<pair<int,int>>::iterator iter = (*forbidden_list_).begin(); iter != (*forbidden_list_).end();) {
//        if ((*forbidden_arcs_)[(*iter).first][(*iter).second] > 0) {
//            --(*forbidden_arcs_)[(*iter).first][(*iter).second];
//            ++iter;
//        }
//        else
//            iter = (*forbidden_list_).erase(iter);
//    }
//
//
//    // add new arcs
//    for (auto &el : new_arcs_) {
//        (*forbidden_arcs_)[el.first][el.second] = tabu_duration;
//        (*forbidden_list_).push_back(el);
//    }
//
//}
//
