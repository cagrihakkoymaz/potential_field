
#ifndef POTENTIAL_FIELD_INCLUDE_GUARD_HPP
#define POTENTIAL_FIELD_INCLUDE_GUARD_HPP
using namespace std;

#include <vector>

/*!
 *  planner  class
 */
namespace global_planner
{
    class planner
    {
    public:
        // starting and goal points of planner generic class
        // initilized as -1 for checking umature initilization
        float tf_goal_x = -1;
        float tf_goal_y = -1;
        float tf_start_x = -1;
        float tf_start_y = -1;

        // map values initilized for adapt planner to  map
        float origin_x;
        float origin_y;
        int map_height;
        int map_width;
        float map_resolution;
    };

    class PFP : public planner // PFP:Potential Field Planner
    {
    public:
        // Construct potential_field planner with parameters
        PFP(const double &ETA_, const double &KP_, const double distance_treshold_);
        ~PFP();

        // Vector used for represent occupancy gridmap cells

        vector<vector<float>> force_attraction;
        vector<vector<float>> force_repulsion;
        vector<vector<float>> force_total;
        vector<vector<int>> obstacles;
        vector<vector<int>> path;

        // Functions used for calculate forces and plan the path

        vector<vector<float>> TFCalculator(vector<vector<float>> &AF_vector, vector<vector<float>> &RF_vector); // Total Force Calculator
        vector<vector<float>> AFCalculator(float tf_goal_x, float tf_goal_y);                                   // Attraction Force Calculator
        vector<vector<float>> RFCalculator(vector<vector<int>> &obs_vector);                                    // Repulsion Force Calculator
        void Path_Genrator(float start_x, float start_y, vector<vector<float>> &total_force);                   // Generate part recursively
                                                                                                                // protected instead of private so that child Class can access
    private:
        // For adjust repulsive and attractive forces impact to total force
        double KP;                // Attractive potential gain
        double ETA;               // Repulsive potential gain
        double distance_treshold; // Treshold coralted with robot size choosed dummy number for this purpose
    };

#endif
}