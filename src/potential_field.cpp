#include "../include/potential_field/potential_field.hpp"

#include <vector>
#include <cmath>
#include <sstream>
#include <iostream>
#include <limits>
using namespace std;

namespace global_planner
{

    // construct potential_field planner with parameters
    PFP::PFP(const double &ETA_, const double &KP_, const double distance_treshold_)
    {
        ETA = ETA_;
        KP = KP_;
        distance_treshold = distance_treshold_;
    }

    PFP::~PFP()
    {
    }

    // Total Force Calculator
    vector<vector<float>>
    PFP::TFCalculator(vector<vector<float>> &AF_vector, vector<vector<float>> &RF_vector)
    {
        vector<vector<float>> temp_force_total(map_height, vector<float>(map_width, 0));

        for (int i = 0; i < map_height; i++)
        {
            for (int j = 0; j < map_width; j++)
            {

                {
                    temp_force_total[i][j] = AF_vector[i][j] + RF_vector[i][j];
                }
            }
        }
        return temp_force_total;
    }

    // Attraction Force Calculator
    vector<vector<float>> PFP::AFCalculator(float tf_goal_x, float tf_goal_y)
    {
        vector<vector<float>> temp_force_attraction(map_height, vector<float>(map_width, 0));

        for (int i = 0; i < map_height; i++)
        {
            for (int j = 0; j < map_width; j++)
            {

                {
                    // Basic 	hypotenuse used for calculate cost of distance
                    temp_force_attraction[i][j] = KP * (sqrt(pow((tf_goal_x - (i + 0.5)), 2) + pow((tf_goal_y - (j + 0.5)), 2)));
                }
            }
        }
        return temp_force_attraction;
    }
    // Repulsion Force Calculator
    vector<vector<float>> PFP::RFCalculator(vector<vector<int>> &obs_vector)
    {
        vector<vector<float>> temp_force_repulsion(map_height, vector<float>(map_width, 0));
        float dist;
        float min_distance = numeric_limits<float>::max();
        int min_obs_index;

        for (int i = 0; i < map_height; i++)
        {
            for (int j = 0; j < map_width; j++)
            {

                {
                    min_distance = numeric_limits<float>::max();
                    for (int k = 0; k < obstacles.size(); k++)
                    {
                        // Find closest obstacle for calculate repulsive
                        dist = sqrt(pow((i - obstacles[k][0]), 2) + pow((j - obstacles[k][1]), 2));
                        if (dist < min_distance)
                        {
                            min_distance = dist;
                            min_obs_index = k;
                        }
                    }
                    if (min_distance > distance_treshold)

                    // If obstacle too far dont added for avoid create local minimum

                    {
                        temp_force_repulsion[i][j] = 0;
                    }
                    else
                    {

                        temp_force_repulsion[i][j] = ETA * (pow(((1 / min_distance)), 3));
                    }
                }
            }
        }

        return temp_force_repulsion;
    }

    void PFP::Path_Genrator(float start_x, float start_y, vector<vector<float>> &total_force) // Generate part recursively

    {

        if (path.size() > 2)
        {
            // Trap for ossilication
            if ((path[path.size() - 1][0] == path[path.size() - 3][0]) && (path[path.size() - 1][1] == path[path.size() - 3][1]))
            {
                cout << "LOCAL MINIMUM -TRAPPED" << endl;
                return;
            }
        }

        // Trap for uninitilized start
        if ((tf_goal_x == -1 || start_x == -1) || (tf_goal_y == -1 || start_y == -1))
        {
            cout << "START or GOAL UNITIALIZED" << endl;
            return;
        }
        // Finisihing condition for recursion
        // Is goal reached
        if (!((floor(tf_goal_x) == start_x) && (floor(tf_goal_y) == start_y)))
        {
            int x_start_index = floor(start_x);
            int y_start_index = floor(start_y);
            int min_x = x_start_index - 1;
            int min_y = y_start_index - 1;
            float min = total_force[x_start_index - 1][y_start_index - 1];

            for (int i = x_start_index - 1; i <= x_start_index + 1; i++)
            {
                for (int j = y_start_index - 1; j <= y_start_index + 1; j++)
                {

                    if (!(i == x_start_index && j == y_start_index))
                    {

                        if (min > total_force[i][j])
                        {
                            min_x = i;
                            min_y = j;
                            min = total_force[i][j];
                        }
                    }
                }
            }

            // New step added to path
            path.push_back({min_x, min_y});

            // Next step calculated from last step

            PFP::Path_Genrator(min_x, min_y, total_force);
        }

        else
        {
            // Reached the goal without trapped local minimum
            cout << "FULL PATH GENERATED" << endl;
        }
    }
}
