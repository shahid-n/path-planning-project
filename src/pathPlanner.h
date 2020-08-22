#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <math.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "rdObject.h"
#include "spline.h"
#include "configParams.h"
#include "helpers.h"

using namespace std;

class pathPlanner
{
  public:
    pathPlanner(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);

    rdObject ego;
    vector<rdObject> rdObjects;
    int lane = 1;
    double ref_velocity = 0;
    json previous_path_x;
    json previous_path_y;
    double end_path_s = 0.0;
    double end_path_d = 0.0;
    vector<double> map_waypoints_x_;
    vector<double> map_waypoints_y_;
    vector<double> map_waypoints_s_;
    vector<double> map_waypoints_dx_;
    vector<double> map_waypoints_dy_;

    void updateSensorFusion(json data);
    double laneSpeed(int lane);
    int fastestLane();
    double safetyDistance(double speed_mps);
    json path();
    double centerLaneD(int lane);
    double safetyCosts(int lane);
    double wrappedDistance(double s1, double s2);

  protected:

  private:
};

#endif // PATHPLANNER_H
