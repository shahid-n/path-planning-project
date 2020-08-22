#include "pathPlanner.h"
#include "configParams.h"
#include "algorithm"
#include "helpers.h"

pathPlanner::pathPlanner(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                         vector<double> map_waypoints_dx, vector<double> map_waypoints_dy)
{
  map_waypoints_x_ = map_waypoints_x;
  map_waypoints_y_ = map_waypoints_y;
  map_waypoints_s_ = map_waypoints_s;
  map_waypoints_dx_ = map_waypoints_dx;
  map_waypoints_dy_ = map_waypoints_dy;
}

void pathPlanner::updateSensorFusion(json data) {
  // Initialise ego vehicle as rdObject with isEgo == true
  ego = rdObject(data, true);

  // Sensor Fusion Data: a list of all other detected objects on the same side of the road.
  rdObjects.clear();
  json sensor_fusion = data[1]["sensor_fusion"];
  for (json sensor_fusion_obj : sensor_fusion) {
    rdObjects.push_back(rdObject(sensor_fusion_obj, false));
  }

  // Previous path data given to the Planner
  previous_path_x = data[1]["previous_path_x"];
  previous_path_y = data[1]["previous_path_y"];
  // Previous path's end s and d values
  end_path_s = data[1]["end_path_s"];
  end_path_d = data[1]["end_path_d"];
}

double pathPlanner::laneSpeed(int lane) {
  double lane_speed = miPerHr_to_mPerSec(MAX_SPEED_MPH);
  for (rdObject rdObj : rdObjects) {
    if (rdObj.lane == lane) {
      double distance_to_obj_ahead = wrappedDistance(ego.s, rdObj.s);
      double distance_to_obj_behind = wrappedDistance(rdObj.s, ego.s);
      if (((distance_to_obj_ahead < FRONT_SENSOR_RANGE_M) || (distance_to_obj_behind < 10))
          && (rdObj.speed < lane_speed)) {
        lane_speed = rdObj.speed;
      }
    }
  }
  return lane_speed;
}

double pathPlanner::safetyCosts(int lane) {
  // Find moving objects in the lane that might cause trouble
  // Are objects within the safety distance buffer in front of or behind us?
  double safety_costs = 0.0;
  for (rdObject rdObj : rdObjects) {
    if (rdObj.lane == lane) {
      if ((wrappedDistance(rdObj.s, ego.s) < safetyDistance(rdObj.speed)) /* ego vehicle in front of object */
          || (wrappedDistance(ego.s, rdObj.s) < safetyDistance(ego.speed)) /* object in front of ego vehicle */) {
        safety_costs += 1.0;
      }
    }
  }
  return safety_costs;
}

// Figure out the fastest lane of the current and the direct neighbours
int pathPlanner::fastestLane() {
  int fastest_lane = ego.lane;
  double fastest_speed = laneSpeed(fastest_lane);

  for (int lane = 0; lane < NUMBER_OF_LANES; ++lane) {
    double lane_speed = laneSpeed(lane);
    if ((lane_speed > fastest_speed) || ((lane_speed == fastest_speed)
                                         && (fabs(lane - ego.lane) < fabs(fastest_lane - ego.lane)))) {
      fastest_speed = lane_speed;
      fastest_lane = lane;
    }
  }
  return fastest_lane;
}

double pathPlanner::centerLaneD(int lane) {
  return LANE_WIDTH*(0.5 + lane);
}

double pathPlanner::safetyDistance(double speed_mps) {
  // see http://www.softschools.com/formulas/physics/stopping_distance_formula/89/
  double reaction_distance = speed_mps*REACTION_TIME_S;
  double brake_distance = speed_mps*speed_mps
      /(2*CAR_COEFFICIENT_OF_FRICION*CAR_ACCELERATION_DUE_TO_GRAVITY_MPS2);
  return reaction_distance + brake_distance;
}

double pathPlanner::wrappedDistance(double back_s, double front_s) {
  double distance = (front_s - back_s + TRACK_LENGTH_M) - TRACK_LENGTH_M;

  if (distance < 0) {
    distance = TRACK_LENGTH_M + distance;
  }
  return distance;
}

json pathPlanner::path() {

  // define the actual (x,y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  int prev_size = previous_path_x.size();

  double obj_s = ego.s;

  if (prev_size > 0) {
    obj_s = end_path_s;
  }

  bool too_close = false;

  for (rdObject rdObj : rdObjects) {
    if ((rdObj.lane == ego.lane) || (rdObj.lane == lane)) { // check the current lane and the destination lane
      double check_speed = rdObj.speed;
      double check_obj_s = rdObj.s;

      check_obj_s += ((double) prev_size*TICK_S*check_speed);

      double wrapped_distance = wrappedDistance(obj_s, check_obj_s);
      if (wrapped_distance < safetyDistance(ego.speed)) {
        int fastest_lane = fastestLane();
        if (lane == ego.lane) {
          if (fastest_lane > lane) {
            // Switch lane if change is safe
            if (safetyCosts(lane + 1) < 0.2) {
              lane += 1;
            }
            else {
              cout  << "Changing to " << fastest_lane << ". Waiting for object to clear safety range in lane "
                    << lane + 1 << endl;
            }
          }
          else if (fastest_lane < lane) {
            // Switch lane if change is safe
            if (safetyCosts(lane - 1) < 0.2) {
              lane -= 1;
            }
            else {
              cout  << "Changing to " << fastest_lane << ". Waiting for object to clear safety range in lane "
                    << lane - 1 << endl;
            }
          }
        }

        // reduce speed if lane change is not possible
        if (lane == ego.lane) {
          too_close = true;
        }
      }
    }
  }

  if (too_close) {
    ref_velocity -= miPerHr_to_mPerSec(1.0);
  }
  else if (ref_velocity < miPerHr_to_mPerSec(MAX_SPEED_MPH)) {
    ref_velocity += miPerHr_to_mPerSec(0.5);
  }

  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = ego.yaw_rad;

  if (prev_size < 2) {
    double prev_obj_x = ego.x - cos(ego.yaw);
    double prev_obj_y = ego.y - sin(ego.yaw);

    ptsx.push_back(prev_obj_x);
    ptsx.push_back(ego.x);

    ptsy.push_back(prev_obj_y);
    ptsy.push_back(ego.y);
  }
  else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  double destinationD = centerLaneD(lane);
  vector<double> next_wp0 = getXY(obj_s + 30, destinationD, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_wp1 = getXY(obj_s + 60, destinationD, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_wp2 = getXY(obj_s + 90, destinationD, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_wp3 = getXY(obj_s + 120, destinationD, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  ptsx.push_back(next_wp3[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  ptsy.push_back(next_wp3[1]);

  for (int i = 0; i < ptsx.size(); ++i) {
    // Shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw);
    ptsy[i] = shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw);
  }

  // Create a spline
  tk::spline s;

  // Set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  // Start with the previous path points from last time
  for (int i = 0; i < previous_path_x.size(); ++i) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 40.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_offset = 0;

  // Fill up the rest of our path planner after filling it with previous points; here we will always output 50 points

  for (int i = 1; i <= NUMBER_OF_PATH_POINTS - previous_path_x.size(); ++i) {
//  Calculate a new intermediate waypoint every 0.02 seconds (TICK_S)
    double N = target_dist/(TICK_S*ref_velocity);
    double x_point = x_offset + target_x/N;
    double y_point = s(x_point);
    x_offset = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotating back to normal after rotating it earlier
    x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
    y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  json msgJson;
  msgJson["next_x"] = next_x_vals;
  msgJson["next_y"] = next_y_vals;

  return msgJson;
}
