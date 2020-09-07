#ifndef CONFIGPARAMS_H_
#define CONFIGPARAMS_H_

double const LANE_WIDTH = 4.0;
double const MAX_SPEED_MPH = 49.5;
double const TICK_S = 0.02;
int const NUMBER_OF_PATH_POINTS = 50;

// The max s value before wrapping around the track back to 0
double const TRACK_LENGTH_M = 6945.554;
double const NUMBER_OF_LANES = 3;

double const FRONT_SENSOR_RANGE_M = 100.0;
double const BACK_SENSOR_RANGE_M = 100.0;
double const MAX_ACCELERATION_MPS2 = 10.0;
double const MAX_JERK_MPS3 = 10.0;

double const MAX_COMFORTABLE_ACCELERATION_MPS2 = 5.0;
double const MAX_COMFORTABLE_JERK_MPS3 = 5.0;
double const REACTION_TIME_S = 0.0;

double const CAR_COEFFICIENT_OF_FRICION = 0.6; // dry road
double const CAR_ACCELERATION_DUE_TO_GRAVITY_MPS2 = 9.80;

#endif /* CONFIGPARAMS_H_ */
