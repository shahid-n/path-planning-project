#include "rdObject.h"
#include "math.h"
#include "configParams.h"
#include "helpers.h"

rdObject::rdObject() {}

rdObject::rdObject(json & j, bool isEgo)
{
  if(isEgo) {
    x = j[1]["x"];
    y = j[1]["y"];
    s = j[1]["s"];
    d = j[1]["d"];
    yaw = j[1]["yaw"];
    yaw_rad = deg2rad(yaw);
    speed = miPerHr_to_mPerSec(j[1]["speed"]);
  }
  else {
    id = j[0];
    x = j[1];
    y = j[2];
    vx = j[3];
    vy = j[4];
    s = j[5];
    d = j[6];

//    lane = (int)(d/LANE_WIDTH);

    yaw = 0.0;
    yaw_rad = 0.0;

    // TO DO: deal with cases where object does not follow s (i.e., keeping in lane), but switches or cuts across lanes

    speed = sqrt(vx*vx + vy*vy);
  }

  lane = (int)(d/LANE_WIDTH);
}
