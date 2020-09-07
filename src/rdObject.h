#ifndef RDOBJECT_H
#define RDOBJECT_H

#include "json.hpp"

using json = nlohmann::json;

class rdObject
{
  public:
    rdObject();
    rdObject(json &j, bool isEgo);

    double s;
    double d;
    double vx;
    double vy;
    double x;
    double y;
    double yaw;
    double yaw_rad;
    int id;
    int lane;
    double speed;

  protected:

  private:
};

#endif // RDOBJECT_H
