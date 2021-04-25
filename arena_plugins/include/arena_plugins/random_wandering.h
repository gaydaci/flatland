#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>


#ifndef ARENA_PLUGINS_RANDOM_WANDERING_H
#define ARENA_PLUGINS_RANDOM_WANDERING_H
using namespace flatland_server;
namespace flatland_plugins {

class RandomWandering : public ModelPlugin {
 public:
  typedef enum {
    None = 0,
    Forward = 1,
    Turning = 2
  } WandererState;

  WandererState state;
  Body* body;
  float configuredLinearVelocity;
  b2Vec2 currentLinearVelocity;
  float configuredAngularVelocity;
  float targetAngularVelocity;
  float angleGoal;
  ros::Subscriber laserSub;
  sensor_msgs::LaserScanConstPtr laserScan;
  bool obstacleNear;

  void OnInitialize(const YAML::Node& config) override;
  void BeforePhysicsStep(const Timekeeper& Timekeeper) override;
  void LaserCallback(const sensor_msgs::LaserScanConstPtr& laser_scan);
  bool ObstacleNear();
  void DoStateTransition();
  void ActivateState(WandererState state_in);
  void DeactivateState(WandererState state_in);
  int GetTurnDirection(float angle, float angle_goal);
  float RandomRange(const float range_lo, const float range_hi);

};

};

#endif