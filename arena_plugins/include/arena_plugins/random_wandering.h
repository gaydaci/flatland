#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
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
  ros::Publisher robo_obstacle_pub_;          ///< Publisher for agent state 
    
  visualization_msgs::Marker  robo_obstacle ;
  visualization_msgs::MarkerArray  robo_obstacles  ;
  sensor_msgs::LaserScanConstPtr laserScan;
  bool obstacleNear;
  b2Body * body_;                            ///< Pointer to base-body
  UpdateTimer update_timer_;              ///< for controlling update rate

  b2Body * safety_dist_b2body_;               ///< Pointer to safety distance circle
  Body * safety_dist_body_;     

  double safety_dist_;
  double safety_dist_original_;
  void OnInitialize(const YAML::Node& config) override;
  void BeforePhysicsStep(const Timekeeper& Timekeeper) override;
  void LaserCallback(const sensor_msgs::LaserScanConstPtr& laser_scan);
  bool ObstacleNear();
  void DoStateTransition();
  void ActivateState(WandererState state_in);
  void DeactivateState(WandererState state_in);
  int GetTurnDirection(float angle, float angle_goal);
  float RandomRange(const float range_lo, const float range_hi);
  
    /**
  * @name          AfterPhysicsStep
  * @brief         override the AfterPhysicsStep method
  * @param[in] timekeeper Object managing the simulation time
  */
  void AfterPhysicsStep(const Timekeeper& timekeeper) override;


  void set_safety_dist_footprint(b2Body * physics_body_, double radius);

  /**
    * @brief To be able to change radius programatically
  */
  void ConfigFootprintDefSafetyDist(b2FixtureDef &fixture_def); 
  /**
  * @brief update safety distance circle, when the agent is chatting.
  * Body Footprint of safety dist circle will be set.
  */
  void updateSafetyDistance();


};

};

#endif