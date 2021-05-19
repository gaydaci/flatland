 /*
 * @name	 	pedsim_movement.cpp
 * @brief	 	The movement of the pedsim agents is as well applied to the flatland models.
 *              Furthermore, a walking pattern is added.
 * @author  	Ronja Gueldenring
 * @date 		2019/04/05
 **/

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/body.h>
#include <flatland_server/types.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <arena_plugins/triangle_profile.h>
#include <cmath> 


#ifndef FLATLAND_PLUGINS_VEHICLE_MOVEMENT_H
#define FLATLAND_PLUGINS_VEHICLE_MOVEMENT_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * @class VehicleMovement
 * @brief The movement of the pedsim agents is as well applied to the flatland models.
 */
class VehicleMovement : public ModelPlugin {
 public:
  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

   /**
    * @name          AfterPhysicsStep
    * @brief         override the AfterPhysicsStep method
    * @param[in] timekeeper Object managing the simulation time
    */
  void AfterPhysicsStep(const Timekeeper& timekeeper) override;


  private: 
    b2Body * body_;                            ///< Pointer to base-body
    UpdateTimer update_timer_;              ///< for controlling update rate

    b2Body * safety_dist_b2body_;               ///< Pointer to safety distance circle
    Body * safety_dist_body_;               ///< Pointer to safety distance circle
    pedsim_msgs::AgentState person;
    ros::Subscriber pedsim_agents_sub_;        ///< Subscriber to pedsim agents state
    ros::Publisher agent_state_pub_;          ///< Publisher for agent state of  every pedsim agent
    
    tf::TransformListener listener_;           ///< Transform Listner
    pedsim_msgs::AgentStatesConstPtr agents_;  ///< most recent pedsim agent state
   
    double safety_dist_;
    double safety_dist_original_;
    std::string body_frame_;                   ///< frame name of base-body
   
    /**
     * @brief Callback for pedsim agent topic
     * @param[in] agents array of all agents
     */
    void agentCallback(const pedsim_msgs::AgentStatesConstPtr& agents);

    /**
     * @brief Method is copy of model_body.cpp to be able to change radius programatically
     * ToDo: Find more elegeant solution!
     */
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
