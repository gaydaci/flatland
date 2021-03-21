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
#include <flatland_server/types.h>
#include <pedsim_msgs/AgentStates.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <arena_plugins/triangle_profile.h>


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



  private: 

    ros::Subscriber pedsim_agents_sub_;        ///< Subscriber to pedsim agents state
    tf::TransformListener listener_;           ///< Transform Listner
    pedsim_msgs::AgentStatesConstPtr agents_;  ///< most recent pedsim agent state
    b2Body * body_;                            ///< Pointer to base-body
    std::string body_frame_;                   ///< frame name of base-body

    /**
     * @brief Callback for pedsim agent topic
     * @param[in] agents array of all agents
     */
    void agentCallback(const pedsim_msgs::AgentStatesConstPtr& agents);
    
};
};

#endif
