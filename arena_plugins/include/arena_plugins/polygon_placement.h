 /*
 * @name	 	pedsim_movement.cpp
 * @brief	 	The movement of the pedsim agents is as well applied to the flatland models.
 *              Furthermore, a walking pattern is added.
 * @author  	Ronja Gueldenring
 * @date 		2019/04/05
 **/

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/body.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <flatland_msgs/DangerZone.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <arena_plugins/triangle_profile.h>
#include<cmath>
#include <algorithm>

#ifndef FLATLAND_PLUGINS_POLYGON_PLACEMENT_H
#define FLATLAND_PLUGINS_POLYGON_PLACEMENT_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * @class PolygonPlacement
 * @brief The movement of the pedsim agents is as well applied to the flatland models.
 * Furthermore a walking pattern is added
 * 
 */
class PolygonPlacement : public ModelPlugin {
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
   * @param[in]     config The plugin YAML node
   */

  void AfterPhysicsStep(const Timekeeper& timekeeper) override;



  private: 

    UpdateTimer update_timer_;              ///< for controlling update rate
    UpdateTimer leg_timer_;                 ///< for controlling step size
    
    b2Body * body_;                         ///< Pointer to base-body

    pedsim_msgs::AgentState person;
    ros::Subscriber pedsim_agents_sub_;     ///< Subscriber to pedsim agents state
    bool rotate;

    // Recent agent state
    pedsim_msgs::AgentStatesConstPtr agents_;///< most recent pedsim agent state
    // std::queue<pedsim_msgs::AgentStatesConstPtr> q_agents_;

    /**
     * @brief Callback for pedsim agent topic
     * @param[in] agents array of all agents
     */
    void agentCallback(const pedsim_msgs::AgentStatesConstPtr& agents);
};
};

#endif
