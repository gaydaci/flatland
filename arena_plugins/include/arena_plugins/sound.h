#include <flatland_server/model_plugin.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <yaml-cpp/yaml.h>
#include <flatland_server/timekeeper.h>

#include <Box2D/Box2D.h>

#include <arena_sound_srvs/UpdateListenerPos.h>

#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>

#ifndef FLATLAND_PLUGINS_SOUND_H
#define FLATLAND_PLUGINS_SOUND_H


using namespace flatland_server;

namespace flatland_plugins {

class SoundPlugin : public ModelPlugin {
  // Listener body perceiving sounds
  flatland_server::Body *body;

  ros::ServiceClient update_listener_position_client_;


  pedsim_msgs::AgentStatesConstPtr agents_;///< most recent pedsim agent state
  ros::Subscriber pedsim_agents_sub_;     ///< Subscriber to pedsim agents state

  // This function must be overridden for initialization. The YAML Node is
  // passed in for processing by the specific plugin
  void OnInitialize(const YAML::Node &config) override;

  //These functions is called before  the physics step
  void BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) override;

  void pedAgentsCallback(const pedsim_msgs::AgentStatesConstPtr& agents);

};
}

#endif