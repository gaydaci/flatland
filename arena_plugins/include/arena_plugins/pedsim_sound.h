#include <flatland_server/model_plugin.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <yaml-cpp/yaml.h>
#include <flatland_server/timekeeper.h>

#include <Box2D/Box2D.h>

#include <arena_sound_srvs/PrepareSource.h>
#include <arena_sound_srvs/PlaySource.h>
#include <arena_sound_srvs/UpdateSourcePos.h>
#include <arena_sound_srvs/SourceStopped.h>

#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>

#ifndef FLATLAND_PLUGINS_PEDSIM_SOUND_H
#define FLATLAND_PLUGINS_PEDSIM_SOUND_H

using namespace flatland_server;

namespace flatland_plugins {

class PedsimSound : public ModelPlugin {
  // Source body moving emmiting sounds
  flatland_server::Body *body;

  ros::ServiceClient prepare_source_client_;
  ros::ServiceClient update_source_position_client_;
  ros::ServiceClient play_source_client_;
  ros::ServiceClient source_stopped_client_;

  int source_id;

  bool source_stopped;

  bool agentCallbackReceived;

  bool started_playing;

  std::string curr_social_state;
  std::string prev_social_state;

  pedsim_msgs::AgentState agent;       ///< Pedsim Agent corresponding to source
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