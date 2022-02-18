#include <arena_plugins/sound.h>
#include <pluginlib/class_list_macros.h>
#include <flatland_server/yaml_reader.h>
#include <ros/ros.h>
#include <ros/node_handle.h>

namespace flatland_plugins {


void SoundPlugin::OnInitialize(const YAML::Node &config) {
  flatland_server::YamlReader reader(config);

  body = GetModel()->GetBody(reader.Get<std::string>("body"));
  
  if (!nh_.ok())
    throw ros::Exception("ROS Node Handle error");
  // check a valid body is given
  if (body == nullptr) {
    throw flatland_server::YAMLException("Body does not exist");
  } else {
    ROS_INFO("SoundPlugin::OnInitialize: source_body.name_ %s\n",
                          body->GetName().c_str());
  }
  
  float pos_x = body->GetPhysicsBody()->GetPosition().x;
  float pos_y = body->GetPhysicsBody()->GetPosition().y;

  update_listener_position_client_ = nh_.serviceClient<arena_sound_srvs::UpdateListenerPos>("update_listener_pos");
  arena_sound_srvs::UpdateListenerPos srv;
  srv.request.pos_x = pos_x;
  srv.request.pos_y = pos_y;
  
  ros::service::waitForService("update_listener_pos", 1000);
  if (update_listener_position_client_.call(srv)) {
    ROS_INFO("Listener's position updated");
  } else {
    ROS_ERROR("Failed to call service update_listener_pos");
  }

  agents_ = NULL;
  pedsim_agents_sub_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &SoundPlugin::pedAgentsCallback, this);
  
}

void SoundPlugin::BeforePhysicsStep(const Timekeeper &timekeeper) {
  
  float pos_x = body->GetPhysicsBody()->GetPosition().x;
  float pos_y = body->GetPhysicsBody()->GetPosition().y;
  // ROS_INFO("SoundPlugin::BeforePhysicsStep\n Listener Body's position is (%f, %f)\n", pos_x, pos_y);
  
  update_listener_position_client_ = nh_.serviceClient<arena_sound_srvs::UpdateListenerPos>("update_listener_pos");
  arena_sound_srvs::UpdateListenerPos srv;
  srv.request.pos_x = pos_x;
  srv.request.pos_y = pos_y;
  
  while (!update_listener_position_client_.isValid()) {
    ROS_WARN("Reconnecting update_listener_position_client-server...");
    update_listener_position_client_.waitForExistence(ros::Duration(1.0));
    update_listener_position_client_ = nh_.serviceClient<arena_sound_srvs::UpdateListenerPos>("update_listener_pos");
  }

  update_listener_position_client_.call(srv);

  if (srv.response.success) {
    // ROS_INFO("Listener's position updated");
  } else {
    ROS_ERROR("Failed to call service update_listener_pos");
  }  
}

void SoundPlugin::pedAgentsCallback(const pedsim_msgs::AgentStatesConstPtr& agents) {
  agents_ = agents;
  
  for (int i = 0; i < agents->agent_states.size(); i++) {
    pedsim_msgs::AgentState p = agents_->agent_states[i];
    // ROS_INFO("SoundPlugin::pedAgentsCallback: Pedsim agent with id: %ld, state: %s and pos: %f, %f",
    //                                     p.id, p.social_state.c_str(), p.pose.position.x,p.pose.position.y);
    if (p.social_state != "Walking" && p.social_state != "Running" && p.social_state != "Waiting"
          && p.social_state != "Driving") {
      ROS_INFO("SoundPlugin::pedAgentsCallback: Pedsim agent with id: %ld is %s!",
                                                  p.id, p.social_state.c_str());
    }
  }
}
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::SoundPlugin, flatland_server::ModelPlugin)