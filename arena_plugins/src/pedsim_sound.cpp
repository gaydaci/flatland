#include <arena_plugins/pedsim_sound.h>
#include <pluginlib/class_list_macros.h>
#include <flatland_server/yaml_reader.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace flatland_plugins {

void PedsimSound::OnInitialize(const YAML::Node &config) {
  flatland_server::YamlReader reader(config);

  body = GetModel()->GetBody(reader.Get<std::string>("body"));
  
  if (!nh_.ok())
    throw ros::Exception("ROS Node Handle error");
  // check a valid body is given
  if (body == nullptr) {
    throw flatland_server::YAMLException("Body does not exist");
  } else {
    // ROS_INFO("PedsimSound::OnInitialize: source_body.name_ %s\n",
    //                 body->GetName().c_str());
  }
  
  float pos_x = body->GetPhysicsBody()->GetPosition().x;
  float pos_y = body->GetPhysicsBody()->GetPosition().y;

  std::string ns_str = GetModel()->GetNameSpace();
  source_id = std::stoi(ns_str.substr(13, ns_str.length()));
  ROS_INFO("namespace: %s , id: %d", ns_str.c_str(), source_id);

  agentCallbackReceived = false;
  started_playing=false;
  pedsim_agents_sub_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &PedsimSound::pedAgentsCallback, this);

  std::string sound_viz_topic_ = "sound_viz_" + std::to_string(source_id);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(sound_viz_topic_, 1);

  source_gain = reader.Get<float>("gain");
  
  prepare_source_client_ = nh_.serviceClient<arena_sound_srvs::PrepareSource>("prepare_source");
  arena_sound_srvs::PrepareSource srv;
  srv.request.source_id = source_id;
  srv.request.pos_x = pos_x;
  srv.request.pos_y = pos_y;
  srv.request.gain = source_gain; 
  ros::service::waitForService("prepare_source", 1000); 
  if (prepare_source_client_.call(srv))
  {
    // ROS_INFO("Prepared source with id %d", source_id);
  } else {
    ROS_ERROR("Failed to call service prepare_source");
  } 

  if(agentCallbackReceived)
    ROS_INFO("Oninitialize: agents social state: %s", agent.social_state.c_str());  
  
}

void PedsimSound::BeforePhysicsStep(const Timekeeper &timekeeper) {
  
  float pos_x = body->GetPhysicsBody()->GetPosition().x;
  float pos_y = body->GetPhysicsBody()->GetPosition().y;
  // ROS_INFO("SoundPlugin::BeforePhysicsStep\nSource Body's position is (%f, %f)\n", pos_x, pos_y);
  
  update_source_position_client_ = nh_.serviceClient<arena_sound_srvs::UpdateSourcePos>("update_source_pos");
  arena_sound_srvs::UpdateSourcePos update_msg;
  update_msg.request.source_id = source_id;
  update_msg.request.pos_x = pos_x;
  update_msg.request.pos_y = pos_y;

  ros::service::waitForService("update_source_pos", 1000);
  if(update_source_position_client_.call(update_msg)) {
    // ROS_INFO("Source's position with ID %d updated", source_id);
  } else {
    ROS_ERROR("Source %d failed to call service update_source_pos", source_id);
  }
  
  if(agentCallbackReceived) {
    // ROS_INFO("PedsimSound: agents %d social state: %s", source_id ,agent.social_state.c_str());

    if(!started_playing) {
      curr_social_state = agent.social_state;
           
      play_source_client_ = nh_.serviceClient<arena_sound_srvs::PlaySource>("play_source");
      arena_sound_srvs::PlaySource play_msg;
      play_msg.request.source_id = source_id;
      // TODO buffer according to social state
      play_msg.request.social_state = curr_social_state;

      ros::service::waitForService("play_source", 1000);
      if(play_source_client_.call(play_msg)) {
        started_playing = true;

        get_source_volume_client_ = nh_.serviceClient<arena_sound_srvs::GetSourceVolume>("get_source_volume");
        arena_sound_srvs::GetSourceVolume vol_msg;
        vol_msg.request.source_id = source_id;
        vol_msg.request.social_state = curr_social_state;

        ros::service::waitForService("get_source_volume", 1000);
        get_source_volume_client_.call(vol_msg);

        publishSoundVisuals(pos_x, pos_y, vol_msg.response.volume);

        ROS_INFO("!!!! Source %d started playing", source_id);
      } else {
        ROS_ERROR("Source %d failed to call service play_source", source_id);
      }
    } else {
      prev_social_state = curr_social_state;
      curr_social_state = agent.social_state;

      arena_sound_srvs::GetSourceVolume vol_msg;
      vol_msg.request.source_id = source_id;
      vol_msg.request.social_state = curr_social_state;

      while (!get_source_volume_client_.isValid()) {
        ROS_WARN("Reconnecting play_source_client-server...");
        get_source_volume_client_.waitForExistence(ros::Duration(1.0));
        get_source_volume_client_ = nh_.serviceClient<arena_sound_srvs::PlaySource>("get_source_volume", true);
      }
      get_source_volume_client_.call(vol_msg);

      publishSoundVisuals(pos_x, pos_y, vol_msg.response.volume);

      if (curr_social_state.compare(prev_social_state)) {
        arena_sound_srvs::PlaySource play_msg;
        play_msg.request.source_id = source_id;
        // TODO buffer according to social state
        play_msg.request.social_state = curr_social_state;
        
        while (!play_source_client_.isValid()) {
          ROS_WARN("Reconnecting play_source_client-server...");
          play_source_client_.waitForExistence(ros::Duration(1.0));
          play_source_client_ = nh_.serviceClient<arena_sound_srvs::PlaySource>("play_source", true);
        }
        
        play_source_client_.call(play_msg);

        if(play_msg.response.success) {
          ROS_INFO("PedsimSound: Changed sound because of agent's %d state transition from %s to %s!!!",
                                 source_id, prev_social_state.c_str(), curr_social_state.c_str());
        } else {
          ROS_ERROR("BeforePhysics: Error changing sound at agent state transition");
        }
      }
    }
  }
}

void PedsimSound::publishSoundVisuals(float pos_x, float pos_y, double volume) {
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = nh_.getNamespace();
  marker.id = source_id;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pos_x;
  marker.pose.position.y = pos_y;
  marker.pose.position.z = 0;

  tf2::Quaternion q;  // use tf2 to convert 2d yaw -> 3d quaternion
  q.setRPY(0, 0, body->GetPhysicsBody()->GetAngle());  // from euler angles: roll, pitch, yaw
  marker.pose.orientation = tf2::toMsg(q);

  float volume_factor = 1.5f;
  marker.scale.z = 1.0;
  marker.scale.x = volume_factor*volume;
  marker.scale.y = volume_factor*volume;

  marker.color.r = 0.0f;
  marker.color.b = volume_factor*volume;
  marker.color.g = 0.0f;
  marker.color.a = 0.5f;

  marker.lifetime = ros::Duration();

  // ROS_INFO("marker_pub_.getNumSubscribers() %d", marker_pub_.getNumSubscribers());
  if (marker_pub_.getNumSubscribers() < 1)
  {
    ROS_WARN("PedsimSound:: No subscriber for the marker!");
  }

  marker_pub_.publish(marker);

}

void PedsimSound::pedAgentsCallback(const pedsim_msgs::AgentStatesConstPtr& agents) {
  agentCallbackReceived = true;

  for (int i = 0; i < agents->agent_states.size(); i++) {
    pedsim_msgs::AgentState p = agents->agent_states[i];
    if (source_id == p.id) {
        agent = p;
        // ROS_INFO("PedismSound::pedAgentsCallback: Pedsim agent with id: %ld, state: %s and pos: %f, %f",
        //                             p.id, p.social_state.c_str(), p.pose.position.x,p.pose.position.y);
        break;
    }

    if (i == agents->agent_states.size() - 1)
    {
        ROS_WARN("Couldn't find Human agent: %d", source_id);
    }
  }
}

}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::PedsimSound, flatland_server::ModelPlugin)