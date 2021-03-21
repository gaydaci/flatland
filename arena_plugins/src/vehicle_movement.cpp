 /*
 * @name	 	vehicle_movement.cpp
 * @brief	 	The movement of the pedsim agents is as well applied to the flatland models.
 **/

#include <arena_plugins/vehicle_movement.h>
#include <arena_plugins/triangle_profile.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>
#include<bits/stdc++.h>
using namespace flatland_server;

namespace flatland_plugins {

void VehicleMovement::OnInitialize(const YAML::Node &config){
    agents_ = NULL;

    //get parameters
    flatland_server::YamlReader reader(config);
    
    //Subscribing to pedsim topic to apply same movement
    std::string pedsim_agents_topic = ros::this_node::getNamespace() + reader.Get<std::string>("agent_topic");
    
    // Subscribe to ped_sims agent topic to retrieve the agents position
    pedsim_agents_sub_ = nh_.subscribe(pedsim_agents_topic, 1, &VehicleMovement::agentCallback, this);

    //Get bodies of pedestrian
    body_ = GetModel()->GetBody(reader.Get<std::string>("base_body"))->GetPhysicsBody();
    
    // check if valid bodies are given
    if (body_ == nullptr) {
        throw flatland_server::YAMLException("Body with with the given name does not exist");
    }
}


void VehicleMovement::BeforePhysicsStep(const Timekeeper &timekeeper) {
    if (agents_ == NULL) {
        return;
    }
    
    // get agents ID via namespace
    std::string ns_str = GetModel()->GetNameSpace();
    int id_ = std::stoi(ns_str.substr(13, ns_str.length()));

    //Find appropriate agent in list
    pedsim_msgs::AgentState person;
    for (int i = 0; i < agents_->agent_states.size(); i++){
        pedsim_msgs::AgentState p = agents_->agent_states[i];
        if (p.id == id_){
            person = p;
            break;
        }

        if (i == agents_->agent_states.size() - 1)
        {
            ROS_WARN("Couldn't find agent: %d", id_);
        }
    };


    float vel_x = person.twist.linear.x;
    float vel_y = person.twist.linear.y;
    float angle_soll = atan2(vel_y, vel_x);
    float angle_ist = body_->GetAngle();

    //Set pedsim_agent position in flatland simulator
    body_->SetTransform(b2Vec2(person.pose.position.x, person.pose.position.y), angle_soll);
    
    //Set pedsim_agent velocity in flatland simulator to approach next position
    body_->SetLinearVelocity(b2Vec2(vel_x, vel_y));
}


void VehicleMovement::agentCallback(const pedsim_msgs::AgentStatesConstPtr& agents){
    agents_ = agents;
}

};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::VehicleMovement, flatland_server::ModelPlugin)

