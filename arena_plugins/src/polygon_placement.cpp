 /*
 * @name	 	pedsim_movement.cpp
 * @brief	 	The movement of the pedsim agents is as well applied to the flatland models.
 *              Furthermore, a walking pattern is added.
 * @author  	Ronja Gueldenring
 * @date 		2019/04/05
 **/

#include <arena_plugins/polygon_placement.h>
#include <arena_plugins/triangle_profile.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>
#include<bits/stdc++.h>
using namespace flatland_server;

namespace flatland_plugins {

void PolygonPlacement::OnInitialize(const YAML::Node &config){
    agents_ = NULL;
    std::random_device r;
    std::default_random_engine generator(r());
    //get parameters
    flatland_server::YamlReader reader(config);
    
    double update_rate = reader.Get<double>("update_rate");
    rotate=true; //could be the setting in yaml file
    // update_timer_.SetRate(update_rate);  // timer to update global movement of agent
    
    std::string pedsim_agents_topic = ros::this_node::getNamespace() + reader.Get<std::string>("agent_topic");


    // Subscribe to ped_sims agent topic to retrieve the agents position
    pedsim_agents_sub_ = nh_.subscribe(pedsim_agents_topic, 1, &PolygonPlacement::agentCallback, this);


    //Get bodies of pedestrian
    body_ = GetModel()->GetBody(reader.Get<std::string>("base_body"))->GetPhysicsBody();

}

void PolygonPlacement::BeforePhysicsStep(const Timekeeper &timekeeper) {
    // check if an update is REQUIRED
    if (agents_ == NULL) { //!update_timer_.CheckUpdate(timekeeper) || 
    ROS_INFO("++++++++++=NULL");
        return;
    }
    // get agents ID via namespace
    std::string ns_str = GetModel()->GetNameSpace();
    // ROS_WARN("name space: %s",ns_str.c_str());
    int id_ = std::stoi(ns_str.substr(15, ns_str.length()));

    //Find appropriate agent in list
    for (int i=0; i < (int)agents_->agent_states.size(); i++){
        //
        pedsim_msgs::AgentState p = agents_->agent_states[i];
        // ROS_INFO("dddd%d",(int)p.type);
        if (p.id==id_+21){
            person = p;
            break;
        }
    };
    //Check if person was found
    if (std::isnan(person.twist.linear.x)){
        ROS_WARN("Couldn't find agent: %d", int(person.id));
        return;
    }
    //modeling of safety distance
    float vel_x = person.twist.linear.x; //
    float vel_y = person.twist.linear.y; // 
    float vel = sqrt(vel_x*vel_x+vel_y*vel_y);
    float angle_soll = atan2(vel_y, vel_x);
    float angle_ist = body_->GetAngle();

    //Set pedsim_agent position in flatland simulator
    if(rotate){
        body_->SetTransform(b2Vec2(person.pose.position.x, person.pose.position.y), angle_soll);
    }else{
        body_->SetTransform(b2Vec2(person.pose.position.x, person.pose.position.y), angle_ist);
    }
    
}

void PolygonPlacement::agentCallback(const pedsim_msgs::AgentStatesConstPtr& agents){
    agents_ = agents;
}

void PolygonPlacement::AfterPhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::PolygonPlacement, flatland_server::ModelPlugin)

