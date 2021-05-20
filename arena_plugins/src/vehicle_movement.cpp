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
#include <iostream>
#include <pwd.h>
#include <string>
using namespace flatland_server;

namespace flatland_plugins {

void VehicleMovement::OnInitialize(const YAML::Node &config){
    agents_ = NULL;

    //get parameters
    flatland_server::YamlReader reader(config);

    safety_dist_ = reader.Get<double>("safety_dist");
    safety_dist_original_ = safety_dist_;
    //Subscribing to pedsim topic to apply same movement
    std::string pedsim_agents_topic = ros::this_node::getNamespace() + reader.Get<std::string>("agent_topic");
    
    std::string agent_state_topic = reader.Get<std::string>("agent_state_pub", "agent_state");

    // Subscribe to ped_sims agent topic to retrieve the agents position
    pedsim_agents_sub_ = nh_.subscribe(pedsim_agents_topic, 1, &VehicleMovement::agentCallback, this);
    // publish pedsim agent AgentState
    agent_state_pub_ = nh_.advertise<pedsim_msgs::AgentState>(agent_state_topic, 1);

    //Get bodies of pedestrian
    body_ = GetModel()->GetBody(reader.Get<std::string>("base_body"))->GetPhysicsBody();
    safety_dist_b2body_ = GetModel()->GetBody(reader.Get<std::string>("safety_dist_body"))->GetPhysicsBody();
    safety_dist_body_ = GetModel()->GetBody(reader.Get<std::string>("safety_dist_body"));
    updateSafetyDistance();

    // check if valid bodies are given
    if (body_ == nullptr) {
        throw flatland_server::YAMLException("Body with with the given name does not exist");
    }
}

void VehicleMovement::updateSafetyDistance(){
    set_safety_dist_footprint(safety_dist_b2body_, safety_dist_);
}

void VehicleMovement::BeforePhysicsStep(const Timekeeper &timekeeper) {
    if (agents_ == NULL) {
        return;
    }
      
    passwd* pw = getpwuid(getuid());
    std::string path(pw->pw_dir);
    YAML::Node config = YAML::LoadFile(path+"/catkin_ws/src/arena-rosnav/simulator_setup/saftey_distance_parameter.yaml");

    // get agents ID via namespace
    std::string ns_str = GetModel()->GetNameSpace();
    // ROS_WARN("name space: %s",ns_str.c_str());
    int id_ = std::stoi(ns_str.substr(13, ns_str.length()));

    //Find appropriate agent in list
    for (int i = 0; i < (int) agents_->agent_states.size(); i++){
        pedsim_msgs::AgentState p = agents_->agent_states[i];
        if (p.id == id_){
            person = p;
            Color c=Color(  0.26, 0.3, 0, 0.3) ;

            if ( config["safety distance factor"][person.social_state].as<float>() > 1.2  ){
                 c=Color(0.93, 0.16, 0.16, 0.3);
            }
            else if(config["safety distance factor"][person.social_state].as<float>() < 0.89){  
                 c=Color(  0.16, 0.93, 0.16, 0.3) ;
            }
       
            safety_dist_body_->SetColor(c);
            safety_dist_= config["safety distance factor"][person.social_state].as<float>() * config["human obstacle safety distance radius"][person.type].as<float>()   ;
            updateSafetyDistance();
            break;
        }

        if (i == agents_->agent_states.size() - 1)
        {
            ROS_WARN("Couldn't find vehicle agent: %d", id_);
            return;
        }
    };
 
    float vel_x = person.twist.linear.x;
    float vel_y = person.twist.linear.y;
    float angle_soll = person.direction;
    float angle_ist = body_->GetAngle();

    //Set pedsim_agent position in flatland simulator
    body_->SetTransform(b2Vec2(person.pose.position.x, person.pose.position.y), angle_soll);
    safety_dist_b2body_->SetTransform(b2Vec2(person.pose.position.x, person.pose.position.y), angle_soll);
    
    //Set pedsim_agent velocity in flatland simulator to approach next position
    body_->SetLinearVelocity(b2Vec2(vel_x, vel_y));
    safety_dist_b2body_->SetLinearVelocity(b2Vec2(vel_x, vel_y));
}

// ToDo: Implelent that more elegant
// Copied this function from model_body.cpp in flatland folder
// This is necessary to be able to set the leg radius auto-generated with variance
// original function just applies the defined radius in yaml-file.
// other option: modify flatland package, but third-party
void VehicleMovement::set_safety_dist_footprint(b2Body * physics_body, double radius){
    Vec2 center = Vec2(0, 0);
    b2FixtureDef fixture_def;
    ConfigFootprintDefSafetyDist(fixture_def);

    b2CircleShape shape;
    shape.m_p.Set(center.x, center.y);
    shape.m_radius = radius;

    fixture_def.shape = &shape;
    b2Fixture* old_fix = physics_body->GetFixtureList();
    physics_body->DestroyFixture(old_fix);
    physics_body->CreateFixture(&fixture_def);
}

// ToDo: Implelent that more elegant
// Copied this function from model_body.cpp in flatland folder
// This is necessary to be able to set the leg radius auto-generated with variance
// original function just applies the defined properties from yaml-file.
// other option: modify flatland package, but third-party
void VehicleMovement::ConfigFootprintDefSafetyDist(b2FixtureDef &fixture_def) {
    // configure physics properties
    fixture_def.density = 0.0;
    fixture_def.friction = 0.0;
    fixture_def.restitution = 0.0;

    // config collision properties
    fixture_def.isSensor = true;
    fixture_def.filter.groupIndex = 0;

    // Defines that body is just seen in layer "2D" and "ped"
    fixture_def.filter.categoryBits = 0x000a;

    bool collision = false;
    if (collision) {
        // b2d docs: maskBits are "I collide with" bitmask
        fixture_def.filter.maskBits = fixture_def.filter.categoryBits;
    } else {
        // "I will collide with nothing"
        fixture_def.filter.maskBits = 0;
    }
}

void VehicleMovement::agentCallback(const pedsim_msgs::AgentStatesConstPtr& agents){
    agents_ = agents;
}

void VehicleMovement::AfterPhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);
  if (publish) {
    // get the state of the body and publish the data
    // publish agent state for every human
    //publish the agent state 
    agent_state_pub_.publish(person);
  }
}

};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::VehicleMovement, flatland_server::ModelPlugin)

