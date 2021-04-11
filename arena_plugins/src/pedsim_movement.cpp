 /*
 * @name	 	pedsim_movement.cpp
 * @brief	 	The movement of the pedsim agents is as well applied to the flatland models.
 *              Furthermore, a walking pattern is added.
 * @author  	Ronja Gueldenring
 * @date 		2019/04/05
 **/

#include <arena_plugins/pedsim_movement.h>
#include <arena_plugins/triangle_profile.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>
#include<bits/stdc++.h>
using namespace flatland_server;

namespace flatland_plugins {

void PedsimMovement::OnInitialize(const YAML::Node &config){
    agents_ = NULL;
    state_ = LEFT;
    init_ = true;

    // random generator to generate leg_offset, step_length with variance.
    std::random_device r;
    std::default_random_engine generator(r());

    //get parameters
    flatland_server::YamlReader reader(config);
    toggle_leg_movement_ = reader.Get<bool>("toggle_leg_movement");
    
    //generating varying walking properties
    std::normal_distribution<double> d_leg_offset{reader.Get<double>("leg_offset") , reader.Get<double>("var_leg_offset")};
    leg_offset_ = d_leg_offset(generator);

    std::normal_distribution<double> d_step_time{reader.Get<double>("step_time") , reader.Get<double>("var_step_time")};
    double step_time = d_step_time(generator);

    std::normal_distribution<double> d_leg_radius{reader.Get<double>("leg_radius") , reader.Get<double>("var_leg_radius")};
    leg_radius_ = d_leg_radius(generator);

    //Subscribing to pedsim topic to apply same movement
    std::string pedsim_agents_topic = ros::this_node::getNamespace() + reader.Get<std::string>("agent_topic");
    
    double update_rate = reader.Get<double>("update_rate");
    // update_timer_.SetRate(update_rate);  // timer to update global movement of agent
    
    //Walking profile
    wp_ = new flatland_plugins::TriangleProfile(step_time);
    // get frame name of base body
    // std::string ns_str = GetModel()->GetNameSpace();
    // body_frame_ = ns_str;
    // body_frame_ += "_base";

    // Subscribe to ped_sims agent topic to retrieve the agents position
    pedsim_agents_sub_ = nh_.subscribe(pedsim_agents_topic, 1, &PedsimMovement::agentCallback, this);

    //Get bodies of pedestrian
    body_ = GetModel()->GetBody(reader.Get<std::string>("base_body"))->GetPhysicsBody();
    left_leg_body_ = GetModel()->GetBody(reader.Get<std::string>("left_leg_body"))->GetPhysicsBody();
    right_leg_body_ = GetModel()->GetBody(reader.Get<std::string>("right_leg_body"))->GetPhysicsBody();
    
    // Set leg radius
    set_circular_footprint(left_leg_body_, leg_radius_);
    set_circular_footprint(right_leg_body_, leg_radius_);
    // check if valid bodies are given
    if (body_ == nullptr || left_leg_body_ == nullptr || right_leg_body_ == nullptr) {
        throw flatland_server::YAMLException("Body with with the given name does not exist");
    }
}

int PedsimMovement::GetAgent(int agentId, pedsim_msgs::AgentState &agent) {
    for (int i = 0; i < agents_->agent_states.size(); i++){
        pedsim_msgs::AgentState p = agents_->agent_states[i];
        if (p.id == agentId){
            agent = p;
            return 0;
        }

        if (i == agents_->agent_states.size() - 1)
        {
            ROS_WARN("Couldn't find agent: %d", agentId);
        }
    }
    return -1;
}

void PedsimMovement::BeforePhysicsStep(const Timekeeper &timekeeper) {
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

    //Initialize agent
    if(init_== true){
        // Set initial leg position
        resetLegPosition(person.twist.linear.x, person.twist.linear.y, 0.0);
        init_ = false;
    }


    float vel_x = person.twist.linear.x;
    float vel_y = person.twist.linear.y;

    float angle_soll = atan2(person.direction.y, person.direction.x);
    float angle_ist = body_->GetAngle();

    //Set pedsim_agent position in flatland simulator
    body_->SetTransform(b2Vec2(person.pose.position.x, person.pose.position.y), angle_soll);
    
    //Set pedsim_agent velocity in flatland simulator to approach next position
    body_->SetLinearVelocity(b2Vec2(vel_x, vel_y));

    //set each leg to the appropriate position.
    if (toggle_leg_movement_){      
        double vel_mult = wp_->get_speed_multiplier(vel_x);
        // ROS_WARN("vel_mult %lf",vel_mult);
        switch (state_){
            //Right leg is moving
            case RIGHT:
                moveRightLeg(vel_x * vel_mult, vel_y * vel_mult, (angle_soll - angle_ist));
                if (vel_mult ==0.0){
                    state_ = LEFT;
                    // ROS_WARN("turn to left");
                }
                break;
            //Left leg is moving
            case LEFT:
                moveLeftLeg(vel_x * vel_mult, vel_y * vel_mult, (angle_soll - angle_ist));
                if (vel_mult ==0.0){
                    state_ = RIGHT;
                    // ROS_WARN("turn to right");
                }
                break;
        }
        //Recorrect leg position according to true person position
        if(wp_->is_leg_in_center())
            resetLegPosition(person.pose.position.x, person.pose.position.y, angle_soll);
        
    }else{
        resetLegPosition(person.pose.position.x, person.pose.position.y, angle_soll);
    }
}

void PedsimMovement::moveLeftLeg(float32 vel_x, float32 vel_y, float32 angle_diff){
    left_leg_body_->SetLinearVelocity(b2Vec2(vel_x, vel_y));
    left_leg_body_->SetAngularVelocity(angle_diff);
}

void PedsimMovement::moveRightLeg(float32 vel_x, float32 vel_y, float32 angle_diff){
    right_leg_body_->SetLinearVelocity(b2Vec2(vel_x, vel_y));
    right_leg_body_->SetAngularVelocity(angle_diff);
}

// If the legs are both at the 0-point, they get corrected to the position from pedsim.
// This is necessary to regularily synchronize legs and pedsim agents.
void PedsimMovement::resetLegPosition(float32 x, float32 y, float32 angle){
    float left_leg_x = x + leg_offset_/2 * cos(M_PI/2 - angle);
    float left_leg_y = y - leg_offset_/2 * sin(M_PI/2 - angle);
    left_leg_body_->SetTransform(b2Vec2(left_leg_x, left_leg_y), angle);
    float right_leg_x = x - leg_offset_/2 * cos(M_PI/2 - angle);
    float right_leg_y = y + leg_offset_/2 * sin(M_PI/2 - angle);
    right_leg_body_->SetTransform(b2Vec2(right_leg_x, right_leg_y), angle);
}

void PedsimMovement::agentCallback(const pedsim_msgs::AgentStatesConstPtr& agents){
    agents_ = agents;
}

// ToDo: Implelent that more elegant
// Copied this function from model_body.cpp in flatland folder
// This is necessary to be able to set the leg radius auto-generated with variance
// original function just applies the defined radius in yaml-file.
// other option: modify flatland package, but third-party
void PedsimMovement::set_circular_footprint(b2Body * physics_body, double radius){
    Vec2 center = Vec2(0, 0);
    b2FixtureDef fixture_def;
    ConfigFootprintDef(fixture_def);

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
void PedsimMovement::ConfigFootprintDef(b2FixtureDef &fixture_def) {
    // configure physics properties
    fixture_def.density = 0.0;
    fixture_def.friction = 0.0;
    fixture_def.restitution = 0.0;

    // config collision properties
    fixture_def.isSensor = false;
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
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::PedsimMovement, flatland_server::ModelPlugin)

