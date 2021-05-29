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
    human_radius=0.4;
    mv = 1.5;
    av =1.5;
    r_static = 0.7;
    useDangerZone=true;  //TODO: will be the parameter set in yaml file
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

    safety_dist_ = reader.Get<double>("safety_dist");
    safety_dist_original_=safety_dist_;

    //Subscribing to pedsim topic to apply same movement
    std::string pedsim_agents_topic = ros::this_node::getNamespace() + reader.Get<std::string>("agent_topic");

    std::string agent_state_topic = reader.Get<std::string>("agent_state_pub", "agent_state");
    std::string danger_zone_topic = reader.Get<std::string>("agent_state_pub", "danger_zone");
    
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
    // publish the state of every pedestrain
    agent_state_pub_ = nh_.advertise<pedsim_msgs::AgentState>(agent_state_topic, 1);
    danger_zone_pub_ = nh_.advertise<flatland_msgs::DangerZone>(danger_zone_topic, 1);

    //Get bodies of pedestrian
    body_ = GetModel()->GetBody(reader.Get<std::string>("base_body"))->GetPhysicsBody();
    left_leg_body_ = GetModel()->GetBody(reader.Get<std::string>("left_leg_body"))->GetPhysicsBody();
    right_leg_body_ = GetModel()->GetBody(reader.Get<std::string>("right_leg_body"))->GetPhysicsBody();
    safety_dist_b2body_ = GetModel()->GetBody(reader.Get<std::string>("safety_dist_body"))->GetPhysicsBody();
    safety_dist_body_ = GetModel()->GetBody(reader.Get<std::string>("safety_dist_body"));
    
    // Set leg radius
    set_circular_footprint(left_leg_body_, leg_radius_);
    set_circular_footprint(right_leg_body_, leg_radius_);
    updateSafetyDistance();

    // check if valid bodies are given
    if (body_ == nullptr || left_leg_body_ == nullptr || right_leg_body_ == nullptr || safety_dist_b2body_==nullptr) {//s
        throw flatland_server::YAMLException("Body with with the given name does not exist");
    }
}

void PedsimMovement::reconfigure(){
    set_circular_footprint(left_leg_body_, leg_radius_);
    set_circular_footprint(right_leg_body_, leg_radius_);
}

void PedsimMovement::updateSafetyDistance(){
    set_safety_dist_footprint(safety_dist_b2body_, safety_dist_);
}

void PedsimMovement::BeforePhysicsStep(const Timekeeper &timekeeper) {
    // check if an update is REQUIRED
    if (agents_ == NULL) { //!update_timer_.CheckUpdate(timekeeper) || 
        return;
    }
    // get agents ID via namespace
    std::string ns_str = GetModel()->GetNameSpace();
    // ROS_WARN("name space: %s",ns_str.c_str());
    int id_ = std::stoi(ns_str.substr(13, ns_str.length()));

    //Find appropriate agent in list
    for (int i=0; i < (int)agents_->agent_states.size(); i++){
        // ROS_INFO("dddd%d",i);
        pedsim_msgs::AgentState p = agents_->agent_states[i];
        if (p.id == id_){
            person = p;
            break;
        }
    };
    //Check if person was found
    if (std::isnan(person.twist.linear.x)){
        ROS_WARN("Couldn't find agent: %d", int(person.id));
        return;
    }
    //Initialize agent
    if(init_== true){
        // Set initial leg position
        resetLegPosition(person.twist.linear.x, person.twist.linear.y, 0.0);
        init_ = false;
    }
    //modeling of safety distance
    vel_x = person.twist.linear.x; //
    vel_y = person.twist.linear.y; // 
    vel = sqrt(vel_x*vel_x+vel_y*vel_y);
    //change visualization of the human if they are talking
    if(useDangerZone==false){
        if (person.social_state=="\"talking\""){
            //r g b a
            Color c=Color(0.93, 0.16, 0.16, 0.3);
            safety_dist_body_->SetColor(c);
            safety_dist_=0.5;
            updateSafetyDistance();
        }else if(person.social_state=="\"running\""){
            //r g b a
            Color c=Color(0.56, 0.7, 0, 0.3);
            safety_dist_body_->SetColor(c);
            safety_dist_=1.5;
            updateSafetyDistance();
        }else if(person.social_state=="\"individual_moving\""){
            //r g b a
            Color c=Color(0.26, 0.3, 0, 0.3);//[0.26, 0.3, 0, 0.3]
            safety_dist_body_->SetColor(c);
            safety_dist_ = safety_dist_original_;
            updateSafetyDistance();
        }
    }else{
        dangerZoneCenter.clear();
        if(vel>0.01){ //this threshold is used for filtering of some rare cases, no influence for performance
            calculateDangerZone(vel);
            velocityAngles.clear();
            double velocityAngle=atan(vel_y/vel_x);
            velocityAngles.push_back(velocityAngle);
            velocityAngles.push_back(velocityAngle+M_PI);
            for(int i=0; i<2; i++){
                double x=pL[0]*cos(velocityAngles[i]);
                double y=pL[0]*sin(velocityAngles[i]);
                if(x*vel_x+y*vel_y<0){
                    dangerZoneCenter.push_back(person.pose.position.x+x);
                    dangerZoneCenter.push_back(person.pose.position.y+y);
                    break;
                }
            }
        }else{// if vel <0.01, it is treated as stopped
            dangerZoneRadius=safety_dist_original_;
            dangerZoneAngle=2*M_PI;        
            dangerZoneCenter.push_back(person.pose.position.x);
            dangerZoneCenter.push_back(person.pose.position.y);
        }
        //
        dangerZone.header=person.header;
        dangerZone.dangerZoneRadius=dangerZoneRadius;
        dangerZone.dangerZoneAngle=dangerZoneAngle;
        dangerZone.dangerZoneCenter=dangerZoneCenter;
    }




    float angle_soll = atan2(vel_y, vel_x);
    float angle_ist = body_->GetAngle();

    //Set pedsim_agent position in flatland simulator
    body_->SetTransform(b2Vec2(person.pose.position.x, person.pose.position.y), angle_soll);
    safety_dist_b2body_->SetTransform(b2Vec2(person.pose.position.x, person.pose.position.y), angle_soll);
    
    //Set pedsim_agent velocity in flatland simulator to approach next position
    body_->SetLinearVelocity(b2Vec2(vel_x, vel_y));
    safety_dist_b2body_->SetLinearVelocity(b2Vec2(vel_x, vel_y));
    
    //set each leg to the appropriate position.
    if (toggle_leg_movement_){
        double vel_mult = wp_->get_speed_multiplier(vel);
        switch (state_){
            //Right leg is moving
            case RIGHT:
                moveRightLeg(vel_x * vel_mult, vel_y * vel_mult, (angle_soll - angle_ist));
                if (vel_mult ==0.0){
                    state_ = LEFT;
                }
                break;
            //Left leg is moving
            case LEFT:
                moveLeftLeg(vel_x * vel_mult, vel_y * vel_mult, (angle_soll - angle_ist));
                if (vel_mult ==0.0){
                    state_ = RIGHT;
                }
                break;
        }
        //Recorrect leg position according to true person position
        if(wp_->is_leg_in_center()||vel<0.01)
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
    Vec2 center = Vec2(0.0, 0.0);
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
// original function just applies the defined radius in yaml-file.
// other option: modify flatland package, but third-party
void PedsimMovement::set_safety_dist_footprint(b2Body * physics_body, double radius){
    Vec2 center = Vec2(0.0, 0.0);
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
void PedsimMovement::ConfigFootprintDef(b2FixtureDef &fixture_def) {
    // configure physics properties
    fixture_def.density = 5.0;
    fixture_def.friction = 1.0;
    fixture_def.restitution = 2.0;

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

// ToDo: Implelent that more elegant
// Copied this function from model_body.cpp in flatland folder
// This is necessary to be able to set the leg radius auto-generated with variance
// original function just applies the defined properties from yaml-file.
// other option: modify flatland package, but third-party
void PedsimMovement::ConfigFootprintDefSafetyDist(b2FixtureDef &fixture_def) {
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

void PedsimMovement::calculateDangerZone(float vel_agent){
    interceptBE.clear();
    slopeBE.clear();
    pL.clear();
    dangerZoneRadius = mv*vel_agent + r_static;
    dangerZoneAngle = 11*M_PI / 6* exp(-1.4*av*vel_agent) +  M_PI/6;
    pB_1 = dangerZoneRadius*cos(dangerZoneAngle/2);
    pB_2 = dangerZoneRadius*sin(dangerZoneAngle/2);
    // pC_1 = dangerZoneRadius*cos(- dangerZoneAngle/2);
    // pC_2= dangerZoneRadius*sin(- dangerZoneAngle/2;
    // float diffY=-pB[1];
    // float diffX=-pB[0];
    a = human_radius*human_radius - pB_1*pB_1;
    b = 2*pB_1*pB_2;
    c = human_radius*human_radius - pB_2*pB_2;
    h = b*b - 4*a*c;
    if(h<0){
        ROS_INFO("no valid root for m+++++h=[%f]",h);
    }else{
        slopeBE1 = (-b+sqrt(h))/(2*a);
        slopeBE2 = (-b-sqrt(h))/(2*a);
    }
    interceptBE1 = pB_2 - slopeBE1*pB_1;
    interceptBE2 = pB_2 - slopeBE2*pB_1;
    interceptBE.push_back(interceptBE1);
    interceptBE.push_back(interceptBE2);
    slopeBE.push_back(slopeBE1);
    slopeBE.push_back(slopeBE2);
    for(int i= 0; i< 2; i++)
    {    
        float x = (- interceptBE[i])/(slopeBE[i]);
        float y = slopeBE[i]*x + interceptBE[i];
        float vAEx=x;
        float vAEy=y;
        if(vAEx*vel_agent<0){
            pL.push_back(x);
            pL.push_back(y);
            break;
        }
    }
    float vLBx=pL[0]-pB_1;
    float vLBy=pL[1]-pB_2;
    float vLAx=pL[0];
    float vLAy=pL[1];
    float dotProductLBLA =vLBx*vLAx+vLBy*vLAy;
    float normLB = sqrt(vLBx*vLBx+vLBy*vLBy);
    float normLA = sqrt(vLAx*vLAx+vLAy*vLAy);
    if (dangerZoneAngle < M_PI){
        float c1 = dotProductLBLA/(normLB*normLA);
        //clamp(-1,1)
        float c=c1>1 ? 1 : c1;
        c=c1<-1? -1 :c1;
        float angle = acos(c);
        dangerZoneAngle = 2*angle;
    }
    // ROS_INFO("safty model pE0[%f]dangerZoneRadius[%f]dangerZoneAngle[%f]", pL[0], dangerZoneRadius, dangerZoneAngle);
    updateDangerousZone(pL[0], dangerZoneRadius, dangerZoneAngle);
}

// bool PedsimMovement::isTheRightE(float vAEx, float vAEy, float vx, float vy){
//     return vAEx*vx + vAEy*vy <0;
// }
void PedsimMovement::updateDangerousZone(float p, float radius, float angle){
    //destroy the old fixtures 
    for(int i = 0; i<12; i++){
        b2Fixture* old_fix = safety_dist_b2body_->GetFixtureList();
        if(old_fix==nullptr){break;}
        safety_dist_b2body_->DestroyFixture(old_fix);
    }
    // create new feature
    b2FixtureDef fixture_def;
    // configure physics properties
    fixture_def.density = 1.0;
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
    float delta = angle/10;
    float last_angle = -angle/2;
    float next_angle;
    float v1 = radius*cos(last_angle);
    float v2 = radius*sin(last_angle);
    Color c=Color(0.93, 0.16, 0.16, 0.3);
    b2PolygonShape shape;
    b2Vec2 verts[3];
    for(int i = 0; i<10; i++){
        next_angle= -angle/2 + (i+1)*delta;
        verts[0].Set(0.0, 0.0);
        verts[1].Set(v1, v2);
        v1= radius*cos(next_angle);
        v2= radius*sin(next_angle);
        verts[2].Set(v1, v2);
        shape.Set(verts, 3);
        fixture_def.shape = &shape;
        safety_dist_b2body_->CreateFixture(&fixture_def);
    }    
    if(p==0.0){
        verts[0].Set(0.0, 0.0);
        v1= radius*cos(-angle/2);
        v2= radius*sin(-angle/2);
        verts[1].Set(v1, v2);
        v1= radius*cos(angle/2);
        v2= radius*sin(angle/2);
        verts[2].Set(v1, v2);
        shape.Set(verts, 3);
        fixture_def.shape = &shape;
        safety_dist_b2body_->CreateFixture(&fixture_def);
    }else{
        //first vertex
        verts[0].Set(0.0, 0.0);
        verts[1].Set(p, 0.0);
        v1= radius*cos(angle/2);
        v2= radius*sin(angle/2);
        verts[2].Set(v1, v2);
        shape.Set(verts, 3);
        fixture_def.shape = &shape;
        safety_dist_b2body_->CreateFixture(&fixture_def);
        //second vertex
        verts[0].Set(0.0, 0.0);
        verts[1].Set(p, 0.0);
        v1= radius*cos(-angle/2);
        v2= radius*sin(-angle/2);
        verts[2].Set(v1, v2);
        shape.Set(verts, 3);
        fixture_def.shape = &shape;
        safety_dist_b2body_->CreateFixture(&fixture_def);
    }
    safety_dist_body_->SetColor(c);
}

void PedsimMovement::AfterPhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);
  if (publish) {
    // get the state of the body and publish the data
    // publish agent state for every human
    //publish the agent state 
    agent_state_pub_.publish(person);
    if(useDangerZone==true){
        danger_zone_pub_.publish(dangerZone);
    }    
  }
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::PedsimMovement, flatland_server::ModelPlugin)

