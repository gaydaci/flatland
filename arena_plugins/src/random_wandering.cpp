#include <arena_plugins/random_wandering.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <random>
#include <visualization_msgs/Marker.h>

using namespace flatland_server;

namespace flatland_plugins {
using namespace std;


void RandomWandering::OnInitialize(const YAML::Node& config) {
  // read yaml file
  YamlReader reader(config);
  safety_dist_ = reader.Get<double>("safety_dist");
  safety_dist_original_=safety_dist_;
  string body_name = reader.Get<string>("body");
  configuredLinearVelocity = reader.Get<float>("linear_velocity");
  configuredAngularVelocity = reader.Get<float>("angular_velocity");
  body = GetModel()->GetBody(body_name);
  body_ =  body->physics_body_ ;
  safety_dist_b2body_ = GetModel()->GetBody(reader.Get<std::string>("safety_dist_body"))->GetPhysicsBody();
  safety_dist_body_ = GetModel()->GetBody(reader.Get<std::string>("safety_dist_body"));
  updateSafetyDistance();
  std::string path = ros::package::getPath("simulator_setup");
  YAML::Node config1 = YAML::LoadFile(path+"/advanced_configs.yaml");
  human_radius=0.4;
  mv = 1.5;
  av =1.5;
  r_static = 0.7;
  useDangerZone=false;
  if (config1["use danger zone"].as<float>()== 1.0) 
  {useDangerZone=true;}

  std::string agent_state_topic = ros::this_node::getNamespace()+ "/"+GetModel()->GetName() ;   
  robo_obstacle_pub_ = nh_.advertise<visualization_msgs::Marker>(agent_state_topic, 1);


  if (body_ == nullptr) {
    throw YAMLException("Body with the name" + Q(body_name) + "does not exits");
  }
  // ROS_WARN("35");

  // set initial values
  state = None;
  obstacleNear = false;
  laserScan = nullptr;
  targetAngularVelocity = 0.0;
  std::string random_wanderer_topic = ros::this_node::getNamespace() + "/randomwanderer/scan" + std::to_string(GetModel()->model_index_);

  // subscribe topics
  laserSub = nh_.subscribe(random_wanderer_topic, 1, &RandomWandering::LaserCallback, this);
}


void RandomWandering::AfterPhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);
  // ROS_WARN("51");
 
  if (publish) {
    
    //publish the agent state 
    robo_obstacle_pub_.publish(robo_obstacle);
  }
}


void RandomWandering::updateSafetyDistance(){
    set_safety_dist_footprint(safety_dist_b2body_, safety_dist_);
}
void RandomWandering::BeforePhysicsStep(const Timekeeper& timekeeper) {

  // ROS_WARN("64");
  robo_obstacle.header.frame_id = "base_link";
  robo_obstacle.header.stamp = ros::Time::now();
  robo_obstacle.ns= GetModel()->GetNameSpace().c_str();
  robo_obstacle.id = 1;
  robo_obstacle.action = visualization_msgs::Marker::ADD;
  robo_obstacle.pose.position.x =  body_->GetPosition().x;
  robo_obstacle.pose.position.y = body_->GetPosition().y;
  robo_obstacle.scale.x = body_->GetLinearVelocity().x;
  robo_obstacle.scale.y = body_->GetLinearVelocity().y;
  DoStateTransition();
  // ROS_WARN("75");

      //modeling of safety distance
  vel_x = robo_obstacle.scale.x ; //
  vel_y = robo_obstacle.scale.y;   // 
  vel = sqrt(vel_x*vel_x+vel_y*vel_y);

  Color c=Color(0.93, 0.16, 0.16, 0.3);
  if(useDangerZone==false){
          //change visualization of the human if they are talking         
    
          safety_dist_body_->SetColor(c);
          updateSafetyDistance();
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
                  dangerZoneCenter.push_back(robo_obstacle.pose.position.x+x);
                  dangerZoneCenter.push_back(robo_obstacle.pose.position.y+y);
                  break;
              }
          }
      }else{// if vel <0.01, it is treated as stopped
          dangerZoneRadius=safety_dist_original_;
          dangerZoneAngle=2*M_PI;        
          dangerZoneCenter.push_back(robo_obstacle.pose.position.x);
          dangerZoneCenter.push_back(robo_obstacle.pose.position.y);
      }
      //
      dangerZone.header=robo_obstacle.header;
      dangerZone.dangerZoneRadius=dangerZoneRadius;
      dangerZone.dangerZoneAngle=dangerZoneAngle;
      dangerZone.dangerZoneCenter=dangerZoneCenter;
  }
  float angle_soll = atan2(body_->GetLinearVelocity().x, body_->GetLinearVelocity().x);

  // body_->SetTransform(body_->GetPosition(), angle_soll);
  safety_dist_b2body_->SetTransform(body_->GetPosition(), angle_soll);
  // //Set pedsim_agent velocity in flatland simulator to approach next position
  safety_dist_b2body_->SetLinearVelocity(body_->GetLinearVelocity());
  

}

// ToDo: Implelent that more elegant
// Copied this function from model_body.cpp in flatland folder
// This is necessary to be able to set the leg radius auto-generated with variance
// original function just applies the defined radius in yaml-file.
// other option: modify flatland package, but third-party
void RandomWandering::set_safety_dist_footprint(b2Body * physics_body, double radius){
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
void RandomWandering::ConfigFootprintDefSafetyDist(b2FixtureDef &fixture_def) {
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


void RandomWandering::LaserCallback(const sensor_msgs::LaserScanConstPtr& laser_scan){
  laserScan = laser_scan;
}


bool RandomWandering::ObstacleNear() {
  if (laserScan != nullptr) {
    for (int i = 0; i < laserScan->ranges.size(); i++)
    {
      float val = laserScan->ranges[i];
      if (!isnan(val)) {
        if (val < 1.0) {
          return true;
        }
      }
    }
  }
  return false;
}


void RandomWandering::DoStateTransition() {
  if (state == Forward) {
    if (ObstacleNear()) {
      ActivateState(Turning);
    }
  } else if (state == Turning) {
    body->physics_body_->SetAngularVelocity(targetAngularVelocity);
    float angle = fmod(body->physics_body_->GetAngle(), 2 * b2_pi);
    // make sure angle is positive
    if (angle < 0) {
      angle += 2 * b2_pi;
    }
    if (angleGoal - 0.1 < angle && angle < angleGoal + 0.1) {
      // we're close to goal angle -> start moving forward again
      ActivateState(Forward);
    }
  } else {
    ActivateState(Forward);
  }
}


void RandomWandering::ActivateState(WandererState state_in) {
  // ROS_INFO("activating state %d", state_in);
  DeactivateState(state);
  state = state_in;
  if (state == Forward) {
    // move in the direction we are currently facing
    float angle = body->physics_body_->GetAngle();
    float v_x = configuredLinearVelocity * cos(angle);
    float v_y = configuredLinearVelocity * sin(angle);
    body->physics_body_->SetLinearVelocity(b2Vec2(v_x, v_y));
  } else if (state == Turning) {
    targetAngularVelocity = configuredAngularVelocity;
    // set random goal angle
    angleGoal = RandomRange(0, 2 * b2_pi);
    float angle = body->physics_body_->GetAngle();
    // get the right turning direction
    float delta = angleGoal - angle;
    while (delta < 0) {
      delta += 2 * b2_pi;
    }

    while (delta > 2 * b2_pi) {
      delta -= 2 * b2_pi;
    }

    if (delta > b2_pi) {
      targetAngularVelocity *= -1;
    }
    body->physics_body_->SetAngularVelocity(targetAngularVelocity);
  }
}

void RandomWandering::DeactivateState(WandererState state_in) {
  if (state == Forward) {
    // stop moving
    body->physics_body_->SetLinearVelocity(b2Vec2(0, 0));
  } else if (state == Turning) {
    // stop turning
    body->physics_body_->SetAngularVelocity(0);
  }
}


int RandomWandering::GetTurnDirection(float angle, float angle_goal) {
  // both arguments should be in the range of 0 to 2*PI
  // returns 
  //  negative int => turn clockwise
  //  positive int => turn anticlockwise
  float diff = angle_goal - angle;
  float diff_abs = abs(diff);
  float diff_pos = abs((angle_goal + 2 * b2_pi) - angle);
  float diff_neg = abs((angle_goal - 2 * b2_pi) - angle);

  // get smallest diff
  if (diff_abs < diff_pos && diff_abs < diff_neg) {
    return static_cast<int>(diff);
  } else if (diff_pos < diff_neg) {
    return 1;
  }

  return -1;
}


float RandomWandering::RandomRange(const float range_lo, const float range_hi) {
  static random_device r{};
  static default_random_engine e1{r()};

  uniform_real_distribution<float> nd(range_lo, range_hi);
  return nd(e1);
}


void RandomWandering::calculateDangerZone(float vel_agent){
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
    c_ = human_radius*human_radius - pB_2*pB_2;
    h = b*b - 4*a*c_;
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
void RandomWandering::updateDangerousZone(float p, float radius, float angle){
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
    Color c=Color(0.93, 0.16, 0.16, 0.3);
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


}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::RandomWandering,
                       flatland_server::ModelPlugin)
