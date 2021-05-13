#include <arena_plugins/random_wandering.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <random>
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
  std::string robo_obstacle_topic = ros::this_node::getNamespace() + "/flatland_server/debug/model/"+GetModel()->GetName() ;
  std::string agent_state_topic = ros::this_node::getNamespace()+ "/"+GetModel()->GetName() ;   
  // Subscribe to ped_sims agent topic to retrieve the agents position
  robo_obstacle_sub_ = nh_.subscribe(robo_obstacle_topic, 1, &RandomWandering::agentCallback, this);
  
  //ROS_WARN("robo_obstacle_topic  %s, agent_state_topic %s",robo_obstacle_topic.c_str(),agent_state_topic.c_str());
  robo_obstacle_pub_ = nh_.advertise<visualization_msgs::Marker>(agent_state_topic, 1);


  if (body_ == nullptr) {
    throw YAMLException("Body with the name" + Q(body_name) + "does not exits");
  }

  // set initial values
  state = None;
  obstacleNear = false;
  laserScan = nullptr;
  targetAngularVelocity = 0.0;
  std::string random_wandrer_topic = ros::this_node::getNamespace() + "randomwanderer/scan";

  // subscribe topics
  laserSub = nh_.subscribe(random_wandrer_topic, 1, &RandomWandering::LaserCallback, this);
}

void RandomWandering::agentCallback(const visualization_msgs::MarkerArray& agents){
    robo_obstacles = agents;
}

void RandomWandering::AfterPhysicsStep(const Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);
  if (publish) {
    robo_obstacle.ns= GetModel()->GetNameSpace().c_str();
    //publish the agent state 
    robo_obstacle_pub_.publish(robo_obstacle);
  }
}


void RandomWandering::updateSafetyDistance(){
    set_safety_dist_footprint(safety_dist_b2body_, safety_dist_);
}
void RandomWandering::BeforePhysicsStep(const Timekeeper& timekeeper) {
  if (robo_obstacles.markers.size() == 0) {
    return;
  }

  currentLinearVelocity = body_->GetLinearVelocity();
  // ROS_INFO("Name of robo obstacle %s ",GetModel()->GetName().c_str());
  robo_obstacle =robo_obstacles.markers[0];
  DoStateTransition();
  robo_obstacle.points[0].x = currentLinearVelocity.x;
  robo_obstacle.points[0].y = currentLinearVelocity.y;
  Color c=Color(0.93, 0.16, 0.16, 0.3);
  safety_dist_body_->SetColor(c);
  updateSafetyDistance();
  float angle_soll = atan2(body_->GetLinearVelocity().x, body_->GetLinearVelocity().x);
  // body_->SetTransform(body_->GetPosition(), angle_soll);
  safety_dist_b2body_->SetTransform(body_->GetPosition(), angle_soll);
  // //Set pedsim_agent velocity in flatland simulator to approach next position
  // body_->SetLinearVelocity(body_->GetLinearVelocity());
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
    body->physics_body_->SetLinearVelocity(b2Vec2());
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


}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::RandomWandering,
                       flatland_server::ModelPlugin)
