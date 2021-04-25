#include <arena_plugins/random_wandering.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

#include <random>

namespace flatland_plugins {
using namespace std;


void RandomWandering::OnInitialize(const YAML::Node& config) {
  // read yaml file
  YamlReader reader(config);
  string body_name = reader.Get<string>("body");
  configuredLinearVelocity = reader.Get<float>("linear_velocity");
  configuredAngularVelocity = reader.Get<float>("angular_velocity");
  body = GetModel()->GetBody(body_name);
  if (body == nullptr) {
    throw YAMLException("Body with the name" + Q(body_name) + "does not exits");
  }

  // set initial values
  state = None;
  obstacleNear = false;
  laserScan = nullptr;
  targetAngularVelocity = 0.0;

  // subscribe topics
  laserSub = nh_.subscribe("randomwanderer/scan", 1, &RandomWandering::LaserCallback, this);
}


void RandomWandering::BeforePhysicsStep(const Timekeeper& timekeeper) {
  currentLinearVelocity = body->physics_body_->GetLinearVelocity();
  DoStateTransition();
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
