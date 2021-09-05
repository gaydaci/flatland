#include <arena_plugins/random_move.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <cmath>
#include <random>

namespace flatland_plugins {
using namespace std;
void RandomMove::OnInitialize(const YAML::Node& config) {
  YamlReader reader(config);
  string body_name = reader.Get<string>("body");
  linear_velocity_ = reader.Get<float>("linear_velocity");
  angular_velocity_max_ = reader.Get<float>("angular_velocity_max");
  body_ = GetModel()->GetBody(body_name);
  angle_ = body_->physics_body_->GetAngle();
  if (body_ == nullptr) {
    throw YAMLException("Body with the name" + Q(body_name) + "does not exits");
  }
  // Make sure there are not unusued keys
  reader.EnsureAccessedAllKeys();
}
float RandomMove::randomRange(const float range_lo, const float range_hi) {
  static random_device r{};
  static default_random_engine e1{r()};

  uniform_real_distribution<float> nd(range_lo, range_hi);
  return nd(e1);
}
void RandomMove::BeforePhysicsStep(const Timekeeper& timekeeper) {
    float v_x = linear_velocity_ * cos(angle_);
    float v_y = linear_velocity_ * sin(angle_);
    body_->physics_body_->SetLinearVelocity(b2Vec2(v_x, v_y));
}
void RandomMove::BeginContact(b2Contact *contact){
  // not perfect, but enough for training 
  Entity *other_entity;
  b2Fixture *this_fixture, *other_fixture;
  if (!FilterContact(contact,other_entity,this_fixture,other_fixture)) 
    return;
  bool is_normal_point_out;
  if (contact->GetFixtureA() == this_fixture) {
    is_normal_point_out = true;
  }else{
    is_normal_point_out = false;
  }
  auto angle = body_->physics_body_->GetAngle();
  b2WorldManifold world_manifold;
  contact->GetWorldManifold(&world_manifold);
  b2Vec2 normal = world_manifold.normal;
  auto normal_angle = std::atan2(normal.x, normal.y);
  if(!is_normal_point_out){
    normal_angle = -normal_angle;
  }
  angle_ = 2 * normal_angle - angle;
  angle_ += randomRange(-1.57, 1.57);
  // float v_x = linear_velocity_ * cos(new_angle_);
  // float v_y = linear_velocity_ * sin(new_angle_);
  // body_->physics_body_->SetLinearVelocity(b2Vec2(v_x, v_y));
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::RandomMove,
                       flatland_server::ModelPlugin)
