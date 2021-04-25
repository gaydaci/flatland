 /*
 * @name	 	pedsim_movement.cpp
 * @brief	 	The movement of the pedsim agents is as well applied to the flatland models.
 *              Furthermore, a walking pattern is added.
 * @author  	Ronja Gueldenring
 * @date 		2019/04/05
 **/

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/body.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <arena_plugins/triangle_profile.h>
#include<cmath> 


#ifndef FLATLAND_PLUGINS_PEDSIM_MOVEMENT_H
#define FLATLAND_PLUGINS_PEDSIM_MOVEMENT_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * @class PedsimMovement
 * @brief The movement of the pedsim agents is as well applied to the flatland models.
 * Furthermore a walking pattern is added
 * 
 */
class PedsimMovement : public ModelPlugin {
 public:
  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

/**
 * @brief For reconfiguring plugin, when model was disabled.
 * Body Footprint of leg(s) will be set.
 */
  void reconfigure() override;


  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;
  int GetAgent(int agentId, pedsim_msgs::AgentState &agent);
     /**
   * @name          AfterPhysicsStep
   * @brief         override the AfterPhysicsStep method
   * @param[in]     config The plugin YAML node
   */

  void AfterPhysicsStep(const Timekeeper& timekeeper) override;



  private: 
      enum LEG
    {
        LEFT,
        RIGHT
    };
    UpdateTimer update_timer_;              ///< for controlling update rate
    UpdateTimer leg_timer_;                 ///< for controlling step size
    
    b2Body * body_;                         ///< Pointer to base-body
    b2Body * left_leg_body_;                ///< Pointer to left_leg-body
    b2Body * right_leg_body_;               ///< Pointer to right_leg-body
    b2Body * safety_dist_b2body_;               ///< Pointer to safety distance circle
    Body * safety_dist_body_;               ///< Pointer to safety distance circle

    ModelBody * left_leg_body_test_;        ///< Pointer to left_leg-body
    ModelBody * right_leg_body_test_;       ///< Pointer to right_leg-body

    pedsim_msgs::AgentState person;
    ros::Subscriber pedsim_agents_sub_;     ///< Subscriber to pedsim agents state
    ros::Publisher agent_state_pub_;          ///< Publisher for agent state of  every pedsim agent

    tf::TransformListener listener_;        ///< Transform Listner

    // Recent agent state
    pedsim_msgs::AgentStatesConstPtr agents_;///< most recent pedsim agent state
    // std::queue<pedsim_msgs::AgentStatesConstPtr> q_agents_;

    double leg_offset_;                      ///< offset between the legs
    bool toggle_leg_movement_;               ///< if true: legs are moving. if false: legs stand still
    int state_;                              ///< state of leg movement
    bool init_;
    double leg_radius_;
    double safety_dist_;
    double safety_dist_original_;
    std::string body_frame_;                  ///< frame name of base-body

    flatland_plugins::TriangleProfile* wp_;

    /**
     * @brief Callback for pedsim agent topic
     * @param[in] agents array of all agents
     */
    void agentCallback(const pedsim_msgs::AgentStatesConstPtr& agents);
    
    /**
     * @brief Reset leg position to position [x, y] considering the leg_offset_
     */
    void resetLegPosition(float32 x, float32 y, float32 angle);

    /**
     * @brief Set velocity [vel_x, vel_y] to right leg, stop left leg
     */
    void moveRightLeg(float32 vel_x, float32 vel_y, float32 angle_diff);

    /**
     * @brief Set velocity [vel_x, vel_y] to left leg, stop right leg
     */
    void moveLeftLeg(float32 vel_x, float32 vel_y, float32 angle_diff);

    /**
     * @brief Method is copy of model_body.cpp to be able to change radius programatically
     * ToDo: Find more elegeant solution!
     */
    void set_circular_footprint(b2Body * physics_body_, double radius);

    /**
     * @brief Method is copy of model_body.cpp to be able to change radius programatically
     * ToDo: Find more elegeant solution!
     */
    void set_safety_dist_footprint(b2Body * physics_body_, double radius);

    /**
     * @brief Method is copy of model_body.cpp to be able to change radius programatically
     * ToDo: Find more elegeant solution!
     */
    void ConfigFootprintDef(b2FixtureDef &fixture_def);

    /**
     * @brief To be able to change radius programatically
     */
    void ConfigFootprintDefSafetyDist(b2FixtureDef &fixture_def); 

  /**
   * @brief update safety distance circle, when the agent is chatting.
   * Body Footprint of safety dist circle will be set.
   */
    void updateSafetyDistance();

};
};

#endif
