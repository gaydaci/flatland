/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 model_spawner.h
 * @brief	 Definition for model spawner
 * @author Chunshang Li
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 *  Modified by Ronja Gueldenring
 */

#include <flatland_server/service_manager.h>
#include <flatland_server/types.h>
#include <flatland_msgs/Model.h>
#include <exception>

namespace flatland_server {

ServiceManager::ServiceManager(SimulationManager *sim_man, World *world)
    : world_(world), sim_man_(sim_man) {
  ros::NodeHandle nh;

  spawn_model_service_ =
      nh.advertiseService("spawn_model", &ServiceManager::SpawnModel, this);
  spawn_models_service_ =
      nh.advertiseService("spawn_models", &ServiceManager::SpawnModels, this);
  respawn_models_service_ =
      nh.advertiseService("respawn_models", &ServiceManager::RespawnModels, this);
  delete_model_service_ =
      nh.advertiseService("delete_model", &ServiceManager::DeleteModel, this);
  delete_models_service_ =
    nh.advertiseService("delete_models", &ServiceManager::DeleteModels, this);
  move_model_service_ =
      nh.advertiseService("move_model", &ServiceManager::MoveModel, this);
  pause_service_ = nh.advertiseService("pause", &ServiceManager::Pause, this);
  resume_service_ =
      nh.advertiseService("resume", &ServiceManager::Resume, this);
  toggle_pause_service_ =
      nh.advertiseService("toggle_pause", &ServiceManager::TogglePause, this);

  // step_service_ =
  //     nh.advertiseService("step", &ServiceManager::Step, this);

  // is_in_step_service_ =
  //     nh.advertiseService("is_in_step", &ServiceManager::isInStep, this);


  if (spawn_model_service_) {
    ROS_INFO_NAMED("Service Manager", "Model spawning service ready to go");
  } else {
    ROS_ERROR_NAMED("Service Manager", "Error starting model spawning service");
  }

  if (delete_model_service_) {
    ROS_INFO_NAMED("Service Manager", "Model deleting service ready to go");
  } else {
    ROS_ERROR_NAMED("Service Manager", "Error starting model deleting service");
  }

  if (move_model_service_) {
    ROS_INFO_NAMED("Service Manager", "Model moving service ready to go");
  } else {
    ROS_ERROR_NAMED("Service Manager", "Error starting model moving service");
  }
}

bool ServiceManager::SpawnModel(flatland_msgs::SpawnModel::Request &request,
                                flatland_msgs::SpawnModel::Response &response) {
  ROS_DEBUG_NAMED("ServiceManager",
                  "Model spawn requested with path(\"%s\"), namespace(\"%s\"), "
                  "name(\'%s\"), pose(%f,%f,%f)",
                  request.yaml_path.c_str(), request.ns.c_str(),
                  request.name.c_str(), request.pose.x, request.pose.y,
                  request.pose.theta);

  Pose pose(request.pose.x, request.pose.y, request.pose.theta);

  try {
    world_->LoadModel(request.yaml_path, request.ns, request.name, pose);
    response.success = true;
    response.message = "";
  } catch (const std::exception &e) {
    response.success = false;
    response.message = std::string(e.what());
    ROS_ERROR_NAMED("ServiceManager", "Failed to load model! Exception: %s",
                    e.what());
  }

  return true;
}

bool ServiceManager::SpawnModels(flatland_msgs::SpawnModels::Request &request,
                                flatland_msgs::SpawnModels::Response &response) {
  ros::WallTime start = ros::WallTime::now();
  ROS_DEBUG_NAMED("ServiceManager",
                  "Request to spawn %d models", request.models.size());
  response.success = true;
  response.message = "";
  for(int i_model=0; i_model < request.models.size(); i_model++){
    flatland_msgs::Model model = request.models[i_model];
    Pose pose(model.pose.x, model.pose.y, model.pose.theta);

    try {
      world_->LoadModel(model.yaml_path, model.ns, model.name, pose);
    } catch (const std::exception &e) {
      response.success = false;
      response.message = std::string(e.what());
      ROS_ERROR_NAMED("ServiceManager", "Failed to load model! Exception: %s",
                      e.what());
    }
  }
  ROS_WARN("Spawning models in flatland: %f", (ros::WallTime::now() - start).toSec());
  return true;
}

bool ServiceManager::RespawnModels(flatland_msgs::RespawnModels::Request &request,
                                flatland_msgs::RespawnModels::Response &response) {
  ROS_DEBUG_NAMED("ServiceManager",
                  "Respawning %d models", request.new_models.size());
  // ROS_INFO("ServiceManager,Respawning %d models", request.new_models.size());
  // ROS_INFO("Servicemanger reuse (%d) human",request.old_model_names.size()); 
   
  ros::WallTime begin = ros::WallTime::now();
  response.success = true;
  response.message = "";

  //Check if there an enabled model that can be reused (== same model type).
  for(int i_new_model=0; i_new_model < request.new_models.size(); i_new_model++){
    flatland_msgs::Model model = request.new_models[i_new_model];
    Model* existing_model = NULL;
    for(int i_old_model=0; i_old_model < request.old_model_names.size(); i_old_model++){
      // ROS_INFO("Servicemanger old model list (%s) ",request.old_model_names[i_old_model].c_str());
    }
    for(int i_old_model=0; i_old_model < request.old_model_names.size(); i_old_model++){
      Model* old_model = world_->GetModel(request.old_model_names[i_old_model]);
      
      if (old_model == NULL){
        // ROS_INFO("Servicemanger reuse (%s) but model=null ",request.old_model_names[i_old_model].c_str());
        request.old_model_names.erase(request.old_model_names.begin() + i_old_model);        
        // ROS_INFO("Servicemanger reuse (%d) but model=null",(int)i_old_model);
        continue;
      }
      if(old_model->GetYamlPath() == model.yaml_path && old_model->IsEnabled()){
        existing_model = old_model;
        // ROS_INFO("Servicemanger reuse (%s) ",request.old_model_names[i_old_model].c_str());
        request.old_model_names.erase(request.old_model_names.begin() + i_old_model);        
        // ROS_INFO("Servicemanger reuse (%d) ",(int)i_old_model);
        break;   
      }
    }
     
    Pose pose(model.pose.x, model.pose.y, model.pose.theta);

    if(existing_model == NULL){
      //Check if there exist a disabled model
      // ROS_INFO("Servicemanger reuse fisrt existing model null ");
        for (const auto &world_model : world_->models_) {
          if(!world_model->IsEnabled() &&  world_model->GetYamlPath()== model.yaml_path){
            existing_model = world_model;
            existing_model->EnableModel();
            world_->plugin_manager_.ReconfigureModelPlugin(existing_model);
            break;
          }
        }
    }

    if(existing_model == NULL){
      //Create new model
      try {
        // ROS_INFO("Servicemanger reuse second existing model null ");
        world_->LoadModel(model.yaml_path, model.ns, model.name, pose);
      } catch (const std::exception &e) {
        response.success = false;
        response.message = std::string(e.what());
        ROS_ERROR_NAMED("ServiceManager", "Failed to load model! Exception: %s",
                        e.what());
      }
    }else{
      //Simply move existing model
      try {
        existing_model->SetNameSpace(model.ns);
        world_->MoveModel(existing_model->GetName(), pose);
      } catch (const std::exception &e) {
        response.success = false;
        response.message = std::string(e.what());
      }
    }
  }

  //Disable remaining models
  //ToDO: Restrict to maximum number?
  for(int i_old_model=0; i_old_model < request.old_model_names.size(); i_old_model++){
      Model* old_model = world_->GetModel(request.old_model_names[i_old_model]);
      // if (old_model == NULL)
      //   continue;
      old_model->DisableModel();
      ROS_WARN("Servicemanger delete old model (%d) ",(int)i_old_model);
  }
  return true;
}


bool ServiceManager::DeleteModel(
    flatland_msgs::DeleteModel::Request &request,
    flatland_msgs::DeleteModel::Response &response) {
  ROS_DEBUG_NAMED("ServiceManager", "Model delete requested with name(\"%s\")",
                  request.name.c_str());

  try {
    world_->DeleteModel(request.name);
    response.success = true;
    response.message = "";
  } catch (const std::exception &e) {
    response.success = false;
    response.message = std::string(e.what());
  }

  return true;
}

bool ServiceManager::DeleteModels(
    flatland_msgs::DeleteModels::Request &request,
    flatland_msgs::DeleteModels::Response &response) {

  ros::WallTime start = ros::WallTime::now();
  ROS_DEBUG_NAMED("ServiceManager", "Deleted %d models",
                  request.name.size());
  response.success = true;
  response.message = "";
  for(int i_model = 0; i_model < request.name.size(); i_model++){
    try {
      world_->DeleteModel(request.name[i_model]);
    } catch (const std::exception &e) {
      response.success = false;
      response.message = std::string(e.what());
    }
  }
  ROS_WARN("Delete models in flatland: %f", (ros::WallTime::now() - start).toSec());
  return true;
}

bool ServiceManager::MoveModel(flatland_msgs::MoveModel::Request &request,
                               flatland_msgs::MoveModel::Response &response) {
  ros::WallTime start = ros::WallTime::now();
  ROS_DEBUG_NAMED("ServiceManager", "Model move requested with name(\"%s\")",
                  request.name.c_str());

  Pose pose(request.pose.x, request.pose.y, request.pose.theta);

  try {
    world_->MoveModel(request.name, pose);
    response.success = true;
    response.message = "";
  } catch (const std::exception &e) {
    response.success = false;
    response.message = std::string(e.what());
  }
  ROS_INFO("Move model in flatland: %f", (ros::WallTime::now() - start).toSec());

  return true;
}

bool ServiceManager::Pause(std_srvs::Empty::Request &request,
                           std_srvs::Empty::Response &response) {
  world_->Pause();
  return true;
}

bool ServiceManager::Resume(std_srvs::Empty::Request &request,
                            std_srvs::Empty::Response &response) {
  world_->Resume();
  return true;
}

bool ServiceManager::TogglePause(std_srvs::Empty::Request &request,
                                 std_srvs::Empty::Response &response) {
  world_->TogglePaused();
  return true;
}

// bool ServiceManager::Step(flatland_msgs::Step::Request &request,
//                    flatland_msgs::Step::Response &response) {
//   if (!world_->isInStep()){
//     float step_time = request.step_time.data;
//     world_->Step(step_time);
//     response.success = true;
//   }else{
//     response.success = false;
//   }
//   return true;
// }


// bool ServiceManager::isInStep(std_srvs::SetBool::Request &request,
//                                  std_srvs::SetBool::Response &response) {
//   if (world_->isInStep()){
//     response.success = true;
//   }else{
//     response.success = false;
//     response.message = std::to_string(ros::Time::now().toSec());
//   }
//   return true;
// }

};
