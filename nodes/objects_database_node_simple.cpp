/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

// Author(s): Matei Ciocarlie

//! Wraps around the most common functionality of the objects database and offers it
//! as ROS services

#include <vector>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include "moveit/robot_model/robot_model.h"
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <visualization_msgs/MarkerArray.h>

#include <manipulation_msgs/GraspPlanning.h>
#include <manipulation_msgs/GraspPlanningAction.h>

#include "household_objects_database/objects_database.h"
#include <household_objects_database_msgs/TranslateRecognitionId.h>
#include <household_objects_database_msgs/GetModelList.h>

#include <tf/transform_datatypes.h>

const std::string GRASP_PLANNING_SERVICE_NAME = "database_grasp_planning";
const std::string GET_MODELS_SERVICE_NAME = "get_model_list";
const std::string VISUALIZE_GRASPS = "visualize_grasps";

using namespace household_objects_database_msgs;
using namespace household_objects_database;
using namespace manipulation_msgs;

geometry_msgs::Vector3 negate(const geometry_msgs::Vector3 &vec)
{
  geometry_msgs::Vector3 v;
  v.x = - vec.x;
  v.y = - vec.y;
  v.z = - vec.z;
  return v;
}

bool greaterScaledQuality(const boost::shared_ptr<DatabaseGrasp> &grasp1, 
			  const boost::shared_ptr<DatabaseGrasp> &grasp2)
{
  if (grasp1->scaled_quality_.data() > grasp2->scaled_quality_.data()) return true;
  return false;
}

//! Wraps around database connection to provide database-related services through ROS
/*! Contains very thin wrappers for getting a list of scaled models and for getting the mesh
  of a model, as well as a complete server for the grasp planning service */
class ObjectsDatabaseNode
{
private:
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;

  //! Node handle in the root namespace
  ros::NodeHandle root_nh_;

  //! Server for the get grasps service
  ros::ServiceServer grasp_planning_srv_;

  ros::Publisher visualize_grasps_publisher_;
  
  //! The database connection itself
  ObjectsDatabase *database_;

  std::map<std::string, geometry_msgs::Vector3> approach_direction_;

  std::map<std::string, std::string> database_hand_ids_;
  
  //! Transform listener
  //  tf::TransformListener listener_;

  //! How to order grasps received from database.
  /*! Possible values: "random" or "quality" */
  std::string grasp_ordering_method_;

  const robot_model::RobotModelConstPtr &robot_model_;  

  //! Callback for the get models service
  bool getModelsCB(GetModelList::Request &request, 
                   GetModelList::Response &response)
  {
    if (!database_)
    {
      response.return_code.code = response.return_code.DATABASE_NOT_CONNECTED;
      return true;
    }
    std::vector< boost::shared_ptr<DatabaseScaledModel> > models;
    if (!database_->getScaledModelsBySet(models, request.model_set))
    {
      response.return_code.code = response.return_code.DATABASE_QUERY_ERROR;
      return true;
    }
    for (size_t i=0; i<models.size(); i++)
    {
      response.model_ids.push_back( models[i]->id_.data() );
    }
    response.return_code.code = response.return_code.SUCCESS;
    return true;
  }

  inline std::vector<double> getVectorDoubleParam(const std::string &name)
  {
    std::vector<double> values;
    XmlRpc::XmlRpcValue list;
    if (!root_nh_.getParamCached(name, list))
    {
      ROS_ERROR("Could not find parameter %s", name.c_str());
      return values;
    }
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Bad parameter type %s", name.c_str());
      return values;
    }
    for (int32_t i=0; i<list.size(); i++)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
      {
        ROS_ERROR("Bad parameter %s", name.c_str());
      }
      values.push_back( static_cast<double>(list[i]) );
    }
    return values;
  }

  inline geometry_msgs::Vector3 approachDirection(const std::string &name, bool &found)
  {
    geometry_msgs::Vector3 app;
    std::vector<double> values = getVectorDoubleParam(name);
    if ( values.size() != 3 ) 
    {
      ROS_ERROR("Bad parameter name %s", name.c_str());
      found = false;      
      return app;
    }
    double length = sqrt( values[0]*values[0] + values[1]*values[1] + values[2]*values[2] );
    if ( fabs(length) < 1.0e-5 )
    {
      ROS_ERROR("Bad parameter name %s", name.c_str());
      found = false;      
      return app;
    }
    app.x = values[0] / length;
    app.y = values[1] / length;
    app.z = values[2] / length;
    found = true;    
    return app;
  }

  geometry_msgs::Pose multiplyPoses(const geometry_msgs::Pose &p1, 
                                    const geometry_msgs::Pose &p2)
  {
    tf::Transform t1;
    tf::poseMsgToTF(p1, t1);
    tf::Transform t2;
    tf::poseMsgToTF(p2, t2);
    t2 = t1 * t2;        
    geometry_msgs::Pose out_pose;
    tf::poseTFToMsg(t2, out_pose);
    return out_pose;
  }

  bool translateIdCB(TranslateRecognitionId::Request &request, TranslateRecognitionId::Response &response)
  {
    std::vector<boost::shared_ptr<DatabaseScaledModel> > models;
    if (!database_)
    {
      ROS_ERROR("Database not connected");
      response.result = response.DATABASE_ERROR;
      return true;
    }
    if (!database_->getScaledModelsByRecognitionId(models, request.recognition_id))
    {
      ROS_ERROR("Query failed");
      response.result = response.DATABASE_ERROR;
      return true;
    }
    if (models.empty())
    {
      ROS_ERROR("Recognition id %s not found", request.recognition_id.c_str());
      response.result = response.ID_NOT_FOUND;
      return true;
    }
    response.household_objects_id = models[0]->id_.data();
    if (models.size() > 1) ROS_WARN("Multiple matches found for recognition id %s. Returning the first one.",
                                    request.recognition_id.c_str());
    response.result = response.SUCCESS;
    return true;
  }

  void visualizeGrasps(const std::vector<Grasp> &grasps)
  {    
    visualization_msgs::MarkerArray marker;
    for(std::size_t i=0; i < grasps.size(); ++i)
    {
      visualization_msgs::Marker m;
      m.action = m.ADD;      
      m.type = m.ARROW;
      m.ns = "grasps";
      m.id = i;
      m.pose = grasps[i].grasp_pose.pose;
      m.header = grasps[i].grasp_pose.header;
      marker.markers.push_back(m);      

      m.scale.x = 0.05;
      m.scale.y = 0.01;
      m.scale.z = 0.01;

      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;

      marker.markers.push_back(m);      
    }
    visualize_grasps_publisher_.publish(marker);
  }
  
  //retrieves all grasps from the database for a given target
  bool getGrasps(const GraspableObject &target, const std::string &arm_name, 
                 std::vector<Grasp> &grasps, GraspPlanningErrorCode &error_code)
  {
    if (!database_)
    {
      ROS_ERROR("Database grasp planning: database not connected");
      error_code.value = error_code.OTHER_ERROR;
      return false;
    }
    std::vector< boost::shared_ptr<DatabaseScaledModel> > models;
    /*    if (database_->getScaledModelsList(models))
    {
      for (size_t i=0; i<models.size(); i++)
	{
	  ROS_INFO("Model id: %d", models[i]->id_.data());
	}
	}*/

    if (target.potential_models.empty())
    {
      ROS_ERROR("Database grasp planning: no potential model information in grasp planning target");
      error_code.value = error_code.OTHER_ERROR;
      return false;      
    }

    if (target.potential_models.size() > 1)
    {
      ROS_WARN("Database grasp planning: target has more than one potential models. "
               "Returning grasps for first model only");
    }

    // get the object id. If the call has specified a new, ORK-style object type, convert that to a
    // household_database model_id.
    int model_id = target.potential_models[0].model_id;
    if (!target.potential_models[0].type.key.empty())
    {
      if (model_id == 0)
      {
        TranslateRecognitionId translate;
        translate.request.recognition_id = target.potential_models[0].type.key;
        translateIdCB(translate.request, translate.response);
        if (translate.response.result == translate.response.SUCCESS)
        {
          model_id = translate.response.household_objects_id;
          ROS_DEBUG_STREAM("Grasp planning: translated ORK key " << target.potential_models[0].type.key << 
                           " into model_id " << model_id);
        }
        else
        {
          ROS_ERROR("Failed to translate ORK key into household model_id");
          error_code.value = error_code.OTHER_ERROR;
          return false;      
        }
      }
      else
      {
        ROS_WARN("Grasp planning: both model_id and ORK key specified in GraspableObject; using model_id and ignoring ORK key");
      }
    }
    //    HandDescription hd;
    std::vector<std::string> hand_ids = robot_model_->getJointModelGroup(arm_name)->getAttachedEndEffectorNames();
    
    for(std::size_t i=0; i < hand_ids.size(); ++i)
    {      
      std::map<std::string, std::string>::const_iterator database_hand_id 
        = database_hand_ids_.find(hand_ids[i]);

      if(database_hand_id == database_hand_ids_.end())
      {        
        ROS_INFO("Could not find database hand id for %s", hand_ids[i].c_str());        
        return false;
      }
      
      
      std::map<std::string, geometry_msgs::Vector3>::const_iterator approach_direction = 
        approach_direction_.find(hand_ids[i]);
      
      if(approach_direction == approach_direction_.end())
      {        
        ROS_INFO("Could not find database hand id for %s", hand_ids[i].c_str());        
        return false;
      }

      const std::pair<std::string, std::string>& end_effector_parent = 
        robot_model_->getEndEffector(hand_ids[i])->getEndEffectorParentGroup();

      //retrieve the raw grasps from the database
      std::vector< boost::shared_ptr<DatabaseGrasp> > db_grasps;
      if (!database_->getClusterRepGrasps(model_id, database_hand_id->second, db_grasps))
      {
        ROS_ERROR("Database grasp planning: database query error");
        error_code.value = error_code.OTHER_ERROR;
        return false;
      }
      ROS_INFO("Database object node: retrieved %u grasps from database", (unsigned int)db_grasps.size());
      
      //order grasps based on request
      if (grasp_ordering_method_ == "random") 
      {
        ROS_INFO("Randomizing grasps");
        std::random_shuffle(db_grasps.begin(), db_grasps.end());
      }
      else if (grasp_ordering_method_ == "quality")
      {
        ROS_INFO("Sorting grasps by scaled quality");
        std::sort(db_grasps.begin(), db_grasps.end(), greaterScaledQuality);
      }
      else
      {
        ROS_WARN("Unknown grasp ordering method requested -- randomizing grasp order");
        std::random_shuffle(db_grasps.begin(), db_grasps.end());
      }
      
      //convert to the Grasp data type
      std::vector< boost::shared_ptr<DatabaseGrasp> >::iterator it;
      for (it = db_grasps.begin(); it != db_grasps.end(); it++)
      {
        ROS_ASSERT( (*it)->final_grasp_posture_.get().joint_angles_.size() == 
                    (*it)->pre_grasp_posture_.get().joint_angles_.size() );
        Grasp grasp;
        std::vector<std::string> joint_names = robot_model_->getEndEffector(hand_ids[i])->getJointModelNames();
        

        if (database_hand_id->second != "WILLOW_GRIPPER_2010")
        {	  
          //check that the number of joints in the ROS description of this hand
          //matches the number of values we have in the database
          if (joint_names.size() != (*it)->final_grasp_posture_.get().joint_angles_.size())
          {
            ROS_ERROR("Database grasp specification does not match ROS description of hand. "
                      "Hand is expected to have %d joints, but database grasp specifies %d values", 
                      (int)joint_names.size(), (int)(*it)->final_grasp_posture_.get().joint_angles_.size());
            continue;
          }
          //for now we silently assume that the order of the joints in the ROS description of
          //the hand is the same as in the database description
          grasp.pre_grasp_posture.name = joint_names;
          grasp.grasp_posture.name = joint_names;
          grasp.pre_grasp_posture.position = (*it)->pre_grasp_posture_.get().joint_angles_;
          grasp.grasp_posture.position = (*it)->final_grasp_posture_.get().joint_angles_;	
        }
        else
        {
	  // AWFUL HACK
	  std::vector<std::string> joint_names_l;
	  joint_names_l.push_back(joint_names[2]);
	  joint_names_l.push_back(joint_names[3]);
	  joint_names_l.push_back(joint_names[4]);
	  joint_names_l.push_back(joint_names[5]);
	  //	  for(std::size_t k =0; k < joint_names.size(); ++k)
	  //	    ROS_INFO("Joint names: %d %s", k, joint_names[k].c_str());
	  joint_names = joint_names_l;
          //unfortunately we have to hack this, as the grasp is really defined by a single
          //DOF, but the urdf for the PR2 gripper is not well set up to do that
          if ( joint_names.size() != 4 || (*it)->final_grasp_posture_.get().joint_angles_.size() != 1)
          {
            ROS_ERROR("PR2 gripper specs and database grasp specs do not match expected values");
            continue;
          }
          grasp.pre_grasp_posture.name = joint_names;
          grasp.grasp_posture.name = joint_names;
          //replicate the single value from the database 4 times
          grasp.pre_grasp_posture.position.resize( joint_names.size(), 
                                                   (*it)->pre_grasp_posture_.get().joint_angles_.at(0));
          grasp.grasp_posture.position.resize( joint_names.size(), 
                                               (*it)->final_grasp_posture_.get().joint_angles_.at(0));
        }
        //for now the effort is not in the database so we hard-code it in here
        //this will change at some point
        grasp.grasp_posture.effort.resize(joint_names.size(), 50);
        grasp.pre_grasp_posture.effort.resize(joint_names.size(), 100);
        //approach and retreat directions are based on pre-defined hand characteristics
        //note that this silently implies that the same convention was used by whatever planner (or human) stored
        //this grasps in the database to begin with
        grasp.approach.direction.header.stamp = ros::Time::now();
        grasp.approach.direction.header.frame_id = end_effector_parent.second;
        grasp.approach.direction.vector = approach_direction->second;
        grasp.retreat.direction.header.stamp = ros::Time::now();
        grasp.retreat.direction.header.frame_id = end_effector_parent.second;
        grasp.retreat.direction.vector = negate( approach_direction->second );      
        //min and desired approach distances are the same for all grasps
        grasp.approach.desired_distance = 0.10;
        grasp.approach.min_distance = 0.05;
        grasp.retreat.desired_distance = 0.10;
        grasp.retreat.min_distance = 0.05;
        //the pose of the grasp
        geometry_msgs::Pose grasp_pose = (*it)->final_grasp_pose_.get().pose_;
        //convert it to the frame of the detection
        grasp_pose = multiplyPoses(target.potential_models[0].pose.pose, grasp_pose);
        //store the scaled quality
        grasp.grasp_quality = (*it)->scaled_quality_.get();
        grasp.grasp_pose.pose = grasp_pose;
        grasp.grasp_pose.header.frame_id = target.reference_frame_id;
        grasp.grasp_pose.header.stamp = ros::Time::now();
        
        //insert the new grasp in the list
        grasps.push_back(grasp);
      }
      visualizeGrasps(grasps);      
      ROS_INFO("Database grasp planner: returning %u grasps for end-effector %s"
               , (unsigned int) grasps.size(), hand_ids[i].c_str());
    }    
    error_code.value = error_code.SUCCESS;
    return true;
  }

  //! Callback for the get grasps service
  bool graspPlanningCB(GraspPlanning::Request &request, GraspPlanning::Response &response)
  {
    getGrasps(request.target, request.arm_name, response.grasps, response.error_code);
    return true;
  }

public:
  ObjectsDatabaseNode(const robot_model::RobotModelConstPtr &robot_model) : priv_nh_("~"), root_nh_(""), robot_model_(robot_model)
  {
    //initialize database connection
    ROS_INFO("Starting up");
    std::string database_host, database_port, database_user, database_pass, database_name;
    root_nh_.param<std::string>("/household_objects_database/database_host", database_host, "");
    int port_int;
    root_nh_.param<int>("/household_objects_database/database_port", port_int, -1);
    std::stringstream ss; ss << port_int; database_port = ss.str();
    root_nh_.param<std::string>("/household_objects_database/database_user", database_user, "");
    root_nh_.param<std::string>("/household_objects_database/database_pass", database_pass, "");
    root_nh_.param<std::string>("/household_objects_database/database_name", database_name, "");

    visualize_grasps_publisher_ = root_nh_.advertise<visualization_msgs::MarkerArray>("visualize_grasps", 20);

    database_ = new ObjectsDatabase(database_host, database_port, database_user, database_pass, database_name);
    if (!database_->isConnected())
    {
      ROS_ERROR("ObjectsDatabaseNode: failed to open model database on host "
		"%s, port %s, user %s with password %s, database %s. Unable to do grasp "
		"planning on database recognized objects. Exiting.",
		database_host.c_str(), database_port.c_str(), 
		database_user.c_str(), database_pass.c_str(), database_name.c_str());
      delete database_; database_ = NULL;
    }

    //advertise services
    grasp_planning_srv_ = root_nh_.advertiseService(GRASP_PLANNING_SERVICE_NAME, 
						    &ObjectsDatabaseNode::graspPlanningCB, this);

    priv_nh_.param<std::string>("grasp_ordering_method", grasp_ordering_method_, "random");
    ROS_INFO("Database connected");
    for(std::size_t i=0; i < robot_model_->getJointModelGroupNames().size(); ++i)
    {
      ROS_INFO("%d %s", i, robot_model_->getJointModelGroupNames()[i].c_str());
      geometry_msgs::Vector3 default_approach_direction;
      default_approach_direction.x = 1.0; 
      default_approach_direction.y = 0.0;
      default_approach_direction.z = 0.0;      
      const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(robot_model_->getJointModelGroupNames()[i]);
      if(jmg && jmg->isEndEffector())
      {
        std::string database_id;        
        root_nh_.param<std::string>(jmg->getEndEffectorName()+"/database_id", database_id, "");
        if(!database_id.empty())
	{
          database_hand_ids_[jmg->getEndEffectorName()] = database_id;        
	  ROS_INFO("Found database id: %s for %s", database_id.c_str(), jmg->getEndEffectorName().c_str());
	}
        bool found = false;        
        geometry_msgs::Vector3 approach_direction = 
          approachDirection(jmg->getEndEffectorName()+"/approach_direction", found);
        if(found)
	{
          approach_direction_[jmg->getEndEffectorName()] = approach_direction;        
	  ROS_INFO("Found approach directions for %s: %f %f %f", 
		   jmg->getEndEffectorName().c_str(),
		   approach_direction.x,
		   approach_direction.y,
		   approach_direction.z);
	}
	//        else
	//          approach_direction_[jmg->getEndEffectorName()] = default_approach_direction;                  
      }      
    }    
  }

  ~ObjectsDatabaseNode()
  {
    delete database_;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects_database_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); 

  ROS_INFO("Loaded robot model");  
  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();    
  ROS_INFO("Loaded robot model");  
  
  ObjectsDatabaseNode node(robot_model);

  ros::waitForShutdown();
  return 0;  
}
