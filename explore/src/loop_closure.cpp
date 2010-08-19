/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *
 *********************************************************************/


/*
 * Implement the loop-closure exploration algorithm, from Stachniss, et
 * al., IROS 2004.
 */

#include <float.h>

#include <ros/console.h>
#include <explore/loop_closure.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

using namespace explore;

LoopClosure::LoopClosure(double addition_dist_min, 
                         double loop_dist_min, 
                         double loop_dist_max, 
                         double slam_entropy_max_,
                         double graph_update_frequency, 
                         actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& move_base_client,
                         costmap_2d::Costmap2DROS& costmap,
                         boost::mutex& control_mutex) :
        curr_node_(NULL), 
        addition_dist_min_(addition_dist_min), 
        loop_dist_min_(loop_dist_min),
        loop_dist_max_(loop_dist_max),
        slam_entropy_max_(slam_entropy_max_),
        graph_update_frequency_(graph_update_frequency),
        move_base_client_(move_base_client),
        nh_(),
        marker_id_(0),
        costmap_(costmap),
        control_mutex_(control_mutex),
        planner_(NULL),
        slam_entropy_(0.0),
        slam_entropy_time_(ros::Time::now().toSec())
{
  marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);
  entropy_subscriber_ = nh_.subscribe("slam_entropy", 1, &LoopClosure::entropyCallback, this);
  planner_ = new navfn::NavfnROS(std::string("loop_closure_planner"), &costmap_);
}

LoopClosure::~LoopClosure()
{
  delete planner_;
}

void
LoopClosure::entropyCallback(const std_msgs::Float64::ConstPtr& entropy)
{
  slam_entropy_ = entropy->data;
  slam_entropy_time_ = ros::Time::now().toSec();
  ROS_DEBUG("Entropy is: %f (%f)", slam_entropy_, slam_entropy_max_);
}

void 
LoopClosure::updateGraph(const tf::Pose& pose)
{
  // Try to add a node
  addNode(pose);

  double time = ros::Time::now().toSec();
  if ((time - slam_entropy_time_) > 5.0f) {
    ROS_WARN("Entropy has not been updated. Loop closure opportunities may be ignored.");
    slam_entropy_time_ = time;
  }

  // Look for an opportunity to close a loop
  if(slam_entropy_ > 0.0 && slam_entropy_ > slam_entropy_max_)
  {
    ROS_DEBUG("Entropy is high enough (%.3f > %.3f); checking for loop closure opportunities",
              slam_entropy_, slam_entropy_max_);
    std::vector<GraphNode*> candidates;
    if(checkLoopClosure(pose, candidates))
    {
      // We found some. Time to control the robot
      control_mutex_.lock();
      // Control the robot
      ROS_INFO("Taking control of the robot for loop closure goals.");

      double min_path_length = DBL_MAX;
      GraphNode* entry_point = candidates[0];

      // Take the candidate node that is closest to our current position as the entry point
      for(unsigned int i = 0; i < candidates.size(); ++i){
        if(candidates[i]->path_length_ < min_path_length){
          entry_point = candidates[i];
          min_path_length = candidates[i]->path_length_;
        }
      }
      
      // We may end up going to several nodes; the first one is the entry
      // point.
      GraphNode* curr_target = entry_point;

      //the node that we'll connect to the target
      GraphNode* connect_node = curr_node_;
      
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      // We retrace our steps as long as entropy is higher than what it was
      // last time we were here.
      do
      {
        ROS_INFO("Loop closure: Sending the robot to %.3f %.3f (%d)",
                 curr_target->pose_.getOrigin().x(),
                 curr_target->pose_.getOrigin().y(),
                 curr_target->id_);
        // TODO: pick a better orientation on the goal pose
        tf::poseTFToMsg(curr_target->pose_, goal.target_pose.pose);
        move_base_client_.sendGoal(goal);
        ros::Rate r(graph_update_frequency_);


        while(!move_base_client_.getState().isDone()){
          //get the current pose of the robot and check if
          tf::Stamped<tf::Pose> robot_pose;
          costmap_.getRobotPose(robot_pose);

          addNode(robot_pose);

          // Compute shortest distances from current node to all nodes.  Distances
          // are stored in nodes themselves.
          dijkstra(curr_node_->id_);

          //now we'll check if we are still far enough topologically from the loop closure target
          //if we are, we'll update the node that we'll connect with the loop closure
          if(curr_target->dijkstra_d_ > loop_dist_max_){
            connect_node = curr_node_;
          }

          visualizeGraph();
          r.sleep();
        }
        // TODO: inspect the result

        // Now go to the node that we went to last time we were here
        curr_target = curr_target->next_;
      } while((curr_target != NULL) && (slam_entropy_ > entry_point->slam_entropy_));

      // We need to connect the last node we added to the entry point in order to log that we've closed the loop
      if(connect_node){
        graph_[connect_node->id_].push_back(entry_point->id_);
        graph_[entry_point->id_].push_back(connect_node->id_);
        ROS_INFO("Adding edge from connector node to entry point");
      }

      control_mutex_.unlock();
      ROS_INFO("Entropy threshold satisfied (%.3f <= %.3f); loop closure terminated",
               slam_entropy_, entry_point->slam_entropy_);
    }
  }

  //actually display the graph in rviz
  visualizeGraph();
}

void 
LoopClosure::addNode(const tf::Pose& pose)
{
  bool add = false;
  // If there are no nodes yet, add one now
  if(curr_node_ == NULL)
    add = true;
  else
  {
    geometry_msgs::PoseStamped planner_start;
    geometry_msgs::Pose temp_pose;
    tf::poseTFToMsg(pose, temp_pose);
    planner_start.header.stamp = ros::Time::now();
    planner_start.header.frame_id = costmap_.getGlobalFrameID();
    planner_start.pose = temp_pose;

    //we'll only compute the potential once from our position
    planner_->computePotential(planner_start.pose.position);

    add = true;
    
    // How far are we from the closest node?
    double mind = DBL_MAX;
    GraphNode* closest_node = NULL;
    for(int i = nodes_.size() - 1; i >= 0; --i)
    {
      GraphNode* index_node = nodes_[i];

      //we need to convert from tf to message types for the planner
      geometry_msgs::PoseStamped planner_end;

      tf::poseTFToMsg(index_node->pose_, temp_pose);
      planner_end.header.stamp = ros::Time::now();
      planner_end.header.frame_id = costmap_.getGlobalFrameID();
      planner_end.pose = temp_pose;

      //now we'll make a plan from one point to the other
      std::vector<geometry_msgs::PoseStamped> plan;
      bool valid_plan = planner_->getPlanFromPotential(planner_end, plan) && !plan.empty();

      // Graph distance check satisfied; compute map distance
      if(valid_plan){
        double path_length = 0.0;

        //compute the path length
        if(plan.size() > 1){
          //double dist = distance(plan[0], plan[1]);
          //path_length = dist * (plan.size() - 1);

          for(unsigned int j = 0; j < plan.size() - 1; ++j){
            double dist = distance(plan[j], plan[j + 1]);
            path_length += dist;
          }

          if(path_length < mind){
            mind = path_length;
            closest_node = index_node;

            //we'll also update our current node here because we always want to add ourselves as a node to the 
            //closest node
            curr_node_ = closest_node;
          }
        }

      }
    }
    if(mind < addition_dist_min_)
    {
      // We found a close-enough node, so we won't add one, unless we lost visibility
      add = false;

      /*
      for(int i = nodes_.size() - 1; i >= 0; --i)
      {
        GraphNode* index_node = nodes_[i];

        //we also need to check if we have visibility from our current node to the previous node
        bool is_visible = true;
        VisibilityChecker checker(visibility_map.getCharMap(), is_visible);

        //get map coordinates
        bool in_bounds = true;
        unsigned int x0, y0;
        if(!visibility_map.worldToMap(pose.getOrigin().x(), pose.getOrigin().y(), x0, y0)){
          ROS_WARN("Attempting to check visibility from a point off the map... this should never happen");
          in_bounds = false;
        }

        unsigned int x1, y1;
        if(!visibility_map.worldToMap(index_node->pose_.getOrigin().x(), index_node->pose_.getOrigin().y(), x1, y1)){
          ROS_WARN("Attempting to check visibility to a point off the map... this should never happen");
          in_bounds = false;
        }

        if(in_bounds){
          //raytrace a line with our visibility checker
          raytraceLine(checker, x0, y0, x1, y1, visibility_map.getSizeInCellsX());

          //check if we have visibility to the node
          if(is_visible){
            add = false;
            break;
          }
        }
      }
      */
    
      
      // Update current node to be this one.
      curr_node_ = closest_node;

      // Store the current slam entropy; we compare to it later to decide
      // to terminate loop closure.
      if(slam_entropy_ > 0.0)
        curr_node_->slam_entropy_ = slam_entropy_;
    }
  }

  if(add)
  {
    ROS_INFO("Adding node at %.3f, %.3f", pose.getOrigin().x(), pose.getOrigin().y());
    GraphNode* n = new GraphNode(nodes_.size(), pose);
    nodes_.push_back(n);
    // Add an empty adjacency list for the new node
    std::vector<int> edges;
    graph_.push_back(edges);

    if(curr_node_)
    {
      // Update both adjacency lists
      graph_[curr_node_->id_].push_back(n->id_);
      graph_[n->id_].push_back(curr_node_->id_);
      curr_node_->next_ = n;
    }
    curr_node_ = n;

    // Store the current slam entropy; we compare to it later to decide
    // to terminate loop closure.
    if(slam_entropy_ > 0.0)
      curr_node_->slam_entropy_ = slam_entropy_;
  }
}

bool
LoopClosure::checkLoopClosure(const tf::Pose& pose, std::vector<GraphNode*>& candidates)
{
  bool ret = false;

  if(curr_node_ == NULL || nodes_.size() < 2)
    return ret;

  // Compute shortest distances from current node to all nodes.  Distances
  // are stored in nodes themselves.
  dijkstra(curr_node_->id_);

  //we'll only compute the potential once
  geometry_msgs::Pose temp_pose;
  geometry_msgs::PoseStamped planner_start;

  tf::poseTFToMsg(pose, temp_pose);
  planner_start.header.stamp = ros::Time::now();
  planner_start.header.frame_id = costmap_.getGlobalFrameID();
  planner_start.pose = temp_pose;

  planner_->computePotential(planner_start.pose.position);

  //get a copy of the costmap that we can do raytracing on
  costmap_2d::Costmap2D visibility_map;
  costmap_.getCostmapCopy(visibility_map);

  for(std::vector<GraphNode*>::iterator it = nodes_.begin();
      it != nodes_.end();
      ++it)
  {

    ROS_DEBUG("Checking %d (%.3f, %.3f)", (*it)->id_, (*it)->pose_.getOrigin().x(), (*it)->pose_.getOrigin().y());
    ROS_DEBUG("Graph distance is %.3f (threshold is %.3f)", (*it)->dijkstra_d_, loop_dist_max_);

    if((*it)->dijkstra_d_ > loop_dist_max_)
    {
      ROS_DEBUG("graph dist check satisfied");
      //we need to convert from tf to message types for the planner
      geometry_msgs::PoseStamped planner_end;

      tf::poseTFToMsg((*it)->pose_, temp_pose);
      planner_end.header.stamp = ros::Time::now();
      planner_end.header.frame_id = costmap_.getGlobalFrameID();
      planner_end.pose = temp_pose;

      //now we'll make a plan from one point to the other
      std::vector<geometry_msgs::PoseStamped> plan;
      bool valid_plan = planner_->getPlanFromPotential(planner_end, plan) && !plan.empty();

      // Graph distance check satisfied; compute map distance
      if(valid_plan){
        double path_length = 0.0;

        //compute the path length
        if(plan.size() > 1){
          //double dist = distance(plan[0], plan[1]);
          //path_length = dist * (plan.size() - 1);

          for(unsigned int j = 0; j < plan.size() - 1; ++j){
            double dist = distance(plan[j], plan[j + 1]);
            path_length += dist;
          }

          ROS_DEBUG("path_length: %.3f (threshold is %.3f)", path_length, loop_dist_min_);
          if(path_length < loop_dist_min_)
          {
            //also check that there is visibility from one node to the other
            bool is_visible = true;
            VisibilityChecker checker(visibility_map.getCharMap(), is_visible);

            //get map coordinates
            bool in_bounds = true;
            unsigned int x0, y0;
            if(!visibility_map.worldToMap(pose.getOrigin().x(), pose.getOrigin().y(), x0, y0)){
              ROS_WARN("Attempting to check visibility from a point off the map... this should never happen");
              in_bounds = false;
            }

            unsigned int x1, y1;
            if(!visibility_map.worldToMap((*it)->pose_.getOrigin().x(), (*it)->pose_.getOrigin().y(), x1, y1)){
              ROS_WARN("Attempting to check visibility to a point off the map... this should never happen");
              in_bounds = false;
            }

            if(in_bounds){
              //raytrace a line with our visibility checker
              raytraceLine(checker, x0, y0, x1, y1, visibility_map.getSizeInCellsX());

              //check if we have visibility to the node
              if(is_visible){
                //update the path distance on the node we're about to push back
                (*it)->path_length_ = path_length;
                // Both checks satisfied
                candidates.push_back(*it);
                ROS_INFO("added loop closure candidate %d (%.3f, %.3f)", 
                    (*it)->id_, (*it)->pose_.getOrigin().x(), (*it)->pose_.getOrigin().y());
                ret = true;
              }
            }
          }
        }
      }
    }
  }

  return ret;
}

// Poor-man's (inefficient) dijkstra implementation.
void
LoopClosure::dijkstra(int source)
{
  std::list<GraphNode*> Q;
  for(std::vector<GraphNode*>::iterator it = nodes_.begin();
      it != nodes_.end();
      ++it)
  {
    if((*it)->id_ == source)
      (*it)->dijkstra_d_ = 0.0;
    else
      (*it)->dijkstra_d_ = DBL_MAX;
    Q.push_back(*it);
  }
  while(!Q.empty())
  {
    std::list<GraphNode*>::iterator it = Q.end();
    for(std::list<GraphNode*>::iterator iit = Q.begin();
        iit != Q.end();
        ++iit)
    {
      if((it == Q.end()) || ((*iit)->dijkstra_d_ < (*it)->dijkstra_d_))
      {
        it = iit;
      }
    }
    GraphNode* n = *it;
    if(n->dijkstra_d_ == DBL_MAX)
      break;
    Q.erase(it);

    for(std::vector<int>::iterator git = graph_[n->id_].begin();
        git != graph_[n->id_].end();
        ++git)
    {
      GraphNode* nn = nodes_[*git];
      double d = (n->pose_.getOrigin() - nn->pose_.getOrigin()).length();
      double alt = n->dijkstra_d_ + d;
      if(alt < nn->dijkstra_d_)
      {
        nn->dijkstra_d_ = alt;
      }
    }
  }
}

void LoopClosure::visualizeNode(const tf::Pose& pose, visualization_msgs::MarkerArray& markers)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "loop_closure";
  marker.id = marker_id_++;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = pose.getOrigin().x();
  marker.pose.position.y = pose.getOrigin().y();
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  //marker.lifetime = ros::Duration(5);
  markers.markers.push_back(marker);
  //marker_publisher_.publish(marker);
}

void LoopClosure::visualizeEdge(const tf::Pose& pose1, const tf::Pose& pose2, visualization_msgs::MarkerArray& markers)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "loop_closure";
  marker.id = marker_id_++;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  geometry_msgs::Point p;
  p.x = pose1.getOrigin().x();
  p.y = pose1.getOrigin().y();
  marker.points.push_back(p);
  p.x = pose2.getOrigin().x();
  p.y = pose2.getOrigin().y();
  marker.points.push_back(p);
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  //marker.lifetime = ros::Duration(5);
  markers.markers.push_back(marker);
  //marker_publisher_.publish(marker);
}

void LoopClosure::visualizeGraph()
{
  visualization_msgs::MarkerArray markers;
  marker_id_ = 0;
  //first we'll go through each node in the graph and  add a marker for it
  for(unsigned int i = 0; i < nodes_.size(); ++i){
    visualizeNode(nodes_[i]->pose_, markers);
  }

  //we'll also add all the edges in the graph... there may be duplicates here, but that should be ok
  for(unsigned int i = 0; i < graph_.size(); ++i){
    for(unsigned int j = 0; j < graph_[i].size(); ++j){
      visualizeEdge(nodes_[i]->pose_, nodes_[graph_[i][j]]->pose_, markers);
    }
  }

  marker_publisher_.publish(markers);
}


