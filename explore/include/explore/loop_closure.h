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

#ifndef EXPLORE_LOOP_CLOSURE_H
#define EXPLORE_LOOP_CLOSURE_H


/*
 * Implement the loop-closure exploration algorithm, from Stachniss, et
 * al., IROS 2004.
 */

#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <std_msgs/Float64.h>
#include <navfn/navfn_ros.h>

#include <ros/node_handle.h>

#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>

#include <visualization_msgs/MarkerArray.h>

namespace explore
{

class GraphNode
{
  public:
    GraphNode(int id, const tf::Pose& pose) : 
            id_(id), pose_(pose), next_(NULL), dijkstra_d_(0.0), slam_entropy_(0.0), path_length_(DBL_MAX) {}

    // id of this node in the graph
    int id_;
    // Pose of this node, in map coords
    tf::Pose pose_;
    // The node to which the robot went after this one, last time it was
    // here.
    GraphNode* next_;
    // Dijkstra distance
    double dijkstra_d_;
    // SLAM entropy last time we were here
    double slam_entropy_;
    //the last path distance computed for the node
    double path_length_;
};

class LoopClosure
{
  public:
    LoopClosure(double addition_dist_min, 
                double loop_dist_min, 
                double loop_dist_max, 
                double slam_entropy_max_,
                double graph_update_frequency,
                actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& move_base_client,
                costmap_2d::Costmap2DROS& costmap,
                boost::mutex& control_mutex);
    ~LoopClosure();

    // Call this periodically to let loop closure update its graph.  It may
    // take control of the robot, and not return until it's done.
    // @param pose : current robot pose
    void updateGraph(const tf::Pose& pose);


  private:
    // Consider adding a node (if distance and visibility constraints are
    // satisfied)
    void addNode(const tf::Pose& pose);
    bool checkLoopClosure(const tf::Pose& pose, std::vector<GraphNode*>& candidates);
    void dijkstra(int source);
    void entropyCallback(const std_msgs::Float64::ConstPtr& entropy);

    void visualizeNode(const tf::Pose& pose, visualization_msgs::MarkerArray& markers);
    void visualizeEdge(const tf::Pose& pose1, const tf::Pose& pose2, visualization_msgs::MarkerArray& markers);
    void visualizeGraph();

    inline double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
      double x1 = p1.pose.position.x;
      double x2 = p2.pose.position.x;
      double y1 = p1.pose.position.y;
      double y2 = p2.pose.position.y;
      return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    inline int sign(int x){
      return x > 0 ? 1.0 : -1.0;
    }

    /**
     * @brief  Raytrace a line and apply some action at each step
     * @param  at The action to take... a functor
     * @param  x0 The starting x coordinate
     * @param  y0 The starting y coordinate
     * @param  x1 The ending x coordinate
     * @param  y1 The ending y coordinate
     * @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
     */
    template <class ActionType>
      inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int size_x, unsigned int max_length = UINT_MAX){
        int dx = x1 - x0;
        int dy = y1 - y0;

        unsigned int abs_dx = abs(dx);
        unsigned int abs_dy = abs(dy);

        int offset_dx = sign(dx);
        int offset_dy = sign(dy) * size_x;

        unsigned int offset = y0 * size_x + x0;

        //we need to chose how much to scale our dominant dimension, based on the maximum length of the line
        double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
        double scale = std::min(1.0,  max_length / dist);

        //if x is dominant
        if(abs_dx >= abs_dy){
          int error_y = abs_dx / 2;
          bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
          return;
        }

        //otherwise y is dominant
        int error_x = abs_dy / 2;
        bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));

      }

    /**
     * @brief  A 2D implementation of Bresenham's raytracing algorithm... applies an action at each step
     */
    template <class ActionType>
      inline void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, 
          unsigned int offset, unsigned int max_length){
        unsigned int end = std::min(max_length, abs_da);
        for(unsigned int i = 0; i < end; ++i){
          at(offset);
          offset += offset_a;
          error_b += abs_db;
          if((unsigned int)error_b >= abs_da){
            offset += offset_b;
            error_b -= abs_da;
          }
        }
        at(offset);
      }

    class VisibilityChecker {
      public:
        VisibilityChecker(const unsigned char* costmap, bool& is_visible) : costmap_(costmap), is_visible_(is_visible) {}
        inline void operator()(unsigned int offset){
          unsigned char cost = costmap_[offset];
          if(cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
            is_visible_ = false;
          }
        }

      public:
        const unsigned char* costmap_;
        bool& is_visible_;
    };
    
    //The node we're currently associated with
    GraphNode* curr_node_;
    // Travel distance required to drop a new node.
    double addition_dist_min_;
    // Minimum distance in map space to try loop closure
    double loop_dist_min_;
    // Maximum distance in graph space to try loop closure
    double loop_dist_max_;
    // Maximum slam entropy we'll tolerate before trying loop closure
    double slam_entropy_max_;

    //Freqeuncy at which to update the graph when actively closing a loop
    double graph_update_frequency_;

    // Action client for commanding move base
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& move_base_client_;
    ros::NodeHandle nh_;
    // List of nodes.
    std::vector<GraphNode*> nodes_;
    // Adjacency list representation of graph
    std::vector<std::vector<int> > graph_;

    ros::Subscriber entropy_subscriber_;
    ros::Publisher marker_publisher_;
    int marker_id_;
    costmap_2d::Costmap2DROS& costmap_;
    // Mutex to lock when commanding the robot
    boost::mutex& control_mutex_;
    navfn::NavfnROS* planner_;
    double slam_entropy_;
    double slam_entropy_time_;
};

}

#endif
