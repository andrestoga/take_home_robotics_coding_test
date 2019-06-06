/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-05-30 19:25:47
 */

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "ros/ros.h"
#include "cell_dijk.h"
#include <algorithm>

namespace multi_agent_plan
{
  class Dijkstra : public DetPlanner2D<CellDijk>
  {
  public:

    std::vector<CellDijk*> open_;
    std::vector<CellDijk*> closed_;

    Dijkstra( int width, int height, bool is_edge_quartile = true );
    Dijkstra( nav_msgs::OccupancyGrid& ogm, bool is_edge_quartile = true );
    Dijkstra(bool is_edge_quartile = true){}

    virtual std::vector<geometry_msgs::Pose2D> pathPlanning( geometry_msgs::Pose2D start_pose, geometry_msgs::Pose2D goal ) override;

  };

  Dijkstra::Dijkstra( int width, int height, bool is_edge_quartile )
  : DetPlanner2D<CellDijk>( width, height, is_edge_quartile )
  {
  }

  Dijkstra::Dijkstra( nav_msgs::OccupancyGrid& ogm, bool is_edge_quartile )
  : DetPlanner2D<CellDijk>( ogm, is_edge_quartile )
  {
    // map_->loadMapFromOccGridMap( ogm );
  }

  // Dijkstra planning
  std::vector<geometry_msgs::Pose2D> Dijkstra::pathPlanning( geometry_msgs::Pose2D start, geometry_msgs::Pose2D goal )
  {
    multi_agent_plan::CellDijk* start_cell = map_->getCell( start.x, start.y );
    start_cell->g_ = 0.0;

    open_.push_back( start_cell );

    while( !open_.empty() )
    {
      multi_agent_plan::CellDijk* cell = open_.front();
      std::pop_heap( open_.begin(), open_.end() );
      open_.pop_back();

      closed_.push_back( cell );

      if ( cell->pose_.x == goal.x && cell->pose_.y == goal.y )
      {
        return extractPath( cell );
      }

      std::vector<CellDijk*> neighbors = map_->expand( cell->pose_.x, cell->pose_.y );

      for (int i = 0; i < neighbors.size(); ++i)
      {
        CellDijk* tmp = neighbors[i];

        if ( -1 == tmp->g_ )
        {
          tmp->parent_ = cell;
          // TODO: Here the cost could be 1.4 for the diagonally
          tmp->g_ = cell->g_ + 1;
          open_.push_back( tmp );
          std::push_heap( open_.begin(), open_.end() );
        }
        else if ( ( cell->g_ + 1 ) < tmp->g_ ) // TODO: Here the cost could be 1.4 for the diagonally
        {
          tmp->g_ = cell->g_ + 1;
          tmp->parent_ = cell;
          std::make_heap( closed_.begin(), closed_.end(), [](const CellDijk* a, const CellDijk* b)
          {
            return a->g_ > b->g_;
          } );
        }
      }
    }

    std::vector<geometry_msgs::Pose2D> failure;

    return failure;
  }
}

#endif