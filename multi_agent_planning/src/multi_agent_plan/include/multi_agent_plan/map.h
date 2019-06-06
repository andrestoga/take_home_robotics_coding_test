/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-04-30 19:09:05
 */

#ifndef MAP_H
#define MAP_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
#include <memory>

namespace multi_agent_plan
{
	template< typename T >
	class Map
	{
	public:

		bool is_edge_quartile_;
		std::unique_ptr<T[]> grid_;

		int width_;
		int height_;

		Map( int width, int height, bool is_edge_quartile = true );

		Map( nav_msgs::OccupancyGrid& map, bool is_edge_quartile = true );

		// Map(){};
		// ~Map();

		bool loadMapFromOccGridMap( nav_msgs::OccupancyGrid& map );
		bool createEmptyMap();
		T* getCell( int x, int y );
		std::vector<T*> expand( int x, int y );
	};

	// TODO: Default arguments to width and height
	template< typename T >
	Map<T>::Map( int width, int height, bool is_edge_quartile )
	: is_edge_quartile_( is_edge_quartile )
	, width_( width )
	, height_( height )
	{
		grid_ = std::unique_ptr<T[]>( new T[ width * height ] );
	}

	template< typename T >
	Map<T>::Map( nav_msgs::OccupancyGrid& map, bool is_edge_quartile )
	: is_edge_quartile_( is_edge_quartile )
	{
		loadMapFromOccGridMap( map );
	}

	// Load map from OGM
	template< typename T >
	bool Map<T>::loadMapFromOccGridMap( nav_msgs::OccupancyGrid& ogm )
	{
		std::stringstream ss;
		int counter = 0;

		width_ = ogm.info.width;
		height_ = ogm.info.height;

		grid_.reset( new T[ width_ * height_ ] );

		for (int yy = 0; yy < ogm.info.height; ++yy)
		{
			for (int xx = 0; xx < ogm.info.width; ++xx)
			{
				T* tmp_cell = &grid_[ ogm.info.width * yy + xx ];
				tmp_cell->pose_.x = xx;
				tmp_cell->pose_.y = yy;

				if( ogm.data[ ogm.info.width * yy + xx ] == 100 )
				{
					tmp_cell->is_occupied_ = true;
				}

				// ss << (int)ogm.data[ ogm.info.width * yy + xx ] << " ";

				// ss << std::boolalpha << grid_[ width_ * yy + xx ].is_occupied_ << " ";

				// ROS_INFO( "%f %f", tmp_cell->pose_.x, tmp_cell->pose_.y );
			}

			// ROS_INFO_STREAM(ss.str());
			// ss.str( std::string() );
			// ss.clear();
		}

		// ROS_INFO("mierda");

		// for (int i = 0; i < height_; ++i)
		// {
		// 	for (int j = 0; j < width_; ++j)
		// 	{
		// 		ss << std::boolalpha << grid_[ width_ * i + j].is_occupied_ << " ";

		// 		// ROS_INFO( "%f %f", grid_[i * width_ + j].pose_.x, grid_[i * width_ + j].pose_.y );
		// 	}

		// 	ROS_INFO_STREAM(ss.str());
		// 	ss.str( std::string() );
		// 	ss.clear();
		// }

		return true;
	}

	template< typename T >
	bool Map<T>::createEmptyMap()
	{
		for (int yy = 0; yy < height_; ++yy)
		{
			for (int xx = 0; xx < width_; ++xx)
			{
				T* tmp_cell = &grid_[ width_ * yy + xx ];
				tmp_cell->pose_.x = xx;
				tmp_cell->pose_.y = yy;
			}
		}

		return true;
	}

	// Function that return a pointer to the cell with pose x and y
	template< typename T >
	T* Map<T>::getCell( int x, int y )
	{
		return &grid_[ width_ * y + x ];
	}

	template< typename T >
	std::vector<T*> Map<T>::expand( int x, int y )
	{
		std::vector<T*> cell_exp;

		for (int yy = -1; yy < 2; ++yy)
		{
			for (int xx = -1; xx < 2; ++xx)
			{
				if ( y + yy >= 0 && y + yy < height_ && x + xx >= 0 && x + xx < width_ )
				{
					int i = y + yy;
					int j = x + xx;

					if ( 0 != xx || 0 != yy )
					{
						if ( is_edge_quartile_ && ( std::abs( yy ) != 1 || std::abs( xx ) != 1 ) )
						{
							T* tmp = getCell( j, i );

							// Don't expand nodes that are occupied.
							if ( !tmp->is_occupied_ )
							{
								cell_exp.push_back( tmp );
							}
						}
						else if ( !is_edge_quartile_ )
						{
							T* tmp = getCell( j, i );

							// Don't expand nodes that are occupied.
							if ( !tmp->is_occupied_ )
							{
								cell_exp.push_back( tmp );
							}
						}
						
					}

				}
			}
		}

		return cell_exp;
	}
}

#endif