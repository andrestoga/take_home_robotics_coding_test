/* map.h
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#ifndef MAP_H
#define MAP_H

#include "ros/ros.h"

namespace multi_agent_plan
{
	class Map
	{
	public:
		/**
		 * @brief      Constructs the object.
		 *
		 * @param[in]  m     The middle point of the square map
		 * @param[in]  n     The size of the square map n*n
		 */
		Map( int m, int n );
		/**
		 * @brief      Deallocates the map
		 */
		// ~Map();
		
		std::vector<int> grid_; // The map representation of the square map.
		const int m_;
		const int n_;
	};	
}

#endif