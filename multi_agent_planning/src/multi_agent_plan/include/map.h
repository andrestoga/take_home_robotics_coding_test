#ifndef AGENT_H
#define AGENT_H

#include "ros/ros.h"

namespace multi_agent_plan
{
	class Map
	{
	public:
		Map();
		~Map();
		
		int grid_**;
		int m_;
		int n_;
	};

	Map::Map( int m, int n )
	{
		grid_ = new int[m][n];
		int m_ = m;
		int n_ = n;
	}
}

#endif