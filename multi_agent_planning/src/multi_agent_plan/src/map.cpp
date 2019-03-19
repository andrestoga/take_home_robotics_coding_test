/* map.cpp
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#include "multi_agent_plan/map.h"

namespace multi_agent_plan
{
	Map::Map( int m, int n )
	: m_(m)
	, n_(n)
	{
		grid_ = new int[n * n];
	}

	Map::~Map()
	{
		delete [] grid_;
	}
}