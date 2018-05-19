#include "router.h"

void Router::initialize( void )
{
  grid.make_grid( tracks );
  grid.add_obstacles( obstacles );
}
