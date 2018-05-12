#include "utils.h"

bool Track::direction( void ) const
{
  return (line.upper.x == line.lower.x);
}
