#include "utils.h"

bool Range::contains( unsigned int val ) const
{
  return (val >= low) && (val <= upp);
}
