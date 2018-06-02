#include "utils.h"

bool Range::contains( uint32_t val ) const
{
  return (val >= low) && (val <= upp);
}
