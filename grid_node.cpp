#include "grid.h"

void GridNode::set_bit( const FlagBit bit, const bool flag )
{
  flags.set( bit, flag );
}

bool GridNode::get_bit( const FlagBit bit ) const
{
  return flags.test( bit );
}
