#include "grid.h"

void GridNode::set_bit( const FlagBit bit, const bool flag )
{
  flags.set( bit, flag );
}

bool GridNode::get_bit( const FlagBit bit ) const
{
  return flags.test( bit );
}

bool GridNode::obstructed( void ) const
{
  return flags.test( FlagBit::obs );
}

bool GridNode::routable( void ) const
{
  return ( range.low != UINT32_MAX );
}
