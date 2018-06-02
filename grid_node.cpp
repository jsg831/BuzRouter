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

bool GridNode::obstructed_low( void ) const
{
  return flags.test( FlagBit::obs_low );
}

bool GridNode::obstructed_upp( void ) const
{
  return flags.test( FlagBit::obs_upp );
}

bool GridNode::routable( void ) const
{
  return ( range.low != -1 );
}

bool GridNode::routable_to( uint32_t i ) const
{
  return range.contains(i);
}
