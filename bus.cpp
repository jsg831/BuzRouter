#include "bus.h"

void Bus::initialize( void )
{
  /* Pinouts */
  pinouts.resize( bits[0].pin_shapes.size() );
  for ( auto p = 0; p < pinouts.size(); ++p ) {
    auto& pinout = pinouts[p];
    auto& dir = pinout.direction;
    if ( bits.size() == 1 ) continue;
    // x-/y-oordinates of bit 0 and bit 1
    const auto& c0 = bits[0].pin_shapes[p].lower.coor;
    const auto& c1 = bits[1].pin_shapes[p].lower.coor;
    // Horizontal if the x-coordinates are the same.
    dir = ( c0[0] == c1[0] );
    pinout.bit_order = ( c0[dir] > c1[dir] );
    pinout.pin_shapes.resize( bits.size() );
    for ( auto i = 0; i < bits.size(); ++i ) {
      if ( pinout.bit_order )
        pinout.pin_shapes[i] = bits[i].pin_shapes[p];
      else
        pinout.pin_shapes[i] = bits[i].pin_shapes[p];
    }
  }
}
