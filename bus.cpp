#include "bus.h"

void Bus::initialize_pinouts( void )
{
  pinouts.resize( bits[0].pin_shapes.size() );
  for ( auto i = 0; i < pinouts.size(); ++i ) {
    auto& pinout = pinouts[i];
    auto& dir = pinout.direction;
    if ( bits.size() == 1 ) {
      pinout.single_bit = 1;
      continue;
    }
    pinout.single_bit = 0;
    // x-/y-oordinates of bit 0 and bit 1
    const auto& c0 = bits[0].pin_shapes[i].lower.coor;
    const auto& c1 = bits[1].pin_shapes[i].lower.coor;
    // Horizontal if the x-coordinates are the same.
    dir = ( c0[0] == c1[0] );
    pinout.bit_order = ( c0[dir] > c1[dir] );
  }
}
