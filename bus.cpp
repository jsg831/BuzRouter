#include "bus.h"

void Bus::initialize( void )
{
  /* Pinouts */
  pinouts.resize( bits[0].pin_shapes.size() );
  for ( unsigned int p = 0; p < pinouts.size(); ++p ) {
    Pinout& pinout = pinouts[p];
    bool& dir = pinout.direction;
    if ( bits.size() == 1 ) continue;
    // x-/y-oordinates of bit 0 and bit 1
    const unsigned int* c0 = bits[0].pin_shapes[p].lower.coor;
    const unsigned int* c1 = bits[1].pin_shapes[p].lower.coor;
    // Horizontal if the x-coordinates are the same.
    dir = ( c0[0] == c1[0] );
    pinout.bit_order = ( c0[dir] > c1[dir] );
    pinout.pin_shapes.resize( bits.size() );
    for ( unsigned int i = 0; i < bits.size(); ++i ) {
      pinout.pin_shapes[i] = bits[i].pin_shapes[p];
    }
    if ( !pinout.bit_order ) std::reverse( pinout.pin_shapes.begin(),
      pinout.pin_shapes.end() );
  }
}
