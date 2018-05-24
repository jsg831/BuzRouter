#ifndef _UTILS_H
#define _UTILS_H

#include <cstdint>

struct Point
{
public:
  uint32_t coor[2]; // (x: 0, y: 1)
};

struct Range
{
public:
  uint32_t low = 0;
  uint32_t upp = UINT32_MAX;

  Range( void ) { }
  Range( uint32_t low, uint32_t upp ) : low(low), upp(upp) { }
};

struct NbitRange : public Range
{
public:
  uint8_t nbit = 0;

  void set_range( const Range& range )
  {
    low = range.low;
    upp = range.upp;
  }
};

struct Node
{
public:
  uint8_t l;
  uint8_t sl;
  uint32_t t;
  uint32_t i;
};

struct Rectangle
{
public:
  uint8_t l;
  uint8_t sl;
  Point lower;
  Point upper;
};

struct Track
{
public:
  uint16_t width;
  Rectangle line;
};

#endif
