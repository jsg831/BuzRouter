#ifndef _UTILS_H
#define _UTILS_H

#include <cstdint>

struct Point
{
public:
  unsigned int coor[2]; // (x: 0, y: 1)
};

struct Range
{
public:
  unsigned int low;
  unsigned int upp;
  Range( void ) {
    low = 0;
    upp = -1;
  }
  Range( unsigned int low, unsigned int upp ) : low(low), upp(upp) { }
  /** Functions **/
  bool contains( unsigned int val ) const;
};

struct Node
{
public:
  unsigned char l;
  unsigned char sl;
  unsigned int t;
  unsigned int i;
};

struct Rectangle
{
public:
  unsigned char l;
  unsigned char sl;
  Point lower;
  Point upper;
};

struct Track
{
public:
  unsigned short width;
  Rectangle line;
};

#endif
