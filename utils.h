#ifndef _UTILS_H
#define _UTILS_H

#include <cstdint>

struct Point
{
public:
  uint32_t x;
  uint32_t y;
};

struct Node
{
public:
  uint8_t l;
  uint32_t nx;
  uint32_t ny;
};

struct Rectangle
{
public:
  uint8_t l;
  Point lower;
  Point upper;
};

struct Track
{
  uint32_t width;
  Rectangle line;
};

#endif
