#ifndef _UTILS_H
#define _UTILS_H

#include <cstdint>

struct Point
{
public:
  uint32_t coor[2]; // (x: 0, y: 1)
};

struct Node
{
public:
  uint8_t l;
  uint8_t sl;
  union {
    uint32_t x;
    uint32_t t; // track index
  };
  union {
    uint32_t y;
    uint32_t i; // intersection index
  };
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
  uint32_t width;
  Rectangle line;
};

#endif
