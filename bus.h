#ifndef _BUS_H
#define _BUS_H

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>
#include "grid.h"
#include "utils.h"

struct Bit
{
public:
  std::string name;
  std::vector<Rectangle> pin_shapes;
};

struct Path
{
  bool bit_order;
  bool heading;
  uint8_t l;
  uint8_t sl;
  Range i;
  std::vector<uint32_t> t;
};

struct BusRoute
{
  std::vector<Path> paths;
};

struct Bus
{
public:
  /* Variables */
  bool valid = 1;
  std::string name;
  std::vector<Bit> bits;
  std::vector< std::vector<uint32_t> > bus_widths;
  std::vector<Pinout> pinouts;
  /* Functions */
  void initialize( void );
};

#endif
