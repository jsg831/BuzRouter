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
