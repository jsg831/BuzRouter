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
  Range i_path;
  std::vector<Range> i_coor;
  std::vector<uint32_t> t;
  std::vector<uint32_t> t_coor;
};

struct BusRoute
{
  uint8_t l_src;
  uint8_t sl_src;
  uint8_t l_tar;
  uint8_t sl_tar;
  std::vector<Path> paths;
  std::vector<Rectangle> wires;
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
  BusRoute route;
  /* Functions */
  void initialize( void );
};

#endif
