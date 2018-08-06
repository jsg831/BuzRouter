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
  bool overlap;
  unsigned char l;
  unsigned char sl;
  Range i_path;
  std::vector<Range> i_coor;
  std::vector<unsigned int> t;
  std::vector<unsigned int> t_coor;
};

struct BusRoute
{
  unsigned char l_src;
  unsigned char sl_src;
  unsigned char l_tar;
  unsigned char sl_tar;
  bool connected;
  bool overlap;
  std::vector<Path> paths;
  std::vector<Rectangle> wires;
};

struct Bus
{
public:
  /* Variables */
  bool valid;
  std::string name;
  std::vector<Bit> bits;
  std::vector< std::vector<unsigned int> > bus_widths;
  std::vector<Pinout> pinouts;
  std::vector<Pinout> steiner_tars;
  std::vector<BusRoute> routes;
  std::vector<Path> main_branchs;
  /* Functions */
  void initialize( void );
};

#endif
