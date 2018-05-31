#ifndef _BUS_H
#define _BUS_H

#include <cstdint>
#include <string>
#include <vector>
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
  std::string name;
  std::vector<uint32_t> layer_widths;
  std::vector<Bit> bits;
  std::vector< std::vector<uint32_t> > bus_widths;
  std::vector<Pinout> pinouts;
  /* Functions */
  void initialize_pinouts( void );
};

#endif
