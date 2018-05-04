#ifndef _BUS_H
#define _BUS_H

#include <cstdint>
#include <string>
#include <vector>
#include "utils.h"

struct Bit
{
  std::string name;
  std::vector<Rectangle> pin_shapes;
};

struct Bus
{
  std::string name;
  std::vector<uint32_t> layer_widths;
  std::vector<Bit> bits;
};

#endif
