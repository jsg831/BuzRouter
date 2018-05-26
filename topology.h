#ifndef _TOPOLOGY_H
#define _TOPOLOGY_H

#include <cstdint>
#include <vector>
#include "utils.h"

struct Pinout
{
public:
  bool single_bit;
  bool direction; // (vertical(x): 0, horizontal(y): 1)
  bool bit_order; // (low-01234567-upp: 0, low-76543210-upp: 1)
  Node source;
  std::vector<Node> pin_nodes;
};

#endif
