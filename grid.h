#ifndef _GRID_H
#define _GRID_H

#include <bitset>
#include <cstdint>
#include <string>
#include <vector>
#include "utils.h"

struct GridNode
{
public:
  uint32_t cost;
  uint32_t width;
private:
  std::bitset<32> flags;
};

struct Layer
{
  std::string name;
  bool direction; // (horizontal: 0, vertical: 1)
  uint32_t spacing;
  std::vector< std::vector<GridNode> > grid_nodes;
};

struct Grid
{
public:
  std::vector<Layer> layers;
  std::vector<uint32_t> axisx_coordinate;
  std::vector<uint32_t> axisy_coordinate;
};

#endif
