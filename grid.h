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

  uint8_t routable_cur;
  uint8_t routable_low;
  uint8_t visited;
private:
  std::bitset<32> flags;
};

struct Sublayer
{
  std::string name;
  uint32_t spacing;
  std::vector< std::vector<GridNode> > grid_nodes;
};

struct Layer
{
  bool direction; // (horizontal: 0, vertical: 1)
  std::vector<Sublayer> sublayers;
};

struct Grid
{
public:
  std::vector<Layer> layers;
  std::vector<uint32_t> axisx_coordinate;
  std::vector<uint32_t> axisy_coordinate;
};

#endif
