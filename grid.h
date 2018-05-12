#ifndef _GRID_H
#define _GRID_H

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <string>
#include <vector>
#include "utils.h"

struct GridNode
{
public:
  /* Variables */
  uint32_t cost;
  uint16_t width_cur;
  uint16_t width_low;

  uint8_t routable_cur;
  uint8_t routable_low;
  uint8_t visited;
private:
  std::bitset<32> flags;
};

struct Sublayer
{
public:
  /* Variables */
  std::string name;
  uint32_t spacing;
  std::vector< std::vector<GridNode> > grid_nodes; // [track][intersection]

  /** Index Conversion **/
  // Sublayer-track index to physical coordinate
  std::vector<uint32_t> sltra_coor;
  // Sublayer-track index to axis index
  std::vector<uint32_t> sltra_axis;
  // Sublayer-track index to layer-track/intersection index
  std::vector<uint32_t> sltra_ltra;
  // Sublayer-track mask of layer-track indices
  std::vector<bool> sltra_ltra_mask;
};

struct Layer
{
public:
  /* Variables */
  bool direction; // (horizontal: 0, vertical: 1)
  std::vector<Sublayer> sublayers;

  /** Index Conversion **/
  // Layer-track index to physical coordinate
  std::vector<uint32_t> ltra_coor;
  // Layer-track/intersection index to axis index
  std::vector<uint32_t> ltra_axis;
  std::vector<uint32_t> lint_axis;
};

struct Grid
{
public:
  /* Variables */
  std::vector<Layer> layers;

  /** Index Conversion **/
  // Axis index to physical coordinates
  std::vector<uint32_t> axis_coor[2]; // (horizontal(y): 0, vertical(x): 1)

  /* Functions */
  /** Initialization **/
  void make_grid( const std::vector<Track>& tracks );
private:
  void remove_duplicates( std::vector<uint32_t>& vec );
  void convert_index( std::vector<uint32_t>& conv_vec,
    const std::vector<uint32_t>& vec, const std::vector<uint32_t>& sub_vec );
};

#endif
