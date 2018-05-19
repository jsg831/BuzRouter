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
  uint64_t cost;
  uint16_t width_cur = 0;
  uint16_t width_low = UINT16_MAX;

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
  // Sublayer-track index to upper-layer-intersection index
  std::vector<uint32_t> sltra_ulint;
  // Sublayer-track index to lower-layer-intersection index
  std::vector<uint32_t> sltra_llint;
  // Upper-layer-intersection index to sublayer-track index
  std::vector<uint32_t> ulint_sltra;
  // Lower-layer-intersection index to sublayer-track index
  std::vector<uint32_t> llint_sltra;
};

struct Layer
{
public:
  /* Variables */
  bool direction; // (vertical: 0, horizontal: 1)
  std::vector<Sublayer> sublayers;

  /** Index Conversion **/
  // Layer-track index to physical coordinate
  std::vector<uint32_t> ltra_coor;
  std::vector<uint32_t> lint_coor;
  // Layer-track/intersection index to axis index
  std::vector<uint32_t> ltra_axis;
  std::vector<uint32_t> lint_axis;
};

struct Grid
{
public:
  /* Variables */
  Rectangle boundary;
  std::vector<Layer> layers;

  /** Index Conversion **/
  // Axis index to physical coordinates
  std::vector<uint32_t> axis_coor[2]; // (vertical(x): 0, horizontal(y): 1)

  /* Functions */
  /** Initialization **/
  void make_grid( std::vector<Track>& tracks );
  void add_obstacles( const std::vector<Rectangle>& obstacles );
private:
  /* Functions */
  /** Utilities **/
  // Overflow-aware addition and substraction for unsigned numbers
  uint32_t safe_add( const uint32_t& a, const uint32_t& b,
    const uint32_t& bound = UINT32_MAX );
  uint32_t safe_sub( const uint32_t& a, const uint32_t& b,
    const uint32_t& bound = 0 );
  void remove_duplicates( std::vector<uint32_t>& vec );
  void convert_index( std::vector<uint32_t>& conv_vec,
    const std::vector<uint32_t>& vec, const std::vector<uint32_t>& sub_vec );
  void convert_subindex( std::vector<uint32_t>& conv_vec,
    const std::vector<uint32_t>& vec, const std::vector<uint32_t>& sub_vec );
  uint32_t find_lower_bound( const uint32_t& val,
    const std::vector<uint32_t>& vec );
  uint32_t find_upper_bound( const uint32_t& val,
    const std::vector<uint32_t>& vec );
  // Resize the widths of tracks inside (lower, upper) and set the widths
  // outside (lower, upper) to zero.
  void resize_width_in( const uint32_t& coor, uint16_t& width,
    const uint32_t& lower, const uint32_t& upper );
  // Resize the widths of tracks outside (lower, upper) and set the widths
  // inside (lower, upper) to zero.
  void resize_width_out( const uint32_t& coor, uint16_t& width,
    const uint32_t& lower, const uint32_t& upper );
};

#endif
