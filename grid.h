#ifndef _GRID_H
#define _GRID_H

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include "utils.h"

#define OBS_EXT 8

struct GridNode
{
public:
  /* Variables */
  uint64_t cost = -1;
  uint16_t width_cur = 0;
  uint16_t width_low = 0;
  // Backtracking
  uint32_t from;
  uint8_t from_sl;
  Range range;
  enum FlagBit { obs, obs_low, obs_upp, src, tar_low, tar_upp, dir, dir_l };
  /* Functions */
  void set_bit( const FlagBit bit, const bool flag );
  bool get_bit( const FlagBit bit ) const;
  bool obstructed( void ) const;
  bool obstructed_low( void ) const;
  bool obstructed_upp( void ) const;
  bool routable( void ) const;
  bool routable_to( uint32_t i ) const;
private:
  std::bitset<32> flags = 0;
};

struct RoutingNode
{
  uint64_t cost = 0;
  Node node;
  bool locked = 0;
  bool via_free = 0;
  struct {
    bool pre;
    bool cur;
  } bit_order;
  struct {
    // (0: lower, 1: upper)
    bool pre;
    bool cur;
  } heading;
  uint8_t l_pre;
  uint8_t sl_pre;
  uint32_t i_pre;
  std::vector<uint32_t> t_pre;
  uint32_t i_cur;
  std::vector<uint32_t> t_cur;
  Range range;
};

struct Pinout
{
  bool direction;
  bool bit_order;
  std::vector<Rectangle> pin_shapes;
  std::vector<RoutingNode> nodes;
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
  /** Routablility **/
  void update_routable_range(
    const std::vector< std::vector<uint32_t> >& bus_widths );
  /** Pinout **/
  bool generate_pinout_nodes( Pinout& pinout, uint8_t l, uint8_t sl,
    bool heading );
  /** Routing **/
  bool update_tracks( Node n, uint8_t b, uint32_t bw, uint32_t i_start,
    bool direction, std::vector<uint32_t>& t );
  bool check_vias( RoutingNode& rn, bool via_type );
  Range routable_range( Node& n, const std::vector<uint32_t>& t,
    bool direction );
private:
  /* Functions */
  /** Utilities **/
  // Overflow-aware addition and substraction for unsigned numbers
  uint32_t safe_add( const uint32_t& a, const uint32_t& b,
    const uint32_t& bound = UINT32_MAX );
  uint32_t safe_sub( const uint32_t& a, const uint32_t& b,
    const uint32_t& bound = 0 );
  // Remove duplicate values in a vector
  void remove_duplicates( std::vector<uint32_t>& vec );
  // Convenient index-converting functions
  void convert_index( std::vector<uint32_t>& conv_vec,
    const std::vector<uint32_t>& vec, const std::vector<uint32_t>& sub_vec );
  void convert_subindex( std::vector<uint32_t>& conv_vec,
    const std::vector<uint32_t>& vec, const std::vector<uint32_t>& sub_vec );
  // Convenient index-searching functions
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
