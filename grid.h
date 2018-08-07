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
  uint64_t cost;
  unsigned short width_cur;
  unsigned short width_low;
  // Backtracking
  unsigned int from;
  unsigned char from_sl;
  unsigned char psudo_block;
  Range range;
  enum FlagBit { obs, obs_low, obs_upp, src, tar_low, tar_upp, dir, dir_l, psu};
  /* Functions */
  void set_bit( const FlagBit bit, const bool flag );
  bool get_bit( const FlagBit bit ) const;
  bool obstructed( void ) const;
  bool obstructed_low( void ) const;
  bool obstructed_upp( void ) const;
  bool routable( void ) const;
  bool routable_to( unsigned int i ) const;

  std::bitset<32> flags = 0;
};

struct RoutingNode
{
  uint64_t cost;
  Node node;
  bool locked;
  struct {
    // (0: lower, 1: upper)
    bool pre;
    bool cur;
  } heading;
  unsigned char l_pre;
  unsigned char sl_pre;
  unsigned int i_pre;
  std::vector<unsigned int> t_pre;
  unsigned int i_cur;
  std::vector<unsigned int> t_cur;
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
  unsigned int spacing;
  std::vector< std::vector<GridNode> > grid_nodes; // [track][intersection]

  /** Index Conversion **/
  // Sublayer-track index to physical coordinate
  std::vector<unsigned int> sltra_coor;
  // Sublayer-track index to axis index
  std::vector<unsigned int> sltra_axis;
  // Sublayer-track index to layer-track/intersection index
  std::vector<unsigned int> sltra_ltra;
  // Sublayer-track index to upper-layer-intersection index
  std::vector<unsigned int> sltra_ulint;
  // Sublayer-track index to lower-layer-intersection index
  std::vector<unsigned int> sltra_llint;
  // Upper-layer-intersection index to sublayer-track index
  std::vector<unsigned int> ulint_sltra;
  // Lower-layer-intersection index to sublayer-track index
  std::vector<unsigned int> llint_sltra;
};

struct Layer
{
public:
  /* Variables */
  bool direction; // (vertical: 0, horizontal: 1)
  std::vector<Sublayer> sublayers;

  /** Index Conversion **/
  // Layer-track index to physical coordinate
  std::vector<unsigned int> ltra_coor;
  std::vector<unsigned int> lint_coor;
  // Layer-track/intersection index to axis index
  std::vector<unsigned int> ltra_axis;
  std::vector<unsigned int> lint_axis;
};

struct Grid
{
public:
  /* Variables */
  Rectangle boundary;
  std::vector<Layer> layers;
  /** Index Conversion **/
  // Axis index to physical coordinates
  std::vector<unsigned int> axis_coor[2]; // (vertical(x): 0, horizontal(y): 1)
  /* Functions */
  /** Initialization **/
  void make_grid( std::vector<Track>& tracks );
  void add_obstacles( const std::vector<Rectangle>& obstacles );
  /** Routablility **/
  void update_routable_range(
    const std::vector< std::vector<unsigned int> >& bus_widths );
  /** Pinout **/
  bool generate_pinout_nodes( Pinout& pinout, unsigned char l, unsigned char sl,
    bool heading, unsigned int bw, std::vector<Rectangle> obstacles,
    unsigned int spacing);
  bool check_pinout_spacing( const unsigned int t_coor, const unsigned int i_coor, unsigned char l,
    unsigned char sl, bool heading, bool direction, std::vector<Rectangle> obstacles,
    unsigned int spacing );
  bool overlap_node2rec_spacing( unsigned int t_coor, unsigned int i_coor,
    bool dir, Rectangle rec, unsigned int spacing );
  /** Routing **/
  bool update_tracks( Node n, unsigned char b, unsigned int bw, unsigned int i_start,
    bool direction, std::vector<unsigned int>& t );
  bool check_vias( RoutingNode& rn, bool via_type );
  Range routable_range( Node& n, const std::vector<unsigned int>& t,
    bool direction );
private:
  /* Functions */
  /** Utilities **/
  // Overflow-aware addition and substraction for unsigned numbers
  uint32_t safe_add( const unsigned int& a, const unsigned int& b,
    const unsigned int& bound = UINT32_MAX );
  uint32_t safe_sub( const unsigned int& a, const unsigned int& b,
    const unsigned int& bound = 0 );
  // Remove duplicate values in a vector
  void remove_duplicates( std::vector<unsigned int>& vec );
  // Convenient index-converting functions
  void convert_index( std::vector<unsigned int>& conv_vec,
    const std::vector<unsigned int>& vec, const std::vector<unsigned int>& sub_vec );
  void convert_subindex( std::vector<unsigned int>& conv_vec,
    const std::vector<unsigned int>& vec, const std::vector<unsigned int>& sub_vec );
  // Convenient index-searching functions
  uint32_t find_lower_bound( const unsigned int& val,
    const std::vector<unsigned int>& vec );
  uint32_t find_upper_bound( const unsigned int& val,
    const std::vector<unsigned int>& vec );
  // Resize the widths of tracks inside (lower, upper) and set the widths
  // outside (lower, upper) to zero.
  void resize_width_in( const unsigned int& coor, unsigned short& width,
    const unsigned int& lower, const unsigned int& upper );
  // Resize the widths of tracks outside (lower, upper) and set the widths
  // inside (lower, upper) to zero.
  void resize_width_out( const unsigned int& coor, unsigned short& width,
    const unsigned int& lower, const unsigned int& upper );
};

#endif
