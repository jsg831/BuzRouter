#include "grid.h"

void Grid::make_grid( std::vector<Track>& tracks )
{
  /* Axes */
  // Add tracks to axis-to-coordinate vector
  for ( unsigned int t = 0; t < tracks.size(); ++t ) {
    Track& track = tracks[t];
    Layer& layer = layers[track.line.l];
    Sublayer& sublayer = layer.sublayers[track.line.sl];
    const bool& dir = layer.direction; // (vertical: 0, horizontal: 1)
    const unsigned int& coor = track.line.upper.coor[dir];

    // Check if the track has enough space to the design boundary
    const unsigned int boundary_low = boundary.lower.coor[dir] + sublayer.spacing;
    const unsigned int boundary_upp = boundary.upper.coor[dir] - sublayer.spacing;
    resize_width_in( coor, track.width, boundary_low, boundary_upp );

    axis_coor[dir].push_back( coor );

    // Add tracks to corresponding layers and sublayers
    layer.ltra_coor.push_back( coor );
    sublayer.sltra_coor.push_back( coor );
  }

  /* Tracks */
  // Remove duplicate coordinates of grid
  remove_duplicates( axis_coor[0] );
  remove_duplicates( axis_coor[1] );

  // Remove duplicate coordinates of layers and sublayers
  for ( unsigned int l = 0; l < layers.size(); ++l ) {
    Layer& layer = layers[l];
    remove_duplicates( layer.ltra_coor );
    for ( unsigned int sl = 0; sl < layer.sublayers.size(); ++sl ) {
      Sublayer& sublayer = layer.sublayers[sl];
      remove_duplicates( sublayer.sltra_coor );
    }
  }

  // Generate layer-/sublayer-track index-converting vectors
  for ( unsigned int l = 0; l < layers.size(); ++l ) {
    Layer& layer = layers[l];
    const std::vector<unsigned int>& axis = axis_coor[layer.direction];
    // Layer
    // Layer-track index to axis index
    convert_subindex( layer.ltra_axis, axis, layer.ltra_coor );

    // Sublayer
    for ( unsigned int sl = 0; sl < layer.sublayers.size(); ++sl ) {
      Sublayer& sublayer = layer.sublayers[sl];
      // Sublayer-track index to axis index
      convert_subindex( sublayer.sltra_axis, axis, sublayer.sltra_coor );

      // Sublayer-track index to layer-track index
      convert_subindex( sublayer.sltra_ltra, layer.ltra_axis,
        sublayer.sltra_axis );
    }
  }

  /* Intersections */
  // Generate layer-intersection index for each layer
  for ( unsigned int i = 0; i < layers.size(); ++i ) {
    Layer& layer = layers[i];
    std::vector<unsigned int>& lint_axis = layer.lint_axis;
    if ( i != 0 )
      lint_axis.insert( lint_axis.end(), layers[i-1].ltra_axis.begin(),
        layers[i-1].ltra_axis.end() );

    if ( i != (layers.size()-1) )
      lint_axis.insert( lint_axis.end(), layers[i+1].ltra_axis.begin(),
        layers[i+1].ltra_axis.end() );

    // Remove duplicate layer-intersection axis index
    remove_duplicates( layer.lint_axis );

    // Layer-intersection index to coordinate
    layer.lint_coor.resize( layer.lint_axis.size() );
    for ( unsigned int j = 0; j < layer.lint_coor.size(); ++j )
      layer.lint_coor[j] = axis_coor[!layer.direction][layer.lint_axis[j]];
  }

  /* Interlayer Conversions */
  for ( unsigned int i = 0; i < layers.size(); ++i ) {
    for ( unsigned int j = 0; j < layers[i].sublayers.size(); ++j ) {
      Sublayer& sublayer = layers[i].sublayers[j];
      if ( i != 0 ) {
        // Current-sublayer-track index to lower-layer-intersection index
        convert_subindex( sublayer.sltra_llint, layers[i-1].lint_axis,
          sublayer.sltra_axis );
        // Lower-layer-intersection to current-sublayer-track index
        convert_index( sublayer.llint_sltra, layers[i-1].lint_axis,
          sublayer.sltra_axis );
      }
      if ( i != layers.size()-1 ) {
        // Current-sublayer-track index to upper-layer-intersection index
        convert_subindex( sublayer.sltra_ulint, layers[i+1].lint_axis,
          sublayer.sltra_axis );
        // Upper-layer-intersection to current-sublayer-track index
        convert_index( sublayer.ulint_sltra, layers[i+1].lint_axis,
          sublayer.sltra_axis );
      }
    }
  }

  /* Grid Nodes */
  // Resize grid nodes of each sublayer of each layer
  for ( unsigned int l = 0; l < layers.size(); ++l ) {
    Layer& layer = layers[l];
    for ( unsigned int sl = 0; sl < layer.sublayers.size(); ++sl ) {
      Sublayer& sublayer = layer.sublayers[sl];
      sublayer.grid_nodes.resize( sublayer.sltra_axis.size() );
      for ( unsigned int t = 0; t < sublayer.grid_nodes.size(); ++t ) {
        std::vector<GridNode>& track = sublayer.grid_nodes[t];
        track.resize( layer.lint_axis.size() );
        for ( unsigned int i = 0; i < track.size(); ++i ) {
          GridNode& grid_node = track[i];
          grid_node.width_cur = 0;
          grid_node.width_low = 0;
          grid_node.flags = 0;
        }
      }
    }
  }

  /* Track Widths */
  for ( unsigned int t = 0; t < tracks.size(); ++t ) {
    Track& track = tracks[t];
    Layer& layer = layers[track.line.l];
    Sublayer& sublayer = layer.sublayers[track.line.sl];
    std::vector< std::vector<GridNode> >& grid_nodes = sublayer.grid_nodes;

    const bool dir = layer.direction; // (vertical: 0, horizontal: 1)
    const unsigned int& track_coor = track.line.upper.coor[dir];
    const unsigned int& lower_coor = track.line.lower.coor[!dir];
    const unsigned int& upper_coor = track.line.upper.coor[!dir];

    unsigned int track_index = find_lower_bound( track_coor, sublayer.sltra_coor );
    unsigned int from_index = find_upper_bound( lower_coor, layer.lint_coor );
    unsigned int to_index = find_lower_bound( upper_coor, layer.lint_coor );

    // Set the widths of on-track grid nodes if smaller than the track width.
    do {
      unsigned short& width_cur = grid_nodes[track_index][from_index].width_cur;
      unsigned short& width_low = grid_nodes[track_index][from_index].width_low;
      if ( width_cur < track.width ) width_cur = track.width;
      if ( width_low < track.width ) width_low = track.width;
    } while ( ++from_index <= to_index );
  }
}

void Grid::add_obstacles( const std::vector<Rectangle>& obstacles )
{
  for ( unsigned int o = 0; o < obstacles.size(); ++o ) {
    const Rectangle& obstacle = obstacles[o];
    Layer& layer = layers[obstacle.l];
    Sublayer& sublayer = layer.sublayers[obstacle.sl];
    std::vector< std::vector<GridNode> >& grid_nodes = sublayer.grid_nodes;

    const bool dir = layer.direction; // (vertical: 0, horizontal: 1)
    const unsigned int& spacing = sublayer.spacing;
    const unsigned int coor_tra_low = safe_sub( obstacle.lower.coor[dir], spacing );
    const unsigned int coor_tra_upp = safe_add( obstacle.upper.coor[dir], spacing );
    // No line-end rules.
    const unsigned int coor_int_low = safe_add( obstacle.lower.coor[!dir], 0 );
    const unsigned int coor_int_upp = safe_sub( obstacle.upper.coor[!dir], 0 );

    // Find the bound of indices where the grid nodes intersect with the
    // obstacle.
    unsigned int from_tra = find_upper_bound( coor_tra_low, sublayer.sltra_coor );
    unsigned int to_tra = find_lower_bound( coor_tra_upp, sublayer.sltra_coor );
    unsigned int from_int = find_upper_bound( coor_int_low, layer.lint_coor );
    unsigned int to_int = find_lower_bound( coor_int_upp, layer.lint_coor );

    if ( from_int == -1 || to_int == -1 ) continue;

    // Extend the bound by OBS_EXT
    unsigned int from_tra_ov = safe_sub( from_tra, OBS_EXT );
    unsigned int to_tra_ov = safe_add( to_tra, OBS_EXT,
      sublayer.sltra_coor.size() - 1 );

    do {
      for ( unsigned int t = from_tra_ov; t <= to_tra_ov; ++t ) {
        GridNode& grid_node = grid_nodes[t][from_int];
        unsigned short& width = ( from_int > to_int ) ? grid_node.width_low :
          grid_node.width_cur;
        if ( t >= from_tra && t <= to_tra ) {
          width = 0;
          if ( from_int <= to_int ) grid_node.set_bit( GridNode::obs, 1 );
        } else {
          resize_width_out( sublayer.sltra_coor[t], width, coor_tra_low,
            coor_tra_upp );
        }
      }
    } while ( ++from_int <= to_int );

    // Mark lower-/upper-sublayer grid nodes intersected with the obstacle
    for ( unsigned int sl = 0; sl < layer.sublayers.size(); ++sl ) {
      if ( sl == obstacle.sl ) continue;

      Sublayer& sublayer = layer.sublayers[sl];
      unsigned int from_tra = find_upper_bound( coor_tra_low, sublayer.sltra_coor );
      unsigned int to_tra = find_lower_bound( coor_tra_upp, sublayer.sltra_coor );
      unsigned int from_int = find_upper_bound( coor_int_low, layer.lint_coor );
      unsigned int to_int = find_lower_bound( coor_int_upp, layer.lint_coor );

      for ( unsigned int t = from_tra; t <= to_tra; ++t ) {
        for ( unsigned int i = from_int; i <= to_int; ++i ) {
          // The obstacle is upper to the lower sublayer and lower to the upper
          // sublayer.
          if ( sl < obstacle.sl )
            sublayer.grid_nodes[t][i].set_bit( GridNode::obs_upp, 1 );
          else
            sublayer.grid_nodes[t][i].set_bit( GridNode::obs_low, 1 );
        }
      }
    }
  }
}

void Grid::update_routable_range(
  const std::vector< std::vector<unsigned int> >& bus_widths )
{
  for ( unsigned int l = 0; l < layers.size(); ++l ) {
    Layer& layer = layers[l];
    for ( unsigned int sl = 0; sl < layer.sublayers.size(); ++sl ) {
      Sublayer& sublayer = layer.sublayers[sl];
      const unsigned int& bus_width = bus_widths[l][sl];
      std::vector< std::vector<GridNode> >& grid_nodes = sublayer.grid_nodes;
      for ( unsigned int t = 0; t < grid_nodes.size(); ++t ) {
        std::vector<GridNode>& track = grid_nodes[t];
        // The start and end of the current routable range
        unsigned int start = -1;
        unsigned int end = 0;
        for ( unsigned int i = 0; i < track.size(); ++i ) {
          GridNode& grid_node = track[i];
          grid_node.cost = -1;
          bool routable = ( grid_node.width_cur >= bus_width );
          // Update the bound of current intersection
          if ( routable && grid_node.width_low >= bus_width ) end = i;
          else end = -1;
          // If an unroutable node is reached, update nodes in the previous
          // routable range.
          if ( i == track.size() - 1 || end == -1 ) {
            for ( unsigned int j = start; j < i; ++j ) {
              track[j].range.upp = ( end == -1 ) ? ( i - 1 ) : i;
            }
            grid_node.range.upp = end;
          }
          // If the node is the first node in a routable region, update start
          if ( routable ) start = ( start == -1 ) ? i : start;
          else start = -1;
          grid_node.range.low = start;
        }
      }
    }
  }
}

bool Grid::generate_pinout_nodes( Pinout& pinout, unsigned char l, unsigned char sl,
  bool heading )
{
  const bool direction = pinout.direction;
  const std::vector<Rectangle>& pin_shapes = pinout.pin_shapes;
  RoutingNode rn;
  rn.cost = 0;
  rn.locked = 1;
  rn.heading.cur = heading;
  rn.node.l = l;
  rn.node.sl = sl;
  rn.t_cur.clear();
  const Layer& layer = layers[l];
  const Sublayer& sublayer = layer.sublayers[sl];
  const std::vector< std::vector<GridNode> >& grid_nodes = sublayer.grid_nodes;
  if ( layer.direction != direction ) return 0;
  const unsigned int i_pin = heading ? pin_shapes[0].upper.coor[!direction]
    : pin_shapes[0].lower.coor[!direction];
  const unsigned int i_node = heading ? find_upper_bound(i_pin, layer.lint_coor)
    : find_lower_bound(i_pin, layer.lint_coor);
  if ( i_node == -1 ) return 0;
  rn.node.i = i_node;
  rn.i_cur = i_node;
  for ( unsigned int p = 0; p < pin_shapes.size(); ++p ) {
    const Rectangle& pin_shape = pin_shapes[p];
    const unsigned int t_pin_upp = pin_shape.upper.coor[direction];
    const unsigned int t_pin_low = pin_shape.lower.coor[direction];
    unsigned int t_node = find_lower_bound(t_pin_upp, sublayer.sltra_coor);
    bool success = 0;
    while ( sublayer.sltra_coor[t_node] >= t_pin_low && t_node != -1 ) {
      const GridNode& grid_node = grid_nodes[t_node][i_node];
      if ( grid_node.routable() ) {
        rn.t_cur.push_back( t_node );
        success = 1;
        break;
      }
      t_node--;
    }
    if ( !success ) return 0;
  }
  rn.node.t = rn.t_cur[0];
  rn.range = routable_range( rn.node, rn.t_cur, rn.heading.cur );
  pinout.nodes.push_back(rn);
  return 1;
}

bool Grid::update_tracks( Node n, unsigned char b, unsigned int bw, unsigned int i_start,
  bool direction, std::vector<unsigned int>& t )
{
  unsigned int coor_pre = direction ? 0 : -1;
  unsigned int nbits = b;
  const Sublayer& sublayer = layers[n.l].sublayers[n.sl];
  const unsigned int s = sublayer.spacing + bw;
  const std::vector< std::vector<GridNode> >& grid_nodes = sublayer.grid_nodes;
  t.clear();
  t.resize(nbits);
  // If the first track is unroutable, return false
  if ( !grid_nodes[n.t][n.i].routable() ) return 0;
  while ( b != 0 && n.t != -1 && n.t != sublayer.sltra_axis.size() ) {
    unsigned int coor_cur = sublayer.sltra_coor[n.t];
    const GridNode& grid_node = grid_nodes[n.t][n.i];
    bool is_valid = direction ? (coor_cur >= (coor_pre + s))
      : (coor_cur <= (coor_pre - s));
    if ( grid_node.routable_to(i_start) && is_valid ) {
      t[direction ? (b - 1) : (nbits - b)] = n.t;
      coor_pre = coor_cur;
      b--;
    }
    n.t = direction ? (n.t + 1) : (n.t - 1);
  }
  return (b == 0);
}

bool Grid::check_vias( RoutingNode& rn, bool via_type )
{
  const Layer& layer_pre = layers[rn.l_pre];
  const Sublayer& sublayer_pre = layer_pre.sublayers[rn.sl_pre];
  const std::vector< std::vector<GridNode> >& grid_nodes_pre = sublayer_pre.grid_nodes;
  const Layer& layer_cur = layers[rn.node.l];
  const Sublayer& sublayer_cur = layer_cur.sublayers[rn.node.sl];
  const std::vector< std::vector<GridNode> >& grid_nodes_cur = sublayer_cur.grid_nodes;
  const std::vector<unsigned int>& conv_cur_pre = (rn.l_pre > rn.node.l) ?
    sublayer_cur.sltra_ulint : sublayer_cur.sltra_llint;
  const std::vector<unsigned int>& conv_pre_cur = (rn.node.l > rn.l_pre) ?
    sublayer_pre.sltra_ulint : sublayer_pre.sltra_llint;
  bool success = 1;
  const unsigned int nbits = rn.t_cur.size();
  for ( unsigned int t = 0; t < nbits; ++t ) {
    const unsigned int& t_pre = rn.t_pre[t];
    const unsigned int& t_cur = via_type ? rn.t_cur[t] : rn.t_cur[nbits-t-1];
    const unsigned int& iv_pre = conv_cur_pre[t_cur];
    const unsigned int& iv_cur = conv_pre_cur[t_pre];
    const GridNode& via_pre = grid_nodes_pre[t_pre][iv_pre];
    const GridNode& via_cur = grid_nodes_cur[t_cur][iv_cur];
    // Check if the previous track is routable to the via
    success &= via_pre.routable_to(rn.i_pre);
    // Check if the current track is routable to the via
    success &= via_cur.routable_to(rn.node.i);
    // Check if the via is obstructed by any intermediate obstacle
    success &= (rn.node.l > rn.l_pre) ?
      !via_pre.obstructed_upp() : !via_pre.obstructed_low();
    success &= (rn.l_pre > rn.node.l) ?
      !via_cur.obstructed_upp() : !via_cur.obstructed_low();
    if ( !success ) return 0;
  }
  return success;
}

Range Grid::routable_range( Node& n, const std::vector<unsigned int>& t,
  bool direction )
{
  const std::vector< std::vector<GridNode> >& grid_nodes = layers[n.l].sublayers[n.sl].grid_nodes;
  Range range(0, -1);
  unsigned int& start = direction ? range.low : range.upp;
  unsigned int& end = direction ? range.upp : range.low;
  // Set the starting intersection
  start = n.i;
  // Find the ending intersection nearest to the starting intersection
  for ( unsigned int i = 0; i < t.size(); ++i ) {
    const unsigned int& track = t[i];
    const Range& node_range = grid_nodes[track][n.i].range;
    if ( direction ) {
      end = ( node_range.upp < end ) ? node_range.upp : end;
    } else {
      end = ( node_range.low > end ) ? node_range.low : end;
    }
  }
  return range;
}

unsigned int Grid::safe_add( const unsigned int& a, const unsigned int& b,
  const unsigned int& bound )
{
  return ( (a + b >= a) && (a + b <= bound) ) ? ( a + b ) : bound;
}

unsigned int Grid::safe_sub( const unsigned int& a, const unsigned int& b,
  const unsigned int& bound )
{
  return ( (a - b <= a) && (a - b >= bound) ) ? ( a - b ) : bound;
}

void Grid::remove_duplicates( std::vector<unsigned int>& vec )
{
  std::sort( vec.begin(), vec.end() );
  std::vector<unsigned int>::iterator it = std::unique( vec.begin(), vec.end() );
  vec.resize( std::distance( vec.begin(), it ) );
}

void Grid::convert_index( std::vector<unsigned int>& conv_vec,
  const std::vector<unsigned int>& vec, const std::vector<unsigned int>& sub_vec )
{
  unsigned int subindex = 0;
  conv_vec.resize( vec.size(), -1 );
  for ( unsigned int index = 0; index < conv_vec.size(); ++index ) {
    if ( vec[index] == sub_vec[subindex] ) {
      conv_vec[index] = subindex;
      ++subindex;
    }
  }
}

void Grid::convert_subindex( std::vector<unsigned int>& conv_vec,
  const std::vector<unsigned int>& vec, const std::vector<unsigned int>& sub_vec )
{
  unsigned int index = 0;
  conv_vec.resize( sub_vec.size() );
  for ( unsigned int subindex = 0; subindex < conv_vec.size(); ++subindex ) {
    while ( vec[index] != sub_vec[subindex]) index++;
    conv_vec[subindex] = index;
  }
}

unsigned int Grid::find_lower_bound( const unsigned int& val,
  const std::vector<unsigned int>& vec )
{
  if ( val < vec.front() ) return -1;
  if ( val >= vec.back() ) return ( vec.size()-1 );
  std::vector<unsigned int>::const_iterator it = std::lower_bound( vec.begin(), vec.end(), val );
  return ( val == *it ) ? ( it-vec.begin() ) : ( it-vec.begin()-1 );
}

unsigned int Grid::find_upper_bound( const unsigned int& val,
  const std::vector<unsigned int>& vec )
{
  if ( val < vec.front() ) return 0;
  if ( val > vec.back() ) return -1;
  std::vector<unsigned int>::const_iterator it = std::lower_bound( vec.begin(), vec.end(), val );
  return ( it-vec.begin() );
}

void Grid::resize_width_in( const unsigned int& coor, unsigned short& width,
  const unsigned int& lower, const unsigned int& upper )
{
  if ( coor <= lower || coor >= upper ) {
    width = 0;
    return;
  }
  unsigned int max_width = std::min( coor - lower, upper - coor ) << 1;
  width = ( max_width < width ) ? max_width : width;
}

void Grid::resize_width_out( const unsigned int& coor, unsigned short& width,
  const unsigned int& lower, const unsigned int& upper )
{
  if ( coor >= lower && coor <= upper ) {
    width = 0;
    return;
  }
  unsigned int max_width = (( coor > upper ) ? (coor - upper) : (lower - coor)) << 1;
  width = ( max_width < width ) ? max_width : width;
}
