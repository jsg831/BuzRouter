#include "grid.h"

void Grid::make_grid( const std::vector<Track>& tracks )
{
  /* Axes */
  // Add tracks to axis-to-coordinate vector
  for ( const auto& track : tracks ) {
    const auto& dir = track.direction();
    const auto& line = track.line;
    const auto& coor = dir ? line.upper.x : line.upper.y;
    axis_coor[dir].push_back( coor );

    // Add tracks to corresponding layers and sublayers
    layers[line.l].ltra_coor.push_back( coor );
    layers[line.l].sublayers[line.sl].sltra_coor.push_back( coor );
  }

  /* Tracks */
  // Remove duplicate coordinates of grid
  for ( auto& axis : axis_coor )
    remove_duplicates( axis );

  // Remove duplicate coordinates of layers and sublayers
  for ( auto& layer : layers ) {
    remove_duplicates( layer.ltra_coor );
    for ( auto& sublayer : layer.sublayers )
      remove_duplicates( sublayer.sltra_coor );
  }

  // Generate layer-/sublayer-track index-converting vectors
  for ( auto& layer : layers ) {
    const auto& axis = axis_coor[layer.direction];
    // Layer
    // Layer-track index to axis index
    convert_subindex( layer.ltra_axis, axis, layer.ltra_coor );

    // Sublayer
    for ( auto& sublayer : layer.sublayers ) {
      // Sublayer-track index to axis index
      convert_subindex( sublayer.sltra_axis, axis, sublayer.sltra_coor );

      // Sublayer-track index to layer-track index
      convert_subindex( sublayer.sltra_ltra, layer.ltra_axis,
        sublayer.sltra_axis );
    }
  }

  /* Intersections */
  // Generate layer-intersection index for each layer
  for ( uint32_t i = 0; i < layers.size(); ++i ) {
    auto& layer = layers[i];
    auto& lint_axis = layer.lint_axis;
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
    for ( uint32_t j = 0; j < layer.lint_coor.size(); ++j )
      layer.lint_coor[j] = axis_coor[!layer.direction][layer.lint_axis[j]];
  }

  /* Interlayer Conversions */
  for ( uint32_t i = 0; i < layers.size(); ++i ) {
    for ( uint32_t j = 0; j < layers[i].sublayers.size(); ++j ) {
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
  for ( auto& layer : layers ) {
    for ( auto& sublayer : layer.sublayers ) {
      sublayer.grid_nodes.resize( sublayer.sltra_axis.size() );
      for ( auto& track : sublayer.grid_nodes ) {
        track.resize( layer.lint_axis.size() );
      }
    }
  }

  /* Track Widths */
  for ( const auto& track : tracks ) {
    const auto& dir = track.direction();

    const auto& track_coor = dir ? track.line.upper.x : track.line.upper.y;
    const auto& lower_coor = dir ? track.line.lower.y : track.line.lower.x;
    const auto& upper_coor = dir ? track.line.upper.y : track.line.upper.x;

    auto& layer = layers[track.line.l];
    auto& sublayer = layer.sublayers[track.line.sl];
    auto& grid_nodes = sublayer.grid_nodes;

    uint32_t track_index = find_lower_bound( track_coor, sublayer.sltra_coor );
    uint32_t from_index = find_upper_bound( lower_coor, layer.lint_coor );
    uint32_t to_index = find_lower_bound( upper_coor, layer.lint_coor );

    // If the track is out of bound, ignore it.
    if ( from_index == UINT32_MAX || to_index == UINT32_MAX ) continue;

    // If the track is an internode track, set the width_low of the upper node.
    if ( from_index > to_index ) {
      auto& width_low = grid_nodes[track_index][from_index].width_low;
      if ( width_low < track.width ) width_low = track.width;
      continue;
    }

    // Set the widths of on-track grid nodes if smaller than the track width.
    do {
      auto& width_cur = grid_nodes[track_index][from_index].width_cur;
      if ( width_cur < track.width ) width_cur = track.width;
    } while ( ++from_index <= to_index );
  }
}

void Grid::remove_duplicates( std::vector<uint32_t>& vec )
{
  std::sort( vec.begin(), vec.end() );
  auto it = std::unique( vec.begin(), vec.end() );
  vec.resize( std::distance( vec.begin(), it ) );
}

void Grid::convert_index( std::vector<uint32_t>& conv_vec,
  const std::vector<uint32_t>& vec, const std::vector<uint32_t>& sub_vec )
{
  uint32_t subindex = 0;
  conv_vec.resize( vec.size(), UINT32_MAX );
  for ( uint32_t index = 0; index < conv_vec.size(); ++index ) {
    if ( vec[index] == sub_vec[subindex] ) {
      conv_vec[index] = subindex;
      ++subindex;
    }
  }
}

void Grid::convert_subindex( std::vector<uint32_t>& conv_vec,
  const std::vector<uint32_t>& vec, const std::vector<uint32_t>& sub_vec )
{
  uint32_t index = 0;
  conv_vec.resize( sub_vec.size() );
  for ( uint32_t subindex = 0; subindex < conv_vec.size(); ++subindex ) {
    while ( vec[index] != sub_vec[subindex]) index++;
    conv_vec[subindex] = index;
  }
}

uint32_t Grid::find_lower_bound( const uint32_t& val,
  const std::vector<uint32_t>& vec )
{
  if ( val < vec.front() ) return UINT32_MAX;
  if ( val >= vec.back() ) return ( vec.size()-1 );
  auto it = std::lower_bound( vec.begin(), vec.end(), val );
  return ( val == *it ) ? ( it-vec.begin() ) : ( it-vec.begin()-1 );
}

uint32_t Grid::find_upper_bound( const uint32_t& val,
  const std::vector<uint32_t>& vec )
{
  if ( val < vec.front() ) return 0;
  if ( val > vec.back() ) return UINT32_MAX;
  auto it = std::lower_bound( vec.begin(), vec.end(), val );
  return ( it-vec.begin() );
}
