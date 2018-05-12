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
    convert_index( layer.ltra_axis, axis, layer.ltra_coor );

    // Sublayer
    for ( auto& sublayer : layer.sublayers ) {
      // Sublayer-track index to axis index
      convert_index( sublayer.sltra_axis, axis, sublayer.sltra_coor );

      // Sublayer-track index to layer-track index
      convert_index( sublayer.sltra_ltra, layer.ltra_axis,
        sublayer.sltra_axis );

      // Generate sublayer-track mask of layer-track indices
      sublayer.sltra_ltra_mask.resize( layer.ltra_axis.size(), 0 );
      for ( const auto& ltra : sublayer.sltra_ltra )
        sublayer.sltra_ltra_mask[ltra] = 1;
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
  uint32_t index = 0;
  conv_vec.clear();
  for ( const auto& val : sub_vec ) {
    while ( vec[index] != val) index++;
    conv_vec.push_back( index );
  }
}
