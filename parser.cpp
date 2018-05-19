#include "parser.h"

void Parser::parse( std::string filename, Router& router )
{
  input_file.open( filename );

  std::string line;
  std::string word;

  bool layer_flag = 0;
  bool track_flag = 0;
  bool obstacle_flag = 0;

  uint32_t layer_index = 0;
  uint32_t sublayer_index = 0;

  if ( input_file.fail() ) return;

  while ( !input_file.eof() ) {
    getline( input_file, line );

    ss.str("");
    ss.clear();
    ss << line;
    ss >> word;

    if ( word == "RUNTIME" ) {
      ss >> word;
      router.runtime = convert( word );
    } else if ( word == "ALPHA" ) {
      router.alpha = convert( word );
    } else if ( word == "BETA" ) {
      ss >> word;
      router.beta = convert( word );
    } else if ( word == "GAMMA" ) {
      ss >> word;
      router.gamma = convert( word );
    } else if ( word == "EPSILON" ) {
      ss >> word;
      router.epsilon = convert( word );
    } else if ( word == "DESIGN_BOUNDARY" ) {
      Point& lower = router.grid.boundary.lower;
      Point& upper = router.grid.boundary.upper;
      ss >> word;
      lower.coor[0] = convert( word.substr( 1, word.size()-1 ) );
      ss >> word;
      lower.coor[1] = convert( word.substr( 0, word.size()-1 ) );
      ss >> word;
      upper.coor[0] = convert( word.substr( 1, word.size()-1 ) );
      ss >> word;
      upper.coor[1] = convert( word.substr( 0, word.size()-1 ) );
    } else if ( word == "LAYERS" ) {
      layer_flag = 1;
      ss >> word;
    } else if ( word == "ENDLAYERS" ) {
      layer_flag = 0;
    } else if ( word == "TRACKS" ) {
      track_flag = 1;
      ss >> word;
    } else if ( word == "ENDTRACKS" ) {
      track_flag = 0;
    } else if ( word == "OBSTACLES") {
      obstacle_flag = 1;
    } else if ( word == "ENDOBSTACLES") {
      obstacle_flag = 0;
    } else if ( layer_flag ) {
      Sublayer sublayer;
      Layer layer;

      sublayer.name = word;

      ss >> word;
      layer.direction = (word == "horizontal");

      ss >> word;
      sublayer.spacing = convert( word );

      if ( router.grid.layers.size() == 0 ) {
        layer.sublayers.push_back( sublayer );
        router.grid.layers.push_back( layer );
      } else {
        if (layer.direction == router.grid.layers.back().direction ) {
          router.grid.layers.back().sublayers.push_back( sublayer );
          ++sublayer_index;
        } else {
          layer.sublayers.push_back( sublayer );
          router.grid.layers.push_back( layer );
          sublayer_index = 0;
          ++layer_index;
        }
      }

      layer_table.insert(std::pair< std::string, std::pair<uint32_t,uint32_t> >
        ( sublayer.name, std::make_pair( layer_index, sublayer_index ) ) );
    } else if ( track_flag ) {
      router.tracks.resize( router.tracks.size()+1 );

      Track& track = router.tracks.back();
      track.line.l = layer_table[word].first;
      track.line.sl = layer_table[word].second;

      Point& lower = track.line.lower;
      Point& upper = track.line.upper;

      ss >> word;
      lower.coor[0] = convert( word.substr( 1, word.size()-1 ) );
      ss >> word;
      lower.coor[1] = convert( word.substr( 0, word.size()-1 ) );
      ss >> word;
      upper.coor[0] = convert( word.substr( 1, word.size()-1 ) );
      ss >> word;
      upper.coor[1] = convert( word.substr( 0, word.size()-1 ) );
      ss >> word;
      track.width = convert( word );
    } else if ( obstacle_flag ) {
      router.obstacles.resize( router.obstacles.size()+1 );

      Rectangle& obstacle = router.obstacles.back();
      obstacle.l = layer_table[word].first;
      obstacle.sl = layer_table[word].second;

      Point& lower = obstacle.lower;
      Point& upper = obstacle.upper;

      ss >> word;
      lower.coor[0] = convert( word.substr( 1, word.size()-1 ) );
      ss >> word;
      lower.coor[1] = convert( word.substr( 0, word.size()-1 ) );
      ss >> word;
      upper.coor[0] = convert( word.substr( 1, word.size()-1 ) );
      ss >> word;
      upper.coor[1] = convert( word.substr( 0, word.size()-1 ) );
    }
  }
}

uint32_t Parser::convert( const std::string& str )
{
  return std::strtoul( str.c_str(), nullptr, 10 );
}
