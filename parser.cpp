#include "parser.h"

void Parser::parse( std::string filename, Router& router )
{
  input_file.open( filename );

  std::string line;
  std::string word;

  bool layer_flag = 0;
  bool track_flag = 0;
  bool obstacle_flag = 0;
  bool buses_flag = 0;
  bool bit_flag = 0;
  bool width_flag = 0;

  uint32_t layer_index = 0;
  uint32_t sublayer_index = 0;
  uint32_t bus_flag = 0;
  uint32_t bus_layer_width = 0;

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
    } else if ( word == "BUSES" ) {
        ss >> word;
        buses_flag = 1;
    } else if ( word == "BUS" ) {
      router.buses.resize( router.buses.size() + 1 );
      Bus &bus = router.buses.back();
      bus_flag = 1;
      ss >> word;
      bus.name = word;
      for ( auto n = 0; n < convert(word.substr( 0, word.size()-1 )); ++n ){
        ss >> word;
        bus.layer_widths.push_back( convert(word.substr( 0, word.size()-1 )) );
      }
      ss >> word;   //"ENDWIDTH"
      bus_flag = 1;
      continue;
    } else if ( word == "ENDBUS" ) {
      bus_flag = 0;
    } else if ( word == "WIDTH" ) {
      ss >> word;
      bus_layer_width = convert(word.substr( 0, word.size() ));
      width_flag = 1;
    } else if ( word == "ENDWIDTH" ) {
      width_flag = 0;
    } else if ( bus_flag == 1 ) {
      ss >> word;
      bus_flag++;
    } else if ( bus_flag == 2 ) {
      bus_flag = 0;
    } else if ( bus_flag == 3 ) {  //Witdh
      ss >> word;
      bus_layer_width = convert(word.substr( 0, word.size() ));
    } else if ( width_flag ) {
      Bus &bus = router.buses.back();
      bus.layer_widths.push_back( convert(word.substr( 0, word.size() )) );
    } else if ( word == "BIT" ) {
      router.buses.back().bits.resize(router.buses.back().bits.size() + 1);
      Bit& bit = router.buses.back().bits.back();
      ss >> word;
      bit.name = word;
      bit_flag = 1;
    } else if( word == "ENDBIT" ) {
      bit_flag = 0;
    } else if ( bit_flag ) {
      Bit& bit = router.buses.back().bits.back();
      bit.pin_shapes.resize(bit.pin_shapes.size() + 1);
      Rectangle &pin = bit.pin_shapes.back();
      std::pair<uint8_t, uint8_t> layer_pair;
      layer_pair = layer_table[word];  // use layer name to search index
      pin.l = layer_pair.first;
      pin.sl = layer_pair.second;
      ss >> word;
      pin.lower.coor[0] = convert( word.substr( 1, word.size()-1 ) );
      ss >> word;
      pin.lower.coor[1] = convert( word.substr( 0, word.size()-1 ) );
      ss >> word;
      pin.upper.coor[0] = convert( word.substr( 1, word.size()-1 ) );
      ss >> word;
      pin.upper.coor[1] = convert( word.substr( 0, word.size()-1 ) );
    }
  }
}

uint32_t Parser::convert( const std::string& str )
{
  return std::strtoul( str.c_str(), nullptr, 10 );
}
