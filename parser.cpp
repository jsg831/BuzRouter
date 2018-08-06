#include "parser.h"

void Parser::parse( std::string filename, Router& router )
{
  l = 0;
  sl = 0;
  input_file.open( filename );
  std::string line;
  std::string word;
  bool layer_flag = 0;
  bool track_flag = 0;
  bool obstacle_flag = 0;
  bool buses_flag = 0;
  bool bit_flag = 0;
  bool width_flag = 0;
  unsigned int layer_index = 0;
  unsigned int sublayer_index = 0;
  unsigned int bus_flag = 0;
  unsigned int bus_layer_width = 0;
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
        layer_width.resize(layer_width.size() + 1);
        layer_width.back().resize(layer_width.back().size() + 1);
      } else {
        if (layer.direction == router.grid.layers.back().direction ) {
          router.grid.layers.back().sublayers.push_back( sublayer );
          ++sublayer_index;
          layer_width.back().resize(layer_width.back().size() + 1);
        } else {
          layer.sublayers.push_back( sublayer );
          router.grid.layers.push_back( layer );
          sublayer_index = 0;
          ++layer_index;
          layer_width.resize(layer_width.size() + 1);
          layer_width.back().resize(layer_width.back().size() + 1);
        }
      }
      layer_table.insert(std::pair< std::string, std::pair<unsigned int,unsigned int> >
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
      l = 0;
      sl = 0;
      bus_flag = 1;
      ss >> word;
      bus.name = word;
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
    } else if ( width_flag ) {
      Bus &bus = router.buses.back();
      bus.bus_widths.resize(layer_width.size());
      for ( unsigned int n = 0; n < layer_width.size(); ++n ){
        bus.bus_widths[n].resize(layer_width[n].size());
      }
      bus.bus_widths[l][sl] = ( convert(word.substr( 0, word.size()) ));
      if(sl >= layer_width[l].size() - 1){
        l++;
        sl = 0;
      } else {
        sl++;
      }
    } else if ( word == "BIT" ) {
      router.buses.back().bits.resize(router.buses.back().bits.size() + 1);
      Bit& bit = router.buses.back().bits.back();
      ss >> word;
      bit.name = word;
      bit_flag = 1;
    } else if ( word == "ENDBIT" ) {
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

unsigned int Parser::convert( const std::string& str )
{
  return std::strtoul( str.c_str(), nullptr, 10 );
}

bool pin_compare( const Rectangle pin_shapes1, const Rectangle pin_shapes2 )
{
  return pin_shapes1.lower.coor[0] < pin_shapes2.lower.coor[0];
}

void Parser::pinshapes_check( Router& router )
{
  for ( unsigned int b = 0; b < router.buses.size(); b ++ ) {
    Bus& bus = router.buses[b];
    for ( unsigned int n = 1; n < bus.bits.size(); n ++ ) {
      Bit& bit = bus.bits[n];
      std::vector <Rectangle> ref_pinshape = bus.bits[n-1].pin_shapes;
      std::vector <Rectangle> pin_shape_copy = bit.pin_shapes;
      for ( unsigned int p = 0; p < bit.pin_shapes.size(); p ++ ) {
        Rectangle& pin_shape = bit.pin_shapes[p];
        unsigned int min_dis = -1;
        unsigned short min_index = p;
        for ( unsigned int c = 0; c < bit.pin_shapes.size(); c ++ ) {
          if ( min_dis > manhattan_distance(pin_shape_copy[p], ref_pinshape[c])) {
            min_dis = manhattan_distance(pin_shape_copy[p], ref_pinshape[c]);
            min_index = c;
          }
        }
        pin_shape = pin_shape_copy[min_index];
      }
    }
  }
}
int Parser::manhattan_distance( Rectangle a, Rectangle b )
{
  int x_dis = abs((int)a.lower.coor[0] - (int)b.lower.coor[0]);
  int y_dis = abs((int)a.lower.coor[1] - (int)b.lower.coor[1]);
  int l_dis = abs((int)a.l - (int)b.l)*VIA_COST;
  return x_dis + y_dis + l_dis;
}
