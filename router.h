#ifndef _ROUTER_H
#define _ROUTER_H

#include <cstdint>
#include <fstream>
#include <iostream>
#include <queue>
#include <vector>
#include "bus.h"
#include "grid.h"
#include "utils.h"

#define VIA_COST UINT32_MAX
#define PSU_COST VIA_COST/10

struct RoutingOrder
{
public:
  bool operator() ( const RoutingNode& _lhs, const RoutingNode& _rhs )
    { return _lhs.cost > _rhs.cost; }
};

class Router
{
public:
  unsigned int runtime;
  unsigned int alpha;
  unsigned int beta;
  unsigned int gamma;
  unsigned int delta;
  unsigned int epsilon;

  std::ofstream output;

  Grid grid;
  std::vector<Track> tracks;
  std::vector<Rectangle> obstacles;

  std::vector<Bus> buses;

  void initialize( std::string& filename );
  void route_all( void );
private:
  bool route( Bus& bus, unsigned int s, unsigned int t );
  bool check_node( RoutingNode& rn, const unsigned char nbits, const uint16_t bw );
  bool check_node_current_layer( RoutingNode& rn, const unsigned char nbits,
    const uint16_t bw );
  void set_source( const RoutingNode& rn, bool bit );
  void set_target( const RoutingNode& rn, bool bit );
  bool backtrack( BusRoute& route, Node node, const Pinout& source,
    const Pinout& target, const std::vector< std::vector<unsigned int> >& bw,
    std::vector<Pinout> &steiner_tars);
  void output_route( const Bus& bus );
  std::string coor_string( const unsigned int& x, const unsigned int& y );
  void generate_path( BusRoute& route, const Pinout& source, const Pinout& target,
     const std::vector< std::vector<unsigned int> >& bw );
  void path2wire( Bus& bus, const std::vector< std::vector<unsigned int> >& bw );
  bool route_pin2net( Bus& bus, unsigned int s);
  void reduce_overlap_path( Bus &bus);
  void set_psudo_blockage( const Bus& bus, bool bit );
};

#endif
