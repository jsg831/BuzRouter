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
  void set_source( const RoutingNode& rn, bool bit );
  void set_target( const RoutingNode& rn, bool bit );
  bool backtrack( BusRoute& route, Node node, const Pinout& source,
    const Pinout& target, const std::vector< std::vector<unsigned int> >& bw );
  void output_route( const Bus& bus );
  std::string coor_string( const unsigned int& x, const unsigned int& y );
};

#endif
