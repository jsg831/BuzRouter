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
  uint32_t runtime;
  uint32_t alpha;
  uint32_t beta;
  uint32_t gamma;
  uint32_t delta;
  uint32_t epsilon;

  std::ofstream output;

  Grid grid;
  std::vector<Track> tracks;
  std::vector<Rectangle> obstacles;

  std::vector<Bus> buses;

  void initialize( std::string& filename );
  void route_all( void );
private:
  bool route( Bus& bus, uint32_t s, uint32_t t );
  bool check_node( RoutingNode& rn, const uint8_t nbits, const uint16_t bw );
  void set_source( const RoutingNode& rn, bool bit );
  void set_target( const RoutingNode& rn, bool bit );
  bool backtrack( BusRoute& route, Node node, const Pinout& source,
    const Pinout& target, const std::vector< std::vector<uint32_t> >& bw,
    std::vector<Pinout> &steiner_tars);
  void output_route( const Bus& bus );
  std::string coor_string( const uint32_t& x, const uint32_t& y );
  void generate_path( BusRoute& route, const Pinout& source, const Pinout& target,
     const std::vector< std::vector<uint32_t> >& bw );
  void path2wire( Bus& bus, const std::vector< std::vector<uint32_t> >& bw );
  bool route_pin2net( Bus& bus, uint32_t s);
};

#endif
