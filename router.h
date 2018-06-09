#ifndef _ROUTER_H
#define _ROUTER_H

#include <cstdint>
#include <queue>
#include <vector>
#include "bus.h"
#include "grid.h"
#include "utils.h"

#define VIA_COST 10000

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

  Grid grid;
  std::vector<Track> tracks;
  std::vector<Rectangle> obstacles;

  std::vector<Bus> buses;

  void initialize( void );
  void route_all( void );
private:
  bool route( const Bus& bus, uint32_t s, uint32_t t );
  void set_target( const RoutingNode& rn, bool bit );
  void backtrack( const Node& source, const Node& target );
  void clear_marks( const std::vector<Node>& nodes );
};

#endif
