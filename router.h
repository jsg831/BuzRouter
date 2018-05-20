#ifndef _ROUTER_H
#define _ROUTER_H

#include <cstdint>
#include <vector>
#include "bus.h"
#include "grid.h"
#include "utils.h"

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

  /** Initialization **/
  void initialize( void );
};

#endif
