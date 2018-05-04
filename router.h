#ifndef _ROUTER_H
#define _ROUTER_H

#include <cstdint>
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

  Rectangle design_boundary;
};

#endif
