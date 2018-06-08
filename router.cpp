#include "router.h"

void Router::initialize( void )
{
  grid.make_grid( tracks );
  grid.add_obstacles( obstacles );
  for ( auto& bus : buses ) {
    bus.initialize();
    grid.update_routable_range( bus.bus_widths );
    for ( auto& pinout : bus.pinouts ) {
      RoutingNode rn;
      const auto l_pin = pinout.pin_shapes[0].l;
      for ( auto d = -1; d <= 1; ++d ) {
        if ( l_pin == 0 && d == -1 ) continue;
        if ( l_pin == grid.layers.size() - 1 && d == 1 ) continue;
        auto l = l_pin + d;
        for ( auto sl = 0; sl < grid.layers[l].sublayers.size(); ++sl ) {
          grid.generate_pinout_nodes( pinout, l, sl, 0 );
          grid.generate_pinout_nodes( pinout, l, sl, 1 );
        }
      }
    }
  }
}
