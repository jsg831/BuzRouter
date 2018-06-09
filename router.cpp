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
      if ( pinout.nodes.size() == 0 ) bus.valid = 0;
    }
  }
}

void Router::route_all( void )
{
  for ( const auto& bus : buses ) {
    if ( !bus.valid ) continue;
    grid.update_routable_range( bus.bus_widths );
    bool success = route( bus, 0, 1 );
  }
}

bool Router::route( const Bus& bus, uint32_t s, uint32_t t )
{
  bool success = 0;
  const auto& source = bus.pinouts[s];
  const auto& target = bus.pinouts[t];
  const auto nbits = source.nodes[0].t_cur.size();
  std::priority_queue<RoutingNode, std::vector<RoutingNode>, RoutingOrder>
    pq;
  std::vector<Node> visited_nodes;
  /* Push source nodes into the routing queue */
  for ( const auto& rn : source.nodes ) {
    pq.push( rn );
  }
  /* Mark all the available target nodes */
  for ( const auto& rn : target.nodes ) {
    set_target( rn, 1 );
  }
  /* Maze routing */
  while ( !pq.empty() ) {
    RoutingNode rn = pq.top();
    pq.pop();
    auto& layer = grid.layers[rn.node.l];
    auto& sublayer = layer.sublayers[rn.node.sl];
    auto& grid_node = sublayer.grid_nodes[rn.node.t][rn.node.i];
    if ( rn.cost >= grid_node.cost ) continue;
    const auto& bw = bus.bus_widths[rn.node.l][rn.node.sl];
    // Check if the target is reached
    if ( rn.heading.pre ? grid_node.get_bit(GridNode::tar_low)
      : grid_node.get_bit(GridNode::tar_upp) ) {
      // Find the matched target routing node
      for ( const auto& rn_tar : target.nodes ) {
        if ( rn_tar.node.l == rn.node.l && rn_tar.node.sl == rn.node.sl
          && rn_tar.heading.cur == !rn.heading.cur ) rn.t_cur = rn_tar.t_cur;
      }
      if ( grid.check_vias(rn, 0) || grid.check_vias(rn, 1) ) {
        success = 1;
        break;
      }
      else continue;
    }
    grid_node.cost = rn.cost;
    // If the node is the first node after changing layer or the node is out
    // of its routable range, update the tracks and check the viability
    if ( !rn.range.contains(rn.node.i) ) {
      if ( rn.locked ) continue;
      if ( !grid.update_tracks(rn.node, nbits, bw, rn.i_cur, rn.heading.pre,
        rn.t_cur) ) continue;
      if ( !(grid.check_vias(rn, 0) || grid.check_vias(rn, 1)) ) continue;
      rn.range = grid.routable_range( rn.node, rn.t_cur, rn.heading.cur );
    }
    // Enqueue neighboring nodes
    RoutingNode rn_q = rn;
    // Current layer
    if ( rn.heading.cur && (rn.node.i != layer.lint_coor.size()-1) ) {
      rn_q.node.i = rn.node.i + 1;
      rn_q.cost += layer.lint_coor[rn_q.node.i] - layer.lint_coor[rn.node.i];
      pq.push( rn_q );
    }
    if ( !rn.heading.cur && (rn.node.i != 0) ) {
      rn_q = rn;
      rn_q.node.i = rn.node.i - 1;
      rn_q.cost += layer.lint_coor[rn.node.i] - layer.lint_coor[rn_q.node.i];
      pq.push( rn_q );
    }
    // Change layer
    rn_q.locked = 0;
    rn_q.cost = rn.cost + VIA_COST;
    rn_q.heading.pre = rn.heading.cur;
    rn_q.l_pre = rn.node.l;
    rn_q.sl_pre = rn.node.sl;
    rn_q.i_pre = rn.node.i;
    rn_q.t_pre = rn.t_cur;
    rn_q.t_cur.clear();
    rn_q.range = Range(-1, -1);
    // Lower layer
    if ( rn.node.l != 0 ) {
      const auto& layer_cur = grid.layers[rn.node.l-1];
      rn_q.node.l = rn.node.l - 1;
      for ( uint8_t sl = 0; sl < layer_cur.sublayers.size(); ++sl ) {
        const auto& sublayer_cur = layer_cur.sublayers[sl];
        rn_q.node.sl = sl;
        rn_q.node.t = sublayer_cur.ulint_sltra[rn.node.i];
        if ( rn_q.node.t == -1 ) continue;
        // Lower heading
        rn_q.heading.cur = 0;
        rn_q.i_cur = sublayer.sltra_llint[rn.t_cur.back()];
        rn_q.node.i = rn_q.i_cur;
        pq.push( rn_q );
        // Upper heading
        rn_q.heading.cur = 1;
        rn_q.i_cur = sublayer.sltra_llint[rn.t_cur.front()];
        rn_q.node.i = rn_q.i_cur;
        pq.push( rn_q );
      }
    }
    // Upper layer
    if ( rn.node.l != grid.layers.size()-1 ) {
      const auto& layer_cur = grid.layers[rn.node.l+1];
      rn_q.node.l = rn.node.l + 1;
      for ( uint8_t sl = 0; sl < layer_cur.sublayers.size(); ++sl ) {
        rn_q.node.sl = sl;
        const auto& sublayer_cur = layer_cur.sublayers[sl];
        rn_q.node.t = sublayer_cur.llint_sltra[rn.node.i];
        if ( rn_q.node.t == -1 ) continue;
        // Lower heading
        rn_q.heading.cur = 0;
        rn_q.i_cur = sublayer.sltra_ulint[rn.t_cur.back()];
        rn_q.node.i = rn_q.i_cur;
        pq.push( rn_q );
        // Upper heading
        rn_q.heading.cur = 1;
        rn_q.i_cur = sublayer.sltra_ulint[rn.t_cur.front()];
        rn_q.node.i = rn_q.i_cur;
        pq.push( rn_q );
      }
    }
  }
  /* Unmark all the target nodes */
  for ( const auto& rn : target.nodes ) {
    set_target( rn, 0 );
  }
  return success;
}

void Router::set_target( const RoutingNode& rn, bool bit )
{
  auto& layer = grid.layers[rn.node.l];
  auto& sublayer = layer.sublayers[rn.node.sl];
  auto& grid_nodes = sublayer.grid_nodes;
  grid_nodes[rn.t_cur.front()][rn.node.i].set_bit( GridNode::tar_upp, bit );
  grid_nodes[rn.t_cur.back()][rn.node.i].set_bit( GridNode::tar_low, bit );
}
