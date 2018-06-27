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
  for ( auto& bus : buses ) {
    if ( !bus.valid ) continue;
    grid.update_routable_range( bus.bus_widths );
    if ( route( bus, 0, 1 ) ) std::cout << "Successful!" << std::endl;
    else std::cout << "Failed!" << std::endl;
  }
}

bool Router::route( Bus& bus, uint32_t s, uint32_t t )
{
  bool success = 0;
  const auto& source = bus.pinouts[s];
  const auto& target = bus.pinouts[t];
  const auto nbits = source.nodes[0].t_cur.size();
  std::priority_queue<RoutingNode, std::vector<RoutingNode>, RoutingOrder>
    pq;
  Node target_node;
  /* Push source nodes into the routing queue */
  for ( const auto& rn : source.nodes ) {
    pq.push( rn );
    set_source( rn, 1 );
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
    const auto& bw = bus.bus_widths[rn.node.l][rn.node.sl];
    if ( rn.cost > grid_node.cost ) continue;
    // Check if the target is reached
    if ( rn.heading.pre ? grid_node.get_bit(GridNode::tar_low)
      : grid_node.get_bit(GridNode::tar_upp) ) {
      // Find the matched target routing node
      for ( const auto& rn_tar : target.nodes ) {
        if ( rn_tar.node.l == rn.node.l && rn_tar.node.sl == rn.node.sl
          && rn_tar.heading.cur == !rn.heading.cur ) rn.t_cur = rn_tar.t_cur;
      }
      if ( !(grid.check_vias(rn, 0) && grid.check_vias(rn, 1)) )
        continue;
      success = 1;
      target_node = rn.node;
      break;
    }
    // Enqueue neighboring nodes
    RoutingNode rn_q = rn;
    // Current layer
    if ( rn.heading.cur && (rn.node.i != layer.lint_coor.size()-1) ) {
      rn_q.node.i = rn.node.i + 1;
      rn_q.cost += layer.lint_coor[rn_q.node.i] - layer.lint_coor[rn.node.i];
      if ( check_node(rn_q, nbits, bw) ) {
        auto& grid_node = sublayer.grid_nodes[rn_q.node.t][rn_q.node.i];
        grid_node.from = rn_q.i_cur;
        grid_node.set_bit(GridNode::dir, 0);
        pq.push(rn_q);
      }
    }
    if ( !rn.heading.cur && (rn.node.i != 0) ) {
      rn_q = rn;
      rn_q.node.i = rn.node.i - 1;
      rn_q.cost += layer.lint_coor[rn.node.i] - layer.lint_coor[rn_q.node.i];
      if ( check_node(rn_q, nbits, bw) ) {
        auto& grid_node = sublayer.grid_nodes[rn_q.node.t][rn_q.node.i];
        grid_node.from = rn_q.i_cur;
        grid_node.set_bit(GridNode::dir, 0);
        pq.push(rn_q);
      }
    }
    // Change layer
    // If the node is the first node in the path, changing layer is not allowed.
    if ( !rn.locked && rn.node.i == rn.i_cur ) continue;
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
      auto& layer_cur = grid.layers[rn.node.l-1];
      rn_q.node.l = rn.node.l - 1;
      for ( uint8_t sl = 0; sl < layer_cur.sublayers.size(); ++sl ) {
        auto& sublayer_cur = layer_cur.sublayers[sl];
        rn_q.node.sl = sl;
        rn_q.node.t = sublayer_cur.ulint_sltra[rn.node.i];
        if ( rn_q.node.t == -1 ) continue;
        // Lower heading
        rn_q.heading.cur = 0;
        rn_q.i_cur = sublayer.sltra_llint[rn.t_cur.back()];
        rn_q.node.i = rn_q.i_cur;
        if ( check_node(rn_q, nbits, bw) ) {
          auto& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
          grid_node.from = rn.node.t;
          grid_node.set_bit(GridNode::dir, 1);
          grid_node.set_bit(GridNode::dir_l, 0);
          pq.push(rn_q);
        }
        // Upper heading
        rn_q.heading.cur = 1;
        rn_q.i_cur = sublayer.sltra_llint[rn.t_cur.front()];
        rn_q.node.i = rn_q.i_cur;
        if ( check_node(rn_q, nbits, bw) ) {
          auto& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
          grid_node.from = rn.node.t;
          grid_node.set_bit(GridNode::dir, 1);
          grid_node.set_bit(GridNode::dir_l, 0);
          pq.push(rn_q);
        }
      }
    }
    // Upper layer
    if ( rn.node.l != grid.layers.size()-1 ) {
      auto& layer_cur = grid.layers[rn.node.l+1];
      rn_q.node.l = rn.node.l + 1;
      for ( uint8_t sl = 0; sl < layer_cur.sublayers.size(); ++sl ) {
        rn_q.node.sl = sl;
        auto& sublayer_cur = layer_cur.sublayers[sl];
        rn_q.node.t = sublayer_cur.llint_sltra[rn.node.i];
        if ( rn_q.node.t == -1 ) continue;
        // Lower heading
        rn_q.heading.cur = 0;
        rn_q.i_cur = sublayer.sltra_ulint[rn.t_cur.back()];
        rn_q.node.i = rn_q.i_cur;
        if ( check_node(rn_q, nbits, bw) ) {
          auto& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
          grid_node.from = rn.node.t;
          grid_node.set_bit(GridNode::dir, 1);
          grid_node.set_bit(GridNode::dir_l, 1);
          pq.push(rn_q);
        }
        // Upper heading
        rn_q.heading.cur = 1;
        rn_q.i_cur = sublayer.sltra_ulint[rn.t_cur.front()];
        rn_q.node.i = rn_q.i_cur;
        if ( check_node(rn_q, nbits, bw) ) {
          auto& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
          grid_node.from = rn.node.t;
          grid_node.set_bit(GridNode::dir, 1);
          grid_node.set_bit(GridNode::dir_l, 1);
          pq.push(rn_q);
        }
      }
    }
  }
  if ( success ) success = backtrack(bus.route, target_node, source, target,
    bus.bus_widths);
  // Add wires as obstacles to the grid map
  if ( success ) grid.add_obstacles(bus.route.wires);
  /* Unmark all the source/target nodes */
  for ( const auto& rn : source.nodes ) {
    set_source( rn, 0 );
  }
  for ( const auto& rn : target.nodes ) {
    set_target( rn, 0 );
  }
  return success;
}

bool Router::check_node( RoutingNode& rn, const uint8_t nbits,
  const uint16_t bw )
{
  auto& layer = grid.layers[rn.node.l];
  auto& sublayer = layer.sublayers[rn.node.sl];
  auto& grid_node = sublayer.grid_nodes[rn.node.t][rn.node.i];
  if ( rn.heading.pre ? grid_node.get_bit(GridNode::tar_low)
    : grid_node.get_bit(GridNode::tar_upp) ) return 1;
  if ( rn.cost >= grid_node.cost ) return 0;
  // If the node is the first node after changing layer or the node is out
  // of its routable range, update the tracks and check the viability
  if ( !rn.range.contains(rn.node.i) ) {
    if ( rn.locked ) return 0;
    if ( !grid.update_tracks(rn.node, nbits, bw, rn.i_cur, rn.heading.pre,
      rn.t_cur) ) return 0;
    if ( !(grid.check_vias(rn, 0) && grid.check_vias(rn, 1)) ) return 0;
    rn.range = grid.routable_range( rn.node, rn.t_cur, rn.heading.cur );
  }
  grid_node.cost = rn.cost;
  return 1;
}

void Router::set_source( const RoutingNode& rn, bool bit )
{
  auto& layer = grid.layers[rn.node.l];
  auto& sublayer = layer.sublayers[rn.node.sl];
  auto& grid_nodes = sublayer.grid_nodes;
  grid_nodes[rn.t_cur.front()][rn.node.i].set_bit( GridNode::src, bit );
}

void Router::set_target( const RoutingNode& rn, bool bit )
{
  auto& layer = grid.layers[rn.node.l];
  auto& sublayer = layer.sublayers[rn.node.sl];
  auto& grid_nodes = sublayer.grid_nodes;
  grid_nodes[rn.t_cur.front()][rn.node.i].set_bit( GridNode::tar_upp, bit );
  grid_nodes[rn.t_cur.back()][rn.node.i].set_bit( GridNode::tar_low, bit );
}

bool Router::backtrack( BusRoute& route, Node node, const Pinout& source,
  const Pinout& target, const std::vector< std::vector<uint32_t> >& bw )
{
  bool src_reached = 0;
  bool first = 1;
  const auto nbits = source.pin_shapes.size();
  std::vector<Node> nodes;
  bool heading_src;
  bool heading_tar;
  std::vector<uint32_t> t_src;
  std::vector<uint32_t> t_tar;
  // Trace back the route nodes
  while ( !src_reached ) {
    nodes.push_back(node);
    const auto& layer = grid.layers[node.l];
    const auto& sublayer = layer.sublayers[node.sl];
    const auto& grid_node = sublayer.grid_nodes[node.t][node.i];
    if ( first ) {
      // Find the matched target routing node
      for ( const auto& rn_tar : target.nodes ) {
        if ( rn_tar.node.l == node.l && rn_tar.node.sl == node.sl
          && rn_tar.node.i == node.i ) {
          t_tar = rn_tar.t_cur;
          heading_tar = rn_tar.heading.cur;
        }
      }
      first = 0;
    }
    if ( grid_node.get_bit(GridNode::src) ) {
      src_reached = 1;
      // Find the matched source routing node
      for ( const auto& rn_src : source.nodes ) {
        if ( rn_src.node.l == node.l && rn_src.node.sl == node.sl
          && rn_src.node.i == node.i ) {
          t_src = rn_src.t_cur;
          heading_src = rn_src.heading.cur;
        }
      }
      continue;
    }
    if ( grid_node.get_bit(GridNode::dir) ) {
      node.sl = grid_node.from_sl;
      if ( grid_node.get_bit(GridNode::dir_l) ) {
        node.l = node.l - 1;
        node.i = sublayer.sltra_llint[node.t];
      } else {
        node.l = node.l + 1;
        node.i = sublayer.sltra_ulint[node.t];
      }
      node.t = grid_node.from;
    } else {
      node.i = grid_node.from;
    }
  }
  std::reverse(nodes.begin(), nodes.end());
  // Find the original track sets
  RoutingNode rn;
  route.paths.clear();
  for ( uint32_t n = 0; n < nodes.size(); ) {
    Path path;
    path.l = nodes[n].l;
    path.sl = nodes[n].sl;
    rn.node = nodes[n];
    if ( n == 0 ) {
      path.bit_order = source.bit_order;
      rn.heading.cur = heading_src;
      rn.t_cur = t_src;
    } else if ( n >= nodes.size()-2 ) {
      path.bit_order = target.bit_order;
      rn.heading.cur = !heading_tar;
      rn.t_cur = t_tar;
    } else {
      path.bit_order = 1;
      rn.heading.cur = ( nodes[n+1].i > nodes[n].i );
      grid.update_tracks(rn.node, nbits, bw[rn.node.l][rn.node.sl],
        nodes[n+1].i, rn.heading.pre, rn.t_cur );
    }
    path.heading = rn.heading.cur;
    path.t = rn.t_cur;
    if ( n != nodes.size()-1 && nodes[n].l == nodes[n+1].l ) {
      path.i_path = path.heading ? Range(nodes[n].i, nodes[n+1].i)
        : Range(nodes[n+1].i, nodes[n].i);
      n += 2;
    } else {
      path.i_path = Range(nodes[n].i, nodes[n].i);
      n += 1;
    }
    rn.heading.pre = rn.heading.cur;
    rn.t_pre = rn.t_cur;
    rn.l_pre = rn.node.l;
    rn.sl_pre = rn.node.sl;
    route.paths.push_back(path);
  }
  for ( auto p = 0; p < route.paths.size(); ++p ) {
    auto& path = route.paths[p];
    const auto& layer = grid.layers[path.l];
    const auto& sublayer = layer.sublayers[path.sl];
    const auto& dir = layer.direction;
    const auto half_width = bw[path.l][path.sl] >> 1;
    // Convert track indices to coordinates
    path.t_coor.resize(nbits);
    for ( auto n = 0; n < nbits; ++n ) {
      path.t_coor[n] = sublayer.sltra_coor[path.t[n]];
    }
    // Find intersection coordinates
    path.i_coor.resize(nbits);
    if ( p != 0 ) {
      const auto& path_pre = route.paths[p-1];
      const auto& layer_pre = grid.layers[path_pre.l];
      const auto& sublayer_pre = layer_pre.sublayers[path_pre.sl];
      for ( auto n = 0; n < nbits; ++n ) {
        uint32_t tp = n;
        if ( path.bit_order != path_pre.bit_order ) tp = nbits-1-n;
        auto& i_start = path.heading ? path.i_coor[n].low : path.i_coor[n].upp;
        if ( path.l > path_pre.l )
          i_start = sublayer_pre.sltra_ulint[path_pre.t[tp]];
        else
          i_start = sublayer_pre.sltra_llint[path_pre.t[tp]];
        i_start = layer.lint_coor[i_start];
      }
    } else {
      for ( auto n = 0; n < nbits; ++n ) {
        auto& i_end = path.heading ? path.i_coor[n].low : path.i_coor[n].upp;
        if ( path.heading )
          i_end = source.pin_shapes[0].upper.coor[!dir];
        else
          i_end = source.pin_shapes[0].lower.coor[!dir];
      }
    }
    if ( p != route.paths.size()-1 ) {
      const auto& path_next = route.paths[p+1];
      const auto& layer_next = grid.layers[path_next.l];
      const auto& sublayer_next = layer_next.sublayers[path_next.sl];
      for ( auto n = 0; n < nbits; ++n ) {
        uint32_t tn = n;
        if ( path.bit_order != path_next.bit_order ) tn = nbits-1-n;
        auto& i_end = path.heading ? path.i_coor[n].upp : path.i_coor[n].low;
        if ( path.l > path_next.l )
          i_end = sublayer_next.sltra_ulint[path_next.t[tn]];
        else
          i_end = sublayer_next.sltra_llint[path_next.t[tn]];
        i_end = layer.lint_coor[i_end];
      }
    } else {
      for ( auto n = 0; n < nbits; ++n ) {
        auto& i_end = path.heading ? path.i_coor[n].upp : path.i_coor[n].low;
        if ( path.heading )
          i_end = target.pin_shapes[0].lower.coor[!dir];
        else
          i_end = target.pin_shapes[0].upper.coor[!dir];
      }
    }
    for ( auto n = 0; n < nbits; ++n ) {
      Rectangle wire;
      wire.l = path.l;
      wire.sl = path.sl;
      if ( layer.direction ) {
        wire.lower.coor[0] = path.i_coor[n].low;
        wire.lower.coor[1] = path.t_coor[n] - half_width;
        wire.upper.coor[0] = path.i_coor[n].upp;
        wire.upper.coor[1] = path.t_coor[n] + half_width;
      } else {
        wire.lower.coor[1] = path.t_coor[n] - half_width;
        wire.lower.coor[0] = path.i_coor[n].low;
        wire.upper.coor[1] = path.t_coor[n] + half_width;
        wire.upper.coor[0] = path.i_coor[n].upp;
      }
      route.wires.push_back(wire);
    }
  }
  return 1;
}
