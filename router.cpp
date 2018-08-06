#include "router.h"

void Router::initialize( std::string& filename )
{
  output.open( filename );
  grid.make_grid( tracks );
  grid.add_obstacles( obstacles );
  for ( int b = 0; b < buses.size(); ++b ) {
    Bus& bus = buses[b];
    std::cout << "bus = " << bus.name << std::endl;
    bus.initialize();
    grid.update_routable_range( bus.bus_widths );
    for ( int p = 0; p < bus.pinouts.size(); ++p ) {
      Pinout& pinout = bus.pinouts[p];
      RoutingNode rn;
      const unsigned char l_pin = pinout.pin_shapes[0].l;
      for ( int d = -1; d <= 1; ++d ) {
        if ( l_pin == 0 && d == -1 ) continue;
        if ( l_pin == grid.layers.size() - 1 && d == 1 ) continue;
        unsigned char l = l_pin + d;
        for ( unsigned char sl = 0; sl < grid.layers[l].sublayers.size(); ++sl ) {
          unsigned int spacing = grid.layers[l].sublayers[sl].spacing;
          grid.generate_pinout_nodes( pinout, l, sl, 0, bus.bus_widths[l][sl],
            obstacles, spacing);
          grid.generate_pinout_nodes( pinout, l, sl, 1, bus.bus_widths[l][sl],
            obstacles, spacing);
        }
      }
      if ( pinout.nodes.size() == 0 ) bus.valid = 0;
      else bus.valid = 1;
    }
    set_psudo_blockage( bus , 1 );
  }
}

void Router::route_all( void )
{
  bool success = 0;
  for (int k = 0; k < buses.size(); k ++ ) { //sequence
  //for ( int k = buses.size() - 1; k >= 0; k --) { //inverse
    Bus& bus = buses[k];
    set_psudo_blockage( bus , 0 );
    if ( !bus.valid ) continue;
    std::cout << "BUS " << bus.name << ": ";
    for( unsigned char n = 0; n < bus.pinouts.size(); n ++) {
      grid.update_routable_range( bus.bus_widths );
      // for first 2 pin route
      if ( n == 0 ) {
        success = route( bus, 0, 1 );
        n = 1;
        if (success == 0) break;
        std::cout << "first 2pin route success" << std::endl;
        const Pinout& source = bus.pinouts[0];
        for ( unsigned int n = 0; n < source.nodes.size(); ++n ) {
          const RoutingNode& rn = source.nodes[n];
          set_source( rn, 0 );
        }
        const Pinout& source1 = bus.pinouts[1];
        for ( unsigned int n = 0; n < source1.nodes.size(); ++n ) {
          const RoutingNode& rn = source1.nodes[n];
          set_source( rn, 0 );
        }
      }
      // for pin to net route (pin index >=2)
      else{
        success = route_pin2net( bus, n );
        if (success == 0) break;
        std::cout << "pin2net route success" << std::endl;
        const Pinout& target = bus.pinouts[n];
        for ( unsigned int n = 0; n < target.nodes.size(); ++n ) {
          const RoutingNode& rn = target.nodes[n];
          set_source( rn, 0 );
        }

      }
      // set dir dir_l to 0
      for ( Layer&layer : grid.layers) {
        for ( Sublayer& sublayer : layer.sublayers) {
          for ( std::vector <GridNode>& grid_track : sublayer.grid_nodes) {
            for ( GridNode& gridnode : grid_track){
              gridnode.set_bit( GridNode::dir, 0 );
              gridnode.set_bit( GridNode::dir_l, 0 );
            }
          }
        }
      }
    }
    if (success) {
      // Add wires as obstacles to the grid map
      reduce_overlap_path( bus );
      for (BusRoute& route : bus.routes) {
        path2wire( bus, bus.bus_widths );
        grid.add_obstacles(route.wires);
      }
      std::cout << "Successful!!!!!!!!!!!!!!!!!!!!" << std::endl;
      output_route( bus );
    } else {
      std::cout << "Failed!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
      reduce_overlap_path( bus );
      for (BusRoute& route : bus.routes) {
        path2wire( bus, bus.bus_widths );
      }
      output_route( bus );
    }
    const Pinout& source = bus.pinouts[0];
    for ( unsigned int n = 0; n < source.nodes.size(); ++n ) {
      const RoutingNode& rn = source.nodes[n];
      set_source( rn, 0 );
    }
    const Pinout& target = bus.pinouts[0];
    for ( unsigned int n = 0; n < target.nodes.size(); ++n ) {
      const RoutingNode& rn = target.nodes[n];
      set_target( rn, 0 );
    }
    // reset target info in gridmap
    for ( Layer& layer : grid.layers) {
      for ( Sublayer& sublayer : layer.sublayers) {
        for ( std::vector<GridNode>& grid_track : sublayer.grid_nodes) {
          for ( GridNode& gridnode : grid_track){
            gridnode.set_bit( GridNode::tar_low, 0 );
            gridnode.set_bit( GridNode::tar_upp, 0 );
          }
        }
      }
    }
  }
  output.close();
}

bool Router::route( Bus& bus, unsigned int s, unsigned int t )
{
  bool success = 0;
  const Pinout& source = bus.pinouts[s];
  const Pinout& target = bus.pinouts[t];
  const unsigned int nbits = source.nodes[0].t_cur.size();
  std::priority_queue<RoutingNode, std::vector<RoutingNode>, RoutingOrder>
    pq;
  Node target_node;
  /* Push source nodes into the routing queue */
  for ( unsigned int n = 0; n < source.nodes.size(); ++n ) {
    const RoutingNode& rn = source.nodes[n];
    pq.push( rn );
    set_source( rn, 1 );
  }
  /* Mark all the available target nodes */
  for ( unsigned int n = 0; n < target.nodes.size(); ++n ) {
    const RoutingNode& rn = target.nodes[n];
    set_target( rn, 1 );
  }
  /* Maze routing */
  while ( !pq.empty() ) {
    RoutingNode rn = pq.top();
    pq.pop();
    Layer& layer = grid.layers[rn.node.l];
    Sublayer& sublayer = layer.sublayers[rn.node.sl];
    GridNode& grid_node = sublayer.grid_nodes[rn.node.t][rn.node.i];
    const unsigned int& bw = bus.bus_widths[rn.node.l][rn.node.sl];

    if ( rn.cost > grid_node.cost ) continue;
    // Check if the target is reached
    if ( rn.heading.pre ? grid_node.get_bit(GridNode::tar_low)
      : grid_node.get_bit(GridNode::tar_upp) ) {
      // Find the matched target routing node
      for ( unsigned int n = 0; n < target.nodes.size(); ++n ) {
        const RoutingNode& rn_tar = target.nodes[n];
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
      // psudo blockage
      if ( grid_node.get_bit( GridNode::psu )){
        rn_q.cost += 10 * VIA_COST;
        //std::cout << "PSUDO!!!! \n";
      }
      if ( check_node(rn_q, nbits, bw) ) {
        GridNode& grid_node = sublayer.grid_nodes[rn_q.node.t][rn_q.node.i];
        grid_node.from = rn_q.i_cur;
        grid_node.set_bit(GridNode::dir, 0);
        pq.push(rn_q);
      }
    }
    if ( !rn.heading.cur && (rn.node.i != 0) ) {
      rn_q = rn;
      rn_q.node.i = rn.node.i - 1;
      rn_q.cost += layer.lint_coor[rn.node.i] - layer.lint_coor[rn_q.node.i];
      // psudo blockage
      if ( grid_node.get_bit( GridNode::psu )){
        rn_q.cost += 10 * VIA_COST;
        //std::cout << "PSUDO!!!! \n";
      }
      if ( check_node(rn_q, nbits, bw) ) {
        GridNode& grid_node = sublayer.grid_nodes[rn_q.node.t][rn_q.node.i];
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
    // psudo blockage
    if ( grid_node.get_bit( GridNode::psu )){
      rn_q.cost += 10 * VIA_COST;
      //std::cout << "PSUDO!!!! \n";
    }
    rn_q.heading.pre = rn.heading.cur;
    rn_q.l_pre = rn.node.l;
    rn_q.sl_pre = rn.node.sl;
    rn_q.i_pre = rn.node.i;
    rn_q.t_pre = rn.t_cur;
    rn_q.t_cur.clear();
    rn_q.range = Range(-1, -1);
    // Lower layer
    if ( rn.node.l != 0 ) {
      Layer& layer_cur = grid.layers[rn.node.l-1];
      rn_q.node.l = rn.node.l - 1;
      for ( unsigned char sl = 0; sl < layer_cur.sublayers.size(); ++sl ) {
        const unsigned int& bw_low = bus.bus_widths[rn_q.node.l][sl];
        Sublayer& sublayer_cur = layer_cur.sublayers[sl];
        rn_q.node.sl = sl;
        rn_q.node.t = sublayer_cur.ulint_sltra[rn.node.i];
        if ( rn_q.node.t == -1 ) continue;
        // Lower heading
        rn_q.heading.cur = 0;
        rn_q.i_cur = sublayer.sltra_llint[rn.t_cur.back()];
        rn_q.node.i = rn_q.i_cur;
        if ( check_node(rn_q, nbits, bw_low) ) {
          GridNode& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
          grid_node.from = rn.node.t;
          grid_node.set_bit(GridNode::dir, 1);
          grid_node.set_bit(GridNode::dir_l, 0);
          pq.push(rn_q);
        }
        // Upper heading
        rn_q.heading.cur = 1;
        rn_q.i_cur = sublayer.sltra_llint[rn.t_cur.front()];
        rn_q.node.i = rn_q.i_cur;
        if ( check_node(rn_q, nbits, bw_low) ) {
          GridNode& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
          grid_node.from = rn.node.t;
          grid_node.set_bit(GridNode::dir, 1);
          grid_node.set_bit(GridNode::dir_l, 0);
          pq.push(rn_q);
        }
      }
    }
    // Upper layer
    if ( rn.node.l != grid.layers.size()-1 ) {
      Layer& layer_cur = grid.layers[rn.node.l+1];
      rn_q.node.l = rn.node.l + 1;
      for ( unsigned char sl = 0; sl < layer_cur.sublayers.size(); ++sl ) {
        const unsigned int& bw_upp = bus.bus_widths[rn_q.node.l][sl];
        rn_q.node.sl = sl;
        Sublayer& sublayer_cur = layer_cur.sublayers[sl];
        rn_q.node.t = sublayer_cur.llint_sltra[rn.node.i];
        if ( rn_q.node.t == -1 ) continue;
        // Lower heading
        rn_q.heading.cur = 0;
        rn_q.i_cur = sublayer.sltra_ulint[rn.t_cur.back()];
        rn_q.node.i = rn_q.i_cur;
        if ( check_node(rn_q, nbits, bw_upp) ) {
          GridNode& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
          grid_node.from = rn.node.t;
          grid_node.set_bit(GridNode::dir, 1);
          grid_node.set_bit(GridNode::dir_l, 1);
          pq.push(rn_q);
        }
        // Upper heading
        rn_q.heading.cur = 1;
        rn_q.i_cur = sublayer.sltra_ulint[rn.t_cur.front()];
        rn_q.node.i = rn_q.i_cur;
        if ( check_node(rn_q, nbits, bw_upp) ) {
          GridNode& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
          grid_node.from = rn.node.t;
          grid_node.set_bit(GridNode::dir, 1);
          grid_node.set_bit(GridNode::dir_l, 1);
          pq.push(rn_q);
        }
      }
    }
  }
  if ( success ) {

    bus.routes.resize(bus.routes.size() + 1);
    BusRoute& route = bus.routes.back();
    route.l_src = source.pin_shapes[0].l;
    route.sl_src = source.pin_shapes[0].sl;
    route.l_tar = target.pin_shapes[0].l;
    route.sl_tar = target.pin_shapes[0].sl;

    success = backtrack(route, target_node, source, target,
      bus.bus_widths, bus.steiner_tars);
    //generate path and path to wire
    generate_path(route, source, target, bus.bus_widths);
  }
  return success;
}

bool Router::route_pin2net( Bus& bus, unsigned int s)
{
  bool success = 0;
  const Pinout& source = bus.pinouts[s];
  Pinout target;
  const unsigned int nbits = source.nodes[0].t_cur.size();
  std::priority_queue<RoutingNode, std::vector<RoutingNode>, RoutingOrder>
    pq;
  Node target_node;
  /* Push source nodes into the routing queue */
  for ( const RoutingNode& rn : source.nodes ) {
    pq.push( rn );
    set_source( rn, 1 );
  }

  /* Maze routing */
  while ( !pq.empty() ) {
    RoutingNode rn = pq.top();
    pq.pop();
    Layer& layer = grid.layers[rn.node.l];
    Sublayer& sublayer = layer.sublayers[rn.node.sl];
    GridNode& grid_node = sublayer.grid_nodes[rn.node.t][rn.node.i];
    const unsigned int& bw = bus.bus_widths[rn.node.l][rn.node.sl];
    /*if ( bus.name == "bus4") {
      std::cout << " t = " << sublayer.sltra_coor[rn.node.t] << " i = "
        << layer.lint_coor[rn.node.i] << std::endl;
    }*/

    // Check if the target is reached
    if ( rn.heading.pre ? grid_node.get_bit(GridNode::tar_low)
      : grid_node.get_bit(GridNode::tar_upp) ) {
      // Find the matched target routing node
      for ( Pinout& pinout_tar : bus.steiner_tars ) {
        const RoutingNode& rn_tar = pinout_tar.nodes[0];
        //find correct target track
        const unsigned int& correct_rn_tar_t = rn.heading.pre ? rn_tar.t_cur.back() : rn_tar.t_cur.front();
        if ( rn_tar.node.l == rn.node.l && rn_tar.node.sl == rn.node.sl
          && rn_tar.heading.cur == !rn.heading.cur &&  correct_rn_tar_t == rn.node.t )
        {
          Rectangle rec;
          rec.l = rn.node.l;
          rec.sl = rn.node.sl;
          //rec.lower.coor[layer.direction] = sublayer.sltra_coor[rn.node.t] - bus.bus_widths[rec.l][rec.sl] / 2;
          //rec.upper.coor[layer.direction] = rec.lower.coor[!layer.direction] + bus.bus_widths[rec.l][rec.sl];

          //copy the info of target node to generate correct pinout
          for (unsigned char n = 0; n < bus.bits.size(); n ++) {
            pinout_tar.pin_shapes[n].lower.coor[!layer.direction] = layer.lint_coor[rn.node.i] - bus.bus_widths[rec.l][rec.sl] / 2;
            pinout_tar.pin_shapes[n].upper.coor[!layer.direction] = pinout_tar.pin_shapes[n].lower.coor[!layer.direction] + bus.bus_widths[rec.l][rec.sl];
          }
          rn.t_cur = rn_tar.t_cur;
          target = pinout_tar;
          target.nodes[0] = rn;
          target.nodes[0].range = grid.routable_range( target.nodes[0].node, target.nodes[0].t_cur,
            target.nodes[0].heading.cur );
          break;
        }
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
      // psudo blockage
      if ( grid_node.get_bit( GridNode::psu )){
        rn_q.cost += 10 * VIA_COST;
        //std::cout << "PSUDO!!!! \n";
      }
      if ( check_node(rn_q, nbits, bw) ) {
        GridNode& grid_node = sublayer.grid_nodes[rn_q.node.t][rn_q.node.i];
        grid_node.from = rn_q.i_cur;
        grid_node.set_bit(GridNode::dir, 0);
        pq.push(rn_q);
      }
    }
    if ( !rn.heading.cur && (rn.node.i != 0) ) {
      rn_q = rn;
      rn_q.node.i = rn.node.i - 1;
      rn_q.cost += layer.lint_coor[rn.node.i] - layer.lint_coor[rn_q.node.i];
      // psudo blockage
      if ( grid_node.get_bit( GridNode::psu )){
        rn_q.cost += 10 * VIA_COST;
        //std::cout << "PSUDO!!!! \n";
      }
      if ( check_node(rn_q, nbits, bw) ) {
        GridNode& grid_node = sublayer.grid_nodes[rn_q.node.t][rn_q.node.i];
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
    // psudo blockage
    if ( grid_node.get_bit( GridNode::psu )){
      rn_q.cost += 10 * VIA_COST;
      //std::cout << "PSUDO!!!! \n";
    }
    rn_q.heading.pre = rn.heading.cur;
    rn_q.l_pre = rn.node.l;
    rn_q.sl_pre = rn.node.sl;
    rn_q.i_pre = rn.node.i;
    rn_q.t_pre = rn.t_cur;
    rn_q.t_cur.clear();
    rn_q.range = Range(-1, -1);
    // Lower layer
    if ( rn.node.l != 0 ) {
      Layer& layer_cur = grid.layers[rn.node.l-1];
      rn_q.node.l = rn.node.l - 1;
      for ( unsigned char sl = 0; sl < layer_cur.sublayers.size(); ++sl ) {
        Sublayer& sublayer_cur = layer_cur.sublayers[sl];
        rn_q.node.sl = sl;
        rn_q.node.t = sublayer_cur.ulint_sltra[rn.node.i];
        if ( rn_q.node.t == -1 ) continue;
        // Lower heading
        rn_q.heading.cur = 0;
        rn_q.i_cur = sublayer.sltra_llint[rn.t_cur.back()];
        rn_q.node.i = rn_q.i_cur;
        if ( check_node(rn_q, nbits, bw) ) {
          GridNode& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
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
          GridNode& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
          grid_node.from = rn.node.t;
          grid_node.set_bit(GridNode::dir, 1);
          grid_node.set_bit(GridNode::dir_l, 0);
          pq.push(rn_q);
        }
      }
    }
    // Upper layer
    if ( rn.node.l != grid.layers.size()-1 ) {
      Layer& layer_cur = grid.layers[rn.node.l+1];
      rn_q.node.l = rn.node.l + 1;
      for ( unsigned char sl = 0; sl < layer_cur.sublayers.size(); ++sl ) {
        rn_q.node.sl = sl;
        Sublayer& sublayer_cur = layer_cur.sublayers[sl];
        rn_q.node.t = sublayer_cur.llint_sltra[rn.node.i];
        if ( rn_q.node.t == -1 ) continue;
        // Lower heading
        rn_q.heading.cur = 0;
        rn_q.i_cur = sublayer.sltra_ulint[rn.t_cur.back()];
        rn_q.node.i = rn_q.i_cur;
        if ( check_node(rn_q, nbits, bw) ) {
          GridNode& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
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
          GridNode& grid_node = sublayer_cur.grid_nodes[rn_q.node.t][rn_q.node.i];
          grid_node.from = rn.node.t;
          grid_node.set_bit(GridNode::dir, 1);
          grid_node.set_bit(GridNode::dir_l, 1);
          pq.push(rn_q);
        }
      }
    }
  }
  if ( success ) {
    bus.routes.resize(bus.routes.size() + 1);
    BusRoute& route = bus.routes.back();
    route.l_src = source.pin_shapes[0].l;
    route.sl_src = source.pin_shapes[0].sl;
    route.l_tar = target_node.l;
    route.sl_tar = target_node.sl;
    success = backtrack(route, target_node, source, target,
      bus.bus_widths, bus.steiner_tars);
    //generate path and path to wire
    generate_path(route, source, target, bus.bus_widths);
  }
  return success;
}

bool Router::check_node( RoutingNode& rn, const unsigned char nbits,
  const uint16_t bw )
{
  Layer& layer = grid.layers[rn.node.l];
  Sublayer& sublayer = layer.sublayers[rn.node.sl];
  GridNode& grid_node = sublayer.grid_nodes[rn.node.t][rn.node.i];
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
  Layer& layer = grid.layers[rn.node.l];
  Sublayer& sublayer = layer.sublayers[rn.node.sl];
  std::vector <std::vector<GridNode> >& grid_nodes = sublayer.grid_nodes;
  grid_nodes[rn.t_cur.front()][rn.node.i].set_bit( GridNode::src, bit );
}

void Router::set_psudo_blockage( const Bus& bus, bool bit )
{
  //std::cout << "bus = " << bus.name << std::endl;
  //std::cout << "pinout size = " << bus.pinouts.size() << std::endl;
  for ( int p = 0; p < bus.pinouts.size(); p ++) {
    Pinout pinout = bus.pinouts[p];
    //std::cout << "pinout node size = " << pinout.nodes.size() << std::endl;
    for ( int r = 0; r < pinout.nodes.size(); r ++) {
      RoutingNode rn = pinout.nodes[r];
      Layer& layer = grid.layers[rn.node.l];
      Sublayer& sublayer = layer.sublayers[rn.node.sl];
      std::vector<std::vector<GridNode>>& grid_nodes = sublayer.grid_nodes;
      RoutingNode rn_q = rn;
      unsigned char extend_const = 25;
      unsigned int head = rn.t_cur.front();
      unsigned int tail = rn.t_cur.back();
      if ( head > tail ) {
        unsigned int tmp = head;
        head = tail;
        tail = tmp;
      }
      //std::cout << "head = " << head << "  tail = " << tail << std::endl;
      for ( unsigned char c = 0; c < extend_const; c ++) {
        if ( head <= 0 ) break;
        head --;
      }
      for ( unsigned char c = 0; c < extend_const; c ++) {
        if ( tail >= sublayer.sltra_axis.size() - 1 ) break;
        tail ++;
      }
      //std::cout << "ext head = " << head << "  ext back = " << tail << std::endl;
      bool stop_extend = 0;
      while ( true ) {
        if (stop_extend && bit ) break;
        if ( abs (rn_q.node.i - rn.node.i ) > bus.bits.size() + extend_const*2 ) break;
        if ( rn_q.node.i == layer.lint_coor.size() && rn.heading.cur) break;
        if ( rn_q.node.i == -1 && !rn.heading.cur) break;
        // set psu from pinout to head to tail
        for ( unsigned int track = head; track <= tail; track ++ ) {
          if ( grid_nodes[track][rn_q.node.i].get_bit( GridNode::obs ))
            stop_extend = 1;
          if ( stop_extend && bit ) {
            //std::cout << bus.name << "obs jump!!\n";
            break;
            continue;
          }
          // current layer
          grid_nodes[track][rn_q.node.i].set_bit( GridNode::psu, bit );
          // Lower layer
          if ( rn.node.l != 0 ) {
            Layer& layer_cur = grid.layers[rn.node.l-1];
            for ( unsigned char sl = 0; sl < layer_cur.sublayers.size(); ++sl ) {
              Sublayer& sublayer_cur = layer_cur.sublayers[sl];
              unsigned int t = sublayer_cur.ulint_sltra[rn_q.node.i];
              if ( t == -1 ) continue;
              unsigned int i = sublayer.sltra_llint[track];
              sublayer_cur.grid_nodes[t][i].set_bit( GridNode::psu, bit );
            }
          }
          // upper layer
          if ( rn.node.l != grid.layers.size()-1 ) {
            Layer& layer_cur = grid.layers[rn.node.l+1];
            for ( unsigned char sl = 0; sl < layer_cur.sublayers.size(); ++sl ) {
              Sublayer& sublayer_cur = layer_cur.sublayers[sl];
              unsigned int t = sublayer_cur.llint_sltra[rn_q.node.i];
              if ( t == -1 ) continue;
              unsigned int i = sublayer.sltra_ulint[track];
              sublayer_cur.grid_nodes[t][i].set_bit( GridNode::psu, bit );
            }
          }
        }
        rn_q.node.i = (rn.heading.cur) ? rn_q.node.i + 1 : rn_q.node.i - 1;
      }
    }
  }
}




void Router::set_target( const RoutingNode& rn, bool bit )
{
  Layer& layer = grid.layers[rn.node.l];
  Sublayer& sublayer = layer.sublayers[rn.node.sl];
  std::vector <std::vector<GridNode> >& grid_nodes = sublayer.grid_nodes;
  grid_nodes[rn.t_cur.front()][rn.node.i].set_bit( GridNode::tar_upp, bit );
  grid_nodes[rn.t_cur.back()][rn.node.i].set_bit( GridNode::tar_low, bit );
}

bool Router::backtrack( BusRoute& route, Node node, const Pinout& source,
  const Pinout& target, const std::vector< std::vector<unsigned int> >& bw,
  std::vector<Pinout> &steiner_tars)
{
  bool src_reached = 0;
  bool first = 1;
  const unsigned int nbits = source.pin_shapes.size();
  std::vector<Node> nodes;
  bool heading_src;
  bool heading_tar;
  std::vector<unsigned int> t_src;
  std::vector<unsigned int> t_tar;
  // Trace back the route nodes
  while ( !src_reached ) {
    nodes.push_back(node);
    const Layer& layer = grid.layers[node.l];
    const Sublayer& sublayer = layer.sublayers[node.sl];
    const GridNode& grid_node = sublayer.grid_nodes[node.t][node.i];
    if ( first ) {
      // Find the matched target routing node
      for ( unsigned int n = 0; n < target.nodes.size(); ++n ) {
        const RoutingNode& rn_tar = target.nodes[n];
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
      for ( unsigned int n = 0; n < source.nodes.size(); ++n ) {
        const RoutingNode& rn_src = source.nodes[n];
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
  //route.paths.clear();
  for ( unsigned int n = 0; n < nodes.size(); ) {
    Path path;
    path.l = nodes[n].l;
    path.sl = nodes[n].sl;
    rn.node = nodes[n];
    if ( n == 0 ) {
      path.bit_order = source.bit_order;
      rn.heading.cur = heading_src;
      rn.t_cur = t_src;
      rn.i_cur = source.nodes[0].node.i;
    } else if ( n >= nodes.size()-2 ) {
      path.bit_order = target.bit_order;
      rn.heading.cur = !heading_tar;
      rn.t_cur = t_tar;
      rn.i_cur = target.nodes[0].node.i;
    } else {
      path.bit_order = 1;
      rn.heading.cur = ( nodes[n+1].i > nodes[n].i );
      grid.update_tracks(rn.node, nbits, bw[rn.node.l][rn.node.sl],
        nodes[n+1].i, rn.heading.pre, rn.t_cur );
    }

    //grid.generate_pinout_tar
    Pinout steiner_tar;
    steiner_tar.direction = grid.layers[rn.node.l].direction;
    steiner_tar.bit_order = path.bit_order;

    Layer& layer = grid.layers[rn.node.l];
    Sublayer& sublayer = layer.sublayers[rn.node.sl];
    std::vector <std::vector<GridNode> >& grid_nodes = sublayer.grid_nodes;

    //set all node between node.i and i_cur to tar_upp/low
    unsigned int low_i = (rn.i_cur < rn.node.i) ? rn.i_cur : rn.node.i;
    unsigned int upp_i = (rn.i_cur > rn.node.i) ? rn.i_cur : rn.node.i;
    /*for( unsigned int n = low_i; n <= upp_i; n ++) {
      if (low_i == upp_i) break;
      grid_nodes[rn.t_cur.front()][n].set_bit( GridNode::tar_upp, 1 );
      grid_nodes[rn.t_cur.back()][n].set_bit( GridNode::tar_low, 1 );
    }*/
    grid_nodes[rn.t_cur.front()][rn.node.i].set_bit( GridNode::tar_upp, 1 );
    grid_nodes[rn.t_cur.back()][rn.node.i].set_bit( GridNode::tar_low, 1 );

    steiner_tar.nodes.push_back(rn);
    // add all pinshape to pinout_target
    for ( unsigned char n = 0; n < rn.t_cur.size(); n ++) {
      Rectangle rec;
      rec.l = rn.node.l;
      rec.sl = rn.node.sl;
      rec.lower.coor[layer.direction] = sublayer.sltra_coor[rn.t_cur[n]] - bw[rec.l][rec.sl] / 2;
      rec.upper.coor[layer.direction] = rec.lower.coor[!layer.direction] + bw[rec.l][rec.sl];
      rec.lower.coor[!layer.direction] = layer.lint_coor[rn.node.i] - bw[rec.l][rec.sl] / 2;
      rec.upper.coor[!layer.direction] = rec.lower.coor[layer.direction] + bw[rec.l][rec.sl];
      steiner_tar.pin_shapes.push_back(rec);
    }
    steiner_tars.push_back(steiner_tar);
    steiner_tar.nodes[0].heading.cur = !steiner_tar.nodes[0].heading.cur;
    steiner_tars.push_back(steiner_tar);

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
  return 1;
}

void Router::generate_path( BusRoute& route, const Pinout& source,
  const Pinout& target, const std::vector< std::vector<unsigned int> >& bw )
{
  const unsigned int nbits = source.pin_shapes.size();
  for ( unsigned int p = 0; p < route.paths.size(); ++p ) {
    Path& path = route.paths[p];
    const Layer& layer = grid.layers[path.l];
    const Sublayer& sublayer = layer.sublayers[path.sl];
    const bool& dir = layer.direction;
    const unsigned int half_width = bw[path.l][path.sl] >> 1;
    path.overlap = 0;
    // Convert track indices to coordinates
    path.t_coor.resize(nbits);
    for ( unsigned int n = 0; n < nbits; ++n ) {
      path.t_coor[n] = sublayer.sltra_coor[path.t[n]];
    }
    // Find intersection coordinates
    path.i_coor.resize(nbits);
    if ( p != 0 ) {
      const Path& path_pre = route.paths[p-1];
      const Layer& layer_pre = grid.layers[path_pre.l];
      const Sublayer& sublayer_pre = layer_pre.sublayers[path_pre.sl];
      for ( unsigned int n = 0; n < nbits; ++n ) {
        unsigned int tp = n;
        if ( path.bit_order != path_pre.bit_order ) tp = nbits-1-n;
        unsigned int& i_start = path.heading ? path.i_coor[n].low : path.i_coor[n].upp;
        if ( path.l > path_pre.l )
          i_start = sublayer_pre.sltra_ulint[path_pre.t[tp]];
        else
          i_start = sublayer_pre.sltra_llint[path_pre.t[tp]];
        i_start = layer.lint_coor[i_start];
      }
    } else {
      for ( unsigned int n = 0; n < nbits; ++n ) {
        unsigned int& i_end = path.heading ? path.i_coor[n].low : path.i_coor[n].upp;
        if ( path.heading )
          i_end = source.pin_shapes[0].upper.coor[!dir];
        else
          i_end = source.pin_shapes[0].lower.coor[!dir];
      }
    }
    if ( p != route.paths.size()-1 ) {
      const Path& path_next = route.paths[p+1];
      const Layer& layer_next = grid.layers[path_next.l];
      const Sublayer& sublayer_next = layer_next.sublayers[path_next.sl];
      for ( unsigned int n = 0; n < nbits; ++n ) {
        unsigned int tn = n;
        if ( path.bit_order != path_next.bit_order ) tn = nbits-1-n;
        unsigned int& i_end = path.heading ? path.i_coor[n].upp : path.i_coor[n].low;
        if ( path.l > path_next.l ){
          i_end = sublayer_next.sltra_ulint[path_next.t[tn]];
        }
        else{
          i_end = sublayer_next.sltra_llint[path_next.t[tn]];
        }
        i_end = layer.lint_coor[i_end];
      }
    } else {
      for ( unsigned int n = 0; n < nbits; ++n ) {
        unsigned int& i_end = path.heading ? path.i_coor[n].upp : path.i_coor[n].low;
        if ( path.heading )
          i_end = target.pin_shapes[0].lower.coor[!dir];
        else
          i_end = target.pin_shapes[0].upper.coor[!dir];
      }
    }
  }
  //route.path_cur_index = route.paths.size();
}

void Router::path2wire( Bus& bus, const std::vector< std::vector<unsigned int> >& bw )
{
  const unsigned int nbits = bus.bits.size();
  for ( BusRoute& route : bus.routes) {
    for ( Path& path : route.paths ){
      const unsigned int half_width = bw[path.l][path.sl] >> 1;
      const Layer& layer = grid.layers[path.l];
      const Sublayer& sublayer = layer.sublayers[path.sl];
      const bool& dir = layer.direction;
      for ( unsigned int n = 0; n < nbits; ++n ) {
        Rectangle wire;
        // std::cout << grid.layers.size()*3 + (unsigned int)path.l << " ";
        wire.l = path.l;
        wire.sl = path.sl;
        if ( layer.direction ) {
          // std::cout << path.i_coor[n].low << " ";
          // std::cout << path.t_coor[n] - half_width << " ";
          // std::cout << path.i_coor[n].upp << " ";
          // std::cout << path.t_coor[n] + half_width << " ";
          wire.lower.coor[0] = path.i_coor[n].low;
          wire.lower.coor[1] = path.t_coor[n] - half_width;
          wire.upper.coor[0] = path.i_coor[n].upp;
          wire.upper.coor[1] = path.t_coor[n] + half_width;
        } else {
          // std::cout << path.t_coor[n] - half_width << " ";
          // std::cout << path.i_coor[n].low << " ";
          // std::cout << path.t_coor[n] + half_width << " ";
          // std::cout << path.i_coor[n].upp << " ";
          wire.lower.coor[0] = path.t_coor[n] - half_width;
          wire.lower.coor[1] = path.i_coor[n].low;
          wire.upper.coor[0] = path.t_coor[n] + half_width;
          wire.upper.coor[1] = path.i_coor[n].upp;
        }
        //std::cout << wire.upper.coor[0] << " " << wire.upper.coor[1] << std::endl;
        // std::cout << std::endl;
        route.wires.push_back(wire);
      }
    }
  }
}

void Router::reduce_overlap_path( Bus &bus )
{
  for (unsigned int n = 1; n < bus.routes.size(); n ++) {
    Path& path_tail = bus.routes[n].paths.back();
    // find overlap path in main branch
    for ( unsigned int m = 0; m < bus.main_branchs.size(); m ++ ) {
      Path& overlap_path = bus.main_branchs[m];
      if ( path_tail.l == overlap_path.l && path_tail.sl == overlap_path.sl &&
        path_tail.t == overlap_path.t) {
        // update overlap path in main path
        bool modify = 0;
        path_tail.overlap = 1;
        for ( unsigned int c = 0; c < bus.bits.size(); c ++ ) {
          Range& overlap_i_coor = overlap_path.i_coor[c];
          unsigned int& path_tail_min = (path_tail.i_coor[c].low < path_tail.i_coor[c].upp)
            ? path_tail.i_coor[c].low : path_tail.i_coor[c].upp;
          unsigned int& path_tail_max = (path_tail.i_coor[c].low > path_tail.i_coor[c].upp)
            ? path_tail.i_coor[c].low : path_tail.i_coor[c].upp;
          unsigned int& overlap_path_min = (overlap_path.i_coor[c].low < overlap_path.i_coor[c].upp)
            ? overlap_path.i_coor[c].low : overlap_path.i_coor[c].upp;
          unsigned int& overlap_path_max = (overlap_path.i_coor[c].low > overlap_path.i_coor[c].upp)
            ? overlap_path.i_coor[c].low : overlap_path.i_coor[c].upp;

          if ( overlap_path_min > path_tail_min ){
            overlap_path_min = path_tail_min;
            std::cout << "overlap_low\n";
            modify = 1;
          }
          if ( overlap_path_max < path_tail_max ){
            overlap_path_max = path_tail_max;
            std::cout << "overlap_upp\n";
            modify = 1;
          }
        }
        if ( modify ) {

          std::cout << "overlap_pre\n";
          continue;
        }
      }
    }
    if (path_tail.overlap)
      break;
    for (unsigned int m = 0; m < n; m ++) {
      for ( unsigned int p = 0; p < bus.routes[m].paths.size(); p ++) {
        Path& overlap_path = bus.routes[m].paths[p];
        if ( path_tail.l == overlap_path.l && path_tail.sl == overlap_path.sl &&
          path_tail.t == overlap_path.t) {
          // update overlap path in main path
          bool modify = 0;
          Path branch;
          branch = overlap_path;
          path_tail.overlap = 1;
          for ( unsigned int c = 0; c < bus.bits.size(); c ++ ) {
            Range& overlap_i_coor = overlap_path.i_coor[c];

            unsigned int& path_tail_min = (path_tail.i_coor[c].low < path_tail.i_coor[c].upp)
              ? path_tail.i_coor[c].low : path_tail.i_coor[c].upp;
            unsigned int& path_tail_max = (path_tail.i_coor[c].low > path_tail.i_coor[c].upp)
              ? path_tail.i_coor[c].low : path_tail.i_coor[c].upp;
            unsigned int& branch_min = (branch.i_coor[c].low < branch.i_coor[c].upp)
              ? branch.i_coor[c].low : branch.i_coor[c].upp;
            unsigned int& branch_max = (branch.i_coor[c].low > branch.i_coor[c].upp)
              ? branch.i_coor[c].low : branch.i_coor[c].upp;

            if ( branch_min > path_tail_min ){
              branch_min = path_tail_min;
              //std::cout << "overlap_low\n";
              modify = 1;
            }
            if ( branch_max < path_tail_max ){
              branch_max = path_tail_max;
              //std::cout << "overlap_upp\n";
              modify = 1;
            }
          }
          if ( modify ) {
            overlap_path.overlap = 1;
            bus.main_branchs.push_back(branch);
            std::cout << "overlap\n";
            break;
          }
        }
      }
      if (path_tail.overlap)
        break;
    }
  }
}

void Router::output_route( const Bus& bus )
{
  const unsigned int& nbits = bus.bits.size();
  output << "BUS " << bus.name << "\n";
  for ( unsigned int n = 0; n < nbits; ++n ) {
    output << "BIT " << bus.bits[n].name << "\n";
    std::string path_string;
    unsigned int path_count = 0;
    for ( unsigned int r = 0; r < bus.routes.size(); r ++) {
      BusRoute route = bus.routes[r];
      for ( unsigned int p = 0; p < route.paths.size(); ++p ) {
        const Path& path = route.paths[p];
        const bool& dir = grid.layers[path.l].direction;
        const bool& bit_order = path.bit_order;
        const bool& heading = path.heading;
        const unsigned int& t_coor = path.t_coor[bit_order ? n : (nbits-n-1)];
        const Range& i_coor = path.i_coor[bit_order ? n : (nbits-n-1)];
        const unsigned int& i_start = heading ? i_coor.low : i_coor.upp;
        const unsigned int& i_end = heading ? i_coor.upp : i_coor.low;
        const unsigned int x_start = dir ? i_start : t_coor;
        const unsigned int y_start = dir ? t_coor : i_start;
        const unsigned int x_end = dir ? i_end : t_coor;
        const unsigned int y_end = dir ? t_coor : i_end;
        const unsigned int x_low = dir ? i_coor.low : t_coor;
        const unsigned int y_low = dir ? t_coor : i_coor.low;
        const unsigned int x_upp = dir ? i_coor.upp : t_coor;
        const unsigned int y_upp = dir ? t_coor : i_coor.upp;
        if ( p == 0 ) {
          const unsigned int from_l = std::min(path.l, route.l_src);
          const unsigned int to_l = std::max(path.l, route.l_src);
          const unsigned int from_sl = ( path.l < route.l_src ) ? path.sl : route.sl_src;
          const unsigned int to_sl = ( path.l < route.l_src ) ? route.sl_src : path.sl;
          const unsigned int pin_side_upp = bus.pinouts[r == 0 ? r : r + 1].pin_shapes[path.bit_order ? n : (nbits - 1 - n)].upper.coor[dir];
          const unsigned int pin_side_low = bus.pinouts[r == 0 ? r : r + 1].pin_shapes[path.bit_order ? n : (nbits - 1 - n)].lower.coor[dir];
          unsigned int via_point = dir ? y_start : x_start;
          via_point += ( bus.bus_widths[to_l][to_sl] / 2 );
          while ( !(via_point <= pin_side_upp && via_point >= pin_side_low) )
            via_point--;
          const unsigned int via_x = dir ? x_start : via_point;
          const unsigned int via_y = dir ? via_point : y_start;
          if ( from_l == to_l ) {
            for ( unsigned int sl = from_sl; sl < to_sl; ++sl ) {
              const std::string& layer_name = grid.layers[to_l].sublayers[sl].name;
              path_string += layer_name + " ";
              path_string += coor_string(via_x, via_y);
              path_string += "\n";
              path_count++;
            }
          } else {
            for ( unsigned int sl = from_sl; sl < grid.layers[from_l].sublayers.size();
              ++sl ) {
              const std::string& layer_name = grid.layers[from_l].sublayers[sl].name;
              path_string += layer_name + " ";
              path_string += coor_string(via_x, via_y);
              path_string += "\n";
              path_count++;
            }
            for ( unsigned int sl = 0; sl < to_sl; ++sl ) {
              const std::string& layer_name = grid.layers[to_l].sublayers[sl].name;
              path_string += layer_name + " ";
              path_string += coor_string(via_x, via_y);
              path_string += "\n";
              path_count++;
            }
          }
        } else {
          const Path& path_pre = route.paths[p-1];
          const unsigned int from_l = std::min(path.l, path_pre.l);
          const unsigned int to_l = std::max(path.l, path_pre.l);
          const unsigned int from_sl = ( path.l < path_pre.l ) ? path.sl : path_pre.sl;
          const unsigned int to_sl = ( path.l < path_pre.l ) ? path_pre.sl : path.sl;
          if ( from_l == to_l ) {
            for ( unsigned int sl = from_sl; sl < to_sl; ++sl ) {
              const std::string& layer_name = grid.layers[to_l].sublayers[sl].name;
              path_string += layer_name + " ";
              path_string += coor_string(x_start, y_start);
              path_string += "\n";
              path_count++;
            }
          } else {
            for ( unsigned int sl = from_sl; sl < grid.layers[from_l].sublayers.size();
              ++sl ) {
              const std::string& layer_name = grid.layers[from_l].sublayers[sl].name;
              path_string += layer_name + " ";
              path_string += coor_string(x_start, y_start);
              path_string += "\n";
              path_count++;
            }
            for ( unsigned int sl = 0; sl < to_sl; ++sl ) {
              const std::string& layer_name = grid.layers[to_l].sublayers[sl].name;
              path_string += layer_name + " ";
              path_string += coor_string(x_start, y_start);
              path_string += "\n";
              path_count++;
            }
          }
        }
        if ( !path.overlap ) {
          const std::string& layer_name = grid.layers[path.l].sublayers[path.sl].name;
          path_string += layer_name + " ";
          path_string += coor_string(x_low, y_low);
          path_string += " ";
          path_string += coor_string(x_upp, y_upp);
          path_string += "\n";
          path_count++;
        }

        if ( p == route.paths.size()-1 ) {
          const unsigned int from_l = std::min(path.l, route.l_tar);
          const unsigned int to_l = std::max(path.l, route.l_tar);
          const unsigned int from_sl = ( path.l < route.l_tar ) ? path.sl : route.sl_tar;
          const unsigned int to_sl = ( path.l < route.l_tar ) ? route.sl_tar : path.sl;
          unsigned int via_x = x_end;
          unsigned int via_y = y_end;
          if( r == 0){
            const unsigned int pin_side_upp = bus.pinouts[1].pin_shapes[path.bit_order ? n : nbits - 1 - n].upper.coor[dir];
            const unsigned int pin_side_low = bus.pinouts[1].pin_shapes[path.bit_order ? n : nbits - 1 - n].lower.coor[dir];
            unsigned int via_point = dir ? y_end : x_end;
            via_point += ( bus.bus_widths[to_l][to_sl] / 2 );
            while (!(via_point <= pin_side_upp && via_point >= pin_side_low))
              via_point--;
            via_x = dir ? x_end : via_point;
            via_y = dir ? via_point : y_end;
          }
          if ( from_l == to_l ) {
            for ( unsigned int sl = from_sl; sl < to_sl; ++sl ) {
              const std::string& layer_name = grid.layers[to_l].sublayers[sl].name;
              path_string += layer_name + " ";
              path_string += coor_string(via_x, via_y);
              path_string += "\n";
              path_count++;
            }
          } else {
            for ( unsigned int sl = from_sl; sl < grid.layers[from_l].sublayers.size();
              ++sl ) {
              const std::string& layer_name = grid.layers[from_l].sublayers[sl].name;
              path_string += layer_name + " ";
              path_string += coor_string(via_x, via_y);
              path_string += "\n";
              path_count++;
            }
            for ( unsigned int sl = 0; sl < to_sl; ++sl ) {
              const std::string& layer_name = grid.layers[to_l].sublayers[sl].name;
              path_string += layer_name + " ";
              path_string += coor_string(via_x, via_y);
              path_string += "\n";
              path_count++;
            }
          }
        }
      }
    }
    for (unsigned char p = 0; p < bus.main_branchs.size(); p ++) {
      const Path& path = bus.main_branchs[p];
      const std::string& layer_name = grid.layers[path.l].sublayers[path.sl].name;
      const bool& dir = grid.layers[path.l].direction;
      const bool& bit_order = path.bit_order;
      const unsigned int& t_coor = path.t_coor[bit_order ? n : (nbits-n-1)];
      const Range& i_coor = path.i_coor[bit_order ? n : (nbits-n-1)];
      const unsigned int x_low = dir ? i_coor.low : t_coor;
      const unsigned int y_low = dir ? t_coor : i_coor.low;
      const unsigned int x_upp = dir ? i_coor.upp : t_coor;
      const unsigned int y_upp = dir ? t_coor : i_coor.upp;
      path_string += layer_name + " ";
      path_string += coor_string(x_low, y_low);
      path_string += " ";
      path_string += coor_string(x_upp, y_upp);
      path_string += "\n";
      path_count++;
    }
    output << "PATH " << path_count << "\n";
    output << path_string;
    output << "ENDPATH\n";
    output << "ENDBIT\n";
  }
  output << "ENDBUS\n";
}

std::string Router::coor_string( const unsigned int& x, const unsigned int& y )
{
  return "(" + std::to_string(x) + " " + std::to_string(y) + ")";
}
