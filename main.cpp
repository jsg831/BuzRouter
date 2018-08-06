#include <iostream>
#include <string>
#include "parser.h"
#include "router.h"

int main( int argc, char const *argv[] )
{
  if ( argc != 3 ) return 0;
  std::string input_filename = argv[1];
  std::string output_filename = argv[2];

  Router router;
  Parser parser;

  parser.parse( input_filename, router );
  parser.pinshapes_check( router );
  router.initialize( output_filename );
  router.route_all();

  return 0;
}
