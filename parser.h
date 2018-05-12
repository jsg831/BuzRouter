#ifndef _PARSER_H
#define _PARSER_H

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>
#include "router.h"

class Parser
{
public:
  void parse( std::string filename, Router& router );
private:
  std::ifstream input_file;
  std::stringstream ss;
  std::map< std::string, std::pair<uint32_t, uint32_t> > layer_table;

  uint32_t convert( const std::string& str );
};

#endif
