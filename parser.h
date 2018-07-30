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
using namespace std;
class Parser
{
public:
  void parse( std::string filename, Router& router);
  void pinshapes_check( Router& router );
private:
  std::ifstream input_file;
  std::stringstream ss;
  std::map< std::string, std::pair<uint32_t, uint32_t> > layer_table;
  std::vector <std::vector<uint32_t> > layer_width;
  uint32_t convert( const std::string& str );
  uint8_t l = 0, sl = 0;
};

#endif
