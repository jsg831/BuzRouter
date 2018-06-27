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
  unsigned char l, sl;
  void parse( std::string filename, Router& router);
private:
  std::ifstream input_file;
  std::stringstream ss;
  std::map< std::string, std::pair<unsigned int, unsigned int> > layer_table;
  std::vector <std::vector<unsigned int> > layer_width;
  unsigned int convert( const std::string& str );
};

#endif
