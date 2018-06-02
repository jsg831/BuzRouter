#ifndef GDT_H
#define GDT_H
#include "gdt2gds.h"
#include "gds2gdt.h"
#include <string>
#include <fstream>
#include "router.h"
#include <iostream>
using namespace std;

class GDT{

	fstream trp;
	FILE *gdt;
public:
	GDT(string fname_in, string fname_out);
	GDT(string fname_in);
	void layout2gdt(vector<vector< Track >> layout1);
	string Filename_extension(string file);
	string GetFileName(string file);
	void layout_append(Rectangle input, vector <pair <uint32_t, uint32_t>> info);
	void end_append();
};
#endif
