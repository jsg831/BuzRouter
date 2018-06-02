#include "GDT.h"

GDT::GDT(string fname_in, string fname_out){
  trp.open(fname_in.c_str(), ios_base::in);
  gdt = fopen(fname_out.c_str(), "w");
}

GDT::GDT(string fname_in){
  fpos_t pos, insert_p;
  char data;
  gdt = fopen(fname_in.c_str(), "r+");
  while(!feof(gdt)){
  	fgetc(gdt);
  }
	fgetpos (gdt, &pos);
  insert_p.__pos = pos.__pos - 4;
  fclose(gdt);
  gdt = fopen(fname_in.c_str(), "r+");
  fgetpos (gdt, &pos);
  while(!feof(gdt)){
    fgetc(gdt);
		fgetpos (gdt, &pos);
    if(pos.__pos >= insert_p.__pos){
    	break;
		}
  }
}

void GDT::layout_append(Rectangle input, vector < pair<uint32_t, uint32_t> > info){
  vector <Point> points;
  points.resize(4);
  points[0] = input.lower;
  points[2] = input.upper;
  points[1].coor[0] = input.lower.coor[0];
  points[1].coor[1] = input.upper.coor[1];
  points[3].coor[0] = input.upper.coor[0];
  points[3].coor[1] = input.lower.coor[1];
  fprintf(gdt, "b{%d xy(", input.l);
  for (auto j = 0; j < points.size(); j++){
    fprintf(gdt, "%.3f %.3f", static_cast<double>(points[j].coor[0])/1000,
      static_cast<double>(points[j].coor[1])/1000);
    if (j != points.size() - 1){
      fprintf(gdt, " ");
    }
  }
  fprintf(gdt, ")");
  for(int j = 0; j < info.size(); j ++){
    if(j == 0)
      fprintf(gdt, " ");
    fprintf(gdt, "pr{%d \'%d\'}", info[j].first, info[j].second);
    if (j != info.size() - 1){
      fprintf(gdt, " ");
    }
  }
  fprintf(gdt, "}\n");
}
void GDT::end_append(){
  fprintf(gdt, "}\n}\n");
  fclose(gdt);
}

void GDT::layout2gdt(vector<vector< Track >> layout1){
  string line;
  fprintf(gdt, "gds2{600\nm = 2014 - 02 - 19 23:14 : 30 a = 2014 - 02 - 19 23 : 14 : 30\nlib 'Flat' 1 1e-09\n");
  fprintf(gdt, "cell{ c = 2014 - 02 - 19 23:14 : 30 m = 2014 - 02 - 19 23 : 14 : 30 'trp'\n");
  for (int i = 0; i < layout1.size(); i++){
    for (int k = 0; k < layout1[i].size(); k++){
      fprintf(gdt, "b{%d xy(", i + 1);
      Rectangle &rec = layout1[i][k].line;
      vector <Point> points;
      points.resize(4);
      if(rec.lower.coor[0] == rec.upper.coor[0]){	//vertical
        points[0].coor[0] = rec.lower.coor[0] - layout1[i][k].width/2;
        points[0].coor[1] = rec.lower.coor[1];
        points[1].coor[0] = rec.lower.coor[0] + layout1[i][k].width/2;
        points[1].coor[1] = rec.lower.coor[1];
        points[2].coor[0] = rec.upper.coor[0] + layout1[i][k].width/2;
        points[2].coor[1] = rec.upper.coor[1];
        points[3].coor[0] = rec.upper.coor[0] - layout1[i][k].width/2;
        points[3].coor[1] = rec.upper.coor[1];
      }	else if(rec.lower.coor[1] == rec.upper.coor[1]){	//horizontal
          points[0].coor[0] = rec.lower.coor[0];
          points[0].coor[1] = rec.lower.coor[1] - layout1[i][k].width/2;
          points[1].coor[0] = rec.lower.coor[0];
          points[1].coor[1] = rec.lower.coor[1] + layout1[i][k].width/2;
          points[2].coor[0] = rec.upper.coor[0];
          points[2].coor[1] = rec.upper.coor[1] + layout1[i][k].width/2;
          points[3].coor[0] = rec.upper.coor[0];
          points[3].coor[1] = rec.upper.coor[1] - layout1[i][k].width/2;
        }	else{
          points[0] = rec.lower;
          points[2] = rec.upper;
          points[1].coor[0] = rec.lower.coor[0];
          points[1].coor[1] = rec.upper.coor[1];
          points[3].coor[0] = rec.upper.coor[0];
          points[3].coor[1] = rec.lower.coor[1];
          }
      for (int j = 0; j < points.size(); j++){
        fprintf(gdt, "%d %d", static_cast<uint32_t>(points[j].coor[0]), static_cast<uint32_t>(points[j].coor[1]));
        if (j != points.size() - 1){
          fprintf(gdt, " ");
        }
      }
      fprintf(gdt, ")}\n");
    }
  }
  fprintf(gdt, "}\n}");
  fclose(gdt);
}

string GDT::Filename_extension(string file){

  int dotpos = file.find_last_of(".");
  string fname, fextension;

  for (int index = 0; index < (int)file.length(); index++)
  {
    if (index < dotpos)
      fname.push_back(file.at(index));

    else if (index > dotpos)
      fextension.push_back(file.at(index));
  }
  return fextension;
}

string GetFileName(string file){
  int dotpos = file.find_last_of(".");
  return file.substr(0, dotpos);
}
