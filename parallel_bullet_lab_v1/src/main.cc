#include "ParallelCore.hh"
#include <iostream>
using namespace parallel_lab;
int main(int argc,char**argv){
 try{
  if(argc<3){std::cerr<<"Usage: parallel_core Delta|Stewart OUTPUT.csv [--no-drive]\n";return 2;}
  Core core(argv[1]);if(argc>3 && std::string(argv[3])=="--no-drive")core.drive=false;
  std::ofstream out(argv[2]);if(!out)throw std::runtime_error("Cannot open CSV");core.header(out);core.record(out);
  const int steps=int((core.drive?core.duration():2.)*1000);double maximum=0;
  for(int i=0;i<steps;i++){core.step(.001);maximum=std::max(maximum,core.closure());if(i%20==19)core.record(out);}
  std::cout<<core.robot<<": "<<core.time<<" simulated seconds; max anchor gap "<<maximum<<" m\n";
  if(maximum>.01)throw std::runtime_error("Anchor gap exceeded 10 mm investigation gate");
 }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<'\n';return 1;}return 0;
}
