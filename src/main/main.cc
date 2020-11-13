#include <iostream>
#include "src/lib/trojanmap.h"

int main() {

  TrojanMap x;
  x.CreateGraphFromCSVFile();
  x.MapNames();
  // std::pair<double,double> val = x.GetPosition("University Park");
  // std::cout << val.first << " " << val.second << std::endl;
  // std::cout << x.GetLat("7360410739") << std::endl;
  // std::cout << x.GetLon("7360410739") << std::endl;
  // std::cout << x.GetName("7360410739") << std::endl;
  std::vector<std::string> val = {"7360410739", "122719216"};
  // x.GetNeighborIDs("7360410739"); 
  // for (auto x : val) std::cout << x << std::endl;
  std::cout << x.CalculatePathLength(val) << std::endl;
  
  x.PrintMenu();
  
  return 0;
}