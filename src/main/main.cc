#include <iostream>
#include "src/lib/trojanmap.h"

int main() {

  TrojanMap x;
  x.CreateGraphFromCSVFile();
  std::cout << x.GetName("122659207");
  // x.PrintMenu();
  
  return 0;
}