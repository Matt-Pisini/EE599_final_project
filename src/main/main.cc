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
  // std::vector<std::string> val = {"7360410739", "122719216"};
  // x.GetNeighborIDs("7360410739"); 
  // for (auto x : val) std::cout << x << std::endl;
  // std::cout << x.CalculatePathLength(val) << std::endl;
  // std::vector<std::string> val = x.GetNeighborIDs("5556117120");
  // for(auto x : val) std::cout << x << std::endl;
  // std::vector<std::string> val = {"2578244375", "5559640911", "6787470571", "6808093910", "6808093913", "6808093919", "6816831441", "6813405269", "6816193784", "6389467806", "6816193783", "123178876", "2613117895", "122719259", "2613117861", "6817230316", "3642819026", "6817230310", "7811699597", "5565967545", "123318572", "6813405206", "6813379482", "544672028", "21306059", "6813379476", "6818390140", "63068610", "6818390143", "7434941012", "4015423966", "5690152766", "6813379440", "6813379466", "21306060", "6813379469", "6813379427", "123005255", "6807200376", "6807200380", "6813379451", "6813379463", "123327639", "6813379460", "4141790922", "4015423963", "1286136447", "1286136422", "4015423962", "6813379494", "63068643", "6813379496", "123241977", "4399914059", "4015372478", "1732243576", "6813379548", "4015372476", "4015372475", "1732243620", "4015372469", "4015372463", "6819179749", "1732243544", "6813405275", "348121996", "348121864", "6813405280", "1472141024", "4015372462", "6813411586", "4015372458", "6813411588", "1837212101", "6820935911", "4547476733"};
  // x.CalculateShortestPath("ChickfilA","Ralphs");
  // auto position = x.GetPosition("ChickfilA");
  // std::cout << position.first << " " << position.second << std::endl;
  // x.PlotPath(val);
  std::vector<std::string> input = {"4015477530"};
  auto result = x.TravellingTrojan(input);
  std::cout << result.second.size() << std::endl;

  x.PrintMenu();
  
  return 0;
}