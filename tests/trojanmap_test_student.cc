#include "src/lib/trojanmap.h"

#include <map>
#include <vector>

#include "gtest/gtest.h"

TEST(TrojanMapTest, getposition) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test erroneous
  auto position = m.GetPosition("basdfa");
  std::pair<double, double> gt1(-1, -1);  
  EXPECT_EQ(position, gt1);
  // Test Coffee Bean1
  position = m.GetPosition("Coffee Bean1");
  std::pair<double, double> gt2(34.0172407,-118.2824946);
  EXPECT_EQ(position, gt2);
}

TEST(TrojanMapTest, shortestpath) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();

  //test wrong input
  auto path = m.CalculateShortestPath("asdf", "ahgfd");
  EXPECT_EQ(path.empty(), true);
  //test wrong input
  path = m.CalculateShortestPath("Ralphs", "ahgfd");
  EXPECT_EQ(path.empty(), true);
  //test wrong input
  path = m.CalculateShortestPath("asdf", "Coffee Bean1");
  EXPECT_EQ(path.empty(), true);
}


TEST(TrojanMapTest, autocomplete) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();

  //test wrong input
   std::vector<std::string> word = m.Autocomplete("ralp");
   std::vector<std::string> w = {"Ralphs"};
  EXPECT_EQ(w, word);

   std::vector<std::string> word2 = m.Autocomplete("targ");
   std::vector<std::string> w2 = {"Target"};
  EXPECT_EQ(w2, word2);
}

TEST(TrojanMapTest, TSP_Test_Brute_Force) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1873056015", "213332060", "1931345270"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1873056015", "213332060", "1931345270", "1873056015"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP_Test2_Brute_Force) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1873056015", "1931345270"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1873056015", "1931345270", "1873056015"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}