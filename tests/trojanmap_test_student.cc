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


TEST(TrojanMapTest,TwoOpt) {
  ALGO_TYPE = 2; //set algo type to 2opt
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1862312636", "7424270441", "67666219", "4015405548", "4015203110", "6807439002"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1862312636", "7424270441", "67666219", "4015405548", "4015203110", "6807439002","1862312636"}; // benchmark, 2 opt should do better than this order, which is the original order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt >= result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt >= result.second[result.second.size()-1]) // counterclockwise //we need results.second to be less that GT for both conditions
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest,ThreeOpt) {
  ALGO_TYPE = 3; //set algo type to 2opt
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1862312636", "7424270441", "67666219", "4015405548", "4015203110", "6807439002"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1862312636", "7424270441", "67666219", "4015405548", "4015203110", "6807439002","1862312636"}; // benchmark, 3 opt should do better than this order, which is the original order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt >= result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt >= result.second[result.second.size()-1]) // counterclockwise //we need results.second to be less that GT for both conditions
    flag = true;
  
  EXPECT_EQ(flag, true);
}


TEST(TrojanMapTest,TwoOpt2) {
  // in this test the path given is the optimal path, should be less than or equal to it
  ALGO_TYPE = 2; //set algo type to 2opt
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"4015372458", "6813411588", "1837212101", "6820935911", "4547476733"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4015372458", "6813411588", "1837212101", "6820935911", "4547476733","4015372458"}; // benchmark, 2 opt should do equal or better than this order, which is the original order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt >= result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt >= result.second[result.second.size()-1]) // counterclockwise //we need results.second to be less that GT for both conditions
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest,ThreeOpt2) {
  // in this test the path given is the optimal path, should be less than or equal to it
  ALGO_TYPE = 3; //set algo type to 2opt
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"4015372458", "6813411588", "1837212101", "6820935911", "4547476733"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4015372458", "6813411588", "1837212101", "6820935911", "4547476733","4015372458"}; // benchmark, 2 opt should do better than this order, which is the original order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt >= result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt >= result.second[result.second.size()-1]) // counterclockwise //we need results.second to be less that GT for both conditions
    flag = true;
  
  EXPECT_EQ(flag, true);
}