#include "src/lib/trojanmap.h"

#include <map>
#include <vector>

#include "gtest/gtest.h"
int ALGO_TYPE;
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
    // Test The Row House
  position = m.GetPosition("The Row House");
  std::pair<double, double> gt3(34.0263672,-118.2785935);
  EXPECT_EQ(position, gt3);
    // Test blank
  position = m.GetPosition("");
  std::pair<double, double> gt4(-1,-1);
  EXPECT_EQ(position, gt4);
   // Test space
  position = m.GetPosition(" ");
  std::pair<double, double> gt5(-1,-1);
  EXPECT_EQ(position, gt5);
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
  path = m.CalculateShortestPath("Ralphs", "The Row House");
  std::vector<std::string> expected = {"2578244375",
"5559640911",
"6787470571",
"6808093910",
"6808093913",
"6808093919",
"6816831441",
"6813405269",
"6816193784",
"6389467806",
"6816193783",
"123178876",
"6987230635",
"6987230634",
"6813405267",
"6807243572",
"2613156405",
"2613117906",
"123178871",
"6813405266",
"6813416159",
"122814447",
"6813416123",
"6813416171",
"6807536647",
"6807320427",
"6807536642",
"6813416166",
"7882624618",
"7200139036",
"122814440",
"6813416163",
"7477947679",
"7298150111",
"6787803640",
"6807554573",
"4536993726"};
EXPECT_EQ(path, expected);
}


TEST(TrojanMapTest, autocomplete) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();

  //test specific
  std::vector<std::string> word = m.Autocomplete("ralp");
  std::vector<std::string> w = {"Ralphs"};
  EXPECT_EQ(w, word);

  std::vector<std::string> word2 = m.Autocomplete("targ");
  std::vector<std::string> w2 = {"Target"};
  EXPECT_EQ(w2, word2);
  //test blank
  std::vector<std::string> word3 = m.Autocomplete("");
  EXPECT_EQ(true, word3.empty());
  //test space
  std::vector<std::string> word4 = m.Autocomplete(" ");
  EXPECT_EQ(true, word4.empty());
  //test space
  std::vector<std::string> word5 = m.Autocomplete("098");
  EXPECT_EQ(true, word5.empty());
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

TEST(TrojanMapTest, TSP_Test3_Brute_Force) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input; // Input location ids 
  auto result = m.TravellingTrojan(input);
  EXPECT_EQ(true, result.second.empty());
  EXPECT_EQ(0, result.first);
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