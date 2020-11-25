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

