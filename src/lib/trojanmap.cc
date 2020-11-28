#include "trojanmap.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <locale>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <set>
#include <cctype>
#include <string>
#include <stack>
#include <chrono>
#include <cstdlib>      // std::rand, std::srand

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

/********************* GLOBALS *********************/

//Set parameters for CalculateShortestPath algorithm
// #define A_ALGORITHM 0
// #define DJIKSTRAS_ALGORITHM 1
int SHORTEST_PATH_ALGO = 0;
// set algo type 1 = Brute Force 
// 2 = 2-Opt
// 3 = 3-Opt
#define ALGO_TYPE 1
//-----------------------------------------------------
// TODO (Students): You do not and should not change the following functions:
//-----------------------------------------------------

/**
 * PrintMenu: Create the menu
 * 
 */
void TrojanMap::PrintMenu() {

  std::string menu =
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the position                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Travelling salesman problem                              \n"
      "* 5. Exit                                                     \n"
      "**************************************************************\n";
  std::cout << menu << std::endl;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = Autocomplete(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the position                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = GetPosition(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1) {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                                            "
        "      \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    auto results = CalculateShortestPath(input1, input2);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
      PlotPath(results);
    } else {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Travelling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    auto results = TravellingTrojan(locations);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    CreateAnimation(results.second);
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '5':
    break;
  default:
    std::cout << "Please select 1 - 5." << std::endl;
    PrintMenu();
    break;
  }
}


/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  std::fstream fin;
  fin.open("src/lib/map.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '['), word.end());
      word.erase(std::remove(word.begin(), word.end(), ']'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void TrojanMap::PlotPoint(std::string id) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(data[id].lat, data[id].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void TrojanMap::PlotPoint(double lat, double lon) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void TrojanMap::PlotPath(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPoints(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}


/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void TrojanMap::CreateAnimation(std::vector<std::vector<std::string>> path_progress){
  cv::VideoWriter video("src/lib/output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(1248,992));
  for(auto location_ids: path_progress) {
    std::string image_path = cv::samples::findFile("src/lib/input.jpg");
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
              cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < location_ids.size(); i++) {
      auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
    video.write(img);
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
	video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon) {
  std::pair<double, double> bottomLeft(34.01001, -118.30000);
  std::pair<double, double> upperRight(34.03302, -118.26502);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1248,
                                   (1 - (lat - bottomLeft.first) / h) * 992);
  return result;
}

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------

/**
 * MapNames: Creates a mapping from name to ID
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
void TrojanMap::MapNames()
{
  for(auto item : data)
  {
    if (item.second.name != "")
    {
      name_to_id[item.second.name] = item.second.id;
    }
  }
}
/**
 * GetLat: Get the latitude of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(std::string id) { return data[id].lat; }

/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id) { return data[id].lon; }

/**
 * GetName: Get the name of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(std::string id) { return data[id].name; }

/**
 * GetNeighborIDs: Get the neighbor ids of a Node.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(std::string id) { return data[id].neighbors;}


/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {Node} a  : node a
 * @param  {Node} b  : node b
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const Node &a, const Node &b) {
  // TODO: Use Haversine Formula:
  // dlon = lon2 - lon1;
  // dlat = lat2 - lat1;
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double a_lat = a.lat * M_PI / 180.0;
  double b_lat = b.lat * M_PI / 180.0;
  double r = pow((sin(dlat / 2)), 2) + cos(a_lat) * cos(b_lat) * pow((sin(dlon / 2)),2);
  double c = 2 * asin(sqrt(r));
  double distances = 3961 * c;

  // where 3961 is the approximate radius of the earth at the latitude of
  // Washington, D.C., in miles
  return distances;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  double sum = 0;
  for (auto it = 0; it < path.size() - 1; it++)
  {
    sum += CalculateDistance(data[path[it]], data[path[it+1]]);
  }
  return sum;
}

/**
 * SINA
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name) {
  std::vector<std::string> results;
  
  for (auto x: data) {
    
    Node node = x.second;
    std::string location = (node.name);
    std::transform(location.begin(), location.end(), location.begin(), ::tolower);
    std::transform(name.begin(), name.end(),name.begin(), ::tolower);
    if (location.rfind(name, 0) == 0) {
  // s starts with prefix
    results.push_back(node.name);
    }


  }
  
  return results;
}

/**
 * MATT
 * GetPosition: Given a location name, return the position.
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  if(name_to_id.empty()) MapNames();
  if(name_to_id.find(name) != name_to_id.end())
  {
    results.first = data[name_to_id[name]].lat;
    results.second = data[name_to_id[name]].lon;
    PlotPoint(name_to_id[name]);
  }
  return results;
}

/**
 * MATT
 * CalculateShortestPath: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath(std::string location1_name, std::string location2_name) 
{
  std::vector<std::string> path;
  if(name_to_id.empty()) MapNames();
  if(!name_to_id.count(location1_name) || !name_to_id.count(location2_name)) return {};
  Node point1 = data[name_to_id[location1_name]];
  Node point2 = data[name_to_id[location2_name]];
  std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
  /************************* A* Algorithm ********************************/
  if(A_ALGORITHM)
  {  
    std::set<std::string> visited_nodes;
    std::stack<std::string> visited_node_stack;
    Node next_hop, current_node;
    current_node = point1;
    double shortest_euclid = DBL_MAX;
    int count = 0;
    double euclid_dist;
    double current_dist = DBL_MAX;
    int increase_counter = 0;
    int increase_thresh = 5;
    while(current_node.id != point2.id)
    {
      count++;
      visited_nodes.insert(current_node.id);      //add node id to visited list
      for(auto items : current_node.neighbors)    //explore all neighbors from current node
      {

        if(visited_nodes.find(items) != visited_nodes.end()){}
        else
        {
          euclid_dist = CalculateDistance(data[items], point2); //calculate Euclidean dist from neighbor to destination (Heuristic)
          if (euclid_dist < shortest_euclid)        //compare neighbor euclid dist to current euclid shortest dist
          {
            shortest_euclid = euclid_dist;        //set euclid dist to new shortest euclid dist
            next_hop = data[items];                                //set next hop to this neighbor node
          }
        }
      }

      if(increase_counter >= increase_thresh)
      {
        std::cout << "Entering back tracking phase." << std::endl;
        for(int i = increase_counter; i > 0; i--)
        {
          if(!visited_node_stack.empty()) visited_node_stack.pop();
        }
        current_node = data[visited_node_stack.top()];
        std::cout << "Popped nodes off stack. Entering probing phase." << std::endl;
        increase_counter = 0;
        bool neighbors = false;
        while(!neighbors)
        {
          std::cout << "Checking for neighbors" << std::endl;
          for(auto items : current_node.neighbors)
          {
            if(visited_nodes.find(items) == visited_nodes.end()) neighbors = true; //unvisited neighbor
          }
          if(!neighbors)
          {
            if(!visited_node_stack.empty())
            {
              visited_node_stack.pop();
              current_node = data[visited_node_stack.top()];
            }
            else break; //stack empty 
          }
        }
        next_hop = current_node;
        current_dist = shortest_euclid;
      }
      else if (next_hop.id == current_node.id)         //node was a dead end, return to previous node and try again
      {
        current_node = data[visited_node_stack.top()];
        visited_node_stack.pop();
        next_hop = current_node;
      }
      else                                        //unique node on new path
      {
        visited_node_stack.push(current_node.id);
        current_node = next_hop;           //make current node the next hop
        if(shortest_euclid <= current_dist)
        {
          std::cout << "Reset" << std::endl;
          current_dist = shortest_euclid;
          increase_counter = 0;
        } 
        else
        {
          std::cout << "Increased" << std::endl;
          increase_counter++;
        }
      }
      
        
      shortest_euclid = DBL_MAX;         //reset shortest distance
    }
    std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
    auto TOTAL_TIME = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    visited_node_stack.push(current_node.id);
    while(!visited_node_stack.empty())
    {
      path.push_back(visited_node_stack.top());
      visited_node_stack.pop();
    }
    std::cout << "A* Algorithm has completed the shortest path calculation!\n" << std::endl;
    std::cout << "Total Time: " << TOTAL_TIME << " microseconds" << std::endl;
    std::cout << "Direct path distance: " << CalculateDistance(data[name_to_id[location1_name]],data[name_to_id[location2_name]]) << " miles" << std::endl;
    std::cout << "Path length: " << CalculatePathLength(path) << " miles" << std::endl;
    std::cout << "Nodes along path: " << path.size() << std::endl;
    std::cout << "Steps taken: " << count << std::endl;
  }

  
  /***********************************************************************/

  /********************** DJIKSTRAS Algorithm ****************************/
  if(DJIKSTRAS_ALGORITHM)
  {
    //build adjacency list
    std::map<std::string, std::vector<std::pair<std::string,double>>> adjacency_list;
    for(auto item : data)
    {
      for(auto neighbor : item.second.neighbors)
      {
        adjacency_list[item.second.id].push_back(std::pair<std::string,double>(neighbor, CalculateDistance(item.second,data[neighbor])));
      }
    }
    typedef std::pair<double,std::pair<std::string,std::string>> pq_node;
    std::priority_queue<pq_node,std::vector<pq_node>,std::greater<pq_node>> pq; //{dist, (prev_node,current_node)}

    //add starting node's neighbors to priority queue
    for(auto x : adjacency_list[point1.id])
    {
      pq.push(make_pair(x.second, std::pair<std::string,std::string>(point1.id, x.first))); //{total_dist,{prev_node, next_node}}
    } 

    //add starting node to visited_nodes map
    std::map<std::string, double> visited_nodes = {{point1.id,0.0}};  //total distance to the dest. node (key)
    std::map<std::string, std::string> direction_map;               //input dest. node (key) and value is prev node on shortest path

    pq_node current;
    int count = 0;
    while(!pq.empty())
    {
      count++;
      //pop shortest path node off priority queue
      current = pq.top();
      pq.pop();

      if(!visited_nodes.count(current.second.second))                 //unvisited node
      {
        visited_nodes[current.second.second] = current.first;         //add to visited nodes {current_node: total_dist}
        direction_map[current.second.second] = current.second.first;  //add to direction map {current_node: prev_node}
      }
      else
      {
        if( (current.first) < visited_nodes[current.second.second])
        {
          visited_nodes[current.second.second] = current.first;
          direction_map[current.second.second] = current.second.first;
        }
      }
      for(auto neighbor : adjacency_list[current.second.second])
      {
        if(neighbor.first != current.second.first && !visited_nodes.count(neighbor.first)) //dont add neighbor that node came from or that has been visited
        {
        //Add node to priority queue
        //Node: {total_dist, {current_node, next_node}}
        pq.push(make_pair(neighbor.second + visited_nodes[current.second.second], std::pair<std::string,std::string>(current.second.second, neighbor.first)));
        }
      }
    }
    std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
    auto TOTAL_TIME = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    
    //use direction map to create path from source to destination
    std::string temp;
    std::string prev_node = point2.id;
    while(prev_node != point1.id)
    {
      path.push_back(prev_node);
      temp = direction_map[prev_node];
      prev_node = temp;
    }
    path.push_back(prev_node);
    std::reverse(path.begin(),path.end()); //list returned is destination->source so we reverse it

    std::cout << "Dijkstra Algorithm has completed the shortest path calculation!\n" << std::endl;
    std::cout << "Iterations: " << count << std::endl;
    std::cout << "Total Time: " << TOTAL_TIME << " microseconds" << std::endl;
    std::cout << "Direct path distance: " << CalculateDistance(point1,point2) << " miles" << std::endl;
    std::cout << "Path length: " << CalculatePathLength(path) << " miles" << std::endl;
    std::cout << "Nodes along path: " << path.size() << std::endl;
  }
  /***********************************************************************/
  PlotPath(path);
  return path;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan(
                                    std::vector<std::string> &location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> results;
  
  // int16_t ALGO_TYPE;
  // std::cout << "Please type in the algorithm you would like to use:" << std::endl << "1 = Brute Force" << std::endl << "2 = 2-Opt" << std::endl << "3 = 3-Opt"<< std::endl;
  // std::cin >> ALGO_TYPE;

  if (ALGO_TYPE == 1) {                       
  //location_ids.push_back(location_ids[0]);
  std::vector<std::string> min_path;
  
  std::sort(location_ids.begin()+1,location_ids.end());
  location_ids.push_back(location_ids[0]);
  results.second.push_back(location_ids);
  while(std::next_permutation(location_ids.begin()+1, location_ids.end()-1)) {
      results.second.push_back(location_ids);
  }
  double min_sum;
  min_sum = DBL_MAX;
//   for (int i = 0; i < results.second.size(); i++) {
//     results.second[i].push_back(location_ids[0]);
    

//   }
  
// for (auto it = results.second.begin(); it != results.second.end();) {
//     std::string first_id = (*it)[0];
//     if(first_id != location_ids[0]) {
//       it  = results.second.erase(it);
//     }
//     else {
//       ++it;
//     }
// }
  //std::vector<double> sums;
  for (auto m: results.second ){
      
      //std::cout<<m[0]<< " " << m.back() <<" "<<m.size()<<std::endl;
      
      double sum = CalculatePathLength(m);
        
    if (sum <= min_sum) {
      min_sum = sum;
      min_path = m;
    }
    //sums.push_back(sum);
    //std::cout<<sum <<std::endl;
  }
  
  results.first = min_sum;
  results.second.push_back(min_path);
  return results;
} 


  if(ALGO_TYPE == 2) {
    //std::vector<std::string> locations  = location_ids;
    //location_ids.push_back(location_ids[0]);
    // for (auto n: locations) {
    //   std::cout << n << std::endl;
    // }
    results.second.push_back(location_ids);
    int improve = 0;
    int size = location_ids.size();
    double best_distance;
    while (improve < 50)  {

      best_distance = CalculatePathLength(location_ids);
      for (int i = 0; i < size -1; i++) {
        for(int k = i+1; k <size; k++) {
          std::vector<std::string> new_locations;
          twoOptSwap(i,k,location_ids, new_locations);
          double new_distance = CalculatePathLength(new_locations);
          
          if(new_distance < best_distance) {
            improve = 0;
            location_ids = new_locations;
            best_distance = new_distance;
            //locations.push_back(location_ids[0]);
            results.second.push_back(location_ids);
          }
        }
      }
      improve++;

    }
    results.first = best_distance;
      for (int i = 0; i < results.second.size(); i++) {
    results.second[i].push_back(location_ids[0]);
    
  }

    return results;
  }


if(ALGO_TYPE == 3) {
    //std::vector<std::string> locations  = location_ids;
    //location_ids.push_back(location_ids[0]);
    // for (auto n: locations) {
    //   std::cout << n << std::endl;
    // }
    results.second.push_back(location_ids);
    int improve = 0;
    int size = location_ids.size();
    double best_distance;
    while (improve < 50)  {

      best_distance = CalculatePathLength(location_ids);
      for (int i = 0; i < size-1; i++) {
        for(int j = i+1; j <size; j++) {
          for(int k = j+1; k < size; k++) {
            std::vector<std::string> new_locations, new_locations1,new_locations2, new_locations3;
            twoOptSwap(i,j,location_ids, new_locations1);
            twoOptSwap(j,k,location_ids, new_locations2);
            twoOptSwap(i,k,location_ids, new_locations3);

            double d1 = CalculatePathLength(new_locations1);
            double d2 = CalculatePathLength(new_locations2);
            double d3 = CalculatePathLength(new_locations3);
                if (d1 < d2 && d1 < d3) {
                  new_locations = new_locations1;
                }
                else if (d2 < d3 && d2 < d1) {
                  new_locations = new_locations2;
                }

                else{
                  new_locations = new_locations3;
                }
            double new_distance = CalculatePathLength(new_locations);
            
            if(new_distance < best_distance) {
              improve = 0;
              location_ids = new_locations;
              best_distance = new_distance;
              //locations.push_back(location_ids[0]);
              results.second.push_back(location_ids);
            }
        }
        }
      }
      improve++;

    }
    results.first = best_distance;
      for (int i = 0; i < results.second.size(); i++) {
    results.second[i].push_back(location_ids[0]);
    
  }

    return results;
  }
  
  }

void twoOptSwap( const int&i, const int &k, std::vector<std::string> &route, std::vector<std::string> &newRoute) {
  
  int size = route.size();
  for (int c = 0; c < i; c++) {
    newRoute.push_back(route[c]);
  }

  for (int c = k; c >= i; --c) {
    newRoute.push_back(route[c]);
  }

  for (int c = k+1; c < size; ++c) {
    newRoute.push_back(route[c]);
  }

}

void permuteBruteForceHelper(std::vector<std::string> &input, std::vector<std::vector<std::string>> &result, std::vector<std::string> currentResult) {

  if(currentResult.size() == input.size()) {
    result.push_back(currentResult);
    return;
  }

  for (int i = 0; i < input.size(); i++)  {
    //std::cout<<input[i] << std::endl;
    if(std::find(currentResult.begin(), currentResult.end(), input[i]) != currentResult.end()) {
      continue;
    }

    std::vector<std::string> nextCurrentResult = currentResult;
    nextCurrentResult.push_back(input[i]);
    permuteBruteForceHelper(input, result, nextCurrentResult);
  }

}

