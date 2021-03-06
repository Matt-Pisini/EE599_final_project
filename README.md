EE599 Final Project
Matthew Pisini & Sina Mahbobi

Extra Credits: 
1) second shortest path algorithm (A*); 
2) 3-opt algorithm for TSP.

About The Program: 
- For CalculateShortestPath(), we employed both A* and Dijkstra's algorithm. By default, Dijkstra's algorithm is used. However, in the menu statement after you select option 3 (CalculateShortestPath), there is an option to choose A* algorithm instead.
- TSP was implemented using a brute force method, and two opt and three opt heuristic methods. There is a global variable ALGO_TYPE for the three different types of TSP solutions. The default is 1, which means the brute force solution. It is changed to 2 for the TwoOpt test within the test function,  and it is changed to 3 for the ThreeOpt test within the test function. To change it when running it from the printMenu function, the user is prompted for which algorithm they would like to use and ALGO_TYPE flag is changed accordingly.
- We added an if statement to PrintMenu() which ensures that the user-entered number of places to travserse is non-negative (otherwise segfault occurred). If the number of places to traverse is 1, that point is returned with distance = 0.

1. High-level overview of your design (Use diagrams and pictures)
The input to the program is a large CSV file with all points on the graph labeled with a unique ID, latitude, longitude, and a list of neighboring nodes. This file is read in and a private data structure is created in our TrojanMap class to hold all the values. The user interfaces with the program by selecting functions from a list of menu options and supplying additional input for the selected task. The following functions are included in the menu: autocomplete, which takes a string and returns any names from the data structure that begin with the specified substring; find the position, which returns the latitude and longitude of the specified node; calculate shortest path, which returns a list of coordinates that is the shortest distance between two nodes and plots the path on a map; and the traveling salesman problem, which finds the shortest path between all nodes specified.


2. Detailed description of each function and its time complexity.
MapNames():
This is a helper function that we created in order to map the name of a node to its ID. We iterate through all members in the data map and create another map for name to ID so that when given the name as a parameter, we can use this map as a lookup table. The time complexity is O(nlogn) because we iterate through all members of 'data' which is n and we insert values into our map every iteration which takes logn.

GetName():
Uses parameter of string id as the key to access the Node structure in 'data' and returns the 'name' member variable. This is O(logn) time complexity.

GetLat():
Uses parameter of string id as the key to access the Node structure in 'data' and returns the 'lat' member variable. This is O(logn) time complexity.

GetLon():
Uses parameter of string id as the key to access the Node structure in 'data' and returns the 'lon' member variable. This is O(logn) time complexity.

GetNeighborIDs():
Uses parameter of string id as the key to access the Node structure in 'data' and returns the 'neighbors' member variable. This is O(logn) time complexity.

CalculateDistance():
This function uses the Haversine formula for calculating the distance between two points on the earth based on their latitude and longitude values. It takes two node data structures by reference as parameters and then uses their member variables of 'lat' and 'lon' to calculate distance using several functions from STL math libraries. The time complexity of this function is constant O(1).

CalculatePathLength():
This function receives by reference a vector or strings as a parameter, which it iterates over calculating the distance between every node using CalculateDistance() functiion and adding it to a sum. The time complexity of this function is O(n).

AutoComplete():
This function takes in a prefix, and returns the different locations with that prefix. we convert everything to lowercase just in case there are capital letters in the prefix or the location but someone types it in in lowercase. The time complexity is O(n).

GetPosition():
Uses parameter of string id as the key to access the Node structure in 'data' and returns the 'lat' and 'lon' member variables as a pair. This is O(logn) time complexity.

CalculateShotestPath():
We implemented 2 different algorithms for the shortest path calculation, A* and Dijkstra's.
A*:
In this implementation, we take a simple iterative approach to reaching our destination. Starting at the current node, we keep exploring nodes until we reach the destination node. In every iteration, we calculate the distance from every neighbor to the final destination and then choose the next hop node as the neighbor with shortest total distance to the destination. We do not consider the neighbor that would return to the previous node. In every iteration, we add the current node to a stack data structure. If a path is taken that leads to a dead end, we can continue to pop previous nodes off the stack until we return to a node where we can explore a new path. This process keeps occurring until we reach the destination node. One other addition to the heuristic aspect of this algorithm is determing when the path is heading in a wrong direction. If 5 consecutive nodes continue to be further away from the destination, then we enter a phase of back-trracking, popping values off the stack until we settle on an unexplored path. 
The time complexity of this implementation is thus O(n(logn + m)), because we go through n iterations of the loop until we reach destination and every time through the loop we always insert into a set which takes logn. The m represents a for loop within our main loop which at worst would be equal to nlogn, when we have a complete graph where each node can reach every other node. We would check all these neighbor node values and we use the ID to check a map which takes logn complexity. The heuristic we use to determine the node closest to the final destination is of constant time complexity, which ensures our A* implementation is lower than its upper bound exploring all iterations of every node. This variable remains as m because in our situation we do not have a complete graph so it will be much less. The back-tracking phase is the same complexity as the normal progression of this function O(nlogn), so entering that phase only prolongs the path finding at the same complexity. 
Time Complxity: O(n(logn + m))

Dijkstra:
In this implementation, we exhaustively find the shortest path by iteterating through all nodes in the list. The first step is creating an adjacency list using 'data'. The adjacency list is a map with the current node as the key and the corresponding value a list of pairs with the members as distance from the node to the neighbor node. We begin by pushing the source's neighbors onto a priority queue that is ordered by the minimal total distance to a node. We keep iterating, popping the shortest distance member off the priority queue and adding all neighbors onto the priority queue until the priority queue is empty (all nodes explored). Additionally, we use two maps, called visted_nodes and direction_map; the first stores the current node and the current total distance to get to it from the source, and the second map stores the current node as key and the previous node from which it come from on the shortest distance path. Every iteration, we pop off the shortest distance node in the queue (previously added neighbors) and add a neighbor node if it hasn't been explored. If it has been explored we add the path from current node to previous node if the total distance is less than the stored value; if the distance is less, we add the value as the new distance in our visted_nodes map and update the previous node in directed_map. We then go in reverse, using the destination as the key in the directed_map and iteratively using the value from that map (previous node value) as the next key to reach the source node. All of those nodes are pushed to a list which is returned from the function. The time complexity of this implementation requires every edge of the graph to be visited with that path being added to a priority queue. We will denote the amount of edges as E and the amount of Vertices (i.e. nodes) as V. So we iterate through our while loop E times, each time pushing onto a priority queue of size V, which would give time complexity of O(ElogV). Additionally, in each iteration we determine whether the node has been visited, and if so we update the value of the path (via our map data structures) if this new path is lesser. So, the leads to a time complexity of O(E(logV+logV)) which simplifies to O(E(logV)).

TSP BruteForce():
 Using a DFS like algorithm, we generate all possible permutations of the random locations, and keep the ones that the start point as the first and last location. We then calculate the path distance for all of these, and append the one with the shortest distance at the end of the vector of results, to show it after all iterations have been shown on openCV. The minimum sum is saved as well and added to results.

 TSP Two Opt:
 The Two Opt solution to TSP is only a heuristic, so it provides a local minima for the distance minimization rather than a global minima. For this alogrithm, it takes O(n^2) time to do each two opt swap. doing this across every value in th evector results in a total on O(n^3) time.In this algorithm, we do a two opt swapt for every vector value after the first, and threshold the improvement in a while loop. When the minimum distance stops improving, it breaks to the loop and the last vector asdded to the result is the local minima we have found for this two opt swap. The two opt swap takes three parameters, the route, i and k. We add from 0 to i to the new route, reverse from i to k and that tot he new route, then add k to the end of the route passed in to the new route. This is the swap. We then do this for value so of i and k, where k starts at the value of i and goes to the end of the route. this results in O(n^3) time for all the swapping.

 TSP Three Opt:
 The Three opt solution is implemented using a series of two opts, since a three opt solution can be broken down into a series of three two opt swaps. From those two opt swaps, we pick the one that reduces the path length the most, and select that one as the new location and new distance. We then use the same process as the the two opt swap, execept that we run through one extra loop to accoount for the third swap. This results in a time complexity of O(n^4). O(n^3) for the swap itself, and then O(n) to iterate through the entire vector.

3. Discussion, conclusion, and lessons learned.

In the shortest path calculation, we employed two algorithms in order to explore tradeoffs between computation time and total path distance. Both Djikstra and A* algorithm are greedy algorithms, but Dijkstra's is guarenteed to get the shortest path becuase it explores all possibilities along shortest paths in the graph, while A* ensures a good path without exhausting all search paths. We added a timing feature to help compare the two algorithms. When running A*, the computation completed in around 1000-1500 microseconds compared to Dijkstra's which took around 350,000-400,000 microseconds, a speed difference of nearly 250x. Although A* comes very close to the perfect path, often times indistinguishable to the naked eye, it still normally had a slightly longer path by a margin of less than 1%. Looking at both algorithms, it would be better to utilize A* because of the massive boost in computational efficiency compared to the marginal loss in path efficiency. However, depending on the nature of the graph Dijkstra could perform much better. The graph we are using is very straightforward and predictable, which is why A* performs so well. However, in a graph where paths are abnormal, A* could prove to be significantly less effective.

When implementing A* for the shortest path algorithm, one issue I was having was returning to a previous nodee if the path I went down was a dead end. I learned that for scenarios like this, when you need to go backwards from a current iteration/poition, a stack is a perfect data structure to use. It is similiar to pipeline design in computer architecture where the processor does predictive branching, but if it makes the wrong prediction it needs to pop off all the values it stored from the cycle before into the registers (aka return previous state). This was a valuable lesson because it really merged my understanding of SW data structures and the underlying computer architecture that it operates on.