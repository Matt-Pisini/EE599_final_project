EE599 Final Project
Matthew Pisini & Sina Mahbobi


1. High-level overview of your design (Use diagrams and pictures)


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

GetPosition():
Uses parameter of string id as the key to access the Node structure in 'data' and returns the 'lat' and 'lon' member variables as a pair. This is O(logn) time complexity.

CalculateShotestPath():
We implemented 2 different algorithms for the shortest path calculation, A* and Dijkstra's.
A*:
In this implementation, we take a simple iterative approach to reaching our destination. Starting at the current node, we keep exploring nodes until we reach the destination node. In every iteration, we calculate the distance from every neighbor to the final destination and then choose the next hop node as the neighbor with shortest total distance to the destination. We do not consider the neighbor that would return to the previous node. In every iteration, we add the current node to a stack data structure. If a path is taken that leads to a dead end, we can continue to pop previous nodes off the stack until we return to a node where we can explore a new path. This process keeps occurring until we reach the destination node. The time complexity of this implementation is thus O(n(logn + m)), because we go through n iterations of the loop until we reach destination and every time through the loop we always insert into a set which takes logn. The m represents a for loop within our main loop which at worst would be equal to nlogn, when we have a complete graph where each node can reach every other node. We would check all these values and we use the ID to check a map which takes logn complexity. This variable remains as m because in our situation we do not have a complete graph.

Dijkstra:
In this implementation, we exhaustively find the shortest path by iteterating through all nodes in the list. The first step is creating an adjacency list using 'data'. The adjacency list is a map with the current node as the key and the corresponding value a list of pairs with the members as distance from the node to the neighbor node. We begin by pushing the source's neighbors onto a priority queue that is ordered by the minimal total distance to a node. We keep iterating, popping the shortest distance member off the priority queue and adding all neighbors onto the priority queue until the priority queue is empty (all nodes explored). Additionally, we use two maps, called visted_nodes and direction_map; the first stores the current node and the current total distance to get to it from the source, and the second map stores the current node as key and the previous node from which it come from on the shortest distance path. Every iteration, we pop off the shortest distance node in the queue (previously added neighbors) and add a neighbor node if it hasn't been explored. If it has been explored we add the path from current node to previous node if the total distance is less than the stored value; if the distance is less, we add the value as the new distance in our visted_nodes map and update the previous node in directed_map. We then go in reverse, using the destination as the key in the directed_map and iteratively using the value from that map (previous node value) as the next key to reach the sourcee node. All of those nodes are pushed to a list which is returned from the function. The time complexity of this implementation  

3. Discussion, conclusion, and lessons learned.

In the shortest path calculation, we employed two algorithms in order to explore tradeoffs between computation time and total path distance. Both Djikstra and A* algorithm are greedy algorithms, but Dijkstra's is guarenteed to get the shortest path becuase it explores all possibilities along shortest paths in the graph, while A* ensures a good path without exhausting all search paths. We added a timing feature to help compare the two algorithms. When running A*, the computation completed in around 1000-1500 microseconds compared to Dijkstra's which took around 350,000-400,000 microseconds, a speed difference of nearly 250x. Although A* comes very close to the perfect path, often times indistinguishable to the naked eye, it still normally had a slightly longer path by a margin of less than 1%. Looking at both algorithms, it would be better to utilize A* because of the massive boost in computational efficiency compared to the marginal loss in path efficiency. However, depending on the nature of the graph Dijkstra could perform much better. The graph we are using is very straightforward and predictable, which is why A* performs so well. However, in a graph where paths are abnormal, A* could prove to be significantly less effective.

When implementing A* for the shortest path algorithm, one issue I was having was returning to a previous nodee if the path I went down was a dead end. I learned that for scenarios like this, when you need to go backwards from a current iteration/poition, a stack is a perfect data structure to use. It is similiar to pipeline design in computer architecture where the processor does predictive branching, but if it makes the wrong prediction it needs to pop off all the values it stored from the cycle before into the registers (aka return previous state). This was a valuable lesson because it really merged my understanding of SW data structures and the underlying computer architecture that it operates on.