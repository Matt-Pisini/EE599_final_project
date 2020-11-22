EE599 Final Project
Matthew Pisini & Sina Mahbobi


1. High-level overview of your design (Use diagrams and pictures)


2. Detailed description of each function and its time complexity.

CalculateShotestPath():
We implemented 2 different algorithms for the shortest path calculation, A* and Dijkstra's.
A*:
In this implementation, we take a simple iterative approach to reaching our destination. Starting at the current node, we keep exploring nodes until we reach the destination node. In every iteration, we calculate the distance from every neighbor to the final destination and then choose the next hop node as the neighbor with shortest total distance to the destination. We do not consider the neighbor that would return to the previous node. In every iteration, we add the current node to a stack data structure. If a path is taken that leads to a dead end, we can continue to pop previous nodes off the stack until we return to a node where we can explore a new path. This process keeps occurring until we reach the destination node. The time complexity of this implementation 

3. Discussion, conclusion, and lessons learned.

When implementing A* for the shortest path algorithm, one issue I was having was returning to a previous nodee if the path I went down was a dead end. I learned that for scenarios like this, when you need to go backwards from a current iteration/poition, a stack is a perfect data structure to use. It is similiar to pipeline design in computer architecture where the processor does predictive branching, but if it makes the wrong prediction it needs to pop off all the values it stored from the cycle before into the registers (aka return previous state). This was a valuable lesson because it really merged my understanding of SW data structures and the underlying computer architecture that it operates on.