#  A-Star-Path-Finding-Visualization

A python program made with Pygame that visualizes path finding approach of A* algorithm using manhattan hueristic.

### Requirement
 
```
pip install pygame
```

### How it works

select any one tile as starting node, 
Then select any other tile as other node, 
Select different tiles to create barriers,
Right click any selected tile to reset it,  
Press '1' for A*, '2' for Dijkstra or '3' for Prim's (the status bar shows the selected one), 
Press Spacebar to see the algorithm work, 
Press Up/Down to speed up or slow down the search, even while it runs, 
Press 'p' to pause and resume, and Right arrow to advance one node while paused, 
Press 'm' to generate a maze with randomized Prim's (start and end are placed in opposite corners), 
Press 'c' to Reset window.

'c', 'm' and '1'/'2'/'3' also work while a search or maze is running: they stop it and take effect right away.

Colors: yellow is the node being expanded, green nodes are queued to be explored, red nodes are explored, purple is the final path.
The status bar counts explored nodes and shows the path length when the search finishes.

Dijkstra is A* without the heuristic, so it explores evenly in every direction.
Prim's builds a minimum spanning tree with random edge weights and shows the tree's route to the end, which is usually not the shortest path.
  