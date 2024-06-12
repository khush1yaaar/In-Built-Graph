# Graph Library

## Overview
This library provides a simple and versatile implementation of a graph data structure in Java, supporting both directed and undirected graphs. It includes various functionalities such as adding/removing edges, BFS, DFS, shortest path algorithms, cycle detection, topological sorting, and finding bridges.

## Features
- **Graph Creation**: Supports both directed and undirected graphs.
- **Graph Representation**: Adjacency list.
- **Graph Traversal**: BFS and DFS.
- **Shortest Path**: Dijkstra and Bellman-Ford algorithms.
- **Cycle Detection**: For both directed and undirected graphs.
- **Topological Sorting**: For directed acyclic graphs (DAGs).
- **Bridges**: Finding all bridges in the graph.
- **Connected Components**: Count number of connected components in the graph.

## Usage

### Creating a Graph
- **Undirected Graph**
  ```java
  ArrayList<Edge> edges = new ArrayList<Edge>();
  edges.add(new Edge(0, 1));
  edges.add(new Edge(0, 2));
  // Add more edges as needed
  Graph graph = new Graph(edges, 7, true);
  ```

- **Directed Graph**
  ```java
  ArrayList<Edge> edges = new ArrayList<Edge>();
  edges.add(new Edge(0, 1));
  edges.add(new Edge(1, 2));
  // Add more edges as needed
  Graph graph = new Graph(edges, 7, false);
  ```

### Adding Edges
- **Undirected Edge**
  ```java
  graph.add(3, 4, true); // Adds an undirected edge between nodes 3 and 4
  ```
- **Directed Edge**
  ```java
  graph.add(3, 4, false); // Adds a directed edge from node 3 to node 4
  ```

### Removing Edges
- **Undirected Edge**
  ```java
  graph.remove(3, 4, true); // Removes the undirected edge between nodes 3 and 4
  ```
- **Directed Edge**
  ```java
  graph.remove(3, 4, false); // Removes the directed edge from node 3 to node 4
  ```

### Graph Traversal
- **BFS**
  ```java
  ArrayList<Integer> bfsTraversal = graph.bfs(0); // Starting from node 0
  ```
- **DFS**
  ```java
  ArrayList<Integer> dfsTraversal = graph.dfs(0); // Starting from node 0
  ```

### Shortest Path
- **Dijkstra's Algorithm**
  ```java
  int shortestPath = graph.shortestPath(0, 5, false, false); // From node 0 to node 5
  ```
- **Bellman-Ford Algorithm**
  ```java
  int shortestPath = graph.shortestPath(0, 5, true, false); // From node 0 to node 5 with negative weights
  ```

### Cycle Detection
- **Undirected Graph**
  ```java
  boolean hasCycle = graph.isCycle(true);
  ```
- **Directed Graph**
  ```java
  boolean hasCycle = graph.isCycle(false);
  ```

### Topological Sorting
- **Topological Sort**
  ```java
  ArrayList<Integer> topoSort = graph.topoSort(false); // Only for directed graphs
  ```

### Finding Bridges
- **Bridges in Graph**
  ```java
  List<List<Integer>> bridges = graph.bridges();
  ```

### Connected Components
- **Number of Components**
  ```java
  int components = graph.components();
  ```

## Helper Functions
These helper functions are used internally by the library:
- `Helper.dijkstra()`
- `Helper.bellmanFord()`
- `Helper.dfsHelper()`
- `Helper.bfsHelper()`
- `Helper.isCycleHelper1()`
- `Helper.isCycleHelper2()`
- `Helper.topoSortHelper()`

## Example
```java
public class Main {
    public static void main(String[] args) {
        ArrayList<Edge> edges = new ArrayList<Edge>();
        edges.add(new Edge(0, 1));
        edges.add(new Edge(0, 2));
        edges.add(new Edge(1, 2));
        edges.add(new Edge(1, 3));
        edges.add(new Edge(2, 4));
        edges.add(new Edge(3, 5));
        edges.add(new Edge(4, 5));
        edges.add(new Edge(5, 6));
        Graph graph = new Graph(edges, 7, true);

        System.out.println("BFS: " + graph.bfs(0));
        System.out.println("DFS: " + graph.dfs(0));
        System.out.println("Shortest Path (0 to 6): " + graph.shortestPath(0, 6, false, false));
        System.out.println("Contains Cycle: " + graph.isCycle(true));
        System.out.println("Number of Components: " + graph.components());
        System.out.println("Bridges: " + graph.bridges());
    }
}
```

## Contributing
Feel free to fork the repository, make improvements, and submit a pull request.

