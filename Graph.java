package InBuiltGraph;

import java.util.ArrayList; 

public class Graph {
        ArrayList<ArrayList<Integer>> adj;
        int[][] matrix;
        int nodes;
        public Graph() {

        }
        public Graph(ArrayList<Edge> edges, int nodes, boolean isUndirected) {
            this.nodes = nodes;
            adj = new ArrayList<>(nodes);
            for(int i=0;i<nodes;i++) {
                this.adj.add(new ArrayList<>());
            }
            if(isUndirected) {
                createUndirectedGraph(edges);
            }
            else{
                createDirectedGraph(edges);
            }
        }
        public void createDirectedGraph(ArrayList<Edge> edges) { // CREATE DIRECTED GRAPH
            for(Edge e : edges) {
                int u = e.u;
                int v = e.v;
                adj.get(u).add(v);
            }
            nodes = adj.size();
        }
        public void createUndirectedGraph(ArrayList<Edge> edges) { // CREATE UNDIRECTED GRAPH
            for(Edge e : edges) {
                int u = e.u;
                int v = e.v;
                adj.get(u).add(v);
                adj.get(v).add(u);
            }
            nodes = adj.size();
        }
        public void add(int u, int v, boolean isUndirected) {
            if (u >= nodes || v >= nodes) { 
                for (int i = nodes; i <= Math.max(u, v); i++) {
                    adj.add(new ArrayList<>());
                }
                nodes = Math.max(u, v) + 1; 
            }
            this.adj.get(u).add(v);
            if(isUndirected) {
                this.adj.get(v).add(u);
            }
            nodes = this.adj.size();
        }
        public void remove(int u, int v, boolean isUndirected) {
            this.adj.get(u).remove(v);
            if(isUndirected) {
                this.adj.get(v).remove(u);
            }
            nodes = this.adj.size();
        }
        public int shortestPath(int src, int dest, boolean isNegativeWeighted) {
            if(isNegativeWeighted) {
                return bellManFord();
            }
            else {
                return dijkstras();
            }
        }

        public boolean isCycle() { // CHECK IF THERE EXISTS A CYCLE
            return false;
        }

        public boolean DAG() { // CHECK IF IT'S A DIRECTED ACYCLIC GRAPH
            return false;
        }

        public ArrayList<Integer> dfs() { // RETURNS DFS TRAVERSAL
            ArrayList<Integer> ans = new ArrayList<>();
            return ans;
        }

        public ArrayList<Integer> bfs() { // RETURNS BFS TRAVERSAL
            ArrayList<Integer> ans = new ArrayList<>();
            return ans;
        }

        public ArrayList<Integer> topo() { // RETURN TOPO SORT
            ArrayList<Integer> ans = new ArrayList<>();
            return ans;
        }

        public boolean find(int x) { // CHECKS IF THE NODE EXISTS OR NOT
            return false;
        }
        public ArrayList<ArrayList<Integer>> toAdjacencyList(int[][] matrix) { // CONVERTS TO ADJACENCY LIST
            ArrayList<ArrayList<Integer>> ans = new ArrayList<>();

            return ans;
        }
        public int[][] toMatrix(ArrayList<ArrayList<Integer>> adj) { // CONVERTS TO 2D MATRIX
            int n = adj.size();
            int[][] ans = new int[n][n];

            return ans;
        }
        public int components() { // RETURNS NUMBER OF COMPONENTS IN THE GRAPH
            return -1;
        }
    
    static class Edge {
        int u;
        int v;
        int wt = 1;
        public Edge(int u, int v) {
            this.u = u;
            this.v = v;
        }
        public Edge(int u, int v, int wt) {
            this.u = u;
            this.v = v;
            this.wt = wt;
        }
    }
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
        
        System.out.println(graph.adj.get(0).get(0));
        System.out.println(graph.nodes);
        graph.add(7,6,true);
        System.out.println(graph.adj.get(7).get(0));
        System.out.println(graph.nodes);
    }
}

// Dijkstra’s Algorithm for shortest paths in weighted graphs.
// Bellman-Ford Algorithm for graphs with negative weights.
// Floyd-Warshall Algorithm for all-pairs shortest paths.
// Tarjan’s Algorithm for finding strongly connected components.
// Minimum Spanning Tree algorithms like Kruskal’s and Prim’s.

