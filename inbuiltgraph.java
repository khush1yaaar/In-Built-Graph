package InBuiltGraph;

import java.util.ArrayList; 

public class inbuiltgraph {
    static class Graph {
        ArrayList<ArrayList<Integer>> adj;
        int[][] matrix;
        public Graph(ArrayList<Edge> edges, boolean isUndirected) {
            if(isUndirected) {
                createUndirectedGraph(edges);
            }
            else{
                createDirectedGraph(edges);
            }
        }
        public void createDirectedGraph(ArrayList<Edge> edges) { // CREATE DIRECTED GRAPH

        }
        public void createUndirectedGraph(ArrayList<Edge> edges) { // CREATE UNDIRECTED GRAPH

        }
        public int shortestPath(int src, int dest) {
            return -1;
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

        Graph graph = new Graph(edges, true);
        graph.isCycle();
    }
}
