import java.util.*;
public class Graph {
        ArrayList<ArrayList<Pair>> adj;
        int[][] matrix;
        int nodes;
        boolean[] vis;
        public Graph() {
            this.adj = new ArrayList<>();
            this.nodes = 0;
        }
        public Graph(ArrayList<Edge> edges, int nodes, boolean isUndirected) {
            this.nodes = nodes;
            this.adj = new ArrayList<>(nodes);
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
                int wt = e.wt;
                adj.get(u).add(new Pair(v, wt));
            }
            nodes = adj.size();
        }
        public void createUndirectedGraph(ArrayList<Edge> edges) { // CREATE UNDIRECTED GRAPH
            for(Edge e : edges) {
                int u = e.u;
                int v = e.v;
                int wt = e.wt;
                adj.get(u).add(new Pair(v, wt));
                adj.get(v).add(new Pair(u, wt));
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
            this.adj.get(u).add(new Pair(v, 1));
            if(isUndirected) {
                this.adj.get(v).add(new Pair(u, 1));
            }
            nodes = this.adj.size();
        }
        public void add(int u, int v, int wt, boolean isUndirected) {
            if (u >= nodes || v >= nodes) { 
                for (int i = nodes; i <= Math.max(u, v); i++) {
                    adj.add(new ArrayList<>());
                }
                nodes = Math.max(u, v) + 1; 
            }
            this.adj.get(u).add(new Pair(v,wt));
            if(isUndirected) {
                this.adj.get(v).add(new Pair(u,wt));
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
        public int shortestPath(int src, int dest, boolean isNegativeWeighted, boolean isUndirected) {
            if (isNegativeWeighted && !isUndirected) {
                //return Helper.bellmanFord(adj, nodes, src, dest);
            } else {
                //return Helper.dijkstra(adj, nodes, src, dest);
            }
            return -1;
        }

        public boolean isCycle(boolean isUndirected) { // CHECK IF THERE EXISTS A CYCLE
            boolean[] path = new boolean[nodes];
            vis = new boolean[nodes];
            if(isUndirected) {
                for(int i=0;i<vis.length;i++) {
                    if(!vis[i]) {
                        if(Helper.isCycleHelper1(adj, vis, i)) {
                            return true;
                        }
                    }
                }
            }
            else{
                for(int i=0;i<vis.length;i++) {
                    if(!vis[i]) {
                        if(Helper.isCycleHelper2(adj, vis, path, i)) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        public boolean DAG() { // CHECK IF IT'S A DIRECTED ACYCLIC GRAPH
            return false;
        }

        public ArrayList<Integer> dfs(int source) { // RETURNS DFS TRAVERSAL
            ArrayList<Integer> ans = new ArrayList<>();
            this.vis = new boolean[nodes];
            for(int i=0;i<vis.length;i++) {
                if(!vis[i]) {
                    Helper.dfsHelper(adj,vis,ans,i);
                }
            }
            return ans;
        }

        public ArrayList<Integer> bfs(int source) { // RETURNS BFS TRAVERSAL
            ArrayList<Integer> ans = new ArrayList<>();
            this.vis = new boolean[nodes];
            for (int i = 0; i < vis.length; i++) {
                vis[i] = false;
            }
            for(int i=0;i<vis.length;i++) {
                if(!vis[i]) {
                    Helper.bfsHelper(adj,vis,ans,i);
                }
            }
            return ans;
        }

        public ArrayList<Integer> topoSort(boolean isUndirected) { // RETURN TOPO SORT
            if(isUndirected || isCycle(isUndirected)) {
                System.out.println("Not Possible");
                return new ArrayList<>();
            }
            ArrayList<Integer> ans = new ArrayList<>();
            this.vis = new boolean[nodes];
            Stack<Integer> st = new Stack<Integer>();
            for (int i = 0; i < vis.length; i++) {
                if (!vis[i]) {
                    Helper.topoSortHelper(i, vis, st, adj);
                }
            }
            while (!st.isEmpty()) {
                ans.add(st.pop());
            }
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
            ArrayList<Integer> temp = new ArrayList<>();
            int ans = 0;
            this.vis = new boolean[nodes];
            for(int i=0;i<vis.length;i++) {
                if(!vis[i]) {
                    ans++;
                    Helper.dfsHelper(adj,vis,temp,i);
                }
            }
            return ans;
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

        System.out.println(graph.bfs(0));
        // System.out.println(graph.adj.get(0).get(0));
        // System.out.println(graph.nodes);
        // graph.add(7,6,true);
        // System.out.println(graph.adj.get(7).get(0));
        // System.out.println(graph.nodes);

        // Graph graph = new Graph();
        // graph.add(0, 2, false);
        // System.out.println(graph.adj.get(0).get(0));
    }

    public static class Pair {
        int dest;
        int wt = 1;

        Pair(int dest, int wt) {
            this.dest = dest;
            this.wt = wt;
        }
    }
    public static class Helper {
        public static void topoSortHelper(int node, boolean vis[], Stack<Integer> st,
            ArrayList<ArrayList<Pair>> adj) {
            vis[node] = true;
            for (Pair curr : adj.get(node)) {
                int it = curr.dest;
                if (!vis[it]) {
                    topoSortHelper(it, vis, st, adj);
                }
            }
            st.push(node);
        }
        public static int dijkstra(ArrayList<ArrayList<Pair>> adj, int nodes, int src, int dest) {
            PriorityQueue<Pair> pq = new PriorityQueue<>(Comparator.comparingInt(a -> a.wt));
            int[] dist = new int[nodes];
            Arrays.fill(dist, Integer.MAX_VALUE);
            dist[src] = 0;
            pq.add(new Pair(src, 0));

            while (!pq.isEmpty()) {
                Pair curr = pq.poll();
                int u = curr.dest;

                if (u == dest) {
                    return dist[u];
                }

                for (Pair neighbor : adj.get(u)) {
                    int v = neighbor.dest;
                    int weight = neighbor.wt;

                    if (dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                        pq.add(new Pair(v, dist[v]));
                    }
                }
            }
            return dist[dest] == Integer.MAX_VALUE ? -1 : dist[dest];
        }

        public static int bellmanFord(ArrayList<ArrayList<Pair>> adj, int nodes, int src, int dest) {
            int[] dist = new int[nodes];
            Arrays.fill(dist, Integer.MAX_VALUE);
            dist[src] = 0;

            for (int i = 0; i < nodes - 1; i++) {
                for (int u = 0; u < nodes; u++) {
                    for (Pair neighbor : adj.get(u)) {
                        int v = neighbor.dest;
                        int weight = neighbor.wt;

                        if (dist[u] != Integer.MAX_VALUE && dist[u] + weight < dist[v]) {
                            dist[v] = dist[u] + weight;
                        }
                    }
                }
            }

            for (int u = 0; u < nodes; u++) {
                for (Pair neighbor : adj.get(u)) {
                    int v = neighbor.dest;
                    int weight = neighbor.wt;

                    if (dist[u] != Integer.MAX_VALUE && dist[u] + weight < dist[v]) {
                        System.out.println("Graph contains negative weight cycle");
                        return -1;
                    }
                }
            }

            return dist[dest] == Integer.MAX_VALUE ? -1 : dist[dest];
        }

        public static void dfsHelper(ArrayList<ArrayList<Pair>> adj, boolean[] vis, ArrayList<Integer> ans , int node) {
            vis[node] = true;
            ans.add(node);
            for(Pair curr : adj.get(node)) {
                int neigh = curr.dest;
                if(!vis[neigh]) {
                    dfsHelper(adj, vis, ans, neigh);
                }
            }
        }
        
        public static void bfsHelper(ArrayList<ArrayList<Pair>> adj, boolean[] vis, ArrayList<Integer> ans , int source) {
            Queue<Integer> q = new LinkedList<>();
            q.add(source);
            vis[source] = true;
            while(!q.isEmpty()) {
                int node = q.remove();
                ans.add(node);
                for(int i=0;i<adj.get(node).size();i++) {
                    Pair curr = adj.get(node).get(i);
                    int neigh = curr.dest;
                    if(!vis[neigh]) {
                        vis[neigh] = true;
                        q.add(neigh);
                    }
                }
            }
        }

        public static boolean isCycleHelper1(ArrayList<ArrayList<Pair>> adj, boolean[] vis, int node) {
            //  UNDIRECTED
            vis[node] = true;
            Queue<Pair> q = new LinkedList<>();
            q.add(new Pair(node, -1));

            while(!q.isEmpty()) {
                int curr = q.peek().dest;
                int parent = q.peek().wt;
                q.remove();

                for(Pair neigh : adj.get(curr)) {
                    int adjN = neigh.dest;
                    if(!vis[adjN]) {
                        vis[adjN] = true;
                        q.add(new Pair(adjN,curr));
                    }
                    else if(adjN != parent) {
                        return true;
                    }
                }
            }
            return false;
        }

        public static boolean isCycleHelper2(ArrayList<ArrayList<Pair>> adj, boolean[] vis,  boolean[] path, int node) {
            vis[node] = true;
            path[node] = true;

            for(Pair curr : adj.get(node)) {
                int neigh = curr.dest;
                if(!vis[neigh]) {
                    if(isCycleHelper2(adj, vis, path, neigh)) {
                        return true;
                    }
                }
                else if(path[neigh]) {
                    return true;
                }
            }
            path[node] = false;
            return false;
        }
    }
}
class Edge {
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
