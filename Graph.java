import java.util.*;

public class Graph {
    ArrayList<ArrayList<Pair>> adj;
    int[][] matrix;
    int nodes;
    
    public Graph() {
        this.adj = new ArrayList<>();
        this.nodes = 0;
    }
    
    public Graph(ArrayList<Edge> edges, int nodes, boolean isUndirected) {
        this.nodes = nodes;
        this.adj = new ArrayList<>(nodes);
        for(int i = 0; i < nodes; i++) {
            this.adj.add(new ArrayList<>());
        }
        if(isUndirected) {
            createUndirectedGraph(edges);
        } else {
            createDirectedGraph(edges);
        }
    }
    
    public void createDirectedGraph(ArrayList<Edge> edges) {
        edges.forEach(e -> adj.get(e.u).add(new Pair(e.v, e.wt)));
        nodes = adj.size();
    }
    
    public void createUndirectedGraph(ArrayList<Edge> edges) {
        edges.forEach(e -> {
            adj.get(e.u).add(new Pair(e.v, e.wt));
            adj.get(e.v).add(new Pair(e.u, e.wt));
        });
        nodes = adj.size();
    }
    
    public void add(int u, int v, boolean isUndirected) {
        ensureCapacity(Math.max(u, v) + 1);
        adj.get(u).add(new Pair(v, 1));
        if(isUndirected) {
            adj.get(v).add(new Pair(u, 1));
        }
    }
    
    public void add(int u, int v, int wt, boolean isUndirected) {
        ensureCapacity(Math.max(u, v) + 1);
        adj.get(u).add(new Pair(v, wt));
        if(isUndirected) {
            adj.get(v).add(new Pair(u, wt));
        }
    }
    
    private void ensureCapacity(int newSize) {
        if(newSize > nodes) {
            for(int i = nodes; i < newSize; i++) {
                adj.add(new ArrayList<>());
            }
            nodes = newSize;
        }
    }
    
    public void remove(int u, int v, boolean isUndirected) {
        if(u >= nodes || v >= nodes) return;
        
        adj.get(u).removeIf(pair -> pair.dest == v);
        if(isUndirected) {
            adj.get(v).removeIf(pair -> pair.dest == u);
        }
    }
    
    public int shortestPath(int src, int dest, boolean isNegativeWeighted, boolean isUndirected) {
        if(src == dest) return 0;
        if(isNegativeWeighted) {
            return Helper.bellmanFord(adj, nodes, src, dest);
        } else {
            return Helper.dijkstra(adj, nodes, src, dest);
        }
    }

    public boolean isCycle(boolean isUndirected) {
        boolean[] vis = new boolean[nodes];
        if(isUndirected) {
            for(int i = 0; i < nodes; i++) {
                if(!vis[i] && Helper.isCycleHelper1(adj, vis, i)) {
                    return true;
                }
            }
        } else {
            boolean[] path = new boolean[nodes];
            for(int i = 0; i < nodes; i++) {
                if(!vis[i] && Helper.isCycleHelper2(adj, vis, path, i)) {
                    return true;
                }
            }
        }
        return false;
    }

    public ArrayList<Integer> dfs(int source) {
        ArrayList<Integer> ans = new ArrayList<>();
        boolean[] vis = new boolean[nodes];
        Helper.dfsHelper(adj, vis, ans, source);
        return ans;
    }

    public ArrayList<Integer> bfs(int source) {
        ArrayList<Integer> ans = new ArrayList<>();
        boolean[] vis = new boolean[nodes];
        Helper.bfsHelper(adj, vis, ans, source);
        return ans;
    }

    public ArrayList<Integer> topoSort(boolean isUndirected) {
        if(isUndirected || isCycle(isUndirected)) {
            System.out.println("Topological sort not possible for undirected or cyclic graph");
            return new ArrayList<>();
        }
        ArrayList<Integer> ans = new ArrayList<>();
        boolean[] vis = new boolean[nodes];
        Stack<Integer> st = new Stack<>();
        for(int i = 0; i < nodes; i++) {
            if(!vis[i]) {
                Helper.topoSortHelper(i, vis, st, adj);
            }
        }
        while(!st.isEmpty()) {
            ans.add(st.pop());
        }
        return ans;
    }
    
    public List<List<Integer>> bridges() {
        int[] vis = new int[nodes];
        int[] tin = new int[nodes];
        int[] low = new int[nodes];
        List<List<Integer>> bridges = new ArrayList<>();
        for(int i = 0; i < nodes; i++) {
            if(vis[i] == 0) {
                Helper.dfs(i, -1, vis, adj, tin, low, bridges);
            }
        }
        return bridges;
    }

    public boolean find(int x) {
        return x >= 0 && x < nodes;
    }
    
    public ArrayList<ArrayList<Pair>> toAdjacencyList(int[][] matrix) {
        ArrayList<ArrayList<Pair>> adjList = new ArrayList<>();
        for(int i = 0; i < matrix.length; i++) {
            adjList.add(new ArrayList<>());
            for(int j = 0; j < matrix[i].length; j++) {
                if(matrix[i][j] != 0) {
                    adjList.get(i).add(new Pair(j, matrix[i][j]));
                }
            }
        }
        return adjList;
    }
    
    public int[][] toMatrix(ArrayList<ArrayList<Pair>> adj) {
        int[][] matrix = new int[nodes][nodes];
        for(int i = 0; i < nodes; i++) {
            for(Pair p : adj.get(i)) {
                matrix[i][p.dest] = p.wt;
            }
        }
        return matrix;
    }
    
    public int components() {
        boolean[] vis = new boolean[nodes];
        int count = 0;
        for(int i = 0; i < nodes; i++) {
            if(!vis[i]) {
                count++;
                Helper.bfsHelper(adj, vis, new ArrayList<>(), i);
            }
        }
        return count;
    }
    
    public List<Integer> dijkstraPath(int src, int dest) {
        return Helper.dijkstraPath(adj, nodes, src, dest);
    }
    
    public int primMST() {
        return Helper.primMST(adj, nodes);
    }
    
    public int kruskalMST() {
        return Helper.kruskalMST(adj, nodes);
    }
    
    public static void main(String[] args) {
        ArrayList<Edge> edges = new ArrayList<>();
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
        System.out.println("Components: " + graph.components());
        System.out.println("Has cycle: " + graph.isCycle(true));
        System.out.println("Bridges: " + graph.bridges());
    }
}

class Edge {
    int u, v, wt;
    
    public Edge(int u, int v) {
        this(u, v, 1);
    }
    
    public Edge(int u, int v, int wt) {
        this.u = u;
        this.v = v;
        this.wt = wt;
    }
}

class Pair {
    int dest, wt;
    
    public Pair(int dest, int wt) {
        this.dest = dest;
        this.wt = wt;
    }
    
    @Override
    public String toString() {
        return "(" + dest + ", " + wt + ")";
    }
}

class Helper {
    public static int timer = 1;
    
    public static void dfs(int node, int parent, int[] vis, ArrayList<ArrayList<Pair>> adj, 
                         int[] tin, int[] low, List<List<Integer>> bridges) {
        vis[node] = 1;
        tin[node] = low[node] = timer++;
        
        for(Pair pair : adj.get(node)) {
            int neighbor = pair.dest;
            if(neighbor == parent) continue;
            
            if(vis[neighbor] == 0) {
                dfs(neighbor, node, vis, adj, tin, low, bridges);
                low[node] = Math.min(low[node], low[neighbor]);
                
                if(low[neighbor] > tin[node]) {
                    bridges.add(Arrays.asList(node, neighbor));
                }
            } else {
                low[node] = Math.min(low[node], tin[neighbor]);
            }
        }
    }
    
    public static void topoSortHelper(int node, boolean[] vis, Stack<Integer> st, 
                                     ArrayList<ArrayList<Pair>> adj) {
        vis[node] = true;
        for(Pair curr : adj.get(node)) {
            if(!vis[curr.dest]) {
                topoSortHelper(curr.dest, vis, st, adj);
            }
        }
        st.push(node);
    }
    
    public static int dijkstra(ArrayList<ArrayList<Pair>> adj, int nodes, int src, int dest) {
        int[] dist = new int[nodes];
        Arrays.fill(dist, Integer.MAX_VALUE);
        dist[src] = 0;
        
        PriorityQueue<Pair> pq = new PriorityQueue<>(Comparator.comparingInt(a -> a.wt));
        pq.add(new Pair(src, 0));
        
        while(!pq.isEmpty()) {
            Pair curr = pq.poll();
            int u = curr.dest;
            
            if(u == dest) return dist[u];
            if(curr.wt > dist[u]) continue;
            
            for(Pair neighbor : adj.get(u)) {
                int v = neighbor.dest;
                int weight = neighbor.wt;
                
                if(dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.add(new Pair(v, dist[v]));
                }
            }
        }
        return dist[dest] == Integer.MAX_VALUE ? -1 : dist[dest];
    }
    
    public static List<Integer> dijkstraPath(ArrayList<ArrayList<Pair>> adj, int nodes, int src, int dest) {
        int[] dist = new int[nodes];
        int[] prev = new int[nodes];
        Arrays.fill(dist, Integer.MAX_VALUE);
        Arrays.fill(prev, -1);
        dist[src] = 0;
        
        PriorityQueue<Pair> pq = new PriorityQueue<>(Comparator.comparingInt(a -> a.wt));
        pq.add(new Pair(src, 0));
        
        while(!pq.isEmpty()) {
            Pair curr = pq.poll();
            int u = curr.dest;
            
            if(u == dest) break;
            if(curr.wt > dist[u]) continue;
            
            for(Pair neighbor : adj.get(u)) {
                int v = neighbor.dest;
                int weight = neighbor.wt;
                
                if(dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.add(new Pair(v, dist[v]));
                }
            }
        }
        
        if(prev[dest] == -1) return Collections.emptyList();
        
        LinkedList<Integer> path = new LinkedList<>();
        for(int at = dest; at != -1; at = prev[at]) {
            path.addFirst(at);
        }
        return path;
    }
    
    public static int bellmanFord(ArrayList<ArrayList<Pair>> adj, int nodes, int src, int dest) {
        int[] dist = new int[nodes];
        Arrays.fill(dist, Integer.MAX_VALUE);
        dist[src] = 0;
        
        // Relax all edges V-1 times
        for(int i = 0; i < nodes - 1; i++) {
            for(int u = 0; u < nodes; u++) {
                if(dist[u] == Integer.MAX_VALUE) continue;
                for(Pair edge : adj.get(u)) {
                    int v = edge.dest;
                    int weight = edge.wt;
                    if(dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                    }
                }
            }
        }
        
        // Check for negative cycles
        for(int u = 0; u < nodes; u++) {
            if(dist[u] == Integer.MAX_VALUE) continue;
            for(Pair edge : adj.get(u)) {
                int v = edge.dest;
                int weight = edge.wt;
                if(dist[u] + weight < dist[v]) {
                    System.out.println("Graph contains negative weight cycle");
                    return -1;
                }
            }
        }
        
        return dist[dest] == Integer.MAX_VALUE ? -1 : dist[dest];
    }
    
    public static void dfsHelper(ArrayList<ArrayList<Pair>> adj, boolean[] vis, 
                                ArrayList<Integer> ans, int node) {
        vis[node] = true;
        ans.add(node);
        for(Pair curr : adj.get(node)) {
            if(!vis[curr.dest]) {
                dfsHelper(adj, vis, ans, curr.dest);
            }
        }
    }
    
    public static void bfsHelper(ArrayList<ArrayList<Pair>> adj, boolean[] vis, 
                                ArrayList<Integer> ans, int source) {
        Queue<Integer> q = new LinkedList<>();
        q.add(source);
        vis[source] = true;
        
        while(!q.isEmpty()) {
            int node = q.poll();
            ans.add(node);
            for(Pair curr : adj.get(node)) {
                if(!vis[curr.dest]) {
                    vis[curr.dest] = true;
                    q.add(curr.dest);
                }
            }
        }
    }
    
    public static boolean isCycleHelper1(ArrayList<ArrayList<Pair>> adj, boolean[] vis, int node) {
        Queue<Pair> q = new LinkedList<>();
        q.add(new Pair(node, -1));
        vis[node] = true;
        
        while(!q.isEmpty()) {
            int curr = q.peek().dest;
            int parent = q.peek().wt;
            q.poll();
            
            for(Pair neighbor : adj.get(curr)) {
                int adjNode = neighbor.dest;
                if(!vis[adjNode]) {
                    vis[adjNode] = true;
                    q.add(new Pair(adjNode, curr));
                } else if(adjNode != parent) {
                    return true;
                }
            }
        }
        return false;
    }
    
    public static boolean isCycleHelper2(ArrayList<ArrayList<Pair>> adj, boolean[] vis, 
                                        boolean[] path, int node) {
        vis[node] = true;
        path[node] = true;
        
        for(Pair curr : adj.get(node)) {
            int neighbor = curr.dest;
            if(!vis[neighbor]) {
                if(isCycleHelper2(adj, vis, path, neighbor)) {
                    return true;
                }
            } else if(path[neighbor]) {
                return true;
            }
        }
        
        path[node] = false;
        return false;
    }
    
    public static int primMST(ArrayList<ArrayList<Pair>> adj, int nodes) {
        boolean[] inMST = new boolean[nodes];
        int[] key = new int[nodes];
        Arrays.fill(key, Integer.MAX_VALUE);
        key[0] = 0;
        int res = 0;
        
        PriorityQueue<Pair> pq = new PriorityQueue<>(Comparator.comparingInt(a -> a.wt));
        pq.add(new Pair(0, 0));
        
        while(!pq.isEmpty()) {
            Pair curr = pq.poll();
            int u = curr.dest;
            
            if(inMST[u]) continue;
            inMST[u] = true;
            res += curr.wt;
            
            for(Pair neighbor : adj.get(u)) {
                int v = neighbor.dest;
                int weight = neighbor.wt;
                if(!inMST[v] && weight < key[v]) {
                    key[v] = weight;
                    pq.add(new Pair(v, key[v]));
                }
            }
        }
        return res;
    }
    
    public static int kruskalMST(ArrayList<ArrayList<Pair>> adj, int nodes) {
        List<Edge> edges = new ArrayList<>();
        for(int u = 0; u < nodes; u++) {
            for(Pair p : adj.get(u)) {
                edges.add(new Edge(u, p.dest, p.wt));
            }
        }
        
        edges.sort(Comparator.comparingInt(e -> e.wt));
        DisjointSet ds = new DisjointSet(nodes);
        int res = 0;
        
        for(Edge e : edges) {
            int u = e.u;
            int v = e.v;
            if(ds.find(u) != ds.find(v)) {
                ds.union(u, v);
                res += e.wt;
            }
        }
        return res;
    }
}

class DisjointSet {
    List<Integer> rank, parent;
    
    public DisjointSet(int n) {
        rank = new ArrayList<>(Collections.nCopies(n + 1, 0));
        parent = new ArrayList<>();
        for(int i = 0; i <= n; i++) {
            parent.add(i);
        }
    }
    
    public int find(int node) {
        if(node == parent.get(node)) {
            return node;
        }
        int ulp = find(parent.get(node));
        parent.set(node, ulp);
        return parent.get(node);
    }
    
    public void union(int u, int v) {
        int ulp_u = find(u);
        int ulp_v = find(v);
        if(ulp_u == ulp_v) return;
        
        if(rank.get(ulp_u) < rank.get(ulp_v)) {
            parent.set(ulp_u, ulp_v);
        } else if(rank.get(ulp_u) > rank.get(ulp_v)) {
            parent.set(ulp_v, ulp_u);
        } else {
            parent.set(ulp_v, ulp_u);
            rank.set(ulp_u, rank.get(ulp_u) + 1);
        }
    }
}