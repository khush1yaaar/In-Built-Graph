import java.util.ArrayList;

public class hello {
    public static void main(String[] args) {
        System.out.println(89+89);
        // Graph graph = new Graph();
        // graph.add(0,1,false);
        // graph.add(2,0,false);
        // graph.add(1,2,false);
        // graph.add(2,3,false);
        // graph.add(3,4,false);
        // graph.add(4,5,false);
        //System.out.println(graph.shortestPath(0, 5, false, false));

        // Graph graph = new Graph();
        // graph.add(0, 1,1,true);
        // graph.add(0, 2,2,true);
        // graph.add(1, 2,7,true);
        // graph.add(1, 3,2,true);
        // graph.add(3, 5,1,true);
        // graph.add(4, 5,2,true);
        // graph.add(5, 6,1,true);

        // ArrayList<Edge> edges = new ArrayList<Edge>();
        // edges.add(new Edge(0, 1,1));
        // edges.add(new Edge(0, 2,4));
        // edges.add(new Edge(1, 2,5));
        // edges.add(new Edge(1, 3,7));
        // edges.add(new Edge(2, 4,3));
        // edges.add(new Edge(3, 5,4)); 
        // edges.add(new Edge(4, 5,6));
        // edges.add(new Edge(5, 6,1));
        // Graph graph = new Graph(edges, 7, true);
        // System.out.println(graph.bfs(0));

        ArrayList<Edge> edges = new ArrayList<Edge>();
        edges.add(new Edge(0, 1));
        edges.add(new Edge(0, 2));
        edges.add(new Edge(1, 2));
        edges.add(new Edge(1, 3));
        edges.add(new Edge(2, 4));
        edges.add(new Edge(3, 5));
        edges.add(new Edge(4, 5));
        edges.add(new Edge(5, 6));
        Graph graph = new Graph(edges, 7, false);

        

        System.out.println(graph.isCycle(false));
    }
}
