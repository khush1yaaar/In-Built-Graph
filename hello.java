import java.util.ArrayList;

public class hello {
    public static void main(String[] args) {
        System.out.println(89+89);
        Graph graph = new Graph();
        graph.add(0,1,false);
        graph.add(2,0,false);
        graph.add(1,2,false);
        graph.add(2,3,false);
        graph.add(3,4,false);
        graph.add(4,5,false);
        //System.out.println(graph.shortestPath(0, 5, false, false));

        Graph graph2 = new Graph();
        //graph2.add(0, 1, 2, false);
        //System.out.println(graph2.adjW.get(0).get(0).dest);
        ArrayList<Edge> edges = new ArrayList<Edge>();
        graph2.add(0, 1,1,true);
        graph2.add(0, 2,2,true);
        graph2.add(1, 2,7,true);
        graph2.add(1, 3,2,true);
        //graph2.add(2, 4,5,true);
        graph2.add(3, 5,1,true);
        graph2.add(4, 5,2,true);
        graph2.add(5, 6,1,true);

        //Graph graph3 = new Graph(edges, 0, true, true);
        System.out.println(graph2.bfs(0));
    }
}
