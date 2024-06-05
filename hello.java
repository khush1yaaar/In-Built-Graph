public class hello {
    public static void main(String[] args) {
        System.out.println(89+89);
        Graph graph = new Graph();
        graph.add(0,1,true);
        graph.add(2,0,true);
        graph.add(1,2,true);
        graph.add(2,3,true);
        graph.add(3,4,true);
        graph.add(4,5,true);
        System.out.println(graph.isCycle(true));
    }
}
