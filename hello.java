public class hello {
    public static void main(String[] args) {
        System.out.println(89+89);
        Graph graph = new Graph();
        graph.add(0,1,true);
        graph.add(0,2,true);
        graph.add(1,2,true);
        graph.add(2,3,true);
        graph.add(5,4,true);
        graph.add(6,7,true);
        System.out.println(graph.components());
    }
}
