/*     1----- 4   
 *   /        | \
 * 0          |   5----6
 *   \        | /
 *     2------3
 */
public class hello {
    public static void main(String[] args) {
        Graph graph = new Graph();
        graph.add(0, 1, true);
        graph.add(0, 2, true);
        graph.add(2, 3, true);
        graph.add(1, 4, true);
        graph.add(4, 3, true);
        graph.add(3, 5, true);
        graph.add(5, 4, true);
        graph.add(5, 6, true);

        //graph.remove(0, 1, true);
        System.out.println(graph.bfs(0));
    }
}
