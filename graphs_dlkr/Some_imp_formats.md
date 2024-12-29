### AdjacencyList:

#### Min Cost to Connect Points

You are given a 2-D integer array `points`, where `points[i] = [xi, yi]`. Each `points[i]` represents a distinct point on a 2-D plane.

The cost of connecting two points `[xi, yi]` and `[xj, yj]` is the Manhattan distance between the two points, i.e. `|xi - xj| + |yi - yj|`.

Return the minimum cost to connect all points together, such that there exists exactly one path between each pair of points.

```java
// Example to make adjacencyList
class Solution {
    public int weight(int i1, int j1, int i2, int j2) {
        return Math.abs(i1 - i2) + Math.abs(j1 - j2);
    }

    public int minCostConnectPoints(int[][] points) {
        int n = points.length;
        Map<Integer, List<int[]>> adjList = new HashMap<>(); // Correct data structure

        // Initialize the adjacency list for each point
        for (int i = 0; i < n; i++) {
            adjList.put(i, new ArrayList<>());
        }

        // Build the adjacency list
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                int weight = weight(points[i][0], points[i][1], points[j][0], points[j][1]);
                adjList.get(i).add(new int[]{j, weight}); // Add edge from i to j
                adjList.get(j).add(new int[]{i, weight}); // Add edge from j to i (undirected graph)
            }
        }

        PriorityQueue<int[]> minHeap = new PriorityQueue<>(Comparator.comparingInt(a -> a[0]));
        minHeap.offer(new int[]{0, 0});
        boolean[] vis = new boolean[n];

        for (int i = 0; i < n; i++) {
            vis[i] = false;
        }

        int sum = 0;
        while (!minHeap.isEmpty()) {
            int[] temp = minHeap.poll();

            if (vis[temp[1]]) continue;
            vis[temp[1]] = true;
            sum += temp[0];
            for (int[] ed : adjList.get(temp[1])) {
                if (!vis[ed[0]]) {
                    minHeap.offer(new int[]{ed[1], ed[0]});
                }
            }
        }

        return sum;
    }
}
```