### Algorithms:

- **Dijkstra**: From a single source, find shortest paths to all nodes. Works on both directed and undirected graphs with non-negative edge weights. Greedy approach.
- **Floyd-Warshall**: Shortest path from every node as a source.
- **Bellman-Ford**: Same as Dijkstra, but works for negative weights. Works on both directed and undirected graphs with non-negative edge weights as well as directed graphs with negative edge weights as long as no negative cycles are present. Dynamic Programming approach.
- **Topological Sort**: Print the nodes with no incoming edges first.
- **MST (Minimum Spanning Tree)**: Connect all the nodes with minimum costs (n nodes // n-1 edges).
  - **Prim's Algorithm**: Build the MST by starting from any node and expanding the tree one edge at a time.
  - **Kruskal's Algorithm**: Build the MST by sorting all edges and adding them one by one, ensuring no cycles are formed. (using disjoint data structure to find common parent and decide whether it is a connected component and Union to get rank and connect them (Refer Disjoint and Union data structure))

### Kruskal's Algorithm - Minimum Spanning Tree(https://takeuforward.org/data-structure/kruskals-algorithm-minimum-spanning-tree-g-47/)

**Problem Statement**: Given a weighted, undirected, and connected graph of V vertices and E edges. The task is to find the sum of weights of the edges of the Minimum Spanning Tree.

**Example 1**:

Input Format: 
V = 5, edges = { {0, 1, 2}, {0, 3, 6}, {1, 2, 3}, {1, 3, 8}, {1, 4, 5}, {4, 2, 7}}

Result: 16
Explanation: The minimum spanning tree for the given graph is drawn below:
MST = {(0, 1), (0, 3), (1, 2), (1, 4)}

**Example 2**:

Input Format: 
V = 5, 
edges = { {0, 1, 2}, {0, 2, 1}, {1, 2, 1}, {2, 3, 2}, {3, 4, 1}, {4, 2, 2}}

Result: 5
Explanation: The minimum spanning tree is drawn below:
MST = {(0, 2), (1, 2), (2, 3), (3, 4)}

**Solution**:
In the previous article on the minimum spanning tree, we had already discussed that there are two ways to find the minimum spanning tree for a given weighted and undirected graph. Among those two algorithms, we have already discussed Prim’s algorithm. 

In this article, we will be discussing another algorithm, named Kruskal’s algorithm, that is also useful in finding the minimum spanning tree.

**Approach**: 
We will be implementing Kruskal’s algorithm using the Disjoint Set data structure that we have previously learned.

Now, we know Disjoint Set provides two methods named findUPar()(This function helps to find the ultimate parent of a particular node) and Union(This basically helps to add the edges between two nodes). To know more about these functionalities, do refer to the article on Disjoint Set.

The algorithm steps are as follows:

1. First, we need to extract the edge information (if not given already) from the given adjacency list in the format of (wt, u, v) where u is the current node, v is the adjacent node and wt is the weight of the edge between node u and v and we will store the tuples in an array.
2. Then the array must be sorted in the ascending order of the weights so that while iterating we can get the edges with the minimum weights first.
3. After that, we will iterate over the edge information, and for each tuple, we will apply the following operation:
   - First, we will take the two nodes u and v from the tuple and check if the ultimate parents of both nodes are the same or not using the findUPar() function provided by the Disjoint Set data structure.
   - If the ultimate parents are the same, we need not do anything to that edge as there already exists a path between the nodes and we will continue to the next tuple.
   - If the ultimate parents are different, we will add the weight of the edge to our final answer (i.e. mstWt variable used in the following code) and apply the union operation (i.e. either unionBySize(u, v) or unionByRank(u, v)) with the nodes u and v. The union operation is also provided by the Disjoint Set.
4. Finally, we will get our answer (in the mstWt variable as used in the following code) successfully.

**Code**:
```java
import java.io.*;
import java.util.*;

// User function Template for Java

class DisjointSet {
    List<Integer> rank = new ArrayList<>();
    List<Integer> parent = new ArrayList<>();
    List<Integer> size = new ArrayList<>();
    public DisjointSet(int n) {
        for (int i = 0; i <= n; i++) {
            rank.add(0);
            parent.add(i);
            size.add(1);
        }
    }

    public int findUPar(int node) {
        if (node == parent.get(node)) {
            return node;
        }
        int ulp = findUPar(parent.get(node));
        parent.set(node, ulp);
        return parent.get(node);
    }

    public void unionByRank(int u, int v) {
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if (ulp_u == ulp_v) return;
        if (rank.get(ulp_u) < rank.get(ulp_v)) {
            parent.set(ulp_u, ulp_v);
        } else if (rank.get(ulp_v) < rank.get(ulp_u)) {
            parent.set(ulp_v, ulp_u);
        } else {
            parent.set(ulp_v, ulp_u);
            int rankU = rank.get(ulp_u);
            rank.set(ulp_u, rankU + 1);
        }
    }

    public void unionBySize(int u, int v) {
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if (ulp_u == ulp_v) return;
        if (size.get(ulp_u) < size.get(ulp_v)) {
            parent.set(ulp_u, ulp_v);
            size.set(ulp_v, size.get(ulp_v) + size.get(ulp_u));
        } else {
            parent.set(ulp_v, ulp_u);
            size.set(ulp_u, size.get(ulp_u) + size.get(ulp_v));
        }
    }
}

class Edge implements Comparable<Edge> {
    int src, dest, weight;
    Edge(int _src, int _dest, int _wt) {
        this.src = _src; this.dest = _dest; this.weight = _wt;
    }
    // Comparator function used for
    // sorting edges based on their weight
    public int compareTo(Edge compareEdge) {
        return this.weight - compareEdge.weight;
    }
};

class Solution {
    // Function to find sum of weights of edges of the Minimum Spanning Tree.
    static int spanningTree(int V, ArrayList<ArrayList<ArrayList<Integer>>> adj) {
        List<Edge> edges = new ArrayList<Edge>();
        // O(N + E)
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < adj.get(i).size(); j++) {
                int adjNode = adj.get(i).get(j).get(0);
                int wt = adj.get(i).get(j).get(1);
                int node = i;
                Edge temp = new Edge(i, adjNode, wt);
                edges.add(temp);
            }
        }
        DisjointSet ds = new DisjointSet(V);
        // M log M
        Collections.sort(edges);
        int mstWt = 0;
        // M x 4 x alpha x 2
        for (int i = 0; i < edges.size(); i++) {
            int wt = edges.get(i).weight;
            int u = edges.get(i).src;
            int v = edges.get(i).dest;

            if (ds.findUPar(u) != ds.findUPar(v)) {
                mstWt += wt;
                ds.unionBySize(u, v);
            }
        }

        return mstWt;
    }
}

class Main {
    public static void main (String[] args) {
        int V = 5;
        ArrayList<ArrayList<ArrayList<Integer>>> adj = new ArrayList<ArrayList<ArrayList<Integer>>>();
        int[][] edges =  {{0, 1, 2}, {0, 2, 1}, {1, 2, 1}, {2, 3, 2}, {3, 4, 1}, {4, 2, 2}};

        for (int i = 0; i < V; i++) {
            adj.add(new ArrayList<ArrayList<Integer>>());
        }

        for (int i = 0; i < 6; i++) {
            int u = edges[i][0];
            int v = edges[i][1];
            int w = edges[i][2];

            ArrayList<Integer> tmp1 = new ArrayList<Integer>();
            ArrayList<Integer> tmp2 = new ArrayList<Integer>();
            tmp1.add(v);
            tmp1.add(w);

            tmp2.add(u);
            tmp2.add(w);

            adj.get(u).add(tmp1);
            adj.get(v).add(tmp2);
        }

        Solution obj = new Solution();
        int mstWt = obj.spanningTree(V, adj);
        System.out.println("The sum of all the edge weights: " + mstWt);
    }
}
```

**Output**: The sum of all the edge weights: 5

**Time Complexity**: O(N+E) + O(E logE) + O(E*4α*2) where N = no. of nodes and E = no. of edges. O(N+E) for extracting edge information from the adjacency list. O(E logE) for sorting the array consists of the edge tuples. Finally, we are using the disjoint set operations inside a loop. The loop will continue to E times. Inside that loop, there are two disjoint set operations like findUPar() and UnionBySize() each taking 4 and so it will result in 4*2. That is why the last term O(E*4*2) is added.

**Space Complexity**: O(N) + O(N) + O(E) where E = no. of edges and N = no. of nodes. O(E) space is taken by the array that we are using to store the edge information. And in the disjoint set data structure, we are using two N-sized arrays i.e. a parent and a size array (as we are using unionBySize() function otherwise, a rank array of the same size if unionByRank() is used) which result in the first two terms O(N).

### AdjacencyList:

#### Min Cost to Connect Points (Prim algorithm example)

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

### Disjoint Set | Union by Rank | Union by Size | Path Compression: G-46

[Disjoint Set | Union by Rank | Union by Size | Path Compression: G-46](https://takeuforward.org/data-structure/disjoint-set-union-by-rank-union-by-size-path-compression-g-46/)

**In this article, we will discuss the Disjoint Set data structure which is a very important topic in the entire graph series.**

**Question**: Given two components of an undirected graph, determine whether node 1 and node 5 are in the same component or not.

**Approach**: 
- Use either DFS or BFS traversal technique to determine if node 1 and node 5 are in the same component. This brute force approach has a time complexity of O(N+E) (N = no. of nodes, E = no. of edges).
- Using a Disjoint Set data structure, we can solve this problem in constant time.

**Dynamic Graph**:
- A dynamic graph refers to a graph that keeps changing its configuration. For example, adding edges one by one changes the structure of the graph at each step.

**Functionalities of Disjoint Set data structure**:
- Finding the parent for a particular node (findPar())
- Union (adds an edge between two nodes)
  - Union by rank
  - Union by size

**Union by rank**:
- Rank refers to the distance between the furthest leaf node and the current node.
- Ultimate parent refers to the topmost node or the root node.

**Algorithm**:
1. Initialize rank array with zero and parent array with the value of nodes.
2. For Union function, find the ultimate parent of two nodes and connect the smaller rank to the larger rank. If ranks are equal, connect any parent to the other and increase the rank by one.

**findPar() function**:
- Finds the ultimate parent for a particular node using path compression technique to reduce time complexity.

**Union by size**:
- Similar to Union by rank but uses size to compare components while connecting. Initialize size array with one.

**Code**:
```java
import java.io.*;
import java.util.*;

class DisjointSet {
    List<Integer> rank = new ArrayList<>();
    List<Integer> parent = new ArrayList<>();
    List<Integer> size = new ArrayList<>();
    public DisjointSet(int n) {
        for (int i = 0; i <= n; i++) {
            rank.add(0);
            parent.add(i);
            size.add(1);
        }
    }

    public int findUPar(int node) {
        if (node == parent.get(node)) {
            return node;
        }
        int ulp = findUPar(parent.get(node));
        parent.set(node, ulp);
        return parent.get(node);
    }

    public void unionByRank(int u, int v) {
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if (ulp_u == ulp_v) return;
        if (rank.get(ulp_u) < rank.get(ulp_v)) {
            parent.set(ulp_u, ulp_v);
        } else if (rank.get(ulp_v) < rank.get(ulp_u)) {
            parent.set(ulp_v, ulp_u);
        } else {
            parent.set(ulp_v, ulp_u);
            int rankU = rank.get(ulp_u);
            rank.set(ulp_u, rankU + 1);
        }
    }

    public void unionBySize(int u, int v) {
        int ulp_u = findUPar(u);
        int ulp_v = findUPar(v);
        if (ulp_u == ulp_v) return;
        if (size.get(ulp_u) < size.get(ulp_v)) {
            parent.set(ulp_u, ulp_v);
            size.set(ulp_v, size.get(ulp_v) + size.get(ulp_u));
        } else {
            parent.set(ulp_v, ulp_u);
            size.set(ulp_u, size.get(ulp_u) + size.get(ulp_v));
        }
    }
}

class Main {
    public static void main (String[] args) {
        DisjointSet ds = new DisjointSet(7);
        ds.unionByRank(1, 2);
        ds.unionByRank(2, 3);
        ds.unionByRank(4, 5);
        ds.unionByRank(6, 7);
        ds.unionByRank(5, 6);

        // if 3 and 7 same or not
        if (ds.findUPar(3) == ds.findUPar(7)) {
            System.out.println("Same");
        } else
            System.out.println("Not Same");

        ds.unionByRank(3, 7);
        if (ds.findUPar(3) == ds.findUPar(7)) {
            System.out.println("Same");
        } else
            System.out.println("Not Same");
    }
}
```

**Output**:
```
Not Same
Same
```

**Time Complexity**: The time complexity is O(4) which is very small and close to 1. So, we can consider 4 as a constant.