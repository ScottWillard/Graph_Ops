#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <stack>
#include <utility>
#include <climits>
#include <algorithm>
#include <functional> 
#include <limits>
#include <iostream>

class Graph {
    private:
        std::unordered_map<char, std::vector<std::pair<char, int>>> adjList;
        std::unordered_map<char, char> parent;
        std::unordered_map<char, int> rank;
    
    public:
        void add_vertex(char vertex) {
            if (adjList.find(vertex) == adjList.end()) {
                adjList[vertex] = std::vector<std::pair<char, int>>();
                parent[vertex] = vertex; // Initialize parent for the new vertex
                rank[vertex] = 0; // Initialize rank for the new vertex
            }
        }

        void add_edge(char v1, char v2, int weight) {
            adjList[v1].push_back(std::make_pair(v2, weight));
            adjList[v2].push_back(std::make_pair(v1, weight));
        }
            
        void BFS(char start_vertex){
            if(adjList.find(start_vertex) == adjList.end()){
                std::cout << start_vertex << " not in graph";
                return;
            }
            std::queue<char> q;
            std::unordered_set<char> visited;
            q.push(start_vertex);
            visited.insert(start_vertex);
            
            while(!q.empty()){
                char curr_vertex = q.front();
                q.pop();
                std::cout << curr_vertex << " ";
                for(auto neighbor : adjList[curr_vertex]){
                    if(visited.find(neighbor.first) == visited.end()){
                        q.push(neighbor.first);
                        visited.insert(neighbor.first);
                    }
                }
            }
            std::cout << std::endl;
        }
            
        void DFS(char start_vertex){
            if(adjList.find(start_vertex) == adjList.end()){
                std::cout << start_vertex << " not in graph";
                return;
            }
            std::stack<char> s;
            std::unordered_set<char> visited;
            s.push(start_vertex);
            visited.insert(start_vertex);
            
            while(!s.empty()){
                char curr_vertex = s.top();
                s.pop();
                std::cout << curr_vertex << " ";
                for(auto neighbor : adjList[curr_vertex]){
                    if(visited.find(neighbor.first) == visited.end()){
                        s.push(neighbor.first);
                        visited.insert(neighbor.first);
                    }
                }
            }
            std::cout << std::endl;
        }
        
    std::unordered_map<char, int> dijkstra(char start_vertex) {
        std::unordered_map<char, int> distances;
        std::unordered_set<char> visited;
        std::priority_queue<std::pair<int, char>, std::vector<std::pair<int, char>>, std::greater<std::pair<int, char>>> pq;

        for (const auto& vertex : adjList) {
            distances[vertex.first] = std::numeric_limits<int>::max();
        }
        distances[start_vertex] = 0;

        pq.push(std::make_pair(0, start_vertex));

        while (!pq.empty()) {
            char current_vertex = pq.top().second;
            pq.pop();

            if (visited.find(current_vertex) != visited.end()) {
                continue;
            }
            visited.insert(current_vertex);

            for (const auto& neighbor : adjList[current_vertex]) {
                char next_vertex = neighbor.first;
                int weight = neighbor.second;

                if (distances[next_vertex] > distances[current_vertex] + weight) {
                    distances[next_vertex] = distances[current_vertex] + weight;
                    pq.push(std::make_pair(distances[next_vertex], next_vertex));
                }
            }
        }
        return distances;
    }

    std::unordered_map<char, std::pair<char, int>> prim() {
        std::unordered_map<char, std::pair<char, int>> parentAndEdge;
        std::unordered_map<char, bool> inMST;

        for (auto const& vertex : adjList) {
            parentAndEdge[vertex.first] = {' ', INT_MAX}; // Initialize parents and edge weights
            inMST[vertex.first] = false;
        }

        char start_vertex = adjList.begin()->first;
        parentAndEdge[start_vertex] = {' ', INT_MAX}; // Initialize the start vertex parent as space

        std::priority_queue<std::pair<int, char>, std::vector<std::pair<int, char>>, std::greater<std::pair<int, char>>> pq;
        pq.push(std::make_pair(0, start_vertex));

        while (!pq.empty()) {
            char u = pq.top().second;
            pq.pop();

            inMST[u] = true;

            for (auto& neighbor : adjList[u]) {
                char v = neighbor.first;
                int weight = neighbor.second;

                if (!inMST[v] && weight < parentAndEdge[v].second) {
                    parentAndEdge[v] = {u, weight};
                    pq.push(std::make_pair(weight, v));
                }
            }
        }

        return parentAndEdge;
    }

    std::vector<std::pair<char, std::pair<char, int>>> kruskal() {
        std::vector<std::pair<char, std::pair<char, int>>> result;

        std::function<char(char)> findParent = [&](char vertex) -> char {
            if (parent[vertex] == vertex)
                return vertex;
            return findParent(parent[vertex]);
        };

    auto merge = [&](char v1, char v2) {
        char root1 = findParent(v1);
        char root2 = findParent(v2);
    
        if (root1 != root2) {
            if (rank[root1] < rank[root2]) {
                std::swap(root1, root2);
            }
            parent[root2] = root1;
            if (rank[root1] == rank[root2]) {
                rank[root1]++;
            }
        }
    };

        std::vector<std::pair<int, std::pair<char, char>>> edges;
        for (const auto& vertex : adjList) {
            char source = vertex.first;
            for (const auto& neighbor : vertex.second) {
                char dest = neighbor.first;
                int weight = neighbor.second;
                edges.push_back({weight, {source, dest}});
            }
        }
        std::sort(edges.begin(), edges.end());

        for (const auto& edge : edges) {
            char source = edge.second.first;
            char dest = edge.second.second;

            if (findParent(source) != findParent(dest)) {
                result.push_back({source, {dest, edge.first}});
                merge(source, dest);
            }
        }

        return result;
    }
            
        std::vector<char> DFS_connected_component(char start_vertex){
            std::vector<char> connected_component;
            
            if(adjList.find(start_vertex) == adjList.end()){
                std::cout << start_vertex << " not in graph";
                return connected_component;
            }
            
            std::stack<char> s;
            std::unordered_set<char> visited;
            s.push(start_vertex);
            visited.insert(start_vertex);
        
            while(!s.empty()){
                char curr_vertex = s.top();
                s.pop();
                connected_component.push_back(curr_vertex); // Add the current vertex to the connected component
                
                for(auto neighbor : adjList[curr_vertex]){
                    if(visited.find(neighbor.first) == visited.end()){
                        s.push(neighbor.first);
                        visited.insert(neighbor.first);
                    }
                }
            }
        
            return connected_component;
        }
        
        std::vector<char> largest_connected_component(){
            std::unordered_set<char> visited;
            std::vector<char> largest_component;
            
            for(auto const& vertex : adjList){
                if(visited.find(vertex.first) == visited.end()){
                    std::vector<char> current_component = DFS_connected_component(vertex.first);
                    if(current_component.size() > largest_component.size()){
                        largest_component = current_component;
                    }
                    // Update visited to include all vertices in the current component
                    visited.insert(current_component.begin(), current_component.end());
                }
            }
            
            return largest_component;
        }
};

int main(){
    // Create three different graphs
    Graph g1, g2, g3;

    // Adding vertices and edges for the first graph (4 vertices)
    for(char v = 'A'; v <= 'D'; ++v){
        g1.add_vertex(v);
    }
    g1.add_edge('A', 'B', 3);
    g1.add_edge('B', 'C', 5);
    g1.add_edge('C', 'D', 2);


    for(char v = 'A'; v <= 'F'; ++v){
        g2.add_vertex(v);
    }
    g2.add_edge('A', 'B', 4);
    g2.add_edge('B', 'C', 6);
    g2.add_edge('C', 'D', 3);
    g2.add_edge('D', 'E', 7);
    g2.add_edge('E', 'F', 1);


    for(char v = 'A'; v <= 'H'; ++v){
        g3.add_vertex(v);
    }
    g3.add_edge('A', 'B', 2);
    g3.add_edge('B', 'C', 8);

    // Find the number of vertices for each graph
    std::vector<char> largest_component_g1 = g1.largest_connected_component();
    int num_vertices_g1 = largest_component_g1.size();

    std::vector<char> largest_component_g2 = g2.largest_connected_component();
    int num_vertices_g2 = largest_component_g2.size();

    std::vector<char> largest_component_g3 = g3.largest_connected_component();
    int num_vertices_g3 = largest_component_g3.size();

    // Print the sizes of the graphs
    std::cout << "Graph 1 - Number of Vertices: " << num_vertices_g1 << std::endl;
    std::cout << "Graph 2 - Number of Vertices: " << num_vertices_g2 << std::endl;
    std::cout << "Graph 3 - Number of Vertices: " << num_vertices_g3 << std::endl;

    // Determine the largest graph based on the number of vertices
    if (num_vertices_g1 > num_vertices_g2 && num_vertices_g1 > num_vertices_g3) {
        std::cout << "Graph 1 has the largest number of vertices: " << num_vertices_g1 << std::endl;
    } else if (num_vertices_g2 > num_vertices_g1 && num_vertices_g2 > num_vertices_g3) {
        std::cout << "Graph 2 has the largest number of vertices: " << num_vertices_g2 << std::endl;
    } else if (num_vertices_g3 > num_vertices_g1 && num_vertices_g3 > num_vertices_g2) {
        std::cout << "Graph 3 has the largest number of vertices: " << num_vertices_g3 << std::endl;
    } else {
        std::cout << "There is more than one graph with the largest number of vertices." << std::endl;
    }

    // Test Dijkstra's algorithm
    std::cout << "Dijkstra's algorithm test:" << std::endl;
    std::unordered_map<char, int> shortest_paths = g1.dijkstra('A');
    for(auto const& path : shortest_paths){
        std::cout << "Shortest path from A to " << path.first << " is " << path.second << std::endl;
    }

    // Test Prim's algorithm
    std::cout << "\nPrim's algorithm test:" << std::endl;
    std::unordered_map<char, std::pair<char, int>> mst_prim = g2.prim();
    
    for (const auto& edge : mst_prim) {
        std::cout << "Edge between " << edge.first << " and its parent: " 
                  << edge.second.first << " with weight: " << edge.second.second << std::endl;
    }

    // Test Kruskal's algorithm
    std::cout << "\nKruskal's algorithm test:" << std::endl;
    std::vector<std::pair<char, std::pair<char, int>>> mst_kruskal = g3.kruskal();
    for(auto const& edge : mst_kruskal){
        std::cout << "Edge between " << edge.first << " and " << edge.second.first << " with weight: " << edge.second.second << std::endl;
    }

    return 0;
}
