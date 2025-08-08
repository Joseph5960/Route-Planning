#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
using namespace std;

struct Edge { int v; double w; };                  // 边：v=终点, w=权重
using Graph = vector<vector<Edge>>;

pair<vector<double>, vector<int>> dijkstra(int n, const Graph& g, int s) {
    const double INF = numeric_limits<double>::infinity();
    vector<double> dist(n, INF);
    vector<int> prev(n, -1);
    using State = pair<double,int>;                // (dist, node)
    priority_queue<State, vector<State>, greater<State>> pq;

    dist[s] = 0.0;
    pq.emplace(0.0, s);

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;                 // 旧状态
        for (auto [v, w] : g[u]) {
            if (dist[v] > dist[u] + w) {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.emplace(dist[v], v);
            }
        }
    }
    return {dist, prev};
}

// 回溯 s->t 路径
vector<int> reconstruct_path(int s, int t, const vector<int>& prev) {
    vector<int> path;
    for (int cur = t; cur != -1; cur = prev[cur]) path.push_back(cur);
    reverse(path.begin(), path.end());
    if (path.empty() || path.front() != s) path.clear();
    return path;
}

// 使用示例
int main_dijkstra_demo() {
    int n = 5;
    Graph g(n);
    auto add = [&](int u,int v,double w){ g[u].push_back({v,w}); };
    add(0,1,2); add(0,2,5); add(1,2,1); add(1,3,2); add(2,3,1); add(3,4,3);

    auto [dist, prev] = dijkstra(n, g, /*s=*/0);
    int t = 4;
    auto path = reconstruct_path(0, t, prev);

    cout << "dist[0->4] = " << dist[t] << "\npath: ";
    for (int x: path) cout << x << " ";
    cout << "\n";
    return 0;
}


