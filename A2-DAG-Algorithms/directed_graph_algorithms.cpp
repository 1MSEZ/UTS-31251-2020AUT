//Directed_Graph_algorithms.cpp Devloped by Oscar TIAN 13390657 using VSCode+mingw compiler, indentation followed by C++ Standard.

#include <bits/stdc++.h>
// included the standard libraries required by the assignment using bits/stdc++.h.

#include "directed_graph.hpp"

using namespace std;

class Compare
{ //overload for pair comparison in shortest_path.
public:
  bool operator()(pair<vertex<int>, int> const &a, pair<vertex<int>, int> const &b) const
  {
    return a.second > b.second;
  }
};

/*
 * Computes the shortest distance from u to v in graph g.
 * The shortest path corresponds to a sequence of vertices starting from u and ends at v,
 * which has the smallest total weight of edges among all possible paths from u to v.
 */
template <typename T>
vector<vertex<T>> shortest_path(directed_graph<T> g, int u_id, int v_id)
{
  int vertices_num = g.num_vertices(); // returns the num vertices in the graph.
  vector<vertex<T>> short_path;        //a vector storing the shortest path recorded.
  priority_queue<pair<vertex<T>, int>, vector<pair<vertex<T>, int>>, Compare> pq;
  vector<int> dist(vertices_num + 50);     //a vector for storing distance with extra allocated memory.
  vector<int> prev(vertices_num + 50);     //a vector for storing previous vertex with extra allocated memory.
  vector<bool> visited(vertices_num + 50); //a vector for storing visited vertex with extra allocated memory.
  for (vertex<T> v : g.get_vertices())
  {                          //iteration through vertices.
    dist[v.id] = 0x3f3f3f3f; //allocated a large value for distance.
    prev[v.id] = -1;         //check for which vertex it's coming from previously.
    visited[v.id] = false;   //set visited to false.
    if (v.id == u_id)
    {
      pq.push(make_pair(v, 0)); //add the starting point to the priority queue.
    }
  }
  dist[u_id] = 0;
  while (!pq.empty())
  { //if priorty queue isn't empty.
    pair<vertex<T>, int> t = pq.top();
    pq.pop();
    vertex<T> u = t.first;
    int cost = t.second;
    if (visited[u.id])
      continue;
    visited[u.id] = true;
    for (vertex<T> v : g.get_neighbours(u.id))
    {
      if (!visited[v.id] && dist[v.id] > dist[u.id] + g.get_weight(u.id, v.id))
      {
        dist[v.id] = dist[u.id] + g.get_weight(u.id, v.id);
        pq.push(make_pair(v, dist[v.id]));
        prev[v.id] = u.id;
        //                cout << u.id << " " << v.id << endl;
      }
    }
  }
  int t = v_id;
  while (t != -1)
  {
    for (vertex<T> v : g.get_vertices())
    {
      if (v.id == t)
      {
        short_path.push_back(v);
        break;
      }
    }
    t = prev[t];
  }
  reverse(short_path.begin(), short_path.end());
  //    for (auto i : dist) {
  //        cout << i << " ";
  //    }
  //    cout << endl;
  return short_path;
}

/*
 * Computes the strongly connected components of the graph.
 * A strongly connected component is a subset of the vertices
 * such that for every pair u, v of vertices in the subset,
 * v is reachable from u and u is reachable from v.
 */

template <typename T> 
vector<vector<vertex<T>>> strongly_connected_components(directed_graph<T> g)
{
   stack<vertex<T>> stk;
  vector<vector<vertex<T>>> stcc;
  vector<int> stackscc(g.num_vertices());
  vector<int> disc(g.num_vertices()); 
  vector<int> low(g.num_vertices()); 
 

   for (vertex<T> v : g.get_vertices())
  {
    disc[v.id] = 0;
    low[v.id] = 0;
    stackscc[v.id] = false;
  } 

  for (vertex<T> v : g.get_vertices())
  {
    if (!disc[v.id])
    {
      scc_util(g, v, disc, low, stackscc, stk, stcc);
    }
  }


  return stcc;
}

template <typename T>
void scc_util(directed_graph<T> g, vertex<T> u, vector<int> &disc, vector<int> &low, vector<int> &stackscc, stack<vertex<T>> &stk, vector<vector<vertex<T>>> &stcc)
{
  static int time = 0; 
  disc[u.id] = low[u.id] = ++time;
  stk.push(u);
  stackscc[u.id] = true;

  for (vertex<T> v : g.get_neighbours(u.id))
  {
    if (!disc[v.id])
    {
      scc_util(g, v, disc, low, stackscc, stk, stcc);
      low[u.id] = min(low[u.id], low[v.id]);
    }
    else if (stackscc[v.id] == true)
    {
      low[u.id] = min(low[u.id], disc[v.id]);
    }
  }


  if (low[u.id] == disc[u.id])
  {
    vector<vertex<T>> scc_only;
    while (!stk.empty())
    {
      vertex<T> top = stk.top();
      stk.pop();
      stackscc[top.id] = false;
      scc_only.push_back(top);
      
      if (top.id == u.id)
        break;
    }
 
    stcc.push_back(scc_only);
  }
} 

/*
 * Computes a topological ordering of the vertices.
 * For every vertex u in the order, and any of its
 * neighbours v, v appears later in the order than u.
 * You will be given a DAG as the argument.
 */
template <typename T>
void topoSortHelp(directed_graph<T> g, vertex<T> v, vector<bool> &visited, stack<vertex<T>> &topoStack)
{
  visited[v.id] = true;
  for (vertex<T> w : g.get_neighbours(v.id))
  {
    if (!visited[w.id])
    {
      topoSortHelp(g, w, visited, topoStack);
    }
  }
  topoStack.push(v);
}

template <typename T>
vector<vertex<T>> topological_sort(directed_graph<T> g)
{
  vector<vertex<T>> topoSorted;
  stack<vertex<T>> topoStack;
  vector<bool> visited(g.num_vertices() + 50);

  for (vertex<T> v : g.get_vertices())
  {
    visited[v.id] = false;
  }

  for (vertex<T> v : g.get_vertices())
  {
    if (!visited[v.id])
    {
      topoSortHelp(g, v, visited, topoStack);
    }
  }

  while (!topoStack.empty())
  {
    topoSorted.push_back(topoStack.top());
    topoStack.pop();
  }

  return topoSorted;
}

//Chu-Liu(Edmond)'s Algorithm for Low_Cost_Delivery 
//reference(please use google translate.):https://blog.csdn.net/txl199106/article/details/62045479
struct Edge {
    int from, to, cost;
};
const int MAXN = 12345;
const int MAXM = 12345;
const int INF = 0x3f3f3f3f;
Edge edge[MAXM];
int pre[MAXN];//store parent node
int vis[MAXN];//for marking purpose
int id[MAXN];//id[i] record node id
int in[MAXN];//in[i]record i's metrics
int zhuliu(int root, int n, int m) {
    int res = 0, u, v;
    while(1) {
        for(int i = 0; i < n; i++)
            in[i] = INF;//initialise
        for(int i = 0; i < m; i++) {
            Edge E = edge[i];
            if(E.from != E.to && E.cost < in[E.to]) {
                pre[E.to] = E.from;//record pre
                in[E.to] = E.cost;//update
            }
        }
        for(int i = 0; i < n; i++)
            if(i != root && in[i] == INF)
                return -1;//there are no mst
        //finding cycles
        int tn = 0;//checking for how many cycles currently contains
        memset(id, -1, sizeof(id));
        memset(vis, -1, sizeof(vis));
        in[root] = 0;//root
        for(int i = 0; i < n; i++) {
            res += in[i];//add
            v = i;
            while(vis[v] != i && id[v] == -1 && v != root) {
                vis[v] = i;//mark
                v = pre[v];//finding all previous items
            }
            if(v != root && id[v] == -1) { 
                for(int u = pre[v]; u != v; u = pre[u])
                    id[u] = tn;//record the corresponding id
                id[v] = tn++;
            }
        }
        if(tn == 0) break;//there is no cycle
        for(int i = 0; i < n; i++)
            if(id[i] == -1)
                id[i] = tn++;
        for(int i = 0; i < m; i++) {
            v = edge[i].to;
            edge[i].from = id[edge[i].from];
            edge[i].to = id[edge[i].to];
            if(edge[i].from != edge[i].to)
                edge[i].cost -= in[v];
        }
        n = tn;//record and repeat until there is no more cycle.
        root = id[root];
    }
    return res;
}


/*
 * Computes the lowest cost-per-person for delivery over the graph.
 * u is the source vertex, which send deliveries to all other vertices.
 * vertices denote cities; vertex weights denote cities' population;
 * edge weights denote the fixed delivery cost between cities, which is irrelevant to
 * the amount of goods being delivered.
 */
template <typename T>
T low_cost_delivery(directed_graph<T> g, int u_id) {
    int n = 0, m = 0;
    for(vertex<T> v : g.get_vertices()) {
        n++;
        for(vertex<T> w : g.get_vertices()) {
            if(g.get_weight(v.id, w.id) > 0) {
                edge[m].from = v.id;
                edge[m].to = w.id;
                edge[m].cost = g.get_weight(v.id, w.id);
                m++;
            }
        }
    }
//    printf("%d %d\n", n, m);
    T sumV = 0; //set a init sum for vertex weight.
    T sumE = zhuliu(u_id, n, m); //set a init sum for edge weight.
    //cout << sumE << endl;
    T Low_cost = 0; //sum of the equation of sumE / sumV
    for (vertex<T> v : g.get_vertices()) {
        //iterate through vertices to get vertex weight sum.
        if (v.id == u_id)
            continue;
        sumV += v.weight;
    }

    //cout << "test " << sumE << " " << sumV << endl;
    Low_cost = sumE / sumV; //get the lowest cost by using this equation.
    return Low_cost; //return the final cost.
}