/*
* A main function for you to build and run your
* own tests with.
* This file is not part of the marking, so you
* can do anything you want here.
*/
#include <iostream>

#include "directed_graph_algorithms.cpp"

int main() {
    directed_graph<int> g;
    vertex<int> va = vertex<int>(0, 800);        //A
    vertex<int> vb = vertex<int>(1, 300);        //B
    vertex<int> vc = vertex<int>(2, 400);        //C
    vertex<int> vd = vertex<int>(3, 710);        //D
    vertex<int> ve = vertex<int>(4, 221);        //E
 
    g.add_vertex(va);
    g.add_vertex(vb);
    g.add_vertex(vc);
    g.add_vertex(vd);
    g.add_vertex(ve);

    g.add_edge(va.id, vb.id, 600); //A-B 6
    g.add_edge(va.id, vc.id, 900); //A-C 9
    g.add_edge(vb.id, ve.id, 3000); //B-E 3
    g.add_edge(vc.id, vd.id, 4000); //C-D 4
    g.add_edge(vd.id, va.id, 1); //D-A 1
    g.add_edge(vd.id, vc.id, 700); //D-C 7
    g.add_edge(vd.id, ve.id, 500); //D-E 5


   cout << "Cost from A = " << low_cost_delivery(g, 0) << endl;
    cout << "Topological Sort" << endl;
vector<vertex<int>> top_sort = topological_sort(g);
    for (auto vert : top_sort) {
        cout << "(" << vert.id << ", " << vert.weight << ")";
    }

    cout << endl;

    cout << "Strongly Connected Components" << endl;

    vector<vector<vertex<int>>> scc = strongly_connected_components(g);

    for (auto vector : scc) {
        for (auto vertex : vector) {
            cout << "(" << vertex.id << ", " << vertex.weight << ")";
        }
        cout << endl;
    }

    cout << "Shortest Path" << endl;
    vector<vertex<int>> short_path = shortest_path(g, 0, 4);
    for (auto vertex : short_path) {
            cout << "(" << vertex.id << ", " << vertex.weight << ")";
    }
    cout << endl;

  
}