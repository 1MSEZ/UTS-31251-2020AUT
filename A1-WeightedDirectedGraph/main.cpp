
#include "directed_graph.hpp"

int main() {
    directed_graph<int> g;

 
    
     
    //Vertex indexes start from 0, following visual representation from 
    //assessment description A=0, B=1, etc
    vertex<int> va = vertex<int>(0, 800);        //A
    vertex<int> vb = vertex<int>(1, 3000);        //B
    vertex<int> vc = vertex<int>(2, 400);        //C
    vertex<int> vd = vertex<int>(3, 710);        //D
    vertex<int> ve = vertex<int>(4, 221);        //E
 
    g.add_vertex(va);
    g.add_vertex(vb);
    g.add_vertex(vc);
    g.add_vertex(vd);
    g.add_vertex(ve);

   // g.print_matrix();

    g.add_edge(va.id, vb.id, 6); //A-B 6
    g.add_edge(va.id, vc.id, 9); //A-C 9
    g.add_edge(vb.id, ve.id, 3); //B-E 3
    g.add_edge(vc.id, vd.id, 4); //C-D 4
    g.add_edge(vd.id, va.id, 1); //D-A 1
    g.add_edge(vd.id, vc.id, 7); //D-C 7
    g.add_edge(vd.id, ve.id, 5); //D-E 5
    //g.remove_edge(vd.id, ve.id); //remove edge test. 
    //cout << "ve is removed" << endl;
    //g.remove_vertex(); //remove vertex test.
    cout << boolalpha;
    cout << "---adjacent test---" << endl;
    cout << "Is vertex va adjacent to vertex vb? " << g.adjacent(va.id, vb.id) << endl;
    cout << "Is vertex va adjacent to vertex ve? " << g.adjacent(va.id, ve.id) << endl;
    cout << "---add/remove, matrix test---" << endl;
    cout << "Printing with " << g.num_edges() << " edges and " << g.num_vertices() << " vertices:" << endl;
   

   // cout << "Printing with " << g.num_edges() << " edges and " << g.num_vertices()  << " vertices:" << endl;
  //  g.print_matrix(); 
    cout << "---degree test---" << endl;
    cout << "(A) should have in-degree of " << g.in_degree(0) << endl;
    cout << "(A) should have out-degree of " << g.out_degree(0) << endl;
    cout << "(A) should have total degree of " << g.degree(0) << endl;
    cout << "---neighbour test---" << endl;
   vector<vertex<int>> neighbours = g.get_neighbours(0);
    for (vertex<int> sb : neighbours) {
         cout << "a is neighbour with " << sb.id << endl;
    }
    
cout << "---2nd neighbour test---" << endl;
 vector<vertex<int>> secondNeighbour_list = g.get_second_order_neighbours(0);
    for (vertex<int> sb : secondNeighbour_list) {
         cout << "a is a second neighbour with " <<sb.id << endl;
    }

cout << "---dfs test---" << endl;
  vector<vertex<int>> dfs_list = g.depth_first(0);
   for (vertex<int> dl : dfs_list) {
         cout << "dfs staring from 0 is " <<dl.id << endl;
    }

cout << "---bfs test---" << endl;
    vector<vertex<int>> bfs_list = g.breadth_first(0);
   for (vertex<int> bl : bfs_list) {
         cout << "bfs staring from 0 is " <<bl.id << endl;
    }
cout << "---reachable test---" << endl;
cout << "is 0 reachable -> 1? " << g.reachable(0,1) << endl;
cout << "is 4 reachable -> 0? " << g.reachable(4,0) << endl;
cout << "---contain_cycles test---" << endl;
cout << "does the graph contain cycles? " << g.contain_cycles() << endl;

directed_graph<int> tree = g.out_tree(0);
cout << "---pre-order test---" << endl;
for (vertex<int> po : tree.pre_order_traversal(0, tree)) {
    cout << po.id << " ";
} 
cout << endl;

cout << "---in-order test---" << endl;
for (vertex<int> io : tree.in_order_traversal(0, tree)) {
    cout << io.id << " ";
}
cout << endl;

cout << "---post-order test---" << endl;
for (vertex<int> to : tree.post_order_traversal(0, tree)) {
    cout << to.id << " ";
}
cout << endl;

cout << "---ssort test(insert)---" << endl;
	vector<vertex<int>> significance = g.significance_sorting();
	for (vertex<int> ss : significance) {
	 	cout << "(" << ss.id << ", " << ss.weight << ") ";
	}
	cout << endl << " " << endl;


   // cin.get(); //Don't stop debugging */
}
