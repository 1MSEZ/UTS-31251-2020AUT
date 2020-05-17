#ifndef DIRECTED_GRAPH_H
#define DIRECTED_GRAPH_H

#include<iostream>
#include<string>
#include<vector>
// include more libraries here if you need to

using namespace std; // the standard namespace are here just in case.

/*
	the vertex class
*/
template <typename T>
class vertex {

public:
	int id;
	T weight;

	vertex(int x, T y) : id(x), weight(y) {}

	// add more functions here if you need to
};

/*
	the graph class
*/
template <typename T>
class directed_graph {

private:

	//You will need to add some data members here
	//to actually represent the graph internally,
	//and keep track of whatever you need to.

public:

	directed_graph(); //A constructor for directed_graph. The graph should start empty.
	~directed_graph(); //A destructor. Depending on how you do things, this may not be necessary.

	bool contains(const int&) const; //Returns true if the graph contains the given vertex_id, false otherwise.
	bool adjacent(const int&, const int&) const; //Returns true if the first vertex is adjacent to the second, false otherwise.

	void add_vertex(const vertex<T>&); //Adds the passed in vertex to the graph (with no edges).
	void add_edge(const int&, const int&, const T&); //Adds a weighted edge from the first vertex to the second.

	void remove_vertex(const int&); //Removes the given vertex. Should also clear any incident edges.
	void remove_edge(const int&, const int&); //Removes the edge between the two vertices, if it exists.

	size_t in_degree(const int&) const; //Returns number of edges coming in to a vertex.
	size_t out_degree(const int&) const; //Returns the number of edges leaving a vertex.
	size_t degree(const int&) const; //Returns the degree of the vertex (both in edges and out edges).

	size_t num_vertices() const; //Returns the total number of vertices in the graph.
	size_t num_edges() const; //Returns the total number of edges in the graph.

	vector<vertex<T>> get_vertices(); //Returns a vector containing all the vertices.
	vector<vertex<T>> get_neighbours(const int&); //Returns a vector containing all the vertices reachable from the given vertex. The vertex is not considered a neighbour of itself.
	vector<vertex<T>> get_second_order_neighbours(const int&); // Returns a vector containing all the second_order_neighbours (i.e., neighbours of neighbours) of the given vertex.
															  // A vector cannot be considered a second_order_neighbour of itself.
	bool reachable(const int&, const int&) const; //Returns true if the second vertex is reachable from the first (can you follow a path of out-edges to get from the first to the second?). Returns false otherwise.
	bool contain_cycles() const; // Return true if the graph contains cycles (there is a path from any vertices directly/indirectly to itself), false otherwise.

	vector<vertex<T>> depth_first(const int&); //Returns the vertices of the graph in the order they are visited in by a depth-first traversal starting at the given vertex.
	vector<vertex<T>> breadth_first(const int&); //Returns the vertices of the graph in the order they are visisted in by a breadth-first traversal starting at the given vertex.

	directed_graph<T> out_tree(const int&); //Returns a tree starting at the given vertex using the out-edges. This means every vertex in the tree is reachable from the root.

	vector<vertex<T>> pre_order_traversal(const int&, directed_graph<T>&); // returns the vertices in the visiting order of a pre-order traversal of the tree starting at the given vertex.
	vector<vertex<T>> in_order_traversal(const int&, directed_graph<T>&); // returns the vertices in the visiting order of an in-order traversal of the tree starting at the given vertex.
	vector<vertex<T>> post_order_traversal(const int&, directed_graph<T>&); // returns the vertices in ther visitig order of a post-order traversal of the tree starting at the given vertex.

	vector<vertex<T>> significance_sorting(); // Return a vector containing a sorted list of the vertices in descending order of their significance.

};

// Define all your methods down here (or move them up into the header, but be careful you don't double up). If you want to move this into another file, you can, but you should #include the file here.
// Although these are just the same names copied from above, you may find a few more clues in the full method headers.
// Note also that C++ is sensitive to the order you declare and define things in - you have to have it available before you use it.

template <typename T>
directed_graph<T>::directed_graph() {}

template <typename T>
directed_graph<T>::~directed_graph() {}

template <typename T>
bool directed_graph<T>::contains(const int& u_id) const { return false; }

template <typename T>
bool directed_graph<T>::adjacent(const int& u_id, const int& v_id) const { return false; }

template <typename T>
void directed_graph<T>::add_vertex(const vertex<T>& u) {}

template <typename T>
void directed_graph<T>::add_edge(const int& u_id, const int& v_id, const T& weight) {}

template <typename T>
void directed_graph<T>::remove_vertex(const int& u_id) {}

template <typename T>
void directed_graph<T>::remove_edge(const int& u_id, const int& v_id) {}

template <typename T>
size_t directed_graph<T>::in_degree(const int& u_id) const { return 0; }

template <typename T>
size_t directed_graph<T>::out_degree(const int& u_id) const { return 0; }

template <typename T>
size_t directed_graph<T>::degree(const int& u_id) const { return 0; }

template <typename T>
size_t directed_graph<T>::num_vertices() const { return 0; }

template <typename T>
size_t directed_graph<T>::num_edges() const { return 0; }

template <typename T>
vector<vertex<T>> directed_graph<T>::get_vertices() { return vector<vertex<T>>(); }

template <typename T>
vector<vertex<T>> directed_graph<T>::get_neighbours(const int& u_id) { return vector<vertex<T>>(); }

template <typename T>
vector<vertex<T>> directed_graph<T>::get_second_order_neighbours(const int& u_id) { return vector<vertex<T>>(); }

template <typename T>
bool directed_graph<T>::reachable(const int& u_id, const int& v_id) const { return false; }

template <typename T>
bool directed_graph<T>::contain_cycles() const { return false; }

template <typename T>
vector<vertex<T>> directed_graph<T>::depth_first(const int& u_id) { return vector<vertex<T>>(); }

template <typename T>
vector<vertex<T>> directed_graph<T>::breadth_first(const int& u_id) { return vector<vertex<T>>(); }

template <typename T>
directed_graph<T> directed_graph<T>::out_tree(const int& u_id) { return directed_graph<T>(); }

template <typename T>
vector<vertex<T>> directed_graph<T>::pre_order_traversal(const int& u_id, directed_graph<T>& mst) { return vector<vertex<T>>(); }

template <typename T>
vector<vertex<T>> directed_graph<T>::in_order_traversal(const int& u_id, directed_graph<T>& mst) { return vector<vertex<T>>(); }

template <typename T>
vector<vertex<T>> directed_graph<T>::post_order_traversal(const int& u_id, directed_graph<T>& mst) { return vector<vertex<T>>(); }

template <typename T>
vector<vertex<T>> directed_graph<T>::significance_sorting() { return vector<vertex<T>>(); }

#endif