//Directed_Graph.hpp Devloped by Oscar TIAN 13390657 using VSCode and CLion, indentation followed by C++ Standard.
#ifndef DIRECTED_GRAPH_H
#define DIRECTED_GRAPH_H

#include <bits/stdc++.h>
// included the standard libraries required by the assignment using bits/stdc++.h.

using namespace std; // the standard namespace are here just in case.

/*
	the vertex class
*/
template <typename T>
class vertex
{

public:
    int id;
    T weight;
    vertex() {}
    vertex(int x, T y) : id(x), weight(y) {}

    // add more functions here if you need to

    //id < a.id and id == a.id is used to compare two vertices using their indices.
    //reload for sort and find.
    bool operator<(const vertex<T> &a) const
    {
        return id < a.id;
    }
    bool operator==(const vertex<T> &a) const
    {
        return id == a.id;
    }
};

/*
	the graph class
*/
template <typename T>
class directed_graph
{

private:
    size_t edges_num;
    size_t vertices_num;
    vector<vertex<T>> vertices;
    vector<vector<T>> adj_matrix;

    //You will need to add some data members here
    //to actually represent the graph internally,
    //and keep track of whatever you need to.

public:
    directed_graph();  //A constructor for directed_graph. The graph should start empty.
    ~directed_graph(); //A destructor. Depending on how you do things, this may not be necessary.

    bool contains(const int &) const;              //Returns true if the graph contains the given vertex_id, false otherwise.
    bool adjacent(const int &, const int &) const; //Returns true if the first vertex is adjacent to the second, false otherwise.

    void add_vertex(const vertex<T> &);                 //Adds the passed in vertex to the graph (with no edges).
    void add_edge(const int &, const int &, const T &); //Adds a weighted edge from the first vertex to the second.

    void remove_vertex(const int &);            //Removes the given vertex. Should also clear any incident edges.
    void remove_edge(const int &, const int &); //Removes the edge between the two vertices, if it exists.

    size_t in_degree(const int &) const;  //Returns number of edges coming in to a vertex.
    size_t out_degree(const int &) const; //Returns the number of edges leaving a vertex.
    size_t degree(const int &) const;     //Returns the degree of the vertex (both in edges and out edges).

    size_t num_vertices() const; //Returns the total number of vertices in the graph.
    size_t num_edges() const;    //Returns the total number of edges in the graph.

    vector<vertex<T>> get_vertices();                           //Returns a vector containing all the vertices.
    vector<vertex<T>> get_neighbours(const int &);              //Returns a vector containing all the vertices reachable from the given vertex. The vertex is not considered a neighbour of itself.
    vector<vertex<T>> get_second_order_neighbours(const int &); // Returns a vector containing all the second_order_neighbours (i.e., neighbours of neighbours) of the given vertex.
                                                                // A vector cannot be considered a second_order_neighbour of itself.

    bool reachable(const int &, const int &) const; //Returns true if the second vertex is reachable from the first (can you follow a path of out-edges to get from the first to the second?). Returns false otherwise.
    bool contain_cycles() const;                    // Return true if the graph contains cycles (there is a path from any vertices directly/indirectly to itself), false otherwise.
    bool cycles_helper(int &, vector<T> &) const;

    vector<vertex<T>> depth_first(const int &) const; //Returns the vertices of the graph in the order they are visited in by a depth-first traversal starting at the given vertex.
    vector<vertex<T>> breadth_first(const int &);     //Returns the vertices of the graph in the order they are visisted in by a breadth-first traversal starting at the given vertex.

    directed_graph<T> out_tree(const int &); //Returns a spanning tree of the graph starting at the given vertex using the out-edges. This means every vertex in the tree is reachable from the root.

    vector<vertex<T>> pre_order_traversal(const int &, directed_graph<T> &);  // returns the vertices in the visiting order of a pre-order traversal of the minimum spanning tree starting at the given vertex.
    vector<vertex<T>> in_order_traversal(const int &, directed_graph<T> &);   // returns the vertices in the visiting order of an in-order traversal of the minimum spanning tree starting at the given vertex.
    vector<vertex<T>> post_order_traversal(const int &, directed_graph<T> &); // returns the vertices in ther visitig order of a post-order traversal of the minimum spanning tree starting at the given vertex.
    void pot(const int &, directed_graph<T> &, vector<vertex<T>> &);
    void iot(const int &, directed_graph<T> &, vector<vertex<T>> &);
    void tot(const int &, directed_graph<T> &, vector<vertex<T>> &);
    vector<vertex<T>> significance_sorting(); // Return a vector containing a sorted list of the vertices in descending order of their significance.
};

// Define all your methods down here (or move them up into the header, but be careful you don't double up). If you want to move this into another file, you can, but you should #include the file here.
// Although these are just the same names copied from above, you may find a few more clues in the full method headers.
// Note also that C++ is sensitive to the order you declare and define things in - you have to have it available before you use it.

template <typename T>
directed_graph<T>::directed_graph()
{
    edges_num = 0;    // number of edge list set to 0;
    vertices_num = 0; // number of vertice list set to 0;
}

template <typename T>
directed_graph<T>::~directed_graph()
{
    //unused desctructor for directed_graph.
}

template <typename T>
bool directed_graph<T>::contains(const int &u_id) const
{
    for (vertex<T> v : vertices)
    { // a loop throughout the vectices
        if (v.id == u_id)
        {                // if v.id exists return ture
            return true; // otherwise false
        }
    }
    return false;
}

template <typename T>
bool directed_graph<T>::adjacent(const int &u_id, const int &v_id) const
{
    if (contains(u_id) && contains(v_id))
    {
        if (u_id >= 0 && v_id >= 0 && u_id < vertices_num && v_id < vertices_num)
        {
            return adj_matrix[u_id][v_id] > 0; //return true if u is adjacent to v
        }
    }
    return false; // otherwise false
}

template <typename T>
void directed_graph<T>::add_vertex(const vertex<T> &u)
{
    vertices.push_back(u); //push vertex to the vector
    if (u.id + 1 >= vertices_num)
    {
        vertices_num = u.id + 1;
        adj_matrix.resize(vertices_num); // resize the vector
        for (unsigned i = 0; i < vertices_num; i++)
        {                                       //a loop throughout the vector and increse if met the criteria
            adj_matrix[i].resize(vertices_num); //resize the list in the graph
        }
    }
}

template <typename T>
void directed_graph<T>::add_edge(const int &u_id, const int &v_id, const T &weight)
{
    if (contains(u_id) && contains(v_id))
    { //check if the graph contains u or v
        adj_matrix[u_id][v_id] = weight;
        edges_num++; //add the edge
    }
}

template <typename T>
void directed_graph<T>::remove_vertex(const int &u_id)
{
    for (unsigned int i = 0; i < vertices.size(); i++)
    {
        if (vertices[i].id == u_id)
        { //if the vertices exist
            for (vertex<T> v : vertices)
            {                            // iterate through the vertices
                remove_edge(u_id, v.id); //remove the adjacent edge as required
                remove_edge(v.id, u_id); //remove the adjacent edge as required
            }
            vertices.erase(vertices.begin() + i); //remove vertice from the matrix
            break;
        }
    }
}

template <typename T>
void directed_graph<T>::remove_edge(const int &u_id, const int &v_id)
{
    if (u_id > 0 && v_id > 0 && u_id < vertices_num && v_id < vertices_num)
    {
        if (adj_matrix[u_id][v_id] > 0)
        {
            adj_matrix[u_id][v_id] = false; //find if the edge exists
            edges_num--;                    //remove the edge
        }
    }
}

template <typename T>
size_t directed_graph<T>::in_degree(const int &u_id) const
{
    int in_deg = 0; //set in_degree to 0.
    for (unsigned i = 0; i < adj_matrix.size(); ++i)
    {                            //loop through the matrix
        if (adj_matrix[i][u_id]) //if in degree exists in the matrix from the node
            in_deg++;            //increment the in_degree value by 1
    }
    return in_deg;
}

template <typename T>
size_t directed_graph<T>::out_degree(const int &u_id) const
{
    int out_deg = 0; //set out_degree to 0.
    for (unsigned i = 0; i < adj_matrix.size(); ++i)
    {                            //loop through the matrix
        if (adj_matrix[u_id][i]) //if out degree exists in the matrix from the node
            out_deg++;           //increment the out_degree value by 1
    }
    return out_deg;
}

template <typename T>
size_t directed_graph<T>::degree(const int &u_id) const
{
    int degree = in_degree(u_id) + out_degree(u_id);
    return degree; //combine in/out_degree value and return it
}

template <typename T>
size_t directed_graph<T>::num_vertices() const { return vertices.size(); } //return the size of vertices.

template <typename T>
size_t directed_graph<T>::num_edges() const { return edges_num; } // return the edges in the matrix.

template <typename T>
vector<vertex<T>> directed_graph<T>::get_vertices() { return vertices; } //return the vertices in the matrix.

template <typename T>
vector<vertex<T>> directed_graph<T>::get_neighbours(const int &u_id)
{
    vector<vertex<T>> neighbours; //a vector to store the neighbours.
    if (contains(u_id))
    { //check if the vertex exist.
        for (unsigned i = 0; i < adj_matrix.size(); i++)
        {
            if (adjacent(u_id, i))
            { //if the vertex is adjacent to the node.
                for (vertex<T> v : vertices)
                { // loop throught the vertices.
                    if (v.id == i)
                    {
                        neighbours.push_back(v); // push the vertex into the vector.
                        break;
                    }
                }
            }
        }
    }
    return neighbours; //return the result.
}

template <typename T>
vector<vertex<T>> directed_graph<T>::get_second_order_neighbours(const int &u_id)
{
    vector<vertex<T>> firstNeighbour = get_neighbours(u_id); //reference the first neighbour.
    vector<vertex<T>> SOneighbours;                          //a vector to store second order neighbours.
    for (unsigned int i = 0; i < firstNeighbour.size(); i++)
    {
        int v_id = firstNeighbour[i].id; //set v_id as the name for first neighbour's index.
        for (unsigned int j = 0; j < adj_matrix[v_id].size(); j++)
        {
            if (adj_matrix[v_id][j] > 0 && j != u_id && find(SOneighbours.begin(), SOneighbours.end(), vertices[j]) == SOneighbours.end())
            {
                SOneighbours.push_back(vertices[j]); //find what's included for the second neighbour and only push back the second neighbours.
            }
        }
    }
    sort(SOneighbours.begin(), SOneighbours.end()); //sort the order for the second neighbour. (e.g 4 3, to 3 4)
    return SOneighbours;                            //return the result.
}

template <typename T>
bool directed_graph<T>::reachable(const int &u_id, const int &v_id) const
{
    vector<vertex<T>> dfs_result = depth_first(u_id); //using the depth_first method for checking the vertexes.
    for (vertex<T> v : dfs_result)
    {
        if (v_id == v.id)
        { //Returns true if the second vertex is reachable from the first
            return true;
        }
    }
    return false;
}

template <typename T>
bool directed_graph<T>::cycles_helper(int &u_id, vector<T> &cycle) const
{
    cycle[u_id] = 1; //cycle u_id = 1 means the current vertex is being processed in function call stack.
    for (vertex<T> v : vertices)
    { // iterate through the adjacent vertices.
        int v_id = v.id;
        if (adj_matrix[u_id][v_id] > 0)
        {
            // if there is cycle[v_id]
            if (cycle[v_id] == 1)
                return true;
            //if v_id isn't processed and there is a back
            if (cycle[v_id] == 0 && cycles_helper(v_id, cycle))
                return true;
        }
    }
    //mark vertices as processed with 2.
    cycle[u_id] = 2;

    return false;
}

//continues with cycles_helper method, returns true if there is a cycle in the graph.
//this method is implemented by referencing the example of geeksforgeeks, detect cycle in directed graph using colours.
//https://www.geeksforgeeks.org/detect-cycle-direct-graph-using-colors/
template <typename T>
bool directed_graph<T>::contain_cycles() const
{
    {
        //initialize all vertices as 0;
        vector<T> cycle(vertices_num + 50);
        for (vertex<T> v : vertices)
            cycle[v.id] = 0;
        //DFS travel beginning with all vertices.
        for (vertex<T> v : vertices)
        {
            int v_id = v.id;
            if (cycle[v_id] == 0)
            {
                if (cycles_helper(v_id, cycle) == true)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

template <typename T>
vector<vertex<T>> directed_graph<T>::depth_first(const int &u_id) const
{
    vector<vertex<T>> dfs_list;
    stack<vertex<T>> dfs_stack;
    vector<bool> visited(vertices_num + 50); // allocate extra memory to avoid out of bound error
    for (vertex<T> v : vertices)
    {
        int v_id = v.id;
        visited[v_id] = false;
        if (v_id == u_id)
        {
            dfs_stack.push(v);
        }
    }
    while (!dfs_stack.empty())
    {
        vertex<T> v = dfs_stack.top(); //The next vertex is added to process
        dfs_stack.pop();               //Removes processed vertex from vector
        int v_id = v.id;
        if (!visited[v_id]) //check for not visited.
        {

            visited[v_id] = true;  //sets visited to true
            dfs_list.push_back(v); //Adds to the list if it has not been visited

            for (vertex<T> t : vertices) //Loops through outgoing edges
            {
                int t_id = t.id;
                if (adj_matrix[v_id][t_id] > 0 && !visited[t_id]) //Checks if it has been visited
                {

                    dfs_stack.push(t);
                }
            }
        }
    }
    return dfs_list; //returns dfs_order
}

template <typename T>
vector<vertex<T>> directed_graph<T>::breadth_first(const int &u_id)
{
    vector<vertex<T>> bfs_list;
    queue<vertex<T>> bfs_queue;
    vector<bool> visited(vertices_num + 50); // allocate extra memory to avoid out of bound error
    for (vertex<T> v : vertices)
    {
        int v_id = v.id;
        visited[v_id] = false;
        if (v_id == u_id)
        {
            bfs_queue.push(v);
        }
    }
    while (!bfs_queue.empty())
    {
        vertex<T> v = bfs_queue.front(); //The next vertex is added to process
        bfs_queue.pop();                 //Removes processed vertex from vector
        int v_id = v.id;
        if (!visited[v_id]) //Checks if the vertex has been looped and if it has, it is not checked
        {

            visited[v_id] = true;  //Sets visited
            bfs_list.push_back(v); //Adds to the list if it has not been visited

            for (vertex<T> t : vertices) //iterate vertices
            {
                int t_id = t.id;
                if (adj_matrix[v_id][t_id] > 0 && !visited[t_id]) //Checks if it has been visited
                {
                    bfs_queue.push(t);
                }
            }
        }
    }
    return bfs_list;
}

template <typename T>
directed_graph<T> directed_graph<T>::out_tree(const int &u_id)
{
    directed_graph<T> otree;
    queue<pair<pair<int, int>, int>> uQueue; //the first pair contains a source vertex and an outgoing vertex, with the weight for the edge.
    vector<bool> visited(vertices_num + 50); // stores visited vertices, extra memory allocation to prevent boundary error.
    for (vertex<T> v : vertices)             // interation of vertices
    {
        int v_id = v.id;
        visited[v_id] = false;
        if (u_id == v_id)
        {
            uQueue.push({{-1, u_id}, -1}); // -1, since the root doesn't have a parent vertex.
        }
    }

    while (!uQueue.empty())
    {
        pair<pair<int, int>, int> n = uQueue.front();
        uQueue.pop();
        int parent_node = n.first.first;
        int child_node = n.first.second;
        int weight_edge = n.second;
        if (!visited[child_node])
        {
            T weight = -1;
            for (vertex<T> v : vertices)
            {
                if (v.id == child_node)
                {
                    weight = v.weight;
                    break;
                }
            }
            otree.add_vertex(vertex<T>(child_node, weight));
            if (parent_node != -1)
                otree.add_edge(parent_node, child_node, weight_edge);

            for (vertex<T> v : vertices)
            {
                int v_id = v.id;
                if (adj_matrix[child_node][v_id] > 0)
                {
                    uQueue.push({{child_node, v_id}, adj_matrix[child_node][v_id]});
                }
            }
            visited[child_node] = true;
        }
    }
    return otree;
}

/* ---For all traversals, a support method is provided to check if the child node belongs to the left or right.
    By changing the order of each push back, it provides the main method the ability to travel by pre/in/out order.
    pre order eg. 1. ret.push_back(current_vertex);
    2. if (counter > 0) pot(left_child.id, mst, ret);
    3. if (counter > 1) pot(right_child.id, mst, ret);
    this checks from the root vertex then left/right.
    put 1 in between 2 3 gives in order as we check from the left.
    put 1 after 3 gives post order as we check from the right.

--- */
template <typename T>
vector<vertex<T>> directed_graph<T>::pre_order_traversal(const int &u_id, directed_graph<T> &mst)
{
    vector<vertex<T>> ret;
    ret.clear();
    pot(u_id, mst, ret);
    return ret;
}

template <typename T>
void directed_graph<T>::pot(const int &u_id, directed_graph<T> &mst, vector<vertex<T>> &ret)
{
    vertex<T> current_vertex;
    for (vertex<T> v : vertices)
    {
        if (v.id == u_id)
        {
            current_vertex = v;
            break;
        }
    }

    vertex<T> left_child, right_child;
    int counter = 0;
    for (vertex<T> v : vertices)
    {
        if (mst.adj_matrix[u_id][v.id] > 0)
        {
            if (counter == 0)
            {
                left_child = v;
            }
            else
            {
                right_child = v;
            }
            counter++;
        }
    }

    ret.push_back(current_vertex);
    if (counter > 0)
        pot(left_child.id, mst, ret);
    if (counter > 1)
        pot(right_child.id, mst, ret);
}

/* ---For all traversals, a support method is provided to check if the child node belongs to the left or right.--- */
template <typename T>
vector<vertex<T>> directed_graph<T>::in_order_traversal(const int &u_id, directed_graph<T> &mst)
{
    vector<vertex<T>> ret;
    ret.clear();
    iot(u_id, mst, ret);
    return ret;
}

template <typename T>
void directed_graph<T>::iot(const int &u_id, directed_graph<T> &mst, vector<vertex<T>> &ret)
{
    vertex<T> current_vertex;
    for (vertex<T> v : vertices)
    {
        if (v.id == u_id)
        {
            current_vertex = v;
            break;
        }
    }

    vertex<T> left_child, right_child;
    int counter = 0;
    for (vertex<T> v : vertices)
    {
        if (mst.adj_matrix[u_id][v.id] > 0)
        {
            if (counter == 0)
            {
                left_child = v;
            }
            else
            {
                right_child = v;
            }
            counter++;
        }
    }

    if (counter > 0)
        iot(left_child.id, mst, ret);
    ret.push_back(current_vertex);
    if (counter > 1)
        iot(right_child.id, mst, ret);
}

/* ---For all traversals, a support method is provided to check if the child node belongs to the left or right.--- */
template <typename T>
vector<vertex<T>> directed_graph<T>::post_order_traversal(const int &u_id, directed_graph<T> &mst)
{
    vector<vertex<T>> ret;
    ret.clear();
    tot(u_id, mst, ret);
    return ret;
}

template <typename T>
void directed_graph<T>::tot(const int &u_id, directed_graph<T> &mst, vector<vertex<T>> &ret)
{
    vertex<T> current_vertex;
    for (vertex<T> v : vertices)
    {
        if (v.id == u_id)
        {
            current_vertex = v;
            break;
        }
    }

    vertex<T> left_child, right_child;
    int counter = 0;
    for (vertex<T> v : vertices)
    {
        if (mst.adj_matrix[u_id][v.id] > 0)
        {
            if (counter == 0)
            {
                left_child = v;
            }
            else
            {
                right_child = v;
            }
            counter++;
        }
    }

    if (counter > 0)
        tot(left_child.id, mst, ret);
    if (counter > 1)
        tot(right_child.id, mst, ret);
    ret.push_back(current_vertex);
}

template <typename T>
vector<vertex<T>> directed_graph<T>::significance_sorting()
{
    //insertion sorting the graph by their weight.
    vector<vertex<T>> vertices = get_vertices(); //include all vertices.
    for (int i = 1; i < vertices.size(); i++)
    {
        vertex<T> v = vertices[i];
        int ssort = i - 1;
        while (ssort >= 0 && vertices[ssort].weight < v.weight)
        {
            vertices[ssort + 1] = vertices[ssort];
            ssort--;
        }
        vertices[ssort + 1] = v;
    }
    return vertices;
}

#endif
