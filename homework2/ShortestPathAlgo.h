/**
 * ShortestPathAlgo.h
 *
 *  Created on: Dec 25, 2016
 *      Author: Michael Melachridis
 *       Email: michael.melachridis@gmail.com
 */

#ifndef SHORTESTPATHALGO_H_
#define SHORTESTPATHALGO_H_

#include "Graph.h"
#include "PriorityQueue.h"
#include <algorithm>

namespace Dijkstra
{
	/**
	 * This class implements the mechanics of Dijkstra’s algorithm
	 *
	 * @author Michael Melachridis
	 */
	class ShortestPathAlgo
	{
		typedef std::vector<Node<int>*> listOfVertices;

	public:

		ShortestPathAlgo(Graph &graph);
		~ShortestPathAlgo() {};

		/**
		 * List of all vertices in G(V,E)
		 * @return list of all vertices
		 */
		const listOfVertices vertices() const;

		/**
		 *
		 * Finds the shortest paths from src to all other vertices
		 *
		 * find shortest path between src-dest (i.e u-w)
		 * @param src	The source vertex
		 * @param dest 	The destination vertex
		 *
		 * @return The sequence of vertices representing shortest path u-v1-v2-…-vn-w.
		 */
		const listOfVertices path(int src, int dest);

		/**
		 * Returns the path cost associated with the shortest path.
		 *
		 * @param x Node
		 * @param y Node
		 * @return the path cost
		 */
		double pathSize(int x, int y);

		/**
		 * Prints the shortest path u-v1-v2-…-vn-w.
		 */
		void printShortestPath(int u, int w);

	private:

		/**
		 * Validate vertex and abort unless {@code 0 <= v < V}
		 */
		void validateVertex(int v);

		/**
		 * Returns true if there is a path from the source vertex {@code s} to vertex {@code v}.
		 *
		 * @param  v the destination vertex
		 * @return {@code true} if there is a path from the source vertex
		 *         {@code s} to vertex {@code v}; {@code false} otherwise
		 */
		bool hasPathTo(int v);

		/**
		 * Returns a shortest path from the source vertex {@code s} to vertex {@code v}.
		 *
		 * @param  v the destination vertex
		 * @return a shortest path from the source vertex {@code s} to vertex {@code v}
		 *         as an vector of edges, and {@code NULL} if no such path
		 */
		std::vector<Edge<int>*> pathTo(int v);

		Graph &graph;
		std::vector<double> distTo;        	// distTo[v] = distance  of shortest u->w path
		std::vector<Edge<int>* > edgeTo;   	// edgeTo[v] = last edge on shortest u->w path
		PriorityQueue<double> pq;			// priority queue of vertices
	};
}



#endif /* SHORTESTPATHALGO_H_ */
