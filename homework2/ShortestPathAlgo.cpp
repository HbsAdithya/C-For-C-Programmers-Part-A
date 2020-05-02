/**
 * ShortestPathAlgo.cpp
 *
 *  Created on: Dec 25, 2016
 *      Author: Michael Melachridis
 *       Email: michael.melachridis@gmail.com
 */

#include "ShortestPathAlgo.h"

#include <vector>
#include <iostream>
#include <algorithm>
#include <iomanip>

namespace Dijkstra
{
	ShortestPathAlgo::ShortestPathAlgo(Graph &graph) :
			graph(graph),
			distTo(std::vector<double>(graph.getV(), std::numeric_limits<double>::max())),
			edgeTo(std::vector<Edge<int>* >(graph.getV())),
			pq(graph.getV())
	{
	}

	const ShortestPathAlgo::listOfVertices ShortestPathAlgo::vertices() const {
		listOfVertices v(graph.getV());

		for (unsigned int i = 0; i < graph.getAdjacencyList().size(); ++i) {
			Graph::listOfEdgesConstItr it = graph.getAdjacencyList()[i].begin();

			while (it != graph.getAdjacencyList()[i].end()) {
				v.push_back((*it)->getX());
				++it;
			}
		}
		return v;
	}

	bool ShortestPathAlgo::hasPathTo(int v) {
		validateVertex(v);
		return distTo[v] < std::numeric_limits<double>::max();
	}

	const ShortestPathAlgo::listOfVertices ShortestPathAlgo::path(int src, int dest) {

		std::vector< std::list<Edge<int>* > > adj = graph.getAdjacencyList();

		validateVertex(src);

		pq.push(src, 0);
		distTo[src] = 0;

		while (!pq.isEmpty()) {

			int u = pq.top();
			pq.pop();

			Graph::listOfEdgesConstItr itr = adj[u].begin(); // neighbors
			while (itr != adj[u].end()) {

				// Get vertex label and weight of current adjacent
				int y = (*itr)->getY()->getValue();
				double w = (*itr)->getWeight();

				// relax
				if (distTo[y] > distTo[u] + w) {
					distTo[y] = distTo[u] + w;
					pq.push(y, distTo[y]);
					edgeTo[y] = (*itr);

					if (pq.contains(y)) {
						pq.changePriority(y, distTo[y]);
					} else {
						pq.push(y, distTo[y]);
					}
				}
				++itr;
			}
		}

		listOfVertices shortestPath; // the sequence of vertices representing shortest path u-v1-v2-…-vn-w
		if (hasPathTo(dest)) {
			for (Edge<int>* e : pathTo(dest)) {
				shortestPath.push_back(e->getX());
			}
		}

		return shortestPath;
	}

	double ShortestPathAlgo::pathSize(int x, int y) {
		validateVertex(x);
		validateVertex(y);

		return distTo[y];
	}

	void ShortestPathAlgo::printShortestPath(int u, int w) {
		validateVertex(u);
		validateVertex(w);

		std::stringstream ss;
		ss << std::setprecision(2);

		const listOfVertices sp = path(u, w);

		// Print shortest path
		ss << "Vertex   Distance from Source   Shortest Path" << std::endl;
		for (int t = u; t <= w; t++) {
			if (hasPathTo(t)) {
				ss << t << "\t\t" << distTo[t] << "\t\t";

				for (Edge<int>* e : pathTo(t)) {
					ss << e->getX()->getValue() << "->";
				}
				ss << t << std::endl;
			} else {
				ss << u << "\t\t" << t << "\t\tno path" << std::endl;
			}
		}

		ss << std::endl;

		ss << "path cost from " << u << " to " << w << ": " << pathSize(u, w) << std::endl;
		ss << "shortest path sequence from " << u << " to " << w << ": ";
		for (unsigned int v = 0; v < sp.size(); ++v)
			ss << sp[v]->getValue() << "->";
		ss << w << std::endl;;

		std::cout << ss.str();
	}

	void ShortestPathAlgo::validateVertex(int v) {
		int V = distTo.size();
		assert(v > 0 || v <= V);  // vertex | 0 | (V-1)
	}

	std::vector<Edge<int>*> ShortestPathAlgo::pathTo(int v) {
		validateVertex(v);

		if (!hasPathTo(v))
			return std::vector<Edge<int>*>();

		std::vector<Edge<int>*> path;

		for (auto e = edgeTo[v]; e != NULL; e = edgeTo[e->getX()->getValue()]) {
			path.push_back(e);
		}

		std::reverse(path.begin(), path.end());

		return path;
	}
}

