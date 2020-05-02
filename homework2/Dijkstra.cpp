#include "Graph.h"
#include "PriorityQueue.h"
#include "ShortestPathAlgo.h"

using namespace Dijkstra;

int main(int argc, char** argv)
{
	std::cout << "-------------------------------------" << std::endl;
	std::cout << "Dijkstra shortest path algorithm demo" << std::endl;
	std::cout << "-------------------------------------" << std::endl << std::endl;

	Graph randomG;
	randomG.generateRandomGraph(0.2, 0, 50);
	std::cout << randomG << std::endl;

	ShortestPathAlgo shortestPath(randomG);
	shortestPath.printShortestPath(0, 49);

	std::cout << std::endl;
	std::cout << "-------------------------------------" << std::endl;
	std::cout << "              Example 2              " << std::endl;
	std::cout << "-------------------------------------" << std::endl;

	Graph graph(9);
	graph.addEdge(0, 1, 4);
	graph.addEdge(0, 7, 8);
	graph.addEdge(1, 2, 8);
	graph.addEdge(1, 7, 11);
	graph.addEdge(2, 3, 7);
	graph.addEdge(2, 8, 2);
	graph.addEdge(2, 5, 4);
	graph.addEdge(3, 4, 9);
	graph.addEdge(3, 5, 14);
	graph.addEdge(4, 5, 10);
	graph.addEdge(5, 6, 2);
	graph.addEdge(6, 7, 1);
	graph.addEdge(6, 8, 6);
	graph.addEdge(7, 8, 7);

	std::cout << graph << std::endl;
	ShortestPathAlgo sp(graph);
	sp.printShortestPath(0, 8);

	return 0;
}
