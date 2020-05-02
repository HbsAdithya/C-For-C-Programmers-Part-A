#include "Graph.h"
#include "MST.h"

#include <iostream>
#include <string>

using namespace Algorithms;

int main(int argc, char** argv)
{
	std::cout << "-------------------------------------" << std::endl;
	std::cout << "Minimum Spanning Tree Algorithm DEMO " << std::endl;
	std::cout << "-------------------------------------" << std::endl << std::endl;

	Graph randomG;
	randomG.generateRandomGraph(0.2, 0, 50);
	std::cout << randomG << std::endl;


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

	std::cout << "-------------------------------------" << std::endl;
	std::cout << "              Example 3              " << std::endl;
	std::cout << "-------------------------------------" << std::endl;

	std::string sampleFile = "graph_data.txt";
	if (argc == 2) {
		sampleFile = argv[1];
	}
	Graph mstGraph(sampleFile);
	std::cout << mstGraph << std::endl;

	MST primMst(mstGraph);
	for (Edge<int>* e : primMst.edges()) {
		std::cout << e->getX()->getValue() << "-" << e->getY()->getValue() << " (" << e->getWeight() << ")" << std::endl;
	}
	std::cout << std::endl;
	std::cout << "Total cost: " << primMst.cost() << std::endl;

	return 0;
}
