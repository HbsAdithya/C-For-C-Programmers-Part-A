#ifndef EDGE_H_
#define EDGE_H_

#include "Node.h"

namespace Dijkstra
{
	/**
	 * The Edge ADT
	 *
	 * @author Michael Melachridis
	 */
	template <typename T>
	class Edge 
	{
	public:
		Edge() : x(NULL), y(NULL), weight(0.0) {}

		Edge(Node<T>* x, Node<T>* y, double weight = 0.0) : x(x), y(y), weight(weight) {}

		~Edge() {
			delete x;
			delete y;
		}

		inline bool operator==(const Edge<T>& other) { 
			return (other.getX() == this->getX() && other.getY() == this->getY()); 
		}
	
		inline Node<T>* getX() const {
			return x;
		}

		inline Node<T>* getY() const {
			return y;
		}

		inline double getWeight() const {
			return weight;
		}

		/**
		 * Prints the value of this node instance
		 */
		friend std::ostream& operator<< (std::ostream& os, const Edge<T>& edge) {
			std::stringstream ss;
			ss << "(" << edge.x << " " << "-" << " " << edge.y << ")";
			return (os << ss.str());
		}

	private:
		Node<T>* x;	// vertex X
		Node<T>* y; // vertex Y
		double	weight;
	};
} // namespace Dijkstra
#endif
