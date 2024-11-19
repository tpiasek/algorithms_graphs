#ifndef NODE_HPP_
#define NODE_HPP_


template <typename T>
class Node
{
  public:
	T data;
	Node* next;

	Node() : data(NULL), next(nullptr) {}
	Node(T data) : data(data), next(nullptr) {}
	Node(T data, Node* next) : data(data), next(next) {}
};


#endif /* NODE_HPP_ */
