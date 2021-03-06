// A_Star_algorithm.cpp : Defines the entry point for the console application.
//

// TODO: Convert variables to pointers, guarantee parent linkage works

#include "stdafx.h"
#include "C_AStar.h"

#include <memory>

int main( ) {
	// expensive diagonal map
	//S_Node node_graph[k_map_dimensions] = {
	//	S_Node{ 1, 0, 0, nullptr },
	//	S_Node{ 1, 0, 1, nullptr },
	//	S_Node{ 1, 1, 0, nullptr },
	//	S_Node{ 10, 1, 1, nullptr },

	//	S_Node{ 1, 0, 2, nullptr },
	//	S_Node{ 1, 2, 0, nullptr },
	//	S_Node{ 1, 2, 1, nullptr },
	//	S_Node{ 1, 1, 2, nullptr },

	//	S_Node{ 1, 0, 3, nullptr },
	//	S_Node{ 1, 3, 0, nullptr },
	//	S_Node{ 1, 3, 1, nullptr },
	//	S_Node{ 1, 1, 3, nullptr },

	//	S_Node{ 10, 2, 2, nullptr },
	//	S_Node{ 1, 2, 3, nullptr },
	//	S_Node{ 1, 3, 2, nullptr },
	//	S_Node{ 1, 3, 3, nullptr }
	//};

	//// evenly distributed map
	//S_Node node_graph[k_map_dimensions] = {
	//	S_Node{ 1, 0, 0, nullptr },
	//	S_Node{ 1, 0, 1, nullptr },
	//	S_Node{ 5, 1, 0, nullptr },
	//	S_Node{ 3, 1, 1, nullptr },

	//	S_Node{ 1, 0, 2, nullptr },
	//	S_Node{ 4, 2, 0, nullptr },
	//	S_Node{ 6, 2, 1, nullptr },
	//	S_Node{ 4, 1, 2, nullptr },

	//	S_Node{ 1, 0, 3, nullptr },
	//	S_Node{ 6, 3, 0, nullptr },
	//	S_Node{ 3, 3, 1, nullptr },
	//	S_Node{ 1, 1, 3, nullptr },

	//	S_Node{ 6, 2, 2, nullptr },
	//	S_Node{ 1, 2, 3, nullptr },
	//	S_Node{ 6, 3, 2, nullptr },
	//	S_Node{ 2, 3, 3, nullptr }
	//};

	//// low-cost map
	//S_Node node_graph[k_map_dimensions] = {
	//	S_Node{ 1, 0, 0, nullptr },
	//	S_Node{ 1, 0, 1, nullptr },
	//	S_Node{ 1, 1, 0, nullptr },
	//	S_Node{ 1, 1, 1, nullptr },

	//	S_Node{ 1, 0, 2, nullptr },
	//	S_Node{ 1, 2, 0, nullptr },
	//	S_Node{ 1, 2, 1, nullptr },
	//	S_Node{ 1, 1, 2, nullptr },

	//	S_Node{ 1, 0, 3, nullptr },
	//	S_Node{ 1, 3, 0, nullptr },
	//	S_Node{ 1, 3, 1, nullptr },
	//	S_Node{ 1, 1, 3, nullptr },

	//	S_Node{ 1, 2, 2, nullptr },
	//	S_Node{ 1, 2, 3, nullptr },
	//	S_Node{ 1, 3, 2, nullptr },
	//	S_Node{ 1, 3, 3, nullptr }
	//};

	/* force no diagonal to avoud going up and right */
	S_Node node_graph[k_map_dimensions] = {
		S_Node{ 1,  0, 0, nullptr },
		S_Node{ 10, 0, 1, nullptr },
		S_Node{ 1 , 1, 0, nullptr },
		S_Node{ 1,  1, 1, nullptr },

		S_Node{ 10, 0, 2, nullptr },
		S_Node{ 1, 2, 0, nullptr },
		S_Node{ 1, 2, 1, nullptr },
		S_Node{ 10, 1, 2, nullptr },

		S_Node{ 10, 0, 3, nullptr },
		S_Node{ 1, 3, 0, nullptr },
		S_Node{ 1, 3, 1, nullptr },
		S_Node{ 1, 1, 3, nullptr },

		S_Node{ 10, 2, 2, nullptr },
		S_Node{ 1, 2, 3, nullptr },
		S_Node{ 1, 3, 2, nullptr },
		S_Node{ 1, 3, 3, nullptr }
	};

	constexpr bool diagonal_movement = false;
	std::unique_ptr<C_AStar> a_star_pathfinding = std::make_unique<C_AStar>( diagonal_movement );

	constexpr int start_position = 0;
	constexpr int end_position = k_map_dimensions - 1;

	auto start_node = node_graph[start_position];
	auto end_node = node_graph[end_position];

	std::cout << "Diagonal movement enabled: " << (a_star_pathfinding->is_diagonal_enabled( ) ? "yes" : "no") << std::endl;
	std::cout << "start_node: " << start_node.x_pos << ", " << start_node.y_pos << std::endl;
	std::cout << "goal_node:  " << end_node.x_pos   << ", " << end_node.y_pos   << std::endl;
	std::cout << std::endl;

	if( !a_star_pathfinding->find_path( start_node, end_node, node_graph ) ) {
		std::cout << "No path found" << std::endl;
	} else {
		std::cout << "Path found!" << std::endl << std::endl << "Route taken: " << std::endl;
		a_star_pathfinding->print_path( );
		std::cout << std::endl;
	}

	start_node = node_graph[9];
	end_node = node_graph[4];

	a_star_pathfinding->set_diagonal_movement( true );

	std::cout << "Diagonal movement enabled: " << (a_star_pathfinding->is_diagonal_enabled( ) ? "yes" : "no") << std::endl;
	std::cout << "start_node: " << start_node.x_pos << ", " << start_node.y_pos << std::endl;
	std::cout << "goal_node:  " << end_node.x_pos << ", " << end_node.y_pos << std::endl;
	std::cout << std::endl;

	if( !a_star_pathfinding->find_path( start_node, end_node, node_graph ) ) {
		std::cout << "No path found" << std::endl;
	} else {
		std::cout << "Path found!" << std::endl << std::endl << "Route taken: " << std::endl;
		a_star_pathfinding->print_path( );
	}

	std::cin.get( );
	return 0;
}