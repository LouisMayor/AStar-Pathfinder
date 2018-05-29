#pragma once

/*******************************
*******************************/

#include <queue>
#include <list>
#include <assert.h>
#include <iostream>
#include <math.h>
#include "Node.h"

constexpr int k_map_width = 4;
constexpr int k_map_dimensions = k_map_width * k_map_width;

// given location x,y. find node on 'node_graph' with given value
// return data associated with found node
static S_Node* fn_find_node( S_Node& par_node, S_Node par_node_graph[k_map_dimensions] ) {
	for( int i = 0; i < k_map_dimensions; ++i ) {
		auto current_node = &par_node_graph[i];
		if( current_node->x_pos == par_node.x_pos && current_node->y_pos == par_node.y_pos ) {
			return current_node;
		}
	}
	return nullptr;
}

static inline bool fn_compare_node_f( const S_Node& x, const S_Node& y ) {
	return (x.f < y.f);
}

static inline bool fn_is_end_node( const S_Node& par_current_node, const S_Node& par_end_node ) {
	return (par_current_node.x_pos == par_end_node.x_pos && par_current_node.y_pos == par_end_node.y_pos);
}

static inline bool fn_is_valid_node( const S_Node& par_current_node ) {
	return (par_current_node.f > -1);
}

static inline int fn_manhattan_distance( const S_Node& par_current_node, const S_Node& par_end_node ) {
	return (abs( par_current_node.x_pos - par_end_node.x_pos ) + abs( par_current_node.y_pos - par_end_node.y_pos ));
}

static bool fn_is_on_list( const S_Node& par_current_node, const std::list<S_Node>& par_list ) {
	for( auto& node : par_list ) {
		if( node.x_pos == par_current_node.x_pos && node.y_pos == par_current_node.y_pos ) {
			return true;
		}
	}
	return false;
}

enum class e_sucessor_directions {
	north,
	north_east,
	east,
	south_east,
	south,
	south_west,
	west,
	north_west,
	size
};

class C_AStar {
public:
	/*******************************
		Constructors & Destructors
	*******************************/
	C_AStar( ) {
		m_diagonal_movement = false;
	}

	C_AStar( bool allow_diagonal_movement = false ) {
		m_diagonal_movement = allow_diagonal_movement;
	}

	~C_AStar( ) {

	}

	/*******************************
		Public: Member Functions
	*******************************/
	void inline set_diagonal_movement( bool allow_diagonal_movement ) {
		m_diagonal_movement = allow_diagonal_movement;
	}

	constexpr bool inline is_diagonal_enabled( ) {
		return m_diagonal_movement;
	}

	void print_path( ) {
		for( auto& i : m_path ) {
			std::cout << "Node coords: " << i.x_pos << ", " << i.y_pos << std::endl;
		}
	}

	// pass current node, i.e. starting node. and the final destination
	bool find_path( S_Node& par_start_node, const S_Node& par_end_node, S_Node par_node_graph[k_map_dimensions] ) {
		// clear open/close lists, and recorded path (incase of new search)		
		m_close_list.clear( );
		m_open_list.clear( );
		m_path.clear( );
		// append starting node to our open list
		par_start_node.h = fn_manhattan_distance( par_start_node, par_end_node );
		par_start_node.f = par_start_node.g + par_start_node.h;
		m_open_list.push_back( par_start_node );

		while( !m_open_list.empty( ) ) {
			// sort nodes, want to find node with the lowest f
			m_open_list.sort( fn_compare_node_f );
			// aquire node with lowest f from the front of the list
			auto current_node = m_open_list.front( );
			// remove 'current_node' from 'open_list'
			m_open_list.pop_front( );
			// add to recorded path vector
			m_path.push_back( current_node );
#ifdef _DEBUG
			std::cout << "Node Coords: " << current_node.x_pos << " " << current_node.y_pos << std::endl;
			std::cout << "Node G: " << current_node.g << std::endl;
			std::cout << "Node H: " << current_node.h << std::endl;
			std::cout << "Node F: " << current_node.f << std::endl;
#endif
			// generate 8 potential successor nodes
			S_Node successor_node_graph[static_cast< int >(e_sucessor_directions::size)];
			compute_successor_nodes( current_node, successor_node_graph, par_node_graph );

			// check each successor node on 'successor_node_graph'
			for( auto& successor_node : successor_node_graph ) {
				// check if node actually exists, and not off the grid, etc...
				if( fn_is_valid_node( successor_node ) ) {
					// check if node is the end destination
					if( fn_is_end_node( successor_node, par_end_node ) ) {
#ifdef _DEBUG
						std::cout << std::endl;
						std::cout << "Node Coords: " << successor_node.x_pos << " " << successor_node.y_pos << std::endl;
						std::cout << "Node G: " << successor_node.g << std::endl;
						std::cout << "Node H: " << successor_node.h << std::endl;
						std::cout << "Node F: " << successor_node.f << std::endl;
						std::cout << std::endl;
#endif
						m_path.push_back( successor_node );
						return true;
					}

					// check if successor node is not on the 'm_close_list'
					if( !fn_is_on_list( successor_node, m_close_list ) ) {
						// check if successor node is not on 'm_open_list'
						if( !fn_is_on_list( successor_node, m_open_list ) ) {
							// compute node costs
							successor_node.g += 1;
							successor_node.h = fn_manhattan_distance( successor_node, par_end_node );
							successor_node.f = successor_node.g + successor_node.h;
							// add successor node onto 'm_open_list'
							m_open_list.push_back( successor_node );
						}
					}
				}
			}

#ifdef _DEBUG
			std::cout << std::endl;
			std::cout << "successor node list: " << std::endl;
			for( auto& i : successor_node_graph ) {
				static int s_counter = 0;
				std::string valid_node = (fn_is_valid_node( i ) ? "valid" : "invalid");
				std::cout << "Node: " << s_counter++ << " ( " << i.x_pos << ", " << i.y_pos << " )" << " g value: " << i.g << " " << " h value: " << i.h << " f value: " << i.f << " " << "node status: " << valid_node.c_str( ) << std::endl;
			}
			std::cout << std::endl;
#endif
			m_close_list.push_back( current_node );
		}
		return false;
	}


	/*******************************
		Private: Member Functions
	*******************************/
private:
	// finds the 4/8 successor nodes for 'current_node', and places them into 'successor_node_graph'
	void compute_successor_nodes( S_Node& current_node, S_Node successor_node_graph[static_cast< int >(e_sucessor_directions::size)], S_Node node_graph[k_map_dimensions] ) {
		S_Node* tmp_node = nullptr;

		// north
		S_Node  north_node = { 0, current_node.x_pos, current_node.y_pos + 1, nullptr };
		tmp_node = fn_find_node( north_node, node_graph );

		if( tmp_node != nullptr ) {
			successor_node_graph[static_cast< int >(e_sucessor_directions::north)].set_node( *tmp_node );
		}

		// east
		S_Node  east_node = { 0, current_node.x_pos + 1, current_node.y_pos, nullptr };
		tmp_node = fn_find_node( east_node, node_graph );

		if( tmp_node != nullptr ) {
			successor_node_graph[static_cast< int >(e_sucessor_directions::east)].set_node( *tmp_node );
		}

		// south
		S_Node  south_node = { 0, current_node.x_pos, current_node.y_pos - 1, nullptr };
		tmp_node = fn_find_node( south_node, node_graph );

		if( tmp_node != nullptr ) {
			successor_node_graph[static_cast< int >(e_sucessor_directions::south)].set_node( *tmp_node );
		}

		// west
		S_Node  west_node = { 0, current_node.x_pos - 1, current_node.y_pos, nullptr };
		tmp_node = fn_find_node( west_node, node_graph );

		if( tmp_node != nullptr ) {
			successor_node_graph[static_cast< int >(e_sucessor_directions::west)].set_node( *tmp_node );
		}

		// allow additional 4 nodes to be added
		if( m_diagonal_movement ) {
			// north_east
			S_Node  north_east_node = { 0, current_node.x_pos + 1, current_node.y_pos + 1, nullptr };
			tmp_node = fn_find_node( north_east_node, node_graph );

			if( tmp_node != nullptr ) {
				successor_node_graph[static_cast< int >(e_sucessor_directions::north_east)].set_node( *tmp_node );
			}

			// south_east
			S_Node  south_east_node = { 0, current_node.x_pos + 1, current_node.y_pos - 1, nullptr };
			tmp_node = fn_find_node( south_east_node, node_graph );

			if( tmp_node != nullptr ) {
				successor_node_graph[static_cast< int >(e_sucessor_directions::south_east)].set_node( *tmp_node );
			}

			// south_west
			S_Node  south_west_node = { 0, current_node.x_pos - 1, current_node.y_pos - 1, nullptr };
			tmp_node = fn_find_node( south_west_node, node_graph );

			if( tmp_node != nullptr ) {
				successor_node_graph[static_cast< int >(e_sucessor_directions::south_west)].set_node( *tmp_node );
			}

			// north_west
			S_Node  north_west_node = { 0, current_node.x_pos - 1, current_node.y_pos + 1, nullptr };
			tmp_node = fn_find_node( north_west_node, node_graph );

			if( tmp_node != nullptr ) {
				successor_node_graph[static_cast< int >(e_sucessor_directions::north_west)].set_node( *tmp_node );
			}
		}
	}

	/*******************************
		Private: Data Members
	*******************************/
private:
	// open/close node list
	std::list<S_Node> m_open_list = {};
	std::list<S_Node> m_close_list = {};

	// recorded path
	std::vector<S_Node> m_path = {};

	// movements
	bool m_diagonal_movement = false;
};