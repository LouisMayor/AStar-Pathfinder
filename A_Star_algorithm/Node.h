#pragma once

// assumptions are you would read data from file to initialise each node with values
struct S_Node {
	// g - movement cost from start to end
	int g = 0;

	// h - estimated cost of moving from current node to final destination
	int h = 0;

	// f - combined value of g and h
	int f = -1;

	// coords
	int x_pos = 0, y_pos = 0;

	// parent
	S_Node* parent_node = nullptr;

	S_Node( ) {

	}

	S_Node( int par_g, int par_x_pos, int par_y_pos, S_Node* par_parent_node ) {
		this->g = par_g;
		this->x_pos = par_x_pos;
		this->y_pos = par_y_pos;
	}

	~S_Node( ) {
	}

	void set_node( const S_Node& data ) {
		this->g = data.g;
		this->h = data.h;
		this->f = data.g + data.h;
		this->x_pos = data.x_pos;
		this->y_pos = data.y_pos;
	}

	// copy address of parent_node
	void deep_copy_parent( S_Node* par_parent_node ) {
		this->parent_node = par_parent_node;
	}

	// copy data from parent_node
	void shallow_copy_parent( S_Node* par_parent_node ) {
		if( parent_node != nullptr ) {
			this->parent_node->h = par_parent_node->h;
			this->parent_node->g = par_parent_node->g;
			this->parent_node->f = par_parent_node->f;
			this->parent_node->x_pos = par_parent_node->x_pos;
			this->parent_node->y_pos = par_parent_node->y_pos;
		}
	}
};