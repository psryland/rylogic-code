//*********************************************
// Renderer
//  Copyright (c) Rylogic Ltd 2012
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// One or more trees of models.
	//        A          H
	//      /   \       / \
	//     B     C     I   J
	//   / | \   |         |
	//  D  E  F  G         K
	//
	// - Serialised as: A0 B1 D2 E2 F2 C1 G2  H0 I1 J1 K2 (i.e. a depth first traversal of the trees)
	// - Children = all nodes to the right with level > the current.
	// - Multiple trees are stored contiguously
	struct ModelTreeNode
	{
		// Model to parent transform
		m4x4 m_o2p = m4x4::Identity();

		// Model instance name. Owned, because consumers may keep the tree after the loader's source data is released.
		std::string m_name = {};

		// The renderer model for this node in the model tree
		ModelPtr m_model = {};

		// The depth of this node in the tree. m_level == 0 for root nodes.
		int m_level = 0;
	};

	using ModelTree = pr::vector<ModelTreeNode>;
}