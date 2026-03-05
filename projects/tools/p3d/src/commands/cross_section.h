//**********************************************
// P3D Graphics Tool
//  Copyright (c) Rylogic Ltd 2019
//**********************************************

#pragma once
#include "src/forward.h"

// Clip a model using the given plane
void CrossSection(pr::geometry::p3d::Mesh& mesh, pr::v4 plane);

// Clip a model using the given plane
void CrossSection(pr::geometry::p3d::File& p3d, pr::v4 plane);
