//**********************************************
// P3D Graphics Tool
//  Copyright (c) Rylogic Ltd 2019
//**********************************************

#pragma once
#include "src/forward.h"

// Populate the p3d data structures from a p3d file
std::unique_ptr<pr::geometry::p3d::File> CreateFromP3D(std::filesystem::path const& filepath);

// Populates the p3d data structures from a 3ds file
std::unique_ptr<pr::geometry::p3d::File> CreateFrom3DS(std::filesystem::path const& filepath);

// Populates the p3d data structures from a stl file
std::unique_ptr<pr::geometry::p3d::File> CreateFromSTL(std::filesystem::path const& filepath);

// Populates the p3d data structures from a glTF or GLB file.
// Images embedded in the model are written into 'texture_dir' and referenced by a content-derived id.
std::unique_ptr<pr::geometry::p3d::File> CreateFromGLTF(std::filesystem::path const& filepath, std::filesystem::path const& texture_dir);

// Popultes the p3d data structures from an obj file
std::unique_ptr<pr::geometry::p3d::File> CreateFromOBJ(std::filesystem::path const& filepath);

// Write 'p3d' as a p3d format file
void WriteP3d(std::unique_ptr<pr::geometry::p3d::File> const& p3d, std::filesystem::path const& outfile, pr::geometry::p3d::EFlags flags);

// Write 'p3d' as a cpp source file
void WriteCpp(std::unique_ptr<pr::geometry::p3d::File> const& p3d, std::filesystem::path const& outfile, std::string indent);

// Write 'p3d' as ldr script
void WriteLdr(std::unique_ptr<pr::geometry::p3d::File> const& p3d, std::filesystem::path const& outfile, std::string indent);
