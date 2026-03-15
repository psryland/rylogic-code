//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	// GPU-friendly contact data for the resolve shader.
	// Contains everything needed to compute and apply the restitution impulse.
	struct alignas(16) GpuResolveContact
	{
		v4 axis;          // collision normal (in A's object space)
		v4 point;         // contact point at estimated collision time (in A's space)
		m4x4 b2a;         // B-to-A transform
		int body_idx_a;   // index into RigidBodyDynamics buffer
		int body_idx_b;   // index into RigidBodyDynamics buffer
		float elasticity; // combined material elasticity (normal)
		float friction;   // combined material static friction
	};
	static_assert(sizeof(GpuResolveContact) == 112, "GpuResolveContact must be 112 bytes for GPU alignment");

	struct GpuResolver
	{
		Gpu& m_gpu;
		ComputeStep m_cs_resolve;
		D3DPtr<ID3D12Resource> m_r_contacts;
		D3DPtr<ID3D12Resource> m_r_colours;
		int m_capacity;

		explicit GpuResolver(Gpu& gpu);

		// Resolve collisions on the GPU using graph-coloured batches.
		// 'contacts' are the filtered/prepared contacts with body indices and materials.
		// 'colours' is the per-contact colour assignment.
		// 'max_colour' is the number of colour batches to dispatch.
		// 'bodies_resource' is the GPU-resident RigidBodyDynamics buffer from the integrator.
		void Resolve(
			GpuJob& job,
			std::span<GpuResolveContact const> contacts,
			std::span<int const> colours,
			int max_colour,
			ID3D12Resource* bodies_resource);

	private:

		void CompileShader();
		void ResizeBuffers(CmdList& cmd_list, int capacity);
	};

	// Greedy graph colouring: assign colours so no two contacts sharing a body have the same colour.
	// Returns {per-contact colour, number of colour batches}.
	std::pair<std::vector<int>, int> GraphColourContacts(std::span<GpuResolveContact const> contacts);
}
