//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	struct Sampler :RefCounted<Sampler>
	{
		using Descriptor = ::pr::compute::Descriptor;

		// Notes:
		//  - Sampler follows the same pattern as TextureBase, see texture_base.h for more info.

		Renderer*  m_rdr;  // The manager that created this sampler
		RdrId      m_id;   // Id for this texture in the resource manager
		Descriptor m_samp; // The sampler descriptor
		string32   m_name; // Human readable id for the texture

		Sampler(Renderer& rdr, SamplerDesc const& desc);
		Sampler(Sampler&&) = delete;
		Sampler(Sampler const&) = delete;
		Sampler& operator=(Sampler&&) = delete;
		Sampler& operator=(Sampler const&) = delete;
		virtual ~Sampler();

		// Access the renderer
		Renderer& rdr() const;

		// Delegates to call when the sampler is destructed
		// WARNING: Don't add lambdas that capture a ref counted pointer to the sampler
		// or the sampler will never get destructed, since the ref will never hit zero.
		EventHandler<Sampler&, EmptyArgs const&, true> OnDestruction;

		// Ref counting clean up
		static void RefCountZero(RefCounted<Sampler>* doomed);
		protected: virtual void Delete();
	};
}
