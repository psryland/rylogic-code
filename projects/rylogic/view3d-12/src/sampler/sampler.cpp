//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/sampler/sampler.h"
#include "pr/view3d-12/sampler/sampler_desc.h"
#include "pr/view3d-12/main/renderer.h"

namespace pr::rdr12
{
	using ::pr::compute::MakeId;

	Sampler::Sampler(Renderer& rdr, SamplerDesc const& desc)
		:RefCounted<Sampler>()
		,m_rdr(&rdr)
		,m_id(desc.m_id == AutoId ? MakeId(this) : desc.m_id)
		,m_samp()
		,m_name(desc.m_name)
	{
		ResourceStore::Access store(rdr);
		m_samp = store.Descriptors().Create(desc.m_sdesc);
	}
	Sampler::~Sampler()
	{
		OnDestruction(*this, EmptyArgs());

		// Release the sampler resource
		if (m_samp)
		{
			ResourceStore::Access store(rdr());
			store.Descriptors().Release(m_samp);
		}
	}

	// Access the renderer
	Renderer& Sampler::rdr() const
	{
		return *m_rdr;
	}

	// Ref counting clean up function
	void Sampler::RefCountZero(RefCounted<Sampler>* doomed)
	{
		auto sam = static_cast<Sampler*>(doomed);
		sam->Delete();
	}
	void Sampler::Delete()
	{
		ResourceStore::Access store(rdr());
		store.Delete(this);
	}
}
