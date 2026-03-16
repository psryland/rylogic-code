//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/resource/resource_store.h"

namespace pr::rdr12
{
	Model::Model(
		Renderer& rdr,
		int64_t vcount,
		int64_t icount,
		SizeAndAlign16 vstride,
		SizeAndAlign16 istride,
		ID3D12Resource* vb,
		ID3D12Resource* ib,
		BBox const& bbox,
		m4x4 const& m2root,
		std::string_view name
	)
		: m_rdr(&rdr)
		, m_vb(vb, true)
		, m_ib(ib, true)
		, m_vb_view({
			.BufferLocation = m_vb->GetGPUVirtualAddress(),
			.SizeInBytes = s_cast<UINT>(vcount * vstride.size()),
			.StrideInBytes = s_cast<UINT>(vstride.size()),
		})
		, m_ib_view({
			.BufferLocation = m_ib->GetGPUVirtualAddress(),
			.SizeInBytes = s_cast<UINT>(icount * istride.size()),
			.Format =
				istride.size() == sizeof(uint32_t) ? DXGI_FORMAT_R32_UINT :
				istride.size() == sizeof(uint16_t) ? DXGI_FORMAT_R16_UINT :
				throw std::runtime_error("Unsupported index buffer format"),
		})
		, m_nuggets()
		, m_vcount(vcount)
		, m_icount(icount)
		, m_m2root(m2root)
		, m_skin()
		, m_bbox(bbox)
		, m_name(name)
		, m_vstride(vstride)
		, m_istride(istride)
		, m_dbg_flags(EDbgFlags::None)
	{
		Check(m_vb->SetName(FmtS(L"%S:VB:%d", name, vcount)));
		Check(m_ib->SetName(FmtS(L"%S:IB:%d", name, icount)));
	}
	Model::~Model()
	{
		DeleteNuggets();
	}

	// Renderer access
	Renderer const& Model::rdr() const
	{
		return *m_rdr;
	}
	Renderer& Model::rdr()
	{
		return *m_rdr;
	}
	
	// Allow update of the vertex/index buffers
	GfxUpdateSubresourceScope Model::UpdateVertices(GfxCmdList& cmd_list, GpuUploadBuffer& upload, Range vrange)
	{
		if (vrange == Range::Reset())
			vrange = Range(0, m_vcount);

		// Vertex buffers are 1-D buffers, so the "element" is bytes
		vrange.m_beg *= m_vstride.size();
		vrange.m_end *= m_vstride.size();
		return GfxUpdateSubresourceScope(cmd_list, upload, m_vb.get(), m_vstride.align(), s_cast<int>(vrange.m_beg), s_cast<int>(vrange.size()));
	}
	GfxUpdateSubresourceScope Model::UpdateIndices(GfxCmdList& cmd_list, GpuUploadBuffer& upload, Range irange)
	{
		if (irange == Range::Reset())
			irange = Range(0, m_icount);

		// Index buffers are 1-D buffers, so the "element" is bytes
		irange.m_beg *= m_istride.size();
		irange.m_end *= m_istride.size();
		return GfxUpdateSubresourceScope(cmd_list, upload, m_ib.get(), m_istride.align(), s_cast<int>(irange.m_beg), s_cast<int>(irange.size()));
	}

	// Create a nugget from a range within this model
	// Ranges are model relative, i.e. the first vert in the model is range [0,1)
	// Remember you might need to delete render nuggets first
	void Model::CreateNugget(ResourceFactory& factory, NuggetDesc const& nugget_data)
	{
		auto ndata = NuggetDesc(nugget_data);

		// Invalid ranges are assumed to mean the entire model
		if (ndata.m_vrange == Range::Reset())
			ndata.vrange(0, m_vcount);
		if (ndata.m_irange == Range::Reset())
			ndata.irange(0, m_icount);

		// Sanity checks
		#if PR_DBG_RDR

		// Verify the ranges do not overlap with existing nuggets in this chain, unless explicitly allowed.
		if (!IsWithin(Range(0, m_vcount), ndata.m_vrange))
			throw std::runtime_error(std::format("V-Range exceeds the size of this model  ({})", m_name.c_str()));
		if (!IsWithin(Range(0, m_icount), ndata.m_irange))
			throw std::runtime_error(std::format("I-Range exceeds the size of this model ({})", m_name.c_str()));
		if (!AllSet(ndata.m_nflags, ENuggetFlag::RangesCanOverlap))
			for (auto& nug : m_nuggets)
				if (Intersects(ndata.m_irange, nug.m_irange))
					throw std::runtime_error(std::format("A render nugget covering this index range already exists. Did you forget the 'ENuggetFlag::RangesCanOverlap' flag, or is a DeleteNuggets() call needed ({})", m_name.c_str()));
		#endif

		auto nug = factory.CreateNugget(ndata, this);
		m_nuggets.push_back(*nug);
	}

	// Clear the render nuggets for this model.
	void Model::DeleteNuggets()
	{
		ResourceStore::Access store(rdr());
		for (;!m_nuggets.empty();)
			store.Delete(&m_nuggets.front());
	}

	// Ref-counting clean up function
	void Model::RefCountZero(RefCounted<Model>* doomed)
	{
		auto mdl = static_cast<Model*>(doomed);
		mdl->rdr().DeferRelease(mdl->m_vb);
		mdl->rdr().DeferRelease(mdl->m_ib);
		ResourceStore::Access store(mdl->rdr());
		store.Delete(mdl);
	}
}
