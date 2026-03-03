//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_gizmo.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/ldraw/ldraw_commands.h"
#include "pr/view3d-12/model/model_generator.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/utility/conversion.h"
#include "view3d-12/src/ldraw/sources/source_base.h"
#include "view3d-12/src/ldraw/sources/source_file.h"
#include "view3d-12/src/ldraw/sources/source_string.h"
#include "view3d-12/src/dll/context.h"
#include "view3d-12/src/dll/v3d_window.h"

namespace pr::rdr12
{
	Context::Context(HINSTANCE instance, view3d::ReportErrorCB global_error_cb)
		: m_rdr(RdrSettings(instance).DebugLayer(PR_DBG_RDR).DefaultAdapter())
		, m_windows()
		, m_sources(m_rdr, *this)
		, m_inits()
		, m_mutex()
		, ReportError()
		, ParsingProgress()
		, SourcesChanged()
	{
		ReportError += global_error_cb;
	}
	Context::~Context()
	{
		for (auto& wnd : m_windows)
			delete wnd;
	}

	// Report an error handled at the DLL API layer
	void Context::ReportAPIError(char const* func_name, view3d::Window wnd, std::exception const* ex)
	{
		// Create the error message
		auto msg = Fmt<pr::string<char>>("%s failed.\n%s", func_name, ex ? ex->what() : "Unknown exception occurred.");
		if (msg.last() != '\n')
			msg.push_back('\n');

		// If a window handle is provided, report via the window's event.
		// Otherwise, fall back to the global error handler
		if (wnd != nullptr)
			wnd->ReportError(msg.c_str(), "", 0, 0);
		else
			ReportError(msg.c_str(), "", 0, 0);
	}

	// Create/Destroy windows
	V3dWindow* Context::WindowCreate(HWND hwnd, view3d::WindowOptions const& opts)
	{
		try
		{
			V3dWindow* win;
			m_windows.push_back(win = new V3dWindow(rdr(), hwnd, opts));
			return win;
		}
		catch (std::exception const& e)
		{
			if (opts.m_error_cb) opts.m_error_cb(std::format("Failed to create View3D Window.\n{}", e.what()).c_str(), "", 0, 0);
			return nullptr;
		}
		catch (...)
		{
			if (opts.m_error_cb) opts.m_error_cb("Failed to create View3D Window.\nUnknown reason", "", 0, 0);
			return nullptr;
		}
	}
	void Context::WindowDestroy(V3dWindow* window)
	{
		erase_first(m_windows, [=](auto& wnd) { return wnd == window; });
		delete window;
	}

	// Create an include handler that can load from directories or embedded resources.
	PathResolver IncludeHandler(view3d::Includes const* includes)
	{
		PathResolver inc;
		if (includes != nullptr)
		{
			if (includes->m_include_paths != nullptr)
				inc.SearchPathList(includes->m_include_paths);

			if (includes->m_module_count != 0)
				inc.ResourceModules(std::initializer_list<HMODULE>(includes->m_modules, includes->m_modules + includes->m_module_count));
		}
		return std::move(inc);
	}

	// Load/Add ldr objects from a script file. Returns the Guid of the context that the objects were added to.
	Guid Context::LoadScriptFile(std::filesystem::path ldr_script, EEncoding enc, Guid const* context_id, PathResolver const& includes, ldraw::AddCompleteCB add_complete) // worker thread context
	{
		return m_sources.AddFile(ldr_script, enc, context_id, includes, add_complete);
	}

	// Load/Add ldr objects from a script string. Returns the Guid of the context that the objects were added to.
	template <typename Char>
	Guid Context::LoadScriptString(std::basic_string_view<Char> ldr_script, EEncoding enc, Guid const* context_id, PathResolver const& includes, ldraw::AddCompleteCB add_complete) // worker thread context
	{
		return m_sources.AddString(ldr_script, enc, context_id, includes, add_complete);
	}
	template Guid Context::LoadScriptString<wchar_t>(std::wstring_view ldr_script, EEncoding enc, Guid const* context_id, PathResolver const& includes, ldraw::AddCompleteCB add_complete);
	template Guid Context::LoadScriptString<char>(std::string_view ldr_script, EEncoding enc, Guid const* context_id, PathResolver const& includes, ldraw::AddCompleteCB add_complete);

	// Load/Add ldraw objects from binary data. Returns the Guid of the context that the objects were added to.
	Guid Context::LoadScriptBinary(std::span<std::byte const> data, Guid const* context_id, ldraw::AddCompleteCB add_complete)
	{
		return m_sources.AddBinary(data, context_id, add_complete);
	}

	// Cancel an in-progress load operation
	void Context::CancelLoad(Guid const& context_id)
	{
		m_sources.CancelLoad(context_id);
	}

	// Enable/Disable streaming script sources.
	ldraw::EStreamingState Context::StreamingState() const
	{
		return m_sources.StreamingState();
	}
	void Context::Streaming(bool enabled, uint16_t port)
	{
		if (enabled)
			m_sources.AllowConnections(port);
		else
			m_sources.StopConnections();
	}

	// Create an object from geometry
	ldraw::LdrObject* Context::ObjectCreate(char const* name, Colour32 colour, std::span<view3d::Vertex const> verts, std::span<uint16_t const> indices, std::span<view3d::Nugget const> nuggets, Guid const& context_id)
	{
		using namespace pr::script;

		auto geom = EGeom::None;
		pr::vector<NuggetDesc> ngt;

		// Generate the nuggets first so we can tell what geometry data is needed
		for (auto const& nugget : nuggets)
		{
			// Create the renderer nugget
			NuggetDesc nug = NuggetDesc(static_cast<ETopo>(nugget.m_topo), static_cast<EGeom>(nugget.m_geom))
				.vrange(nugget.m_v0 != nugget.m_v1 ? Range(nugget.m_v0, nugget.m_v1) : Range(0, verts.size()))
				.irange(nugget.m_i0 != nugget.m_i1 ? Range(nugget.m_i0, nugget.m_i1) : Range(0, indices.size()))
				.tex_diffuse(Texture2DPtr(nugget.m_tex_diffuse, true))
				.sam_diffuse(SamplerPtr(nugget.m_sam_diffuse, true))
				.flags(static_cast<ENuggetFlag>(nugget.m_nflags))
				.rel_reflec(nugget.m_rel_reflec)
				.tint(nugget.m_tint);

			if (nugget.m_cull_mode != view3d::ECullMode::Default) nug.pso<EPipeState::CullMode>(static_cast<D3D12_CULL_MODE>(nugget.m_cull_mode));
			if (nugget.m_fill_mode != view3d::EFillMode::Default) nug.pso<EPipeState::FillMode>(static_cast<D3D12_FILL_MODE>(nugget.m_fill_mode));
			for (auto const& shdr : nugget.shader_span())
				nug.use_shader_overlay(static_cast<ERenderStep>(shdr.m_rdr_step), ShaderPtr(shdr.m_shader, true));

			ngt.push_back(nug);

			// Sanity check the nugget
			PR_ASSERT(PR_DBG, nug.m_vrange.begin() <= nug.m_vrange.end() && nug.m_vrange.end() <= isize(verts), "Invalid nugget V-range");
			PR_ASSERT(PR_DBG, nug.m_irange.begin() <= nug.m_irange.end() && nug.m_irange.end() <= isize(indices), "Invalid nugget I-range");

			// Union of geometry data type
			geom |= nug.m_geom;
		}

		// Vertex buffer
		pr::vector<v4> pos;
		{
			pos.resize(verts.size());
			for (int i = 0, iend = isize(verts); i != iend; ++i)
				pos[i] = To<v4>(verts[i].pos);
		}

		// Colour buffer
		pr::vector<Colour32> col;
		if (AllSet(geom, EGeom::Colr))
		{
			col.resize(verts.size());
			for (int i = 0, iend = isize(verts); i != iend; ++i)
				col[i] = verts[i].col;
		}

		// Normals
		pr::vector<v4> nrm;
		if (AllSet(geom, EGeom::Norm))
		{
			nrm.resize(verts.size());
			for (int i = 0, iend = isize(verts); i != iend; ++i)
				nrm[i] = To<v4>(verts[i].norm);
		}

		// Texture coords
		pr::vector<v2> tex;
		if (AllSet(geom, EGeom::Tex0))
		{
			tex.resize(verts.size());
			for (int i = 0, iend = isize(verts); i != iend; ++i)
				tex[i] = To<v2>(verts[i].tex);
		}

		// Indices
		auto& ind = indices;

		// Create the model
		MeshCreationData cdata = MeshCreationData().verts(pos).indices(ind).nuggets(ngt).colours(col).normals(nrm).tex(tex);
		auto obj = Create(m_rdr, ldraw::ELdrObject::Custom, cdata, context_id);

		// Add to the sources
		obj->m_name = name;
		obj->m_base_colour = colour;
		m_sources.Add(obj);

		// Return the created object
		return obj.get();
	}

	// Load/Add ldr objects and return the first object from the script
	template <typename Char>
	ldraw::LdrObject* Context::ObjectCreateLdr(std::basic_string_view<Char> ldr_script, bool file, EEncoding enc, Guid const* context_id, view3d::Includes const* includes)
	{
		// Get the context id for this script
		auto id = context_id ? *context_id : GenerateGUID();

		// Create an include handler
		auto include_handler = IncludeHandler(includes);

		// Any LdrObject* we return must not get deleted by a Reload() of its source.
		// That's why these sources are not added to 'm_sources'. The Reload() feature
		// only works for objects that are managed by Guid. However, external code can
		// watch for the Reload notification and manually reload objects, replacing the
		// LdrObject* pointers they hold.

		// Load the ldr script
		ldraw::ParseResult output;
		if (file)
		{
			ldraw::SourceFile src{ &id, ldr_script, enc, include_handler };
			output = src.Load(rdr());
		}
		else
		{
			ldraw::SourceString<Char> src{ &id, ldr_script, enc, include_handler };
			output = src.Load(rdr());
		}
		if (output.m_objects.empty())
			return nullptr;

		// Return the first object.
		auto& obj = output.m_objects.front();
		m_sources.Add(obj);
		return obj.get();
	}
	template ldraw::LdrObject* Context::ObjectCreateLdr<wchar_t>(std::wstring_view ldr_script, bool file, EEncoding enc, Guid const* context_id, view3d::Includes const* includes);
	template ldraw::LdrObject* Context::ObjectCreateLdr<char>(std::string_view ldr_script, bool file, EEncoding enc, Guid const* context_id, view3d::Includes const* includes);

	// Create an LdrObject from the p3d model
	ldraw::LdrObject* Context::ObjectCreateP3D(char const* name, Colour32 colour, std::filesystem::path const& p3d_filepath, Guid const* context_id)
	{
		// Get the context id
		auto id = context_id ? *context_id : GenerateGUID();

		// Create an ldr object
		auto obj = ldraw::CreateP3D(m_rdr, ldraw::ELdrObject::Model, p3d_filepath, id);
		obj->m_name = name;
		obj->m_base_colour = colour;
		m_sources.Add(obj);
		return obj.get();
	}
	ldraw::LdrObject* Context::ObjectCreateP3D(char const* name, Colour32 colour, std::span<std::byte const> p3d_data, Guid const* context_id)
	{
		// Get the context id
		auto id = context_id ? *context_id : pr::GenerateGUID();

		// Create an ldr object
		auto obj = rdr12::ldraw::CreateP3D(m_rdr, ldraw::ELdrObject::Model, p3d_data, id);
		obj->m_name = name;
		obj->m_base_colour = colour;
		m_sources.Add(obj);
		return obj.get();
	}

	// Modify an ldr object using a callback to populate the model data.
	static void __stdcall EditModel(void* ctx, Model* model, Renderer&)
	{
		using namespace pr::rdr12;
		auto& edit_cb = *static_cast<view3d::EditObjectCB*>(ctx);

		// Thread local storage for editing dynamic models
		thread_local static pr::vector<view3d::Vertex>         cache_vbuf;
		thread_local static pr::vector<uint16_t>               cache_ibuf;
		thread_local static pr::vector<view3d::Nugget>         cache_nbuf;

		if (!model)
			throw std::runtime_error("model is null");

		auto& vbuf = cache_vbuf;
		auto& ibuf = cache_ibuf;
		auto& nbuf = cache_nbuf;

		// Create buffers to be filled by the user callback
		// Note: We can't fill the buffers with the existing model data because that requires
		// reading from video memory (slow, or not possible for some model types).
		vbuf.resize(s_cast<size_t>(model->m_vcount));
		ibuf.resize(s_cast<size_t>(model->m_icount));
		nbuf.resize(0);

#if 0
		// If the model already has nuggets, initialise 'nbuf' with them
		if (!model->m_nuggets.empty())
		{
			auto ValueOrDefault = [](auto* v, decltype(*v) d) -> decltype(*v) { return v ? *v : d; };

			for (auto& nug : model->m_nuggets)
			{
				nbuf.push_back(view3d::Nugget{
					.m_topo = static_cast<view3d::ETopo>(nug.m_topo),
					.m_geom = static_cast<view3d::EGeom>(nug.m_geom),
					.m_tex_diffuse = nug.m_tex_diffuse.get(),
					.m_sam_diffuse = nug.m_sam_diffuse.get(),
					.m_shaders = {.m_shaders = nug.m_shaders.data(), .m_count = isize(nug.m_shaders) },
					.m_v0 = s_cast<int>(nug.m_vrange.begin()),
					.m_v1 = s_cast<int>(nug.m_vrange.end()),
					.m_i0 = s_cast<int>(nug.m_irange.begin()),
					.m_i1 = s_cast<int>(nug.m_irange.end()),
					.m_nflags = static_cast<view3d::ENuggetFlag>(nug.m_nflags),
					.m_cull_mode = static_cast<view3d::ECullMode>(ValueOrDefault(nug.m_pso.Find<EPipeState::CullMode>(), D3D12_CULL_MODE(0))),
					.m_fill_mode = static_cast<view3d::EFillMode>(ValueOrDefault(nug.m_pso.Find<EPipeState::FillMode>(), D3D12_FILL_MODE(0))),
					.m_tint = To<view3d::Colour>(nug.m_tint),
					.m_rel_reflec = nug.m_rel_reflec,

				});
			}
		}
#endif

		// Get the user to generate/update the model
		auto add_nugget = view3d::AddNuggetCB{ &nbuf, [](void* ctx, view3d::Nugget const& n) { static_cast<pr::vector<view3d::Nugget>*>(ctx)->push_back(n); } };
		auto [new_vcount, new_icount] = edit_cb(isize(vbuf), isize(ibuf), vbuf.data(), ibuf.data(), add_nugget);

		// Sanity check results
		if (new_vcount > isize(vbuf)) throw std::runtime_error("Dynamic model buffer overrun (v-buf)");
		if (new_icount > isize(ibuf)) throw std::runtime_error("Dynamic model buffer overrun (i-buf)");

		ResourceFactory factory(model->rdr());

		{// Update the model geometry
			auto update_v = model->UpdateVertices(factory.CmdList(), factory.UploadBuffer(), { 0, new_vcount });
			auto update_i = model->UpdateIndices(factory.CmdList(), factory.UploadBuffer(), { 0, new_icount });

			model->m_bbox.reset();

			auto vin = vbuf.data();
			auto iin = ibuf.data();
			auto vout = update_v.ptr<Vert>();
			auto iout = update_i.ptr<uint16_t>();

			// Copy the model data into the model
			for (int i = 0; i != new_vcount; ++i, ++vin)
			{
				SetPCNT(*vout++, To<v4>(vin->pos), Colour(vin->col), To<v4>(vin->norm), To<v2>(vin->tex));
				Grow(model->m_bbox, To<v4>(vin->pos));
			}
			for (int i = 0; i != new_icount; ++i, ++iin)
			{
				*iout++ = *iin;
			}

			update_v.Commit();
			update_i.Commit();
		}

		// Update the model nuggets
		model->DeleteNuggets();
		for (auto& nug : nbuf)
		{
			auto n = NuggetDesc{};
			n.m_topo = static_cast<ETopo>(nug.m_topo);
			n.m_geom = static_cast<EGeom>(nug.m_geom);
			n.m_tex_diffuse = Texture2DPtr(nug.m_tex_diffuse, true);
			n.m_sam_diffuse = SamplerPtr(nug.m_sam_diffuse, true);

			for (auto& shdr : nug.shader_span())
				n.m_shdr_overlays.push_back({ ShaderPtr(shdr.m_shader, true), static_cast<ERenderStep>(shdr.m_rdr_step) });

			if (nug.m_cull_mode != view3d::ECullMode::Default)
				n.m_pso.Set<EPipeState::CullMode>(static_cast<D3D12_CULL_MODE>(nug.m_cull_mode));
			if (nug.m_fill_mode != view3d::EFillMode::Default)
				n.m_pso.Set<EPipeState::FillMode>(static_cast<D3D12_FILL_MODE>(nug.m_fill_mode));

			n.m_nflags = static_cast<ENuggetFlag>(nug.m_nflags);
			n.m_tint = nug.m_tint != 0 ? To<Colour>(nug.m_tint) : Colour32White;
			n.m_rel_reflec = nug.m_rel_reflec;
			n.m_vrange = nug.m_v0 < nug.m_v1 ? Range{ nug.m_v0, nug.m_v1 } : Range{ 0, new_vcount };
			n.m_irange = nug.m_i0 < nug.m_i1 ? Range{ nug.m_i0, nug.m_i1 } : Range{ 0, new_icount };
			model->CreateNugget(factory, n);
		}

		// Release memory after large allocations
		if (vbuf.capacity() > 0x100000) vbuf.clear();
		if (ibuf.capacity() > 0x100000) ibuf.clear();
		if (nbuf.capacity() > 0x100000) nbuf.clear();
	}
	ldraw::LdrObject* Context::ObjectCreateByCallback(char const* name, Colour32 colour, int vcount, int icount, int ncount, view3d::EditObjectCB edit_cb, Guid const& context_id)
	{
		auto obj = ldraw::CreateEditCB(m_rdr, ldraw::ELdrObject::Custom, vcount, icount, ncount, { &edit_cb, EditModel }, context_id);
		obj->m_name = name;
		obj->m_base_colour = colour;
		m_sources.Add(obj);
		return obj.get();
	}
	void Context::ObjectEdit(ldraw::LdrObject* object, view3d::EditObjectCB edit_cb)
	{
		// Remove the object from any windows it might be in
		for (auto& wnd : m_windows)
			wnd->Remove(object);

		// Callback to edit the geometry
		Edit(m_rdr, object, { &edit_cb, EditModel });
	}

	// Update the model in an existing object
	template <typename Char>
	void Context::UpdateObject(ldraw::LdrObject* object, std::basic_string_view<Char> ldr_script, ldraw::EUpdateObject flags)
	{
		// Remove the object from any windows it might be in
		for (auto& wnd : m_windows)
			wnd->Remove(object);

		// Update the object model
		mem_istream<Char> src{ ldr_script, 0 };
		rdr12::ldraw::TextReader reader(src, {});
		ldraw::Update(m_rdr, object, reader, flags);
	}
	template void Context::UpdateObject<wchar_t>(ldraw::LdrObject* object, std::wstring_view, ldraw::EUpdateObject flags);
	template void Context::UpdateObject<char>(ldraw::LdrObject* object, std::string_view, ldraw::EUpdateObject flags);

	// Delete a single object
	void Context::DeleteObject(ldraw::LdrObject* object)
	{
		// Remove the object from any windows it's in
		for (auto& wnd : m_windows)
			wnd->Remove(object);

		// Delete the object from the object container
		m_sources.Remove(object);
	}

	// Delete all objects
	void Context::DeleteAllObjects()
	{
		// Remove the objects from any windows they're in
		for (auto& wnd : m_windows)
			wnd->RemoveAllObjects();

		// Clear the object container. The unique pointers should delete the objects
		m_sources.ClearAll();
	}

	// Delete all objects with matching ids
	void Context::DeleteAllObjectsById(view3d::GuidPredCB pred)
	{
		// Remove objects from any windows they might be assigned to
		for (auto& wnd : m_windows)
			wnd->Remove(pred, false);

		// Remove sources that match the given set of context ids to delete
		m_sources.Remove(pred);
	}

	// Delete all objects not displayed in any windows
	void Context::DeleteUnused(view3d::GuidPredCB pred)
	{
		// Build a set of context ids, included in 'context_ids', and not used in any windows
		GuidSet unused;

		// Initialise 'unused' with all context ids (filtered by 'context_ids')
		for (auto& src : m_sources.Sources())
		{
			if (!pred(src.first)) continue;
			unused.insert(src.first);
		}

		// Remove those that are used in the windows
		for (auto& wnd : m_windows)
		{
			for (auto& id : wnd->m_guids)
				unused.erase(id);
		}

		// Remove unused sources
		if (!unused.empty())
		{
			m_sources.Remove([&unused](Guid const& id) { return unused.contains(id); });
		}
	}

	// Enumerate all sources in the store
	void Context::EnumSources(view3d::EnumGuidsCB enum_guids_cb)
	{
		for (auto& pair : m_sources.Sources())
		{
			auto& src = pair.second;
			if (!enum_guids_cb(src->m_context_id))
				return;
		}
	}
	
	// Return details about a source
	view3d::SourceInfo Context::SourceInfo(Guid const& context_id)
	{
		auto* src = FindSource(context_id);
		if (src == nullptr)
			return {};

		auto text_format = false;
		auto filepath = (wchar_t const*)nullptr;
		if (auto const* file_src = dynamic_cast<ldraw::SourceFile const*>(src))
		{
			filepath = file_src->m_filepath.c_str();
			text_format = file_src->m_text_format;
		}

		return view3d::SourceInfo{
			.m_name = src->m_name.c_str(),
			.m_filepath = filepath,
			.m_context_id = context_id,
			.m_object_count = isize(src->m_output.m_objects),
			.m_text_format = text_format ? 1 : 0,
		};
	}

	// Get/Set the name of a source
	string32 const& Context::SourceName(Guid const& context_id)
	{
		if (auto* src = FindSource(context_id))
			return src->m_name;

		static string32 const null_name = {};
		return null_name;
	}
	void Context::SourceName(Guid const& context_id, std::string_view name)
	{
		if (auto* src = FindSource(context_id))
			src->m_name = name;
	}

	// Create a gizmo object and add it to the gizmo collection
	ldraw::LdrGizmo* Context::GizmoCreate(ldraw::EGizmoMode mode, m4x4 const& o2w)
	{
		return m_sources.CreateGizmo(mode, o2w);
	}

	// Destroy a gizmo
	void Context::GizmoDelete(ldraw::LdrGizmo* gizmo)
	{
		// Remove the gizmo from any windows it's in
		for (auto& wnd : m_windows)
			wnd->Remove(gizmo);

		// Delete the gizmo from the sources
		m_sources.RemoveGizmo(gizmo);
	}

	// Reload sources
	void Context::ReloadScriptSources()
	{
		m_sources.Reload();
	}

	// Reload objects from a source
	void Context::ReloadScriptSources(std::span<Guid const> context_ids)
	{
		m_sources.Reload(context_ids);
	}

	// Poll for changed script source files, and reload any that have changed
	void Context::CheckForChangedSources()
	{
		m_sources.RefreshChangedFiles();
	}

	// Find the source associated with a context id
	ldraw::SourceBase const* Context::FindSource(Guid const& context_id) const
	{
		auto const& srcs = m_sources.Sources();
		auto iter = srcs.find(context_id);
		return iter != std::end(srcs) ? iter->second.get() : nullptr;
	}
	ldraw::SourceBase* Context::FindSource(Guid const& context_id)
	{
		auto& srcs = m_sources.Sources();
		auto iter = srcs.find(context_id);
		return iter != std::end(srcs) ? iter->second.get() : nullptr;
	}

	// Parse error event.
	void Context::OnError(ldraw::ParseErrorEventArgs const& args)
	{
		auto filepath = args.m_loc.m_filepath.generic_string();
		ReportError(args.m_msg.c_str(), filepath.c_str(), args.m_loc.m_line, args.m_loc.m_offset);
	}

	// An event raised during parsing. This is called in the context of the threads that call 'AddFile'. Do not sign up while AddFile calls are running.
	void Context::OnParsingProgress(ldraw::ParsingProgressEventArgs& args)
	{
		auto context_id = args.m_context_id;
		auto filepath = args.m_loc.m_filepath.generic_string();
		auto file_offset = s_cast<int64_t>(args.m_loc.m_offset);
		auto file_size = s_cast<int64_t>(args.m_loc.m_filesize);
		BOOL complete = args.m_complete;
		BOOL cancel = false;
		ParsingProgress(context_id, filepath.c_str(), file_offset, file_size, complete, cancel);
		args.m_cancel = cancel != 0;
	}

	// Store change event. Called before and after a change to the collection of objects in the store.
	void Context::OnStoreChange(ldraw::StoreChangeEventArgs const& args)
	{
		view3d::ESourcesChangedReason reason = {};
		switch (args.m_trigger)
		{
			case ldraw::EDataChangeTrigger::NewData:
			{
				// On NewData, do nothing. Callers will add objects to windows as they see fit.
				reason = view3d::ESourcesChangedReason::NewData;
				break;
			}
			case ldraw::EDataChangeTrigger::Reload:
			{
				for (auto& wnd : m_windows)
				{
					// When a source is about to be reloaded, remove it's objects from the windows, but keep the context ids so we know what to reload.
					if (args.m_before)
					{
						wnd->Remove({ &args.m_context_ids, ldraw::MatchContextIdInSpan }, true);
					}

					// After reload, each window re-adds objects from the previous contexts
					else
					{
						struct Ids { std::span<Guid const> ctx_ids; GuidSet const& wnd_ids; } ids = { args.m_context_ids, wnd->m_guids };
						constexpr auto ReAdd = [](void* ctx, Guid const& id)
						{
							auto& x = *static_cast<Ids*>(ctx);
							return x.wnd_ids.contains(id) && std::ranges::find(x.ctx_ids, id) != end(x.ctx_ids);
						};
						wnd->Add(m_sources.Sources(), { &ids, ReAdd });
					}

					wnd->Invalidate();
				}
				reason = view3d::ESourcesChangedReason::Reload;
				break;
			}
			case ldraw::EDataChangeTrigger::Removal:
			{
				// When a source is about to be removed, remove it's objects from the windows.
				if (args.m_before)
				{
					for (auto& wnd : m_windows)
						wnd->Remove({ &args.m_context_ids, ldraw::MatchContextIdInSpan }, false);
				}
				reason = view3d::ESourcesChangedReason::Removal;
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown store changed reason");
			}
		}

		// Notify of updated sources
		SourcesChanged(reason, args.m_context_ids.data(), isize(args.m_context_ids), args.m_before);
	}

	// Process any received commands in the source
	void Context::OnHandleCommands(ldraw::SourceBase& source)
	{
		using namespace ldraw;

		byte_data_cptr ptr(source.m_output.m_commands);
		for (; ptr; )
		{
			try
			{
				// Process the command
				switch (ptr.as<ldraw::ECommandId>())
				{
					case ECommandId::Invalid:
					{
						ptr.read<Command_Invalid>();
						break;
					}
					case ECommandId::AddToScene:
					{
						auto const& cmd = ptr.read<Command_AddToScene>();

						// Look for the window to add objects to. Ignore windows out of range
						if (cmd.m_scene_id < 0 || cmd.m_scene_id >= isize(m_windows))
							break;

						// Add all objects from 'source' to 'window'
						auto& window = *m_windows[cmd.m_scene_id];
						for (auto& obj : source.m_output.m_objects)
							window.Add(obj.get());

						break;
					}
					case ECommandId::CameraToWorld:
					{
						throw std::runtime_error("not implemented");
					}
					case ECommandId::CameraPosition:
					{
						throw std::runtime_error("not implemented");
					}
					case ECommandId::ObjectToWorld:
					{
						// TODO: Support 'Parent.Child.Child' syntax
						auto const& cmd = ptr.read<Command_ObjectToWorld>();
						auto target = string32(cmd.m_object_name);

						// Find the first object matching 'cmd.m_object_name' (in 'source.m_context')
						auto iter = pr::find_if(source.m_output.m_objects, [&target](LdrObjectPtr& ptr) { return ptr->m_name == target; });
						if (iter == std::end(source.m_output.m_objects))
							break;

						// Update the object to world transform for the object
						(*iter)->O2W(cmd.m_o2w);
						break;
					}
					case ECommandId::Render:
					{
						auto const& cmd = ptr.read<Command_Render>();

						// Look for the window to add objects to. Ignore windows out of range
						if (cmd.m_scene_id < 0 || cmd.m_scene_id >= isize(m_windows))
							break;

						// Render the window
						auto& window = *m_windows[cmd.m_scene_id];
						window.Render();
						break;
					}
					default:
					{
						assert(false); // to trap them here
						throw std::runtime_error("Unsupported command");
					}
				}
			}
			catch (std::exception const& ex)
			{
				ReportError(std::format("Command Error: {}", ex.what()).c_str(), "", 0, 0);
			}
		}

		// All commands have been executed
		source.m_output.m_commands.resize(0);
	}
}
