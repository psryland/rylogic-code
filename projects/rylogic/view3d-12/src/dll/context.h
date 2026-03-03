//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "view3d-12/src/ldraw/sources/ldraw_sources.h"
#include "view3d-12/src/dll/dll_forward.h"

namespace pr::rdr12
{
	struct Context : ldraw::ISourceEvents
	{
		using InitSet = std::unordered_set<view3d::DllHandle>;
		using WindowCont = std::vector<V3dWindow*>;
		using ScriptSources = pr::rdr12::ldraw::ScriptSources;

		inline static Guid const GuidDemoSceneObjects = { 0xFE51C164, 0x9E57, 0x456F, 0x9D, 0x8D, 0x39, 0xE3, 0xFA, 0xAF, 0xD3, 0xE7 };

		Renderer             m_rdr;      // The renderer
		WindowCont           m_windows;  // The created windows
		ScriptSources        m_sources;  // A container of Ldr objects and a file watcher
		InitSet              m_inits;    // A unique id assigned to each Initialise call
		std::recursive_mutex m_mutex;

		Context(HINSTANCE instance, view3d::ReportErrorCB global_error_cb);
		Context(Context&&) = delete;
		Context(Context const&) = delete;
		Context& operator=(Context&) = delete;
		Context& operator=(Context const&) = delete;
		~Context();
		
		// Access functions
		Context* This() { return this; }
		Renderer& rdr() { return m_rdr; }

		// Report an error handled at the DLL API layer
		void ReportAPIError(char const* func_name, view3d::Window wnd, std::exception const* ex);

		// Create/Destroy windows
		V3dWindow* WindowCreate(HWND hwnd, view3d::WindowOptions const& opts);
		void WindowDestroy(V3dWindow* window);

		// Global error callback. Can be called in a worker thread context
		MultiCast<view3d::ReportErrorCB, true> ReportError;

		// Event raised when script sources are parsed during adding/updating
		MultiCast<view3d::ParsingProgressCB, true> ParsingProgress;

		// Event raised when the script sources are updated
		MultiCast<view3d::SourcesChangedCB, true> SourcesChanged;

		// Load/Add ldraw objects from a script file. Returns the Guid of the context that the objects were added to.
		Guid LoadScriptFile(std::filesystem::path ldr_script, EEncoding enc, Guid const* context_id, PathResolver const& includes, ldraw::AddCompleteCB add_complete);

		// Load/Add ldraw objects from a script string or file. Returns the Guid of the context that the objects were added to.
		template <typename Char>
		Guid LoadScriptString(std::basic_string_view<Char> ldr_script, EEncoding enc, Guid const* context_id, PathResolver const& includes, ldraw::AddCompleteCB add_complete);

		// Load/Add ldraw objects from binary data. Returns the Guid of the context that the objects were added to.
		Guid LoadScriptBinary(std::span<std::byte const> data, Guid const* context_id, ldraw::AddCompleteCB add_complete);

		// Cancel an in-progress load operation. Thread-safe.
		void CancelLoad(Guid const& context_id);

		// Enable/Disable streaming script sources.
		ldraw::EStreamingState StreamingState() const;
		void Streaming(bool enabled, uint16_t port);

		// Create an object from geometry
		ldraw::LdrObject* ObjectCreate(char const* name, Colour32 colour, std::span<view3d::Vertex const> verts, std::span<uint16_t const> indices, std::span<view3d::Nugget const> nuggets, Guid const& context_id);

		// Load/Add ldr objects and return the first object from the script
		template <typename Char>
		ldraw::LdrObject* ObjectCreateLdr(std::basic_string_view<Char> ldr_script, bool file, EEncoding enc, Guid const* context_id, view3d::Includes const* includes);

		// Create an LdrObject from the p3d model
		ldraw::LdrObject* ObjectCreateP3D(char const* name, Colour32 colour, std::filesystem::path const& p3d_filepath, Guid const* context_id);
		ldraw::LdrObject* ObjectCreateP3D(char const* name, Colour32 colour, std::span<std::byte const> p3d_data, Guid const* context_id);

		// Modify an ldr object using a callback to populate the model data.
		ldraw::LdrObject* ObjectCreateByCallback(char const* name, Colour32 colour, int vcount, int icount, int ncount, view3d::EditObjectCB edit_cb, Guid const& context_id);
		void ObjectEdit(ldraw::LdrObject* object, view3d::EditObjectCB edit_cb);

		// Update the model in an existing object
		template <typename Char>
		void UpdateObject(ldraw::LdrObject* object, std::basic_string_view<Char> ldr_script, ldraw::EUpdateObject flags);

		// Delete a single object
		void DeleteObject(ldraw::LdrObject* object);

		// Delete all objects
		void DeleteAllObjects();

		// Delete all objects with matching ids
		void DeleteAllObjectsById(view3d::GuidPredCB pred);
		
		// Delete all objects not displayed in any windows
		void DeleteUnused(view3d::GuidPredCB pred);

		// Enumerate all sources in the store
		void EnumSources(view3d::EnumGuidsCB enum_guids_cb);

		// Return details about a source
		view3d::SourceInfo SourceInfo(Guid const& context_id);

		// Get/Set the name of a source
		string32 const& SourceName(Guid const& context_id);
		void SourceName(Guid const& context_id, std::string_view name);

		// Create a gizmo object and add it to the gizmo collection
		ldraw::LdrGizmo* GizmoCreate(ldraw::EGizmoMode mode, m4x4 const& o2w);

		// Destroy a gizmo
		void GizmoDelete(ldraw::LdrGizmo* gizmo);
		
		// Reload file sources
		void ReloadScriptSources();
		void ReloadScriptSources(std::span<Guid const> context_ids);

		// Poll for changed script source files, and reload any that have changed
		void CheckForChangedSources();

	protected:

		// Find the source associated with a context id
		ldraw::SourceBase const* FindSource(Guid const& context_id) const;
		ldraw::SourceBase* FindSource(Guid const& context_id);

		// Parse error event.
		void OnError(ldraw::ParseErrorEventArgs const&) override;

		// An event raised during parsing. This is called in the context of the threads that call 'AddFile'. Do not sign up while AddFile calls are running.
		void OnParsingProgress(ldraw::ParsingProgressEventArgs&) override;

		// Store change event. Called before and after a change to the collection of objects in the store.
		void OnStoreChange(ldraw::StoreChangeEventArgs const&) override;

		// Process any received commands in the source
		void OnHandleCommands(ldraw::SourceBase& source) override;
	};
}
