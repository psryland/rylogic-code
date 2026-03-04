//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// A container of Ldr script sources that can watch for external changes.
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "view3d-12/src/ldraw/sources/source_base.h"

namespace pr::rdr12::ldraw
{
	// A container that doesn't invalidate on add/remove is needed because
	// the file watcher contains a pointer into the 'Source' objects.
	using SourceCont = std::unordered_map<Guid, std::shared_ptr<SourceBase>>;

	// Socket listening state
	enum class EStreamingState
	{
		Disconnected,
		Listening,
		Connected,
	};

	// Store change event args
	struct StoreChangeEventArgs
	{
		// The event that triggered the data change
		EDataChangeTrigger m_trigger;

		// The context ids that changed
		std::span<Guid const> m_context_ids;

		// Contains the results of parsing including the object container that the objects where added to
		ParseResult const* m_result;

		// True if this event is just prior to the changes being made to the store
		bool m_before;

		StoreChangeEventArgs(EDataChangeTrigger trigger, std::span<Guid const> context_ids, ParseResult const* result, bool before)
			: m_trigger(trigger)
			, m_context_ids(context_ids)
			, m_result(result)
			, m_before(before)
		{
		}
	};

	// Interface for handling source events
	struct ISourceEvents
	{
		virtual ~ISourceEvents() = default;

		// Parse error event.
		virtual void OnError(ParseErrorEventArgs const&) = 0;

		// An event raised during parsing. This is called in the context of the threads that call 'AddFile'. Do not sign up while AddFile calls are running.
		virtual void OnParsingProgress(ParsingProgressEventArgs&) = 0;

		// Store change event. Called before and after a change to the collection of objects in the store.
		virtual void OnStoreChange(StoreChangeEventArgs const&) = 0;

		// Process any received commands in the source. All commands are expected to be processed
		virtual void OnHandleCommands(SourceBase& source) = 0;
	};

	// Predicates for matching context ids
	inline bool __stdcall MatchContextId(void* ctx, Guid const& id)
	{
		return *static_cast<Guid const*>(ctx) == id;
	}
	inline bool __stdcall MatchContextIdInSpan(void* ctx, Guid const& id)
	{
		auto ids = *static_cast<std::span<Guid const>*>(ctx);
		return std::ranges::find(ids, id) != end(ids);
	}

	// A collection of LDraw script sources
	struct ScriptSources :IFileChangedHandler
	{
		// Notes:
		//  - A collection of sources of ldr objects.
		//  - Typically ldraw sources are files, but string sources and stream sources are
		//    also supported.
		//  - This class maintains a map from context ids to a collection of sources.
		//  - All sources have a unique context id. When reloaded, objects previously
		//    associated with that file context id are removed. String scripts have a user
		//    provided id. String scripts are not reloaded because they shouldn't change
		//    externally. Callers should manage the removal of objects associated with string
		//    script sources.
		//  - This class manages the file watching/reload mechanism because when an included
		//    file changes, and reload of the root file is needed, even if unchanged.
		//  - If a file in a context id set has changed, an event is raised allowing the
		//    change to be ignored. The event args contain the context id and list of
		//    associated files.

		using GuidCont = pr::vector<Guid>;
		using GuidSet = pr::AsyncWrap<std::unordered_map<Guid, std::stop_source, std::hash<Guid>>>;
		using filepath_t = std::filesystem::path;
		
	private:

		SourceCont        m_srcs;           // The sources of ldr script
		GizmoCont         m_gizmos;         // The created ldr gizmos
		Renderer*         m_rdr;            // Renderer used to create models
		ISourceEvents*    m_events;         // Event handler
		Winsock           m_winsock;        // The 'winsock' instance we're bound to
		GuidSet           m_loading;        // Context ids in the process of being loaded
		FileWatch         m_watcher;        // The watcher of files
		std::jthread      m_listen_thread;  // Thread that listens for incoming connections
		std::thread::id   m_main_thread_id; // The main thread id
		uint16_t          m_listen_port;    // The port we're listening on
		std::atomic<bool> m_shutting_down;  // Set during destruction to prevent new loads and signal workers

	public:

		explicit ScriptSources(Renderer& rdr, ISourceEvents& events);
		~ScriptSources();

		// Renderer access
		Renderer& rdr() const;

		// The ldr script sources
		SourceCont const& Sources() const;

		// The store of gizmos
		GizmoCont const& Gizmos() const;

		// Remove all objects and sources
		void ClearAll();

		// Remove a single object from the object container
		void Remove(LdrObject* object, EDataChangeTrigger trigger = EDataChangeTrigger::Removal);

		// Remove all objects associated with 'context_ids'
		void Remove(std::function<bool(Guid const&)> pred, EDataChangeTrigger trigger = EDataChangeTrigger::Removal);
		void Remove(Guid const& context_id, EDataChangeTrigger trigger = EDataChangeTrigger::Removal);

		// Reload a range of sources
		void Reload(std::span<Guid const> ids);

		// Reload all sources
		void Reload();

		// Check all file sources for modifications and reload any that have changed
		void RefreshChangedFiles();

		// Add an object created externally
		Guid Add(LdrObjectPtr object);

		// Parse a string containing ldraw script.
		// This function can be called from any thread and may be called concurrently by multiple threads.
		// Returns the GUID of the context that the objects were added to.
		template <typename Char>
		Guid AddString(std::basic_string_view<Char> script, EEncoding enc, Guid const* context_id, PathResolver const& includes, AddCompleteCB add_complete);

		// Parse file containing ldraw script.
		// This function can be called from any thread and may be called concurrently by multiple threads.
		// Returns the GUID of the context that the objects were added to.
		Guid AddFile(std::filesystem::path script, EEncoding enc, Guid const* context_id, PathResolver const& includes, AddCompleteCB add_complete);

		// Parse binary data containing ldraw script
		// This function can be called from any thread and may be called concurrently by multiple threads.
		// Returns the GUID of the context that the objects were added to.
		Guid AddBinary(std::span<std::byte const> data, Guid const* context_id, AddCompleteCB add_complete);

		// Cancel an in-progress load operation. Thread-safe.
		void CancelLoad(Guid const& context_id);

		// The state of the streaming connection
		EStreamingState StreamingState() const;

		// Allow connections on 'port'
		void AllowConnections(uint16_t listen_port);

		// Close all connections and stop listening
		void StopConnections();

		// Create a gizmo object and add it to the gizmo collection
		LdrGizmo* CreateGizmo(EGizmoMode mode, m4x4 const& o2w);

		// Destroy a gizmo
		void RemoveGizmo(LdrGizmo* gizmo);

	private:

		// 'filepath' is the name of the changed file
		void FileWatch_OnFileChanged(wchar_t const*, Guid const& context_id, void*, bool&);

		// Handler for when new data is received from a source
		void SourceNotifyHandler(std::shared_ptr<SourceBase> src, NotifyEventArgs const& args);
	};
}
