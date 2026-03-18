//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"

namespace pr::rdr12::ldraw
{
	// Callback after data has been added to the store
	using AddCompleteCB = std::function<void(Guid const&, bool)>;

	// The event that triggered the store change
	enum class EStoreChangeInitiator
	{
		// PR_CODE_SYNC_BEGIN(pr::rdr12::ldraw::EStoreChangeInitiator, source_of_truth)

		// A new source (e.g. file, string, stream, etc) was added to the store
		NewSource,

		// A source was removed from the store
		SourceRemoved,

		// Existing sources refreshed their data
		Reload,

		// More data for an existing source was added (typically from streaming sources)
		AppendData,

		// The DeleteObject API call was made
		ObjectsDeleted,
	
		// PR_CODE_SYNC_END();
	};

	// Flags describing how the store was changed
	enum class EStoreChangeFlags
	{
		// PR_CODE_SYNC_BEGIN(pr::rdr12::ldraw::EStoreChangeFlags, source_of_truth)

		None = 0,

		// One or more objects were removed
		ObjectsAdded = 1 << 0,

		// One or more objects were removed
		ObjectsRemoved = 1 << 1,

		// A new context id was added to the store
		ContextIdAdded = 1 << 2,

		// A context id was removed from the store
		ContextIdRemoved = 1 << 3,

		// Objects in the store were refreshed from the sources (e.g. after a Load() call)
		ExistingObjectsRefreshed = 1 << 4,

		ObjectsChanged = ObjectsAdded | ObjectsRemoved,
		ContextIdsChanged = ContextIdAdded | ContextIdRemoved,

		_flags_enum = 0,

		// PR_CODE_SYNC_END();
	};

	// Event args for the SourceBase Notify event
	struct NotifyEventArgs
	{
		// The load results
		ParseResult m_output;

		// The original trigger that initiated the data change in the store
		EStoreChangeInitiator m_initiator;

		// What changed in the store
		EStoreChangeFlags m_change_flags;

		// Called after data has been added to the store
		AddCompleteCB m_add_complete;
	};

	// Base class for a source of LDraw objects
	struct SourceBase : std::enable_shared_from_this<SourceBase>
	{
		// Notes:
		//  - Sources are containers of LdrObjects associated with a GUID context id.
		//  - Sources do their parsing an a background thread, returning a new 'ParseResult' object.
		//  - Sources fire the 'NewData' event when new data is ready (e.g. after a Reload)

		using filepath_t = std::filesystem::path;
		using PathsCont = pr::vector<filepath_t>;
		using ErrorCont = pr::vector<ParseErrorEventArgs>;

		string32    m_name;       // A name associated with this source
		ParseResult m_output;     // Objects created by this source
		Guid        m_context_id; // Id for the group of files that this object is part of
		PathsCont   m_filepaths;  // Dependent files of this source
		ErrorCont   m_errors;

		SourceBase(Guid const* context_id);
		SourceBase(SourceBase&&) = default;
		SourceBase(SourceBase const&) = default;
		SourceBase& operator=(SourceBase&&) = default;
		SourceBase& operator=(SourceBase const&) = default;
		virtual ~SourceBase() = default;

		// Stop any background activity. Called before destruction to prevent
		// races between worker threads and shared_ptr release.
		virtual void Stop() {}

		// An event raised during parsing.
		EventHandler<SourceBase&, ParsingProgressEventArgs&, true> ParsingProgress;

		// Parse the contents of the script from this source.
		ParseResult Load(Renderer& rdr, std::stop_token stop_token = {});

		// An event raised when something happens with this source (e.g, has new data, disconnected, etc)
		// This is called from outside the class because the 'Load' method cannot both return and move the result
		// into the notify. Not all callers want to handle notify.
		EventHandler<std::shared_ptr<SourceBase>, NotifyEventArgs const&, true> Notify;

	protected:

		// Callbacks for parsing
		static void __stdcall OnReportError(void* ctx, EParseError err, Location const& loc, std::string_view msg);
		static bool __stdcall OnProgress(void* ctx, Guid const& context_id, ParseResult const& out, Location const& loc, bool complete);

		// Regenerate the output from the source
		virtual ParseResult ReadSource(Renderer& rdr, std::stop_token stop_token);
	};

	// Create a stable Guid from a filepath
	Guid ContextIdFromFilepath(std::filesystem::path const& filepath);
}
