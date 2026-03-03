//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/ldraw/sources/ldraw_sources.h"
#include "view3d-12/src/ldraw/sources/source_base.h"
#include "view3d-12/src/ldraw/sources/source_string.h"
#include "view3d-12/src/ldraw/sources/source_binary.h"
#include "view3d-12/src/ldraw/sources/source_file.h"
#include "view3d-12/src/ldraw/sources/source_stream.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_gizmo.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/ldraw/ldraw_reader_binary.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/main/renderer.h"

namespace pr::rdr12::ldraw
{
	ScriptSources::ScriptSources(Renderer& rdr, ISourceEvents& events)
		: m_srcs()
		, m_gizmos()
		, m_rdr(&rdr)
		, m_events(&events)
		, m_winsock()
		, m_loading()
		, m_watcher()
		, m_listen_thread()
		, m_main_thread_id(std::this_thread::get_id())
		, m_listen_port()
		, m_shutdown_latch()
	{
		m_rdr->AddPollCB(m_watcher.PollCB, seconds_t(1.0));
	}
	ScriptSources::~ScriptSources()
	{
		// Cancel all in-progress loads and wait for worker threads to finish
		if (auto loading = m_loading.lock())
		{
			// Create a latch with count+1: workers count down once each, and we count down
			// the extra 1 after setting up. This avoids UB if count is 0 (latch(0) + count_down = UB).
			auto count = static_cast<ptrdiff_t>(loading->size());
			m_shutdown_latch.emplace(count + 1);

			// Signal all workers to stop
			for (auto& [id, ss] : *loading)
				ss.request_stop();

			// Count down the +1 we added, then release the mutex
			m_shutdown_latch->count_down();
		}
		
		// Wait for all worker threads to count down the latch
		m_shutdown_latch->wait();

		m_rdr->RemovePollCB(m_watcher.PollCB);
		StopConnections();
	}

	// Renderer access
	Renderer& ScriptSources::rdr() const
	{
		return *m_rdr;
	}

	// The ldr script sources
	SourceCont const& ScriptSources::Sources() const
	{
		return m_srcs;
	}

	// The store of gizmos
	GizmoCont const& ScriptSources::Gizmos() const
	{
		return m_gizmos;
	}

	// Remove all objects and sources
	void ScriptSources::ClearAll()
	{
		assert(std::this_thread::get_id() == m_main_thread_id);

		// Make a copy of the GUIDs removed
		GuidCont guids;
		for (auto& src : m_srcs)
			guids.push_back(src.first);

		m_srcs.clear();
		m_gizmos.clear();
		m_watcher.RemoveAll();

		// Notify of the object container change
		m_events->OnStoreChange({EDataChangeTrigger::Removal, guids, nullptr, false});
	}

	// Remove a single object from the store
	void ScriptSources::Remove(LdrObject* object, EDataChangeTrigger trigger)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		auto id = object->m_context_id;

		// Remove the object from the source it belongs to
		auto& src = m_srcs[id];
		auto count = src->m_output.m_objects.size();
		ldraw::Remove(src->m_output.m_objects, object);

		// Notify of the object container change
		if (src->m_output.m_objects.size() != count)
			m_events->OnStoreChange({ trigger, {&id, 1}, nullptr, false });

		// If that was the last object for the source, remove the source too
		if (src->m_output.m_objects.empty())
			Remove(id);
	}

	// Remove all objects associated with 'context_ids'
	void ScriptSources::Remove(std::function<bool(Guid const&)> pred, EDataChangeTrigger trigger)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);

		// Build the set of ids to remove
		GuidCont removed;
		for (auto& src : m_srcs)
		{
			auto const& id = src.second->m_context_id;
			if (!pred(id))
				continue;

			removed.push_back(id);
		}

		// Notify of the object container about to change
		if (!removed.empty())
		{
			m_events->OnStoreChange({ trigger, removed, nullptr, true });
		}

		// Remove the sources
		for (auto& id : removed)
		{
			// Delete any associated files and watches
			m_watcher.RemoveAll(id);

			// Delete the source and its associated objects
			m_srcs.erase(id);
		}

		// Notify of the object container change
		if (!removed.empty())
		{
			m_events->OnStoreChange({ trigger, removed, nullptr, false });
		}
	}
	void ScriptSources::Remove(Guid const& context_id, EDataChangeTrigger trigger)
	{
		Remove([&](Guid const& id) { return context_id == id; }, trigger);
	}

	// Reload a range of sources
	void ScriptSources::Reload(std::span<Guid const> ids_)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);
		pr::vector<Guid> ids = ids_;

		// Notify of a reload about to start
		m_events->OnStoreChange({ EDataChangeTrigger::Reload, ids, nullptr, true });

		// Reload each source in a background thread
		std::for_each(std::execution::par, std::begin(ids), std::end(ids), [this](Guid const& id)
		{
			auto& src = m_srcs[id];
			auto output = src->Load(rdr());
			src->Notify(src, NotifyEventArgs{ std::move(output), ENotifyReason::LoadComplete, EDataChangeTrigger::Reload, nullptr });
		});

		// Queue a notify load complete after all reloads have been queued
		rdr().RunOnMainThread([this, ids = std::move(ids)]() mutable noexcept
		{
			m_events->OnStoreChange({ EDataChangeTrigger::Reload, ids, nullptr, false });
		});
	}

	// Reload all sources
	void ScriptSources::Reload()
	{
		GuidCont ids;
		for (auto& src : m_srcs)
			ids.push_back(src.first);

		Reload(ids);
	}

	// Check all file sources for modifications and reload any that have changed
	void ScriptSources::RefreshChangedFiles()
	{
		m_watcher.CheckForChangedFiles();
	}

	// Add an object created externally
	Guid ScriptSources::Add(LdrObjectPtr object) // worker thread context
	{
		// Create a source wrapper for this object
		auto src = std::shared_ptr<SourceBase>(new SourceBase{ &object->m_context_id });
		src->Notify += std::bind(&ScriptSources::SourceNotifyHandler, this, _1, _2);
		src->m_output.m_objects.push_back(object);
		
		// Parse the script
		auto output = src->Load(rdr());
		src->Notify(src, NotifyEventArgs{ std::move(output), ENotifyReason::LoadComplete, EDataChangeTrigger::NewData, nullptr });
		return src->m_context_id;
	}

	// Parse a string containing ldr script.
	// This function can be called from any thread and may be called concurrently by multiple threads.
	// Returns the GUID of the context that the objects were added to.
	template <typename Char>
	Guid ScriptSources::AddString(std::basic_string_view<Char> script, EEncoding enc, Guid const* context_id, PathResolver const& includes, AddCompleteCB add_complete) // worker thread context
	{
		// Note: when called from a worker thread, this function returns after objects have
		// been created, but before they've been added to the main 'm_srcs' collection.
		// The 'add_complete' callback function should be used as a continuation function.
		std::stop_token ss_token;
		std::shared_ptr<SourceString<Char>> src;
		if (auto loading = m_loading.lock())
		{
			// Ignore 'Add' calls during shutdown
			if (m_shutdown_latch)
				return {};

			// On cleanup, remove from 'loading' and count down the shutdown latch if it's set
			src.reset(new SourceString<Char>{ context_id, script, enc, includes }, [this](auto* ptr)
			{
				if (auto loading = m_loading.lock())
				{
					loading->erase(ptr->m_context_id);
					if (m_shutdown_latch)
						m_shutdown_latch->count_down();
				}
			});

			// Insert a stop source for this context id, so that workers can check for shutdown
			std::stop_source ss;
			ss_token = ss.get_token();
			loading->emplace(src->m_context_id, std::move(ss));
		}

		src->ParsingProgress += [this](auto&, auto& args) { m_events->OnParsingProgress(args); };

		// Start a task to 'parse' it
		auto output = src->Load(rdr(), ss_token);
		src->Notify += std::bind(&ScriptSources::SourceNotifyHandler, this, _1, _2);
		src->Notify(src, NotifyEventArgs{ std::move(output), ENotifyReason::LoadComplete, EDataChangeTrigger::NewData, add_complete });
		return src->m_context_id;
	}
	template Guid ScriptSources::AddString<wchar_t>(std::wstring_view script, EEncoding enc, Guid const* context_id, PathResolver const& includes, AddCompleteCB add_complete);
	template Guid ScriptSources::AddString<char>(std::string_view script, EEncoding enc, Guid const* context_id, PathResolver const& includes, AddCompleteCB add_complete);

	// Parse file containing ldr script.
	// This function can be called from any thread and may be called concurrently by multiple threads.
	// Returns the GUID of the context that the objects were added to.
	Guid ScriptSources::AddFile(std::filesystem::path filepath, EEncoding enc, Guid const* context_id, PathResolver const& includes, AddCompleteCB add_complete) // worker thread context
	{
		// Note: when called from a worker thread, this function returns after objects have
		// been created, but before they've been added to the main 'm_srcs' collection.
		// The 'add_complete' callback function should be used as a continuation function.
		std::stop_token ss_token;
		std::shared_ptr<SourceFile> src;
		if (auto loading = m_loading.lock())
		{
			// Ignore 'Add' calls during shutdown
			if (m_shutdown_latch)
				return {};

			// On cleanup, remove from 'loading' and count down the shutdown latch if it's set
			src.reset(new SourceFile{ context_id, filepath, enc, includes }, [this](auto* ptr)
			{
				if (auto loading = m_loading.lock())
				{
					loading->erase(ptr->m_context_id);
					if (m_shutdown_latch)
						m_shutdown_latch->count_down();
				}
			});

			// Insert a stop source for this context id, so that workers can check for shutdown
			std::stop_source ss;
			ss_token = ss.get_token();
			loading->emplace(src->m_context_id, std::move(ss));
		}

		src->ParsingProgress += [this](auto&, auto& args) { m_events->OnParsingProgress(args); };

		// Start a task to 'parse' it
		auto output = src->Load(rdr(), ss_token);
		src->Notify += std::bind(&ScriptSources::SourceNotifyHandler, this, _1, _2);
		src->Notify(src, NotifyEventArgs{ std::move(output), ENotifyReason::LoadComplete, EDataChangeTrigger::NewData, add_complete });
		return src->m_context_id;
	}

	// Parse binary data containing ldraw script
	// This function can be called from any thread and may be called concurrently by multiple threads.
	// Returns the GUID of the context that the objects were added to.
	Guid ScriptSources::AddBinary(std::span<std::byte const> data, Guid const* context_id, AddCompleteCB add_complete)
	{
		// Note: when called from a worker thread, this function returns after objects have
		// been created, but before they've been added to the main 'm_srcs' collection.
		// The 'add_complete' callback function should be used as a continuation function.
		std::stop_token ss_token;
		std::shared_ptr<SourceBinary> src;
		if (auto loading = m_loading.lock())
		{
			// Ignore 'Add' calls during shutdown
			if (m_shutdown_latch)
				return {};

			// On cleanup, remove from 'loading' and count down the shutdown latch if it's set
			src.reset(new SourceBinary{ context_id, data }, [this](auto* ptr)
			{
				if (auto loading = m_loading.lock())
				{
					loading->erase(ptr->m_context_id);
					if (m_shutdown_latch)
						m_shutdown_latch->count_down();
				}
			});

			// Insert a stop source for this context id, so that workers can check for shutdown
			std::stop_source ss;
			ss_token = ss.get_token();
			loading->emplace(src->m_context_id, std::move(ss));
		}

		src->ParsingProgress += [this](auto&, auto& args) { m_events->OnParsingProgress(args); };

		// Start a task to 'parse' it
		auto output = src->Load(rdr(), ss_token);
		src->Notify += std::bind(&ScriptSources::SourceNotifyHandler, this, _1, _2);
		src->Notify(src, NotifyEventArgs{ std::move(output), ENotifyReason::LoadComplete, EDataChangeTrigger::NewData, add_complete });
		return src->m_context_id;
	}

	// Cancel an in-progress load operation
	void ScriptSources::CancelLoad(Guid const& context_id)
	{
		if (auto loading = m_loading.lock())
		{
			auto it = loading->find(context_id);
			if (it != std::end(*loading))
				it->second.request_stop();
		}
	}

	// The state of the streaming connection
	EStreamingState ScriptSources::StreamingState() const
	{
		return
			!m_listen_thread.joinable() ? EStreamingState::Disconnected :
			std::ranges::any_of(m_srcs, [](auto& src) { return dynamic_cast<SourceStream const*>(src.second.get()) != nullptr; }) ? EStreamingState::Connected :
			EStreamingState::Listening;
	}

	// Allow connections on 'port'
	void ScriptSources::AllowConnections(uint16_t listen_port)
	{
		using Socket = network::Socket;

		StopConnections();

		// Start the thread for incoming connections
		m_listen_port = listen_port;
		m_listen_thread = std::jthread([this, listen_port]()
		{
			threads::SetCurrentThreadName("Stream Sources Listen Thread");
			enum class EState { Disconnected, Idle, Listening, Broken } state = EState::Disconnected;

			Socket listen_socket;

			// Check for client connections to the server and dump old connections
			for (;!m_listen_thread.get_stop_token().stop_requested();)
			{
				// Don't exit the thread unless shutting down.
				// Try to handle re-connections, and other errors gracefully.
				try
				{
					switch (state)
					{
						case EState::Disconnected:
						{
							// Create the listen socket. If this fails with WSAEACCESS, it's probably because the firewall is blocking it
							listen_socket = Socket(::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
							if (listen_socket == nullptr)
								network::Throw(WSAGetLastError());

							// Bind the local address to the socket
							sockaddr_in my_address = {};
							my_address.sin_family = AF_INET;
							my_address.sin_addr.S_un.S_addr = INADDR_ANY;
							my_address.sin_port = htons(static_cast<u_short>(listen_port));
							auto result = ::bind(listen_socket, (sockaddr const*)&my_address, sizeof(my_address));
							if (result == SOCKET_ERROR)
								network::Throw(WSAGetLastError());

							state = EState::Idle;
							break;
						}
						case EState::Idle:
						{
							// Start listening for incoming connections
							auto result = ::listen(listen_socket, SOMAXCONN);
							if (result != SOCKET_ERROR || WSAGetLastError() == WSAEISCONN) // No error, or already connected
							{
								state = EState::Listening;
								break;
							}

							// Listen failed, check the error code
							auto code = WSAGetLastError();
							switch (code)
							{
								case WSAEINPROGRESS: // A blocking Windows Sockets 1.1 call is in progress, or the service provider is still processing a callback function.
								case WSAENETDOWN:    // The network subsystem has failed.
								case WSAEWOULDBLOCK:
								{
									// Retry after a delay
									std::this_thread::sleep_for(std::chrono::milliseconds(200));
									break;
								}
								default:
								{
									network::Throw(code);
									break;
								}
							}
							break;
						}
						case EState::Listening:
						{
							// Wait for new connections
							if (network::SelectToRecv(listen_socket, 5000))
							{
								// Someone is trying to connect
								sockaddr_in client_addr = {};
								auto client_addr_size = static_cast<int>(sizeof(client_addr));
								auto client = Socket(::accept(listen_socket, (sockaddr*)&client_addr, &client_addr_size));
								network::Check(client != INVALID_SOCKET, "Accepting connection failed");

								// Add this connection as a new source. SourceStream streams start their own thread internally.
								// We just need to call 'SourceNotifyHandler' to register it as a source.
								auto src = std::shared_ptr<SourceStream>(new SourceStream{ nullptr, &rdr(), std::move(client), client_addr });
								src->Notify += std::bind(&ScriptSources::SourceNotifyHandler, this, _1, _2);
								src->Notify(src, NotifyEventArgs{ {}, ENotifyReason::LoadComplete, EDataChangeTrigger::NewData, nullptr });
							}
							break;
						}
						case EState::Broken:
						{
							// Clean up the broken socket
							listen_socket = nullptr;
							state = EState::Disconnected;
							break;
						}
						default:
						{
							throw std::runtime_error("Unknown state");
						}
					}
				}
				catch (std::exception const& ex)
				{
					assert(false && ex.what()); (void)ex;
					state = EState::Broken;
				}
				catch (...)
				{
					assert(false && "Unhandled exception in TCP listen thread");
					state = EState::Broken;
				}
			}

			// Clean up the broken socket
			listen_socket = nullptr;
			state = EState::Disconnected;
		});
	}

	// Close all connections and stop listening
	void ScriptSources::StopConnections()
	{
		// Stop the incoming connections thread
		m_listen_thread.get_stop_source().request_stop();
		if (m_listen_thread.joinable())
			m_listen_thread.join();

		// Remove any stream sources
		std::erase_if(m_srcs, [](auto& src)
		{
			return dynamic_cast<SourceStream*>(src.second.get()) != nullptr;
		});
	}

	// Create a gizmo object and add it to the gizmo collection
	LdrGizmo* ScriptSources::CreateGizmo(EGizmoMode mode, m4x4 const& o2w)
	{
		LdrGizmoPtr giz(new LdrGizmo(rdr(), mode, o2w), true);
		m_gizmos.push_back(giz);
		return giz.m_ptr;
	}

	// Destroy a gizmo
	void ScriptSources::RemoveGizmo(LdrGizmo* gizmo)
	{
		// Delete the gizmo from the gizmo container (removing the last reference)
		erase_first(m_gizmos, [&](LdrGizmoPtr const& p){ return p.m_ptr == gizmo; });
	}

	// 'filepath' is the name of the changed file
	void ScriptSources::FileWatch_OnFileChanged(wchar_t const*, Guid const& context_id, void*, bool&)
	{
		assert(std::this_thread::get_id() == m_main_thread_id);

		// Look for a source that matches 'context_id'
		auto iter = m_srcs.find(context_id);
		if (iter == std::end(m_srcs))
			return;

		// Create a stop token for this load operation
		std::stop_token ss_token;
		if (auto loading = m_loading.lock())
		{
			// Skip files that are already in the process of loading
			if (loading->find(context_id) != std::end(*loading))
				return;

			std::stop_source ss;
			ss_token = ss.get_token();
			loading->emplace(context_id, std::move(ss));
		}

		// Reload that file group (asynchronously)
		std::jthread([this, src = iter->second, context_id, ss_token]() mutable
		{
			// Note: if loading a file fails, don't use 'MarkAsChanged' to trigger another load
			// attempt. Doing so results in an infinite loop trying to load a broken file.
			auto output = src->Load(rdr(), ss_token);
			src->Notify(src, NotifyEventArgs{ std::move(output), ENotifyReason::LoadComplete, EDataChangeTrigger::Reload, nullptr });
		}).detach();
	}

	// Handler for when new data is received from a source
	void ScriptSources::SourceNotifyHandler(std::shared_ptr<SourceBase> src, NotifyEventArgs const& args) // worker thread context
	{
		// Notes:
		//  - This function is called by worker threads when their source has completed loading.
		//    **It can be called on any thread context**.
		//  - Sources have a Load function that generates a ParseResult (new instance)
		//  - Load should be threadsafe so it can be called in parallel on all sources.
		//  - Once the new parse result is ready, 'Notify' on the source is called to tell
		//    this function to add the new (or reloaded) data.
		//  - This function merges or replaces the data for 'src'. Note that the old data
		//    remains in scope until after the last StoreChanged event so that callers can
		//    still reference the old data if needed.
		//  - V3dWindows watch for the store changed event and manage their lists of objects.
		//  - In C#, References to 'Object' need to be kept in sync with the native code by
		//    watching for the SceneChanged events.

		// Marshal to the main thread.
		if (std::this_thread::get_id() != m_main_thread_id)
		{
			// Note: During shutdown, the main thread is waiting for all worker threads to finish.
			// Don't queue callbacks that might be called after this ScriptSources object is deleted.
			if (auto loading = m_loading.lock())
			{
				// Don't queue callbacks during shutdown
				if (m_shutdown_latch)
					return;

				// Make a copy of the 'args' cause the caller is going out of scope.
				// Copying 'ParseResult' isn't too back, because the vectors are just pointers to objects.
				return rdr().RunOnMainThread([this, src, args = NotifyEventArgs(args)]() mutable noexcept
				{
					SourceNotifyHandler(src, args);
				});
			}
		}

		// Should be on the main thread now
		assert(std::this_thread::get_id() == m_main_thread_id);
		auto context_id = src->m_context_id;

		switch (args.m_reason)
		{
			case ENotifyReason::LoadComplete:
			{
				// Don't remove previous objects associated with 'context',
				// leave that to the caller via the 'on_add' callback.

				// Notify of the store about to change
				m_events->OnStoreChange({ args.m_trigger, { &context_id, 1 }, &args.m_output, true });
				if (args.m_add_complete) args.m_add_complete(context_id, true);

				// Add any dependent files to the file watcher
				for (auto& fp : src->m_filepaths)
					m_watcher.Add(fp.c_str(), this, context_id);

				// Notify of any errors that occurred
				for (auto& err : src->m_errors)
					m_events->OnError(err);

				// Update the store
				auto& existing = m_srcs[context_id];
				if (existing == nullptr)
					existing = src;

				// Remove existing data if this is a reload but keep it alive until the final StoreChange event is finished.
				ParseResult previous_data;
				if (args.m_trigger == EDataChangeTrigger::Reload)
					std::swap(previous_data, existing->m_output);

				// Merged the output with the existing output.
				// This is a merge, rather than a replace, because stream sources add data incrementally
				existing->m_output += args.m_output;

				// Notify of the store change
				m_events->OnStoreChange({ args.m_trigger, { &context_id, 1 }, &existing->m_output, false });
				if (args.m_add_complete) args.m_add_complete(context_id, false);

				// Process any commands
				if (!existing->m_output.m_commands.empty())
					m_events->OnHandleCommands(*existing);

				break;
			}
			case ENotifyReason::Disconnected:
			{
				// 'source' has disconnected.
				Remove(src->m_context_id);
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown notify reason");
			}
		}
	}
}
