//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/ldraw/sources/source_base.h"

namespace pr::rdr12::ldraw
{
	SourceBase::SourceBase(Guid const* context_id)
		: m_name()
		, m_output()
		, m_context_id(context_id ? *context_id : GenerateGUID())
		, m_filepaths()
		, m_errors()
		, ParsingProgress()
	{
		m_name = To<string32>(m_context_id);
	}

	// Parse the contents of the script
	ParseResult SourceBase::Load(Renderer& rdr, std::stop_token stop_token) // worker thread context
	{
		// This function may be called synchronously or in a worker thread (it's the caller's choice).
		// This function simply returns a new ParseResults instance, it's up to the caller to manage
		// how the result is stored because they know when it's safe to delete the previous result.
		try
		{
			// Parse the script
			return ReadSource(rdr, std::move(stop_token));
		}
		catch (std::exception const& ex)
		{
			ParseErrorEventArgs err{ ex.what(), ldraw::EParseError::UnknownError, {} };
			m_errors.push_back(err);
			return {};
		}
	}

	// Regenerate the output from the source
	ParseResult SourceBase::ReadSource(Renderer&, std::stop_token)
	{
		return std::move(m_output);
	}
	
	// Create a stable Guid from a filepath
	Guid ContextIdFromFilepath(std::filesystem::path const& filepath)
	{
		static Guid const LdrawSourceFileNS = { 0xA9C66A7D, 0xD1F3, 0x4CFA, 0x84, 0xE0, 0xCF, 0x99, 0x12, 0xB3, 0x18, 0x9D };
		return GenerateGUID(LdrawSourceFileNS, filepath.lexically_normal().string().c_str());
	}

	// Callbacks for parsing
	void __stdcall SourceBase::OnReportError(void* ctx, EParseError err, Location const& loc, std::string_view msg)
	{
		auto this_ = static_cast<SourceBase*>(ctx);
		if (this_->m_errors.size() >= 100) throw std::runtime_error("Too many errors");
		this_->m_errors.push_back(ParseErrorEventArgs{ msg, err, loc });
	}
	bool __stdcall SourceBase::OnProgress(void* ctx, Guid const& context_id, ParseResult const& out, Location const& loc, bool complete)
	{
		auto this_ = static_cast<SourceBase*>(ctx);
		ParsingProgressEventArgs args{ context_id, out, loc, complete };
		this_->ParsingProgress(*this_, args);
		return !args.m_cancel;
	}
}

