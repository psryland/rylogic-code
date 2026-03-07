//*****************************************
// XmlLite wrapper
//	Copyright (C) Paul Ryland 2009
//*****************************************
// Required lib: xmllite.lib

#pragma once

#include <vector>
#include <string>
#include <locale>
#include <sstream>
#include <exception>
#include <algorithm>
#include <xmllite.h>
#include <Shlwapi.h>
#include "pr/common/flags_enum.h"
#include "pr/common/refptr.h"
#include "pr/str/string_core.h"

#pragma comment(lib, "xmllite.lib")
#pragma comment(lib, "shlwapi.lib")

namespace pr::xml
{
	enum class EProperty
	{
		None               = 0,
		MultiLanguage      = 1 << XmlWriterProperty_MultiLanguage,
		Indent             = 1 << XmlWriterProperty_Indent,
		ByteOrderMark      = 1 << XmlWriterProperty_ByteOrderMark,
		OmitXmlDeclaration = 1 << XmlWriterProperty_OmitXmlDeclaration,
		ConformanceLevel   = 1 << XmlWriterProperty_ConformanceLevel,
		_flags_enum = 0,
	};

	// Forward declarations
	struct Attr;
	struct Node;
	using HashValue = int;
	using StrVec = std::vector<std::wstring>;
	using AttrVec = std::vector<Attr>;
	using NodeVec = std::vector<Node>;
	template <typename T> using Ptr = pr::RefPtr<T>;

	// Convert a wide string into a hash code (crc32 algorithm)
	inline HashValue Hash(std::wstring const& str)
	{
		unsigned char const* ptr = reinterpret_cast<unsigned char const*>(str.c_str());
		unsigned char const* end = ptr + str.size()*sizeof(*str.c_str());
		unsigned int crc = 0xFFFFFFFF;
		for (; ptr != end; ++ptr)
		{
			unsigned char b = static_cast<unsigned char>(crc ^ *ptr);
			unsigned int value = 0xFF ^ b;
			for (int j = 8; j > 0; --j) { value = (value >> 1) ^ ((value & 1) ? 0xEDB88320 : 0); }
			value = value ^ 0xFF000000;
			crc = value ^ (crc >> 8);
		}

		// using 'HashValue' as a signed int so that enum's declared equal to this
		// hash values do not have signed/unsigned warning problems
		return reinterpret_cast<HashValue const&>(crc);
	}

	// Helpers for converting things to 'std::wstring'
	inline std::wstring const& str(std::wstring const& s) { return s; }
	inline std::wstring str(std::string const& s)         { return std::wstring(s.begin(), s.end()); }
	inline std::wstring str(char const* s)                { return std::wstring(s, s + strlen(s)); }
	template <typename T> inline std::wstring str(T v)    { std::wstringstream s; s << v; return s.str(); }

	// An XML element attribute
	struct Attr
	{
		std::wstring m_prefix;
		std::wstring m_localname;
		std::wstring m_value;
			
		Attr(std::wstring const& prefix, std::wstring const& localname, std::wstring const& value)
			:m_prefix(prefix)
			,m_localname(localname)
			,m_value(value)
		{}

		// Return the tag name for this attribute
		std::wstring tag() const
		{
			return m_prefix + (m_prefix.empty() ? L"" : L":") + m_localname;
		}

		// Return the value of this attribute
		std::wstring value() const
		{
			return m_value;
		}
	};

	// An XML element
	struct Node
	{
		std::wstring m_prefix;     // The tag prefix for the element
		std::wstring m_tag;        // The tag for the element
		std::wstring m_value;      // The value of this element
		bool         m_cdata;      // true if m_value is literal data [CDATA[...]]
		NodeVec      m_child;      // Child elements
		AttrVec      m_attr;       // Attributes
		AttrVec      m_proc_instr; // Processing instructions
		StrVec       m_comments;   // Comments

		Node()
			:m_tag(L"root")
			, m_cdata(false)
		{}
		template <typename Str1>
		explicit Node(Str1 const& tag)
			:m_prefix()
			,m_tag(str(tag))
			,m_value()
			,m_cdata(false)
		{}
		template <typename Str1, typename Str2>
		Node(Str1 const& tag, Str2 const& value)
			:m_prefix()
			,m_tag(str(tag))
			,m_value(str(value))
			,m_cdata(false)
		{}
		template <typename Str1, typename Str2, typename Str3>
		Node(Str1 const& prefix, Str2 const& tag, Str3 const& value, bool cdata = false)
			:m_prefix(str(prefix))
			,m_tag(str(tag))
			,m_value(str(value))
			,m_cdata(cdata)
		{}

		// Convert the contained value to a type
		template <typename Type> Type as() const;
		template <> bool              as() const { return _wcsicmp(m_value.c_str(), L"true") == 0 || _wtoi(m_value.c_str()) != 0; }
		template <> char              as() const { return static_cast<char>(as<int>()); }
		template <> unsigned char     as() const { return static_cast<unsigned char>(as<unsigned int>()); }
		template <> short             as() const { return static_cast<short>(as<int>()); }
		template <> unsigned short    as() const { return static_cast<unsigned short>(as<unsigned int>()); }
		template <> int               as() const { return _wtoi(m_value.c_str()); }
		template <> unsigned int      as() const { return static_cast<unsigned int>(_wtoi(m_value.c_str())); }
		template <> float             as() const { return static_cast<float>(_wtof(m_value.c_str())); }
		template <> double            as() const { return _wtof(m_value.c_str()); }
		template <> std::string       as() const { return pr::Narrow(m_value); }
		template <> std::wstring      as() const { return m_value; }

		// Child node access
		NodeVec::const_iterator begin() const { return m_child.begin(); }
		NodeVec::const_iterator end() const   { return m_child.end(); }
		NodeVec::iterator begin()             { return m_child.begin(); }
		NodeVec::iterator end()               { return m_child.end(); }

		// Return the tag name for this node
		std::wstring tag() const
		{
			return m_prefix + (m_prefix.empty() ? L"" : L":") + m_tag;
		}

		// Return a hash value for the name of this node
		HashValue hash() const
		{
			return Hash(tag());
		}

		// Add a child node to this node. Returns the added child.
		Node& add(Node const& node)
		{
			m_child.push_back(node);
			return m_child.back();
		}

		// Return the first element with a name matching 'name'
		Node const* element(std::wstring const& tag) const
		{
			auto iter = std::find_if(std::begin(m_child), std::end(m_child), [&](Node const& n){ return n.tag() == tag; });
			return iter != std::end(m_child) ? &*iter : nullptr;
		}
		Node* element(std::wstring const& tag)
		{
			auto iter = std::find_if(std::begin(m_child), std::end(m_child), [&](Node const& n){ return n.tag() == tag; });
			return iter != std::end(m_child) ? &*iter : nullptr;
		}

		// Return the first element with a name matching 'name' that passes 'pred'
		template <typename Pred> Node const* element(std::wstring const& tag, Pred pred) const
		{
			auto iter = std::find_if(std::begin(m_child), std::end(m_child), [&](Node const& n){ return n.tag() == tag && pred(n); });
			return iter != std::end(m_child) ? &*iter : nullptr;
		}
		template <typename Pred> Node* element(std::wstring const& tag, Pred pred)
		{
			auto iter = std::find_if(std::begin(m_child), std::end(m_child), [&](Node const& n){ return n.tag() == tag && pred(n); });
			return iter != std::end(m_child) ? &*iter : nullptr;
		}

		// Return the first attribute with a name matching 'name'
		Attr const* attribute(std::wstring const& tag) const
		{
			auto iter = std::find_if(std::begin(m_attr), std::end(m_attr), [&](Attr const& a){ return a.tag() == tag; });
			return iter != std::end(m_attr) ? &*iter : nullptr;
		}
		Attr* attribute(std::wstring const& tag)
		{
			auto iter = std::find_if(std::begin(m_attr), std::end(m_attr), [&](Attr const& a){ return a.tag() == tag; });
			return iter != std::end(m_attr) ? &*iter : nullptr;
		}

		// Access child elements via-array style syntax
		Node const& operator[](int idx) const
		{
			return m_child[idx];
		}
		Node& operator[](int idx)
		{
			return m_child[idx];
		}
		Node const& operator[](std::wstring const& tag) const
		{
			auto ptr = element(tag);
			if (ptr == nullptr) throw std::exception("element not found");
			return *ptr;
		}
		Node& operator[](std::wstring const& tag)
		{
			auto ptr = element(tag);
			if (ptr == nullptr) throw std::exception("element not found");
			return *ptr;
		}
	};

	namespace impl
	{
		// Helper for throwing failed HRESULTs
		inline void Check(HRESULT hr)
		{
			if (!FAILED(hr)) return;

			std::stringstream ss;
			ss << "XML exception - ";

			#pragma region Xml Error Codes
			switch (hr)
			{
			default:
				{
					char msg[16384];
					DWORD length(sizeof(msg) / sizeof(msg[0]));
					if (FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM|FORMAT_MESSAGE_IGNORE_INSERTS, NULL, hr, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), msg, length, NULL))
						ss << msg << std::endl;
					else
						ss << "unknown error code" << std::endl;
				}
				break;
			case MX_E_INPUTEND:               ss << "unexpected end of input"; break;
			case MX_E_ENCODING:               ss << "unrecognized encoding"; break;
			case MX_E_ENCODINGSWITCH:         ss << "unable to switch the encoding"; break;
			case MX_E_ENCODINGSIGNATURE:      ss << "unrecognized input signature"; break;
			case WC_E_WHITESPACE:             ss << "whitespace expected"; break;
			case WC_E_SEMICOLON:              ss << "semicolon expected"; break;
			case WC_E_GREATERTHAN:            ss << "'>' expected"; break;
			case WC_E_QUOTE:                  ss << "quote expected"; break;
			case WC_E_EQUAL:                  ss << "equal expected"; break;
			case WC_E_LESSTHAN:               ss << "well-formedness constraint: no '<' in attribute value"; break;
			case WC_E_HEXDIGIT:               ss << "hexadecimal digit expected"; break;
			case WC_E_DIGIT:                  ss << "decimal digit expected"; break;
			case WC_E_LEFTBRACKET:            ss << "'[' expected"; break;
			case WC_E_LEFTPAREN:              ss << "'(' expected"; break;
			case WC_E_XMLCHARACTER:           ss << "illegal xml character"; break;
			case WC_E_NAMECHARACTER:          ss << "illegal name character"; break;
			case WC_E_SYNTAX:                 ss << "incorrect document syntax"; break;
			case WC_E_CDSECT:                 ss << "incorrect CDATA section syntax"; break;
			case WC_E_COMMENT:                ss << "incorrect comment syntax"; break;
			case WC_E_CONDSECT:               ss << "incorrect conditional section syntax"; break;
			case WC_E_DECLATTLIST:            ss << "incorrect ATTLIST declaration syntax"; break;
			case WC_E_DECLDOCTYPE:            ss << "incorrect DOCTYPE declaration syntax"; break;
			case WC_E_DECLELEMENT:            ss << "incorrect ELEMENT declaration syntax"; break;
			case WC_E_DECLENTITY:             ss << "incorrect ENTITY declaration syntax"; break;
			case WC_E_DECLNOTATION:           ss << "incorrect NOTATION declaration syntax"; break;
			case WC_E_NDATA:                  ss << "NDATA expected"; break;
			case WC_E_PUBLIC:                 ss << "PUBLIC expected"; break;
			case WC_E_SYSTEM:                 ss << "SYSTEM expected"; break;
			case WC_E_NAME:                   ss << "name expected"; break;
			case WC_E_ROOTELEMENT:            ss << "one root element"; break;
			case WC_E_ELEMENTMATCH:           ss << "well-formedness constraint: element type match"; break;
			case WC_E_UNIQUEATTRIBUTE:        ss << "well-formedness constraint: unique attribute spec"; break;
			case WC_E_TEXTXMLDECL:            ss << "text/xmldecl not at the beginning of input"; break;
			case WC_E_LEADINGXML:             ss << "leading 'xml'"; break;
			case WC_E_TEXTDECL:               ss << "incorrect text declaration syntax"; break;
			case WC_E_XMLDECL:                ss << "incorrect xml declaration syntax"; break;
			case WC_E_ENCNAME:                ss << "incorrect encoding name syntax"; break;
			case WC_E_PUBLICID:               ss << "incorrect public identifier syntax"; break;
			case WC_E_PESINTERNALSUBSET:      ss << "well-formedness constraint: pes in internal subset"; break;
			case WC_E_PESBETWEENDECLS:        ss << "well-formedness constraint: pes between declarations"; break;
			case WC_E_NORECURSION:            ss << "well-formedness constraint: no recursion"; break;
			case WC_E_ENTITYCONTENT:          ss << "entity content not well formed"; break;
			case WC_E_UNDECLAREDENTITY:       ss << "well-formedness constraint: undeclared entity"; break;
			case WC_E_PARSEDENTITY:           ss << "well-formedness constraint: parsed entity"; break;
			case WC_E_NOEXTERNALENTITYREF:    ss << "well-formedness constraint: no external entity references"; break;
			case WC_E_PI:                     ss << "incorrect processing instruction syntax"; break;
			case WC_E_SYSTEMID:               ss << "incorrect system identifier syntax"; break;
			case WC_E_QUESTIONMARK:           ss << "'?' expected"; break;
			case WC_E_CDSECTEND:              ss << "no ']]>' in element content"; break;
			case WC_E_MOREDATA:               ss << "not all chunks of value have been read"; break;
			case WC_E_DTDPROHIBITED:          ss << "DTD was found but is prohibited"; break;
			case WC_E_INVALIDXMLSPACE:        ss << "xml:space attribute with invalid value"; break;
			case NC_E_QNAMECHARACTER:         ss << "illegal qualified name character"; break;
			case NC_E_QNAMECOLON:             ss << "multiple colons in qualified name"; break;
			case NC_E_NAMECOLON:              ss << "colon in name"; break;
			case NC_E_DECLAREDPREFIX:         ss << "declared prefix"; break;
			case NC_E_UNDECLAREDPREFIX:       ss << "undeclared prefix"; break;
			case NC_E_EMPTYURI:               ss << "non default namespace with empty uri"; break;
			case NC_E_XMLPREFIXRESERVED:      ss << "'xml' prefix is reserved and must have the http://www.w3.org/XML/1998/namespace URI"; break;
			case NC_E_XMLNSPREFIXRESERVED:    ss << "'xmlns' prefix is reserved for use by XML"; break;
			case NC_E_XMLURIRESERVED:         ss << "xml namespace URI (http://www.w3.org/XML/1998/namespace) must be assigned only to prefix 'xml'"; break;
			case NC_E_XMLNSURIRESERVED:       ss << "xmlns namespace URI (http://www.w3.org/2000/xmlns/) is reserved and must not be used"; break;
			case SC_E_MAXELEMENTDEPTH:        ss << "element depth exceeds limit in XmlReaderProperty_MaxElementDepth"; break;
			case SC_E_MAXENTITYEXPANSION:     ss << "entity expansion exceeds limit in XmlReaderProperty_MaxEntityExpansion"; break;
			case WR_E_NONWHITESPACE:          ss << "writer: specified string is not whitespace"; break;
			case WR_E_NSPREFIXDECLARED:       ss << "writer: namespace prefix is already declared with a different namespace"; break;
			case WR_E_NSPREFIXWITHEMPTYNSURI: ss << "writer: It is not allowed to declare a namespace prefix with empty URI (for example xmlns:p='')."; break;
			case WR_E_DUPLICATEATTRIBUTE:     ss << "writer: duplicate attribute"; break;
			case WR_E_XMLNSPREFIXDECLARATION: ss << "writer: can not redefine the xmlns prefix"; break;
			case WR_E_XMLPREFIXDECLARATION:   ss << "writer: xml prefix must have the http://www.w3.org/XML/1998/namespace URI"; break;
			case WR_E_XMLURIDECLARATION:      ss << "writer: xml namespace URI (http://www.w3.org/XML/1998/namespace) must be assigned only to prefix 'xml'"; break;
			case WR_E_XMLNSURIDECLARATION:    ss << "writer: xmlns namespace URI (http://www.w3.org/2000/xmlns/) is reserved and must not be used"; break;
			case WR_E_NAMESPACEUNDECLARED:    ss << "writer: namespace is not declared"; break;
			case WR_E_INVALIDXMLSPACE:        ss << "writer: invalid value of xml:space attribute (allowed values are 'default' and 'preserve')"; break;
			case WR_E_INVALIDACTION:          ss << "writer: performing the requested action would result in invalid XML document"; break;
			case WR_E_INVALIDSURROGATEPAIR:   ss << "writer: input contains invalid or incomplete surrogate pair"; break;
			case XML_E_INVALID_DECIMAL:       ss << "character in character entity is not a decimal digit as was expected."; break;
			case XML_E_INVALID_HEXIDECIMAL:   ss << "character in character entity is not a hexadecimal digit as was expected."; break;
			case XML_E_INVALID_UNICODE:       ss << "character entity has invalid Unicode value."; break;
			}
			#pragma endregion

			throw std::runtime_error(ss.str().c_str());
		}

		// Parse XML tag attributes
		template <typename Node>
		void ParseAttributes(Ptr<IXmlReader>& reader, Node& node)
		{
			// Parse all attributes
			HRESULT hr;
			for (hr = reader->MoveToFirstAttribute(); hr == S_OK; hr = reader->MoveToNextAttribute())
			{
				if (reader->IsDefault())
					continue;

				// e.g. <prefix:localname="value"/>
				WCHAR const *prefix, *localname, *value;
				Check(reader->GetPrefix(&prefix, 0));
				Check(reader->GetLocalName(&localname, 0));
				Check(reader->GetValue(&value, 0));
				node.m_attr.push_back(Attr(prefix, localname, value));
			}
		}

		// Parse an ending XML tag
		template <typename Node>
		void ParseEndElement(Ptr<IXmlReader>& reader, Node& node)
		{
			WCHAR const *prefix, *localname;
			Check(reader->GetPrefix(&prefix, 0));
			Check(reader->GetLocalName(&localname, 0));
			node.m_prefix = prefix;
			node.m_tag = localname;
		}

		// Parse a string value between tags
		template <typename Node>
		void ParseValue(Ptr<IXmlReader>& reader, Node& node)
		{
			WCHAR const* value;
			Check(reader->GetValue(&value, 0));
			node.m_value = value;
		}

		// Parse a processing instruction
		template <typename Node>
		void ParseProcessingInstruction(Ptr<IXmlReader>& reader, Node& node)
		{
			WCHAR const *localname, *value;
			Check(reader->GetLocalName(&localname, 0));
			Check(reader->GetValue(&value, 0));
			node.m_proc_instr.push_back(Attr(L"", localname, value));
		}

		// Parse an XML element 
		template <typename Node>
		void ParseComment(Ptr<IXmlReader>& reader, Node& node)
		{
			WCHAR const* value;
			Check(reader->GetValue(&value, 0));
			if (value != nullptr)
				node.m_comments.push_back(value);
		}

		// Parse an XML element 
		template <typename Node>
		void ParseElement(Ptr<IXmlReader>& reader, Node& node, bool top_level)
		{
			if (reader->IsEmptyElement())
				return ParseEndElement(reader, node);

			HRESULT hr;
			XmlNodeType xml_node;
			for (hr = reader->Read(&xml_node); hr == S_OK || hr == E_PENDING; hr = reader->Read(&xml_node))
			{
				if (hr == E_PENDING) { ::Sleep(100); continue; }
				switch (xml_node)
				{
				default: break;
				case XmlNodeType_XmlDeclaration:
					ParseAttributes(reader, node);
					break;
				case XmlNodeType_Element:
					if (top_level)
						ParseElement(reader, node, false);
					else
					{
						Node child;
						ParseElement(reader, child, false);
						node.m_child.push_back(child);
					}
					break;
				case XmlNodeType_EndElement:
					ParseEndElement(reader, node);
					return;
				case XmlNodeType_Text:
				case XmlNodeType_CDATA:
					ParseValue(reader, node);
					break;
				case XmlNodeType_ProcessingInstruction:
					ParseProcessingInstruction(reader, node);
					break;
				case XmlNodeType_Comment:
					ParseComment(reader, node);
					break;
				case XmlNodeType_Whitespace:
					break;
				case XmlNodeType_DocumentType:
					break;
				}
			}
			if (hr != S_FALSE)
				Check(hr);
		}

		// Write an XML element 
		template <typename Node>
		void WriteElement(Ptr<IXmlWriter>& writer, Node& node)
		{
			// Write the comments about the element
			for (auto& comment : node.m_comments)
				Check(writer->WriteComment(comment.c_str()));

			// Write the processing instructions
			for (auto& attr : node.m_proc_instr)
				Check(writer->WriteProcessingInstruction(attr.m_localname.c_str(), attr.m_value.c_str()));

			// Begin the element
			Check(writer->WriteStartElement(0, node.m_tag.c_str(), 0));

			// Write the attributes
			for (auto& attr : node.m_attr)
				Check(writer->WriteAttributeString(attr.m_prefix.c_str(), attr.m_localname.c_str(), 0, attr.m_value.c_str()));

			if (node.m_child.empty())
			{
				// Write the value
				if (!node.m_cdata)
					Check(writer->WriteString(node.m_value.c_str()));
				else
					Check(writer->WriteCData(node.m_value.c_str()));
			}
			else
			{
				// Write the children
				for (auto& c : node.m_child)
					WriteElement(writer, c);
			}

			// End the element
			Check(writer->WriteEndElement());
		}
	}

	// Parse XML data from a stream generating a 'Node' tree
	inline Node Load(Ptr<IStream>& stream)
	{
		Node out;
		Ptr<IXmlReader> reader;
		impl::Check(CreateXmlReader(__uuidof(IXmlReader), (void**)&reader, 0));
		impl::Check(reader->SetInput(stream.get())); // Set it as the stream source for the reader
		try
		{
			impl::ParseElement(reader, out, true);
			return out;
		}
		catch (std::exception& ex)
		{
			UINT line_num, line_pos;
			if (SUCCEEDED(reader->GetLineNumber(&line_num)) &&
				SUCCEEDED(reader->GetLinePosition(&line_pos)))
			{
				std::stringstream ss;
				ss << ex.what() << std::endl
					<< "line: " << line_num << std::endl
					<< "pos: " << line_pos << std::endl;
				ex = std::exception(ss.str().c_str());
			}
			throw;
		}
	}
	inline Node Load(char const* filename)
	{
		// Create an 'IStream' from a file
		Ptr<IStream> stream;
		impl::Check(SHCreateStreamOnFileA(filename, STGM_READ, stream.address_of()));
		return Load(stream);
	}
	inline Node Load(wchar_t const* filename)
	{
		// Create an 'IStream' from a file
		Ptr<IStream> stream;
		impl::Check(SHCreateStreamOnFileW(filename, STGM_READ, stream.address_of()));
		return Load(stream);
	}
	inline Node Load(char const* xml_string, std::size_t length)
	{
		// Create an 'IStream' from a string
		Ptr<IStream> stream(SHCreateMemStream((BYTE const*)xml_string, (UINT)length), false);
		return Load(stream);
	}

	// Save data in an XML format
	// 'properties' is a bitwise combination of 'EProperty' (note, only some supported)
	inline void Save(Ptr<IStream>& stream, Node const& in, EProperty properties = EProperty::Indent)
	{
		Ptr<IXmlWriter> writer;
		impl::Check(CreateXmlWriter(__uuidof(IXmlWriter), (void**)&writer, 0));

		// Set it as the stream source for the writer
		impl::Check(writer->SetOutput(stream.get()));

		if (AllSet(properties, EProperty::Indent))
			impl::Check(writer->SetProperty(XmlWriterProperty_Indent, TRUE));
		if (AllSet(properties, EProperty::ByteOrderMark))
			impl::Check(writer->SetProperty(XmlWriterProperty_ByteOrderMark, TRUE));
		if (AllSet(properties, EProperty::OmitXmlDeclaration))
			impl::Check(writer->SetProperty(XmlWriterProperty_OmitXmlDeclaration, TRUE));

		// Start the document
		impl::Check(writer->WriteStartDocument(XmlStandalone_Omit));
		impl::WriteElement(writer, in);

		// End the document
		impl::Check(writer->WriteEndDocument());
		impl::Check(writer->Flush());
	}
	inline void Save(std::filesystem::path const& filename, Node const& in, EProperty properties = EProperty::Indent)
	{
		// Create an 'IStream' from a file
		Ptr<IStream> stream;
		impl::Check(SHCreateStreamOnFileW(filename.c_str(), STGM_WRITE | STGM_CREATE, stream.address_of()));
		Save(stream, in, properties);
	}

	// Test for a node equal to a hash value
	inline bool operator == (Node const& lhs, HashValue rhs) { return lhs.hash() == rhs; }
	inline bool operator == (HashValue lhs, Node const& rhs) { return lhs == rhs.hash(); }
	inline bool operator != (Node const& lhs, HashValue rhs) { return lhs.hash() != rhs; }
	inline bool operator != (HashValue lhs, Node const& rhs) { return lhs != rhs.hash(); }

	// Returns true if the prefix and tag in 'node' match 'prefix_tag'
	// 'prefix_tag' should have the format "prefix:tag".
	// If the node has no prefix then just the tag is compared and 'prefix_tag' should have the format "tag".
	inline bool operator == (Node const& node, std::wstring const prefix_tag) { return prefix_tag.compare(node.tag()) == 0; }
	inline bool operator != (Node const& node, std::wstring const prefix_tag) { return !(node == prefix_tag); }

	// Find a child node within 'node' that matches 'prefix:tag'
	inline Node const* Find(Node const& node, HashValue hash)
	{
		auto iter = std::find(std::begin(node.m_child), std::end(node.m_child), hash);
		return iter != std::end(node.m_child) ? &*iter : nullptr;
	}
	inline Node const* Next(Node const& node, Node const* prev, HashValue hash)
	{
		if (prev == 0 || node.m_child.empty()) return nullptr;
		Node const* end = &node.m_child[0] + node.m_child.size();
		Node const* iter = std::find(prev + 1, end, hash);
		return iter != end ? iter : 0;
	}
	inline Node const* Find(Node const& node, Node const* prev, std::wstring const& tag, std::wstring const& prefix = L"")
	{
		return Next(node, prev, Hash(prefix + (prefix.empty() ? L"" : L":") + tag));
	}

	// Return a named child node of 'parent'
	// Throws an HRESULT if the child is not found
	inline Node const& Child(Node const& parent, HashValue hash)
	{
		Node const* node = Find(parent, hash);
		if (node == 0) impl::Check(ERROR_NOT_FOUND);
		return *node;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::storage
{
	PRUnitTest(XmlTests)
	{
		char const xml[] =
R"(
<root>
	<node0>1</node0>
	<child>
		<node1>a string</node1>
	</child>
</root>
)";

		// XML doesn't accept \0 in the string
		xml::Node root = xml::Load(xml, sizeof(xml) - 1);
		PR_EXPECT(root.m_child.size() == 2U);
		PR_EXPECT(root.element(L"node0") != nullptr);
		PR_EXPECT(root.element(L"child") != nullptr);
		PR_EXPECT(root.element(L"boris") == nullptr);

		auto& node0 = root.m_child[0];
		PR_EXPECT(node0.m_child.size() == 0U);
		PR_EXPECT(node0.as<int>() == 1);

		auto& child = root.m_child[1];
		PR_EXPECT(child.m_child.size() == 1U);
		PR_EXPECT(child.element(L"node1") != nullptr);

		auto& node1 = child.m_child[0];
		PR_EXPECT(node1.m_child.size() == 0U);
		PR_EXPECT(node1.as<std::string>() == "a string");
	}
}
#endif