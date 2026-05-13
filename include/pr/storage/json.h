//**********************************
// JSON DOM
//  Copyright (c) Rylogic Ltd 2024
//**********************************
#pragma once
#include <vector>
#include <string>
#include <string_view>
#include <variant>
#include <algorithm>
#include <charconv>
#include <utility>
#include <ostream>
#include <fstream>
#include <execution>
#include <filesystem>
#include <type_traits>
#include <format>
#include <span>
#include <iterator>
#include <cmath>

// Example use:
#if 0
{ // Write
	json::Document doc;

	doc.root()["key0"] = 123;
	doc.root()["key1"] = 456.78;
	doc.root()["key2"] = json::Array{ 1, 2, 3, 4 };
	doc.root()["key3"] = json::Object{};

	auto& child = root["key3"].to_object();
	child["key10"] = nullptr;
	child["key11"] = true;
	child["key12"] = json::Array{
		true,
		"value3",
		80085,
	};

	json::Object key4 = {
		{"four", 4},
		{"five", "five"},
	};
	root["key4"] = std::move(key4);

	std::string str = json::Write(doc, {});
}
{ // Read
	auto doc = json::Read(filepath);

	// root is an object {}
	auto root = doc.to_object();
	auto k0 = root["key0"].to<std::string>();

	// root is an array []
	for (auto& jitem : doc.to_array())
	{
	}
}
#endif

namespace pr::json
{
	struct Value;

	using key_t = std::string;
	using keys_t = std::vector<key_t>;
	using vals_t = std::vector<Value>;
	static constexpr int GrowRate = 100;

	template <typename T> concept enum_type = std::is_enum_v<std::decay_t<T>>;

	Value const& NullValue();
	std::string EscapeString(std::string_view str);
	std::string UnescapeString(std::string_view str);

	// Compound types
	struct Array
	{
		vals_t values;

		Array() = default;
		Array(std::initializer_list<Value> values_)
			: values(values_)
		{}

		// Array interface
		bool empty() const
		{
			return values.size() == 0;
		}
		size_t size() const
		{
			return values.size();
		}

		// RAnged based for interation
		using citer_t = vals_t::const_iterator;
		using miter_t = vals_t::iterator;
		citer_t begin() const
		{
			return values.begin();
		}
		citer_t end() const
		{
			return values.end();
		}

		// Append items to the array
		void push_back(Value&& value)
		{
			values.push_back(std::move(value));

			if (values.size() == values.capacity())
				values.reserve(values.capacity() * GrowRate);
		}
		void push_back(Value const& value)
		{
			values.push_back(value);

			if (values.size() == values.capacity())
				values.reserve(values.capacity() * GrowRate);
		}

		// Access the value associated with 'index'
		Value const& operator [] (size_t index) const
		{
			if (index >= values.size()) return NullValue();
			return values[index];
		}
		Value& operator[](size_t index)
		{
			values.resize(std::max<size_t>(values.size(), index + 1));
			return values[index];
		}
	};
	struct Object
	{
		keys_t keys;
		vals_t values;

		Object() = default;
		Object(std::initializer_list<std::pair<std::string_view, Value>> items);

		// Array interface
		bool empty() const
		{
			return keys.empty();
		}
		size_t size() const
		{
			return keys.size();
		}

		// Range based for iteration
		template <typename key_iter_t, typename val_iter_t> struct Iter
		{
			using pair_t = struct {
				typename key_iter_t::value_type const& key;
				typename val_iter_t::value_type const& val;
			};
			key_iter_t key;
			val_iter_t value;

			pair_t operator * () const
			{
				return pair_t{ *key, *value };
			}
			Iter& operator ++ ()
			{
				++key;
				++value;
				return *this;
			}
			Iter& operator += (ptrdiff_t ofs)
			{
				key += ofs;
				value += ofs;
				return *this;
			}
			friend Iter operator + (Iter const& lhs, ptrdiff_t rhs)
			{
				return Iter(lhs) += rhs;
			}
			friend ptrdiff_t operator - (Iter const& lhs, Iter const& rhs)
			{
				return lhs.key - rhs.key;
			}
			friend bool operator != (Iter const& lhs, Iter const& rhs)
			{
				return lhs.key != rhs.key;
			}
		};
		using citer_t = Iter<keys_t::const_iterator, vals_t::const_iterator>;
		using miter_t = Iter<keys_t::iterator, vals_t::iterator>;
		citer_t begin() const
		{
			return citer_t{ keys.begin(), values.begin() };
		}
		miter_t begin()
		{
			return miter_t{ keys.begin(), values.begin() };
		}
		citer_t end() const
		{
			return citer_t{ keys.end(), values.end() };
		}
		miter_t end()
		{
			return miter_t{ keys.end(), values.end() };
		}

		// Find 'key' in the object and return its index
		size_t index_of(std::string_view key) const
		{
			auto it = std::find_if(keys.begin(), keys.end(), [key](auto& k) { return k == key; });
			return std::distance(keys.begin(), it);
		}

		// Access the value associated with 'index'
		Value const& operator [] (size_t index) const
		{
			if (index >= values.size()) return NullValue();
			return values[index];
		}
		Value& operator[](size_t index)
		{
			if (index >= values.size()) throw std::out_of_range("Index out of range");
			return values[index];
		}

		// Access the value associated with 'key'
		Value const& operator [] (std::string_view key) const
		{
			auto idx = index_of(key);
			return (*this)[idx];
		}
		Value& operator[](std::string_view key)
		{
			auto idx = index_of(key);
			if (idx == keys.size())
			{
				idx = keys.size();
				keys.push_back(std::string{ key });
				values.push_back(NullValue());

				if (keys.size() == keys.capacity())
				{
					keys.reserve(keys.capacity() * GrowRate);
					values.reserve(values.capacity() * GrowRate);
				}
			}
			return (*this)[idx];
		}

		// Look for an item. Returns nullptr if not found
		Value const* find(std::string_view key) const
		{
			auto idx = index_of(key);
			return idx < keys.size() ? &values[idx] : nullptr;
		}
		Value* find(std::string_view key)
		{
			auto idx = index_of(key);
			return idx < keys.size() ? &values[idx] : nullptr;
		}
	};

	// Variant type
	struct Value
	{
		enum TypeIndex { Null, Bool, String, Number, ChildArray, ChildObject };
		using variant_t = std::variant<nullptr_t, bool, std::string, double, Array, Object>;
		variant_t value;

		Value() = default;
		Value(Value&& rhs) = default;
		Value(Value const& rhs) = default;
		Value& operator = (Value&& rhs) = default;
		Value& operator = (Value const& rhs) = default;

		Value(nullptr_t)
			: value(nullptr)
		{}
		Value(bool value_)
			: value(value_)
		{}
		Value(std::string&& value_)
			: value(std::move(value_))
		{}
		Value(std::string const& value_)
			: value(value_)
		{}
		Value(std::string_view value_)
			: Value(std::string{ value_ })
		{}
		Value(char const* value_)
			: Value(std::string{ value_ })
		{}
		Value(float value_)
			: Value(static_cast<double>(value_))
		{}
		Value(double value_)
			: value(value_)
		{}
		Value(int32_t value_)
			: Value(static_cast<int64_t>(value_))
		{}
		Value(int64_t value_)
			: Value(static_cast<double>(value_))
		{}
		Value(enum_type auto value_)
			: Value(static_cast<int64_t>(value_))
		{}
		Value(Array&& value_)
			: value(std::move(value_))
		{}
		Value(Object&& value_)
			: value(std::move(value_))
		{}

		// Convert to a json type
		template <typename T> T const* as() const
		{
			return std::get_if<T>(&value);
		}

		// Value accessors - only const because setting non-variant types is not supported
		// Primary template is intentionally undefined; callers can add explicit specializations for custom conversions.
		template <typename T> T to() const;
		template <enum_type T> T to() const;
		
		// Convert to json object/array types.
		Array const& to_array() const
		{
			return std::get<Array>(value);
		}
		Object const& to_object() const
		{
			return std::get<Object>(value);
		}
		Array& to_array()
		{
			return std::get<Array>(value);
		}
		Object& to_object()
		{
			return std::get<Object>(value);
		}

		// Object/Array accessors
		size_t size() const
		{
			if (auto object = as<Object>())
				return object->size();
			if (auto list = as<Array>())
				return list->size();

			throw std::runtime_error("Not an object or array");
		}
		Value const& operator [] (std::string_view key_) const
		{
			if (auto object = as<Object>())
				return (*object)[key_];

			throw std::runtime_error(std::format("{} is not a key within this object", key_));
		}
		Value const& operator [] (size_t index) const
		{
			if (auto object = as<Object>())
				return (*object)[index];
			if (auto list = as<Array>())
				return (*list)[index];

			throw std::runtime_error(std::format("Index {} is not with this array", index));
		}

		// A string representation of the value
		std::string str() const
		{
			switch (value.index())
			{
				case TypeIndex::Null: return "";
				case TypeIndex::Bool: return std::get<bool>(value) ? "1" : "0";
				case TypeIndex::String: return std::get<std::string>(value);
				case TypeIndex::Number: return std::format("{}", std::get<double>(value));
				case TypeIndex::ChildArray: return "JsonArray";
				case TypeIndex::ChildObject: return "JsonObject";
				default: throw std::runtime_error("Unknown value type");
			}
		}

		// Comparison operators
		friend bool operator == (Value const& lhs, nullptr_t)
		{
			return lhs.value.valueless_by_exception() || std::holds_alternative<nullptr_t>(lhs.value);
		}
		friend bool operator != (Value const& lhs, nullptr_t)
		{
			return !(lhs == nullptr);
		}
	};
	template <> inline std::string const& Value::to<std::string const&>() const
	{
		return std::get<std::string>(value);
	}
	template <> inline std::string Value::to<std::string>() const
	{
		return std::get<std::string>(value);
	}
	template <> inline std::string_view Value::to<std::string_view>() const
	{
		return std::string_view{ to<std::string const&>() };
	}
	template <> inline std::filesystem::path Value::to<std::filesystem::path>() const
	{
		return std::filesystem::path{ to<std::string const&>() };
	}
	template <> inline double Value::to<double>() const
	{
		return std::get<double>(value);
	}
	template <> inline int64_t Value::to<int64_t>() const
	{
		return std::llround(to<double>());
	}
	template <> inline float Value::to<float>() const
	{
		return static_cast<float>(to<double>());
	}
	template <> inline int Value::to<int>() const
	{
		return static_cast<int>(to<int64_t>());
	}
	template <> inline bool Value::to<bool>() const
	{
		if (auto b = std::get_if<bool>(&value); b != nullptr)
			return *b;

		return to<int64_t>() != 0;
	}
	template <enum_type T> inline T Value::to() const
	{
		return static_cast<std::decay_t<T>>(to<int64_t>());
	}

	// JSON DOM
	struct Document : Value
	{
		// Syntax:
		// json::Document doc; // has object root by default
		// json::Document doc(json::Object{}); // object root
		// json::Document doc(json::Array{}); // array root
		Document()
			:Value(Object{})
		{}
		Document(Value&& doc)
			:Value(std::move(doc))
		{}

		// Access the root object assuming it's an object
		Object const& root() const
		{
			return std::get<Object>(value);
		}
		Object& root()
		{
			return std::get<Object>(value);
		}

		// Access the root object assuming it's an array
		Array const& array() const
		{
			return std::get<Array>(value);
		}
		Array& array()
		{
			return std::get<Array>(value);
		}
	};

	// Json parsing options
	struct Options
	{
		// Deserialize options ---
		bool AllowComments = false;
		bool AllowTrailingCommas = false;
		
		// Serialize options ---
		bool Indent = true;
		std::string_view IndentString = "\t";
		
		// Number of elements in a "short" array
		int ShortArrayLength = 20;

		// Length of a "short" string
		int ShortStringLength = 16;

		// Line length for arrays before wrapping (0 = one element per line)
		int LineWrapArrays = 0;

		// Use parallel serialization on large arrays or objects (length >= to this value)
		int ParallelSerialise = 10000;
	};

	// Implementation
	namespace utf8 // Duplicated from 'pr/str/utf8.h'
	{
		using code_point_t = unsigned int;

		// Control bit helpers
		constexpr bool Continuation(char c)
		{
			return (c & 0xC0) == 0x80;
		}
		constexpr bool _1Byte(char c)
		{
			return (c & 0x80) == 0x00;
		}
		constexpr bool _2Byte(char c)
		{
			return (c & 0xE0) == 0xC0;
		}
		constexpr bool _3Byte(char c)
		{
			return (c & 0xF0) == 0xE0;
		}
		constexpr bool _4Byte(char c)
		{
			return (c & 0xF8) == 0xF0;
		}

		// Returns the number of bytes expected for a unicode character starting with 'c'
		constexpr int ByteLength(char c)
		{
			return
				_4Byte(c) ? 4 :
				_3Byte(c) ? 3 :
				_2Byte(c) ? 2 :
				_1Byte(c) ? 1 :
				0; // Invalid utf-8 character
		}

		// Convert utf-8 bytes into a code point.
		inline code_point_t CodePoint(char const*& ptr, char const* end)
		{
			int len;
			if (ptr == end || (len = ByteLength(*ptr)) == 0)
				throw std::runtime_error("Invalid unicode character");
			if (end - ptr < len)
				throw std::runtime_error("Incomplete unicode character");
			if (len == 1)
				return *ptr++;

			code_point_t code = *ptr++ & (0x7F >> len);
			for (--len; len != 0; --len)
			{
				if (!Continuation(*ptr)) throw std::runtime_error("Invalid unicode character");
				code = (code << 6) | (*ptr++ & 0x3F);
			}

			return code;
		}

		// Convert a code point into an escaped string using the '\u' or '\U' format
		inline void Escape(code_point_t code_point, std::string& out)
		{
			constexpr char hex[] = "0123456789ABCDEF";

			auto size = out.size();
			out.resize(size + 2 + (code_point > 0xFFFF ? 8 : 4));
			auto ptr = out.data() + size;

			*ptr++ = '\\';
			*ptr++ = code_point > 0xFFFF ? 'U' : 'u';

			auto count = code_point > 0xFFFF ? 8 : 4;
			for (; count-- != 0; )
				*ptr++ = hex[(code_point >> (count << 2)) & 0xf];
		}

		// Convert an escaped unicode code point into a code point
		inline code_point_t Unescape(std::string_view str)
		{
			auto ptr = str.data();
			auto end = str.data() + str.size();

			if (end - ptr < 2 || ptr[0] != '\\' || (ptr[1] != 'u' && ptr[1] != 'U'))
				throw std::runtime_error("Invalid unicode escape sequence");

			auto len = ptr[1] == 'U' ? 8 : 4;
			if (end - ptr < 2 + len)
				throw std::runtime_error("Incomplete unicode escape sequence");

			ptr += 2;

			constexpr auto Nibble = [](char c)
			{
				if (c >= '0' && c <= '9') return c - '0';
				if (c >= 'A' && c <= 'F') return c - 'A' + 10;
				if (c >= 'a' && c <= 'f') return c - 'a' + 10;
				throw std::runtime_error("Invalid hex character");
			};

			code_point_t code = 0;
			for (auto pow = 1U << ((len - 1) * 4); len-- != 0; pow >>= 4)
				code |= Nibble(*ptr++) * pow;

			return code;
		}

		// True if the first 3 bytes of 'str' are the UTF-8 BOM bytes
		inline bool IsBOM(std::string_view str)
		{
			return
				str.size() >= 3 &&
				static_cast<uint8_t>(str[0]) == 0xEF &&
				static_cast<uint8_t>(str[1]) == 0xBB &&
				static_cast<uint8_t>(str[2]) == 0xBF;
		}
	}
	namespace impl
	{
		enum class EToken
		{
			EndOfString,
			Null,
			False,
			True,
			String,
			Number,
			OpenBrace,
			CloseBrace,
			OpenBracket,
			CloseBracket,
			Colon,
			Comma,
		};
		enum class EEatFlags
		{
			None = 0,
			Comments = 1 << 0,
			AllowEndOfString = 1 << 1,
		};
		struct Token
		{
			EToken token;
			std::string_view data;
			double number = {};
		};
		template <typename T> requires (std::is_enum_v<T>)
		inline bool HasFlag(T bits, T flag)
		{
			using UT = std::underlying_type_t<T>;
			return (static_cast<UT>(bits) & static_cast<UT>(flag)) != 0;
		}
		constexpr bool IsSpace(char ch)
		{
			return ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r' || ch == '\f' || ch == '\v';
		}

		// Remove whitespace characters (or comments) from the start of 'src'
		inline bool EatWS(std::string_view& src, EEatFlags eat_flags)
		{
			auto Eat = [](std::string_view& src, int eat_initial, int eat_final, auto pred)
			{
				src.remove_prefix(eat_initial);
				for (; !src.empty() && pred(src); src.remove_prefix(1)) {}
				if (!src.empty()) src.remove_prefix(eat_final);
			};

			for (; !src.empty(); )
			{
				// Eat whitespace
				if (IsSpace(src[0]))
				{
					Eat(src, 1, 0, [](auto& sv) { return IsSpace(sv[0]); });
					continue;
				}

				// Line comments
				if (HasFlag(eat_flags, EEatFlags::Comments) && src.compare(0, 2, "//") == 0)
				{
					Eat(src, 2, 1, [](auto& sv) { return sv[0] != '\n'; });
					continue;
				}

				// Block comments
				if (HasFlag(eat_flags, EEatFlags::Comments) && src.compare(0, 2, "/*") == 0)
				{
					Eat(src, 2, 2, [](auto& sv) { return sv.compare(0, 2, "*/") != 0; });
					continue;
				}

				// No more whitespace or comments
				break;
			}

			if (src.empty() && !HasFlag(eat_flags, EEatFlags::AllowEndOfString))
				throw std::runtime_error("Unexpected end of string");

			// Always return true to allow if chaining
			return true;
		}

		// Return the next token in the json string
		inline Token NextToken(std::string_view& src, Options const& opts)
		{
			EatWS(src, EEatFlags::AllowEndOfString);
			if (src.empty())
				return { EToken::EndOfString, {} };

			switch (src[0])
			{
				case '{':
				{
					src.remove_prefix(1);
					return { EToken::OpenBrace, {} };
				}
				case '}':
				{
					src.remove_prefix(1);
					return { EToken::CloseBrace, {} };
				}
				case '[':
				{
					src.remove_prefix(1);
					return { EToken::OpenBracket, {} };
				}
				case ']':
				{
					src.remove_prefix(1);
					return { EToken::CloseBracket, {} };
				}
				case ':':
				{
					src.remove_prefix(1);
					return { EToken::Colon, {} };
				}
				case ',':
				{
					src.remove_prefix(1);
					return { EToken::Comma, {} };
				}
				case 't':
				case 'T':
				{
					if (src.size() < 4 || tolower(src[1]) != 'r' || tolower(src[2]) != 'u' || tolower(src[3]) != 'e')
						throw std::runtime_error("Unknown token");

					src.remove_prefix(4);
					return { EToken::True, {} };
				}
				case 'f':
				case 'F':
				{
					if (src.size() < 5 || tolower(src[1]) != 'a' || tolower(src[2]) != 'l' || tolower(src[3]) != 's' || tolower(src[4]) != 'e')
						throw std::runtime_error("Unknown token");

					src.remove_prefix(5);
					return { EToken::False, {} };
				}
				case 'n':
				{
					if (src.size() < 4 || tolower(src[1]) != 'u' || tolower(src[2]) != 'l' || tolower(src[3]) != 'l')
						throw std::runtime_error("Unknown token");

					src.remove_prefix(4);
					return { EToken::Null, {} };
				}
				case '"':
				case '\'':
				{
					auto quote = src[0];

					auto ptr = src.data();
					auto end = src.data() + src.size();

					// Find the end of the string literal
					for (bool esc = true; ptr != end && (esc || *ptr != quote); esc = !esc && *ptr == '\\', ++ptr) {}
					if (ptr == end)
						throw std::runtime_error("Incomplete literal string or character");

					// Unescape the string
					auto str = src.substr(1, ptr - src.data() - 1);
					src.remove_prefix(ptr - src.data() + 1);
					return { EToken::String, str };
				}
				case '-':
				case '+':
				case '0': case '1': case '2': case '3': case '4':
				case '5': case '6': case '7': case '8': case '9':
				{
					auto beg = src.data();
					auto end = src.data() + src.size();
					auto ptr = beg;
					if (*ptr == '+')
						++ptr;

					double number = {};
					auto [num_end, ec] = std::from_chars(ptr, end, number);
					if (ec != std::errc{} || num_end == ptr)
						throw std::runtime_error("Invalid number");

					auto str = src.substr(0, num_end - beg);
					src.remove_prefix(num_end - beg);
					return { EToken::Number, str, number };
				}
				case '/':
				{
					if (src.size() < 2 || src[1] != '/')
						throw std::runtime_error("Unknown token");

					if (!opts.AllowComments)
						throw std::runtime_error("Comments not allowed");

					auto ptr = src.data();
					auto end = src.data() + src.size();

					// Eat up to the end of the line
					for (; ptr != end && *ptr != '\n'; ++ptr) {}
					if (ptr == end)
						return { EToken::EndOfString, {} };

					// Tail recursion optimisation should prevent stack overflow
					src.remove_prefix(ptr - src.data());
					return NextToken(src, opts);
				}
				default:
				{
					throw std::runtime_error("Unknown token");
				}
			}
		}

		// Return the next key in the json string
		inline std::string_view NextKey(std::string_view& src, Options const& opts)
		{
			auto tok = NextToken(src, opts);
			if (tok.token != EToken::String)
				throw std::runtime_error("Expected key");

			return tok.data;
		}

		// Return the next value in the json string
		inline Value NextValue(std::string_view& src, Options const& opts)
		{
			auto eat_flags = opts.AllowComments ? EEatFlags::Comments : EEatFlags::None;
			auto tok = NextToken(src, opts);
			switch (tok.token)
			{
				case EToken::EndOfString:
				{
					throw std::runtime_error("Unexpected end of string");
				}
				case EToken::Null:
				{
					return nullptr;
				}
				case EToken::True:
				{
					return true;
				}
				case EToken::False:
				{
					return false;
				}
				case EToken::String:
				{
					return UnescapeString(tok.data);
				}
				case EToken::Number:
				{
					return tok.number;
				}
				case EToken::OpenBracket:
				{
					auto list = Array{};
					list.values.reserve(4);
					auto require_comma = false;
					for (;;)
					{
						if (EatWS(src, eat_flags) && src[0] == ']')
						{
							std::ignore = NextToken(src, opts);
							break;
						}
						if (require_comma && NextToken(src, opts).token != EToken::Comma)
						{
							throw std::runtime_error("Expected comma");
						}
						if (opts.AllowTrailingCommas && EatWS(src, eat_flags) && src[0] == ']')
						{
							std::ignore = NextToken(src, opts);
							break;
						}

						// Not the end of the array, so add the next value
						list.values.push_back(NextValue(src, opts));
						require_comma = true;
					}
					return list;
				}
				case EToken::OpenBrace:
				{
					auto obj = Object{};
					obj.keys.reserve(8);
					obj.values.reserve(8);
					auto require_comma = false;
					for (;;)
					{
						if (EatWS(src, eat_flags) && src[0] == '}')
						{
							std::ignore = NextToken(src, opts);
							break;
						}
						if (require_comma && NextToken(src, opts).token != EToken::Comma)
						{
							throw std::runtime_error("Expected comma");
						}
						if (opts.AllowTrailingCommas && EatWS(src, eat_flags) && src[0] == '}')
						{
							std::ignore = NextToken(src, opts);
							break;
						}

						// Not the end of the object, so add the next key/value pair
						auto key = NextKey(src, opts);

						if (NextToken(src, opts).token != EToken::Colon)
							throw std::runtime_error("Expected colon");

						auto val = NextValue(src, opts);

						// Add the item
						obj.keys.push_back(std::string{ key });
						obj.values.push_back(std::move(val));
						require_comma = true;
					}
					return obj;
				}
				default:
				{
					throw std::runtime_error("Unknown token");
				}
			}
		}

		// Try to get the remaining byte count without changing the stream position.
		inline size_t StreamSizeRemaining(std::istream& in)
		{
			auto const pos = in.tellg();
			if (pos == std::streampos{ -1 })
			{
				in.clear();
				return std::dynamic_extent;
			}

			in.seekg(0, std::ios::end);
			auto const end = in.tellg();
			in.seekg(pos);
			if (!in.good())
				in.clear();

			return end != std::streampos{ -1 } && end >= pos
				? static_cast<size_t>(end - pos)
				: std::dynamic_extent;
		}

		// Read 'size_estimate' bytes from the stream, or until the end of the stream if 'size_estimate' is std::dynamic_extent
		inline std::string ReadStreamData(std::istream& in, size_t size_estimate)
		{
			auto exact_size = false;
			auto size = size_estimate;
			if (size == std::dynamic_extent)
			{
				size = StreamSizeRemaining(in);
				exact_size = size != std::dynamic_extent;
			}

			if (exact_size)
			{
				if (size > static_cast<size_t>(std::numeric_limits<std::streamsize>::max()))
					throw std::runtime_error("JSON stream is too large");

				std::string data(size, '\0');
				in.read(data.data(), static_cast<std::streamsize>(data.size()));
				data.resize(static_cast<size_t>(in.gcount()));
				return data;
			}

			std::string data;
			if (size != std::dynamic_extent)
				data.reserve(size);

			data.assign(std::istreambuf_iterator<char>{in}, std::istreambuf_iterator<char>{});
			return data;
		}
	}

	// Static null value returned for non-existent keys/indices
	inline Value const& NullValue()
	{
		static Value s_null_value;
		return s_null_value;
	}

	// Convert a normal string to an escaped string
	inline std::string EscapeString(std::string_view str)
	{
		auto ptr = str.data();
		auto end = str.data() + str.size();

		std::string out;
		out.reserve(str.size() * 2);

		for (; ptr != end;)
		{
			auto ch = *ptr++;
			switch (ch)
			{
			case '\\': out.append("\\\\"); break;
			case '"':  out.append("\\\""); break;
			case '\b': out.append("\\b"); break;
			case '\f': out.append("\\f"); break;
			case '\n': out.append("\\n"); break;
			case '\r': out.append("\\r"); break;
			case '\t': out.append("\\t"); break;
			default:
				if (utf8::ByteLength(ch) > 1)
				{
					--ptr; // back up so CodePoint can read the full sequence
					auto code = utf8::CodePoint(ptr, end);
					utf8::Escape(code, out);
				}
				else
				{
					out.push_back(ch);
				}
				break;
			}
		}
		return out;
	}

	// Convert an escaped string to a normal string
	inline std::string UnescapeString(std::string_view str)
	{
		if (str.find('\\') == std::string_view::npos)
			return std::string{ str };

		auto ptr = str.data();
		auto end = str.data() + str.size();

		std::string result(str.size(), '\0');
		auto out = result.data();

		for (; ptr != end; ++ptr)
		{
			if (*ptr != '\\')
			{
				*out++ = *ptr;
				continue;
			}
			switch (*++ptr)
			{
				case '\\': *out++ = '\\'; break;
				case '"': *out++ = '"'; break;
				case '/': *out++ = '/'; break;
				case 'b': *out++ = '\b'; break;
				case 'f': *out++ = '\f'; break;
				case 'n': *out++ = '\n'; break;
				case 'r': *out++ = '\r'; break;
				case 't': *out++ = '\t'; break;
				case 'u':
				{
					if (end - ptr < 5)
						throw std::runtime_error("Incomplete unicode escape sequence");

					auto code =
						((ptr[1] - '0') << 12) +
						((ptr[2] - '0') << 8) +
						((ptr[3] - '0') << 4) +
						((ptr[4] - '0') << 0);

					if (code < 0x80)
					{
						*out++ = static_cast<char>(code);
					}
					else if (code < 0x800)
					{
						*out++ = static_cast<char>(0xC0 | (code >> 6));
						*out++ = static_cast<char>(0x80 | (code & 0x3F));
					}
					else
					{
						*out++ = static_cast<char>(0xE0 | (code >> 12));
						*out++ = static_cast<char>(0x80 | ((code >> 6) & 0x3F));
						*out++ = static_cast<char>(0x80 | (code & 0x3F));
					}
					ptr += 4;
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown escape sequence");
				}
			}
		}

		result.resize(out - result.data());
		return result;
	}

	// Initializer list constructor for objects
	inline Object::Object(std::initializer_list<std::pair<std::string_view, Value>> items)
		: keys()
		, values()
	{
		// Outside the class because we need 'Value' to be defined
		for (auto& [key, value] : items)
		{
			keys.push_back(std::string{ key });
			values.push_back(value);
		}
	}

	// Write a JSON DOM to a string
	inline std::string Write(Value const& value, Options const& opts = {})
	{
		struct Serializer
		{
			using pair_iter_t = typename Object::citer_t;
			using item_iter_t = typename Array::citer_t;
			enum EArrType
			{
				Normal = 0,
				Basic = 1 << 0,
				Short = 1 << 1,
				Homogeneous = 1 << 2,
			};

			Options const& m_opts;

			Serializer(Options const& opts)
				: m_opts(opts)
			{}
			void DoSerialize(std::string& buf, Value const& value, int indent)
			{
				auto const& val = value.value;
				switch (val.index())
				{
					case Value::TypeIndex::Null:
					{
						buf.append("null");
						break;
					}
					case Value::TypeIndex::Bool:
					{
						buf.append(std::get<bool>(val) ? "true" : "false");
						break;
					}
					case Value::TypeIndex::String:
					{
						buf.append("\"").append(EscapeString(std::get<std::string>(val))).append("\"");
						break;
					}
					case Value::TypeIndex::Number:
					{
						char s[32] = {};
						auto [ptr,ec] = std::to_chars(&s[0], &s[0] + _countof(s), std::get<double>(val));
						if (ec == std::errc{})
							buf.append(&s[0], ptr - &s[0]);
						else
							buf.append("NaN");
						break;
					}
					case Value::TypeIndex::ChildArray:
					{
						auto& arr = std::get<Array>(value.value);
						auto arr_type = ClassifyArray(arr);

						buf.append("[");
						if (arr.size() < m_opts.ParallelSerialise)
						{
							WriteValues(buf, arr.begin(), arr.end(), arr_type, indent);
							if (!HasFlag(arr_type, EArrType::Short))
								Indent(buf, indent);
						}
						else
						{
							auto num_threads = std::thread::hardware_concurrency();
							auto chunk_size = (arr.size() + num_threads - 1) / num_threads;
							std::vector<std::string> chunks(num_threads);
							
							std::for_each(std::execution::par, begin(chunks), end(chunks), [&arr, &chunks, chunk_size, arr_type, indent, this](std::string& chunk)
							{
								auto i = static_cast<ptrdiff_t>(&chunk - chunks.data());
								auto ptr = arr.begin() + std::min((i + 0) * chunk_size, arr.size());
								auto end = arr.begin() + std::min((i + 1) * chunk_size, arr.size());

								chunk.resize(0);
								WriteValues(chunk, ptr, end, arr_type, indent);
							});

							// Write the chunks to 'out'
							char const* comma = "";
							for (auto const& chunk : chunks)
							{
								if (chunk.empty()) break;
								buf.append(comma).append(chunk);
								comma = ",";
							}

							if (!HasFlag(arr_type, EArrType::Short))
								Indent(buf, indent);
						}
						buf.append("]");
						break;
					}
					case Value::TypeIndex::ChildObject:
					{
						auto const& obj = std::get<Object>(value.value);

						buf.append("{");
						if (obj.size() < m_opts.ParallelSerialise)
						{
							WritePairs(buf, obj.begin(), obj.end(), indent);
							Indent(buf, indent);
						}
						else
						{
							auto num_threads = std::thread::hardware_concurrency();
							auto chunk_size = (obj.size() + num_threads - 1) / num_threads;
							std::vector<std::string> chunks(num_threads);
							
							std::for_each(std::execution::par, begin(chunks), end(chunks), [&obj, &chunks, chunk_size, indent, this](std::string& chunk)
							{
								auto i = static_cast<ptrdiff_t>(&chunk - chunks.data());
								auto ptr = obj.begin() + std::min((i + 0) * chunk_size, obj.size());
								auto end = obj.begin() + std::min((i + 1) * chunk_size, obj.size());
								
								chunk.resize(0);
								WritePairs(chunk, ptr, end, indent);
							});

							// Write the chunks to 'out'
							char const* comma = "";
							for (auto const& chunk: chunks)
							{
								if (chunk.empty()) break;
								buf.append(comma).append(chunk);
								comma = ",";
							}

							Indent(buf, indent);
						}
						buf.append("}");
						break;
					}
				}
			}
			EArrType ClassifyArray(Array const& arr) const
			{
				auto arr_type = EArrType::Basic | EArrType::Short | EArrType::Homogeneous;

				if (arr.size() == 0)
					return static_cast<EArrType>(arr_type);

				if (arr.size() > m_opts.ShortArrayLength)
					arr_type &= ~EArrType::Short;

				auto nested_count = 0;
				auto idx0 = arr.begin()->value.index();
				for (auto const& val : arr)
				{
					auto idx = val.value.index();

					if (idx != idx0)
					{
						arr_type &= ~EArrType::Homogeneous;
					}
					if (idx == Value::TypeIndex::ChildArray)
					{
						arr_type &= ~EArrType::Basic;
						nested_count += static_cast<int>(val.to_array().size());
					}
					if (idx == Value::TypeIndex::ChildObject)
					{
						arr_type &= ~EArrType::Basic;
						nested_count += static_cast<int>(val.to_object().size());
					}
					if (idx == Value::TypeIndex::String && std::get<std::string>(val.value).size() > m_opts.ShortStringLength)
					{
						arr_type &= ~EArrType::Short;
					}
				}

				if (nested_count > m_opts.ShortArrayLength)
					arr_type &= ~EArrType::Short;

				return static_cast<EArrType>(arr_type);
			}
			void Indent(std::string& buf, int indent)
			{
				if (!m_opts.Indent || m_opts.IndentString.empty())
					return;

				buf.append("\n");
				for (int i = 0; i != indent; ++i)
					buf.append(m_opts.IndentString);
			}
			void WritePairs(std::string& buf, pair_iter_t beg, pair_iter_t end, int indent)
			{
				char const* comma = "";
				char const* space = m_opts.Indent ? " " : "";
				for (pair_iter_t ptr = beg; ptr != end; ++ptr)
				{
					auto const& [key, val] = *ptr;

					buf.append(comma);
					Indent(buf, indent + 1);
					buf.append("\"").append(EscapeString(key)).append("\":").append(space);
					DoSerialize(buf, val, indent + 1);
					comma = ",";
				}
			}
			void WriteValues(std::string& buf, item_iter_t beg, item_iter_t end, EArrType arr_type, int indent)
			{
				char const* comma = "";
				auto linestart = buf.size();
				bool linewrap = !HasFlag(arr_type, EArrType::Short);

				for (item_iter_t ptr = beg; ptr != end; ++ptr)
				{
					auto const& val = *ptr;

					buf.append(comma);
					if (linewrap)
					{
						Indent(buf, indent + 1);
						linestart = buf.size();
						linewrap = false;
					}
					DoSerialize(buf, val, indent + 1);
					linewrap =
						!HasFlag(arr_type, EArrType::Short) && (
						!HasFlag(arr_type, EArrType::Basic) || buf.size() - linestart > m_opts.LineWrapArrays);

					comma = ",";
				}
			}
			bool HasFlag(EArrType ty, EArrType flags) const
			{
				return (ty & flags) == flags;
			}
		};

		std::string buf;
		buf.reserve(10ULL * 1024 * 1024);

		Serializer s(opts);
		s.DoSerialize(buf, value, 0);
		return std::move(buf);
	}
	inline std::ostream& Write(std::ostream& out, Value const& value, Options const& opts = {})
	{
		return out << Write(value, opts);
	}
	inline void Write(std::filesystem::path filepath, Value const& value, Options const& opts = {})
	{
		std::ofstream file(filepath);
		if (!file.is_open())
			throw std::runtime_error(std::format("Failed to open file '{}'", filepath.string()));

		Write(file, value, opts);
	}

	// Parse a UTF-8 JSON string into a DOM tree.
	inline Value Read(std::string_view src, Options const& opts = {})
	{
		using namespace impl;

		auto start = src;
		try
		{
			return NextValue(src, opts);
		}
		catch (std::runtime_error& ex)
		{
			ex = std::runtime_error(std::format("Parsing failed at offset {} - {}", src.data() - start.data(), ex.what()));
			throw;
		}
	}

	// Read JSON data from a stream into a DOM tree.
	inline Value Read(std::istream& in, Options const& opts = {}, size_t size_estimate = std::dynamic_extent)
	{
		using namespace impl;

		auto data = ReadStreamData(in, size_estimate);
		auto src = std::string_view{ data };
		if (utf8::IsBOM(src))
			src.remove_prefix(3);

		return Read(src, opts);
	}

	// Read a JSON file into a DOM tree.
	inline Value Read(std::filesystem::path const& path, Options const& opts = {})
	{
		std::ifstream file(path, std::ios::binary);
		if (!file.is_open())
			throw std::runtime_error(std::format("Failed to open file '{}'", path.string()));

		auto size = std::filesystem::file_size(path);
		if (size > std::numeric_limits<size_t>::max())
			throw std::runtime_error(std::format("File '{}' is too large", path.string()));

		std::string data(static_cast<size_t>(size), '\0');
		file.read(data.data(), data.size());
		auto src = std::string_view{ data };

		// Skip the BOM if present
		if (utf8::IsBOM(src))
			src.remove_prefix(3);

		return Read(src, opts);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::storage
{
	PRUnitTestClass(JsonTests)
	{
		PRUnitTestMethod(Reading)
		{
			char const test_data[] =
				"{\n"
				"	\"key1\": \"value1\",\n"
				"	\"key2\": 123,\n"
				"	\"key3\": true,\n"
				"	\"key4\": false,\n"
				"	\"key5\": null,\n"
				"	\"key6\": {\n"
				"		\"key7\": \"value7\",\n"
				"		\"key8\": 456,\n"
				"		\"key9\": true,\n"
				"		\"key10\": false,\n"
				"		\"key11\": null,\n"
				"		\"key12\": {\n"
				"			\"key13\": \"value13\",\n"
				"			\"key14\": 789,\n"
				"			\"key15\": true,\n"
				"			\"key16\": false,\n"
				"			\"key17\": null\n"
				"		},\n"
				"		// Comments allowed\n"
				"		\"key18\": [\n"
				"			\"value19\",\n"
				"			123,\n"
				"			true,\n"
				"			false,\n"
				"			null\n"
				"		]\n"
				"	},\n"
				"	\"key20\": [\n"
				"		\"value21\",\n"
				"		123,\n"
				"		true,\n"
				"		false,\n"
				"		null,\n"
				"	]\n"
				"}\n";

			auto root = json::Read(std::string_view{ test_data }, json::Options{ .AllowComments = true, .AllowTrailingCommas = true });
			PR_EXPECT(root["key1"].to<std::string_view>() == "value1");
			PR_EXPECT(root["key2"].to<double>() == 123);
			PR_EXPECT(root["key3"].to<bool>() == true);
			PR_EXPECT(root["key4"].to<bool>() == false);
			PR_EXPECT(root["key5"] == nullptr);
			PR_EXPECT(root["key6"] != nullptr);
			PR_EXPECT(root["key6"]["key12"]["key14"].to<int>() == 789);
			PR_EXPECT(root["key6"]["key12"].to_object().size() == 5);
			PR_EXPECT(root["key6"]["key18"].to_array().empty() == false);
		}
		PRUnitTestMethod(ReadingEscapedStrings)
		{
			char const test_data[] =
				"{\n"
				"	\"EscapedString\": \"This is a string with a \\\"quote\\\" in it\",\n"
				"	\"SearchPaths\": [\n"
				"		\"C:\\\\Work\\\\Path\",\n"
				"	],\n"
				"}\n";

			auto root = json::Read(std::string_view{ test_data }, json::Options{ .AllowComments = true, .AllowTrailingCommas = true });
			PR_EXPECT(root["SearchPaths"][0].to<std::string>() == "C:\\Work\\Path");
			PR_EXPECT(root["EscapedString"].to<std::string>() == "This is a string with a \"quote\" in it");
		}
		PRUnitTestMethod(ReadingStream)
		{
			char const test_data[] =
				"\xEF\xBB\xBF"
				"{\"key\":123}";

			std::istringstream stream(test_data);
			auto root = json::Read(stream);
			PR_EXPECT(root["key"].to<int>() == 123);

			std::istringstream stream_with_estimate("{\"key\":456}");
			root = json::Read(stream_with_estimate, {}, 4);
			PR_EXPECT(root["key"].to<int>() == 456);
		}
		PRUnitTestMethod(Writing)
		{
			json::Document doc;
			auto& root = doc.root();

			root["key1"] = nullptr;
			root["key2"] = true;
			root["key3"] = false;
			root["key4"] = "value1";
			root["key5"] = 123;
			root["key6"] = 456.78;
			root["key7"] = json::Array{ 1, 2, 3, 4 };
			root["key8"] = json::Object{};

			auto& child = root["key8"].to_object();
			child["key10"] = nullptr;
			child["key11"] = true;
			child["key12"] = false;
			child["key13"] = "value2";
			child["key14"] = 456;
			child["key15"] = 123.45;
			child["key16"] = json::Array{
				true,
				"value3",
				80085,
			};
			child["key17"] = json::Object{
				{"one", "1"},
				{"two", 2},
				{"three", true},
			};

			json::Object key9 = {
				{"four", 4},
				{"five", "five"},
			};
			root["key9"] = std::move(key9);
			
			char const* expected = 
				"{\n"
				"	\"key1\": null,\n"
				"	\"key2\": true,\n"
				"	\"key3\": false,\n"
				"	\"key4\": \"value1\",\n"
				"	\"key5\": 123,\n"
				"	\"key6\": 456.78,\n"
				"	\"key7\": [1,2,3,4],\n"
				"	\"key8\": {\n"
				"		\"key10\": null,\n"
				"		\"key11\": true,\n"
				"		\"key12\": false,\n"
				"		\"key13\": \"value2\",\n"
				"		\"key14\": 456,\n"
				"		\"key15\": 123.45,\n"
				"		\"key16\": [true,\"value3\",80085],\n"
				"		\"key17\": {\n"
				"			\"one\": \"1\",\n"
				"			\"two\": 2,\n"
				"			\"three\": true\n"
				"		}\n"
				"	},\n"
				"	\"key9\": {\n"
				"		\"four\": 4,\n"
				"		\"five\": \"five\"\n"
				"	}\n"
				"}";

			PR_EXPECT(root["key1"] == nullptr);
			PR_EXPECT(root["key2"].to<bool>() == true);
			PR_EXPECT(root["key3"].to<bool>() == false);
			PR_EXPECT(root["key4"].to<std::string_view>() == "value1");
			PR_EXPECT(root["key5"].to<int>() == 123);
			PR_EXPECT(root["key6"].to<double>() == 456.78);
			PR_EXPECT(root["key7"] != nullptr);
			
			auto& arr = root["key7"].to_array();
			PR_EXPECT(arr.size() == 4);
			PR_EXPECT(arr[0].to<int>() == 1);
			PR_EXPECT(arr[1].to<int>() == 2);
			PR_EXPECT(arr[2].to<int>() == 3);
			PR_EXPECT(arr[3].to<int>() == 4);

			auto& obj = root["key8"].to_object();
			PR_EXPECT(obj.size() == 8);
			PR_EXPECT(obj["key10"] == nullptr);
			PR_EXPECT(obj["key11"].to<bool>() == true);
			PR_EXPECT(obj["key12"].to<bool>() == false);
			PR_EXPECT(obj["key13"].to<std::string_view>() == "value2");
			PR_EXPECT(obj["key14"].to<int>() == 456);
			PR_EXPECT(obj["key15"].to<double>() == 123.45);

			PR_EXPECT(obj["key16"] != nullptr);
			PR_EXPECT(obj["key16"].size() == 3);
			PR_EXPECT(obj["key16"][0].to<bool>() == true);
			PR_EXPECT(obj["key16"][1].to<std::string_view>() == "value3");
			PR_EXPECT(obj["key16"][2].to<int>() == 80085);

			PR_EXPECT(obj["key17"] != nullptr);
			PR_EXPECT(obj["key17"].size() == 3);
			PR_EXPECT(obj["key17"]["one"].to<std::string_view>() == "1");
			PR_EXPECT(obj["key17"]["two"].to<int>() == 2);
			PR_EXPECT(obj["key17"]["three"].to<bool>() == true);

			std::string str = json::Write(doc, json::Options{ .Indent = true });
			PR_EXPECT(str == expected);
			str = json::Write(doc, json::Options{ .Indent = true, .ParallelSerialise = 2 });
			PR_EXPECT(str == expected);
		}
	};
}
#endif
