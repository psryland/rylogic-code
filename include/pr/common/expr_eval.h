//*************************************************************
// Expression Evaluator
//  Copyright (c) Rylogic Ltd 2008
//*************************************************************
#pragma once
#include <cmath>
#include <array>
#include <string_view>
#include <algorithm>
#include <type_traits>
#include <initializer_list>
#include <cassert>
#include "pr/common/fmt.h"
#include "pr/common/hash.h"
#include "pr/maths/maths.h"
#include "pr/container/vector.h"
#include "pr/container/byte_data.h"

namespace pr::eval
{
	// Expression tokens
	enum class ETok :uint8_t
	{
		None,
		If,
		Else,
		Comma,
		LogOR,
		LogAND,
		BitOR,
		BitXOR,
		BitAND,
		LogEql,
		LogNEql,
		LogLT,
		LogLTEql,
		LogGT,
		LogGTEql,
		LeftShift,
		RightShift,
		Add,
		Sub,
		Mul,
		Div,
		Mod,
		UnaryPlus,
		UnaryMinus,
		Comp,
		Not,
		Abs,
		Ceil,
		Floor,
		Round,
		Min,
		Max,
		Clamp,
		Sin,
		Cos,
		Tan,
		ASin,
		ACos,
		ATan,
		ATan2,
		SinH,
		CosH,
		TanH,
		Exp,
		Log,
		Log10,
		Pow,
		Sqr,
		Sqrt,
		Len2,
		Len3,
		Len4,
		Deg,
		Rad,
		Hash,
		OpenParenthesis,
		CloseParenthesis,
		Value,
		Identifier,
	};

	// Convert a string into a character stream
	template <typename Char> struct char_range
	{
		Char const* m_ptr;
		Char const* m_end;

		char_range() = default;
		char_range(Char const* ptr, Char const* end)
			:m_ptr(ptr)
			,m_end(end)
		{}
		char_range(Char const* ptr)
			:char_range(ptr, ptr + std::char_traits<Char>::length(ptr))
		{}
		char_range(std::basic_string<Char> const& s)
			:char_range(s.data(), s.data() + s.size())
		{}
		char_range(std::basic_string_view<Char> sv)
			:char_range(sv.data(), sv.data() + sv.size())
		{}
		operator bool() const
		{
			return m_ptr != m_end;
		}
		Char operator *() const
		{
			return m_ptr != m_end ? *m_ptr : '\0';
		}
		char_range& operator ++()
		{
			m_ptr += int(m_ptr != m_end);
			return *this;
		}
		char_range& operator += (int n)
		{
			m_ptr += std::min(n, static_cast<int>(m_end - m_ptr));
			return *this;
		}
		char_range operator + (int n) const
		{
			return char_range(m_ptr + std::min(n, static_cast<int>(m_end - m_ptr)), m_end);
		}
		size_t size() const
		{
			return m_end - m_ptr;
		}
		std::basic_string<Char> str() const
		{
			return std::basic_string<Char>(m_ptr, m_end);
		}
		std::basic_string_view<Char> str_view() const
		{
			return std::basic_string_view<Char>(m_ptr, m_end - m_ptr);
		}
	};

	// Identifier - use narrow strings because they're smaller
	using Ident = std::string;

	// Identifier hash
	using IdentHash = pr::hash::HashValue32;
	constexpr IdentHash hashname(std::string_view name)
	{
		return hash::HashCT(name.data(), name.data() + name.size());
	}

	// An integral or floating point value
	struct alignas(16) Val
	{
		enum class EType :int { Unknown, Intg, Real, Intg4, Real4 };

		// The value
		union
		{
			long long m_ll;
			double m_db;
			pr::v4 m_v4;
			pr::iv4 m_i4;
		};
		EType m_ty;
		uint8_t pad[12];

		Val() = default;
		Val(long long ll) noexcept
			:m_ll(ll)
			, m_ty(EType::Intg)
			, pad()
		{
		}
		Val(double db) noexcept
			:m_db(db)
			, m_ty(EType::Real)
			, pad()
		{
		}
		Val(int i) noexcept
			:Val(static_cast<long long>(i))
		{
		}
		Val(float f) noexcept
			:Val(static_cast<double>(f))
		{
		}
		Val(iv4 vec) noexcept
			:m_i4(vec)
			,m_ty(EType::Intg4)
			,pad()
		{}
		Val(v4 vec) noexcept
			:m_v4(vec)
			,m_ty(EType::Real4)
			,pad()
		{}
		Val& operator = (iv4 v) noexcept
		{
			m_i4 = v;
			m_ty = EType::Intg4;
			return *this;
		}
		Val& operator = (v4 v) noexcept
		{
			m_v4 = v;
			m_ty = EType::Real4;
			return *this;
		}
		Val& operator = (unsigned long long v) noexcept
		{
			m_ll = static_cast<long long>(v);
			m_ty = EType::Intg;
			return *this;
		}
		Val& operator = (long long v) noexcept
		{
			m_ll = v;
			m_ty = EType::Intg;
			return *this;
		}
		Val& operator = (double v) noexcept
		{
			m_db = v;
			m_ty = EType::Real;
			return *this;
		}
		Val& operator = (wchar_t v) noexcept
		{
			m_ll = v;
			m_ty = EType::Intg;
			return *this;
		}
		Val& operator = (char v) noexcept
		{
			m_ll = v;
			m_ty = EType::Intg;
			return *this;
		}
		Val& operator = (bool v) noexcept
		{
			m_ll = v;
			m_ty = EType::Intg;
			return *this;
		}

		// True if this value has a known type
		bool has_value() const
		{
			return is_valid(m_ty);
		}

		// Read the value, promoting if needed
		long long ll() const
		{
			if (m_ty == EType::Intg) return m_ll;
			if (m_ty == EType::Real) return static_cast<long long>(m_db);
			if (m_ty == EType::Intg4) throw std::runtime_error("Cannot demote ivec4 to long long");
			if (m_ty == EType::Real4) throw std::runtime_error("Cannot demote vec4 to long long");
			throw std::runtime_error("Value not given. Value type is unknown");
		}
		int intg() const
		{
			return static_cast<int>(ll());
		}
		double db() const
		{
			if (m_ty == EType::Real) return m_db;
			if (m_ty == EType::Intg) return static_cast<double>(m_ll);
			if (m_ty == EType::Intg4) throw std::runtime_error("Cannot demote ivec4 to double");
			if (m_ty == EType::Real4) throw std::runtime_error("Cannot demote vec4 to double");
			throw std::runtime_error("Value not given. Value type is unknown");
		}
		float flt() const
		{
			return static_cast<float>(db());
		}
		pr::iv4 i4() const
		{
			if (m_ty == EType::Intg4) return m_i4;
			if (m_ty == EType::Real4) return To<pr::iv4>(m_v4);
			if (m_ty == EType::Intg) return pr::iv4(static_cast<int>(m_ll));
			if (m_ty == EType::Real) return pr::iv4(static_cast<int>(m_db));
			throw std::runtime_error("Value not given. Value type is unknown");
		}
		pr::v4 v4() const
		{
			if (m_ty == EType::Real4) return m_v4;
			if (m_ty == EType::Intg4) return To<pr::v4>(m_i4);
			if (m_ty == EType::Intg) return pr::v4(static_cast<float>(m_ll));
			if (m_ty == EType::Real) return pr::v4(static_cast<float>(m_db));
			throw std::runtime_error("Value not given. Value type is unknown");
		}

		// Operators
		friend Val  operator +  (Val const& rhs)
		{
			return rhs;
		}
		friend Val  operator -  (Val const& rhs)
		{
			switch (rhs.m_ty)
			{
			case EType::Intg: return Val(-rhs.ll());
			case EType::Real: return Val(-rhs.db());
			case EType::Intg4: return Val(-rhs.i4());
			case EType::Real4: return Val(-rhs.v4());
			default: throw std::runtime_error("Unknown value type for unary minus");
			}
		}
		friend Val  operator +  (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(lhs.ll() + rhs.ll());
			case EType::Real: return Val(lhs.db() + rhs.db());
			case EType::Intg4: return Val(lhs.i4() + rhs.i4());
			case EType::Real4: return Val(lhs.v4() + rhs.v4());
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator -  (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(lhs.ll() - rhs.ll());
			case EType::Real: return Val(lhs.db() - rhs.db());
			case EType::Intg4: return Val(lhs.i4() - rhs.i4());
			case EType::Real4: return Val(lhs.v4() - rhs.v4());
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator *  (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(lhs.ll() * rhs.ll());
			case EType::Real: return Val(lhs.db() * rhs.db());
			case EType::Intg4: return Val(lhs.i4() * rhs.i4());
			case EType::Real4: return Val(lhs.v4() * rhs.v4());
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator /  (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(lhs.ll() / rhs.ll());
			case EType::Real: return Val(lhs.db() / rhs.db());
			case EType::Intg4: return Val(lhs.i4() / rhs.i4());
			case EType::Real4: return Val(lhs.v4() / rhs.v4());
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator %  (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(lhs.ll() % rhs.ll());
			case EType::Real: return Val(std::fmod(lhs.db(), rhs.db()));
			case EType::Intg4: return Val(lhs.i4() % rhs.i4());
			case EType::Real4: return Val(lhs.v4() % rhs.v4());
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator ~  (Val const& rhs)
		{
			switch (rhs.m_ty)
			{
			case EType::Intg: return Val(~rhs.ll());
			case EType::Real: throw std::runtime_error("Twos complement is not supported for double");
			case EType::Intg4: return Val(~rhs.i4());
			case EType::Real4: throw std::runtime_error("Twos complement is not supported for vector4");
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator !  (Val const& rhs)
		{
			switch (rhs.m_ty)
			{
			case EType::Intg: return Val(!rhs.ll());
			case EType::Real: throw std::runtime_error("Logical NOT is not supported for double");
			case EType::Intg4: return Val(!rhs.i4());
			case EType::Real4: throw std::runtime_error("Logical NOT is not supported for vector4");
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator |  (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(lhs.ll() | rhs.ll());
			case EType::Real: throw std::runtime_error("Bitwise OR is not supported for double");
			case EType::Intg4: return Val(lhs.i4() | rhs.i4());
			case EType::Real4: throw std::runtime_error("Bitwise OR is not supported for vector4");
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator &  (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(lhs.ll() & rhs.ll());
			case EType::Real: throw std::runtime_error("Bitwise AND is not supported for double");
			case EType::Intg4: return Val(lhs.i4() & rhs.i4());
			case EType::Real4: throw std::runtime_error("Bitwise AND is not supported for vector4");
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator ^  (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(lhs.ll() ^ rhs.ll());
			case EType::Real: throw std::runtime_error("Bitwise XOR is not supported for double");
			case EType::Intg4: return Val(lhs.i4() ^ rhs.i4());
			case EType::Real4: throw std::runtime_error("Bitwise XOR is not supported for vector4");
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator << (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(static_cast<int64_t>(static_cast<uint64_t>(lhs.ll()) << rhs.ll()));
			case EType::Real: throw std::runtime_error("Bitwise LEFT SHIFT is not supported for double");
			case EType::Intg4: return Val(lhs.i4() << rhs.i4());
			case EType::Real4: throw std::runtime_error("Bitwise LEFT SHIFT is not supported for vector4");
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator >> (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(static_cast<int64_t>(static_cast<uint64_t>(lhs.ll()) >> rhs.ll()));
			case EType::Real: throw std::runtime_error("Bitwise RIGHT SHIFT is not supported for double");
			case EType::Intg4: return Val(lhs.i4() >> rhs.i4());
			case EType::Real4: throw std::runtime_error("Bitwise RIGHT SHIFT is not supported for vector4");
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator || (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(lhs.ll() || rhs.ll());
			case EType::Real: throw std::runtime_error("Logical OR is not supported for double");
			case EType::Intg4: return Val(lhs.i4() || rhs.i4());
			case EType::Real4: throw std::runtime_error("Logical OR is not supported for vector4");
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend Val  operator && (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return Val(lhs.ll() && rhs.ll());
			case EType::Real: throw std::runtime_error("Logical AND is not supported for double");
			case EType::Intg4: return Val(lhs.i4() && rhs.i4());
			case EType::Real4: throw std::runtime_error("Logical AND is not supported for vector4");
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend bool operator == (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return lhs.ll() == rhs.ll();
			case EType::Real: return lhs.db() == rhs.db();
			case EType::Intg4: return lhs.i4() == rhs.i4();
			case EType::Real4: return lhs.v4() == rhs.v4();
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend bool operator != (Val const& lhs, Val const& rhs)
		{
			return !(lhs == rhs);
		}
		friend bool operator <  (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return lhs.ll() < rhs.ll();
			case EType::Real: return lhs.db() < rhs.db();
			case EType::Intg4: return lhs.i4() < rhs.i4();
			case EType::Real4: return lhs.v4() < rhs.v4();
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend bool operator <= (Val const& lhs, Val const& rhs)
		{
			switch (common_type(lhs.m_ty, rhs.m_ty))
			{
			case EType::Intg: return lhs.ll() <= rhs.ll();
			case EType::Real: return lhs.db() <= rhs.db();
			case EType::Intg4: return lhs.i4() <= rhs.i4();
			case EType::Real4: return lhs.v4() <= rhs.v4();
			default: throw std::runtime_error("Unknown value type");
			}
		}
		friend bool operator >  (Val const& lhs, Val const& rhs)
		{
			return !(lhs <= rhs);
		}
		friend bool operator >= (Val const& lhs, Val const& rhs)
		{
			return !(lhs < rhs);
		}

		static constexpr bool is_valid(EType ty) { return ty == EType::Intg || ty == EType::Real || ty == EType::Intg4 || ty == EType::Real4; }
		static constexpr bool is_valid(Val v) { return is_valid(v.m_ty); }
		static constexpr EType common_type(EType lhs, EType rhs)
		{
			return
				(lhs == rhs) ? lhs :
				(lhs == EType::Real4 || rhs == EType::Real4) ? EType::Real4 :
				(lhs == EType::Intg4 || rhs == EType::Intg4) ? EType::Intg4 :
				(lhs == EType::Real || rhs == EType::Real) ? EType::Real :
				EType::Intg;
		}
	};
	static_assert(std::is_trivially_copyable_v<Val>, "Val must be pod for performance");
	static_assert(std::alignment_of_v<Val> == 16, "Val should have 16 byte alignment");

	// A collection of args with some rules enforced
	struct ArgSet
	{
		// Notes:
		//  ArgSet is treated as a value type, so doesn't
		//  include argument name strings for performance.

	private:
		struct Arg
		{
			Val m_value;
			IdentHash m_hash;

			Arg() = default;
			Arg(IdentHash hash, Val const& val)
				:m_value(val)
				,m_hash(hash)
			{}
			bool has_value() const
			{
				return m_value.has_value();
			}
		};
		static_assert(std::is_trivially_copyable_v<Arg>, "Arg must be pod for performance");

		// The names (hashes) and default values of the unique identifiers
		// in the expression (in order of discovery from left to right).
		vector<Arg, 4> m_args;

		// Find the argument matching 'hash'
		Arg const* find(IdentHash hash) const
		{
			for (auto& arg : m_args)
			{
				if (arg.m_hash != hash) continue;
				return &arg;
			}
			return nullptr;
		}
		Arg* find(IdentHash hash)
		{
			return const_cast<Arg*>(std::as_const(*this).find(hash));
		}

		// Add or replace an argument value by hash
		void add_internal(IdentHash hash, Val const& val)
		{
			auto arg = find(hash);
			if (arg == nullptr)
				m_args.push_back(Arg(hash, val));
			else
				arg->m_value = val;
		}

	public:

		ArgSet()
			:m_args()
		{}

		// The number of arguments in the set
		std::size_t size() const
		{
			return m_args.size();
		}

		// The number of arguments without assigned values
		int unassigned_count() const
		{
			int count = 0;
			for (auto& a : m_args)
				count += int(!a.has_value());

			return count;
		}

		// True if 'name' is already an argument
		bool contains(std::string_view name) const
		{
			return contains(hashname(name));
		}

		// True if 'hash' is already an argument
		bool contains(IdentHash hash) const
		{
			return find(hash) != nullptr;
		}

		// Add or replace an unassigned argument
		void add(std::string_view name)
		{
			add_internal(hashname(name), Val{});
		}

		// Add or replace an argument value by name
		void add(std::string_view name, Val const& val)
		{
			assert(Val::is_valid(val));
			add_internal(hashname(name), val);
		}

		// Add or replace an argument value by hash
		void add(IdentHash hash, Val const& val)
		{
			assert(Val::is_valid(val));
			add_internal(hash, val);
		}

		// Add or replace arguments from another arg set
		void add(ArgSet const& rhs)
		{
			for (auto& arg : rhs.m_args)
				add_internal(arg.m_hash, arg.m_value);
		}

		// Assign a value to an argument by the hash of its argument name
		void set(IdentHash hash, Val const& val)
		{
			assert(Val::is_valid(val));
			auto arg = find(hash);
			if (arg == nullptr) throw std::invalid_argument("No argument with this hash exists");
			arg->m_value = val;
		}

		// Assign a value to an argument by name
		void set(std::string_view name, Val const& val)
		{
			set(hashname(name), val);
		}

		// True if all arguments have an assigned value
		bool all_assigned() const
		{
			for (auto& arg : m_args)
			{
				if (arg.has_value()) continue;
				return false;
			}
			return true;
		}

		// Get the value of an argument by name
		Val const& operator()(std::string_view name) const
		{
			auto arg = find(hashname(name));
			if (arg == nullptr) throw std::runtime_error(Fmt("Argument %.*s not found", int(name.size()), name.data()));
			return arg->m_value;
		}

		// Get the value of an argument by hash
		Val const& operator()(IdentHash hash) const
		{
			auto arg = find(hash);
			if (arg == nullptr) throw std::runtime_error(Fmt("Argument (hash: %d) not found", hash));
			return arg->m_value;
		}

		// Get/Set an argument by index (in discovery order)
		Val const& operator [](int i) const
		{
			if (i < 0 || i >= static_cast<int>(m_args.size())) throw std::out_of_range(Fmt("Argument index %d is out of range", i));
			return m_args[i].m_value;
		}
		Val& operator [](int i)
		{
			if (i < 0 || i >= static_cast<int>(m_args.size())) throw std::out_of_range(Fmt("Argument index %d is out of range", i));
			return m_args[i].m_value;
		}

		// Iteration
		Arg const* begin() const
		{
			return m_args.data();
		}
		Arg const* end() const
		{
			return begin() + m_args.size();
		}
		Arg* begin()
		{
			return m_args.data();
		}
		Arg* end()
		{
			return begin() + m_args.size();
		}
	};

	// A fixed size stack for expression evaluation
	template <int S> struct Stack
	{
		Val m_buf[S];
		Val* m_ptr;

		Stack()
			:m_buf()
			,m_ptr(&m_buf[0])
		{}
		std::size_t size() const
		{
			return m_ptr - &m_buf[0];
		}
		void push_back(Val val)
		{
			assert(Val::is_valid(val.m_ty));
			if (m_ptr != &m_buf[0] + S) *m_ptr++ = val;
			else throw std::overflow_error("Insufficient stack space");
		}
		void pop_back()
		{
			if (m_ptr != &m_buf[0]) --m_ptr;
			else throw std::out_of_range("Stack is empty");
		}
		Val back() const
		{
			if (m_ptr != &m_buf[0]) return *(m_ptr - 1);
			else throw std::out_of_range("Stack is empty");
		}
	};

	// A compiled expression
	struct Expression
	{
		using ArgPair = struct { std::string_view name; Val val; };
		using ArgNames = vector<Ident>;

		// The compiled expression
		pr::byte_data<> m_op;

		// The arguments (and default values) of the unique identifiers in the expression (in order of discovery from left to right).
		ArgSet m_args;

		// The unique argument names in the expression
		ArgNames m_arg_names;

		// Is callable/valid test
		explicit operator bool() const
		{
			return !m_op.empty();
		}

		// Evaluate using the given args
		Val operator()(ArgSet const& args) const
		{
			return call(args);
		}

		// Evaluate the expression with the given named arguments. e.g. expr({"x", 1.2}, {"y", 3})
		Val operator()(std::initializer_list<ArgPair> arg_pairs) const
		{
			ArgSet args;
			for (auto& a : arg_pairs)
				args.add(a.name, a.val);
			return call(args);
		}

		// Evaluate the expression using arguments given in order. e.g. expr(1.2, 3)
		template <typename... A> Val operator()(A... a) const
		{
			if constexpr (sizeof...(A) == 0)
			{
				return call();
			}
			else
			{
				// Unpack the arguments into an array
				std::array<Val, sizeof...(A)> values = {a...};
				if (values.size() > m_args.size())
					throw std::runtime_error("Too many arguments given");

				// Update any unassigned arguments in order.
				auto args = m_args;
				for (int i = 0, j = 0; i != int(args.size()) && j != int(values.size()); ++i)
				{
					if (args[i].has_value()) continue;
					args[i] = values[j++];
				}

				// Evaluate
				return call(args);
			}
		}

		// Execute the expression with the given arguments. You can pass 'm_args' to this if you don't care about default values and you've assign values to them all.
		template <typename Stack = Stack<64>>
		Val call(ArgSet const& args = {}) const
		{
			// Check all arguments have a value
			if (!args.all_assigned())
				throw std::runtime_error("Unassigned argument values");

			// Note:
			//  - Parameters are pushed onto the stack in left to right order,
			//    so when popping them off, the first is the rightmost argument.
			//  - Operators should be implemented in 'Val', not here. That way
			//    Val can be extended more easily
			Stack stack;
			for (size_t i = 0, iend = m_op.size(); i != iend;)
			{
				auto tok = m_op.read<ETok>(i);
				switch (tok)
				{
					case ETok::None:
					{
						break;
					}
					case ETok::Identifier:
					{
						auto hash = m_op.read<IdentHash>(i);
						stack.push_back(args(hash));
						break;
					}
					case ETok::Value:
					{
						// Deserialise a 'Val' instance
						auto ty = m_op.read<Val::EType>(i);
						switch (ty)
						{
							case Val::EType::Intg: stack.push_back(m_op.read<long long>(i)); break;
							case Val::EType::Real: stack.push_back(m_op.read<double>(i)); break;
							case Val::EType::Intg4: stack.push_back(m_op.read<v4>(i)); break;
							case Val::EType::Real4: stack.push_back(m_op.read<iv4>(i)); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Add:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for add expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a + b);
						break;
					}
					case ETok::Sub:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for subtract expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a - b);
						break;
					}
					case ETok::Mul:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for multiply expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a * b);
						break;
					}
					case ETok::Div:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for divide expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a / b);
						break;
					}
					case ETok::Mod:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for modulus expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a % b);
						break;
					}
					case ETok::UnaryPlus:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for unary plus expression");
						auto x = stack.back(); stack.pop_back();
						stack.push_back(+x);
						break;
					}
					case ETok::UnaryMinus:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for unary minus expression");
						auto x = stack.back(); stack.pop_back();
						stack.push_back(-x);
						break;
					}
					case ETok::Comp:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for twos complement expression");
						auto x = stack.back(); stack.pop_back();
						stack.push_back(~x);
						break;
					}
					case ETok::Not:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for boolean NOT expression");
						auto x = stack.back(); stack.pop_back();
						stack.push_back(!x);
						break;
					}
					case ETok::LogOR:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for logical OR expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a || b);
						break;
					}
					case ETok::LogAND:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for logical AND expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a && b);
						break;
					}
					case ETok::LogEql:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for equals expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a == b);
						break;
					}
					case ETok::LogNEql:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for not equal expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a != b);
						break;
					}
					case ETok::LogLT:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for less than expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a < b);
						break;
					}
					case ETok::LogLTEql:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for less than or equal expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a <= b);
						break;
					}
					case ETok::LogGT:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for greater than expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a > b);
						break;
					}
					case ETok::LogGTEql:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for greater than or equal expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a >= b);
						break;
					}
					case ETok::BitOR:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for bitwise OR expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a | b);
						break;
					}
					case ETok::BitAND:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for bitwise AND expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a & b);
						break;
					}
					case ETok::BitXOR:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for bitwise XOR expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a ^ b);
						break;
					}
					case ETok::LeftShift:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for bitwise left shift expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a << b);
						break;
					}
					case ETok::RightShift:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for bitwise left shift expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						stack.push_back(a >> b);
						break;
					}
					case ETok::Ceil:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for ceil() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Ceil(x.db())); break;
							case Val::EType::Real: stack.push_back(Ceil(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Ceil(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Ceil(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Floor:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for floor() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Floor(x.db())); break;
							case Val::EType::Real: stack.push_back(Floor(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Floor(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Floor(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Round:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for round() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Round(x.db())); break;
							case Val::EType::Real: stack.push_back(Round(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Round(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Round(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Min:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for min() expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						switch (Val::common_type(a.m_ty, b.m_ty))
						{
							case Val::EType::Intg: stack.push_back(Min(a.ll(), b.ll())); break;
							case Val::EType::Real: stack.push_back(Min(a.db(), b.db())); break;
							case Val::EType::Intg4: stack.push_back(Min(a.i4(), b.i4())); break;
							case Val::EType::Real4: stack.push_back(Min(a.v4(), b.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Max:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for max() expression");
						auto b = stack.back(); stack.pop_back();
						auto a = stack.back(); stack.pop_back();
						switch (Val::common_type(a.m_ty, b.m_ty))
						{
							case Val::EType::Intg: stack.push_back(Max(a.ll(), b.ll())); break;
							case Val::EType::Real: stack.push_back(Max(a.db(), b.db())); break;
							case Val::EType::Intg4: stack.push_back(Max(a.i4(), b.i4())); break;
							case Val::EType::Real4: stack.push_back(Max(a.v4(), b.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Clamp:
					{
						if (stack.size() < 3) throw std::runtime_error("Insufficient arguments for clamp() expression");
						auto mx = stack.back(); stack.pop_back();
						auto mn = stack.back(); stack.pop_back();
						auto x = stack.back(); stack.pop_back();
						switch (Val::common_type(x.m_ty, Val::common_type(mn.m_ty, mx.m_ty)))
						{
							case Val::EType::Intg: stack.push_back(Clamp(x.ll(), mn.ll(), mx.ll())); break;
							case Val::EType::Real: stack.push_back(Clamp(x.db(), mn.db(), mx.db())); break;
							case Val::EType::Intg4: stack.push_back(Clamp(x.i4(), mn.i4(), mx.i4())); break;
							case Val::EType::Real4: stack.push_back(Clamp(x.v4(), mn.v4(), mx.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Abs:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for abs() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Abs(x.ll())); break;
							case Val::EType::Real: stack.push_back(Abs(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Abs(x.i4())); break;
							case Val::EType::Real4: stack.push_back(Abs(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Sin:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for sin() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Sin(x.db())); break;
							case Val::EType::Real: stack.push_back(Sin(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Sin(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Sin(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Cos:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for cos() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Cos(x.db())); break;
							case Val::EType::Real: stack.push_back(Cos(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Cos(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Cos(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Tan:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for tan() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Tan(x.db())); break;
							case Val::EType::Real: stack.push_back(Tan(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Tan(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Tan(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::ASin:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for asin() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Asin(x.db())); break;
							case Val::EType::Real: stack.push_back(Asin(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Asin(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Asin(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::ACos:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for acos() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Acos(x.db())); break;
							case Val::EType::Real: stack.push_back(Acos(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Acos(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Acos(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::ATan:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for atan() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Atan(x.db())); break;
							case Val::EType::Real: stack.push_back(Atan(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Atan(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Atan(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::ATan2:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for atan2() expression");
						auto x = stack.back(); stack.pop_back();
						auto y = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Atan2(y.db(), x.db())); break;
							case Val::EType::Real: stack.push_back(Atan2(y.db(), x.db())); break;
							case Val::EType::Intg4: stack.push_back(Atan2(y.v4(), x.v4())); break;
							case Val::EType::Real4: stack.push_back(Atan2(y.v4(), x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::SinH:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for sinh() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Sinh(x.db())); break;
							case Val::EType::Real: stack.push_back(Sinh(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Sinh(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Sinh(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::CosH:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for cosh() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Cosh(x.db())); break;
							case Val::EType::Real: stack.push_back(Cosh(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Cosh(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Cosh(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::TanH:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for tanh() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Tanh(x.db())); break;
							case Val::EType::Real: stack.push_back(Tanh(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Tanh(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Tanh(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Exp:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for exp() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Exp(x.db())); break;
							case Val::EType::Real: stack.push_back(Exp(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Exp(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Exp(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Log:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for log() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Log(x.db())); break;
							case Val::EType::Real: stack.push_back(Log(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Log(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Log(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Log10:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for log10() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Log10(x.db())); break;
							case Val::EType::Real: stack.push_back(Log10(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Log10(x.v4())); break;
							case Val::EType::Real4: stack.push_back(Log10(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Pow:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for pow() expression");
						auto y = stack.back(); stack.pop_back();
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Pow(x.db(), y.db())); break;
							case Val::EType::Real: stack.push_back(Pow(x.db(), y.db())); break;
							case Val::EType::Intg4: stack.push_back(Pow(x.v4(), y.v4())); break;
							case Val::EType::Real4: stack.push_back(Pow(x.v4(), y.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Sqr:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for sqr() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Sqr(x.ll())); break;
							case Val::EType::Real: stack.push_back(Sqr(x.db())); break;
							case Val::EType::Intg4: stack.push_back(Sqr(x.i4())); break;
							case Val::EType::Real4: stack.push_back(Sqr(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Sqrt:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for sqrt() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Sqrt(x.db())); break;
							case Val::EType::Real: stack.push_back(Sqrt(x.db())); break;
							case Val::EType::Intg4: stack.push_back(CompSqrt(x.v4())); break;
							case Val::EType::Real4: stack.push_back(CompSqrt(x.v4())); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Len2:
					{
						if (stack.size() < 2) throw std::runtime_error("Insufficient arguments for len2() expression");
						auto y = stack.back(); stack.pop_back();
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Len(x.db(), y.db())); break;
							case Val::EType::Real: stack.push_back(Len(x.db(), y.db())); break;
							case Val::EType::Intg4: stack.push_back(CompOp(x.v4(), y.v4(), [](auto x, auto y) { return Len(x, y); })); break;
							case Val::EType::Real4: stack.push_back(CompOp(x.v4(), y.v4(), [](auto x, auto y) { return Len(x, y); })); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Len3:
					{
						if (stack.size() < 3) throw std::runtime_error("Insufficient arguments for len3() expression");
						auto z = stack.back(); stack.pop_back();
						auto y = stack.back(); stack.pop_back();
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Len(x.db(), y.db(), z.db())); break;
							case Val::EType::Real: stack.push_back(Len(x.db(), y.db(), z.db())); break;
							case Val::EType::Intg4: stack.push_back(CompOp(x.v4(), y.v4(), z.v4(), [](auto x, auto y, auto z) { return Len(x, y, z); })); break;
							case Val::EType::Real4: stack.push_back(CompOp(x.v4(), y.v4(), z.v4(), [](auto x, auto y, auto z) { return Len(x, y, z); })); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Len4:
					{
						if (stack.size() < 4) throw std::runtime_error("Insufficient arguments for len4() expression");
						auto w = stack.back(); stack.pop_back();
						auto z = stack.back(); stack.pop_back();
						auto y = stack.back(); stack.pop_back();
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(Len(x.db(), y.db(), z.db(), w.db())); break;
							case Val::EType::Real: stack.push_back(Len(x.db(), y.db(), z.db(), w.db())); break;
							case Val::EType::Intg4: stack.push_back(CompOp(x.v4(), y.v4(), z.v4(), w.v4(), [](auto x, auto y, auto z, auto w) { return Len(x, y, z, w); })); break;
							case Val::EType::Real4: stack.push_back(CompOp(x.v4(), y.v4(), z.v4(), w.v4(), [](auto x, auto y, auto z, auto w) { return Len(x, y, z, w); })); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Deg:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for deg() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(x.db() * maths::E60_by_tau); break;
							case Val::EType::Real: stack.push_back(x.db() * maths::E60_by_tau); break;
							case Val::EType::Intg4: stack.push_back(x.v4() * maths::E60_by_tauf); break;
							case Val::EType::Real4: stack.push_back(x.v4() * maths::E60_by_tauf); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::Rad:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for rad() expression");
						auto x = stack.back(); stack.pop_back();
						switch (x.m_ty)
						{
							case Val::EType::Intg: stack.push_back(x.db() * maths::tau_by_360); break;
							case Val::EType::Real: stack.push_back(x.db() * maths::tau_by_360); break;
							case Val::EType::Intg4: stack.push_back(x.v4() * maths::tau_by_360f); break;
							case Val::EType::Real4: stack.push_back(x.v4() * maths::tau_by_360f); break;
							default: throw std::runtime_error("Unknown value type");
						}
						break;
					}
					case ETok::If:
					{
						if (stack.size() < 1) throw std::runtime_error("Insufficient arguments for if expression");
						auto boolean = stack.back(); stack.pop_back();
						auto jmp = m_op.read<int>(i);
						if (boolean == Val(0))
						{
							i += jmp;

							// If the next instruction is an 'else' statement, skip over it so that the else body
							// gets executed. Remember If == branch-if-zero, Else == branch-always
							if (m_op.at_byte_ofs<ETok>(i) == ETok::Else)
								i += sizeof(ETok) + sizeof(int);
						}
						break;
					}
					case ETok::Else:
					{
						auto jmp = m_op.read<int>(i);
						i += jmp;
						break;
					}
					default:
					{
						throw std::runtime_error("Unknown expression token");
					}
				}
			}
			if (stack.size() != 1)
				throw std::runtime_error("Expression does not evaluate to a single result");

			return stack.back();
		}
	};

	// Returns the precedence of a token
	// How to work out precedence:
	//   NewOp = the op whose precedence you want to know
	//   RhsOp = an op in this list
	// Ask, "If I encounter RhsOp next after NewOp, should NewOp go on hold while
	//  RhsOp is evaluated, or should I stop and evaluate up to NewOp before carrying on".
	//  Also the visa versa case.
	// If NewOp should go on hold, then it has lower precedence (i.e. NewOp < RhsOp)
	// If NewOp needs evaluating, then RhsOp has lower precedence (i.e. RhsOp > NewOp)
	constexpr int Precedence(ETok tok)
	{
		switch (tok)
		{
		default: throw std::runtime_error("Unknown token");
		case ETok::None            : return   0;
		case ETok::Comma           : return  20;
		case ETok::If              : return  30;
		case ETok::Else            : return  30;
		case ETok::LogOR           : return  40;
		case ETok::LogAND          : return  50;
		case ETok::BitOR           : return  60;
		case ETok::BitXOR          : return  70;
		case ETok::BitAND          : return  80;
		case ETok::LogEql          : return  90;
		case ETok::LogNEql         : return  90;
		case ETok::LogLT           : return 100;
		case ETok::LogLTEql        : return 100;
		case ETok::LogGT           : return 100;
		case ETok::LogGTEql        : return 100;
		case ETok::LeftShift       : return 110;
		case ETok::RightShift      : return 110;
		case ETok::Add             : return 120;
		case ETok::Sub             : return 120;
		case ETok::Mul             : return 130;
		case ETok::Div             : return 130;
		case ETok::Mod             : return 130;
		case ETok::UnaryPlus       : return 140;
		case ETok::UnaryMinus      : return 140;
		case ETok::Comp            : return 140;
		case ETok::Not             : return 140;
		case ETok::Abs             : return 200;
		case ETok::Ceil            : return 200;
		case ETok::Floor           : return 200;
		case ETok::Round           : return 200;
		case ETok::Min             : return 200;
		case ETok::Max             : return 200;
		case ETok::Clamp           : return 200;
		case ETok::Sin             : return 200;
		case ETok::Cos             : return 200;
		case ETok::Tan             : return 200;
		case ETok::ASin            : return 200;
		case ETok::ACos            : return 200;
		case ETok::ATan            : return 200;
		case ETok::ATan2           : return 200;
		case ETok::SinH            : return 200;
		case ETok::CosH            : return 200;
		case ETok::TanH            : return 200;
		case ETok::Exp             : return 200;
		case ETok::Log             : return 200;
		case ETok::Log10           : return 200;
		case ETok::Pow             : return 200;
		case ETok::Sqr             : return 200;
		case ETok::Sqrt            : return 200;
		case ETok::Len2            : return 200;
		case ETok::Len3            : return 200;
		case ETok::Len4            : return 200;
		case ETok::Deg             : return 200;
		case ETok::Rad             : return 200;
		case ETok::Hash            : return 200;
		case ETok::OpenParenthesis : return 300;
		case ETok::CloseParenthesis: return 300;
		case ETok::Value           : return 1000;
		case ETok::Identifier      : return 1000;
		}
	}

	// Advance 'expr' to the next non-whitespace character
	template <typename Char> char_range<Char> EatWS(char_range<Char>& expr)
	{
		for (; expr && std::isspace(*expr); ++expr) {}
		return expr;
	}

	// Read a value (greedily) from 'expr'
	template <typename Char> bool ReadValue(char_range<Char>& expr, Val& out)
	{
		// Conversion functions
		auto ReadReal = [](char_range<Char> src, Val& v) -> size_t
		{
			try
			{
				size_t len;
				v = std::stod(src.str(), &len);
				return len;
			}
			catch (std::invalid_argument const&)
			{
				return 0;
			}
		};
		auto RealIntegral = [](char_range<Char> src, Val& v) -> size_t
		{
			Val sval; size_t len0;
			try { sval = std::stoll(src.str(), &len0, 0); }
			catch (std::out_of_range const&) { len0 = 0; }
			catch (std::invalid_argument const&) { len0 = 0; }
				
			Val uval; size_t len1;
			try { uval = std::stoull(src.str(), &len1, 0); }
			catch (std::out_of_range const&) { len1 = 0; }
			catch (std::invalid_argument const&) { len1 = 0; }

			auto len = std::max(len0, len1);
			if (len == 0)
				return 0;

			// Ignore the optional suffix
			src += static_cast<int>(len);
			if (*src == 'u' || *src == 'U') { ++src; ++len; }
			if (*src == 'l' || *src == 'L') { ++src; ++len; }
			if (*src == 'l' || *src == 'L') { ++src; ++len; }

			// Return the integral value
			v = len0 != 0 ? sval :
				len1 != 0 ? uval :
				throw std::runtime_error("at least one is supposed to be valid");
			return len;
		};
		auto ReadLiteral = [](char_range<Char> src, Val& v) -> size_t
		{
			auto beg = src.m_ptr;

			if (*src != '\'')
				return 0;

			// Allow for escaped characters
			if (*++src == '\\')
			{
				switch (*++src)
				{
				default: break;
				case 'a':  v = '\a'; ++src; break;
				case 'b':  v = '\b'; ++src; break;
				case 'f':  v = '\f'; ++src; break;
				case 'n':  v = '\n'; ++src; break;
				case 'r':  v = '\r'; ++src; break;
				case 't':  v = '\t'; ++src; break;
				case 'v':  v = '\v'; ++src; break;
				case '\'': v = '\''; ++src; break;
				case '\"': v = '\"'; ++src; break;
				case '\\': v = '\\'; ++src; break;
				case '\?': v = '\?'; ++src; break;
				case '0':
				case '1':
				case '2':
				case '3':
					{
						// ASCII character in octal
						std::basic_string<Char> oct;
						for (; *src && *src >= '0' && *src <= '7'; ++src) oct.append(1, *src);
						try { v = std::stoll(oct, nullptr, 8); }
						catch (std::out_of_range const&) { return 0; }
						break;
					}
				case 'x':
					{
						// ASCII or UNICODE character in hex
						std::basic_string<Char> hex;
						for (; *src && std::isxdigit(*src); ++src) hex.append(1, *src);
						try { v = std::stoll(hex, nullptr, 16); }
						catch (std::out_of_range const&) { return 0; }
						break;
					}
				}
			}
			else
			{
				v = *src;
				++src;
			}
				
			if (*src != '\'')
				return 0;
					
			++src;
			return src.m_ptr - beg;
		};

		// Greedy read, whichever consumes the most characters is the interpretted value.
		// Prefer integral over real because inegral values get promoted to real.
		Val v; size_t len = 0;
		if (auto l = RealIntegral(expr, v); l > len) { len = l; out = v; }
		if (auto l = ReadReal    (expr, v); l > len) { len = l; out = v; }
		if (auto l = ReadLiteral (expr, v); l > len) { len = l; out = v; }
		if (len == 0)
			return false;

		expr += static_cast<int>(len);
		return true;
	}

	// Read an identifier (greedily) from 'expr'
	template <typename Char> bool ReadIdentifier(char_range<Char>& expr, char_range<Char>& ident)
	{
		// Identifiers start with a letter or underscore
		if (!std::isalpha(*expr) && *expr != '_')
			return false;

		// Greedily read letters, digits, and underscores.
		ident.m_ptr = expr.m_ptr;
		for (; expr && (std::isalnum(*expr) || *expr == '_'); ++expr) {}
		ident.m_end = expr.m_ptr;
		return true;
	}

	// Extract a token from 'expr'
	// 'follows_value' should be true if the preceding expression evaluates to a value
	template <typename Char> ETok Token(char_range<Char>& expr, Val& val, char_range<Char>& ident, bool follows_value)
	{
		// Case insensitive string compare
		auto cmp = [](char_range<Char> s, char const* pattern)
		{
			for (; s && *pattern == std::tolower(*s); ++s, ++pattern) {}
			return *pattern == 0;
		};

		// Skip any leading whitespace
		if (!EatWS(expr))
			return ETok::None;

		// Try an operator
		// Convert Add/Sub to unary plus/minus by looking at the previous expression
		// If the previous expression evaluates to a value then Add/Sub are binary expressions
		switch (std::tolower(*expr))
		{
		default: break;
		case '+': 
			{
				expr += 1;
				return follows_value ? ETok::Add : ETok::UnaryPlus;
			}
		case '-': 
			{
				expr += 1;
				return follows_value ? ETok::Sub : ETok::UnaryMinus;
			}
		case '*': 
			{
				expr += 1;
				return ETok::Mul;
			}
		case '/': 
			{
				expr += 1;
				return ETok::Div;
			}
		case '%':
			{
				expr += 1;
				return ETok::Mod;
			}
		case '~':
			{
				expr += 1;
				return ETok::Comp;
			}
		case ',':
			{
				expr += 1;
				return ETok::Comma;
			}
		case '^':
			{
				expr += 1;
				return ETok::BitXOR;
			}
		case '(':
			{
				expr += 1;
				return ETok::OpenParenthesis;
			}
		case ')': 
			{
				expr += 1;
				return ETok::CloseParenthesis;
			}
		case '?':
			{
				expr += 1;
				return ETok::If;
			}
		case ':':
			{
				expr += 1;
				return ETok::Else;
			}
		case '<':
			{
				if (cmp(expr, "<<")) { expr += 2; return ETok::LeftShift; }
				if (cmp(expr, "<=")) { expr += 2; return ETok::LogLTEql; }
				expr += 1;
				return ETok::LogLT;
			}
		case '>':
			{
				if (cmp(expr, ">>")) { expr += 2; return ETok::RightShift; }
				if (cmp(expr, ">=")) { expr += 2; return ETok::LogGTEql; }
				expr += 1;
				return ETok::LogGT;
			}
		case '|':
			{
				if (cmp(expr, "||")) { expr += 2; return ETok::LogOR; }
				expr += 1;
				return ETok::BitOR;
			}
		case '&':
			{
				if (cmp(expr, "&&")) { expr += 2; return ETok::LogAND; }
				expr += 1;
				return ETok::BitAND;
			}
		case '=':
			{
				if (cmp(expr, "==")) { expr += 2; return ETok::LogEql; }
				break;
			}
		case '!':
			{
				if (cmp(expr, "!=")) { expr += 2; return ETok::LogNEql; }
				expr += 1;
				return ETok::Not;
			}
		case 'a':
			{
				if (cmp(expr, "abs")) { expr += 3; return ETok::Abs; }
				if (cmp(expr, "asin")) { expr += 4; return ETok::ASin; }
				if (cmp(expr, "acos")) { expr += 4; return ETok::ACos; }
				if (cmp(expr, "atan2")) { expr += 5; return ETok::ATan2; }
				if (cmp(expr, "atan")) { expr += 4; return ETok::ATan; }
				break;
			}
		case 'c':
			{
				if (cmp(expr, "clamp")) { expr += 5; return ETok::Clamp; }
				if (cmp(expr, "ceil")) { expr += 4; return ETok::Ceil; }
				if (cmp(expr, "cosh")) { expr += 4; return ETok::CosH; }
				if (cmp(expr, "cos")) { expr += 3; return ETok::Cos; }
				break;
			}
		case 'd':
			{
				if (cmp(expr, "deg")) { expr += 3; return ETok::Deg; }
				break;
			}
		case 'e':
			{
				if (cmp(expr, "exp")) { expr += 3; return ETok::Exp; }
				break;
			}
		case 'f':
			{
				if (cmp(expr, "floor")) { expr += 5; return ETok::Floor; }
				if (cmp(expr, "false")) { expr += 5; val = 0LL; return ETok::Value; }
				break;
			}
		case 'h':
			{
				if (cmp(expr, "hash")) { expr += 4; return ETok::Hash; }
				break;
			}
		case 'l':
			{
				if (cmp(expr, "log10")) { expr += 5; return ETok::Log10; }
				if (cmp(expr, "log")) { expr += 3; return ETok::Log; }
				if (cmp(expr, "len2")) { expr += 4; return ETok::Len2; }
				if (cmp(expr, "len3")) { expr += 4; return ETok::Len3; }
				if (cmp(expr, "len4")) { expr += 4; return ETok::Len4; }
				break;
			}
		case 'm':
			{
				if (cmp(expr, "min")) { expr += 3; return ETok::Min; }
				if (cmp(expr, "max")) { expr += 3; return ETok::Max; }
				break;
			}
		case 'p':
			{
				if (cmp(expr, "pow")) { expr += 3; return ETok::Pow; }
				if (cmp(expr, "phi")) { expr += 3; val = maths::golden_ratio; return ETok::Value; }
				if (cmp(expr, "pi")) { expr += 2; val = maths::tau_by_2; return ETok::Value; }
				break;
			}
		case 'r':
			{
				if (cmp(expr, "round")) { expr += 5; return ETok::Round; }
				if (cmp(expr, "rad")) { expr += 3; return ETok::Rad; }
				break;
			}
		case 's':
			{
				if (cmp(expr, "sinh")) { expr += 4; return ETok::SinH; }
				if (cmp(expr, "sin")) { expr += 3; return ETok::Sin; }
				if (cmp(expr, "sqrt")) { expr += 4; return ETok::Sqrt; }
				if (cmp(expr, "sqr")) { expr += 3; return ETok::Sqr; }
				break;
			}
		case 't':
			{
				if (cmp(expr, "tanh")) { expr += 4; return ETok::TanH; }
				if (cmp(expr, "tan")) { expr += 3; return ETok::Tan; }
				if (cmp(expr, "tau")) { expr += 3; val = maths::tau; return ETok::Value; }
				if (cmp(expr, "true")) { expr += 4; val = 1LL; return ETok::Value; }
				break;
			}
		}

		// Try a variable (identifier)
		if (ReadIdentifier(expr, ident))
			return ETok::Identifier;

		// Try an operand
		if (ReadValue(expr, val))
			return ETok::Value;
			
		return ETok::None;
	}

	// Compile an expression
	// Returns true if a complete expression is consumed from 'expr'. False if the expression is incomplete.
	template <typename Char> bool Compile(char_range<Char>& expr, Expression& compiled, ETok parent_op, bool l2r = true)
	{
		// A flag used to distingush ambiguous operators such as + and -
		bool follows_value = false;

		// Each time round the while loop should result in an operation being
		// added to the expression. Operation tokens result in recursive calls.
		for (; expr; )
		{
			Val val;
			char_range<Char> ident;
			auto expr0 = expr;
			auto tok = Token(expr, val, ident, follows_value);
			follows_value = true;

			// If the next token has lower precedence than the parent operation
			// then return to allow the parent op to evaluate.
			auto prec0 = Precedence(tok);
			auto prec1 = Precedence(parent_op);
			if (prec0 < prec1 || (prec0 == prec1 && l2r))
			{
				// Restore the expr so that the last token is returned to the expr.
				expr = expr0;
				return true;
			}

			switch (tok)
			{
				case ETok::None:
				{
					return expr;
				}
				case ETok::Identifier:
				{
					auto name = Narrow(ident.str_view());
					auto hash = hashname(name);
					auto nue = !compiled.m_args.contains(hash);
					compiled.m_op.push_back(tok);
					compiled.m_op.push_back(hash);
					compiled.m_args.add(name);
					if (nue) compiled.m_arg_names.push_back(std::move(name));
					break;
				}
				case ETok::Value:
				{
					// Manually serialise 'val' to avoid structure padding being added to the CodeBuf.
					// Could make this a function, but it's only used in a couple of places and I don't
					// want to pollute the 'eval' namespace.
					compiled.m_op.push_back(tok);
					compiled.m_op.push_back(val.m_ty);
					val.m_ty == Val::EType::Intg ? compiled.m_op.push_back(val.m_ll) :
						val.m_ty == Val::EType::Real ? compiled.m_op.push_back(val.m_db) :
						throw std::runtime_error("Invalid literal value");
					break;
				}
				case ETok::Add:
				case ETok::Sub:
				case ETok::Mul:
				case ETok::Div:
				case ETok::Mod:
				{
					if (!Compile(expr, compiled, tok)) return false;
					compiled.m_op.push_back(tok);
					break;
				}
				case ETok::UnaryPlus:
				case ETok::UnaryMinus:
				case ETok::Not:
				case ETok::Comp:
				{
					if (!Compile(expr, compiled, tok, false)) return false;
					compiled.m_op.push_back(tok);
					break;
				}
				case ETok::LogOR:
				case ETok::LogAND:
				case ETok::LogEql:
				case ETok::LogNEql:
				case ETok::LogLTEql:
				case ETok::LogGTEql:
				case ETok::LogLT:
				case ETok::LogGT:
				{
					if (!Compile(expr, compiled, tok)) return false;
					compiled.m_op.push_back(tok);
					break;
				}
				case ETok::BitOR:
				case ETok::BitXOR:
				case ETok::BitAND:
				case ETok::LeftShift:
				case ETok::RightShift:
				{
					if (!Compile(expr, compiled, tok)) return false;
					compiled.m_op.push_back(tok);
					break;
				}
				case ETok::Ceil:
				case ETok::Floor:
				case ETok::Round:
				case ETok::Min:
				case ETok::Max:
				case ETok::Clamp:
				case ETok::Abs:
				case ETok::Sin:
				case ETok::Cos:
				case ETok::Tan:
				case ETok::ASin:
				case ETok::ACos:
				case ETok::ATan:
				case ETok::ATan2:
				case ETok::SinH:
				case ETok::CosH:
				case ETok::TanH:
				case ETok::Exp:
				case ETok::Log:
				case ETok::Log10:
				case ETok::Pow:
				case ETok::Sqr:
				case ETok::Sqrt:
				case ETok::Len2:
				case ETok::Len3:
				case ETok::Len4:
				case ETok::Deg:
				case ETok::Rad:
				{
					if (!Compile(expr, compiled, tok)) return false;
					compiled.m_op.push_back(tok);
					break;
				}
				case ETok::Hash:
				{
					// Hash only supports literal strings, which are turned into int64 values
					std::basic_string<Char> str;
					EatWS(expr);
					if (*expr == '(') ++expr; else return false;
					EatWS(expr);
					if (*expr == '"') ++expr; else return false;
					for (bool esc = false; expr && (esc || *expr != '\"'); esc = !esc && *expr == '\\', ++expr) str.append(1, *expr);
					if (*expr == '"') ++expr; else return false;

					auto hash = static_cast<long long>(hash::HashCT(str.data(), str.data() + str.size()));
					compiled.m_op.push_back(ETok::Value);
					compiled.m_op.push_back(Val::EType::Intg);
					compiled.m_op.push_back(hash);
					break;
				}
				case ETok::Comma:
				{
					follows_value = false;
					break;
				}
				case ETok::OpenParenthesis:
				{
					// Parent op is 'None' because it has the lowest precedence
					if (!Compile(expr, compiled, ETok::None)) return false;
					break;
				}
				case ETok::CloseParenthesis:
				{
					// Wait for the parent op to be the 'Open Parenthesis'
					if (parent_op != ETok::None) expr += -1;
					return true;
				}
				case ETok::If:
				{
					// The boolean expression should already be in 'm_op' because it occurs to
					// the left of the ternary ?: operator.

					// Add the 'If' token which is basically a branch-if-zero instruction. i.e.
					// if the previous value is zero, branch past the 'if' body.
					compiled.m_op.push_back(tok);

					// Record the location of the branch offset so it can be updated
					// and write a dummy branch offset in the meantime.
					auto ofs0 = compiled.m_op.size();
					compiled.m_op.push_back(0);

					// Compile the 'if' body
					if (!Compile(expr, compiled, ETok::If))
						return false;

					// Determine the offset to jump over the if body. The jump is from the byte after the jump value.
					auto jmp = static_cast<int>(compiled.m_op.size() - ofs0 - sizeof(int));
					compiled.m_op.at_byte_ofs<int>(ofs0) = jmp;
					break;
				}
				case ETok::Else:
				{
					// Add the 'Else' token which is basically a branch-always instruction.
					// Executing an 'If' statement will jump over this instruction so that the else statement is executed.
					compiled.m_op.push_back(tok);

					// Record the location of the branch offset so it can be updated
					// and write a dummy branch offset in the meantime.
					auto ofs0 = compiled.m_op.size();
					compiled.m_op.push_back(0);

					// Compile the else body
					if (!Compile(expr, compiled, ETok::Else))
						return false;

					// Determine the offset to jump over the else body
					auto jmp = static_cast<int>(compiled.m_op.size() - ofs0 - sizeof(int));
					compiled.m_op.at_byte_ofs<int>(ofs0) = jmp;
					return true;
				}
				default:
				{
					throw std::runtime_error("unknown expression token");
				}
			}
		}
		return true;
	}

	// Compile an expression
	template <typename Char>
	inline Expression Compile(char_range<Char> expr)
	{
		Expression compiled;
		return Compile(expr, compiled, ETok::None)
			? std::move(compiled)
			: throw std::runtime_error("Expression is incomplete");
	}

	// Compile an expression. Throws on syntax error
	inline eval::Expression Compile(std::string_view expr)
	{
		return eval::Compile<char>(expr);
	}
	inline eval::Expression Compile(std::wstring_view expr)
	{
		return eval::Compile<wchar_t>(expr);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(ExprEvalTests)
	{
		using namespace pr::eval;

		{// Compiled expressions
			{ // constants
				auto expr = Compile("2 + x");
				PR_EXPECT(expr(3) == Val(5));
				PR_EXPECT(expr(1.5) == Val(3.5));
				PR_EXPECT(expr(iv4(3)) == Val(iv4(5)));
				PR_EXPECT(expr(v4(1.5f)) == Val(v4(3.5f)));
			}
			{ // add
				auto expr = Compile("x + y");
				PR_EXPECT(expr(3, 5) == Val(8));
				PR_EXPECT(expr({{"y", 5.25}, {"x", -2.5}}) == Val(2.75));
				PR_EXPECT(expr({{"y", iv4(5)}, {"x", iv4(3)}}) == Val(iv4(8)));
				PR_EXPECT(expr({{"y", v4(5.25f)}, {"x", v4(-2.5f)}}) == Val(v4(2.75f)));
			}
			{ // subtract
				auto expr0 = Compile("x - y");
				PR_EXPECT(expr0(3, 5) == Val(-2));
				PR_EXPECT(expr0({{"y", 2.25}, {"x", -2.25}}) == Val(-4.5));
				PR_EXPECT(expr0({{"y", iv4(5)}, {"x", iv4(3)}}) == Val(iv4(-2)));
				PR_EXPECT(expr0({{"y", v4(2.25f)}, {"x", v4(-2.25f)}}) == Val(v4(-4.5f)));

				auto expr1 = Compile("x - y - z");
				PR_EXPECT(expr1(10, 3, 2) == Val(5));
				PR_EXPECT(expr1(10.0, 3.0, 2.0) == Val(5.0));
				PR_EXPECT(expr1(iv4(10), iv4(3), iv4(2)) == Val(iv4(5)));
				PR_EXPECT(expr1(v4(10), v4(3), v4(2)) == Val(v4(5)));
			}
			{ // multiply
				auto expr = Compile("x*x + y");
				PR_EXPECT(expr(3, 5) == Val(14));
				PR_EXPECT(expr({{"y", 2.5}, {"x", -2.5}}) == Val(8.75));
				PR_EXPECT(expr({{"y", iv4(5)}, {"x", iv4(3)}}) == Val(iv4(14)));
				PR_EXPECT(expr({{"y", v4(2.5f)}, {"x", v4(-2.5f)}}) == Val(v4(8.75f)));
			}
			{ // divide
				auto expr = Compile("x / y");
				PR_EXPECT(expr(5, 2) == Val(2));
				PR_EXPECT(expr({{"y", 2.0}, {"x", -5.0}}) == Val(-2.5));
				PR_EXPECT(expr(iv4(5), iv4(2)) == Val(iv4(2)));
				PR_EXPECT(expr(v4(5), v4(2)) == Val(v4(2.5f)));
			}
			{ // modulus
				auto expr = Compile("x % y");
				PR_EXPECT(expr(5, 2) == Val(1));
				PR_EXPECT(FEql(expr(11.3, 3.1).db(), Val(std::fmod(11.3, 3.1)).db()));
				PR_EXPECT(expr(iv4(5), iv4(2)) == Val(iv4(1)));
				PR_EXPECT(expr(v4(5), v4(2)) == Val(v4(1)));
			}
			{ // unary plus
				auto expr = Compile("+x");
				PR_EXPECT(expr(5) == Val(5));
				PR_EXPECT(expr(-5.0) == Val(-5.0));
				PR_EXPECT(expr(iv4(-5)) == Val(iv4(-5)));
				PR_EXPECT(expr(v4(-5.0f)) == Val(v4(-5.0f)));
			}
			{ // unary minus
				auto expr0 = Compile("-x");
				PR_EXPECT(expr0(5) == Val(-5));
				PR_EXPECT(expr0(-5.0) == Val(+5.0));
				PR_EXPECT(expr0(iv4(-1,-2,-3,-4)) == Val(iv4(1,2,3,4)));
				PR_EXPECT(expr0(v4(-1,-2,-3,-4)) == Val(v4(1,2,3,4)));

				auto expr1 = Compile("-+x");
				PR_EXPECT(expr1(5) == Val(-5));
				PR_EXPECT(expr1(iv4(5)) == Val(iv4(-5)));

				auto expr2 = Compile("-++-1");
				PR_EXPECT(expr2() == Val(1));
			}
			{ // twos complement
				auto expr = Compile("~x");
				PR_EXPECT(expr(5) == Val(~5));
				PR_EXPECT(expr(iv4(5)) == Val(iv4(~5)));
			}
			{ // boolean not
				auto expr0 = Compile("!x");
				PR_EXPECT(expr0(5) == Val(0));
				PR_EXPECT(expr0(0) == Val(1));
				PR_EXPECT(expr0(iv4(5)) == Val(iv4(0)));
				PR_EXPECT(expr0(iv4(0)) == Val(iv4(1)));

				auto expr1 = Compile("!!true");
				PR_EXPECT(expr1() == Val(1));

				auto expr2 = Compile("-!!!false");
				PR_EXPECT(expr2() == Val(-1));
			}
			{ // logical OR
				auto expr = Compile("x || y");
				PR_EXPECT(expr(0, 0) == Val(0));
				PR_EXPECT(expr(5, 0) == Val(1));
				PR_EXPECT(expr(0, 5) == Val(1));
				PR_EXPECT(expr(3, 5) == Val(1));
				PR_EXPECT(expr(iv4(0,5,0,3), iv4(0,0,5,5)) == Val(iv4(0,1,1,1)));
			}
			{ // logical AND
				auto expr = Compile("x && y");
				PR_EXPECT(expr(0, 0) == Val(0));
				PR_EXPECT(expr(5, 0) == Val(0));
				PR_EXPECT(expr(0, 5) == Val(0));
				PR_EXPECT(expr(3, 5) == Val(1));
				PR_EXPECT(expr(iv4(0,5,0,3), iv4(0,0,5,5)) == Val(iv4(0,0,0,1)));
			}
			{ // logical Equal
				auto expr = Compile("x == y");
				PR_EXPECT(expr(0, 0) == Val(1));
				PR_EXPECT(expr(5, 0) == Val(0));
				PR_EXPECT(expr(5, 5) == Val(1));
				PR_EXPECT(expr(3.5, 3.5) == Val(1));
				PR_EXPECT(expr(3.5, 5.3) == Val(0));
				PR_EXPECT(expr(iv4(0,0,0,0), iv4(0,0,0,0)) == Val(iv4(1)));
				PR_EXPECT(expr(iv4(1,2,3,4), iv4(1,2,3,4)) == Val(iv4(1)));
				PR_EXPECT(expr(iv4(1,2,3,4), iv4(4,3,2,1)) == Val(iv4(0)));
				PR_EXPECT(expr(v4(0,0,0,0), v4(0,0,0,0)) == Val(v4(1.f)));
				PR_EXPECT(expr(v4(1,2,3,4), v4(1,2,3,4)) == Val(v4(1.f)));
				PR_EXPECT(expr(v4(1,2,3,4), v4(4,3,2,1)) == Val(v4(0.f)));
			}
			{ // logical Not Equal
				auto expr = Compile("x != y");
				PR_EXPECT(expr(0, 0) == Val(0));
				PR_EXPECT(expr(5, 0) == Val(1));
				PR_EXPECT(expr(5, 5) == Val(0));
				PR_EXPECT(expr(3.5, 3.5) == Val(0));
				PR_EXPECT(expr(3.5, 5.3) == Val(1));
				PR_EXPECT(expr(iv4(0,0,0,0), iv4(0,0,0,0)) == Val(iv4(0)));
				PR_EXPECT(expr(iv4(1,2,3,4), iv4(1,2,3,4)) == Val(iv4(0)));
				PR_EXPECT(expr(iv4(1,2,3,4), iv4(4,3,2,1)) == Val(iv4(1)));
				PR_EXPECT(expr(v4(0,0,0,0), v4(0,0,0,0)) == Val(v4(0.f)));
				PR_EXPECT(expr(v4(1,2,3,4), v4(1,2,3,4)) == Val(v4(0.f)));
				PR_EXPECT(expr(v4(1,2,3,4), v4(4,3,2,1)) == Val(v4(1.f)));
			}
			{ // logical less than
				auto expr = Compile("x < y");
				PR_EXPECT(expr(0, 0) == Val(0));
				PR_EXPECT(expr(5, 0) == Val(0));
				PR_EXPECT(expr(3, 5) == Val(1));
				PR_EXPECT(expr(3.5, 3.5) == Val(0));
				PR_EXPECT(expr(3.5, 5.3) == Val(1));
			}
			{ // logical less than or equal
				auto expr = Compile("x <= y");
				PR_EXPECT(expr(0, 0) == Val(1));
				PR_EXPECT(expr(5, 0) == Val(0));
				PR_EXPECT(expr(3, 5) == Val(1));
				PR_EXPECT(expr(3.5, 3.5) == Val(1));
				PR_EXPECT(expr(3.5, 5.3) == Val(1));
			}
			{ // logical greater than
				auto expr = Compile("x > y");
				PR_EXPECT(expr(0, 0) == Val(0));
				PR_EXPECT(expr(5, 0) == Val(1));
				PR_EXPECT(expr(3, 5) == Val(0));
				PR_EXPECT(expr(3.5, 3.5) == Val(0));
				PR_EXPECT(expr(3.5, 5.3) == Val(0));
			}
			{ // logical greater than or equal
				auto expr = Compile("x >= y");
				PR_EXPECT(expr(0, 0) == Val(1));
				PR_EXPECT(expr(5, 0) == Val(1));
				PR_EXPECT(expr(3, 5) == Val(0));
				PR_EXPECT(expr(3.5, 3.5) == Val(1));
				PR_EXPECT(expr(3.5, 5.3) == Val(0));
			}
			{ // bitwise OR
				auto expr = Compile("x | y");
				PR_EXPECT(expr(0x55, 0xAA) == Val(0xFF));
				PR_EXPECT(expr(0x8000, 1) == Val(0x8001));
				PR_EXPECT(expr(iv4(0x55), iv4(0xAA)) == Val(iv4(0xFF)));
				PR_EXPECT(expr(iv4(0x8000), iv4(1)) == Val(iv4(0x8001)));
			}
			{ // bitwise AND
				auto expr = Compile("x & y");
				PR_EXPECT(expr(0xFFF0, 0x0FFF) == Val(0x0FF0));
				PR_EXPECT(expr(iv4(0xFFF0), iv4(0x0FFF)) == Val(iv4(0x0FF0)));
			}
			{ // bitwise XOR
				auto expr = Compile("x ^ y");
				PR_EXPECT(expr(0xA5, 0x55) == Val(0xF0));
				PR_EXPECT(expr(iv4(0xA5), iv4(0x55)) == Val(iv4(0xF0)));
			}
			{ // left shift
				auto expr = Compile("x << y");
				PR_EXPECT(expr(0x3, 2) == Val(0xC));
				PR_EXPECT(expr(iv4(0x3), 2) == Val(iv4(0xC)));
			}
			{ // right shift
				auto expr = Compile("x >> y");
				PR_EXPECT(expr(0xC, 2) == Val(0x3));
				PR_EXPECT(expr(iv4(0xC), iv4(1,2,3,4)) == Val(iv4(0x6,0x3,0x1,0x0)));
			}
			{ // ceil
				auto expr = Compile("ceil(x)");
				PR_EXPECT(FEql(expr(3.4).db(), 4.0));
				PR_EXPECT(FEql(expr(-3.4).db(), -3.0));
				PR_EXPECT(FEql(expr(v4(-1.2f, 3.4f, -5.6f, 7.8f)).v4(), v4(-1.0f, 4.0f, -5.0f, 8.0f)));
			}
			{ // floor
				auto expr = Compile("floor(x)");
				PR_EXPECT(FEql(expr(3.4).db(), 3.0));
				PR_EXPECT(FEql(expr(-3.4).db(), -4.0));
				PR_EXPECT(FEql(expr(v4(-1.2f, 3.4f, -5.6f, 7.8f)).v4(), v4(-2.0f, 3.0f, -6.0f, 7.0f)));
			}
			{ // round
				auto expr = Compile("round(x)");
				PR_EXPECT(FEql(expr(+3.5).db(), +4.0));
				PR_EXPECT(FEql(expr(-3.5).db(), -4.0));
				PR_EXPECT(FEql(expr(+3.2).db(), +3.0));
				PR_EXPECT(FEql(expr(-3.2).db(), -3.0));
				PR_EXPECT(FEql(expr(v4(-1.2f, 3.4f, -5.6f, 7.8f)).v4(), v4(-1.0f, 3.0f, -6.0f, 8.0f)));
			}
			{ // min
				auto expr = Compile("min(x,y)");
				PR_EXPECT(FEql(expr(-3.2, -3.4).db(), -3.4));
				PR_EXPECT(FEql(expr(v4(-1.2f, 3.4f, -5.6f, 7.8f), v4(-0.2f, 3.5f, -5.7f, 7.7f)).v4(), v4(-1.2f, 3.4f, -5.7f, 7.7f)));
			}
			{ // max
				auto expr = Compile("max(x,y)");
				PR_EXPECT(FEql(expr(-3.2, -3.4).db(), -3.2));
				PR_EXPECT(FEql(expr(v4(-1.2f, 3.4f, -5.6f, 7.8f), v4(-0.2f, 3.5f, -5.7f, 7.7f)).v4(), v4(-0.2f, 3.5f, -5.6f, 7.8f)));
			}
			{ // clamp
				auto expr = Compile("clamp(x,mn,mx)");
				PR_EXPECT(FEql(expr(+10.0, -3.4, -3.2).db(), -3.2));
				PR_EXPECT(FEql(expr(-10.0, -3.4, -3.2).db(), -3.4));
				PR_EXPECT(FEql(expr(v4(-0.2f, 3.4f, -5.6f, 7.8f), -v4One, +v4One).v4(), v4(-0.2f, 1.0f, -1.0f, 1.0f)));
			}
			{ // sin
				auto expr = Compile("sin(x)");
				PR_EXPECT(FEql(expr(-0.8).db(), Sin(-0.8)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Sin(-0.2f), Sin(0.4f), Sin(-0.6f), Sin(0.8f))));
			}
			{ // cos
				auto expr = Compile("cos(x)");
				PR_EXPECT(FEql(expr(0.2).db(), Cos(0.2)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Cos(-0.2f), Cos(0.4f), Cos(-0.6f), Cos(0.8f))));
			}
			{ // tan
				auto expr = Compile("tan(x)");
				PR_EXPECT(FEql(expr(0.2).db(), Tan(0.2)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Tan(-0.2f), Tan(0.4f), Tan(-0.6f), Tan(0.8f))));
			}
			{ // asin
				auto expr = Compile("asin(x)");
				PR_EXPECT(FEql(expr(-0.8).db(), Asin(-0.8)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Asin(-0.2f), Asin(0.4f), Asin(-0.6f), Asin(0.8f))));
			}
			{ // acos
				auto expr = Compile("acos(x)");
				PR_EXPECT(FEql(expr(-0.8).db(), Acos(-0.8)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Acos(-0.2f), Acos(0.4f), Acos(-0.6f), Acos(0.8f))));
			}
			{ // atan
				auto expr = Compile("atan(x)");
				PR_EXPECT(FEql(expr(-0.8).db(), Atan(-0.8)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Atan(-0.2f), Atan(0.4f), Atan(-0.6f), Atan(0.8f))));
			}
			{ // atan2
				auto expr = Compile("atan2(y,x)");
				PR_EXPECT(FEql(expr(2.3, -3.9).db(), Atan2(2.3, -3.9)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f), v4(+0.1f, -0.3f, +0.5f, -0.7f)).v4(), v4(Atan2(-0.2f, 0.1f), Atan2(0.4f, -0.3f), Atan2(-0.6f, 0.5f), Atan2(0.8f, -0.7f))));
			}
			{ // sinh
				auto expr = Compile("sinh(x)");
				PR_EXPECT(FEql(expr(-0.8).db(), Sinh(-0.8)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Sinh(-0.2f), Sinh(0.4f), Sinh(-0.6f), Sinh(0.8f))));
			}
			{ // cosh
				auto expr = Compile("cosh(x)");
				PR_EXPECT(FEql(expr(-0.8).db(), Cosh(-0.8)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Cosh(-0.2f), Cosh(0.4f), Cosh(-0.6f), Cosh(0.8f))));
			}
			{ // tanh
				auto expr = Compile("tanh(x)");
				PR_EXPECT(FEql(expr(-0.8).db(), Tanh(-0.8)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Tanh(-0.2f), Tanh(0.4f), Tanh(-0.6f), Tanh(0.8f))));
			}
			{ // exp
				auto expr = Compile("exp(x)");
				PR_EXPECT(FEql(expr(2.3).db(), Exp(2.3)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Exp(-0.2f), Exp(0.4f), Exp(-0.6f), Exp(0.8f))));
			}
			{ // log
				auto expr = Compile("log(x)");
				PR_EXPECT(FEql(expr(209.3).db(), Log(209.3)));
				PR_EXPECT(FEql(expr(v4(0.2f, 0.4f, 0.6f, 0.8f)).v4(), v4(Log(0.2f), Log(0.4f), Log(0.6f), Log(0.8f))));
			}
			{ // log10
				auto expr = Compile("log10(x)");
				PR_EXPECT(FEql(expr(209.3).db(), Log10(209.3)));
				PR_EXPECT(FEql(expr(v4(1.2f, 10.4f, 100.6f, 1000.8f)).v4(), v4(Log10(1.2f), Log10(10.4f), Log10(100.6f), Log10(1000.8f))));
			}
			{ // pow
				auto expr = Compile("pow(x,y)");
				PR_EXPECT(FEql(expr(2.3, -1.3).db(), Pow(2.3, -1.3)));
				PR_EXPECT(FEql(expr(v4(0.2f, 0.4f, 0.6f, 0.8f), v4(+0.1f, -0.3f, +0.5f, -0.7f)).v4(), v4(Pow(0.2f, 0.1f), Pow(0.4f, -0.3f), Pow(0.6f, 0.5f), Pow(0.8f, -0.7f))));
			}
			{ // sqrt
				auto expr = Compile("sqrt(x)");
				PR_EXPECT(FEql(expr(2.3).db(), Sqrt(2.3)));
				PR_EXPECT(FEql(expr(v4(0.2f, 0.4f, 0.6f, 0.8f)).v4(), v4(Sqrt(0.2f), Sqrt(0.4f), Sqrt(0.6f), Sqrt(0.8f))));
			}
			{ // sqr
				auto expr = Compile("sqr(x)");
				PR_EXPECT(FEql(expr(-2.3).db(), Sqr(-2.3)));
				PR_EXPECT(FEql(expr(v4(-0.2f, 0.4f, -0.6f, 0.8f)).v4(), v4(Sqr(-0.2f), Sqr(0.4f), Sqr(-0.6f), Sqr(0.8f))));
			}
			{ // len2
				auto expr = Compile("len2(x,y)");
				PR_EXPECT(FEql(expr(3, 4).db(), Val(std::sqrt(3.0 * 3.0 + 4.0 * 4.0)).db()));
			}
			{ // len3
				auto expr = Compile("len3(x,y,z)");
				PR_EXPECT(FEql(expr(3, 4, 5).db(), Val(std::sqrt(3.0 * 3.0 + 4.0 * 4.0 + 5.0 * 5.0)).db()));
			}
			{ // len4
				auto expr = Compile("len4(x,y,z,w)");
				PR_EXPECT(FEql(expr(3, 4, 5, 6).db(), Val(std::sqrt(3.0 * 3.0 + 4.0 * 4.0 + 5.0 * 5.0 + 6.0 * 6.0)).db()));
			}
			{ // deg
				auto expr = Compile("deg(x)");
				PR_EXPECT(FEql(expr(-1.24).db(), Val(-1.24 * maths::E60_by_tau).db()));
			}
			{ // rad
				auto expr = Compile("rad(x)");
				PR_EXPECT(FEql(expr(241.32).db(), Val(241.32 * maths::tau_by_360).db()));
			}
			{ // hash
				auto expr = Compile("hash(\"A String\")");
				PR_EXPECT(expr() == Val(hash::HashCT("A String")));
			}
			{ // long expression, no variables
				auto expr = Compile("sqr(sqrt(2.3)*-abs(4%2)/15.0-tan(TAU/-6))");
				auto res = Sqr(Sqrt(2.3) * -Abs(4 % 2) / 15.0 - Tan(maths::tau / -6));
				PR_EXPECT(FEql(expr().db(), res));
			}
			{ // long expression, with variables
				auto expr = Compile("sqr(sqrt(x)*-abs(y%3)/x-tan(TAU/-y))");
				auto res = Sqr(Sqrt(2.3) * -Abs(13 % 3) / 2.3 - Tan(maths::tau / -13));
				PR_EXPECT(FEql(expr(2.3, 13).db(), res));
				PR_EXPECT(FEql(expr(v4(2.3f), v4(13.0f)).v4(), v4(float(res))));
			}
			{ // large values
				auto expr = Compile("123456789000000 / 2");
				PR_EXPECT(expr().ll() == 123456789000000LL / 2);
			}
			{ // operator ?:
				auto expr0 = Compile("x != y ? 5 : 6");
				PR_EXPECT(expr0(1, 2).ll() == Val(5));
				PR_EXPECT(expr0(1, 1).ll() == Val(6));

				auto expr1 = Compile("true ? x : 6 + 1");
				PR_EXPECT(expr1(1).ll() == Val(1));

				auto expr2 = Compile("false ? x : 6 + 1");
				PR_EXPECT(expr2(1).ll() == Val(7));
			}
			{ // Parenthesis
				auto expr = Compile("sqr(-2) ? (1+2) : max(-2,-3)");
				PR_EXPECT(expr() == Val(3));
			}
			{ // literals
				auto expr = Compile("'1' + '2'");
				PR_EXPECT(expr() == Val('1' + '2'));
			}
		}
	}
}
#endif





	#if 0
	// Evaluate an expression.
	// Called recursively for each operation within an expression.
	// 'parent_op' is used to determine precedence order.
	template <typename Char> bool Eval(char_range<Char>& expr, std::span<Val> result, int ridx, ETok parent_op, bool l2r = true)
	{
		// Each time round the while loop should result in a value.
		// Operation tokens result in recursive calls.
		bool follows_value = false;
		for (; expr; )
		{
			Val val;
			char_range<Char> ident;
			auto expr0 = expr;
			auto tok = Token(expr, val, ident, follows_value);
			follows_value = true;

			// If the next token has lower precedence than the parent operation
			// then return to allow the parent op to evaluate.
			auto prec0 = Precedence(tok);
			auto prec1 = Precedence(parent_op);
			if (prec0 < prec1 || (prec0 == prec1 && l2r))
			{
				// Restore the expr so that the last token is returned to the expr.
				expr = expr0;
				return true;
			}
				
			switch (tok)
			{
			case ETok::None:
				{
					return expr;
				}
			case ETok::Identifier:
				{
					throw std::runtime_error("evaluated expressions cannot contain variables");
				}
			case ETok::Value:
				{
					result[ridx] = val;
					break;
				}
			case ETok::Add:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] + rhs[0];
					break;
				}
			case ETok::Sub:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] - rhs[0];
					break;
				}
			case ETok::Mul:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] * rhs[0];
					break;
				}
			case ETok::Div:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] / rhs[0];
					break;
				}
			case ETok::Mod:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] % rhs[0];
					break;
				}
			case ETok::UnaryPlus:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok, false)) return false;
					result[ridx] = rhs[0];
					break;
				}
			case ETok::UnaryMinus:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok, false)) return false;
					result[ridx] = -rhs[0];
					break;
				}
			case ETok::Comp:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok, false)) return false;
					result[ridx] = ~rhs[0];
					break;
				}
			case ETok::Not:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok, false)) return false;
					result[ridx] = !rhs[0];
					break;
				}
			case ETok::LogOR:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] || rhs[0];
					break;
				}
			case ETok::LogAND:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] && rhs[0];
					break;
				}
			case ETok::LogEql:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] == rhs[0];
					break;
				}
			case ETok::LogNEql:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] != rhs[0];
					break;
				}
			case ETok::LogLT:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] < rhs[0];
					break;
				}
			case ETok::LogLTEql:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] <= rhs[0];
					break;
				}
			case ETok::LogGT:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] > rhs[0];
					break;
				}
			case ETok::LogGTEql:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] >= rhs[0];
					break;
				}
			case ETok::BitOR:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] | rhs[0];
					break;
				}
			case ETok::BitXOR:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] ^ rhs[0];
					break;
				}
			case ETok::BitAND:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] & rhs[0];
					break;
				}
			case ETok::LeftShift:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] << rhs[0];
					break;
				}
			case ETok::RightShift:
				{
					std::array<Val, 1> rhs;
					if (!Eval(expr, rhs, 0, tok)) return false;
					result[ridx] = result[ridx] >> rhs[0];
					break;
				}
			case ETok::Ceil:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					switch (x.m_ty)
					{
					case Val::EType::Intg: result[ridx].push_back(x.db() * maths::E60_by_tau); break;
					case Val::EType::Real: result[ridx].push_back(x.db() * maths::E60_by_tau); break;
					case Val::EType::Vec4: result[ridx].push_back(CompOp(x.v4(), [](auto x) { return x * maths::E60_by_tauf; })); break;
					default: throw std::runtime_error("Unknown value type");
					}
//todo - update needed
					result[ridx] = std::ceil(args[0].db());
					break;
				}
			case ETok::Floor:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::floor(args[0].db());
					break;
				}
			case ETok::Round:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::round(args[0].db());
					break;
				}
			case ETok::Min:
				{
					std::array<Val, 2> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = args[0] < args[1] ? args[0] : args[1];
					break;
				}
			case ETok::Max:
				{
					std::array<Val, 2> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = args[0] < args[1] ? args[1] : args[0];
					break;
				}
			case ETok::Clamp:
				{
					std::array<Val, 3> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = args[0] < args[1] ? args[1] : args[2] < args[0] ? args[2] : args[0];
					break;
				}
			case ETok::Abs:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = 
						args[0].m_ty == Val::EType::Intg ? std::abs(args[0].ll()) :
						args[0].m_ty == Val::EType::Real ? std::abs(args[0].db()) :
						throw std::runtime_error("Invalid argument for 'abs'");
					break;
				}
			case ETok::Sin:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::sin(args[0].db());
					break;
				}
			case ETok::Cos:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::cos(args[0].db());
					break;
				}
			case ETok::Tan:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::tan(args[0].db());
					break;
				}
			case ETok::ASin:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::asin(args[0].db());
					break;
				}
			case ETok::ACos:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::acos(args[0].db());
					break;
				}
			case ETok::ATan:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::atan(args[0].db());
					break;
				}
			case ETok::ATan2:
				{
					std::array<Val, 2> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::atan2(args[0].db(), args[1].db());
					break;
				}
			case ETok::SinH:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::sinh(args[0].db());
					break;
				}
			case ETok::CosH:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::cosh(args[0].db());
					break;
				}
			case ETok::TanH:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::tanh(args[0].db());
					break;
				}
			case ETok::Exp:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::exp(args[0].db());
					break;
				}
			case ETok::Log:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::log(args[0].db());
					break;
				}
			case ETok::Log10:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::log10(args[0].db());
					break;
				}
			case ETok::Pow:
				{
					std::array<Val, 2> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::pow(args[0].db(), args[1].db());
					break;
				}
			case ETok::Sqr:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = args[0] * args[0];
					break;
				}
			case ETok::Sqrt:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::sqrt(args[0].db());
					break;
				}
			case ETok::Len2:
				{
					std::array<Val, 2> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::sqrt(0.0 + args[0].db()*args[0].db() + args[1].db()*args[1].db());
					break;
				}
			case ETok::Len3:
				{
					std::array<Val, 3> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::sqrt(0.0 + args[0].db()*args[0].db() + args[1].db()*args[1].db() + args[2].db()*args[2].db());
					break;
				}
			case ETok::Len4:
				{
					std::array<Val, 4> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = std::sqrt(0.0 + args[0].db()*args[0].db() + args[1].db()*args[1].db() + args[2].db()*args[2].db() + args[3].db()*args[3].db());
					break;
				}
			case ETok::Deg:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = args[0].db() * (360.0 / TAU);
					break;
				}
			case ETok::Rad:
				{
					std::array<Val, 1> args;
					if (!Eval(expr, args, 0, tok)) return false;
					result[ridx] = args[0].db() * (TAU / 360.0);
					break;
				}
			case ETok::Hash:
				{
					std::basic_string<Char> str;
					EatWS(expr);
					if (*expr == '(') ++expr; else return false;
					EatWS(expr);
					if (*expr == '"') ++expr; else return false;
					for (bool esc = false; expr && (esc || *expr != '\"'); esc = !esc && *expr == '\\', ++expr) str.append(1, *expr);
					if (*expr == '"') ++expr; else return false;
					result[ridx] = static_cast<long long>(hash::HashCT(str.data(), str.data() + str.size()));
					break;
				}
			case ETok::Comma:
				{
					if (ridx + 1 == static_cast<int>(result.size()))
						throw std::runtime_error("too many parameters");

					++ridx;
					follows_value = false;
					break;
				}
			case ETok::OpenParenthesis:
				{
					// Parent op is 'None' because it has the lowest precedence
					if (!Eval(expr, result, ridx, ETok::None)) return false;
					break;
				}
			case ETok::CloseParenthesis:
				{
					// Wait for the parent op to be the 'Open Parenthesis'
					if (parent_op != ETok::None) expr += -1;
					return true;
				}
			case ETok::If:
				{
					std::array<Val, 2> args;
					if (!Eval(expr, args, 0, ETok::None)) return false;
					result[ridx] = result[ridx].ll() != 0 ? args[0] : args[1];
					break;
				}
			case ETok::Else:
				{
					if (ridx + 1 == static_cast<int>(result.size()))
						throw std::runtime_error("result buffer too small");

					if (!Eval(expr, result, ++ridx, ETok::Else)) return false;
					++ridx;
					return true;
				}
			default:
				{
					throw std::runtime_error("unknown expression token");
				}
			}
		}
		return true;
	}

	// Evaluate an expression.
	template <typename Char>
	inline Val Evaluate(char_range<Char> expr)
	{
		auto expr = Compile(expr);
		return expr();
		//std::array<Val, 1> result;
		//Eval(expr, result, 0, ETok::None);
		//return result[0];
	}
	template <typename Char, typename ResType>
	inline bool Evaluate(char_range<Char> expr, ResType& out)
	{
		try
		{
			auto expr = Compile(expr);
			auto result = expr();

			if constexpr(std::is_integral_v<ResType>)
				out = static_cast<ResType>(val.ll());
			else if constexpr(std::is_floating_point_v<ResType>)
				out = static_cast<ResType>(val.db());
			else
				static_assert(false, "Unsupported result type");
			
			return true;
		}
		catch (std::exception const&)
		{
			return false;
		}

		//try
		//{
		//	auto val = Evaluate<Char>(expr);
		//	if constexpr(std::is_integral_v<ResType>)
		//		out = static_cast<ResType>(val.ll());
		//	else if constexpr(std::is_floating_point_v<ResType>)
		//		out = static_cast<ResType>(val.db());
		//	else
		//		static_assert(false, "Unsupported result type");
		//
		//	return true;
		//}
		//catch (std::exception const&)
		//{
		//	return false;
		//}
	}
	// Evaluate an expression. Throws on syntax error.
	inline eval::Val Evaluate(std::string_view expr)
	{
		return eval::Evaluate<char>(expr);
	}
	inline eval::Val Evaluate(std::wstring_view expr)
	{
		return eval::Evaluate<wchar_t>(expr);
	}
	template <typename ResType> inline bool Evaluate(std::string_view expr, ResType& out)
	{
		return eval::Evaluate<char, ResType>(expr, out);
	}
	template <typename ResType> inline bool Evaluate(std::wstring_view expr, ResType& out)
	{
		return eval::Evaluate<wchar_t, ResType>(expr, out);
	}
	#endif
	#if 0
	namespace unitests::expr_eval
	{
		template <typename ResType> bool Expr(std::string_view expr, ResType result) { static_assert(false); }
		template <> bool Expr(std::string_view expr, double                  result) { double        val; return Evaluate(expr, val) && FEql(val, result); }
		template <> bool Expr(std::string_view expr, float                   result) { float         val; return Evaluate(expr, val) && FEql(val, result); }
		template <> bool Expr(std::string_view expr, uint64_t                result) { uint64_t      val; return Evaluate(expr, val) && val == result; }
		template <> bool Expr(std::string_view expr, int64_t                 result) { int64_t       val; return Evaluate(expr, val) && val == result; }
		template <> bool Expr(std::string_view expr, unsigned long           result) { unsigned long val; return Evaluate(expr, val) && val == result; }
		template <> bool Expr(std::string_view expr, long                    result) { long          val; return Evaluate(expr, val) && val == result; }
		template <> bool Expr(std::string_view expr, uint32_t                result) { uint32_t      val; return Evaluate(expr, val) && val == result; }
		template <> bool Expr(std::string_view expr, int                     result) { int           val; return Evaluate(expr, val) && val == result; }
		template <> bool Expr(std::string_view expr, bool                    result) { bool          val; return Evaluate(expr, val) && val == result; }

		template <typename ValueType>
		bool ReadValue(pr::eval::char_range<char> expr, ValueType expected_result)
		{
			pr::eval::Val val;
			if (!pr::eval::ReadValue(expr, val))
				return false;

			if constexpr (std::is_integral_v<ValueType> && std::is_unsigned_v<ValueType>)
				return static_cast<uint64_t>(expected_result) == static_cast<uint64_t>(val.ll());
			else if constexpr (std::is_integral_v<ValueType>)
				return expected_result == static_cast<ValueType>(val.ll());
			else if constexpr (std::is_floating_point_v<ValueType>)
				return expected_result == static_cast<ValueType>(val.db());
			else
				static_assert(false);
		}
	}
	#endif
	#if 0
	{
		#define VAL(exp) ReadValue(#exp, (exp))
		PR_EXPECT(VAL(1));
		PR_EXPECT(VAL(1.0));
		PR_EXPECT(VAL(-1));
		PR_EXPECT(VAL(-1.0));
		PR_EXPECT(VAL(10U));
		PR_EXPECT(VAL(100L));
		PR_EXPECT(VAL(-100L));
		PR_EXPECT(VAL(0x1000UL));
		PR_EXPECT(VAL(0x7FFFFFFF));
		PR_EXPECT(VAL(0x80000000));
		PR_EXPECT(VAL(0xFFFFFFFF));
		PR_EXPECT(VAL(0xFFFFFFFFU));
		PR_EXPECT(VAL(0xFFFFFFFFULL));
		PR_EXPECT(VAL(0x7FFFFFFFFFFFFFFFLL));
		PR_EXPECT(VAL(0xFFFFFFFFFFFFFFFFULL));
		#undef VAL
	}
	{
		#define EXPR(exp) Expr(#exp, (exp))
		PR_EXPECT(EXPR(1.0));
		PR_EXPECT(EXPR(+1.0));
		PR_EXPECT(EXPR(-1.0));
		PR_EXPECT(EXPR(-(1.0 + 2.0)));
		PR_EXPECT(EXPR(8.0 * -1.0));
		PR_EXPECT(EXPR(4.0 * -1.0 + 2.0));
		PR_EXPECT(EXPR(1.0 + +2.0));
		PR_EXPECT(EXPR(1.0 - -2.0));
		PR_EXPECT(EXPR(1.0 - 2.0 - 3.0 + 4.0));
		PR_EXPECT(EXPR(1.0 * +2.0));
		PR_EXPECT(EXPR(1 / 2));
		PR_EXPECT(EXPR(1.0 / 2.0));
		PR_EXPECT(EXPR(1.0 / 2.0 + 3.0));
		PR_EXPECT(EXPR(1.0 / 2.0 * 3.0));
		PR_EXPECT(EXPR((1 || 0) && 2));
		PR_EXPECT(EXPR(((13 ^ 7) | 6) & 14));
		PR_EXPECT(EXPR((8 < 9) + (3 <= 3) + (8 > 9) + (2 >= 2) + (1 != 2) + (2 == 2)));
		PR_EXPECT(EXPR(1.0 + 2.0 * 3.0 - 4.0));
		PR_EXPECT(EXPR(2.0 * 3.0 + 1.0 - 4.0));
		PR_EXPECT(EXPR(1.0 - 4.0 + 2.0 * 3.0));
		PR_EXPECT(EXPR((1.0 + 2.0) * 3.0 - 4.0));
		PR_EXPECT(EXPR(1.0 + 2.0 * -(3.0 - 4.0)));
		PR_EXPECT(EXPR(1.0 + (2.0 * (3.0 - 4.0))));
		PR_EXPECT(EXPR((1.0 + 2.0) * (3.0 - 4.0)));
		PR_EXPECT(EXPR(~37 & ~0));
		PR_EXPECT(EXPR(!37 | !0));
		PR_EXPECT(EXPR(~(0xFFFFFFFF >> 2)));
		PR_EXPECT(EXPR(~(4294967295 >> 2)));
		PR_EXPECT(EXPR(~(0xFFFFFFFFLL >> 2)));
		PR_EXPECT(EXPR(~(4294967295LL >> 2)));
		PR_EXPECT(Expr("sin(1.0 + 2.0)", std::sin(1.0 + 2.0)));
		PR_EXPECT(Expr("cos(TAU)", std::cos(eval::TAU)));
		PR_EXPECT(Expr("tan(PHI)", std::tan(eval::PHI)));
		PR_EXPECT(Expr("abs( 1.0)", std::abs(1.0)));
		PR_EXPECT(Expr("abs(-1.0)", std::abs(-1.0)));
		PR_EXPECT(EXPR(11 % 3));
		PR_EXPECT(Expr("11.3 % 3.1", std::fmod(11.3, 3.1)));
		PR_EXPECT(Expr("3.0 * (17.3 % 2.1)", 3.0 * std::fmod(17.3, 2.1)));
		PR_EXPECT(EXPR(1 << 10));
		PR_EXPECT(EXPR(1024 >> 3));
		PR_EXPECT(Expr("ceil(3.4)", std::ceil(3.4)));
		PR_EXPECT(Expr("ceil(-3.4)", std::ceil(-3.4)));
		PR_EXPECT(Expr("floor(3.4)", std::floor(3.4)));
		PR_EXPECT(Expr("floor(-3.4)", std::floor(-3.4)));
		PR_EXPECT(Expr("round( 3.5)", std::round(3.5)));
		PR_EXPECT(Expr("round(-3.5)", std::round(-3.5)));
		PR_EXPECT(Expr("round( 3.2)", std::round(3.2)));
		PR_EXPECT(Expr("round(-3.2)", std::round(-3.2)));
		PR_EXPECT(Expr("asin(-0.8)", std::asin(-0.8)));
		PR_EXPECT(Expr("acos(0.2)", std::acos(0.2)));
		PR_EXPECT(Expr("atan(2.3/12.9)", std::atan(2.3 / 12.9)));
		PR_EXPECT(Expr("atan2(2.3,-3.9)", std::atan2(2.3, -3.9)));
		PR_EXPECT(Expr("sinh(0.8)", std::sinh(0.8)));
		PR_EXPECT(Expr("cosh(0.2)", std::cosh(0.2)));
		PR_EXPECT(Expr("tanh(2.3)", std::tanh(2.3)));
		PR_EXPECT(Expr("exp(2.3)", std::exp(2.3)));
		PR_EXPECT(Expr("log(209.3)", std::log(209.3)));
		PR_EXPECT(Expr("log10(209.3)", std::log10(209.3)));
		PR_EXPECT(Expr("pow(2.3, -1.3)", std::pow(2.3, -1.3)));
		PR_EXPECT(Expr("sqrt(2.3)", std::sqrt(2.3)));
		PR_EXPECT(Expr("sqr(-2.3)", pr::Sqr(-2.3)));
		PR_EXPECT(Expr("len2(3,4)", std::sqrt(3.0 * 3.0 + 4.0 * 4.0)));
		PR_EXPECT(Expr("len3(3,4,5)", std::sqrt(3.0 * 3.0 + 4.0 * 4.0 + 5.0 * 5.0)));
		PR_EXPECT(Expr("len4(3,4,5,6)", std::sqrt(3.0 * 3.0 + 4.0 * 4.0 + 5.0 * 5.0 + 6.0 * 6.0)));
		PR_EXPECT(Expr("deg(-1.24)", -1.24 * (360.0 / eval::TAU)));
		PR_EXPECT(Expr("rad(241.32)", 241.32 * (eval::TAU / 360.0)));
		PR_EXPECT(Expr("min(-3.2, -3.4)", std::min(-3.2, -3.4)));
		PR_EXPECT(Expr("max(-3.2, -3.4)", std::max(-3.2, -3.4)));
		PR_EXPECT(Expr("clamp(10.0, -3.4, -3.2)", pr::Clamp(10.0, -3.4, -3.2)));
		PR_EXPECT(Expr("hash(\"A String\")", hash::HashCT("A String")));
		PR_EXPECT(Expr("sqr(sqrt(2.3)*-abs(4%2)/15.0-tan(TAU/-6))", pr::Sqr(std::sqrt(2.3) * -std::abs(4 % 2) / 15.0 - std::tan(eval::TAU / -6))));
		{
			long long v1 = 0, v0 = 123456789000000LL / 2;
			PR_EXPECT(Evaluate("123456789000000 / 2", v1));
			PR_EXPECT(v0 == v1);
		}
		PR_EXPECT(Expr("1 != 2 ? 5 : 6", 5));
		PR_EXPECT(Expr("1 == 2 ? 5 : 6", 6));
		PR_EXPECT(Expr("true ? 5 : 6 + 1", 5));
		PR_EXPECT(Expr("false ? 5 : 6 + 1", 7));
		PR_EXPECT(Expr("sqr(-2) ? (1+2) : max(-2,-3)", 3));
		PR_EXPECT(Expr("-+1", -1));
		PR_EXPECT(Expr("-++-1", 1));
		PR_EXPECT(Expr("!!true", 1));
		PR_EXPECT(Expr("-!!!false", -1));
		PR_EXPECT(Expr("10 - 3 - 2", 5));
		PR_EXPECT(Expr("'1' + '2'", '1' + '2'));
		#undef EXPR
	}
	#endif
