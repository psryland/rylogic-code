//********************************
// PR3D Model file format
//  Copyright (c) Rylogic Ltd 2014
//********************************
// Index buffer for indices with runtime stride width
#pragma once
#include <cstdint>
#include <cassert>
#include "pr/common/cast.h"
#include "pr/container/byte_data.h"

namespace pr::geometry
{
	// Notes:
	//  - This buffer type allows indices to be treated as any integral type with static casting to the underlying
	//    runtime index type. Use it to write 32-bit indices into a buffer of 16-bit indices (for e.g.)
	//  - Use 'buf.data<uint16_t>()' if you know the stride is sizeof(uint16_t), otherwise use `begin<int>()' to
	//    get an iterator that you can write ints to.
	//  - Converting from a runtime stride to a compile-time type is a PITA. There will always be some indirection.
	//    Having a conditional before every index access sounds expensive, but the alternative is a function pointer
	//    which cannot be inlined. The best case is for the caller to switch on 'm_stride' and have separate loops for
	//    each possible stride size but this is a burden on the caller. I think the next best option is to switch inside
	//    loops and rely on the optimiser to move the conditional outside the loops in user code.

	namespace index_buffer
	{
		// Pointer type for accessing the indices
		template <bool Const>
		union ptr_t
		{
			using byte_t = std::conditional_t<Const, std::byte const, std::byte>;

			byte_t* p;
			std::conditional_t<Const, uint64_t const*, uint64_t*> u64;
			std::conditional_t<Const, uint32_t const*, uint32_t*> u32;
			std::conditional_t<Const, uint16_t const*, uint16_t*> u16;
			std::conditional_t<Const, uint8_t const*, uint8_t*> u08;

			template <typename Idx> Idx get(int stride) const
			{
				switch (stride)
				{
					case 8: return s_cast<Idx>(*u64);
					case 4: return s_cast<Idx>(*u32);
					case 2: return s_cast<Idx>(*u16);
					case 1: return s_cast<Idx>(*u08);
					default: throw std::runtime_error("Unsupported underlying data format");
				}
			}
			template <typename Idx> void set(int stride, Idx value)
			{
				switch (stride)
				{
					case 8: *u64 = s_cast<uint64_t>(value); break;
					case 4: *u32 = s_cast<uint32_t>(value); break;
					case 2: *u16 = s_cast<uint16_t>(value); break;
					case 1: *u08 = s_cast<uint8_t>(value); break;
					default: throw std::runtime_error("Unsupported underlying data format");
				}
			}
			operator ptr_t<true>() const
			{
				return ptr_t<true>{p};
			}
		};

		// Assignment proxy
		template <std::integral Idx>
		struct proxy_t
		{
			ptr_t<false> m_ptr;
			int m_stride;

			proxy_t& operator = (Idx v)
			{
				m_ptr.set<Idx>(m_stride, v);
				return *this;
			}
			operator Idx() const
			{
				return m_ptr.get<Idx>(m_stride);
			}

			friend void swap(proxy_t&& lhs, proxy_t&& rhs)
			{
				// Don't allow swap for different strides, use manual assignment instead.
				auto tmp = lhs.m_ptr.get<Idx>(lhs.m_stride);
				lhs.m_ptr.set(lhs.m_stride, rhs.m_ptr.get<Idx>(rhs.m_stride));
				rhs.m_ptr.set(rhs.m_stride, tmp);
			}
			friend bool operator == (proxy_t lhs, proxy_t rhs)
			{
				return lhs.m_ptr.get<int64_t>(lhs.m_stride) == rhs.m_ptr.get<int64_t>(rhs.m_stride);
			}
			friend bool operator != (proxy_t lhs, proxy_t rhs)
			{
				return !(lhs == rhs);
			}
		};

		// Index iterator
		template <std::integral Idx>
		struct iter_t
		{
			static constexpr bool Const = std::is_const_v<Idx>;
			using byte_t = std::conditional_t<Const, std::byte const, std::byte>;
			using proxy_t = proxy_t<Idx>;
			using ptr_t = ptr_t<Const>;
	
			using iterator_concept  = std::contiguous_iterator_tag;
			using iterator_category = std::random_access_iterator_tag;
			using value_type        = std::remove_cv_t<Idx>;
			using difference_type   = ptrdiff_t;
			using pointer           = Idx*;
			using reference         = Idx&;

			ptr_t m_ptr;
			int m_stride;

			iter_t()
				: m_ptr()
				, m_stride()
			{
			}
			iter_t(byte_t* p, int stride)
				: m_ptr(p)
				, m_stride(stride)
			{
			}
			Idx operator*() const
			{
				return m_ptr.get<Idx>(m_stride);
			}
			auto operator*() requires (!Const)
			{
				return proxy_t{ m_ptr, m_stride };
			}
			Idx operator->() const
			{
				return m_ptr.get<Idx>(m_stride);
			}
			auto operator->() requires (!Const)
			{
				return proxy_t{ m_ptr, m_stride };
			}
			iter_t& operator ++() noexcept
			{
				m_ptr.u08 += m_stride;
				return *this;
			}
			iter_t& operator --() noexcept
			{
				m_ptr.u08 -= m_stride;
				return *this;
			}
			iter_t operator ++(int) noexcept
			{
				auto iter = *this;
				return ++*this, iter;
			}
			iter_t operator --(int) noexcept
			{
				auto iter = *this;
				return --*this, iter;
			}
			iter_t& operator +=(difference_type ofs)
			{
				m_ptr.u08 += ofs * m_stride;
				return *this;
			}
			iter_t& operator -=(difference_type ofs)
			{
				m_ptr.u08 -= ofs * m_stride;
				return *this;
			}
			operator iter_t<Idx const>() const requires (!Const)
			{
				return iter_t<Idx const>(m_ptr.p, m_stride);
			}

			friend bool operator == (iter_t const& lhs, iter_t const& rhs)
			{
				return lhs.m_ptr.p == rhs.m_ptr.p;
			}
			friend bool operator != (iter_t const& lhs, iter_t const& rhs)
			{
				return !(lhs == rhs);
			}
			friend bool operator < (iter_t const& lhs, iter_t const& rhs)
			{
				return lhs.m_ptr.p < rhs.m_ptr.p;
			}
			friend iter_t operator + (iter_t const& lhs, difference_type rhs)
			{
				auto iter = lhs;
				return iter += rhs;
			}
			friend iter_t operator - (iter_t const& lhs, difference_type rhs)
			{
				auto iter = lhs;
				return iter -= rhs;
			}
			friend difference_type operator - (iter_t const& lhs, iter_t const& rhs)
			{
				assert(lhs.m_stride == rhs.m_stride);
				return (lhs.m_ptr.u08 - rhs.m_ptr.u08) / lhs.m_stride;
			}
		};

		// View of a type agnostic index buffer
		template <bool Const>
		struct index_span
		{
			using cptr_t = ptr_t<true>;
			using mptr_t = ptr_t<false>;
			using byte_t = std::conditional_t<Const, std::byte const, std::byte>;

		private:

			// Span of the index data
			std::span<byte_t> m_span;

			// Index stride. n = bytes per index
			int m_stride;

		public:

			index_span()
				: m_span()
				, m_stride(0)
			{
			}
			index_span(std::span<byte_t> span, int stride)
				: m_span(span)
				, m_stride(stride)
			{
			}
			template <std::integral Idx> index_span(std::span<Idx const> span)
				: m_span(byte_span(span))
				, m_stride(sizeof(Idx))
			{
			}
			template <std::integral Idx> index_span(std::initializer_list<Idx const> indices)
				: index_span(std::span<Idx const>{ indices.begin(), indices.size() })
			{}

			// True if the span is empty
			bool empty() const
			{
				return m_span.empty();
			}

			// The number of indices in this buffer
			size_t size() const
			{
				assert(m_stride != 0 && isize(m_span) % m_stride == 0);
				return s_cast<size_t>(m_span.size() / m_stride);
			}
			size_t size_bytes() const
			{
				return m_span.size();
			}

			// The width of each index in bytes
			int stride() const
			{
				return m_stride;
			}

			// Access the raw pointer. sizeof(Idx) must == stride()
			template <std::integral Idx> Idx const* data() const
			{
				if (sizeof(Idx) != m_stride) throw std::runtime_error("Index data size mismatch");
				return type_ptr<Idx const>(m_span.data());
			}
			template <std::integral Idx> Idx* data() requires (!Const)
			{
				// If you want to write indices using a type other than the underlying index type, use begin<Idx>().
				if (sizeof(Idx) != m_stride) throw std::runtime_error("Index data size mismatch");
				return type_ptr<Idx>(m_span.data());
			}

			// Iteration - interpret the index buffer as Idx regardless of the stride of contained data.
			template <std::integral Idx> iter_t<Idx const> begin() const
			{
				return iter_t<Idx const>(m_span.data(), m_stride);
			}
			template <std::integral Idx> iter_t<Idx const> cbegin()
			{
				return std::as_const(*this).begin<Idx>();
			}
			template <std::integral Idx> iter_t<Idx> begin() requires (!Const)
			{
				return iter_t<Idx>(m_span.data(), m_stride);
			}
			template <std::integral Idx> iter_t<Idx const> end() const
			{
				// The buffer may contain padding...
				auto size_in_bytes = static_cast<size_t>(size() * m_stride);
				return iter_t<Idx const>{ m_span.data() + size_in_bytes, m_stride };
			}
			template <std::integral Idx> iter_t<Idx const> cend()
			{
				return std::as_const(*this).end<Idx>();
			}
			template <std::integral Idx> iter_t<Idx> end() requires (!Const)
			{
				// The buffer may contain padding...
				auto size_in_bytes = static_cast<size_t>(size() * m_stride);
				return iter_t<Idx>{ m_span.data() + size_in_bytes, m_stride };
			}

			// Access the raw data. sizeof(Idx) must == stride()
			template <std::integral Idx> std::span<Idx const> span() const
			{
				if (sizeof(Idx) != stride()) throw std::runtime_error("Index data size mismatch");
				return type_span<Idx const>(m_span);
			}
			template <std::integral Idx> std::span<Idx> span() requires (!Const)
			{
				if (sizeof(Idx) != stride()) throw std::runtime_error("Index data size mismatch");
				return type_span<Idx>(m_span);
			}
			std::span<std::byte const> span() const
			{
				return m_span;
			}
			std::span<std::byte> span() requires (!Const)
			{
				return m_span;
			}

			// Ranged-for helper for looping over indices as type 'Idx'
			template <std::integral Idx> auto span_as() const
			{
				struct casting_span_t
				{
					index_span const* m_this;
					iter_t<Idx const> begin() const { return m_this->begin<Idx>(); }
					iter_t<Idx const> end() const { return m_this->end<Idx>(); }
				};
				return casting_span_t{ this };
			}

			// Implicit cast to byte span
			operator std::span<std::byte const>() const
			{
				return m_span;
			}
			operator std::span<std::byte>() requires (!Const)
			{
				return m_span;
			}
		};
	}

	// Type agnostic index spans
	using index_cspan = index_buffer::index_span<true>;
	using index_span = index_buffer::index_span<false>;

	// Buffer of type agnostic index data
	struct IdxBuf
	{
		using cptr_t = index_buffer::ptr_t<true>;
		using mptr_t = index_buffer::ptr_t<false>;
		template <typename T> using iter_t = index_buffer::iter_t<T>;

	private:

		// Index buffer
		pr::byte_data<alignof(uint64_t)> m_buf;

		// Index stride. n = bytes per index
		int m_stride;

	public:

		// Constructor
		IdxBuf()
			: m_buf()
			, m_stride(1)
		{
		}
		explicit IdxBuf(int stride)
			: m_buf()
			, m_stride(std::max(stride, 1))
		{
			assert(stride > 0 && "Stride must be >= 1");
		}
		explicit IdxBuf(std::span<int const> indices)
			: m_buf(indices)
			, m_stride(sizeof(int))
		{
		}
		// Don't provide this. It's ambiguous with `IdxBuf(stride)`
		// explicit IdxBuf(std::initializer_list<int const> indices) {}
		IdxBuf(IdxBuf&& rhs) noexcept
			: m_buf(std::move(rhs.m_buf))
			, m_stride(rhs.m_stride)
		{
			rhs.m_stride = 1;
		}
		IdxBuf(IdxBuf const& rhs)
			: IdxBuf(rhs.stride())
		{
			append(rhs);
		}
		IdxBuf& operator = (IdxBuf&& rhs) noexcept
		{
			if (&rhs == this) return *this;
			m_buf = std::move(rhs.m_buf);
			m_stride = rhs.m_stride;
			rhs.m_stride = 1;
			return *this;
		}
		IdxBuf& operator = (IdxBuf const& rhs)
		{
			if (&rhs == this) return *this;
			return *this = static_cast<index_cspan>(rhs);
		}
		IdxBuf& operator = (index_cspan rhs)
		{
			resize(0, stride());
			return append(rhs);
		}

		// Index accessor
		uint64_t operator [] (size_t i) const
		{
			assert(i < size());
			auto p = cptr_t{ .p = m_buf.data() + i * stride() };
			return p.get<uint64_t const>(stride());
		}
		auto operator [] (size_t i)
		{
			assert(i < size());
			auto p = mptr_t{ .p = m_buf.data() + i * stride() };
			return index_buffer::proxy_t<uint64_t>{p, stride()};
		}

		// Access the raw pointer. sizeof(Idx) must == stride()
		template <std::integral Idx> Idx const* data() const
		{
			if (sizeof(Idx) != stride()) throw std::runtime_error("Index data size mismatch");
			return m_buf.data<Idx const>();
		}
		template <std::integral Idx> Idx* data()
		{
			// If you want to write indices using a type other than the underlying index type, use begin<Idx>().
			if (sizeof(Idx) != stride()) throw std::runtime_error("Index data size mismatch");
			return m_buf.data<Idx>();
		}

		// True if the buffer is empty
		bool empty() const
		{
			return m_buf.empty();
		}

		// The number of indices in this buffer
		size_t size() const
		{
			assert(stride() != 0);
			return s_cast<size_t>(m_buf.size() / stride());
		}
		size_t size_bytes() const
		{
			return m_buf.size<std::byte>();
		}

		// The width of each index in bytes
		int stride() const
		{
			return m_stride;
		}

		// The maximum value an index can have
		uint64_t max_value() const
		{
			switch (stride())
			{
				case 8: return std::numeric_limits<uint64_t>::max();
				case 4: return std::numeric_limits<uint32_t>::max();
				case 2: return std::numeric_limits<uint16_t>::max();
				case 1: return std::numeric_limits<uint8_t>::max();
				default: throw std::runtime_error("Unsupported stride value");
			}
		}

		// Resize the index buffer to hold 'count' indices of size 'sizeof(Idx)'
		template <std::integral Idx> void resize(size_t count)
		{
			resize(count, sizeof(Idx));
		}
		void resize(size_t count, int stride)
		{
			assert(stride > 0 && "Stride must be >= 1");
			stride = std::max(stride, 1);

			// Don't make 'stride' defaulted to stride(). It's too easily to confuse with resize in bytes
			auto remaining = std::min(count, size());
			if (remaining == 0 || stride == m_stride)
			{
				m_buf.resize<std::byte>(count * stride, {});
			}
			else if (stride < m_stride)
			{
				cptr_t s = { .p = m_buf.data() };
				mptr_t d = { .p = m_buf.data() };
				for (auto c = remaining; c-- != 0;)
				{
					d.set(stride, s.get<uint64_t>(m_stride));
					s.u08 += m_stride;
					d.u08 += stride;
				}

				m_buf.resize<std::byte>(count * stride, {});
			}
			else if (stride > m_stride)
			{
				m_buf.resize<std::byte>(count * stride, {});

				cptr_t s = { .p = m_buf.data() + remaining * m_stride };
				mptr_t d = { .p = m_buf.data() + remaining * stride };
				for (auto c = remaining; c-- != 0;)
				{
					d.u08 -= stride;
					s.u08 -= m_stride;
					d.set(stride, s.get<uint64_t>(m_stride));
				}
			}
			m_stride = stride;
		}

		// Reserve memory for indices
		template <std::integral Idx> void reserve(size_t count)
		{
			reserve(count, sizeof(Idx));
		}
		void reserve(size_t count, int stride)
		{
			// Don't make 'stride' defaulted to stride(). It's too easily to confuse with reserve in bytes
			m_buf.reserve(count * stride);
		}

		// Capacity for indices
		size_t capacity() const
		{
			return m_buf.capacity() / stride();
		}

		// Push an index into the buffer
		template <std::integral Idx>
		void push_back(Idx idx)
		{
			if (idx < 0 || s_cast<uint64_t>(idx) > max_value())
				throw std::runtime_error("Index value out of range for this stride size");

			switch (stride())
			{
				case 8: m_buf.push_back(s_cast<uint64_t>(idx)); break;
				case 4: m_buf.push_back(s_cast<uint32_t>(idx)); break;
				case 2: m_buf.push_back(s_cast<uint16_t>(idx)); break;
				case 1: m_buf.push_back(s_cast<uint8_t>(idx)); break;
				default: throw std::runtime_error("Unsupported index stride");
			}
		}

		// Push a special '-1' index into the buffer
		void push_back_strip_cut()
		{
			switch (stride())
			{
				case 8: m_buf.push_back<uint64_t>(0xFFFFFFFFFFFFFFFF); break;
				case 4: m_buf.push_back<uint32_t>(0xFFFFFFFF); break;
				case 2: m_buf.push_back<uint16_t>(0xFFFF); break;
				case 1: m_buf.push_back<uint8_t >(0xFF); break;
				default: throw std::runtime_error("Unsupported index stride");
			}
		}

		// Write a strip-cut (primitive restart) sentinel into an already-sized buffer at index position 'i'.
		void set_strip_cut(size_t i)
		{
			assert(i < size());
			auto p = mptr_t{ .p = m_buf.data() + i * stride() };
			switch (stride())
			{
				case 8: *p.u64 = 0xFFFFFFFFFFFFFFFF; break;
				case 4: *p.u32 = 0xFFFFFFFF; break;
				case 2: *p.u16 = 0xFFFF; break;
				case 1: *p.u08 = 0xFF; break;
				default: throw std::runtime_error("Unsupported index stride");
			}
		}

		// Append indices
		IdxBuf& append(index_cspan rhs)
		{
			if (stride() == rhs.stride())
			{
				m_buf.append(rhs.span());
			}
			else
			{
				switch (stride())
				{
					case 8:
					{
						for (auto idx : rhs.span_as<uint64_t const>())
							m_buf.push_back(idx);
						break;
					}
					case 4:
					{
						for (auto idx : rhs.span_as<uint32_t const>())
							m_buf.push_back(idx);
						break;
					}
					case 2:
					{
						for (auto idx : rhs.span_as<uint16_t const>())
							m_buf.push_back(idx);
						break;
					}
					case 1:
					{
						for (auto idx : rhs.span_as<uint8_t const>())
							m_buf.push_back(idx);
						break;
					}
					default:
					{
						throw std::runtime_error("Unsupported index stride");
					}
				}
			}

			return *this;
		}
		template <std::integral Idx> IdxBuf& append(std::span<Idx const> data)
		{
			return append(index_cspan(data));
		}
		template <std::integral Idx> IdxBuf& append(std::initializer_list<Idx const> data)
		{
			return append<Idx>(std::span<Idx const>{ data.begin(), data.size() });
		}

		// Iteration - interpret the index buffer as Idx regardless of the stride of contained data.
		template <std::integral Idx> iter_t<Idx const> begin() const
		{
			return iter_t<Idx const>{ m_buf.data(), stride() };
		}
		template <std::integral Idx> iter_t<Idx const> cbegin()
		{
			return std::as_const(*this).begin<Idx>();
		}
		template <std::integral Idx> iter_t<Idx> begin()
		{
			return iter_t<Idx>{ m_buf.data(), stride() };
		}
		template <std::integral Idx> iter_t<Idx const> end() const
		{
			// The buffer may contain padding...
			auto size_in_bytes = static_cast<size_t>(size() * stride());
			return iter_t<Idx const>{ m_buf.data() + size_in_bytes, stride() };
		}
		template <std::integral Idx> iter_t<Idx const> cend()
		{
			return std::as_const(*this).end<Idx>();
		}
		template <std::integral Idx> iter_t<Idx> end()
		{
			// The buffer may contain padding...
			auto size_in_bytes = static_cast<size_t>(size() * stride());
			return iter_t<Idx>{ m_buf.data() + size_in_bytes, stride() };
		}

		// Access the raw data. sizeof(Idx) must == stride()
		template <std::integral Idx> std::span<Idx const> span() const
		{
			if (sizeof(Idx) != stride()) throw std::runtime_error("Index data size mismatch");
			return m_buf.span<Idx>();
		}
		template <std::integral Idx> std::span<Idx> span()
		{
			if (sizeof(Idx) != stride()) throw std::runtime_error("Index data size mismatch");
			return m_buf.span<Idx>();
		}

		// Ranged-for helper for looping over indices as type 'Idx'
		template <std::integral Idx> auto span_as() const
		{
			struct casting_span_t
			{
				IdxBuf const* m_this;
				iter_t<Idx const> begin() const { return m_this->begin<Idx>(); }
				iter_t<Idx const> end() const { return m_this->end<Idx>(); }
			};
			return casting_span_t{this};
		}

		// Implicit cast to index span
		operator index_cspan() const
		{
			return { m_buf.span(), stride() };
		}
		operator index_span()
		{
			return { m_buf.span(), stride() };
		}

		// Implicit cast to byte span
		operator std::span<std::byte const>() const
		{
			return m_buf.span();
		}
		operator std::span<std::byte>()
		{
			return m_buf.span();
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::geometry::unittests
{
	PRUnitTest(IdxBufTest, Quick, uint64_t, uint32_t, uint16_t, uint8_t, int8_t, int16_t, int32_t, int64_t)
	{
		IdxBuf ibuf0(sizeof(T));
		PR_EXPECT(ibuf0.empty());
		PR_EXPECT(ibuf0.size() == 0);
		PR_EXPECT(ibuf0.stride() == sizeof(T));
		PR_EXPECT(ibuf0.max_value() == std::numeric_limits<std::make_unsigned_t<T>>::max());

		IdxBuf ibuf1(sizeof(int));
		ibuf1 = { 0, 1, 2, 3, 4, 5, 6 };
		PR_EXPECT(!ibuf1.empty());
		PR_EXPECT(ibuf1.size() == 7);
		PR_EXPECT(ibuf1.stride() == sizeof(int));

		// Casting copy
		ibuf0 = ibuf1;
		PR_EXPECT(ibuf0.size() == 7);
		PR_EXPECT(ibuf1.size() == 7);
		PR_EXPECT(ibuf0.stride() == sizeof(T));
		PR_EXPECT(ibuf1.stride() == sizeof(int));

		// Move
		ibuf0.resize(0, ibuf0.stride());
		PR_EXPECT(ibuf0.empty());
		PR_EXPECT(!ibuf1.empty());
		ibuf0 = std::move(ibuf1);
		PR_EXPECT(!ibuf0.empty());
		PR_EXPECT(ibuf0.size() == 7);
		PR_EXPECT(ibuf0.stride() == sizeof(int));

		ibuf0.resize<T>(ibuf0.size());
		PR_EXPECT(ibuf0.stride() == sizeof(T));

		// Initializser assignment
		ibuf0 = { 1,2,3,4 };
		PR_EXPECT(ibuf0.size() == 4);
		PR_EXPECT(ibuf0.stride() == sizeof(T));

		// Index accessor
		PR_EXPECT(ibuf0[3] == 4);
		PR_EXPECT(sizeof(ibuf0[3]) == sizeof(index_buffer::proxy_t<T>));

		// Access the raw pointer. sizeof(Idx) must == stride()
		auto const* data = ibuf0.data<T>();
		PR_EXPECT(sizeof(data[0]) == sizeof(T));
		PR_EXPECT(data[0] == 1);
		PR_EXPECT(data[1] == 2);
		PR_EXPECT(data[2] == 3);
		PR_EXPECT(data[3] == 4);

		// Resize the index buffer to hold 'count' indices of size 'sizeof(Idx)'
		ibuf0.resize(2, 1);
		PR_EXPECT(ibuf0.size() == 2);
		PR_EXPECT(ibuf0.stride() == 1);
		PR_EXPECT(ibuf0.size_bytes() == 2);
		PR_EXPECT(ibuf0[0] == 1);
		PR_EXPECT(ibuf0[1] == 2);
		ibuf0.resize<T>(4);
		PR_EXPECT(ibuf0.size() == 4);
		PR_EXPECT(ibuf0.stride() == sizeof(T));
		PR_EXPECT(ibuf0.size_bytes() == 4 * sizeof(T));
		PR_EXPECT(ibuf0[0] == 1);
		PR_EXPECT(ibuf0[1] == 2);
		PR_EXPECT(ibuf0[2] == 0);
		PR_EXPECT(ibuf0[3] == 0);

		// Reserve memory for indices
		ibuf0.reserve<T>(10);
		PR_EXPECT(ibuf0.capacity() >= 10);

		// Push an index into the buffer
		ibuf0.push_back<uint8_t >(4);
		ibuf0.push_back<uint16_t>(5);
		ibuf0.push_back<uint32_t>(6);
		ibuf0.push_back<uint64_t>(7);
		ibuf0.push_back<int8_t  >(8);
		ibuf0.push_back<int16_t >(9);
		ibuf0.push_back<int32_t >(10);
		ibuf0.push_back<int64_t >(11);
		ibuf0.push_back_strip_cut();
		PR_EXPECT(ibuf0.size() == 13);
		PR_EXPECT(ibuf0.stride() == sizeof(T));
		PR_EXPECT(ibuf0.size_bytes() == 13 * sizeof(T));
		PR_EXPECT(ibuf0[4] == 4);
		PR_EXPECT(ibuf0[5] == 5);
		PR_EXPECT(ibuf0[6] == 6);
		PR_EXPECT(ibuf0[7] == 7);
		PR_EXPECT(ibuf0[8] == 8);
		PR_EXPECT(ibuf0[9] == 9);
		PR_EXPECT(ibuf0[10] == 10);
		PR_EXPECT(ibuf0[11] == 11);
		PR_EXPECT(ibuf0[12] == std::make_unsigned_t<T>(-1));

		// Indexer assignment
		ibuf0[0] = 9;
		PR_EXPECT(ibuf0[0] == 9);
		ibuf0.resize<T>(1);
		PR_EXPECT(ibuf0.size() == 1);
		PR_EXPECT(ibuf0[0] == 9);

		// Append indices
		ibuf0[0] = 0;
		ibuf0.append<int>({ 1, 2, 3, 4, 5 });
		PR_EXPECT(ibuf0.size() == 6);
		PR_EXPECT(ibuf0[0] == 0);
		PR_EXPECT(ibuf0[1] == 1);
		PR_EXPECT(ibuf0[2] == 2);
		PR_EXPECT(ibuf0[3] == 3);
		PR_EXPECT(ibuf0[4] == 4);
		PR_EXPECT(ibuf0[5] == 5);

		// Iteration
		int k = 0;
		for (auto i = ibuf0.begin<int>(); i != ibuf0.end<int>(); ++i)
		{
			PR_EXPECT(*i == k++);
			PR_EXPECT(sizeof(*i) == sizeof(index_buffer::proxy_t<T>)); // can't avoid this
		}

		k = 0;
		for (auto i = ibuf0.cbegin<int>(); i != ibuf0.cend<int>(); ++i)
		{
			PR_EXPECT(*i == k++);
			PR_EXPECT(sizeof(*i) == sizeof(int));
		}

		k = 0;
		for (auto i = std::as_const(ibuf0).begin<int>(); i != std::as_const(ibuf0).end<int>(); ++i)
		{
			PR_EXPECT(*i == k++);
			PR_EXPECT(sizeof(*i) == sizeof(int));
		}

		k = 0;
		for (auto i : ibuf0.span<T>())
		{
			PR_EXPECT(i == T(k++));
			PR_EXPECT(sizeof(i) == sizeof(T));
		}

		k = 0;
		for (auto i : ibuf0.span_as<short>())
		{
			PR_EXPECT(i == (short)k++);
			PR_EXPECT(sizeof(i) == sizeof(short));
		}

		// Implicit cast to index span
		auto idx_span = static_cast<index_cspan>(ibuf0);
		PR_EXPECT(idx_span.size() == ibuf0.size());
		PR_EXPECT(idx_span.stride() == ibuf0.stride());

		// Implicit cast to byte span
		auto byte_span = static_cast<std::span<std::byte const>>(ibuf0);
		PR_EXPECT(byte_span.size() == ibuf0.size() * sizeof(T));
	}
	PRUnitTest(IdxSpanTest, Quick, uint8_t, uint16_t, uint32_t, int8_t, int16_t, int32_t, int64_t)
	{
		IdxBuf src(sizeof(T));
		src.append<T>({ 0, 1, 2, 3, 4 });

		index_span ispan = src;
		PR_EXPECT(!ispan.empty());
		PR_EXPECT(ispan.size() == 5);
		PR_EXPECT(ispan.size_bytes() == 5 * sizeof(T));
		PR_EXPECT(ispan.stride() == sizeof(T));

		// Access the raw pointer. sizeof(Idx) must == stride()
		auto const* data = ispan.data<T>();
		PR_EXPECT(sizeof(data[0]) == sizeof(T));
		PR_EXPECT(data[0] == 0);
		PR_EXPECT(data[1] == 1);
		PR_EXPECT(data[2] == 2);
		PR_EXPECT(data[3] == 3);
		PR_EXPECT(data[4] == 4);

		// Iteration
		int k = 0;
		for (auto i = ispan.begin<int>(); i != ispan.end<int>(); ++i)
		{
			PR_EXPECT(*i == k++);
			PR_EXPECT(sizeof(*i) == sizeof(index_buffer::proxy_t<T>)); // can't avoid this
		}

		k = 0;
		for (auto i = ispan.cbegin<int>(); i != ispan.cend<int>(); ++i)
		{
			PR_EXPECT(*i == k++);
			PR_EXPECT(sizeof(*i) == sizeof(int));
		}

		k = 0;
		for (auto i : ispan.span<T>())
		{
			PR_EXPECT(i == T(k++));
			PR_EXPECT(sizeof(i) == sizeof(T));
		}

		k = 0;
		for (auto i : ispan.span_as<short>())
		{
			PR_EXPECT(i == (short)k++);
			PR_EXPECT(sizeof(i) == sizeof(short));
		}

		// Implicit cast to byte span
		auto byte_span = static_cast<std::span<std::byte const>>(ispan);
		PR_EXPECT(byte_span.size() == ispan.size() * sizeof(T));
	}
}
#endif
