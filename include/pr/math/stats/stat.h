//*********************************************************************
// Stat
//  Copyright (c) Rylogic Ltd 2008
//*********************************************************************
#pragma once
#include <cmath>
#include <cfloat>
#include <cassert>
#include <vector>
#include <functional>
#include <concepts>
#include <limits>
#include <utility>

namespace pr::math
{
	// Components
	namespace stats
	{
		template <typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
		struct Count
		{
			int m_count;

			void Reset() noexcept
			{
				m_count = 0;
			}
			void Add(Type) noexcept
			{
				++m_count;
			}
		};

		template <typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
		struct MinMax
		{
			Type m_min;
			Type m_max;
			
			void Reset() noexcept
			{
				m_min = (std::numeric_limits<Type>::max)();
				m_max = std::numeric_limits<Type>::lowest();
			}	
			void Add(Type value) noexcept
			{
				if (value < m_min) m_min = value;
				if (value > m_max) m_max = value;
			}
		};

		template <typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
		struct Mean
		{
			Type m_mean;

			void Reset() noexcept
			{
				m_mean = {};
			}	
			void Add(Type value, int count) noexcept
			{
				auto diff = value - m_mean;
				m_mean += diff * static_cast<Scalar>(1.0 / count);
			}
		};

		template <typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
		struct Variance
		{
			Type m_var;
			
			// Use the population standard deviation when all data values in a set have been considered.
			// Use the sample standard deviation when the data values used are only a sample of the total population
			Type PopStdDev(int count) const noexcept
			{
				return static_cast<Type>(sqrt(PopStdVar(count)));
			}
			Type SamStdDev(int count) const noexcept
			{
				return static_cast<Type>(sqrt(SamStdVar(count)));
			}
			Type PopStdVar(int count) const noexcept
			{
				return static_cast<Type>(m_var * (1.0 / (count + (count == 0))));
			}
			Type SamStdVar(int count) const noexcept
			{
				return static_cast<Type>(m_var * (1.0 / (count - (count != 1))));
			}

			void Reset() noexcept
			{
				m_var = {};
			}
			void Add(Type value, Type mean, int count) noexcept
			{
				auto diff = value - mean;
				m_var  += diff * diff * ((count - 1) * static_cast<Scalar>(1.0 / count));
			}
		};

		template <typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
		struct EMA
		{
			//  avr(k) = a * X(k) + (1 - a) * avr(k-1)
			//         = a * X(k) + avr(k-1) - a * avr(k-1)
			//         = avr(k-1) + a * X(k) - a * avr(k-1)
			//         = avr(k-1) + a * (X(k) - avr(k-1))
			//    'a' is the exponential smoothing factor between (0,1)
			//    define: a = 2 / (N + 1), where 'N' is roughly the window size of an equivalent moving window average
			//    The interval over which the weights decrease by a factor of two (half-life) is approximately N/2.8854
			// Exponential moving variance:
			// (k-1)var(k) = SUM{w(k) * U(k)²}, where: U(k) = X(k) - avr(k)
			//             = w(1)*U(1)² + w(2)*U(2)² + ... + w(k)*U(k)², where: w(1)+w(2)+...+w(k) = k
			// If we say:  w(k) = k * a, ('a' between (0,1) as above) then SUM{w(k-1)} = k * (1-a)
			// so consider var(k-1):
			//  (k-2)var(k-1) = w(1)*U(1)² + w(2)*U(2)² + ... + w(k-1)*U(k-1)², where: w(1)+w(2)+...+w(k-1) = k - 1
			// when we add the next term:
			//  (k-1)var(k)   = w(1)*U(1)² + w(2)*U(2)² + ... + w(k-1)*U(k-1)² + w(k)*U(k)² (note w(1)..w(k-1) will be different values to above)
			// we need:
			//   k = k*a + k*(1-a) = w(k) + b*SUM{w(k-1)}
			// => k*(1-a) = b*SUM{w(k-1)}
			//          b = (1-a)*k/SUM{w(k-1)}
			//            = (1-a)*k/(k-1)
			// so:
			// (k-1)var(k) = a*k*U(k)² + b*(k-2)var(k-1)
			//             = a*k*U(k)² + (1-a)*(k/(k-1)) * (k-2)var(k-1)
			//  let: D(k) = X(k) - avr(k-1)
			//       U(k) = X(k) - avr(k-1) + avr(k-1) - avr(k)
			//            = D(k) + avr(k-1) - avr(k)
			//            = D(k) - (avr(k) - avr(k-1))
			//            = D(k) - a * (X(k) - avr(k-1))
			//            = D(k) - a * D(k)
			//            = (1-a)*D(k)
			// then:
			// (k-1)var(k) = a*k*U(k)² + (1-a)*(k/(k-1)) * (k-2)var(k-1)
			//             = a*k*(1-a)²*D(k)² + (1-a)*(k/(k-1)) * (k-2)var(k-1)
			//             = a*k*b²*D(k)² + (b*k/(k-1)) * (k-2)var(k-1)         where: b = (1-a)
			//             = (b*k/(k-1))*((a*b*(k-1)*D(k)² + (k-2)var(k-1))
			// 'Type' is typically a floating point type, although this does
			// work for any type that defines the necessary operators.

			int m_size;
			int m_count;
			Type m_mean;
			Type m_var;

			EMA(int window_size) noexcept
				:m_size(window_size)
				,m_count()
				,m_mean()
				,m_var()
			{}

			int WindowSize() const noexcept
			{
				return m_size;
			}
			int Count() const noexcept
			{
				return m_count;
			}
			Type Mean() const noexcept
			{
				return m_mean;
			}

			// Use the population standard deviation when all data values in a set have been considered.
			// Use the sample standard deviation when the data values used are only a sample of the total population
			// Note: for a moving variance the choice between population/sample SD is a bit arbitrary
			Type PopStdDev() const noexcept
			{
				return static_cast<Type>(CompSqrt(PopStdVar()));
			}
			Type SamStdDev() const noexcept
			{
				return static_cast<Type>(CompSqrt(SamStdVar()));
			}
			Type PopStdVar() const noexcept
			{
				return m_var * static_cast<Scalar>(1.0 / (m_count + (m_count == 0)));
			}
			Type SamStdVar() const noexcept
			{
				return m_var * static_cast<Scalar>(1.0 / (m_count - (m_count != 1)));
			}

			void Reset(int window_size) noexcept
			{
				m_size = window_size;
				m_count = 0;
				m_mean = {};
				m_var = {};
			}
			void Add(Type const& value) noexcept
			{
				if (m_count >= m_size)
				{
					++m_count;
					auto diff = value - m_mean;
					auto a = static_cast<Scalar>(2.0 / (m_size + 1.0));
					auto b = static_cast<Scalar>(1.0 - a);
					m_mean = static_cast<Type>(m_mean + diff * a);
					m_var  = static_cast<Type>((b*m_count/(m_count-1)) * (a*b*(m_count-1)*diff*diff + m_var));
				}
				else // use standard mean/var until 'm_size' is reached
				{
					++m_count;
					auto diff = value - m_mean;
					auto inv_count = static_cast<Scalar>(1.0 / m_count);
					m_mean += static_cast<Type>(diff * inv_count);
					m_var  += static_cast<Type>(diff * diff * ((m_count - 1) * inv_count));
				}
			}
		};

		template <typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
		struct SMA
		{
			// Let: D(k) = X(k) - X(k-N) => X(k-N) = X(k) - D(k)
			// Average:
			//   avr(k) = avr(k-1) + (X(k) - X(k-N)) / N
			//          = avr(k-1) + D(k) / N
			// 'Type' is typically a floating point type, although this does
			// work for any type that defines the necessary operators.

			std::vector<Type> m_window;
			int m_count;
			Type m_mean;
			Type* m_in;

			SMA(int window_size) noexcept
				: m_window(window_size)
				, m_count()
				, m_mean()
				, m_in(m_window.data())
			{
				Reset(window_size);
			}

			int Count() const noexcept
			{
				return m_count;
			}
			Type Mean() const noexcept
			{
				return m_mean;
			}

			// NOTE: no recursive variance because we would need to buffer the averages as well
			// so that we could remove (X(k-N) - avr(k-N))^2 at each iteration
			// Use the population standard deviation when all data values in a set have been considered.
			// Use the sample standard deviation when the data values used are only a sample of the total population
			Type PopStdDev() const noexcept
			{
				return static_cast<Type>(sqrt(PopStdVar()));
			}
			Type SamStdDev() const noexcept
			{
				return static_cast<Type>(sqrt(SamStdVar()));
			}
			Type PopStdVar() const noexcept
			{
				return static_cast<Type>(Var() * static_cast<Scalar>(1.0 / (m_count + (m_count == 0))));
			}
			Type SamStdVar() const noexcept
			{
				return static_cast<Type>(Var() * static_cast<Scalar>(1.0 / (m_count - (m_count != 1))));
			}
			Type Var() const noexcept
			{
				auto var = Type();
				auto count = m_count;
				auto const* end_ = m_window.data() + m_window.size();
				for (auto i = m_in; i-- != m_window.data() && count; --count) { auto diff = *i - m_mean; var += diff * diff; }
				for (auto i = end_; i-- != m_window.data() && count; --count) { auto diff = *i - m_mean; var += diff * diff; }
				return var;
			}

			void Reset() noexcept
			{
				Reset(static_cast<int>(m_window.size()));
			}
			void Reset(int window_size) noexcept
			{
				m_window.resize(window_size);
				m_count = 0;
				m_mean = {};
				m_in = m_window.data();
			}
			void Add(Type const& value) noexcept
			{
				if (m_count < static_cast<int>(m_window.size()))
				{
					++m_count;
					auto diff = value - m_mean;
					m_mean += diff * static_cast<Scalar>(1.0 / m_count);
					*m_in++ = value;
				}
				else if (m_count != 0)
				{
					if (m_in == m_window.data() + m_window.size())
						m_in = m_window.data();

					auto diff = value - *m_in;
					m_mean += diff * static_cast<Scalar>(1.0 / m_count);
					*m_in++ = value;
				}
			}
		};
	}

	// Common stats
	template <typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
	struct Stat
	{
		stats::Count<Type, Scalar> m_count;
		stats::MinMax<Type, Scalar> m_minmax;
		stats::Mean<Type, Scalar> m_mean;
		stats::Variance<Type, Scalar> m_variance;

		Stat() noexcept
		{
			Reset();
		}

		int Count() const noexcept
		{
			return m_count.m_count;
		}
		Type Min() const noexcept
		{
			return m_minmax.m_min;
		}
		Type Max() const noexcept
		{
			return m_minmax.m_max;
		}
		Type Mean() const noexcept
		{
			return m_mean.m_mean;
		}
		Type Sum() const noexcept
		{
			return Mean() * Count();
		}

		// Use the population standard deviation when all data values in a set have been considered.
		// Use the sample standard deviation when the data values used are only a sample of the total population
		Type PopStdDev() const noexcept
		{
			return m_variance.PopStdDev(m_count.m_count);
		}
		Type SamStdDev() const noexcept
		{
			return m_variance.SamStdDev(m_count.m_count);
		}
		Type PopStdVar() const noexcept
		{
			return m_variance.PopStdVar(m_count.m_count);
		}
		Type SamStdVar(int count) const noexcept
		{
			return m_variance.SamStdVar(m_count.m_count);
		}

		void Reset() noexcept
		{
			m_count.Reset();
			m_minmax.Reset();
			m_mean.Reset();
			m_variance.Reset();
		}
		void Add(Type value) noexcept
		{
			m_count.Add(value);
			m_minmax.Add(value);
			auto prev_mean = m_mean.m_mean; // Capture mean before update (Welford's algorithm needs the old mean)
			m_mean.Add(value, m_count.m_count);
			m_variance.Add(value, prev_mean, m_count.m_count);
		}
	};

	// ToDo: Build these out of the components above

	// Running Average
	// 'Type' is typically a floating point type, although this does
	// work for any type that defines the necessary operators.
	template <typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
	class Avr
	{
		// let: D(k) = X(k) - avr(k-1)           => X(k) = D(k) + avr(k-1)
		//  avr(k-1) = SUM{X(k-1)} / (k-1)       => SUM{X(k-1)} = (k-1)*avr(k-1)
		//  avr(k)   = (SUM{X(k-1)} + X(k)) / k
		//           = ((k-1)*avr(k-1) + D(k) + avr(k-1)) / k
		//           = (k*avr(k-1) - avr(k-1) + D(k) + avr(k-1)) / k
		//           = (k*avr(k-1) + D(k)) / k
		//           = avr(k-1) + D(k) / k

	protected:
		Type  m_mean;
		int  m_count;

	public:
		Avr() noexcept
			:m_mean()
			,m_count()
		{}

		int Count() const noexcept
		{
			return m_count;
		}
		Type Mean() const noexcept
		{
			return m_mean;
		}
		Type Sum() const noexcept
		{
			return m_mean * m_count;
		}

		// Reset the average
		void Reset() noexcept
		{
			m_count = 0;
			m_mean  = Type();
		}

		// Accumulate the mean for 'value' in a single pass.
		void Add(Type const& value) noexcept
		{
			++m_count;
			auto diff = value - m_mean;
			auto inv_count = static_cast<Scalar>(1.0 / m_count);
			m_mean += diff * inv_count;
		}
	};

	// Running Average and Variance
	// 'Type' is typically a floating point type, although this does
	// work for any type that defines the necessary operators.
	template <typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
	class AvrVar
	{
		// let: D(k) = X(k) - avr(k-1)           => X(k) = D(k) + avr(k-1)
		//  avr(k-1) = SUM{X(k-1)} / (k-1)       => SUM{X(k-1)} = (k-1)*avr(k-1)
		//  avr(k)   = (SUM{X(k-1)} + X(k)) / k
		//           = ((k-1)*avr(k-1) + D(k) + avr(k-1)) / k
		//           = (k*avr(k-1) - avr(k-1) + D(k) + avr(k-1)) / k
		//           = (k*avr(k-1) + D(k)) / k
		//           = avr(k-1) + D(k) / k
		// Running variance:
		//  var(k)   = (1/(k-1)) * SUM{(X(k) - avr(k))²}
		// (k-1)*var(k) = SUM{(X(k) - avr(k))²}
		//              = SUM{(X(k)² - 2*avr(k)*X(k) + avr(k)²}
		//              = SUM{X(k)²} - 2*avr(k)*SUM{X(k)} + k*avr(k)²
		//              = SUM{X(k)²} - 2*avr(k)*k*avr(k) + k*avr(k)²
		//              = SUM{X(k)²} - 2*k*avr(k)² + k*avr(k)²
		//              = SUM{X(k)²} - k*avr(k)²
		// so:
		//  (k-2)*var(k-1) = SUM{X(k-1)²} - (k-1)*avr(k-1)²
		// taking:
		//  (k-1)*var(k) - (k-2)*var(k-1) = SUM{X(k)²} - k*avr(k)² - SUM{X(k-1)²} + (k-1)*avr(k-1)²
		//                                = X(k)² - k*avr(k)² + (k-1)*avr(k-1)²
		//                                = X(k)²                               - k*(avr(k-1) + D(k)/k)²                       + k*avr(k-1)² - avr(k-1)²
		//                                = (D(k) + avr(k-1))²                  - k*(avr(k-1) + D(k)/k)²                       + k*avr(k-1)² - avr(k-1)²
		//                                = D(k)² + 2*D(k)*avr(k-1) + avr(k-1)² - k*(avr(k-1)² + 2*D(k)*avr(k-1)/k + D(k)²/k²) + k*avr(k-1)² - avr(k-1)²
		//                                = D(k)² + 2*D(k)*avr(k-1) + avr(k-1)² - k*avr(k-1)² - 2*D(k)*avr(k-1) - D(k)²/k      + k*avr(k-1)² - avr(k-1)²
		//                                = D(k)² - D(k)²/k
		//                                = ((k-1)/k) * D(k)²

	protected:
		Type  m_mean;
		Type  m_var;
		int  m_count;

	public:
		AvrVar() noexcept
			:m_mean()
			,m_var()
			,m_count()
		{}

		int Count() const { return m_count; }
		Type Mean() const  { return m_mean; }
		Type Sum() const   { return m_mean * m_count; }

		// Use the population standard deviation when all data values in a set have been considered.
		// Use the sample standard deviation when the data values used are only a sample of the total population
		Type PopStdDev() const { return static_cast<Type>(sqrt(PopStdVar())); }
		Type SamStdDev() const { return static_cast<Type>(sqrt(SamStdVar())); }
		Type PopStdVar() const { return static_cast<Type>(m_var * (1.0 / (m_count + (m_count == 0)))); }
		Type SamStdVar() const { return static_cast<Type>(m_var * (1.0 / (m_count - (m_count != 1)))); }

		// Reset the average
		void Reset() noexcept
		{
			m_count = 0;
			m_mean  = Type();
			m_var   = Type();
		}

		// Accumulate statistics for 'value' in a single pass.
		// Note, this method is more accurate than the sum of squares, square of sums approach.
		void Add(Type const& value) noexcept
		{
			++m_count;
			auto diff = value - m_mean;
			auto inv_count = static_cast<Scalar>(1.0 / m_count);
			m_mean += diff * inv_count;
			m_var  += diff * diff * ((m_count - 1) * inv_count);
		}
	};

	// Exponential Moving Average
	template <typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
	struct ExpMovingAvr
	{
		//  avr(k) = a * X(k) + (1 - a) * avr(k-1)
		//         = a * X(k) + avr(k-1) - a * avr(k-1)
		//         = avr(k-1) + a * X(k) - a * avr(k-1)
		//         = avr(k-1) + a * (X(k) - avr(k-1))
		//    'a' is the exponential smoothing factor between (0,1)
		//    define: a = 2 / (N + 1), where 'N' is roughly the window size of an equivalent moving window average
		//    The interval over which the weights decrease by a factor of two (half-life) is approximately N/2.8854
		// Exponential moving variance:
		// (k-1)var(k) = SUM{w(k) * U(k)²}, where: U(k) = X(k) - avr(k)
		//             = w(1)*U(1)² + w(2)*U(2)² + ... + w(k)*U(k)², where: w(1)+w(2)+...+w(k) = k
		// If we say:  w(k) = k * a, ('a' between (0,1) as above) then SUM{w(k-1)} = k * (1-a)
		// so consider var(k-1):
		//  (k-2)var(k-1) = w(1)*U(1)² + w(2)*U(2)² + ... + w(k-1)*U(k-1)², where: w(1)+w(2)+...+w(k-1) = k - 1
		// when we add the next term:
		//  (k-1)var(k)   = w(1)*U(1)² + w(2)*U(2)² + ... + w(k-1)*U(k-1)² + w(k)*U(k)² (note w(1)..w(k-1) will be different values to above)
		// we need:
		//   k = k*a + k*(1-a) = w(k) + b*SUM{w(k-1)}
		// => k*(1-a) = b*SUM{w(k-1)}
		//          b = (1-a)*k/SUM{w(k-1)}
		//            = (1-a)*k/(k-1)
		// so:
		// (k-1)var(k) = a*k*U(k)² + b*(k-2)var(k-1)
		//             = a*k*U(k)² + (1-a)*(k/(k-1)) * (k-2)var(k-1)
		//  let: D(k) = X(k) - avr(k-1)
		//       U(k) = X(k) - avr(k-1) + avr(k-1) - avr(k)
		//            = D(k) + avr(k-1) - avr(k)
		//            = D(k) - (avr(k) - avr(k-1))
		//            = D(k) - a * (X(k) - avr(k-1))
		//            = D(k) - a * D(k)
		//            = (1-a)*D(k)
		// then:
		// (k-1)var(k) = a*k*U(k)² + (1-a)*(k/(k-1)) * (k-2)var(k-1)
		//             = a*k*(1-a)²*D(k)² + (1-a)*(k/(k-1)) * (k-2)var(k-1)
		//             = a*k*b²*D(k)² + (b*k/(k-1)) * (k-2)var(k-1)         where: b = (1-a)
		//             = (b*k/(k-1))*((a*b*(k-1)*D(k)² + (k-2)var(k-1))
		// 'Type' is typically a floating point type, although this does
		// work for any type that defines the necessary operators.

	protected:
		Type m_mean;
		Type m_var;
		int m_size;
		int m_count;

	public:
		ExpMovingAvr(int window_size) noexcept
			:m_mean()
			,m_var()
			,m_size(window_size)
			,m_count()
		{}

		int WindowSize() const { return m_size; }
		int Count() const { return m_count; }
		Type Mean() const { return m_mean; }

		// Use the population standard deviation when all data values in a set have been considered.
		// Use the sample standard deviation when the data values used are only a sample of the total population
		// Note: for a moving variance the choice between population/sample SD is a bit arbitrary
		Type PopStdDev() const { return static_cast<Type>(CompSqrt(PopStdVar())); }
		Type SamStdDev() const { return static_cast<Type>(CompSqrt(SamStdVar())); }
		Type PopStdVar() const { return m_var * static_cast<Scalar>(1.0 / (m_count + (m_count == 0))); }
		Type SamStdVar() const { return m_var * static_cast<Scalar>(1.0 / (m_count - (m_count != 1))); }

		void Reset(int window_size) noexcept
		{
			m_size  = window_size;
			m_count = 0;
			m_mean  = Type();
			m_var   = Type();
		}
		void Add(Type const& value) noexcept
		{
			if (m_count >= m_size)
			{
				++m_count;
				auto diff = value - m_mean;
				auto a = static_cast<Scalar>(2.0 / (m_size + 1.0));
				auto b = static_cast<Scalar>(1.0 - a);
				m_mean = static_cast<Type>(m_mean + diff * a);
				m_var  = static_cast<Type>((b*m_count/(m_count-1)) * (a*b*(m_count-1)*diff*diff + m_var));
			}
			else // use standard mean/var until 'm_size' is reached
			{
				++m_count;
				auto diff = value - m_mean;
				auto inv_count = static_cast<Scalar>(1.0 / m_count);
				m_mean += static_cast<Type>(diff * inv_count);
				m_var  += static_cast<Type>(diff * diff * ((m_count - 1) * inv_count));
			}
		}
	};

	// Moving Window Average
	template <int MaxWindowSize, typename Type = double, std::floating_point Scalar = std::conditional_t<std::is_floating_point_v<Type>, Type, double>>
	class MovingAvr
	{
		// Let: D(k) = X(k) - X(k-N) => X(k-N) = X(k) - D(k)
		// Average:
		//   avr(k) = avr(k-1) + (X(k) - X(k-N)) / N
		//          = avr(k-1) + D(k) / N
		// 'Type' is typically a floating point type, although this does
		// work for any type that defines the necessary operators.

	protected:
		Type m_window[MaxWindowSize], *m_in;
		Type m_mean;
		int m_count;
		int m_size;

		Type Var() const noexcept
		{
			auto var = Type();
			auto count = m_count;
			auto const* end = &m_window[0] + m_size;
			for (auto i = m_in; i-- != m_window && count; --count) { auto diff = *i - m_mean; var += diff * diff; }
			for (auto i = end;  i-- != m_window && count; --count) { auto diff = *i - m_mean; var += diff * diff; }
			return var;
		}

	public:
		MovingAvr(int window_size = MaxWindowSize) noexcept
		{
			Reset(window_size);
		}

		int Count() const { return m_count; }
		Type Mean() const { return m_mean; }

		// NOTE: no recursive variance because we would need to buffer the averages as well
		// so that we could remove (X(k-N) - avr(k-N))^2 at each iteration
		// Use the population standard deviation when all data values in a set have been considered.
		// Use the sample standard deviation when the data values used are only a sample of the total population
		Type PopStdDev() const { return static_cast<Type>(sqrt(PopStdVar())); }
		Type SamStdDev() const { return static_cast<Type>(sqrt(SamStdVar())); }
		Type PopStdVar() const { return static_cast<Type>(Var() * static_cast<Scalar>(1.0 / (m_count + (m_count == 0)))); }
		Type SamStdVar() const { return static_cast<Type>(Var() * static_cast<Scalar>(1.0 / (m_count - (m_count != 1)))); }

		void Reset(int window_size = MaxWindowSize) noexcept
		{
			pr_assert(window_size <= MaxWindowSize);
			m_in    = &m_window[0];
			m_size  = window_size;
			m_mean  = Type();
			m_count = 0;
		}
		void Add(Type const& value) noexcept
		{
			if (m_count == m_size)
			{
				if (m_in == m_window + m_size) m_in = &m_window[0];
				auto diff = value - *m_in;
				auto inv_count = static_cast<Scalar>(1.0 / m_size);
				m_mean += diff * inv_count;
				*m_in++ = value;
			}
			else
			{
				++m_count;
				auto diff = value - m_mean;
				auto inv_count = static_cast<Scalar>(1.0 / m_count);
				m_mean += diff * inv_count;
				*m_in++ = value;
			}
		}
	};
}
