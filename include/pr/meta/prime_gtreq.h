// Shamelessly lifted from Steve Robb's fuz library

#pragma once

#include "pr/meta/is_prime.h"

namespace pr
{
	namespace meta
	{
		namespace impl
		{
			namespace prime_gtreq
			{
				template <std::size_t OddValue, bool IsPrime>
				struct impl
				{
					static const std::size_t next  = OddValue + static_cast<std::size_t>(2);
					static const std::size_t value = impl<next, meta::is_prime<next>::value>::value;
				};

				template <std::size_t OddValue>
				struct impl<OddValue, true>
				{
					static const std::size_t value = OddValue;
				};
			}
		}

		template <std::size_t Value>
		struct prime_gtreq
		{
			static const std::size_t value = (Value <= 2) ? 2 : impl::prime_gtreq::impl<Value|1, meta::is_prime<Value|1>::value>::value;
		};
	}
}