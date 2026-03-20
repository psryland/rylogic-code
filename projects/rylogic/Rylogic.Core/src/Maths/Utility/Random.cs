using Rylogic.Maths;
using System;
using System.Collections.Generic;
using System.Text;

namespace Rylogic
{
	public class DistinctRandomSequence
	{
		// Notes:
		// Based on quadratic residues. See 'http://preshing.com/20121224/how-to-generate-a-sequence-of-unique-random-integers/'
		private readonly int m_prime;
		private readonly int m_max;
		private int m_index;
		private int m_offset;

		// because this is the largest signed int prime value
		public const int MaxValue = 2147483629;

		/// <summary>
		/// Generates a non-repeating pseudo random sequence of numbers in the range [0,max).
		/// After 'max' values have been returned, the sequence repeats, outputting the same sequence.
		/// 'seed' is the random generator seed.
		/// 'offset' is the offset into the cyclic sequence of numbers</summary>
		public DistinctRandomSequence(int max)
			: this(max, new Random().Next())
		{ }
		public DistinctRandomSequence(int max, int seed)
		{
			if (max > MaxValue)
				throw new ArgumentException($"{nameof(max)} must be <= {MaxValue}", nameof(max));

			m_max = max;

			// Select a prime greater than 'max' that is equivalent to "3 % 4"
			m_prime = (int)Math_.PrimeGtrEq(max);
			for (; (m_prime % 4) != 3; )
				m_prime = (int)Math_.PrimeGtrEq(m_prime + 1);

			// Use an index and offset to randomise the order of returned values
			var rng = new Random(seed);
			m_index = rng.Next(m_max);
			m_offset = rng.Next();
		}

		/// <summary>Return the next value in the sequence</summary>
		public int Next()
		{
			for (; ; )
			{
				var value = Permute(Permute(m_index++) + m_offset);
				if (value >= m_max) continue;
				return value;
			}
		}

		/// <summary></summary>
		private int Permute(int index)
		{
			var x = (long)index;
			var residue = (int)((x * x) % m_prime);
			return 2 * (index % m_prime) <= m_prime ? residue : m_prime - residue;
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using System.Text;
	using System.Linq;
	using Maths;
	using System.Diagnostics;

	[TestFixture]
	public class TestMathsDRS
	{
		[Test]
		public void TestDistinctRandomSequence()
		{
			foreach (var sz in new[] { 10, 100, 1000, 64, 13 })
			{
				var buf = new bool[sz];
				var drs = new DistinctRandomSequence(buf.Length, sz);
				for (int i = 0; i != buf.Length; ++i)
				{
					var x = drs.Next();
					Assert.False(buf[x]);
					buf[x] = true;
				}
				Assert.True(buf.All(x => x));
			}
		}
	}
}
#endif