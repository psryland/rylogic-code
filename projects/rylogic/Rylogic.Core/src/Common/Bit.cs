//***************************************************
// Bit operations
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Collections.Generic;
using System.Text;
using Rylogic.Utility;

namespace Rylogic.Common
{
	/// <summary> Bit manipulation functions</summary>
	public static class Bit
	{
		public enum EState { Clear = 0, Set = 1, Toggle = 2 }

		/// <summary>Create a 32-bit mask from 1 &lt;&lt; n</summary>
		public static uint Bit32(int n) => 1U << n;

		/// <summary>Create a 64-bit mask from 1 &lt;&lt; n</summary>
		public static ulong Bit64(int n) => 1UL << n;

		/// <summary>A bit mask with the lower 'n' bits set. e.g. Mask(4) = 0b00001111</summary>
		public static ulong Mask(int n) => n < 64 ? ~(~0UL << n) : ~0UL;

		/// <summary>Get the Nth bit of this value</summary>
		public static int BitAt(this ulong x, int bit) => (int)((x >> bit) & 1);
		public static int BitAt(this  long x, int bit) => (int)((x >> bit) & 1);

		/// <summary>Get the range of bits as a value. e.g. x = 1011101, start=2,count=3 = [2,4] => 111 = 7</summary>
		[Obsolete] public static ulong Bits(this ulong x, int start, int count) => (x >> start) & ~(~0UL << count);
		[Obsolete] public static long Bits(this long x, int start, int count) => unchecked((long)Bits((ulong)x, start, count));

		/// <summary>Set/Clear bits in 'value'</summary>
		public static ulong SetBits(ulong value, ulong mask, bool state)
		{
			return state ? value | mask : value & ~mask;
		}
		public static long SetBits(long value, long mask, bool state)
		{
			return state ? value | mask : value & ~mask;
		}
		public static uint SetBits(uint value, uint mask, bool state)
		{
			return state ? value | mask : value & ~mask;
		}
		public static int SetBits(int value, int mask, bool state)
		{
			return state ? value | mask : value & ~mask;
		}
		public static T SetBits<T>(T value, T mask, bool state) where T :struct, IConvertible
		{
			return state
				? Operators<T>.BitwiseOR(value, mask)
				: Operators<T>.BitwiseAND(value, Operators<T>.OnesComp(mask));
		}

		/// <summary>Set/Clear bits in 'value'</summary>
		public static ulong SetBits(ulong value, ulong mask, ulong state)
		{
			return (value & ~mask) | (state & mask);
		}
		public static long SetBits(long value, long mask, long state)
		{
			return (value & ~mask) | (state & mask);
		}
		public static uint SetBits(uint value, uint mask, uint state)
		{
			return (value & ~mask) | (state & mask);
		}
		public static int SetBits(int value, int mask, int state)
		{
			return (value & ~mask) | (state & mask);
		}
		public static T SetBits<T>(T value, T mask, T state) where T :struct, IConvertible
		{
			return Operators<T>.BitwiseOR(
				Operators<T>.BitwiseAND(value, Operators<T>.OnesComp(mask)),
				Operators<T>.BitwiseAND(state, mask));
		}

		/// <summary>Returns true if 'value & mask' != 0</summary>
		public static bool AnySet(ulong value, ulong mask)
		{
			return (value & mask) != 0;
		}
		public static bool AnySet(long value, long mask)
		{
			return (value & mask) != 0;
		}
		public static bool AnySet(uint value, uint mask)
		{
			return (value & mask) != 0;
		}
		public static bool AnySet(int value, int mask)
		{
			return (value & mask) != 0;
		}
		public static bool AnySet(byte value, byte mask)
		{
			return (value & mask) != 0;
		}
		public static bool AnySet<T>(T value, T mask) where T :struct, IConvertible
		{
			return !Equals(Operators<T>.BitwiseAND(value, mask), default(T));
		}

		/// <summary>Returns true if 'value & mask' == mask</summary>
		public static bool AllSet(ulong value, ulong mask)
		{
			return (value & mask) == mask;
		}
		public static bool AllSet(long value, long mask)
		{
			return (value & mask) == mask;
		}
		public static bool AllSet(uint value, uint mask)
		{
			return (value & mask) == mask;
		}
		public static bool AllSet(int value, int mask)
		{
			return (value & mask) == mask;
		}
		public static bool AllSet(byte value, byte mask)
		{
			return (value & mask) == mask;
		}
		public static bool AllSet<T>(T value, T mask) where T :struct, IConvertible
		{
			return Equals(Operators<T>.BitwiseAND(value, mask), mask);
		}

		/// <summary>Iterate over the index positions of bits in a bit field</summary>
		public static IEnumerable<int> EnumBitIndices(int value)
		{
			for (var i = 0; value != 0; ++i, value >>= 1)
				if ((value&1) != 0) yield return i;
		}

		/// <summary>Iterate over the index positions of bits in a bit field</summary>
		public static IEnumerable<int> EnumBitIndices(ulong value)
		{
			for (var i = 0; value != 0; ++i, value >>= 1)
				if ((value & 1) != 0) yield return i;
		}

		/// <summary>Iterate over the set bits in a bit field</summary>
		public static IEnumerable<int> EnumBitMasks(int value)
		{
			for (int mask; value != 0; value -= mask)
				yield return mask = value & (value ^ (value - 1));
		}

		/// <summary>Iterate over the set bits in a bit field</summary>
		public static IEnumerable<ulong> EnumBitMasks(ulong value)
		{
			for (ulong mask; value != 0; value -= mask)
				yield return mask = value & (value ^ (value - 1));
		}

		/// <summary>Iterate over the bits in the range</summary>
		public static IEnumerable<int> EnumBitSeq(this ulong x, int start, int count, bool low_to_high = true)
		{
			for (int i = 0; i != count; ++i)
				yield return x.BitAt(start + (low_to_high ? i : count - 1 - i));
		}

		/// <summary>Returns a value containing the reverse of the bits in 'value'</summary>
		public static byte ReverseBits(byte value)
		{
			return (byte)(((value * 0x0802LU & 0x22110LU) | (value * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
		}

		/// <summary>Returns a value containing the reverse of the bits in 'value'</summary>
		public static ulong ReverseBits(ulong value)
		{
			value = ((value >>  1) & 0x5555555555555555UL) | ((value & 0x5555555555555555UL) <<  1); // swap odd and even bits
			value = ((value >>  2) & 0x3333333333333333UL) | ((value & 0x3333333333333333UL) <<  2); // swap consecutive pairs
			value = ((value >>  4) & 0x0F0F0F0F0F0F0F0FUL) | ((value & 0x0F0F0F0F0F0F0F0FUL) <<  4); // swap nibbles ...
			value = ((value >>  8) & 0x00FF00FF00FF00FFUL) | ((value & 0x00FF00FF00FF00FFUL) <<  8); // swap bytes
			value = ((value >> 16) & 0x0000FFFF0000FFFFUL) | ((value & 0x0000FFFF0000FFFFUL) << 16); // swap 2-byte words
			value = ((value >> 32) & 0x00000000FFFFFFFFUL) | ((value & 0x00000000FFFFFFFFUL) << 32); // swap 4-byte words
			return value;
		}

		/// <summary>
		/// Extract the bit range [hi,lo] (inclusive) from 'value'.
		/// 'hi' and 'lo' are zero-based bit indices. The returned result is shifted down by .lo'.
		/// e.g. Bits(0b11111111, 6, 3) => 0b01111000 => 0b0001111 => 0xF</summary>
		public static ulong Bits_(ulong value, int hi, int lo)
		{
			var mask = (1UL << (hi - lo + 1)) - 1;
			return (value >> lo) & mask;
		}

		///<summary>
		/// Move 'value' to the range [hi,lo] (inclusive) (masking if necessary).
		/// 'hi' and 'lo' are zero-based bit indices.
		/// e.g. BitStuff(0b11111111, 6, 3) => 0b00001111 => 0b01111000</summary>
		public static ulong BitStuff(ulong value, int hi, int lo)
		{
			var mask = (1UL << (hi - lo + 1)) - 1;
			return (value & mask) << lo;
		}

		/// <summary>Return the index of the single bit set in 'n'</summary>
		public static int Index(ulong n)
		{
			if (!IsPowerOfTwo(n)) throw new ArgumentException("BitIndex only works for powers of two");
			return HighBitIndex(n);
		}
		public static int Index(long n)
		{
			return Index(unchecked((ulong)n));
		}

		/// <summary>Returns a bit mask containing only the lowest bit of 'n'</summary>
		public static ulong LowBit(ulong n)
		{
			return unchecked(n - ((n - 1) & n));
		}
		public static long LowBit(long n)
		{
			return unchecked(n - ((n - 1) & n));
		}

		/// <summary>
		/// Returns the bit position of the highest bit.
		/// Note: returns 0 for n==1 and n==0
		/// Also, is the floor of the log base 2 for a 32 bit integer</summary>
		public static int HighBitIndex(ulong n)
		{
			var pos = 0;
			var
			shift = (n & 0xFFFFFFFF00000000UL) != 0 ? 1 << 5 : 0; n >>= shift; pos |= shift;
			shift = (n &         0xFFFF0000UL) != 0 ? 1 << 4 : 0; n >>= shift; pos |= shift;
			shift = (n &             0xFF00UL) != 0 ? 1 << 3 : 0; n >>= shift; pos |= shift;
			shift = (n &               0xF0UL) != 0 ? 1 << 2 : 0; n >>= shift; pos |= shift;
			shift = (n &                0xCUL) != 0 ? 1 << 1 : 0; n >>= shift; pos |= shift;
			shift = (n &                0x2UL) != 0 ? 1 << 0 : 0;              pos |= shift;
			return pos;
		}
		public static int HighBitIndex(long n)
		{
			return HighBitIndex(unchecked((ulong)n));
		}

		/// <summary>
		/// Returns the bit position of the lowest bit.
		/// Note: returns 0 for n==1 and n==0</summary>
		public static int LowBitIndex(ulong n)
		{
			return HighBitIndex(LowBit(n));
		}
		public static int LowBitIndex(long n)
		{
			return HighBitIndex(LowBit(n));
		}

		/// <summary>Return a bit mask contain only the highest bit of 'n'</summary> Must be a faster way?
		public static ulong HighBit(ulong n)
		{
			return 1UL << HighBitIndex(n);
		}
		public static long HighBit(long n)
		{
			return 1L << HighBitIndex(n);
		}

		/// <summary>Returns true if 'n' is a exact power of two</summary>
		public static bool IsPowerOfTwo(ulong n)
		{
			// Zero is not a power of two because 2^n means "1 doubled n times". There is no number of times you can double 1 to get zero.
			// Incidentally, this is why '2^0 == 1', "1 doubled no times" is still 1.
			return ((n - 1) & n) == 0 && n != 0;
		}
		public static bool IsPowerOfTwo(long n)
		{
			return ((n - 1) & n) == 0 && n != 0;
		}

		/// <summary>Count the bits set in 'value'</summary>
		public static int CountBits(ulong value)
		{
			int count = 0;
			while (value != 0)
			{
				++count;
				value &= (value - 1);
			}
			return count;
		}
		public static int CountBits(long value)
		{
			return CountBits(unchecked((ulong)value));
		}

		/// <summary>Return the bit position of the 'n'th set bit, starting from the LSB.
		/// 'n' is a zero based index, E.g.
		/// BitIndex(0b10010010110, 0) == 1
		/// BitIndex(0b10010010110, 1) == 2
		/// BitIndex(0b10010010110, 2) == 4
		/// BitIndex(0b10010010110, 3) == 7
		/// BitIndex(0b10010010110, 4) == 10
		/// BitIndex(0b10010010110, 5) == -1
		/// Returns -1 if less than 'index' bits are set</summary>
		public static int BitIndex(ulong value, int n)
		{
			ulong b = 1;
			for (int pos = 0; pos != 32; ++pos, b <<= 1)
				if ((value & b) != 0 && --n == -1) return pos;
			return -1;
		}

		//// http://infolab.stanford.edu/~manku/bitcount/bitcount.html
		//// Constant time bit count works for 32-bit numbers only.
		//// Fix last line for 64-bit numbers
		//public uint CountBits(uint n)
		//{
		//    uint tmp;
		//    tmp = n - ((n >> 1) & 033333333333)
		//            - ((n >> 2) & 011111111111);
		//    return ((tmp + (tmp >> 3)) & 030707070707) % 63;
		//}

		/// <summary>
		/// Interleaves the lower 16 bits of x and y, so the bits of x
		/// are in the even positions and bits from y in the odd.
		/// Returns the resulting 32-bit Morton Number.
		/// </summary>
		public static ulong InterleaveBits(ulong x, ulong y)
		{
			// todo: upgrade this to 64-bit
			var B = new ulong[]{0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF};
			var S = new []{1, 2, 4, 8};
			x = (x | (x << S[3])) & B[3];
			x = (x | (x << S[2])) & B[2];
			x = (x | (x << S[1])) & B[1];
			x = (x | (x << S[0])) & B[0];
			y = (y | (y << S[3])) & B[3];
			y = (y | (y << S[2])) & B[2];
			y = (y | (y << S[1])) & B[1];
			y = (y | (y << S[0])) & B[0];
			return x | (y << 1);
		}

		/// <summary>Convert an array of signs to an index. signs[start] is the MSB, signs[start+length-1] is the LSB. "sign >= 0 ? 1 : 0"</summary>
		public static int SignsToIndex(IList<int> signs)
		{
			return SignsToIndex(signs, 0, signs.Count);
		}
		public static int SignsToIndex(IList<int> signs, int start, int length)
		{
			var idx = 0;
			for (int i = 0; i != length; ++i)
			{
				idx = (idx << 1) | (signs[start+i] >= 0 ? 1 : 0);
			}
			return idx;
		}

		/// <summary>Convert a string (e.g. "10010110101") into a uint</summary>
		public static ulong Parse(string bits)
		{
			if (bits == null) throw new ArgumentNullException("bits", "Bits.Parse() string argument was null");
			uint n = 0;
			foreach (char c in bits)
			{
				if      (c == '0') n = (n << 1);
				else if (c == '1') n = (n << 1) | 1U;
				else throw new FormatException("Bits.Parse() Argument must be a string containing only '0's and '1's");
			}
			return n;
		}

		/// <summary>Try to convert a string (e.g. "10010110101") into a uint</summary>
		public static bool TryParse(string bits, out ulong value)
		{
			value = 0;
			if (bits == null) return false;
			foreach (char c in bits)
			{
				if      (c == '0') value = (value << 1);
				else if (c == '1') value = (value << 1) | 1U;
				else return false;
			}
			return true;
		}

		/// <summary>Convert the bit field 'bits' into a string with at least 'min_digits' characters</summary>
		public static string ToString(ulong bits, int min_digits)
		{
			const ulong MSB = 1UL << 63;
			unchecked
			{
				var i = 0;
				var str = new StringBuilder(64);
				for (; i != 64 && min_digits != 64 && (bits & MSB) == 0; ++i, ++min_digits, bits <<= 1) {}
				for (; i != 64; ++i, bits <<= 1) { str.Append((bits & MSB) != 0 ? '1' : '0'); }
				return str.ToString();
			}
		}
		public static string ToString(long bits, int min_digits)
		{
			return unchecked(ToString((ulong)bits, min_digits));
		}

		/// <summary>Convert the bit field 'bits' into a string</summary>
		public static string ToString(ulong bits) => ToString(bits, 0);
		public static string ToString(long bits) => ToString(bits, 0);

		///<summary>Encode an integer value as a variable length int byte array</summary>
		public static IList<byte> EncodeVarInt(long value)
		{
			// VarInt encoding (as used in Google Protobuf).
			// https://developers.google.com/protocol-buffers/docs/encoding#varints
			// Algorthm:
			//  Convert value to unsigned using zig-zap encoding
			//  Stop consuming bytes if the high bit is set.
			//  Trim the high bit, then left-side-append bits to form the number.
			// e.g.
			//  300 = 0000010 0101100
			//  Encoded becomes: 10101100 00000010
			//  Decoding:
			//   byte[0] = (10101100 & 0x7F) => 0101100
			//   byte[1] = (00000010 & 0x7F) => 0000010 ++ 0101100 => 100101100
			//   stop because high bit wasn't set.

			var val = unchecked((ulong)((value << 1) ^ (value >> 63)));
			return EncodeVarUInt(val);
		}
		public static IList<byte> EncodeVarUInt(ulong value)
		{
			var res = new List<byte>();
			for (;;)
			{
				var highbit = value >= 128 ? 0x80UL : 0x00UL;
				res.Add((byte)(highbit | (value & 0x7f)));
				if (highbit == 0) break;
				value >>= 7;
			}
			return res;
		}

		/// <summary>Decode a variable length integer. 'start' is the offset into 'arr'.</summary>
		public static long DecodeVarInt(IList<byte> arr, int start = 0)
		{
			return DecodeVarInt(arr, ref start);
		}
		public static ulong DecodeVarUInt(IList<byte> arr, int start = 0)
		{
			return DecodeVarUInt(arr, ref start);
		}

		/// <summary>Decode a variable length integer. 'start' is the offset into 'arr'. 'count' is the number of bytes consumed</summary>
		public static long DecodeVarInt(IList<byte> arr, int start, out int count)
		{
			var index = start;
			var res = DecodeVarInt(arr, ref index);
			count = index - start;
			return res;
		}
		public static ulong DecodeVarUInt(IList<byte> arr, int start, out int count)
		{
			var index = start;
			var res = DecodeVarUInt(arr, ref index);
			count = index - start;
			return res;
		}

		/// <summary>Decode a variable length integer. 'index' is the offset into 'arr'. On return, 'index' points to the first byte after the variable length int</summary>
		public static long DecodeVarInt(IList<byte> arr, ref int index)
		{
			var res = DecodeVarUInt(arr, ref index);
			var val = (long)(res >> 1) ^ (-(long)(res & 1));
			return val;
		}
		public static ulong DecodeVarUInt(IList<byte> arr, ref int index)
		{
			if (index < 0 || index >= arr.Count)
				throw new ArgumentOutOfRangeException($"Index ({index}) out of range");

			var res = 0UL;
			for (var shift = 0; ;)
			{
				res += (ulong)(arr[index] & 0x7F) << shift;
				if ((arr[index++] & 0x80) == 0) break;
				if (index == arr.Count) throw new Exception("Invalid VarInt encoding");
				if ((shift += 7) >= 64) throw new Exception("Encoded VarInt overflows an int64");
			}
			return res;
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using System.Linq;
	using Rylogic.Common;

	[TestFixture]
	public class TestBit
	{
		public enum NonFlags
		{
			One = 1,
			Two = 2,
		}
		[Flags]
		public enum Flags
		{
			One = 1 << 0,
			Two = 1 << 1,
			Three = 1 << 2,
			Four = 1 << 3,
			Five = 1 << 4,
		}

		[Test]
		public void HasFlag()
		{
			const NonFlags nf = NonFlags.One;
			const Flags f = Flags.One;

			Assert.Throws(typeof(ArgumentException), () => Assert.True(f.HasFlag(nf)));
			Assert.True(f.HasFlag(Flags.One));
			Assert.False(f.HasFlag(Flags.One | Flags.Two));
			Assert.True(Bit.AllSet(f, Flags.One));
			Assert.False(Bit.AllSet(f, Flags.Two));

			var mask = Bit.SetBits(f, Flags.One | Flags.Two | Flags.Three, true);
			Assert.Equal(mask, Flags.One | Flags.Two | Flags.Three);
		}
		[Test]
		public void BitParse()
		{
			const string bitstr0 = "100100010100110010110";
			const string bitstr1 = "011011101011001101001";

			var bits0 = Bit.Parse(bitstr0);
			var bits1 = Bit.Parse(bitstr1);
			Assert.Equal(bitstr0, Bit.ToString(bits0));
			Assert.Equal(bitstr1, Bit.ToString(bits1, bitstr1.Length));
		}
		[Test]
		public void BitAnySet()
		{
			const string bitstr0 = "100100010100110010110";
			const string bitstr1 = "011011101011001101001";
			var bits0 = Bit.Parse(bitstr0);
			var bits1 = Bit.Parse(bitstr1);
			Assert.False(Bit.AnySet(bits0, bits1));
		}
		[Test]
		public void BitIndex()
		{
			const string bitstr0 = "100100010100110010110";
			var bits0 = Bit.Parse(bitstr0);
			Assert.Equal(8, Bit.BitIndex(bits0, 4));
			Assert.Equal(13, Bit.BitIndex(bits0, 6));
			Assert.Equal(-1, Bit.BitIndex(bits0, 11));
		}
		[Test]
		public void EnumBitIndices()
		{
			const string bitstr0 = "100100010100110010110";
			var bits0 = Bit.Parse(bitstr0);
			var bitidxs = new[] { 1, 2, 4, 7, 8, 11, 13, 17, 20 };
			var idxs = Bit.EnumBitIndices(bits0).ToArray();
			Assert.True(bitidxs.SequenceEqual(idxs));
		}
		[Test]
		public void EnumBitMasks()
		{
			const string bitstr0 = "100100010100110010110";
			var bits0 = Bit.Parse(bitstr0);
			var bitmasks = new[]
			{
				Bit.Parse("000000000000000000010"),
				Bit.Parse("000000000000000000100"),
				Bit.Parse("000000000000000010000"),
				Bit.Parse("000000000000010000000"),
				Bit.Parse("000000000000100000000"),
				Bit.Parse("000000000100000000000"),
				Bit.Parse("000000010000000000000"),
				Bit.Parse("000100000000000000000"),
				Bit.Parse("100000000000000000000")
			};
			var masks = Bit.EnumBitMasks(bits0).ToArray();
			Assert.True(bitmasks.SequenceEqual(masks));
		}
		[Test]
		public void VarInt()
		{
			var values = new[]
			{
				+0L, +127L, +128L, +300L, +30_000_000_000_000L, long.MaxValue,
				-0L, -127L, -128L, -300L, -30_000_000_000_000L, long.MinValue,
			};
			foreach (var val in values)
			{
				var buf = Bit.EncodeVarInt(val);
				var res = Bit.DecodeVarInt(buf);
				Assert.True(res == val);
			}
		}
		[Test]
		public void VarUInt()
		{
			var values = new[] { 0UL, 127UL, 128UL, 300UL, 30_000_000_000_000UL, ulong.MaxValue };
			foreach (var val in values)
			{
				var buf = Bit.EncodeVarUInt(val);
				var res = Bit.DecodeVarUInt(buf);
				Assert.True(res == val);
			}
		}
	}
}
#endif
