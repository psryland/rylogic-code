//***************************************************
// Colour32
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using Rylogic.Common;
using Rylogic.Maths;

namespace Rylogic.Gfx
{
	[StructLayout(LayoutKind.Sequential)]
	public struct Colour32 :IComparable<Colour32> ,IComparable
	{
		private uint m_argb;

		static Colour32()
		{
			NamedValues = typeof(Colour32)
				.GetProperties(BindingFlags.Static | BindingFlags.Public)
				.Where(x => x.PropertyType == typeof(Colour32))
				.ToDictionary(x => x.Name, x => (Colour32)x.GetValue(null)!);
		}
		public Colour32(uint argb)
		{
			m_argb = argb;
		}
		public Colour32(byte a, byte r, byte g, byte b)
			:this(((uint)a << 24) | ((uint)r << 16) | ((uint)g << 8) | b)
		{ }
		public Colour32(float a, float r, float g, float b)
			:this((byte)(0xFF * a), (byte)(0xFF * r), (byte)(0xFF * g), (byte)(0xFF * b))
		{ }

		/// <summary>Uint representation</summary>
		public uint ARGB => m_argb;

		/// <summary>Opaque colour</summary>
		public Colour32 RGB => new(m_argb | 0xFF000000);

		/// <summary>Test if this colour has transparency</summary>
		public bool HasAlpha => (m_argb & 0xFF000000) != 0xFF000000;

		// Byte components
		public byte A { get => (byte)((m_argb >> 24) & 0xFF); set => m_argb = ((uint)value << 24) | (m_argb & 0x00FFFFFF); }
		public byte R { get => (byte)((m_argb >> 16) & 0xFF); set => m_argb = ((uint)value << 16) | (m_argb & 0xFF00FFFF); }
		public byte G { get => (byte)((m_argb >>  8) & 0xFF); set => m_argb = ((uint)value <<  8) | (m_argb & 0xFFFF00FF); }
		public byte B { get => (byte)((m_argb >>  0) & 0xFF); set => m_argb = ((uint)value <<  0) | (m_argb & 0xFFFFFF00); }

		// Normalised float components
		public float Af { get => A / 255f; set => A = (byte)((int)(value * 255) & 0xFF); }
		public float Rf { get => R / 255f; set => R = (byte)((int)(value * 255) & 0xFF); }
		public float Gf { get => G / 255f; set => G = (byte)((int)(value * 255) & 0xFF); }
		public float Bf { get => B / 255f; set => B = (byte)((int)(value * 255) & 0xFF); }

		// Conversion
		public Color ToColor() => Color.FromArgb(A, R, G, B);
		public Colour128 ToColour128() => new(Af, Rf, Gf, Bf);
		public override string ToString() => $"{A:X2}{R:X2}{G:X2}{B:X2}";

		// Interop
		public static implicit operator uint(Colour32 col)  { return col.m_argb; }
		public static implicit operator Color(Colour32 col) { return Color.FromArgb(col.A, col.R, col.G, col.B); }
		public static implicit operator Colour32(Color col) { return new Colour32(unchecked((uint)col.ToArgb())); }
		public static implicit operator Colour32(uint col)  { return new Colour32(col); }

		// Operators
		public static Colour32 operator + (Colour32 lhs, Colour32 rhs)
		{
			return new Colour32(Sat(lhs.A+rhs.A), Sat(lhs.R+rhs.R), Sat(lhs.G+rhs.G), Sat(lhs.B+rhs.B));
		}
		public static Colour32 operator - (Colour32 lhs, Colour32 rhs)
		{
			return new Colour32(Sat(lhs.A-rhs.A), Sat(lhs.R-rhs.R), Sat(lhs.G-rhs.G), Sat(lhs.B-rhs.B));
		}

		/// <summary>Saturate 'i'</summary>
		private static byte Sat(int i)
		{
			return (byte)Math.Min(0xff, Math.Max(0, i));
		}

		/// <summary>Linearly interpolate two colours (with alpha)</summary>
		public Colour32 Lerp(Colour32 rhs, double t)
		{
			return Lerp(this, rhs, t);
		}
		public static Colour32 Lerp(Colour32 lhs, Colour32 rhs, double t)
		{
			var t0 = Math_.Clamp(1.0 - t, 0.0, 1.0);
			var t1 = Math_.Clamp(0.0 + t, 0.0, 1.0);
			return new Colour32(
				(byte)Math_.Clamp(lhs.A * t0 + rhs.A * t1, 0, 0xff),
				(byte)Math_.Clamp(lhs.R * t0 + rhs.R * t1, 0, 0xff),
				(byte)Math_.Clamp(lhs.G * t0 + rhs.G * t1, 0, 0xff),
				(byte)Math_.Clamp(lhs.B * t0 + rhs.B * t1, 0, 0xff)
			);
		}
		public static Colour32 Lerp(double t, params (Colour32, double)[] p)
		{
			// e.g. Colour32.Lerp(i/9.0, new[] { (Colour32.White, 0.2), (Colour32.Yellow, 0.5), (Colour32.Red, 1.0) });
			if (p.Length == 0)
				throw new Exception("Colour32.Lerp requires at least one colour to blend");

			var idx = 0;
			for (; idx != p.Length && t > p[idx].Item2; ++idx) { }

			if (idx == 0)
				return p[0].Item1;
			if (idx == p.Length)
				return p[p.Length - 1].Item1;

			var tt = Math_.Frac(p[idx - 1].Item2, t, p[idx].Item2);
			return Lerp(p[idx - 1].Item1, p[idx].Item1, tt);
		}

		/// <summary>Linearly interpolate the non-alpha channels of two colours (lhs.A is used)</summary>
		public Colour32 LerpRGB(Colour32 rhs, double t)
		{
			return LerpRGB(this, rhs, t);
		}
		public static Colour32 LerpRGB(Colour32 lhs, Colour32 rhs, double t)
		{
			var t0 = Math_.Clamp(1.0 - t, 0.0, 1.0);
			var t1 = Math_.Clamp(0.0 + t, 0.0, 1.0);
			return new Colour32(
				lhs.A,
				(byte)Math_.Clamp(lhs.R * t0 + rhs.R * t1, 0, 0xff),
				(byte)Math_.Clamp(lhs.G * t0 + rhs.G * t1, 0, 0xff),
				(byte)Math_.Clamp(lhs.B * t0 + rhs.B * t1, 0, 0xff));
		}
		public static Colour32 LerpRGB(double t, params (Colour32, double)[] p)
		{
			// e.g. Colour32.Lerp(i/9.0, new[] { (Colour32.White, 0.2), (Colour32.Yellow, 0.5), (Colour32.Red, 1.0) });
			if (p.Length == 0)
				throw new Exception("Colour32.Lerp requires at least one colour to blend");

			var idx = 0;
			for (; idx != p.Length && t > p[idx].Item2; ++idx) { }
			if (idx == 0) return p[0].Item1;
			if (idx == p.Length) return p[p.Length - 1].Item1;
			return LerpRGB(p[idx - 1].Item1, p[idx].Item1, Math_.Frac(p[idx - 1].Item2, t, p[idx].Item2));
		}

		/// <summary>Lerp this colour toward black by 't'</summary>
		public Colour32 Darken(double t, bool alpha_too = false)
		{
			return alpha_too ? Lerp(Black, t) : LerpRGB(Black, t);
		}

		/// <summary>Lerp this colour toward white by 't'</summary>
		public Colour32 Lighten(double t, bool alpha_too = false)
		{
			return alpha_too ? Lerp(White, t) : LerpRGB(White, t);
		}

		/// <summary>Invert the colour e.g. White -> Black</summary>
		public Colour32 Invert(bool alpha_too = false)
		{
			return alpha_too ? ~m_argb : Bit.SetBits(m_argb, 0x00FFFFFF, ~m_argb);
		}

		/// <summary>Return Black or White, whichever is best contrast to this colour</summary>
		public Colour32 InvertBW(Colour32? black = null, Colour32? white = null)
		{
			return Intensity > 0.6 ? (black ?? Black) : (white ?? White);
		}

		/// <summary>Return this colour with the alpha value changed</summary>
		public Colour32 Alpha(byte alpha)
		{
			return Bit.SetBits(m_argb, 0xFF000000, (uint)alpha << 24);
		}
		public Colour32 Alpha(double alpha)
		{
			return Alpha((byte)(alpha * 0xFF));
		}

		/// <summary>A normalised measure of how bright the colour is (Based on the HSP Colour Model)</summary>
		public double Intensity => Math.Sqrt(0.299 * Rf * Rf + 0.587 * Gf * Gf + 0.114 * Bf * Bf);

		#region Parse
		public static Colour32 Parse(string s, int radix = 16)
		{
			return TryParse(s, out var col, radix) ? col :
				throw new FormatException($"String '{s}' is not a valid colour");
		}
		public static bool TryParse(string s, out Colour32 col, int radix = 16)
		{
			if (s == null)
				throw new ArgumentNullException("s", "Colour32 parse string argument was null");
			if (radix != 16 && radix != 10)
				throw new FormatException("Colour32 parse only supports radix = 10 or 16");

			byte a, r, g, b;

			// Accept a named colour
			if (NamedValues.TryGetValue(s, out col))
				return true;

			// Accept #AARRGGBB
			if (s.Length == 9 && s[0] == '#' &&
				uint.TryParse(s.Substring(1, 8), NumberStyles.HexNumber, null, out var aarrggbb))
			{
				col = new Colour32(aarrggbb);
				return true;
			}

			// Accept #RRGGBB
			if (s.Length == 7 && s[0] == '#' &&
				byte.TryParse(s.Substring(1, 2), NumberStyles.HexNumber, null, out r) &&
				byte.TryParse(s.Substring(3, 2), NumberStyles.HexNumber, null, out g) &&
				byte.TryParse(s.Substring(5, 2), NumberStyles.HexNumber, null, out b))
			{
				col = new Colour32(0xFF, r, g, b);
				return true;
			}

			// Accept #ARGB
			if (s.Length == 5 && s[0] == '#' &&
				byte.TryParse(s.Substring(1, 1), NumberStyles.HexNumber, null, out a) &&
				byte.TryParse(s.Substring(2, 1), NumberStyles.HexNumber, null, out r) &&
				byte.TryParse(s.Substring(3, 1), NumberStyles.HexNumber, null, out g) &&
				byte.TryParse(s.Substring(4, 1), NumberStyles.HexNumber, null, out b))
			{
				// 17 because 17 * 15 = 255
				col = new Colour32((byte)(17 * a), (byte)(17 * r), (byte)(17 * g), (byte)(17 * b));
				return true;
			}
			
			// Accept #RGB
			if (s.Length == 4 && s[0] == '#' &&
				byte.TryParse(s.Substring(1, 1), NumberStyles.HexNumber, null, out r) &&
				byte.TryParse(s.Substring(2, 1), NumberStyles.HexNumber, null, out g) &&
				byte.TryParse(s.Substring(3, 1), NumberStyles.HexNumber, null, out b))
			{
				// 17 because 17 * 15 = 255
				col = new Colour32(0xFF, (byte)(17 * r), (byte)(17 * g), (byte)(17 * b));
				return true;
			}

			// Accept Hex or Decimal integers
			return
				radix == 16 ? uint.TryParse(s, NumberStyles.HexNumber, null, out col.m_argb) :
				radix == 10 ? uint.TryParse(s, NumberStyles.Integer, null, out col.m_argb) :
				uint.TryParse(s, out col.m_argb);
		}
		public static Colour32? TryParse(string s, int radix = 16)
		{
			return TryParse(s, out var col, radix) ? (Colour32?)col : null;
		}
		#endregion

		#region IComparible
		public int CompareTo(Colour32 rhs)
		{
			// Compare using RGBA so that the same colours with different alpha are adjacent
			var lhs = this;
			var l_rgb = ((lhs.m_argb << 8) & 0xFFFFFF00) | ((lhs.m_argb >> 24) & 0xFF);
			var r_rgb = ((rhs.m_argb << 8) & 0xFFFFFF00) | ((rhs.m_argb >> 24) & 0xFF);
			return l_rgb.CompareTo(r_rgb);
		}
		public int CompareTo(object? rhs)
		{
			if (rhs == null) throw new ArgumentNullException(nameof(rhs));
			return CompareTo((Colour32)rhs);
		}
		#endregion

		#region Constants
		public static Dictionary<string, Colour32> NamedValues { get; }

		public static Colour32 Zero => new(0x00, 0x00, 0x00, 0x00);
		public static Colour32 Black => new(0xFF, 0x00, 0x00, 0x00);
		public static Colour32 White => new(0xFF, 0xFF, 0xFF, 0xFF);
		public static Colour32 Red => new(0xFF, 0xFF, 0x00, 0x00);
		public static Colour32 Green => new(0xFF, 0x00, 0xFF, 0x00);
		public static Colour32 Blue => new(0xFF, 0x00, 0x00, 0xFF);
		public static Colour32 Yellow => new(0xFF, 0xFF, 0xFF, 0x00);
		public static Colour32 Aqua => new(0xFF, 0x00, 0xFF, 0xFF);
		public static Colour32 Magenta => new(0xFF, 0xFF, 0x00, 0xFF);
		public static Colour32 Gray => new(0xFF, 0x80, 0x80, 0x80);

		public static Colour32 MediumBlue => Color.MediumBlue;
		public static Colour32 MediumOrchid => Color.MediumOrchid;
		public static Colour32 MediumPurple => Color.MediumPurple;
		public static Colour32 MediumSeaGreen => Color.MediumSeaGreen;
		public static Colour32 MediumSlateBlue => Color.MediumSlateBlue;
		public static Colour32 MediumSpringGreen => Color.MediumSpringGreen;
		public static Colour32 MediumTurquoise => Color.MediumTurquoise;
		public static Colour32 MediumAquamarine => Color.MediumAquamarine;
		public static Colour32 MediumVioletRed => Color.MediumVioletRed;
		public static Colour32 MintCream => Color.MintCream;
		public static Colour32 MistyRose => Color.MistyRose;
		public static Colour32 Moccasin => Color.Moccasin;
		public static Colour32 NavajoWhite => Color.NavajoWhite;
		public static Colour32 Navy => Color.Navy;
		public static Colour32 OldLace => Color.OldLace;
		public static Colour32 Olive => Color.Olive;
		public static Colour32 MidnightBlue => Color.MidnightBlue;
		public static Colour32 OliveDrab => Color.OliveDrab;
		public static Colour32 Maroon => Color.Maroon;
		public static Colour32 Linen => Color.Linen;
		public static Colour32 LemonChiffon => Color.LemonChiffon;
		public static Colour32 LightBlue => Color.LightBlue;
		public static Colour32 LightCoral => Color.LightCoral;
		public static Colour32 LightCyan => Color.LightCyan;
		public static Colour32 LightGoldenrodYellow => Color.LightGoldenrodYellow;
		public static Colour32 LightGray => Color.LightGray;
		public static Colour32 LightGreen => Color.LightGreen;
		public static Colour32 LightPink => Color.LightPink;
		public static Colour32 LightSeaGreen => Color.LightSeaGreen;
		public static Colour32 LightSkyBlue => Color.LightSkyBlue;
		public static Colour32 LightSlateGray => Color.LightSlateGray;
		public static Colour32 LightSteelBlue => Color.LightSteelBlue;
		public static Colour32 LightYellow => Color.LightYellow;
		public static Colour32 Lime => Color.Lime;
		public static Colour32 LimeGreen => Color.LimeGreen;
		public static Colour32 LightSalmon => Color.LightSalmon;
		public static Colour32 Orange => Color.Orange;
		public static Colour32 OrangeRed => Color.OrangeRed;
		public static Colour32 Orchid => Color.Orchid;
		public static Colour32 SkyBlue => Color.SkyBlue;
		public static Colour32 SlateBlue => Color.SlateBlue;
		public static Colour32 SlateGray => Color.SlateGray;
		public static Colour32 Snow => Color.Snow;
		public static Colour32 SpringGreen => Color.SpringGreen;
		public static Colour32 SteelBlue => Color.SteelBlue;
		public static Colour32 Tan => Color.Tan;
		public static Colour32 Silver => Color.Silver;
		public static Colour32 Teal => Color.Teal;
		public static Colour32 Tomato => Color.Tomato;
		public static Colour32 Transparent => Color.Transparent;
		public static Colour32 Turquoise => Color.Turquoise;
		public static Colour32 Violet => Color.Violet;
		public static Colour32 Wheat => Color.Wheat;
		public static Colour32 WhiteSmoke => Color.WhiteSmoke;
		public static Colour32 Thistle => Color.Thistle;
		public static Colour32 Sienna => Color.Sienna;
		public static Colour32 SeaShell => Color.SeaShell;
		public static Colour32 SeaGreen => Color.SeaGreen;
		public static Colour32 PaleGoldenrod => Color.PaleGoldenrod;
		public static Colour32 PaleGreen => Color.PaleGreen;
		public static Colour32 PaleTurquoise => Color.PaleTurquoise;
		public static Colour32 PaleVioletRed => Color.PaleVioletRed;
		public static Colour32 PapayaWhip => Color.PapayaWhip;
		public static Colour32 PeachPuff => Color.PeachPuff;
		public static Colour32 Peru => Color.Peru;
		public static Colour32 Pink => Color.Pink;
		public static Colour32 Plum => Color.Plum;
		public static Colour32 PowderBlue => Color.PowderBlue;
		public static Colour32 Purple => Color.PowderBlue;
		public static Colour32 RosyBrown => Color.RosyBrown;
		public static Colour32 RoyalBlue => Color.RoyalBlue;
		public static Colour32 SaddleBrown => Color.SaddleBrown;
		public static Colour32 Salmon => Color.Salmon;
		public static Colour32 SandyBrown => Color.SandyBrown;
		public static Colour32 LavenderBlush => Color.LavenderBlush;
		public static Colour32 LawnGreen => Color.LawnGreen;
		public static Colour32 Khaki => Color.Khaki;
		public static Colour32 DarkMagenta => Color.DarkMagenta;
		public static Colour32 DarkKhaki => Color.DarkKhaki;
		public static Colour32 DarkGreen => Color.DarkGreen;
		public static Colour32 DarkGray => Color.DarkGray;
		public static Colour32 DarkGoldenrod => Color.DarkGoldenrod;
		public static Colour32 Lavender => Color.Lavender;
		public static Colour32 DarkBlue => Color.DarkBlue;
		public static Colour32 Cyan => Color.Cyan;
		public static Colour32 Crimson => Color.Crimson;
		public static Colour32 Cornsilk => Color.Cornsilk;
		public static Colour32 CornflowerBlue => Color.CornflowerBlue;
		public static Colour32 Coral => Color.Coral;
		public static Colour32 Chocolate => Color.Chocolate;
		public static Colour32 DarkOliveGreen => Color.DarkOliveGreen;
		public static Colour32 Chartreuse => Color.Chartreuse;
		public static Colour32 BurlyWood => Color.BurlyWood;
		public static Colour32 Brown => Color.Brown;
		public static Colour32 BlueViolet => Color.BlueViolet;
		public static Colour32 BlanchedAlmond => Color.BlanchedAlmond;
		public static Colour32 Bisque => Color.Bisque;
		public static Colour32 Beige => Color.Beige;
		public static Colour32 Azure => Color.Azure;
		public static Colour32 Aquamarine => Color.Aquamarine;
		public static Colour32 AntiqueWhite => Color.AntiqueWhite;
		public static Colour32 AliceBlue => Color.AliceBlue;
		public static Colour32 CadetBlue => Color.CadetBlue;
		public static Colour32 DarkOrange => Color.DarkOrange;
		public static Colour32 DarkCyan => Color.DarkCyan;
		public static Colour32 DarkRed => Color.DarkRed;
		public static Colour32 Ivory => Color.Ivory;
		public static Colour32 Indigo => Color.Indigo;
		public static Colour32 IndianRed => Color.IndianRed;
		public static Colour32 HotPink => Color.HotPink;
		public static Colour32 Honeydew => Color.Honeydew;
		public static Colour32 DarkOrchid => Color.DarkOrchid;
		public static Colour32 Goldenrod => Color.Goldenrod;
		public static Colour32 Gold => Color.Gold;
		public static Colour32 GhostWhite => Color.GhostWhite;
		public static Colour32 Gainsboro => Color.Gainsboro;
		public static Colour32 Fuchsia => Color.Fuchsia;
		public static Colour32 GreenYellow => Color.GreenYellow;
		public static Colour32 FloralWhite => Color.FloralWhite;
		public static Colour32 ForestGreen => Color.ForestGreen;
		public static Colour32 DarkSalmon => Color.DarkSalmon;
		public static Colour32 DarkSeaGreen => Color.DarkSeaGreen;
		public static Colour32 DarkSlateBlue => Color.DarkSlateBlue;
		public static Colour32 DarkSlateGray => Color.DarkSlateGray;
		public static Colour32 DarkTurquoise => Color.DarkTurquoise;
		public static Colour32 DarkViolet => Color.DarkViolet;
		public static Colour32 YellowGreen => Color.YellowGreen;
		public static Colour32 DeepSkyBlue => Color.DeepSkyBlue;
		public static Colour32 DimGray => Color.DimGray;
		public static Colour32 DeepPink => Color.DeepPink;
		public static Colour32 DodgerBlue => Color.DodgerBlue;
		public static Colour32 Firebrick => Color.Firebrick;
		#endregion

		#region Palettes
		public static class Palette
		{
			/// <summary>Plot colour generator - Qualitative data - Colours that are all distinct representing categorical data (e.g. countries, genders, etc)</summary>
			public static class Qualitative
			{
				// Notes:
				//  - See https://chartio.com/learn/charts/how-to-choose-colors-data-visualization

				public static int Index = 0;
				public static Colour32 Next() => Next(Index++);
				public static Colour32 Next(int i) => m_palette[i % m_palette.Length];
				private static readonly Colour32[] m_palette =
				{
					// See 'art/jpg/palette-qualitative.jpg'
					0xFF0B84A5, 0xFFF6C85F, 0xFF6F4E7C, 0xFF9DD866, 0xFFCA472F, 0xFFFFA056, 0xFF8DDDD0,
					0xFFB35EDB, 0xFF53DB8E, 0xFF41C1DB, 0xFF8C90DB, 0xFFDB7FAE, 0xFFDBB132, 0xFF76DB69,
				};
			}

			/// <summary>Plot colour generator - High contrast</summary>
			public static class HighContrast
			{
				public static int Index = 0;
				public static Colour32 Next() => Next(Index++);
				public static Colour32 Next(int i) => m_palette[i % m_palette.Length];
				private static readonly Colour32[] m_palette =
				{
					Colour32.Black     ,
					Colour32.Blue      , Colour32.Red         , Colour32.Green      ,
					Colour32.DarkBlue  , Colour32.DarkRed     , Colour32.DarkGreen  ,
					Colour32.Purple    , Colour32.Turquoise   , Colour32.Magenta    ,
					Colour32.Orange    , Colour32.Yellow      ,
					Colour32.LightBlue , Colour32.LightSalmon , Colour32.LightGreen ,
				};
			}

			/// <summary>Plot colour generator - Basic</summary>
			public static class Basic
			{
				public static int Index = 0;
				public static Colour32 Next() => Next(Index++);
				public static Colour32 Next(int i) => m_palette[i % m_palette.Length];
				private static readonly Colour32[] m_palette =
				{
					0xFF0000FF, 0xFF00FF00, 0xFFFF0000, 0xFFFFFF00, 0xFFFF00FF,
					0xFF00FFFF, 0xFF8080FF, 0xFF80FF80, 0xFFFF8080,
				};
			}
		}
		#endregion
	}
}
