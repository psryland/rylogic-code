using System;
using System.Diagnostics;
using System.Text;
using Rylogic.Extn;
using Rylogic.Maths;
using Rylogic.Str;

namespace Rylogic.Script
{
	[DebuggerDisplay("{Description,nq}")]
	public class Reader :IDisposable
	{
		// Notes:
		//  - The extract functions come in four forms:
		//      Thing Thing();
		//      Thing ThingS();
		//      bool Thing(Thing&);
		//      bool ThingS(Thing&);
		//    All functions read one instance of type 'Thing' from the script.
		//    The postfix 'S' means 'within a section'. e.g. { "value" }
		//    If an error occurs in the first two forms, ReportError is called. If ReportError doesn't throw, then a default instance is returned.
		//    If an error occurs in the second two forms,  ReportError is called. If ReportError doesn't throw, then false is returned.

		public Reader(bool case_sensitive = false, IIncludeHandler? inc = null, IMacroHandler? mac = null, EmbeddedCodeFactory? emb = null)
		{
			Src = new Preprocessor(inc, mac, emb);
			Delimiters = " \t\r\n\v,;";
			LastKeyword = string.Empty;
			CaseSensitive = case_sensitive;
		}
		public Reader(Src src, bool case_sensitive = false, IIncludeHandler? inc = null, IMacroHandler? mac = null, EmbeddedCodeFactory? emb = null)
		{
			Src = new Preprocessor(src, inc, mac, emb);
			Delimiters = " \t\r\n\v,;";
			LastKeyword = string.Empty;
			CaseSensitive = case_sensitive;
		}
		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}
		protected virtual void Dispose(bool _)
		{
			Src.Dispose();
		}

		/// <summary>Access the underlying source</summary>
		public Src Src { get; private set; }

		/// <summary>Return the current source location</summary>
		public Loc Location => Src.Location;

		/// <summary>Access the include handler</summary>
		public IIncludeHandler Includes => Src is Preprocessor pp ? pp.Includes : new NoIncludes();

		/// <summary>Access the macro handler</summary>
		public IMacroHandler Macros => Src is Preprocessor pp ? pp.Macros : new NoMacros();

		// Get/Set delimiter characters
		public string Delimiters { get; set; }

		/// <summary>Get/Set case sensitive keywords on/off</summary>
		public bool CaseSensitive { get; set; }

		/// <summary>The last keyword read</summary>
		public string LastKeyword { get; private set; }

		/// <summary>Return true if the end of the source has been reached</summary>
		public bool IsSourceEnd
		{
			get
			{
				Extract.EatDelimiters(Src, Delimiters);
				return Src == 0;
			}
		}

		/// <summary>Return true if the next token is a keyword</summary>
		public bool IsKeyword
		{
			get
			{
				Extract.EatDelimiters(Src, Delimiters);
				return Src == '*';
			}
		}

		/// <summary>Returns true if the next non-whitespace character is the start/end of a section</summary>
		public bool IsSectionStart
		{
			get
			{
				Extract.EatDelimiters(Src, Delimiters);
				return Src == '{';
			}
		}
		public bool IsSectionEnd
		{
			get
			{
				Extract.EatDelimiters(Src, Delimiters);
				return Src == '}';
			}
		}

		/// <summary>Return true if the next token is not a keyword, the section end, or the end of the source</summary>
		public bool IsValue => !IsKeyword && !IsSectionEnd && !IsSourceEnd;

		/// <summary>Move to the start/end of a section and then one past it</summary>
		public bool SectionStart()
		{
			if (IsSectionStart) { ++Src; return true; }
			return ReportError(EResult.TokenNotFound, Src.Location, "expected '{'");
		}
		public bool SectionEnd()
		{
			if (IsSectionEnd) { ++Src; return true; }
			return ReportError(EResult.TokenNotFound, Src.Location, "expected '}'");
		}

		/// <summary>Move to the start of the next line</summary>
		public bool NewLine()
		{
			Extract.EatLine(Src, 0, 0, true);
			return Src != 0;
		}

		///<summary>
		/// Advance the source to the next '{' within the current scope.
		/// If true is returned, the current position should be a section start character.
		/// If false, then the current position will be '*', '}', or the end of the stream.</summary>
		public bool FindSectionStart()
		{
			for (; Src != 0 && Src != '{' && Src != '}' && Src != '*';)
			{
				if (Src == '\"')
				{
					Extract.EatLiteral(Src);
					continue;
				}
				++Src;
			}
			return Src == '{';
		}

		///<summary>
		/// Advance the source to the end of the current section
		/// On return the current position should be the section end character
		/// or the end of the input stream (if called from file scope).</summary>
		public bool FindSectionEnd()
		{
			for (int nest = IsSectionStart ? 0 : 1; Src != 0;)
			{
				if (Src == '\"') { Extract.EatLiteral(Src); continue; }
				nest += (Src == '{') ? 1 : 0;
				nest -= (Src == '}') ? 1 : 0;
				if (nest == 0) break;
				++Src;
			}
			return Src == '}';
		}

		/// <summary>
		/// Scans forward until a keyword identifier is found within the current scope.
		/// Non-keyword tokens are skipped. If a section is found it is skipped.
		/// If a keyword is found, the source is position at the next character after the keyword
		/// Returns true if a keyword is found, false otherwise.</summary>
		public bool NextKeyword(out string kw)
		{
			kw = string.Empty;
			for (; Src != 0 && Src != '}' && Src != '*';)
			{
				if (Src == '\"') { Extract.EatLiteral(Src); continue; }
				if (Src == '{')  { Src += FindSectionEnd() ? 1 : 0; continue; }
				++Src;
			}
			if (Src == '*') ++Src; else return false;
			if (!Extract.Identifier(out kw, Src, Delimiters)) return false;
			if (!CaseSensitive) kw = kw.ToLowerInvariant();
			LastKeyword = kw;
			return true;
		}
		public bool NextKeyword<TEnum>(out TEnum enum_kw)
			where TEnum : struct, IConvertible
		{
			enum_kw = default!;
			if (!NextKeyword(out var kw))
				return false;

			LastKeyword = kw;
			enum_kw = Enum<TEnum>.Parse(kw);
			return true;
		}
		public string NextKeyword()
		{
			if (!NextKeyword(out var kw))
				ReportError(EResult.TokenNotFound, Location, "keyword expected");

			return kw;
		}
		public TEnum NextKeyword<TEnum>()
			where TEnum : struct, IConvertible
		{
			if (!NextKeyword<TEnum>(out var kw))
				ReportError(EResult.TokenNotFound, Location, "keyword expected");
			
			return kw;
		}

		///<summary>
		/// Scans forward until a keyword matching 'named_kw' is found within the current scope.
		/// Returns false if the named keyword is not found, true if it is.</summary>
		public bool FindKeyword(string named_kw)
		{
			string kw;
			for (; NextKeyword(out kw) && kw != named_kw;) { }
			return named_kw == kw;
		}

		/// <summary>
		/// Scans forward until a keyword matching 'named_kw' is found within the current scope.
		/// Calls ReportError if not found.</summary>
		public Reader Keyword(string named_kw)
		{
			if (!FindKeyword(named_kw)) ReportError(EResult.KeywordNotFound, Location, $"keyword '{named_kw}' expected");
			return this;
		}

		/// <summary>Extract a token from the source. A token is a contiguous block of non-separator characters</summary>
		public string Token()
		{
			return Token(out var token) ? token : string.Empty;
		}
		public string TokenS()
		{
			return TokenS(out var token) ? token : string.Empty;
		}
		public bool Token(out string token)
		{
			return Extract.Token(out token, Src, Delimiters) || ReportError(EResult.TokenNotFound, Location, "token expected");
		}
		public bool TokenS(out string token)
		{
			token = string.Empty;
			return SectionStart() && Token(out token) && SectionEnd();
		}

		/// <summary>Extract a token using additional delimiters</summary>
		public string Token(string delim)
		{
			return Token(out var token, delim) ? token : string.Empty;
		}
		public string TokenS(string delim)
		{
			return TokenS(out var token, delim) ? token : string.Empty;
		}
		public bool Token(out string token, string delim)
		{
			return Extract.Token(out token, Src, $"{Delimiters}{delim}") || ReportError(EResult.TokenNotFound, Location, "token expected");
		}
		public bool TokenS(out string token, string delim)
		{
			token = string.Empty;
			return SectionStart() && Token(out token, delim) && SectionEnd();
		}

		/// <summary>
		/// Read an identifier from the source.
		/// An identifier is one of (A-Z,a-z,'_') followed by (A-Z,a-z,'_',0-9) in a contiguous block.
		/// </summary>
		public string Identifier()
		{
			return Identifier(out var word) ? word : string.Empty;
		}
		public string IdentifierS()
		{
			return IdentifierS(out var word) ? word : string.Empty;
		}
		public bool Identifier(out string word)
		{
			return Extract.Identifier(out word, Src, Delimiters) || ReportError(EResult.TokenNotFound, Location, "identifier expected");
		}
		public bool IdentifierS(out string word)
		{
			word = string.Empty;
			return SectionStart() && Identifier(out word) && SectionEnd();
		}

		/// <summary>
		/// Extract multiple identifiers from the source separated by 'sep'. E.g. House.Room.Item
		/// Remember about deconstructing assignment, e.g. var (x, y, _) = Identifiers('.')</summary>
		public string[] Identifiers(char sep = '.')
		{
			return Identifiers(out var ids, sep) ? ids : Array.Empty<string>();
		}
		public string[] IdentifiersS(char sep = '.')
		{
			return IdentifiersS(out var ids, sep) ? ids : Array.Empty<string>();
		}
		public bool Identifiers(out string[] ids, char sep = '.')
		{
			return Extract.Identifiers(out ids, Src, sep, Delimiters) || ReportError(EResult.TokenNotFound, Location, "identifiers expected");
		}
		public bool IdentifiersS(out string[] ids, char sep = '.')
		{
			ids = Array.Empty<string>();
			return SectionStart() && Identifiers(out ids, sep) && SectionEnd();
		}

		/// <summary>Extract a string from the source. A string is a sequence of characters between quotes.</summary>
		public string String()
		{
			return String(out var str) ? str : string.Empty;
		}
		public string StringS()
		{
			return StringS(out var str) ? str : string.Empty;
		}
		public bool String(out string str)
		{
			if (!Extract.String(out str, Src, Delimiters))
				return ReportError(EResult.TokenNotFound, Location, "string expected");

			str = Str_.ProcessIndentedNewlines(str);
			return true;
		}
		public bool StringS(out string str)
		{
			str = string.Empty;
			return SectionStart() && String(out str) && SectionEnd();
		}

		/// <summary>Extract a C-style string from the source.</summary>
		public string CString()
		{
			return CString(out var cstring) ? cstring : string.Empty;
		}
		public string CStringS()
		{
			return CStringS(out var cstring) ? cstring : string.Empty;
		}
		public bool CString(out string cstring)
		{
			return Extract.String(out cstring, Src, '\\', null, Delimiters) || ReportError(EResult.TokenNotFound, Location, "'cstring' expected");
		}
		public bool CStringS(out string cstring)
		{
			cstring = string.Empty;
			return SectionStart() && CString(out cstring) && SectionEnd();
		}

		/// <summary>Extract a bool from the source.</summary>
		public bool Bool()
		{
			return Bool(out var bool_) ? bool_ : false;
		}
		public bool BoolS()
		{
			return BoolS(out var bool_) ? bool_ : false;
		}
		public bool Bool(out bool bool_)
		{
			return Extract.Bool(out bool_, Src, Delimiters) || ReportError(EResult.TokenNotFound, Location, "bool expected");
		}
		public bool BoolS(out bool bool_)
		{
			bool_ = false;
			return SectionStart() && Bool(out bool_) && SectionEnd();
		}
		public bool Bool(out bool[] bools, int count)
		{
			int i = 0;
			bools = new bool[count];
			for (; i != bools.Length && Bool(out bools[i]); ++i) { }
			return i == bools.Length;
		}
		public bool BoolS(out bool[] bools, int count)
		{
			bools = Array.Empty<bool>();
			return SectionStart() && Bool(out bools, count) && SectionEnd();
		}

		/// <summary>Extract an integral type from the source.</summary>
		public long Int(int radix)
		{
			return Int(out var int_, radix) ? int_ : 0;
		}
		public long IntS(int radix)
		{
			return IntS(out var int_, radix) ? int_ : 0;
		}
		public bool Int(out long int_, int radix)
		{
			return Extract.Int(out int_, radix, Src, Delimiters) || ReportError(EResult.TokenNotFound, Location, "integral value expected");
		}
		public bool IntS(out long int_, int radix)
		{
			int_ = 0;
			return SectionStart() && Int(out int_, radix) && SectionEnd();
		}
		public bool Int(out long[] ints, int count, int radix)
		{
			int i = 0;
			ints = new long[count];
			for (; i != ints.Length && Int(out ints[i], radix); ++i) { }
			return i == ints.Length;
		}
		public bool IntS(out long[] ints, int count, int radix)
		{
			ints = Array.Empty<long>();
			return SectionStart() && Int(out ints, count, radix) && SectionEnd();
		}

		/// <summary>Extract a real from the source.</summary>
		public double Real()
		{
			return Real(out var real_) ? real_ : 0.0;
		}
		public double RealS()
		{
			return RealS(out var real_) ? real_ : 0.0;
		}
		public bool Real(out double real_)
		{
			return Extract.Real(out real_, Src, Delimiters) || ReportError(EResult.TokenNotFound, Location, "real expected");
		}
		public bool RealS(out double real_)
		{
			real_ = 0.0;
			return SectionStart() && Real(out real_) && SectionEnd();
		}
		public bool Real(out double[] reals, int count)
		{
			int i = 0;
			reals = new double[count];
			for (; i != reals.Length && Real(out reals[i]); ++i) { }
			return i == reals.Length;
		}
		public bool RealS(out double[] reals, int count)
		{
			reals = Array.Empty<double>();
			return SectionStart() && Real(out reals, count) && SectionEnd();
		}

		/// <summary>Extract an enum value from the source.</summary>
		public TEnum EnumValue<TEnum>()
			where TEnum : struct, IConvertible
		{
			return EnumValue<TEnum>(out var enum_) ? enum_ : default!;
		}
		public TEnum EnumValueS<TEnum>()
			where TEnum : struct, IConvertible
		{
			return EnumValueS<TEnum>(out var enum_) ? enum_ : default!;
		}
		public bool EnumValue<TEnum>(out TEnum enum_, int radix = 10)
			where TEnum : struct, IConvertible
		{
			return Extract.EnumValue(out enum_, Src, radix, Delimiters) || ReportError(EResult.TokenNotFound, Location, "enum integral value expected");
		}
		public bool EnumValueS<TEnum>(out TEnum enum_, int radix = 10)
			where TEnum : struct, IConvertible
		{
			enum_ = default!;
			return SectionStart() && EnumValue(out enum_, radix) && SectionEnd();
		}

		/// <summary>Extract an enum identifier from the source.</summary>
		public TEnum EnumName<TEnum>()
			where TEnum : struct, IConvertible
		{
			return EnumName<TEnum>(out var enum_) ? enum_ : default!;
		}
		public TEnum EnumNameS<TEnum>()
			where TEnum : struct, IConvertible
		{
			return EnumNameS<TEnum>(out var enum_) ? enum_ : default!;
		}
		public bool EnumName<TEnum>(out TEnum enum_)
			where TEnum : struct, IConvertible
		{
			return Extract.EnumName<TEnum>(out enum_, Src, Delimiters) || ReportError(EResult.TokenNotFound, Location, "enum member string name expected");
		}
		public bool EnumNameS<TEnum>(out TEnum enum_)
			where TEnum : struct, IConvertible
		{
			enum_ = default!;
			return SectionStart() && EnumName(out enum_) && SectionEnd();
		}

		/// <summary>Extract a 2D real vector from the source</summary>
		public v2 Vector2()
		{
			return Vector2(out var vector) ? vector : v2.Zero;
		}
		public v2 Vector2S()
		{
			return Vector2S(out var vector) ? vector : v2.Zero;
		}
		public bool Vector2(out v2 vector)
		{
			vector = v2.Zero;
			if (Real(out var x)) vector.x = (float)x;
			if (Real(out var y)) vector.y = (float)y;
			return true;
		}
		public bool Vector2S(out v2 vector)
		{
			vector = v2.Zero;
			return SectionStart() && Vector2(out vector) && SectionEnd();
		}

		/// <summary>Extract a 3D real vector from the source</summary>
		public v4 Vector3(float w)
		{
			return Vector3(out var vector, w) ? vector : new v4(0,0,0,w);
		}
		public v4 Vector3S(float w)
		{
			return Vector3S(out var vector, w) ? vector : new v4(0,0,0,w);
		}
		public bool Vector3(out v4 vector, float w)
		{
			vector = v4.Zero;
			if (Real(out var x)) vector.x = (float)x;
			if (Real(out var y)) vector.y = (float)y;
			if (Real(out var z)) vector.z = (float)z;
			vector.w = w;
			return true;
		}
		public bool Vector3S(out v4 vector, float w)
		{
			vector = v4.Zero;
			return SectionStart() && Vector3(out vector, w) && SectionEnd();
		}

		/// <summary>Extract a 4D real vector from the source</summary>
		public v4 Vector4()
		{
			return Vector4(out var vector) ? vector : v4.Zero;
		}
		public v4 Vector4S()
		{
			return Vector4S(out var vector) ? vector : v4.Zero;
		}
		public bool Vector4(out v4 vector)
		{
			vector = v4.Zero;
			if (Real(out var x)) vector.x = (float)x;
			if (Real(out var y)) vector.y = (float)y;
			if (Real(out var z)) vector.z = (float)z;
			if (Real(out var w)) vector.w = (float)w;
			return true;
		}
		public bool Vector4S(out v4 vector)
		{
			vector = v4.Zero;
			return SectionStart() && Vector4(out vector) && SectionEnd();
		}

		/// <summary>Extract a quaternion from the source</summary>
		public Quat Quaternion()
		{
			return Quaternion(out var quaternion) ? quaternion : Quat.Identity;
		}
		public Quat QuaternionS()
		{
			return QuaternionS(out var quaternion) ? quaternion : Quat.Identity;
		}
		public bool Quaternion(out Quat quaternion)
		{
			quaternion = Quat.Identity;
			if (Real(out var x)) quaternion.x = (float)x;
			if (Real(out var y)) quaternion.y = (float)y;
			if (Real(out var z)) quaternion.z = (float)z;
			if (Real(out var w)) quaternion.w = (float)w;
			return true;
		}
		public bool QuaternionS(out Quat quaternion)
		{
			quaternion = Quat.Identity;
			return SectionStart() && Quaternion(out quaternion) && SectionEnd();
		}

		/// <summary>Extract a 3x3 matrix from the source</summary>
		public m3x3 Matrix3x3()
		{
			return Matrix3x3(out var transform) ? transform : m3x3.Identity;
		}
		public m3x3 Matrix3x3S()
		{
			return Matrix3x3S(out var transform) ? transform : m3x3.Identity;
		}
		public bool Matrix3x3(out m3x3 transform)
		{
			transform = m3x3.Identity;
			v4 rx, ry, rz;
			if (!Vector3(out rx, 0) || !Vector3(out ry, 0) || !Vector3(out rz, 0))
				return false;
			transform.x = rx.xyz;
			transform.y = ry.xyz;
			transform.z = rz.xyz;
			return true;
		}
		public bool Matrix3x3S(out m3x3 transform)
		{
			transform = m3x3.Identity;
			return SectionStart() && Matrix3x3(out transform) && SectionEnd();
		}

		/// <summary>Extract a 4x4 matrix from the source</summary>
		public m4x4 Matrix4x4()
		{
			return Matrix4x4(out var transform) ? transform : m4x4.Identity;
		}
		public m4x4 Matrix4x4S()
		{
			return Matrix4x4S(out var transform) ? transform : m4x4.Identity;
		}
		public bool Matrix4x4(out m4x4 transform)
		{
			transform = m4x4.Identity;
			return
				Vector4(out transform.x) &&
				Vector4(out transform.y) &&
				Vector4(out transform.z) && 
				Vector4(out transform.w);
		}
		public bool Matrix4x4S(out m4x4 transform)
		{
			transform = m4x4.Identity;
			return SectionStart() && Matrix4x4(out transform) && SectionEnd();
		}

		/// <summary>Extract a byte array</summary>
		public byte[] Data(int length, int radix = 16)
		{
			var data = new byte[length];
			return Data(data, 0, length, radix) ? data : Array.Empty<byte>();
		}
		public byte[] DataS(int length, int radix = 16)
		{
			var data = new byte[length];
			return DataS(data, 0, length, radix) ? data : Array.Empty<byte>();
		}
		public bool Data(out byte[] data, int count, int radix = 16)
		{
			data = new byte[count];
			return Data(data, 0, data.Length, radix);
		}
		public bool DataS(out byte[] data, int count, int radix = 16)
		{
			data = Array.Empty<byte>();
			return SectionStart() && Data(out data, count, radix) && SectionEnd();
		}
		public bool Data(byte[] data, int radix = 16)
		{
			return Data(data, 0, data.Length, radix);
		}
		public bool Data(byte[] data, int start, int count, int radix = 16)
		{
			return Extract.Data(data, start, count, Src, radix, Delimiters) || ReportError(EResult.TokenNotFound, Location, "integral value expected");
		}
		public bool DataS(byte[] data, int start = 0, int length = int.MaxValue, int radix = 16)
		{
			return SectionStart() && Data(data, start, length, radix) && SectionEnd();
		}

		/// <summary>Extract a transform description. 'rot' should be a valid initial transform</summary>
		public m3x3 Rotation()
		{
			var rot = m3x3.Identity;
			return Rotation(ref rot) ? rot : m3x3.Identity;
		}
		public m3x3 RotationS()
		{
			var rot = m3x3.Identity;
			return RotationS(ref rot) ? rot : m3x3.Identity;
		}
		public bool Rotation(ref m3x3 rot)
		{
			if (!Math_.IsFinite(rot))
				throw new Exception("A valid matrix must be passed to this function, as it pre-multiplies the transform with the one read from the script");
			
			var o2w = new m4x4(rot, v4.Origin);
			if (!Transform(ref o2w)) return false;
			rot = o2w.rot;
			return true;
		}
		public bool RotationS(ref m3x3 rot)
		{
			if (!Math_.IsFinite(rot))
				throw new Exception("A valid matrix must be passed to this function, as it pre-multiplies the transform with the one read from the script");

			return SectionStart() && Rotation(ref rot) && SectionEnd();
		}

		/// <summary>Extract a transform description accumulatively. 'o2w' should be a valid initial transform.</summary>
		public m4x4 Transform()
		{
			var o2w = m4x4.Identity;
			return Transform(ref o2w) ? o2w : m4x4.Identity;
		}
		public m4x4 TransformS()
		{
			var o2w = m4x4.Identity;
			return TransformS(ref o2w) ? o2w : m4x4.Identity;
		}
		public bool Transform(ref m4x4 o2w)
		{
			if (!Math_.IsFinite(o2w))
				throw new Exception("A valid matrix must be passed to this function, as it pre-multiplies the transform with the one read from the script");

			// Parse the transform
			var p2w = m4x4.Identity;
			for (; NextKeyword(out var kw);)
			{
				switch (kw.ToLower())
				{
				case "m4x4":
					{
						var m = Matrix4x4S();
						if (m.w.w != 1)
						{
							ReportError(EResult.UnknownValue, Location, "M4x4 must be an affine transform with: w.w == 1");
							break;
						}
						p2w = m * p2w;
						break;
					}
				case "m3x3":
					{
						var m = Matrix3x3S();
						p2w = new m4x4(m, v4.Origin) * p2w;
						break;
					}
				case "pos":
					{
						var pos = Vector3S(1.0f);
						p2w = new m4x4(m3x3.Identity, pos) * p2w;
						break;
					}
				case "align":
					{
						SectionStart();
						var axis_id = Int(10);
						var direction = Vector3(0.0f);
						SectionEnd();

						if (!AxisId.Try((int)axis_id, out var axis))
						{
							ReportError(EResult.UnknownValue, Location, "axis_id must one of ±1, ±2, ±3");
							break;
						}

						p2w = m4x4.Transform(axis, direction, v4.Origin) * p2w;
						break;
					}
				case "quat":
					{
						var q = new Quat(Vector4S());
						p2w = m4x4.Transform(q, v4.Origin) * p2w;
						break;
					}
				case "quatpos":
					{
						SectionStart();
						var q = new Quat(Vector4());
						var p = Vector3(1.0f);
						SectionEnd();
						p2w = m4x4.Transform(q, p) * p2w;
						break;
					}
				case "rand4x4":
					{
						SectionStart();
						var centre = Vector3(1.0f);
						var radius = (float)Real();
						SectionEnd();
						p2w = m4x4.Random(centre, radius, m_rng) * p2w;
						break;
					}
				case "randpos":
					{
						SectionStart();
						var centre = Vector3(1.0f);
						var radius = (float)Real();
						SectionEnd();
						p2w = m4x4.Translation(v4.Random3(centre, radius, 1.0f, m_rng)) * p2w;
						break;
					}
				case "randori":
					{
						var rot = m3x3.Random(m_rng);
						p2w = new m4x4(rot, v4.Origin) * p2w;
						break;
					}
				case "euler":
					{
						var angles = Vector3S(0.0f);
						p2w = m4x4.TransformDeg(angles.x, angles.y, angles.z, v4.Origin) * p2w;
						break;
					}
				case "scale":
					{
						var scale = v4.One;
						SectionStart();
						scale.x = (float)Real();
						if (IsSectionEnd)
						{
							scale.y = scale.x;
							scale.z = scale.x;
						}
						else
						{
							scale.y = (float)Real();
							scale.z = (float)Real();
						}
						SectionEnd();
						p2w = m4x4.Scale(scale.x, scale.y, scale.z, v4.Origin) * p2w;
						break;
					}
				case "transpose":
					{
						p2w = Math_.Transpose(p2w);
						break;
					}
				case "inverse":
					{
						p2w = Math_.IsOrthonormal(p2w) ? Math_.InvertAffine(p2w) : Math_.Invert(p2w);
						break;
					}
				case "normalise":
					{
						p2w.x = Math_.Normalise(p2w.x);
						p2w.y = Math_.Normalise(p2w.y);
						p2w.z = Math_.Normalise(p2w.z);
						break;
					}
				case "orthonormalise":
					{
						p2w = Math_.Orthonormalise(p2w);
						break;
					}
				default:
					{
						ReportError(EResult.UnknownToken, Location, $"Token '{kw}' is not valid within an *o2w section");
						break;
					}
				}
			}

			// Pre-multiply the object to world transform
			o2w = p2w * o2w;
			return true;
		}
		public bool TransformS(ref m4x4 o2w)
		{
			return SectionStart() && Transform(ref o2w) && SectionEnd();
		}

		/// <summary>Extract a complete section as a preprocessed string. Note: To embed arbitrary text in a script use #lit/#end and then Section()</summary>
		public string Section(bool include_braces)
		{
			return Section(out var s, include_braces) ? s : string.Empty;
		}
		public bool Section(out string str, bool include_braces)
		{
			str = string.Empty;
			var len = 0;

			// Consume whitespace up to the section start
			if (!IsSectionStart)
				ReportError(EResult.TokenNotFound, Location, "Expected a section");

			// Buffer the '{'
			if (Src[len] == '{') ++len;
			else
			{
				ReportError(EResult.TokenNotFound, Location, "Expected '{'");
				return false;
			}

			// Buffer the section in 'Src'
			var lit = new InLiteral();
			for (int nest = 1; Src[len] != 0;)
			{
				// If we're in a string/character literal, then ignore any '{''}' characters
				if (lit.WithinLiteral(Src[len]))
				{
					++len;
					continue;
				}
				nest += Src[len] == '{' ? 1 : 0;
				nest -= Src[len] == '}' ? 1 : 0;
				if (nest == 0) break;
				++len;
			}

			// Buffer the '}'
			if (Src[len] == '}') ++len;
			else
			{
				ReportError(EResult.TokenNotFound, Location, "Expected '}'");
				return false;
			}

			// Return the section with or without braces
			str = include_braces
				? Src.Buffer.ToString(0, len)
				: Src.Buffer.ToString(1, len-2);
			Src += len;
			return true;
		}

		/// <summary>Configurable error handling</summary>
		public Func<EResult, Loc, string, bool> ReportError { get; set; } = DefaultErrorHandler;
		public static bool DefaultErrorHandler(EResult result, Loc loc, string msg) => throw new ScriptException(result, loc, msg);

		/// <summary></summary>
		public string Description => Src.Description;

		/// <summary>Random number source</summary>
		private static readonly Random m_rng = new((int)(Stopwatch.GetTimestamp() & 0x7FFFFFFF));
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Script;

	[TestFixture]
	public partial class TestScriptReader
	{
		[Test]
		public void BasicExtract()
		{
			const string src =
				"*Identifier ident\n" +
				"*String \"simple string\"\n" +
				"*CString \"C:\\\\Path\\\\Filename.txt\"\n" +
				"*Bool true\n" +
				"*Intg -23\n" +
				"*Intg16 ABCDEF00\n" +
				"*Real -2.3e+3\n" +
				"*BoolArray 1 0 true false\n" +
				"*IntArray -3 2 +1 -0\n" +
				"*RealArray 2.3 -1.0e-1 2 -0.2\n" +
				"*Vector3 1.0 2.0 3.0\n" +
				"*Vector4 4.0 3.0 2.0 1.0\n" +
				"*Quaternion 0.0 -1.0 -2.0 -3.0\n" +
				"*M3x3 1.0 0.0 0.0  0.0 1.0 0.0  0.0 0.0 1.0\n" +
				"*M4x4 1.0 0.0 0.0 0.0  0.0 1.0 0.0 0.0  0.0 0.0 1.0 0.0  0.0 0.0 0.0 1.0\n" +
				"*Data 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F 00\n" +
				"*Junk\n" +
				"*Section {*SubSection { *Data \n NUM \"With a }\\\"string\\\"{ in it\" }}    \n" +
				"*Section {*SubSection { *Data \n NUM \"With a }\\\"string\\\"{ in it\" }}    \n" +
				"*Token 123token\n" +
				"*LastThing";

			using var reader = new Reader(new StringSrc(src), true);
			Assert.Equal(true, reader.CaseSensitive);
			Assert.True(reader.NextKeyword(out var kw)); Assert.Equal("Identifier", kw);
			Assert.True(reader.Identifier(out var ident)); Assert.Equal("ident", ident);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("String", kw);
			Assert.True(reader.String(out var str0)); Assert.Equal("simple string", str0);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("CString", kw);
			Assert.True(reader.CString(out var str1)); Assert.Equal("C:\\Path\\Filename.txt", str1);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("Bool", kw);
			Assert.True(reader.Bool(out var bool0)); Assert.Equal(true, bool0);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("Intg", kw);
			Assert.True(reader.Int(out var int0, 10)); Assert.Equal(-23L, int0);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("Intg16", kw);
			Assert.True(reader.Int(out var int1, 16)); Assert.Equal(0xABCDEF00L, int1);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("Real", kw);
			Assert.True(reader.Real(out var real0)); Assert.Equal(-2.3e+3, real0);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("BoolArray", kw);
			Assert.True(reader.Bool(out var bools, 4)); Assert.Equal(new[] { true, false, true, false }, bools);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("IntArray", kw);
			Assert.True(reader.Int(out var ints, 4, 10)); Assert.Equal(new[] { -3L, +2L, +1L, -0L }, ints);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("RealArray", kw);
			Assert.True(reader.Real(out var reals, 4)); Assert.Equal(new[] { 2.3, -1.0e-1, +2.0, -0.2 }, reals);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("Vector3", kw);
			Assert.True(reader.Vector3(out var vec0, -1.0f)); Assert.Equal(new v4(1.0f, 2.0f, 3.0f, -1.0f), vec0);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("Vector4", kw);
			Assert.True(reader.Vector4(out var vec1)); Assert.Equal(new v4(4.0f, 3.0f, 2.0f, 1.0f), vec1);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("Quaternion", kw);
			Assert.True(reader.Quaternion(out var quat0)); Assert.Equal(new Quat(0.0f, -1.0f, -2.0f, -3.0f), quat0);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("M3x3", kw);
			Assert.True(reader.Matrix3x3(out var mat0)); Assert.Equal(m3x3.Identity, mat0);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("M4x4", kw);
			Assert.True(reader.Matrix4x4(out var mat1)); Assert.Equal(m4x4.Identity, mat1);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("Data", kw);
			Assert.True(reader.Data(out var data0, 16)); Assert.Equal("ABCDEFGHIJKLMNO\0", Encoding.UTF8.GetString(data0));
			Assert.True(reader.FindKeyword("Section"));
			Assert.True(reader.Section(out var str2, false)); Assert.Equal("*SubSection { *Data \n NUM \"With a }\\\"string\\\"{ in it\" }", str2);
			Assert.True(reader.FindKeyword("Section"));
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("Token", kw);
			Assert.True(reader.Token(out var str3)); Assert.Equal("123token", str3);
			Assert.True(reader.NextKeyword(out kw)); Assert.Equal("LastThing", kw);
			Assert.True(!reader.IsKeyword);
			Assert.True(!reader.IsSectionStart);
			Assert.True(!reader.IsSectionEnd);
			Assert.True(reader.IsSourceEnd);
		}
		[Test]
		public void Identifiers()
		{
			const string src =
				"A.B\n" +
				"a.b.c\n" +
				"A.B.C.D\n";

			using var reader = new Reader(new StringSrc(src));
			{
				var (a, b, _) = reader.Identifiers();
				Assert.Equal("A", a);
				Assert.Equal("B", b);
			}
			{
				var (a, b, c, _) = reader.Identifiers();
				Assert.Equal("a", a);
				Assert.Equal("b", b);
				Assert.Equal("c", c);
			}
			{
				var (a, b, c, d, _) = reader.Identifiers();
				Assert.Equal("A", a);
				Assert.Equal("B", b);
				Assert.Equal("C", c);
				Assert.Equal("D", d);
			}
		}
	}
}
#endif
