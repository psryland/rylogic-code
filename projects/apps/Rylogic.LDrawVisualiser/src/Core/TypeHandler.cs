namespace Rylogic.LDrawVisualiser.Core;

using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using EnvDTE;
using Microsoft.VisualStudio.Shell;
using Rylogic.LDrawVisualiser.Core.TypeReaders;

/// <summary>
/// Dispatches debug expressions to type-specific readers. Detects the active debug
/// language per call (native C++ vs managed) and routes to readers that support that
/// language. Readers live under TypeReaders/ and implement ITypeReader.
/// </summary>
public class TypeHandler
{
	private readonly Debugger m_debugger;
	private readonly List<ITypeReader> m_readers;
	private readonly ScriptReader m_script_readers;

	public TypeHandler(Debugger debugger)
		: this(debugger, new ExpressionCache())
	{
	}

	public TypeHandler(Debugger debugger, ExpressionCache cache)
	{
		m_debugger = debugger;
		Cache = cache;
		m_script_readers = new ScriptReader();
		m_readers = new List<ITypeReader>
		{
			// Script-registered readers run first so scripts can override built-ins.
			m_script_readers,

			// Scalars
			new SingleReader(),
			new DoubleReader(),
			new ScalarReader(),
			new BoolReader(),
			new StringReader(),

			// Vectors
			new Vec2Reader(),
			new Vec3Reader(),
			new Vec4Reader(),

			// Quaternion
			new QuatReader(),

			// Matrices
			new Mat2x2Reader(),
			new Mat3x3Reader(),
			new Mat4x4Reader(),

			// Bounding box
			new BBoxReader(),

			// Native-only collision shapes
			new ShapeReader(),
		};
		m_script_readers.SetOwner(this);
	}

	/// <summary>Register a script-supplied reader for 'type_name'. See DebugProxy.RegisterReader</summary>
	public void Register(string type_name, Func<dynamic, object?> reader) => m_script_readers.Register(type_name, reader);

	/// <summary>The expression-value cache shared with the owning DebugProxy chain.
	/// Exposed so script-registered readers can spawn child proxies that honour the
	/// same cache state as the parent.</summary>
	public ExpressionCache Cache { get; }

	/// <summary>Read the value of 'expr' (which has reported type 'ty'). Returns null if no reader matched</summary>
	public object? Dispatch(string ty, string expr)
	{
		ThreadHelper.ThrowIfNotOnUIThread();

		var lang = LanguageDetector.Detect(m_debugger);
		switch (lang)
		{
			case ELanguage.Unknown:
			{
				return null;
			}
			case ELanguage.Native:
			case ELanguage.Managed:
			{
				if (TryRead(m_script_readers, lang, ty, expr, out var script_result))
					return script_result;

				ty = NormaliseTypeName(ty);

				// Let exact script readers handle indexed types, but don't let built-ins match
				// an element type substring within a bracketed declaration. Returning null keeps
				// the expression chainable so an indexer can be appended and evaluated.
				if (IsIndexableType(ty))
					return null;

				// Return the result of the first reader that produces a non-null result.
				foreach (var reader in m_readers)
				{
					if (ReferenceEquals(reader, m_script_readers))
						continue;

					if (TryRead(reader, lang, ty, expr, out var result))
						return result;
				}
				return null;
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}

	/// <summary>Try to read an expression using 'reader'</summary>
	private bool TryRead(ITypeReader reader, ELanguage lang, string ty, string expr, out object? result)
	{
		if (!reader.CanRead(lang, ty))
		{
			result = null;
			return false;
		}

		result = reader.Read(m_debugger, lang, ty, expr);
		return result != null;
	}

	/// <summary>Remove CV and refs from a typename</summary>
	private static string NormaliseTypeName(string ty)
	{
		// Const/volatile qualifiers don't change the type from a debugging point of view
		ty = Regex.Replace(ty, @"\b(const|volatile)\b", "").Trim();
		ty = Regex.Replace(ty, @"\s+", " ");

		// Nor do reference qualifiers
		while (ty.EndsWith("&", StringComparison.Ordinal))
			ty = ty.Substring(0, ty.Length - 1).TrimEnd();

		return ty;
	}

	/// <summary>True if the type is an indexable type</summary>
	private static bool IsIndexableType(string type_name)
	{
		return Regex.IsMatch(type_name, @"\[[^\]]*\]");
	}
}
