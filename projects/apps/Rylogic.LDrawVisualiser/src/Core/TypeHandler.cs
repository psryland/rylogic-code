namespace Rylogic.LDrawVisualiser.Core;

using System;
using System.Collections.Generic;
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
	{
		m_debugger = debugger;
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
				// Return the result of the first reader that produces a non-null result.
				foreach (var reader in m_readers)
				{
					if (!reader.CanRead(lang, ty))
						continue;

					var result = reader.Read(m_debugger, lang, ty, expr);
					if (result != null)
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
}
