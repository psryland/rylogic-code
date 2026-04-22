namespace Rylogic.LDrawVisualiser.Core;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using EnvDTE;
using Rylogic.Maths;

public class TypeHandler
{
	private readonly Debugger m_debugger;
	private readonly List<Pattern> m_patterns;

	[System.Diagnostics.DebuggerDisplay("{Regex}")]
	private class Pattern
	{
		public readonly Regex Regex;
		public readonly Func<string, string, object?> Handler;

		public Pattern(Regex regex, Func<string, string, object?> handler)
		{
			Regex = regex;
			Handler = handler;
		}
	}

	public TypeHandler(Debugger debugger)
	{
		m_debugger = debugger;
		m_patterns = [
			new Pattern(new Regex("^int$"), HandleScalar),
			new Pattern(new Regex("^float$"), HandleScalar),
			new Pattern(new Regex(@"^.*pr::math::Vec4<float>.*"), HandleVec4),
			new Pattern(new Regex(@"^.*pr::math::Mat4x4<float>.*"), HandleMat4x4),
			new Pattern(new Regex(@"^.*pr::collision::Shape.*$"), HandleShape),
		];
	}

	/// <summary>Convert a type to it's value</summary>
	public object? Dispatch(string ty, string expr)
	{
		// Return the result of the first match that produces a non-null result
		var patterns = m_patterns.Where(x => x.Regex.IsMatch(ty)).ToArray();
		foreach (var pattern in patterns)
		{
			var result = pattern.Handler(ty, expr);
			if (result != null)
				return result;
		}
		return null;
	}

	/// <summary>Handle a scalar type</summary>
	private object? HandleScalar(string ty, string expr)
	{
		switch (ty)
		{
			case "int":
			{
				if (DebugMemoryReader.ReadBytes(m_debugger, expr, 4) is not byte[] data) return null;
				return BitConverter.ToInt32(data, 0);
			}
			case "float":
			{
				if (DebugMemoryReader.ReadBytes(m_debugger, expr, 4) is not byte[] data) return null;
				return BitConverter.ToSingle(data, 0);
			}
		}

		return null;
	}

	/// <summary>Read a native pr::math::Vec4 from memory</summary>
	private object? HandleVec4(string ty, string expr)
	{
		if (DebugMemoryReader.ReadBytes(m_debugger, expr, 4 * 4) is not byte[] data)
			return null;

		var vec = new float[4];
		for (int i = 0; i < 4; i++)
			vec[i] = BitConverter.ToSingle(data, i * 4);

		return new v4(vec);
	}

	/// <summary>Read a native pr::math::Mat4x4 from memory</summary>
	private object? HandleMat4x4(string ty, string expr)
	{
		if (DebugMemoryReader.ReadBytes(m_debugger, expr, 16 * 4) is not byte[] data)
			return null;

		var mat = new float[16];
		for (int i = 0; i < 16; i++)
			mat[i] = BitConverter.ToSingle(data, i * 4);

		return new m4x4(mat);
	}

	/// <summary>Check if a native type name is a collision shape type</summary>
	private object? HandleShape(string ty, string expr)
	{
		// Auto-detect collision shape types and read raw memory
		if (DebugMemoryReader.ReadShapeBytes(m_debugger, expr) is not byte[] data)
			return null;

		return data;
	}
}
