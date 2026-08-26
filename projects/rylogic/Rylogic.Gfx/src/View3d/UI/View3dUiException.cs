using System;

namespace Rylogic.Gfx.UI;

/// <summary>An error reported by the native View3DUI runtime, carrying the stable status category and native diagnostic message.</summary>
public sealed class View3dUiException : Exception
{
	/// <summary>Create an exception for one native status and message.</summary>
	internal View3dUiException(EStatus status, string message)
		: base(message)
	{
		Status = status;
	}

	/// <summary>The stable native status code.</summary>
	public EStatus Status { get; }
}
