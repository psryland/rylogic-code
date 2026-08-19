using System.Windows;

namespace Rylogic.LDraw;

/// <summary>Provides WPF-specific fluent builder operations for LDraw text.</summary>
public static class LdrTextWpfExtensions
{
	/// <summary>Sets text padding from WPF thickness values.</summary>
	public static LdrText padding(this LdrText text, Thickness padding)
	{
		return text.padding((float)padding.Left, (float)padding.Top, (float)padding.Right, (float)padding.Bottom);
	}
}
