using System;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.Text.RegularExpressions;
using Rylogic.Common;

namespace RyLogViewer
{
	/// <summary>A text transform that matches a pattern and produces replacement text</summary>
	public class Transform : Pattern
	{
		public Transform()
			: this(EPattern.RegularExpression, string.Empty, string.Empty)
		{ }
		public Transform(EPattern patn_type, string expr, string replace)
			: base(patn_type, expr)
		{
			Replace = replace;
		}
		public Transform(Transform rhs)
			: base(rhs)
		{
			Replace = rhs.Replace;
		}

		/// <summary>The replacement template string</summary>
		public string Replace { get; set; }

		/// <summary>Apply the transform to text, returning the result</summary>
		public string Txfm(string text)
		{
			if (!Active || !IsValid) return text;
			try
			{
				return Regex.Replace(text, Replace);
			}
			catch
			{
				return text;
			}
		}

		public override string ToString()
		{
			return $"{Expr} → {Replace}";
		}
	}

	/// <summary>Observable collection of transforms with change notification</summary>
	public class TransformContainer : ObservableCollection<Transform>
	{
		/// <summary>Raised when transforms are added, removed, or modified</summary>
		public event EventHandler? TransformsChanged;

		/// <inheritdoc/>
		protected override void OnCollectionChanged(NotifyCollectionChangedEventArgs e)
		{
			base.OnCollectionChanged(e);
			TransformsChanged?.Invoke(this, EventArgs.Empty);
		}

		/// <summary>Notify that a transform was edited in-place</summary>
		public void NotifyItemChanged()
		{
			TransformsChanged?.Invoke(this, EventArgs.Empty);
		}
	}
}
