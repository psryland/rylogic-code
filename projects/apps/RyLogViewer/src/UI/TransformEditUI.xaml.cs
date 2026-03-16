using System.Windows;
using Rylogic.Common;

namespace RyLogViewer
{
	public partial class TransformEditUI : Window
	{
		public TransformEditUI()
			: this(null)
		{ }
		public TransformEditUI(Transform? existing)
		{
			InitializeComponent();
			Result = null;

			if (existing != null)
			{
				m_type.SelectedIndex = existing.PatnType switch
				{
					EPattern.Wildcard => 1,
					EPattern.RegularExpression => 2,
					_ => 0,
				};
				m_match.Text = existing.Expr;
				m_replace.Text = existing.Replace;
				m_ignore_case.IsChecked = existing.IgnoreCase;
				m_active.IsChecked = existing.Active;
			}
			else
			{
				m_type.SelectedIndex = 2; // Default to regex
			}
		}

		/// <summary>The resulting transform, set when the user clicks OK</summary>
		public Transform? Result { get; private set; }

		private void OK_Click(object sender, RoutedEventArgs e)
		{
			var patn_type = m_type.SelectedIndex switch
			{
				1 => EPattern.Wildcard,
				2 => EPattern.RegularExpression,
				_ => EPattern.Substring,
			};
			Result = new Transform(patn_type, m_match.Text, m_replace.Text)
			{
				IgnoreCase = m_ignore_case.IsChecked == true,
				Active = m_active.IsChecked == true,
			};
			DialogResult = true;
		}
	}
}
