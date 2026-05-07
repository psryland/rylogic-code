using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using Rylogic.Common;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public static class ComboBox_
	{
		/// <summary>Return the editable text box from within this combo box</summary>
		public static TextBox EditableTextBox(this ComboBox cb)
		{
			return (TextBox)cb.Template.FindName("PART_EditableTextBox", cb);
		}

		/// <summary>Return the editable text box from within this combo box</summary>
		public static Popup DropDownList(this ComboBox cb)
		{
			return (Popup)cb.Template.FindName("PART_Popup", cb);
		}

		/// <summary>Returns a disposable object that preserves the current selection</summary>
		public static Scope<RangeI> SelectionScope(this ComboBox edit)
		{
			return edit.IsEditable
				? edit.EditableTextBox().SelectionScope()
				: Scope.Create(() => RangeI.Invalid, null);
		}

		/// <summary>Global command for combo boxes to update their Text binding</summary>
		public static Command UpdateSource { get; } = Command.Create<object>(null!, UpdateSourceInternal);
		private static void UpdateSourceInternal(object? parameter)
		{
			if (parameter is ComboBox cb && BindingOperations.GetBindingExpression(cb, ComboBox.TextProperty) is BindingExpression binding)
				binding.UpdateSource();
		}
	}
}
