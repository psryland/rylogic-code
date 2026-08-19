//***************************************************
// Control Extensions
//  Copyright (c) Rylogic Ltd 2010
//***************************************************

using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Reflection;
using System.Windows.Forms;
using System.Windows.Threading;
using System.Xml.Linq;
using Rylogic.Attrib;
using Rylogic.Common;
using Rylogic.Container;
using Rylogic.Extn;
using Rylogic.Interop.Win32;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WinForms
{
	public static class Gui_
	{
		/// <summary>Show the folder browser dialog</summary>
		public static DialogResult ShowDialog(this Windows.OpenFolderUI dlg, IWin32Window owner)
		{
			return dlg.ShowDialog(owner.Handle) ? DialogResult.OK : DialogResult.Cancel ;
		}
	}

	/// <summary>General control extensions</summary>
	public static class Control_
	{
		/// <summary>Singleton storing user data associated with a control/GUID</summary>
		[ThreadStatic]
		private static LazyDictionary<Control, Dictionary<Guid, object?>> m_ctrl_user_data = new();

		/// <summary>Get the user data for this control associated with 'guid'. If not found, and 'make' != null then 'make' is called with the result added and returned</summary>
		public static T? UserData<T>(this Control ctrl, Guid guid, Func<T?>? factory = null)
		{
			// Look for existing user data associated with 'guid'
			var map = m_ctrl_user_data[ctrl];
			if (map.TryGetValue(guid, out var user_data))
				return (T?)user_data;

			// If not found, but a construction delegate is provided, create the user
			// data, add it to the map, and return it
			if (factory is not null)
			{
				var new_user_data = factory();
				map.Add(guid, new_user_data);
				return (T?)new_user_data;
			}

			return default(T);
		}

		/// <summary>Set some user data for this control. Setting 'data' == null removes the user data associated with 'guid'</summary>
		public static void UserData<T>(this Control ctrl, Guid guid, T data)
		{
			if (data != null)
			{
				var map = m_ctrl_user_data[ctrl];
				map[guid] = data;
			}
			else
			{
				if (m_ctrl_user_data.TryGetValue(ctrl, out var map))
				{
					map.Remove(guid);
					if (map.Count == 0)
						m_ctrl_user_data.Remove(ctrl);
				}
			}
		}

		/// <summary>Tests if this component is being used by the VS designer</summary>
		public static bool IsInDesignMode<TComponent>(this TComponent c) where TComponent:Component
		{
			if (LicenseManager.UsageMode == LicenseUsageMode.Designtime)
				return true;

			// Test the protected DesignMode component property
			if ((bool)DesignModeProp.GetValue(c, null))
				return true;

			// 'DesignMode' doesn't work in constructors or grandchild controls... test recursively
			var ctrl = c as Control;
			if (ctrl != null && ctrl.Parent != null && ctrl.Parent.IsInDesignMode())
				return true;

			return false;
		}
		private static PropertyInfo DesignModeProp
		{
			get
			{
				if (m_impl_design_mode_prop == null)
				{
					m_impl_design_mode_prop = typeof(Component).GetProperty("DesignMode", BindingFlags.NonPublic|BindingFlags.Instance);
					if (m_impl_design_mode_prop == null) throw new Exception("Component.DesignMode property missing");
				}
				return m_impl_design_mode_prop;
			}
		}
		private static PropertyInfo m_impl_design_mode_prop = null!;

		/// <summary>Set control style on this control</summary>
		public static void SetStyle(this Control ctrl, ControlStyles style, bool state)
		{
			var mi = ctrl.GetType().GetMethod("SetStyle", BindingFlags.Instance|BindingFlags.NonPublic);
			mi.Invoke(ctrl, new object[]{style, state});
		}

		/// <summary>Wrapper of begin invoke that takes a lambda</summary>
		[Obsolete("Use Dispatcher instead. It doesn't require a window handle")]
		public static IAsyncResult BeginInvoke(this Form form, Action action)
		{
			// Has to be form or the overload isn't found
			// Note, the params overload isn't needed because 'action' is a lambda
			return form.BeginInvoke(action);
		}
		[Obsolete("Use Dispatcher instead. It doesn't require a window handle")]
		public static IAsyncResult BeginInvoke(this Control ctrl, Action action)
		{
			// Note, the params overload isn't needed because 'action' is a lambda
			return ctrl.BeginInvoke(action);
		}

		/// <summary>Invoke that takes a lambda</summary>
		[Obsolete("Use Dispatcher instead. It doesn't require a window handle")]
		public static object Invoke(this Form form, Action action)
		{
			// Has to be form or the overload isn't found
			// Note, the params overload isn't needed because 'action' is a lambda
			return form.Invoke(action);
		}
		[Obsolete("Use Dispatcher instead. It doesn't require a window handle")]
		public static object Invoke(this Control ctrl, Action action)
		{
			// Note, the params overload isn't needed because 'action' is a lambda
			return ctrl.Invoke(action);
		}

		/// <summary>Enumerators the parents of this control.</summary>
		public static IEnumerable<Control> Parents(this Control ctrl, bool leaf_to_root = true)
		{
			if (leaf_to_root)
			{
				for (var p = ctrl.Parent; p != null; p = p.Parent)
					yield return p;
			}
			else
			{
				var p = ctrl.Parents(leaf_to_root: true).ToArray();
				for (var i = p.Length; i-- != 0;)
					yield return p[i];
			}
		}

		/// <summary>Search up the control hierarchy for the first parent of type 'TParent'</summary>
		public static TParent FindParentOfType<TParent>(this Control ctrl)
		{
			return ctrl.Parents(leaf_to_root:true).OfType<TParent>().FirstOrDefault();
		}

		/// <summary>Add to the controls collection and return the control for method chaining</summary>
		public static T Add2<T>(this Control.ControlCollection collection, T ctrl) where T:Control
		{
			collection.Add(ctrl);
			return ctrl;
		}

		/// <summary>Get the tool tip associated with this control</summary>
		public static string ToolTip(this Control ctrl, ToolTip tt)
		{
			return tt.GetToolTip(ctrl);
		}

		/// <summary>Set the tool tip for this control</summary>
		public static void ToolTip(this Control ctrl, ToolTip tt, string caption)
		{
			tt.SetToolTip(ctrl, caption);
		}

		/// <summary>Display a hint balloon. Note, is difficult to get working, use HintBalloon instead</summary>
		public static void ShowHintBalloon(this Control ctrl, ToolTip tt, string msg, int duration = 5000, Point? pt = null)
		{
			// Only show the tool tip if not currently showing or it's different
			var prev = tt.GetToolTip(ctrl);
			if (prev != msg)
			{
				// Show the tool tip
				tt.Show(msg, ctrl, pt ?? ctrl.ClientRectangle.Centre(), duration);

				// Set the tool tip for the control so we can detect expired
				tt.SetToolTip(ctrl, msg);
				SynchronizationContext_.BeginInvokeDelayed(() =>
				{
					if (tt.GetToolTip(ctrl) == msg)
						tt.SetToolTip(ctrl, null);
				}, TimeSpan.FromMilliseconds(duration));
			}
		}

		/// <summary>Map a point from 'src' control's client space, to 'dst' control's client space</summary>
		public static Point MapPoint(Control src, Control dst, Point pt)
		{
			pt = src != null ? src.PointToScreen(pt) : pt;
			pt = dst != null ? dst.PointToClient(pt) : pt;
			return pt;
		}

		/// <summary>Map a rectangle from 'src' control's client space, to 'dst' control's client space</summary>
		public static Rectangle MapRectangle(Control src, Control dst, Rectangle rect)
		{
			rect = src != null ? src.RectangleToScreen(rect) : rect;
			rect = dst != null ? dst.RectangleToClient(rect) : rect;
			return rect;
		}

		/// <summary>Returns the location of this item in screen space</summary>
		public static Point ScreenLocation(this Control item)
		{
			var parent = item.Parent;
			if (parent == null) return item.Bounds.Location;
			return parent.PointToScreen(item.Bounds.Location);
		}

		/// <summary>Returns the bounds of this item in form space</summary>
		public static Rectangle ParentFormRectangle(this Control item)
		{
			var parent = item.Parent;
			var top    = item.TopLevelControl;
			var srect  = parent == null ? item.Bounds : parent.RectangleToScreen(item.Bounds);
			return top != null ? top.RectangleToClient(srect) : srect;
		}

		/// <summary>Computes the location of the specified client point into parent coordinates</summary>
		public static Point PointToParent(this Control item, Point point)
		{
			var pt = item.PointToScreen(point);
			return item.Parent != null ? item.Parent.PointToClient(pt) : pt;
		}

		/// <summary>Returns the bounds of this item in screen space</summary>
		public static Rectangle ScreenRectangle(this Control item)
		{
			var parent = item.Parent;
			if (parent == null) return item.Bounds;
			return parent.RectangleToScreen(item.Bounds);
		}

		/// <summary>BeginInvoke an action after 'delay' milliseconds (roughly)</summary>
		[Obsolete("Use SynchronizationContext_.BeginInvokeDelayed")] public static void BeginInvokeDelayed(this IComponent ctrl, int delay, Action action)
		{
			new Timer{Enabled = true, Interval = Math.Max(delay,1)}.Tick += (s,a) =>
			{
				var timer = (Timer)s;
				timer.Enabled = false;
				timer.Dispose();
				action();
			};
		}

		/// <summary>Enable double buffering for the control. Note, it's probably better to subclass the control to turn this on</summary>
		public static void DoubleBuffered(this Control ctrl, bool state)
		{
			PropertyInfo pi = ctrl.GetType().GetProperty("DoubleBuffered", BindingFlags.Instance|BindingFlags.NonPublic);
			pi.SetValue(ctrl, state, null);
			MethodInfo mi = ctrl.GetType().GetMethod("SetStyle", BindingFlags.Instance|BindingFlags.NonPublic);
			mi.Invoke(ctrl, new object[]{ControlStyles.DoubleBuffer|ControlStyles.UserPaint|ControlStyles.AllPaintingInWmPaint, state});
		}

		/// <summary>Returns an RAII scope for suspending layout</summary>
		public static Scope SuspendLayout(this Control ctrl, bool layout_on_resume)
		{
			return Scope.Create(ctrl.SuspendLayout, () => ctrl.ResumeLayout(layout_on_resume));
		}

		/// <summary>Yet another window update blocker</summary>
		public static Scope SuspendWindowUpdate(this Control ctrl)
		{
			return ctrl.Handle == IntPtr.Zero ? new Scope { } :
				Scope.Create(
				() => User32.LockWindowUpdate(ctrl.Handle),
				() => User32.LockWindowUpdate(IntPtr.Zero));
		}

		/// <summary>Block redrawing of the control</summary>
		public static Scope SuspendRedraw(this IWin32Window ctrl, bool refresh_on_resume)
		{
			return ctrl.Handle == IntPtr.Zero ? new Scope { } :
				Scope.Create(
				() => User32.SendMessage(ctrl.Handle, Win32.WM_SETREDRAW, 0, 0),
				() =>
					{
						User32.SendMessage(ctrl.Handle, Win32.WM_SETREDRAW, 1, 0);
						if (refresh_on_resume)
							User32.RedrawWindow(ctrl.Handle, IntPtr.Zero, IntPtr.Zero, (uint)(Win32.RDW_ERASE | Win32.RDW_FRAME | Win32.RDW_INVALIDATE | Win32.RDW_ALLCHILDREN));
					});
		}

		/// <summary>Block scrolling events</summary>
		public static Scope SuspendScroll(this IWin32Window ctrl)
		{
			var scroll_pos = new Win32.POINT();
			var event_mask = 0;
			return ctrl.Handle == IntPtr.Zero ? new Scope { } :
				Scope.Create(
				() =>
					{
						User32.SendMessage(ctrl.Handle, Win32.EM_GETSCROLLPOS, (IntPtr)0, ref scroll_pos);
						event_mask = (int)User32.SendMessage(ctrl.Handle, Win32.EM_GETEVENTMASK, 0, 0);
					},
				() =>
					{
						User32.SendMessage(ctrl.Handle, Win32.EM_SETSCROLLPOS, (IntPtr)0, ref scroll_pos);
						User32.SendMessage(ctrl.Handle, Win32.EM_SETEVENTMASK, 0, event_mask);
					});
		}

		/// <summary>Recursively calls 'GetChildAtPoint' until the control with no children at that point is found</summary>
		public static Control GetChildAtScreenPointRec(this Control ctrl, Point screen_pt, GetChildAtPointSkip flags)
		{
			Control parent = null!;
			for (var child = ctrl; child != null;)
			{
				parent = child;
				child = parent.GetChildAtPoint(parent.PointToClient(screen_pt), flags);
			}
			return parent;
		}

		/// <summary>Recursively calls 'GetChildAtPoint' until the control with no children at that point is found</summary>
		public static Control GetChildAtPointRec(this Control ctrl, Point pt, GetChildAtPointSkip flags)
		{
			return GetChildAtScreenPointRec(ctrl, ctrl.PointToScreen(pt), flags);
		}

		/// <summary>Returns true if 'ClickThru' mode is enabled for this control</summary>
		public static bool ClickThru(this Control ctrl)
		{
			var style = User32.GetWindowLong(ctrl.Handle, Win32.GWL_EXSTYLE);
			return (style & Win32.WS_EX_TRANSPARENT) != 0;
		}

		/// <summary>Enable/Disable 'ClickThru' mode</summary>
		public static void ClickThru(this Control ctrl, bool enabled)
		{
			var style = User32.GetWindowLong(ctrl.Handle, Win32.GWL_EXSTYLE);
			User32.SetWindowLong(ctrl.Handle, Win32.GWL_EXSTYLE, enabled
				? Bit.SetBits(style, Win32.WS_EX_TRANSPARENT, true)
				: Bit.SetBits(style, Win32.WS_EX_TRANSPARENT, false));
		}

		/// <summary>Temporarily enabled/disable 'ClickThru' mode</summary>
		public static Scope ClickThruScope(this Control ctrl, bool enabled)
		{
			return Scope.Create(
				() => { var on = ctrl.ClickThru(); ctrl.ClickThru(enabled); return on; },
				on => { ctrl.ClickThru(on); });
		}

		/// <summary>Return a bitmap of this control</summary>
		public static Bitmap ToBitmap(this Control control)
		{
			var bm = new Bitmap(control.Width, control.Height);
			control.DrawToBitmap(bm, bm.Size.ToRect());
			return bm;
		}

		/// <summary>Exports location data for this control</summary>
		public static ControlLocations SaveLocations(this Control cont)
		{
			return new ControlLocations(cont);
		}

		/// <summary>Imports location data for this control</summary>
		public static void LoadLocations(this Control cont, ControlLocations data)
		{
			data.Apply(cont);
		}

		/// <summary>Temporarily change the cursor while over this control</summary>
		public static Scope ChangeCursor(this Control ctrl, Cursor new_cursor)
		{
			var old_cursor = ctrl.Cursor;
			return Scope.Create(
				() => ctrl.Cursor = new_cursor,
				() => ctrl.Cursor = old_cursor);
		}

		/// <summary>Return the ScrollBars value that represents the current visibility of the scroll bars</summary>
		public static ScrollBars ScrollBarVisibility(this Control ctrl)
		{
			var sty = User32.GetWindowLong(ctrl.Handle, Win32.GWL_STYLE);
			var vis = ScrollBars.None;
			if ((sty & Win32.WS_HSCROLL) != 0) vis |= ScrollBars.Horizontal;
			if ((sty & Win32.WS_VSCROLL) != 0) vis |= ScrollBars.Vertical;
			return vis;
		}
		public static bool ScrollBarHVisible(this Control ctrl)
		{
			return ctrl.ScrollBarVisibility().HasFlag(System.Windows.Forms.ScrollBars.Horizontal);
		}
		public static bool ScrollBarVVisible(this Control ctrl)
		{
			return ctrl.ScrollBarVisibility().HasFlag(System.Windows.Forms.ScrollBars.Vertical);
		}

		/// <summary>Centre this control to its parent</summary>
		public static void CentreToParent(this Control ctrl)
		{
			var scn = SystemInformation.VirtualScreen;
			Win32.CenterWindow(ctrl.Handle, scn);
		}

		/// <summary>Return a form that wraps 'ctrl'</summary>
		public static T FormWrap<T>(this Control ctrl, object[]? args = null, string? title = null, Icon? icon = null, Point? loc = null, Size? sz = null, FormBorderStyle? border = null, FormStartPosition? start_pos = null, bool? show_in_taskbar = null) where T:Form, new()
		{
			var f = (T)Activator.CreateInstance(typeof(T), args);

			// Set the window minimum size
			f.ClientSize = ctrl.MinimumSize;
			f.MinimumSize = f.Bounds.Size;

			f.ClientSize = ctrl.Bounds.Size;
			f.ShowIcon = false;

			if (title != null)
			{
				f.Text = title;
			}
			if (icon != null)
			{
				f.Icon = icon;
				f.ShowIcon = true;
			}
			if (loc.HasValue)
			{
				f.StartPosition = FormStartPosition.Manual;
				f.Location = loc.Value;
			}
			if (sz.HasValue)
			{
				f.Size = sz.Value;
			}
			if (border.HasValue)
			{
				f.FormBorderStyle = border.Value;
			}
			if (start_pos.HasValue)
			{
				f.StartPosition = start_pos.Value;
			}
			if (show_in_taskbar.HasValue)
			{
				f.ShowInTaskbar = show_in_taskbar.Value;
			}
			ctrl.Dock = DockStyle.Fill;
			f.Controls.Add(ctrl);
			return f;
		}
		public static Form FormWrap(this Control ctrl, object[]? args = null, string? title = null, Icon? icon = null, Point? loc = null, Size? sz = null, FormBorderStyle? border = null, FormStartPosition? start_pos = null, bool? show_in_taskbar = null)
		{
			return ctrl.FormWrap<Form>(args, title, icon, loc, sz, border, start_pos, show_in_taskbar);
		}

		/// <summary>Returns the name of this control and all parents in the hierarchy</summary>
		public static string HierarchyName(this Control ctrl)
		{
			var ancestors = new List<string>();
			for (var p = ctrl; p != null; p = p.Parent)
				ancestors.Add($"{p.Name}({p.GetType().Name})");
			ancestors.Reverse();
			return string.Join("->", ancestors);
		}

		/// <summary>RAII initialise scope</summary>
		public static Scope InitialiseScope(this ISupportInitialize si)
		{
			return Scope.Create(
				() => si.BeginInit(),
				() => si.EndInit());
		}

		/// <summary>RAII scope for temporarily disabling 'Enabled' on this control</summary>
		public static Scope SuspendEnabled(this Control ctrl)
		{
			return Scope.Create(
				() => { var e = ctrl.Enabled; ctrl.Enabled = false; return e; },
				e => { ctrl.Enabled = e; });
		}

		/// <summary>
		/// Mark a control as 'loading' by assigning it's Tag property to an instance of the 'LoadSaveTag' class
		/// The original Tag value is preserved once the scope exits</summary>
		public static Scope MarkAsLoading(this Control ctrl)
		{
			return Scope.Create(
				() => ctrl.Tag = new LoadSaveTag{OriginalTag = ctrl.Tag, Loading = true},
				() => ctrl.Tag = ((LoadSaveTag)ctrl.Tag).OriginalTag);
		}

		/// <summary>
		/// Mark a control as 'saving' by assigning it's Tag property to an instance of the 'LoadSaveTag' class
		/// The original Tag value is preserved once the scope exits</summary>
		public static Scope MarkAsSaving(this Control ctrl)
		{
			return Scope.Create(
				() => ctrl.Tag = new LoadSaveTag{OriginalTag = ctrl.Tag, Loading = false},
				() => ctrl.Tag = ((LoadSaveTag)ctrl.Tag).OriginalTag);
		}

		/// <summary>True while this control is marked as loading</summary>
		public static bool IsLoading(this Control ctrl)
		{
			return ctrl.Tag is LoadSaveTag && ((LoadSaveTag)ctrl.Tag).Loading;
		}

		/// <summary>True while this control is marked as saving</summary>
		public static bool IsSaving(this Control ctrl)
		{
			return ctrl.Tag is LoadSaveTag && ((LoadSaveTag)ctrl.Tag).Loading == false;
		}
	}

	/// <summary>TextBox control extensions</summary>
	public static class TextBox_
	{
		/// <summary>Returns a disposable object that preserves the current selection</summary>
		public static Scope<RangeI> SelectionScope(this TextBoxBase edit)
		{
			return Scope.Create(
				() => RangeI.FromStartLength(edit.SelectionStart, edit.SelectionLength),
				rn => edit.Select(rn.Begi, rn.Sizei));
		}

		/// <summary>Returns a disposable object that preserves the current selection</summary>
		public static Scope<RangeI> SelectionScope(this ComboBox edit)
		{
			return Scope.Create(
				() => RangeI.FromStartLength(edit.SelectionStart, edit.SelectionLength),
				pt => edit.Select(pt.Begi, pt.Sizei));
		}

		/// <summary>A smarter set text that does sensible things with the caret position</summary>
		public static void SetText(this TextBoxBase tb, string text)
		{
			var idx = tb.SelectionStart;
			tb.SelectedText = text;
			tb.SelectionStart = idx + text.Length;
		}

		/// <summary>A smarter set text that does sensible things with the caret position</summary>
		public static void SetText(this ComboBox cb, string text)
		{
			var idx = cb.SelectionStart;
			cb.SelectedText = text;
			cb.SelectionStart = idx + text.Length;
		}

		/// <summary>Show or hide the caret for this text box. Returns true if successful. Successful Show/Hide calls must be matched</summary>
		public static bool ShowCaret(this TextBoxBase tb, bool show)
		{
			return show
				? User32.ShowCaret(tb.Handle) != 0
				: User32.HideCaret(tb.Handle) != 0;
		}

		/// <summary>A smarter set text that does sensible things with the caret position</summary>
		public static void AppendText(this TextBoxBase tb, string text)
		{
			var carot_at_end = tb.SelectionStart == tb.TextLength && tb.SelectionLength == 0;
			if (carot_at_end)
			{
				tb.SelectedText = text;
				tb.SelectionStart = tb.TextLength;
			}
			else
			{
				using (tb.SelectionScope())
					tb.AppendText(text);
			}
		}

		/// <summary>Add the current combo box text to the drop down list</summary>
		public static void AddTextToDropDownList(this ComboBox cb, int max_history = 10, bool only_if_has_value = true)
		{
			// Need to take a copy of the text, because Remove() will delete the text
			// if the current text is actually a selected item.
			var text = cb.Text ?? string.Empty;
			var selection = new RangeI(cb.SelectionStart, cb.SelectionStart + cb.SelectionLength);

			if (text.HasValue() || !only_if_has_value)
			{
				// Insert at position 0
				cb.Items.Remove(text);
				cb.Items.Insert(0, text);

				while (cb.Items.Count > max_history)
					cb.Items.RemoveAt(cb.Items.Count - 1);

				// Don't set SelectedIndex = 0 here because that causes 'OnTextChanged'
				// It's likely that 'AddTextToDropDownList' is being called from a TextChanged handler
			}

			cb.Select(selection.Begi, selection.Sizei);
		}

		/// <summary>
		/// Set the text box into a state indicating uninitialised, error, or success.
		/// For tool tips, use string.Empty to clear the tool tip, otherwise it will be unchanged</summary>
		public static void HintState(this Control ctrl, EHintState state, ToolTip? tt = null 
			,Color? success_col = null ,Color? error_col = null ,Color? uninit_col = null
			,string? success_tt = null  ,string? error_tt = null  ,string? uninit_tt = null)
		{
			switch (state)
			{
			case EHintState.Uninitialised:
				{
					ctrl.BackColor = uninit_col ?? SystemColors.Window;
					if (tt != null && uninit_tt != null)
						ctrl.ToolTip(tt, uninit_tt);
					break;
				}
			case EHintState.Error:
				{
					ctrl.BackColor = error_col ?? Color.LightSalmon;
					if (tt != null && error_tt != null)
						ctrl.ToolTip(tt, error_tt ?? string.Empty);
					break;
				}
			case EHintState.Success:
				{
					ctrl.BackColor = success_col ?? Color.LightGreen;
					if (tt != null && success_tt != null)
						ctrl.ToolTip(tt, success_tt ?? string.Empty);
					break;
				}
			}
		}

		/// <summary>
		/// Set the text box into a state indicating uninitialised, error, or success.
		/// Uninitialised state is inferred from success and no value in the text box</summary>
		public static void HintState(this TextBoxBase tb, bool success, ToolTip? tt = null 
			,Color? success_col = null ,Color? error_col = null ,Color? uninit_col = null
			,string? success_tt = null  ,string? error_tt = null  ,string? uninit_tt = null)
		{
			var state = 
				(success && !tb.Text.HasValue()) ? EHintState.Uninitialised :
				(success) ? EHintState.Success :
				EHintState.Error;
			tb.HintState(state, tt, success_col, error_col, uninit_col, success_tt, error_tt, uninit_tt);
		}
		public static void HintState(this ComboBox cb, bool success, ToolTip? tt = null 
			,Color? success_col = null ,Color? error_col = null ,Color? uninit_col = null
			,string? success_tt = null  ,string? error_tt = null  ,string? uninit_tt = null)
		{
			var state = 
				(success && !cb.Text.HasValue()) ? EHintState.Uninitialised :
				(success) ? EHintState.Success :
				EHintState.Error;
			cb.HintState(state, tt, success_col, error_col, uninit_col, success_tt, error_tt, uninit_tt);
		}

		/// <summary>States that a text box can be in</summary>
		public enum EHintState
		{
			Uninitialised,
			Error,
			Success,
		}
	}

	/// <summary>ComboBox control extensions</summary>
	public static class ComboBox_
	{
		/// <summary>Add a range</summary>
		public static void AddRange<T>(this System.Windows.Forms.ComboBox.ObjectCollection cb, IEnumerable<T> objs)
		{
			foreach (var obj in objs)
				cb.Add(obj);
		}

		/// <summary>Return the auto-sized with of the drop down menu from it's current content</summary>
		public static int DropDownWidthAutoSize(this System.Windows.Forms.ComboBox cb)
		{
			// Notes: attach to 
			//  cb.DropDown += (s,a) => cb.DropDownWidth = DropDownWidthAutoSize();

			var mi = (MethodInfo?)null;
			return !cb.DisplayMember.HasValue()
				? cb.DropDownWidthAutoSize(x => x.ToString())
				: cb.DropDownWidthAutoSize(x =>
				{
					mi = mi ?? (x.GetType().GetProperty(cb.DisplayMember).GetGetMethod());
					return mi.Invoke(x, null).ToString();
				});
		}
		private static int DropDownWidthAutoSize(this System.Windows.Forms.ComboBox cb, Func<object, string> description)
		{
			// Calculate the width of the items (includes DataSource)
			var width = cb.Width;
			foreach (var obj in cb.Items)
				width = Math.Max(width, TextRenderer.MeasureText(description(obj), cb.Font).Width);

			// Return the calculated width (plus room for the scroll bar)
			return width + (cb.Items.Count > cb.MaxDropDownItems ? SystemInformation.VerticalScrollBarWidth : 0);
		}

		/// <summary>Auto size the drop down list to the content. Attach to 'cb.DropDown'</summary>
		public static void DropDownWidthAutoSize(object sender, EventArgs args)
		{
			var cb = (System.Windows.Forms.ComboBox)sender;
			cb.DropDownWidth = DropDownWidthAutoSize(cb);
		}

		/// <summary>An event handler that converts the enum value into a string description. Attach to the 'Format' event</summary>
		public static void FormatValue(object sender, ListControlConvertEventArgs args)
		{
			// Use the description attribute if available
			var desc = ((Enum)args.ListItem).Desc();
			if (desc != null)
			{
				args.Value = desc;
				return;
			}

			// Otherwise pretty up the string name
			else
			{
				desc = args.ListItem.ToString().Txfm(Str_.ECapitalise.UpperCase, Str_.ECapitalise.LowerCase, Str_.ESeparate.Add, " ");
				args.Value = desc;
			}
		}
	}

	/// <summary>ListBox control extensions</summary>
	public static class ListBox_
	{
		/// <summary>List box select all implementation.</summary>
		public static void SelectAll(this System.Windows.Forms.ListBox lb)
		{
			if (lb.SelectionMode == SelectionMode.MultiSimple ||
				lb.SelectionMode == SelectionMode.MultiExtended)
				Enumerable.Range(0, lb.Items.Count).ForEach(i => lb.SetSelected(i, true));
		}

		/// <summary>List box select none implementation.</summary>
		public static void SelectNone(this System.Windows.Forms.ListBox lb)
		{
			Enumerable.Range(0, lb.Items.Count).ForEach(i => lb.SetSelected(i, false));
		}

		/// <summary>ListBox copy implementation. Returns true if something was added to the clip board</summary>
		public static bool Copy(this System.Windows.Forms.ListBox lb)
		{
			var d = string.Join("\n", lb.SelectedItems.Cast<object>().Select(x => x.ToString()));
			if (!d.HasValue()) return false;
			Clipboard.SetDataObject(d);
			return true;
		}

		/// <summary>Select all rows. Attach to the KeyDown handler</summary>
		public static void SelectAll(object sender, KeyEventArgs e)
		{
			if (e.Handled) return; // already handled
			var lb = (System.Windows.Forms.ListBox)sender;
			if (!e.Control || e.KeyCode != Keys.A) return;
			SelectAll(lb);
			e.Handled = true;
		}

		/// <summary>Copy selected items to the clipboard. Attach to the KeyDown handler</summary>
		public static void Copy(object sender, KeyEventArgs e)
		{
			if (e.Handled) return; // already handled
			var lb = (System.Windows.Forms.ListBox)sender;
			if (!e.Control || e.KeyCode != Keys.C) return;
			if (!Copy(lb)) return;
			e.Handled = true;
		}
	}

	/// <summary>Tree View control extensions</summary>
	public static class TreeView_
	{
		/// <summary>Enumerate through all nodes in a collection</summary>
		private static IEnumerable<TreeNode> AllNodes(TreeNodeCollection nodes, ERecursionOrder order)
		{
			switch (order)
			{
			case ERecursionOrder.BreadthFirst:
				{
					var queue = new Queue<TreeNode>(nodes.Cast<TreeNode>());
					while (queue.Count != 0)
					{
						var node = queue.Dequeue();
						for (int i = 0; i != node.Nodes.Count; ++i) queue.Enqueue(node.Nodes[i]);
						yield return node;
					}
					break;
				}
			case ERecursionOrder.DepthFirst:
				{
					var stack = new Stack<TreeNode>(nodes.Cast<TreeNode>());
					while (stack.Count != 0)
					{
						var node = stack.Pop();
						for (int i = node.Nodes.Count; i-- != 0; ) stack.Push(node.Nodes[i]);
						yield return node;
					}
					break;
				}
			}
		}
		public enum ERecursionOrder { DepthFirst, BreadthFirst }

		/// <summary>Enumerate through all nodes in a the tree</summary>
		public static IEnumerable<TreeNode> AllNodes(this TreeView tree, ERecursionOrder order)
		{
			return AllNodes(tree.Nodes, order);
		}

		/// <summary>Enumerate through all child nodes</summary>
		public static IEnumerable<TreeNode> AllNodes(this TreeNode node, ERecursionOrder order)
		{
			return AllNodes(node.Nodes, order);
		}

		/// <summary>
		/// Return the node corresponding to the given path. Uses the tree's path separator as the delimiter.
		/// If 'add_node' is given, child nodes are added where necessary.</summary>
		public static TreeNode? GetNode(this TreeView tree, string fullpath, Func<string,TreeNode>? add_node = null)
		{
			var parts = fullpath.Split(new[]{tree.PathSeparator}, StringSplitOptions.RemoveEmptyEntries);

			var node = (TreeNode?)null;
			var nodes = tree.Nodes;
			foreach (var part in parts)
			{
				if      (part == ".") continue;
				else if (part == ".." && node != null) node = node.Parent;
				else node = nodes.Cast<TreeNode>().FirstOrDefault(x => string.Compare(x.Name, part) == 0);
				if (node == null && add_node != null)
					node = add_node(part);
				if (node == null)
					throw new Exception($"Node path {fullpath} does not exist");

				nodes = node.Nodes;
			}
			return node;
		}

		/// <summary>
		/// Expands branches of the tree down to 'path'. Uses the tree's path separator as the delimiter.
		/// 'ignore_case' ignores case differences between the parts of 'fullpath' and the node.Name's.</summary>
		public static void Expand(this TreeView tree, string fullpath)
		{
			tree.GetNode(fullpath)?.EnsureVisible();
		}

		/// <summary>Ensure the parents of this node are expanded</summary>
		public static void ExpandParents(this TreeNode node)
		{
			if (node.Parent == null) return;
			if (node.Parent.IsExpanded) return;
			node.Parent.Expand();
			node.Parent.ExpandParents();
		}
	}

	/// <summary>NumericUpDown control extensions</summary>
	public static class Spinner_
	{
		/// <summary>Set the value, min, and max on the spinner in a way that won't throw</summary>
		public static void Set(this NumericUpDown spinner, decimal val, decimal min, decimal max)
		{
			if (min > max)
				throw new Exception("Minimum value is greater than the maximum value");
			if (val != Math_.Clamp(min, val, max))
				throw new Exception("Value is not within the given range of values");

			// Setting to Minimum/Maximum values first to avoids problems if min > Value or max < Value
			spinner.Minimum = decimal.MinValue;
			spinner.Maximum = decimal.MaxValue;

			// Setting Value before Min/Max avoids setting Value twice when Value < MinDate or Value > MaxDate
			spinner.Value = val;
			spinner.Minimum = min;
			spinner.Maximum = max;
		}

		/// <summary>Return the value of the track bar as a normalised fraction</summary>
		public static double ValueFrac(this NumericUpDown spinner)
		{
			return Math_.Frac((double)spinner.Minimum, (double)spinner.Value, (double)spinner.Maximum);
		}

		/// <summary>Return the value of the track bar as a normalised fraction of the min-max range</summary>
		public static void ValueFrac(this NumericUpDown spinner, float frac)
		{
			spinner.Value = Math_.Lerp(spinner.Minimum, spinner.Maximum, frac);
		}

		/// <summary>Set the value of track bar, clamping it to the min/max range</summary>
		public static decimal ValueClamped(this NumericUpDown spinner, decimal value)
		{
			return spinner.Value = Math_.Clamp(value, spinner.Minimum, spinner.Maximum);
		}
	}

	/// <summary>Track bar extensions</summary>
	public static class TrackBar_
	{
		/// <summary>Set the value, min, and max on the spinner in a way that won't throw</summary>
		public static void Set(this TrackBar tb, int val, int min, int max)
		{
			if (min > max)
				throw new Exception("Minimum value is greater than the maximum value");
			if (val != Math_.Clamp(min, val, max))
				throw new Exception("Value is not within the given range of values");

			// Setting to Minimum/Maximum values first to avoid problems if min > Value or max < Value
			tb.Minimum = Math.Min(min, tb.Value);
			tb.Maximum = Math.Max(max, tb.Value);

			// Setting Value before Min/Max avoids setting Value twice when Value < Min or Value > Max
			tb.Value = val;
			tb.Minimum = min;
			tb.Maximum = max;
		}

		/// <summary>Return the value of the track bar as a normalised fraction</summary>
		public static double ValueFrac(this TrackBar tb)
		{
			return Math_.Frac((double)tb.Minimum, (double)tb.Value, (double)tb.Maximum);
		}

		/// <summary>Return the value of the track bar as a normalised fraction of the min-max range</summary>
		public static void ValueFrac(this TrackBar tb, double frac)
		{
			tb.Value = Math_.Lerp(tb.Minimum, tb.Maximum, frac);
		}

		/// <summary>Set the value of track bar, clamping it to the min/max range</summary>
		public static int ValueClamped(this TrackBar tb, int value)
		{
			return tb.Value = Math_.Clamp(value, tb.Minimum, tb.Maximum);
		}
	}

	/// <summary>Progress bar extensions</summary>
	public static class ProgressBar_
	{
		/// <summary>Set the value, min, and max on the spinner in a way that won't throw</summary>
		public static void Set(this ProgressBar pb, int val, int min, int max)
		{
			if (min > max)
				throw new Exception("Minimum value is greater than the maximum value");
			if (val != Math_.Clamp(min, val, max))
				throw new Exception("Value is not within the given range of values");

			// Setting to Minimum/Maximum values first to avoids problems if min > Value or max < Value
			pb.Minimum = int.MinValue;
			pb.Maximum = int.MaxValue;

			// Setting Value before Min/Max avoids setting Value twice when Value < MinDate or Value > MaxDate
			pb.Value = val;
			pb.Minimum = min;
			pb.Maximum = max;
		}
		public static void Set(this ToolStripProgressBar pb, int val, int min, int max)
		{
			if (min > max)
				throw new Exception("Minimum value is greater than the maximum value");
			if (val != Math_.Clamp(min, val, max))
				throw new Exception("Value is not within the given range of values");

			// Setting to Minimum/Maximum values first to avoids problems if min > Value or max < Value
			pb.Minimum = int.MinValue;
			pb.Maximum = int.MaxValue;

			// Setting Value before Min/Max avoids setting Value twice when Value < MinDate or Value > MaxDate
			pb.Value = val;
			pb.Minimum = min;
			pb.Maximum = max;
		}

		/// <summary>Return the value of the progress bar as a normalised fraction</summary>
		public static double ValueFrac(this ProgressBar pb)
		{
			return Math_.Frac((double)pb.Minimum, (double)pb.Value, (double)pb.Maximum);
		}
		public static double ValueFrac(this ToolStripProgressBar pb)
		{
			return Math_.Frac((double)pb.Minimum, (double)pb.Value, (double)pb.Maximum);
		}

		/// <summary>Set the value of the progress bar as a normalised fraction of the min-max range</summary>
		public static void ValueFrac(this ProgressBar pb, double frac)
		{
			pb.Value = Math_.Lerp(pb.Minimum, pb.Maximum, frac);
		}
		public static void ValueFrac(this ToolStripProgressBar pb, double frac)
		{
			pb.Value = Math_.Lerp(pb.Minimum, pb.Maximum, frac);
		}

		/// <summary>Set the value of progress bar, clamping it to the min/max range</summary>
		public static int ValueClamped(this ProgressBar pb, int value)
		{
			return pb.Value = Math_.Clamp(value, pb.Minimum, pb.Maximum);
		}
		public static int ValueClamped(this ToolStripProgressBar pb, int value)
		{
			return pb.Value = Math_.Clamp(value, pb.Minimum, pb.Maximum);
		}

		/// <summary>Set the colour of a progress bar. State = 1(green), 2(red), 3(yellow)</summary>
		public static void BackColor(this ProgressBar pb, int state)
		{
			User32.SendMessage(pb.Handle, 1040, (IntPtr)state, IntPtr.Zero);
		}
	}

	/// <summary>DataGridView control extensions</summary>
	public static class DataGridView_
	{
		// Notes:
		// Grid standard keyboard shortcuts
		// Use: add for each function you want supported
		//   e.g.
		//   m_grid.ClipboardCopyMode = DataGridViewClipboardCopyMode.EnableWithoutHeaderText;
		//   m_grid.KeyDown += DataGridView_.Cut;
		//   m_grid.ContextMenuStrip.Items.Add("Copy", null, (s,a) => DataGridView_.Copy(m_grid, a));

		[Flags] public enum EEditOptions
		{
			Copy      = 1 << 0,
			Cut       = 1 << 1,
			Paste     = 1 << 2,
			Delete    = 1 << 3,
			SelectAll = 1 << 4,

			ReadOnly  = Copy | SelectAll,
			ReadWrite = ReadOnly | Cut | Paste,
			All       = ReadWrite | Delete,
		}

		/// <summary>Grid select all implementation. (for consistency)</summary>
		public static void SelectAll(DataGridView grid)
		{
			grid.SelectAll();
		}

		/// <summary>Select all rows. Attach to the KeyDown handler or use as context menu handler</summary>
		public static void SelectAll(object sender, EventArgs e)
		{
			var dgv = (DataGridView)sender;

			var ke = e as KeyEventArgs;
			if (ke != null)
			{
				if (ke.Handled) return; // already handled
				if (!(Control.ModifierKeys.HasFlag(Keys.Control) && ke.KeyCode == Keys.A)) return; // not ctrl + A
			}
			try
			{
				SelectAll(dgv);
				if (ke != null) ke.Handled = true;
			}
			catch (Exception ex)
			{
				Debug.WriteLine($"DataGridView select all operation failed: {ex.MessageFull()}");
			}
		}

		/// <summary>Grid copy implementation. Returns true if something was added to the clipboard</summary>
		public static bool Copy(DataGridView grid)
		{
			var d = grid.GetClipboardContent();
			if (d == null) return false;
			Clipboard.SetDataObject(d);
			return true;
		}

		/// <summary>Copy selected cells to the clipboard. Attach to the KeyDown handler</summary>
		public static void Copy(object sender, EventArgs e)
		{
			var dgv = (DataGridView)sender;

			var ke = e as KeyEventArgs;
			if (ke != null)
			{
				if (ke.Handled) return; // already handled
				if (!(Control.ModifierKeys.HasFlag(Keys.Control) && ke.KeyCode == Keys.C)) return; // not ctrl + C
			}
			try
			{
				if (!Copy(dgv)) return;
				if (ke != null) ke.Handled = true;
			}
			catch (Exception ex)
			{
				Debug.WriteLine($"DataGridView copy operation failed: {ex.MessageFull()}");
			}
		}

		/// <summary>Grid cut implementation. Returns true if something was cut and added to the clip board</summary>
		public static bool Cut(DataGridView grid)
		{
			DataObject d = grid.GetClipboardContent();
			if (d == null) return false;
			Clipboard.SetDataObject(d);

			// Set the selected cells to defaults
			foreach (DataGridViewCell c in grid.SelectedCells)
				if (!c.ReadOnly) c.Value = c.DefaultNewRowValue;

			return true;
		}

		/// <summary>Cut the selected cells to the clipboard. Cut cells replaced with default values. Attach to the KeyDown handler</summary>
		public static void Cut(object sender, EventArgs e)
		{
			var dgv = (DataGridView)sender;

			var ke = e as KeyEventArgs;
			if (ke != null)
			{
				if (ke.Handled) return; // already handled
				if (!(Control.ModifierKeys.HasFlag(Keys.Control) && ke.KeyCode == Keys.X)) return; // not ctrl + X
			}
			try
			{
				if (!Cut(dgv)) return;
				if (ke != null) ke.Handled = true;
			}
			catch (Exception ex)
			{
				Debug.WriteLine($"DataGridView cut operation failed: {ex.MessageFull()}");
			}
		}

		/// <summary>Grid paste implementation that pastes over existing cells within the current size limits of the grid</summary>
		public static bool Paste(DataGridView grid)
		{
			return PasteReplace(grid);
		}

		/// <summary>Grid paste implementation that pastes over existing cells within the current size limits of the grid</summary>
		public static void Paste(object sender, EventArgs e)
		{
			PasteReplace(sender, e);
		}

		/// <summary>Grid delete implementation. Deletes selected items from the grid setting cell values to null</summary>
		public static void Delete(DataGridView grid)
		{
			foreach (DataGridViewCell c in grid.SelectedCells)
				c.Value = null;
		}

		/// <summary>Delete the contents of the selected cells. Attach to the KeyDown handler</summary>
		public static void Delete(object sender, EventArgs e)
		{
			var dgv = (DataGridView)sender;

			var ke = e as KeyEventArgs;
			if (ke != null)
			{
				if (ke.Handled) return; // already handled
				if (!(ke.KeyCode == Keys.Delete)) return; // not the delete key
			}
			try
			{
				Delete(dgv);
				if (ke != null) ke.Handled = true;
			}
			catch (Exception ex)
			{
				Debug.WriteLine($"DataGridView delete operation failed: {ex.MessageFull()}");
			}
		}

		/// <summary>Grid paste implementation that pastes over existing cells within the current size limits of the grid</summary>
		public static bool PasteReplace(DataGridView grid)
		{
			// Read the lines from the clipboard
			var lines = Clipboard.GetText().Split('\n');

			if (grid.SelectedCells.Count == 1)
			{
				var row = grid.CurrentCell.RowIndex;
				for (var j = 0; j != lines.Length && row != grid.RowCount; ++j, ++row)
				{
					// Skip blank lines
					if (lines[j].Length == 0) continue;

					var col = grid.CurrentCell.ColumnIndex;
					var cells = lines[j].Split(new[] { ' ','\t',',',';','|' }, StringSplitOptions.RemoveEmptyEntries);
					for (var i = 0; i != cells.Length && col != grid.ColumnCount; ++i, ++col)
					{
						var cell = grid[col,row];
						if (cell.ReadOnly) continue;
						if (cells[i].Length == 0) continue;
						try
						{
							var val = Utility.Util.ConvertTo(cells[i].Trim(), cell.ValueType);
							cell.Value = val;
							grid.InvalidateCell(cell);
						}
						catch (FormatException)
						{
							cell.Value = cell.DefaultNewRowValue;
							grid.InvalidateCell(cell);
						}
					}
				}
			}
			else if (grid.SelectedCells.Count > 1)
			{
				// Get a snapshot of the selected grid cells
				var selected_cells = grid.SelectedCells;

				// Find the bounds of the selected cells
				var min = new Point(selected_cells[0].ColumnIndex, selected_cells[0].RowIndex);
				var max = min;
				foreach (DataGridViewCell item in selected_cells)
				{
					min.X = Math.Min(min.X, item.ColumnIndex);
					min.Y = Math.Min(min.Y, item.RowIndex);
					max.X = Math.Max(max.X, item.ColumnIndex+1);
					max.Y = Math.Max(max.Y, item.RowIndex+1);
				}
				var dst_dim = new Point(max.X - min.X, max.Y - min.Y);

				// Read the cells from the clipboard
				var cells = new string[lines.Length][];
				var src_dim = new Point(0,0);
				for (var r = 0; r != lines.Length; ++r)
				{
					var row = lines[r].Split(new[] { ' ','\t',',',';','|' }, StringSplitOptions.RemoveEmptyEntries);
					cells[r] = row;
					src_dim.X = Math.Max(src_dim.X, row.Length);
					src_dim.Y = src_dim.Y + 1;
				}

				// Special case 1-dimensional data. If a row vector is pasted
				// into a column, or visa versa, automatically transpose the data.
				if ((src_dim.X == 1 && dst_dim.Y == 1 && src_dim.Y == dst_dim.X) ||
					(src_dim.Y == 1 && dst_dim.X == 1 && src_dim.X == dst_dim.Y))
				{
					var cellsT = new string[src_dim.X][];
					for (var r = 0; r != cellsT.Length; ++r)
					{
						cellsT[r] = new string[cells.Length];
						for (var c = 0; c != cells.Length; ++c)
							cellsT[r][c] = cells[c][r];
					}

					cells = cellsT;
					var tmp = src_dim.X; src_dim.Y = src_dim.X; src_dim.X = tmp;
				}

				// Paste into the selected cells, filling if the selected cell area
				// is bigger than the clipboard cells
				foreach (DataGridViewCell cell in selected_cells)
				{
					if (cell.ReadOnly) continue;

					try
					{
						// Find the corresponding clipboard data for 'cell'
						var row = Math.Min(cell.RowIndex    - min.Y, cells.Length      - 1);
						var col = Math.Min(cell.ColumnIndex - min.X, cells[row].Length - 1);
						if (row >= 0 && col >= 0 && cells[row][col].Length != 0)
						{
							cell.Value = Convert.ChangeType(cells[row][col], cell.ValueType);
							grid.InvalidateCell(cell);
						}
					}
					catch (FormatException)
					{
						cell.Value = cell.DefaultNewRowValue;
						grid.InvalidateCell(cell);
					}
				}
			}
			return true;
		}

		/// <summary>Paste over existing cells within the current size limits of the grid. Attach to the KeyDown handler</summary>
		public static void PasteReplace(object sender, EventArgs e)
		{
			var dgv = (DataGridView)sender;

			var ke = e as KeyEventArgs;
			if (ke != null)
			{
				if (ke.Handled) return; // already handled
				if (!(Control.ModifierKeys.HasFlag(Keys.Control) && ke.KeyCode == Keys.V)) return; // not ctrl + V
			}
			try
			{
				if (!PasteReplace(dgv)) return;
				if (ke != null) ke.Handled = true;
			}
			catch (Exception ex)
			{
				Debug.WriteLine($"DataGridView paste (replace) operation failed: {ex.MessageFull()}");
			}
		}

		/// <summary>Paste from the first selected cell over anything in the way. Grow the grid if necessary</summary>
		public static bool PasteGrow(DataGridView grid)
		{
			if (grid.SelectedCells.Count > 1) return false;

			var row = grid.CurrentCell.RowIndex;
			var col = grid.CurrentCell.ColumnIndex;

			// Read the lines from the clipboard
			var lines = Clipboard.GetText().Split('\n');

			// Grow the DGV if necessary
			if (row + lines.Length > grid.RowCount)
				grid.RowCount = row + lines.Length;

			for (var j = 0; j != lines.Length; ++j)
			{
				// Skip blank lines
				if (lines[j].Length == 0) continue;

				var cells = lines[j].Split('\t', ',', ';');

				// Grow the grid if necessary
				if (col + cells.Length > grid.ColumnCount)
					grid.ColumnCount = col + cells.Length;

				for (var i = 0; i != cells.Length; ++i)
				{
					var cell = grid[i+col,j+row];
					if (cell.ReadOnly) continue;
					if (cells[i].Length == 0) continue;
					try { cell.Value = Convert.ChangeType(cells[i], cell.ValueType); }
					catch (FormatException) { cell.Value = cell.DefaultNewRowValue; }
				}
			}
			return true;
		}

		/// <summary>Paste from the first selected cell over anything in the way. Grow the grid if necessary. Attach to the KeyDown handler</summary>
		public static void PasteGrow(object sender, EventArgs e)
		{
			var dgv = (DataGridView)sender;

			var ke = e as KeyEventArgs;
			if (ke != null)
			{
				if (ke.Handled) return; // already handled
				if (!(Control.ModifierKeys.HasFlag(Keys.Control) && ke.KeyCode == Keys.V)) return; // not ctrl + V
			}
			try
			{
				if (!PasteGrow(dgv)) return;
				if (ke != null) ke.Handled = true;
			}
			catch (Exception ex)
			{
				Debug.WriteLine($"DataGridView paste (grow) operation failed: {ex.MessageFull()}");
			}
		}

		/// <summary>Combined handler for cut, copy, and paste replace functions. Attach to the KeyDown handler</summary>
		public static void CutCopyPasteReplace(object sender, EventArgs e)
		{
			var ke = e as KeyEventArgs;
			if (ke != null && ke.Handled) return; // already handled
			SelectAll   (sender, e);
			Cut         (sender, e);
			Copy        (sender, e);
			PasteReplace(sender, e);
		}

		/// <summary>Create a context menu with basic Copy,Cut,Paste,Delete options. Assign to ContextMenuStrip</summary>
		public static ContextMenuStrip CMenu(DataGridView grid, EEditOptions edit_options)
		{
			var cmenu = new ContextMenuStrip();
			using (cmenu.SuspendLayout(false))
			{
				if (edit_options.HasFlag(EEditOptions.Copy))
				{
					var opt = cmenu.Items.Add2(new ToolStripMenuItem("Copy"));
					opt.Click += (s,a) => Copy(grid);
				}
				if (edit_options.HasFlag(EEditOptions.Cut))
				{
					var opt = cmenu.Items.Add2(new ToolStripMenuItem("Cut"));
					opt.Click += (s,a) => Cut(grid);
				}
				if (edit_options.HasFlag(EEditOptions.Paste))
				{
					var opt = cmenu.Items.Add2(new ToolStripMenuItem("Paste"));
					opt.Click += (s,a) => Paste(grid);
				}
				cmenu.Items.AddSeparator();
				if (edit_options.HasFlag(EEditOptions.Delete))
				{
					var opt = cmenu.Items.Add2(new ToolStripMenuItem("Delete"));
					opt.Click += (s,a) => Delete(grid);
				}
				cmenu.Items.AddSeparator();
				if (edit_options.HasFlag(EEditOptions.SelectAll))
				{
					var opt = cmenu.Items.Add2(new ToolStripMenuItem("Select All"));
					opt.Click += (s,a) => SelectAll(grid);
				}
				cmenu.Items.TidySeparators();
			}
			return cmenu;
		}

		/// <summary>Cycle to the next sort direction</summary>
		public static SortOrder Next(this SortOrder so, bool three_state = true)
		{
			switch (so)
			{
			default: throw new Exception($"Unknown sort order: {so}");
			case SortOrder.Ascending:  return SortOrder.Descending;
			case SortOrder.Descending: return three_state ? SortOrder.None : SortOrder.Ascending;
			case SortOrder.None:       return SortOrder.Ascending;
			}
		}
		public static ListSortDirection NextDirection(this SortOrder so)
		{
			switch (so)
			{
			default: throw new Exception($"Unknown sort order: {so}");
			case SortOrder.Ascending:  return ListSortDirection.Descending;
			case SortOrder.Descending: return ListSortDirection.Ascending;
			case SortOrder.None:       return ListSortDirection.Ascending;
			}
		}

		/// <summary>Static data associated with a grid to provide this extension functionality</summary>
		[ThreadStatic] private static DGVStateMap GridState = new DGVStateMap();
		private class DGVStateMap :Dictionary<DataGridView, DGVState>
		{
			public new DGVState this[DataGridView grid]
			{
				get { return this.GetOrAdd(grid, k => new DGVState(grid)); }
				set { base[grid] = value; }
			}
		}
		private class DGVState :IDisposable
		{
			public DGVState(DataGridView grid)
			{
				Grid = grid;
				Grid.Disposed += HandleGridDisposed;
			}
			public void Dispose()
			{
				ColumnFilters?.Dispose();
				Grid.Disposed -= HandleGridDisposed;
				GridState.Remove(Grid);
			}
			private void HandleGridDisposed(object sender, EventArgs e)
			{
				Dispose();
			}

			public DataGridView Grid;
			public ColumnFiltersData? ColumnFilters;
			public bool InSetGridColumnSizes;
			public bool FitColumnsPending;
		}

		/// <summary>Make the selection colour of grid cells a shaded version of their unselected colour. (Attach to CellFormatting)</summary>
		public static void HalfBrightSelection(object sender, DataGridViewCellFormattingEventArgs args)
		{
			args.CellStyle.SelectionForeColor = args.CellStyle.ForeColor;
			args.CellStyle.SelectionBackColor = args.CellStyle.BackColor.Lerp(Color.Gray, 0.5f);
		}

		/// <summary>
		/// Handle column sorting for grids with data sources that don't support sorting by default.
		/// 'handle_sort' will be called after the column header glyph has changed.
		/// *WARNING*: 'handle_sort' cannot be removed so only call once and be careful with reference lifetimes</summary>
		public static void SupportSorting(this DataGridView grid, EventHandler<DGVSortEventArgs> handle_sort)
		{
			// TODO, using reflection, you could search the invocation list of ColumnHeaderMouseClick for
			// DGVSortDelegate (sub-classed from Delegate) and remove it if 'handle_sort' is null.
			// DGVSortDelegate could also contain the reference to 'handle_sort'

			grid.ColumnHeaderMouseClick += (s,a) =>
			{
				var dgv = (DataGridView)s;
				if (a.Button == MouseButtons.Left && Control.ModifierKeys == Keys.None)
				{
					// Reset the sort glyph for the other columns
					foreach (var c in dgv.Columns.Cast<DataGridViewColumn>())
					{
						if (c.Index == a.ColumnIndex) continue;
						c.HeaderCell.SortGlyphDirection = SortOrder.None;
					}

					// Set the glyph on the selected column
					var col = dgv.Columns[a.ColumnIndex];
					var hdr = col.HeaderCell;
					hdr.SortGlyphDirection = Enum<SortOrder>.Cycle(hdr.SortGlyphDirection);

					// Apply the sort
					handle_sort?.Invoke(dgv, new DGVSortEventArgs(a.ColumnIndex, hdr.SortGlyphDirection));
				}
			};
		}

		/// <summary>Display a context menu for showing/hiding columns in the grid (at 'location' relative to the grid).</summary>
		public static void ColumnVisibilityContextMenu(this DataGridView grid, Point location, Action<DataGridViewColumn>? on_vis_changed = null)
		{
			var menu = new ContextMenuStrip();
			foreach (var col in grid.Columns.Cast<DataGridViewColumn>())
			{
				var item = new ToolStripMenuItem
				{
					Text = col.HeaderText,
					Checked = col.Visible,
					CheckOnClick = true,
					Tag = col
				};
				item.Click += (s,a)=>
				{
					var c = (DataGridViewColumn)item.Tag;
					c.Visible = item.Checked;
					if (on_vis_changed != null)
						on_vis_changed(c);
				};
				menu.Items.Add(item);
			}
			menu.Show(grid, location);
		}

		/// <summary>Display a context menu for showing/hiding columns in the grid. Attach to MouseDown</summary>
		public static void ColumnVisibility(object sender, MouseEventArgs args)
		{
			if (args.Button == MouseButtons.Right)
			{
				var grid = (DataGridView)sender;

				// Right mouse on a column header displays a context menu for hiding/showing columns
				var hit = grid.HitTestEx(args.X, args.Y);
				if (hit.Type == HitTestInfo.EType.ColumnHeader || hit.Type == HitTestInfo.EType.ColumnDivider)
					grid.ColumnVisibilityContextMenu(hit.GridPoint);
			}
		}

		/// <summary>Select the cell or row when right clicking on the grid (before context menus are displayed). Attach to MouseDown</summary>
		public static void RightMouseSelect(object sender, MouseEventArgs args)
		{
			var grid = (DataGridView)sender;

			// Hit test at the click location
			var hit = grid.HitTestEx(args.X, args.Y);

			// On right mouse, selected the cell or row
			if (args.Button == MouseButtons.Right)
			{
				if (grid.SelectionMode == DataGridViewSelectionMode.CellSelect && hit.Cell != null && !hit.Cell.Selected)
				{
					grid.ClearSelection();
					hit.Cell.Selected = true;
				}
				if (grid.SelectionMode == DataGridViewSelectionMode.FullRowSelect && hit.Row != null && !hit.Row.Selected)
				{
					grid.ClearSelection();
					hit.Row.Selected = true;
				}
				if (grid.SelectionMode == DataGridViewSelectionMode.FullColumnSelect && hit.Column != null && !hit.Column.Selected)
				{
					grid.ClearSelection();
					hit.Column.Selected = true;
				}
			}
		}

		/// <summary>
		/// Returns an array of the fill weights for the columns in this grid.
		/// If 'zero_if_not_visible' is true, then invisible columns return a fill weight of zero
		/// If 'normalised' is true, the returned array sums to one (note, zero_if_not_visible applies before normalisation)</summary>
		public static float[] FillWeights(this DataGridView grid, bool zero_if_not_visible = false, bool normalised = false)
		{
			var columns = grid.Columns.Cast<DataGridViewColumn>();
			var fw = columns.Select(x => !zero_if_not_visible || x.Visible ? x.FillWeight : 0f).ToArray();
			if (normalised)
			{
				var sum = fw.Sum();
				for (int i = 0; i != fw.Length; ++i)
					fw[i] /= sum;
			}
			return fw;
		}

		/// <summary>Scale the fill weights so that they sum up to 1f</summary>
		public static void NormaliseFillWeights(this DataGridView grid)
		{
			var sum = grid.Columns.Cast<DataGridViewColumn>().Sum(x => x.FillWeight);
			foreach (var col in grid.Columns.Cast<DataGridViewColumn>())
				col.FillWeight /= sum;
		}

		/// <summary>Set the FillWeights based on the current column widths</summary>
		public static void SetFillWeightsFromCurrentWidths(this DataGridView grid)
		{
			var widths = grid.Columns.Cast<DataGridViewColumn>().Select(x => (float)x.Width).ToArray();
			var sum = widths.Sum();
			for (int i = 0; i != widths.Length; ++i)
				grid.Columns[i].FillWeight = widths[i] / sum;
		}

		/// <summary>Returns the width of a grid available for columns</summary>
		private static int GridDisplayWidth(DataGridView grid)
		{
			const int GridDisplayRectMargin = 2;
			var width = grid.DisplayRectangle.Width - GridDisplayRectMargin; // DisplayRectangle includes scroll bar width
			if (grid.RowHeadersVisible) width -= grid.RowHeadersWidth;
			return Math.Max(width, 0);
		}

		/// <summary>When a column width is changed, adjust the fill weights to preserve the column's width, squashing up the columns with indices > 'column_index'</summary>
		public static void SetFillWeightsOnColumnWidthChanged(this DataGridView grid, int column_index)
		{
			if (grid.ColumnCount == 0)
				return;

			// Grid column auto sizing only works when the grid isn't trying to resize itself
			if (grid.AutoSizeColumnsMode != DataGridViewAutoSizeColumnsMode.None || (
				grid.Columns[column_index].AutoSizeMode != DataGridViewAutoSizeColumnMode.None &&
				grid.Columns[column_index].AutoSizeMode != DataGridViewAutoSizeColumnMode.NotSet))
				return;
				
			// Get the display area width and the column widths
			var columns         = grid.Columns.Cast<DataGridViewColumn>();
			var grid_width      = GridDisplayWidth(grid);
			var widths          = columns.Select(x => (float)x.Width).ToArray();
			var min_widths      = columns.Select(x => (float)x.MinimumWidth).ToArray();
			var left_width      = widths.Take(column_index+1).Sum();
			var right_width     = widths.Skip(column_index+1).Sum();
			var left_min_width  = min_widths.Take(column_index+1).Sum();
			var right_min_width = min_widths.Skip(column_index+1).Sum();
			float space, scale;

			// Squish columns to the right, leaving left columns unchanged
			if (Control.ModifierKeys == Keys.None)
			{
				// Squish the right hand side columns into the remaining space
				space = Math.Max(grid_width - left_width, right_min_width);
				scale = space / right_width;
				for (int i = column_index+1; i != widths.Length; ++i)
					widths[i] *= scale;
			}

			// Squish columns to the left, and stretch columns to the right
			else if (Control.ModifierKeys == Keys.Alt)
			{
				var left_weights    = columns.Take(column_index+1).Select(x => x.FillWeight).Normalise().ToArray();
				var right_weights   = columns.Skip(column_index+1).Select(x => x.FillWeight).Normalise().ToArray();

				// Squish the left hand side columns
				space = Math.Max(left_width, left_min_width);
				for (int i = 0; i != column_index+1; ++i)
					widths[i] = left_weights[i] * space;

				// Expand the right hand side columns into the remaining space
				space = Math.Max(grid_width - space, right_min_width);
				for (int i = column_index+1; i != widths.Length; ++i)
					widths[i] = right_weights[i-column_index-1] * space;
			}

			// Set the fill weights based on the new distribution of widths
			var total_width = widths.Sum();
			for (int i = 0; i != widths.Length; ++i)
				grid.Columns[i].FillWeight = widths[i] / total_width;
		}

		// Column Width Sizing:
		//   grid.ColumnWidthChanged         += DataGridView_.FitColumnsWithNoLineWrap; // or FitColumnsToDisplayWidth
		//   grid.RowHeadersWidthChanged     += DataGridView_.FitColumnsWithNoLineWrap; // or FitColumnsToDisplayWidth
		//   grid.AutoSizeColumnsModeChanged += DataGridView_.FitColumnsWithNoLineWrap; // or FitColumnsToDisplayWidth
		//   grid.SizeChanged                += DataGridView_.FitColumnsWithNoLineWrap; // or FitColumnsToDisplayWidth
		//   grid.Scroll                     += DataGridView_.FitColumnsWithNoLineWrap; // or FitColumnsToDisplayWidth

		/// <summary>Options for smart column resizing</summary>
		[Flags] public enum EColumnSizeOptions
		{
			None = 0,

			/// <summary>Scale column widths based on their preferred width (which is derived from the cell contents)</summary>
			Preferred = 1 << 0,

			/// <summary>The text in column headers is included when determining the preferred size of a column</summary>
			IncludeColumnHeaders = 1 << 1,

			/// <summary>Columns widths may be expanded to fill the display width</summary>
			GrowToDisplayWidth = 1 << 2,

			/// <summary>Columns widths may be shrunk to fit to the display width</summary>
			ShrinkToDisplayWidth = 1 << 3,

			/// <summary>Columns widths may be expanded or shrunk to fit to the display width</summary>
			FitToDisplayWidth = GrowToDisplayWidth | ShrinkToDisplayWidth,
		}

		/// <summary>Returns an array of ideal widths for the columns based on visibility and FillWeight</summary>
		public static float[] GetColumnWidths(this DataGridView grid, EColumnSizeOptions opts)
		{
			var columns = grid.Columns.Cast<DataGridViewColumn>();

			// Get the visible width available to the columns.
			// Note, when scroll bars are visible, DisplayRectangle excludes their area
			var grid_width = GridDisplayWidth(grid);

			// Get the column fill weights
			var fill_weights = grid.FillWeights(zero_if_not_visible:true, normalised:true);

			// Generate a default set of widths
			// Note, fill weights are relative to the display area, not the area minus row headers if row headers are visible
			var widths = fill_weights.Select(x => x*grid_width).ToArray();

			// Measure each column's preferred width
			var pref_widths = widths.ToArray();
			using (var gfx = grid.CreateGraphics())
			{
				// Return the cell's preferred size
				Func<DataGridViewCell,float> MeasurePreferredWidth = (cell) =>
				{
					if (cell.Value is string text)
					{
						// For some reason, cell.GetPreferredSize or col.GetPreferredWidth don't return correct values
						var sz = gfx.MeasureString(text, cell.InheritedStyle.Font);
						var w = sz.Width + cell.InheritedStyle.Padding.Horizontal; 
						return Math_.Clamp(w, 30, 64000); // DGV throws if width is greater than 65535
					}
					else
					{
						// Default to the cell preferred size
						var sz = cell.PreferredSize;
						var w = sz.Width;
						return Math_.Clamp(w, 30, 64000); // DGV throws if width is greater than 65535
					}
				};

				// Measure the column header cells,
				if (opts.HasFlag(EColumnSizeOptions.IncludeColumnHeaders))
				{
					foreach (var col in columns.Where(x => x.Visible))
						pref_widths[col.Index] = Math.Max(pref_widths[col.Index], MeasurePreferredWidth(col.HeaderCell));
				}

				// Measure the cells in the displayed rows only
				foreach (var row in grid.GetRowsWithState(DataGridViewElementStates.Displayed))
				{
					for (int i = 0, iend = Math.Min(grid.ColumnCount, row.Cells.Count); i != iend; ++i)
					{
						if (!grid.Columns[i].Visible) continue;
						pref_widths[i] = Math.Max(pref_widths[i], MeasurePreferredWidth(row.Cells[i]));
					}
				}
			}

			// Adjust the values in 'widths' based on 'opts'
			if (opts.HasFlag(EColumnSizeOptions.Preferred))
			{
				var total_width = Math.Max(pref_widths.Sum(), 1f);

				// If the total preferred width fits within the display area, and we're allowed to expand the column widths
				// Or, if the total preferred width exceeds the display area, and we're allowed to shrink the column widths
				if ((total_width < grid_width && opts.HasFlag(EColumnSizeOptions.GrowToDisplayWidth  )) ||
					(total_width > grid_width && opts.HasFlag(EColumnSizeOptions.ShrinkToDisplayWidth)))
				{
					// Find the scaling factor that will fit all columns within the display area
					var scale = grid_width / total_width;
					for (int i = 0; i != pref_widths.Length; ++i)
						pref_widths[i] *= scale;
				}
				return pref_widths;
			}
			else
			{
				var total_width = Math.Max(widths.Sum(), 1f);

				// Otherwise, if the total current width fits within the display area, and we're allowed to expand the column widths
				// Or, if the total current width exceeds the display area, and we're allowed to shrink the column widths
				if ((total_width < grid_width && opts.HasFlag(EColumnSizeOptions.GrowToDisplayWidth  )) ||
					(total_width > grid_width && opts.HasFlag(EColumnSizeOptions.ShrinkToDisplayWidth)))
				{
					// Find the scaling factor that will fit all columns within the display area
					var scale = grid_width / total_width;
					for (int i = 0; i != widths.Length; ++i)
						widths[i] *= scale;
				}
				return widths;
			}
		}

		/// <summary>Resize the columns intelligently based on column content, visibility, and fill weights.</summary>
		public static void SetGridColumnSizes(this DataGridView grid, EColumnSizeOptions opts)
		{
			// To handle user resized columns, call SetFillWeightsOnColumnWidthChanged() in the OnColumnWidthChanged()
			// handler like this:
			//    if (!dgv.SettingGridColumnWidths())
			//    {
			//        dgv.SetFillWeightsOnColumnWidthChanged(a.Column.Index);
			//        dgv.SetGridColumnSizes();
			//    }
			//
			// See 'FitColumnsToDisplayWidth' for the case where all columns fit the available width,
			//  or 'FitColumnsWithNoLineWrap' for the log viewer case.

			// Prevent reentrancy
			var grid_state = GridState[grid];
			if (grid_state.InSetGridColumnSizes) return;
			using (Scope.Create(() => grid_state.InSetGridColumnSizes = true, () => grid_state.InSetGridColumnSizes = false))
			{
				// No columns, nothing to resize
				if (grid.ColumnCount == 0)
					return;

				// No resizing unless in AutoSizeColumnsMode.None
				if (grid.AutoSizeColumnsMode != DataGridViewAutoSizeColumnsMode.None)
					return;

				// Get the column widths
				var col_widths = Math_.TruncWithRemainder(grid.GetColumnWidths(opts)).ToArray();

				// Set the column sizes
				using (grid.SuspendLayout(true))
				{
					// Setting 'Width' will fire the OnColumnWidthChanged event
					// Callers should use 'SettingGridColumnWidths' to ignore these events
					foreach (var col in grid.Columns.Cast<DataGridViewColumn>())
						col.Width = col_widths[col.Index];
				}
			}
		}

		/// <summary>True while column widths are being set for this grid</summary>
		public static bool SettingGridColumnWidths(this DataGridView grid)
		{
			return GridState[grid].InSetGridColumnSizes;
		}

		/// <summary>
		/// An event handler that resizes the columns in a grid to fill the available space, while preserving user column size changes.
		/// Attach this handler to 'ColumnWidthChanged', 'RowHeadersWidthChanged', 'AutoSizeColumnsModeChanged', and 'SizeChanged'</summary>
		public static void FitColumnsToDisplayWidth(object sender, EventArgs? args = null)
		{
			var grid = (DataGridView)sender;
			var grid_state = GridState[grid];
			if (grid_state.InSetGridColumnSizes || grid_state.FitColumnsPending || !grid.IsHandleCreated)
				return;

			// Delay the column resize until the last triggering event (particularly scroll events)
			grid_state.FitColumnsPending = true;
			SynchronizationContext_.BeginInvokeDelayed(() =>
			{
				// If this event is a column/row header width changed event,
				// record the user column widths before resizing.
				if (args is DataGridViewColumnEventArgs a)
					grid.SetFillWeightsOnColumnWidthChanged(a.Column.Index);

				// Resize columns to fit
				grid.SetGridColumnSizes(EColumnSizeOptions.FitToDisplayWidth);
				grid_state.FitColumnsPending = false;
			}, TimeSpan.FromMilliseconds(10));
		}
		public static void FitColumnsToDisplayWidthAttach(this DataGridView grid)
		{
			grid.VisibleChanged             += FitColumnsToDisplayWidth;
			grid.ColumnWidthChanged         += FitColumnsToDisplayWidth;
			grid.RowHeadersWidthChanged     += FitColumnsToDisplayWidth;
			grid.AutoSizeColumnsModeChanged += FitColumnsToDisplayWidth;
			grid.SizeChanged                += FitColumnsToDisplayWidth;
			grid.Scroll                     += FitColumnsToDisplayWidth;
		}

		/// <summary>
		/// An event handler that resizes the columns in a grid to fill the available space and
		/// ensure each column does not wrap, while preserving user column size changes.
		/// Attach this handler to 'ColumnWidthChanged', 'RowHeadersWidthChanged', 'AutoSizeColumnsModeChanged', and 'SizeChanged'</summary>
		public static void FitColumnsWithNoLineWrap(object sender, EventArgs? args = null)
		{
			var grid = (DataGridView)sender;
			var grid_state = GridState[grid];
			if (grid_state.InSetGridColumnSizes || grid_state.FitColumnsPending || !grid.IsHandleCreated)
				return;
		
			// Delay the column resize until the last triggering event (particularly scroll events)
			grid_state.FitColumnsPending = true;
			SynchronizationContext_.BeginInvokeDelayed(() =>
			{
				// If this event is a column/row header width changed event,
				// record the user column widths before resizing.
				if (args is DataGridViewColumnEventArgs a)
					grid.SetFillWeightsOnColumnWidthChanged(a.Column.Index);

				// Resize columns to fit
				grid.SetGridColumnSizes(EColumnSizeOptions.GrowToDisplayWidth|EColumnSizeOptions.Preferred);
				grid_state.FitColumnsPending = false;
			}, TimeSpan.FromMilliseconds(10));
		}
		public static void FitColumnsWithNoLineWrapAttach(this DataGridView grid)
		{
			grid.VisibleChanged             += FitColumnsWithNoLineWrap;
			grid.ColumnWidthChanged         += FitColumnsWithNoLineWrap;
			grid.RowHeadersWidthChanged     += FitColumnsWithNoLineWrap;
			grid.AutoSizeColumnsModeChanged += FitColumnsWithNoLineWrap;
			grid.SizeChanged                += FitColumnsWithNoLineWrap;
			grid.Scroll                     += FitColumnsWithNoLineWrap;
		}

		/// <summary>An event handler for auto hiding the column header text when there is only one column visible in the grid</summary>
		public static void AutoHideSingleColumnHeader(object sender, EventArgs args)
		{
			var grid = (DataGridView)sender;
			grid.ColumnHeadersVisible = grid.ColumnCount > 1;
		}

		/// <summary>Attempts to scroll the grid to 'first_row_index' clamped by the number of rows in the grid</summary>
		public static void TryScrollToRowIndex(this DataGridView grid, int first_row_index)
		{
			if (grid.RowCount == 0) return;
			grid.FirstDisplayedScrollingRowIndex = Math_.Clamp(first_row_index, 0, grid.RowCount - 1);
		}

		/// <summary>RAII object for preserving the scroll position in a grid</summary>
		public static Scope ScrollScope(this DataGridView grid)
		{
			return Scope.Create(
			() => new Point(grid.FirstDisplayedScrollingColumnIndex, grid.FirstDisplayedScrollingRowIndex),
			pos =>
			{
				if (pos.X == -1 || pos.Y == -1) return;
				if (grid.ColumnCount == 0 || grid.RowCount == 0) return;
				grid.FirstDisplayedScrollingColumnIndex = Math.Min(pos.X, grid.ColumnCount - 1);
				grid.FirstDisplayedScrollingRowIndex    = Math.Min(pos.Y, grid.RowCount - 1);
			});
		}

		/// <summary>RAII object for preserving the selected cells/rows in a grid</summary>
		public static Scope SelectionScope(this DataGridView grid)
		{
			switch (grid.SelectionMode)
			{
			default:
				throw new Exception("Unknown DataGridView selection mode");

			case DataGridViewSelectionMode.CellSelect:
			case DataGridViewSelectionMode.ColumnHeaderSelect:
			case DataGridViewSelectionMode.RowHeaderSelect:
				return Scope.Create(
					() => grid.SelectedCells.Cast<DataGridViewCell>().Select(c => new {c.ColumnIndex, c.RowIndex}).ToArray(),
					cells =>
					{
						grid.ClearSelection();
						foreach (var c in cells)
						{
							if (c.ColumnIndex >= grid.ColumnCount) continue;
							if (c.RowIndex >= grid.RowCount) continue;
							grid[c.ColumnIndex, c.RowIndex].Selected = true;
						}
					});

			case DataGridViewSelectionMode.FullRowSelect:
				return Scope.Create(
					() => grid.GetRowsWithState(DataGridViewElementStates.Selected).Select(x => x.Index).ToArray(),
					row_indices =>
					{
						grid.ClearSelection();
						foreach (var r in row_indices)
						{
							if (r >= grid.RowCount) continue;
							grid.Rows[r].Selected = true;
						}
					});
			case DataGridViewSelectionMode.FullColumnSelect:
				return Scope.Create(
					() => grid.GetColumnsWithState(DataGridViewElementStates.Selected).Select(x => x.Index).ToArray(),
					col_indices =>
					{
						grid.ClearSelection();
						foreach (var c in col_indices)
						{
							if (c >= grid.ColumnCount) continue;
							grid.Columns[c].Selected = true;
						}
					});
			}
		}

		/// <summary>
		/// Sets the selection to row 'index' and the current cell to the first cell of that row.
		/// Clears all other selection.
		/// If the grid has rows, clamps 'index' to [-1,RowCount).
		/// If index == -1, the selection is cleared. Returns the row actually selected.
		/// If 'make_displayed' is true, scrolls the grid to make 'index' displayed</summary>
		public static int SelectSingleRow(this DataGridView grid, int index, bool make_displayed = false)
		{
			// Clear the current selection
			grid.ClearSelection();

			// If there are no rows, select nothing
			if (grid.RowCount == 0 || index == -1)
			{
				index = -1;
				grid.CurrentCell = null;
			}

			// Otherwise, select the row index and set the current cell
			else
			{
				index = Math_.Clamp(index, 0, grid.RowCount - 1);
				var row = grid.Rows[index];
				row.Selected = true;
				grid.CurrentCell = row.Cells[0];

				// Scroll the row into view
				if (make_displayed && !row.Displayed)
					grid.FirstDisplayedScrollingRowIndex = index;
			}
			return index;
		}

		/// <summary>Returns an enumerator for accessing rows with the given property.</summary>
		public static IEnumerable<DataGridViewRow> GetRowsWithState(this DataGridView grid, DataGridViewElementStates state)
		{
			for (var i = grid.Rows.GetFirstRow(state); i != -1; i = grid.Rows.GetNextRow(i, state))
				yield return grid.Rows[i];
		}

		/// <summary>Returns an enumerator for accessing columns with the given property</summary>
		public static IEnumerable<DataGridViewColumn> GetColumnsWithState(this DataGridView grid, DataGridViewElementStates state, DataGridViewElementStates excl = DataGridViewElementStates.None)
		{
			for (var i = grid.Columns.GetFirstColumn(state, excl); i != null; i = grid.Columns.GetNextColumn(i, state, excl))
				yield return i;
		}

		/// <summary>Return the number of selected rows (up to 'max_count') (i.e. efficiently test for multiple selections)</summary>
		public static int SelectedRowCount(this DataGridView grid, int max_count = int.MaxValue)
		{
			return grid.SelectedRows().CountAtMost(max_count);
		}

		/// <summary>Return the number of selected columns (up to 'max_count') (i.e. efficiently test for multiple selections)</summary>
		public static int SelectedColumnCount(this DataGridView grid, int max_count = int.MaxValue)
		{
			return grid.SelectedColumns().CountAtMost(max_count);
		}

		/// <summary>Return the selected rows. Note: 'SelectedRows' allocates and populates a collection on every call! This is way more efficient</summary>
		public static IEnumerable<DataGridViewRow> SelectedRows(this DataGridView grid)
		{
			foreach (var r in grid.GetRowsWithState(DataGridViewElementStates.Selected))
				yield return r;
		}

		/// <summary>Return the selected columns. Note: 'SelectedColumns' allocates and populates a collection on every call! This is way more efficient</summary>
		public static IEnumerable<DataGridViewColumn> SelectedColumns(this DataGridView grid)
		{
			foreach (var c in grid.GetColumnsWithState(DataGridViewElementStates.Selected))
				yield return c;
		}

		/// <summary>Return the indices of the selected rows. Note: 'SelectedRows' allocates and populates a collection on every call! This is way more efficient</summary>
		public static IEnumerable<int> SelectedRowIndices(this DataGridView grid)
		{
			foreach (var r in grid.SelectedRows())
				yield return r.Index;
		}

		/// <summary>Return the indices of the selected columns. Note: 'SelectedColumns' allocates and populates a collection on every call! This is way more efficient</summary>
		public static IEnumerable<int> SelectedColumnIndices(this DataGridView grid)
		{
			foreach (var r in grid.SelectedColumns())
				yield return r.Index;
		}

		/// <summary>Return the index of the first selected row (or -1 if no rows are selected). This is the selected row with the minimum row index, *not* the same as SelectedRows[0]</summary>
		public static int FirstSelectedRowIndex(this DataGridView grid)
		{
			return grid.Rows.GetFirstRow(DataGridViewElementStates.Selected);
		}

		/// <summary>Return the index of the last selected row (or -1 if no rows are selected). This is the selected row with the maximum row index, *not* the same as SelectedRows[count-1]</summary>
		public static int LastSelectedRowIndex(this DataGridView grid)
		{
			return grid.Rows.GetLastRow(DataGridViewElementStates.Selected);
		}

		/// <summary>Return the (inclusive) bounds of the selected rows (or [-1,-1] if no rows are selected). Warning: 'Empty' means only 1 row is selected (unless it's -1). Note: independent of the order of SelectedRows</summary>
		public static RangeI SelectedRowIndexRange(this DataGridView grid)
		{
			return new RangeI(grid.FirstSelectedRowIndex(), grid.LastSelectedRowIndex());
		}

		/// <summary>Return the first selected row, regardless of multi-select grids</summary>
		public static DataGridViewRow? FirstSelectedRow(this DataGridView grid)
		{
			var i = grid.FirstSelectedRowIndex();
			return i != -1 ? grid.Rows[i] : null;
		}

		/// <summary>Return the last selected row, regardless of multi-select grids</summary>
		public static DataGridViewRow? LastSelectedRow(this DataGridView grid)
		{
			var i = grid.LastSelectedRowIndex();
			return i != -1 ? grid.Rows[i] : null;
		}

		/// <summary>Checks if the given column/row are within the grid and returns the associated column and cell</summary>
		public static bool Within(this DataGridView grid, int column_index, int row_index, out DataGridViewColumn? col, out DataGridViewCell? cell)
		{
			col = null; cell = null;
			if (column_index < 0 || column_index >= grid.ColumnCount) return false;
			if (row_index    < 0 || row_index    >= grid.RowCount) return false;
			col = grid.Columns[column_index];
			cell = grid[column_index, row_index];
			return true;
		}

		/// <summary>Checks if the given column/row are within the grid and returns the associated column</summary>
		public static bool Within(this DataGridView grid, int column_index, int row_index, out DataGridViewColumn? col)
		{
			DataGridViewCell? dummy;
			return Within(grid, column_index, row_index, out col, out dummy);
		}

		/// <summary>Checks if the given column/row are within the grid and returns the associated cell</summary>
		public static bool Within(this DataGridView grid, int column_index, int row_index, out DataGridViewCell? cell)
		{
			DataGridViewColumn? dummy;
			return Within(grid, column_index, row_index, out dummy, out cell);
		}

		/// <summary>Checks if the given column/row are within the grid</summary>
		public static bool Within(this DataGridView grid, int column_index, int row_index)
		{
			DataGridViewCell? dummy;
			return Within(grid, column_index, row_index, out dummy);
		}

		/// <summary>Checks if the given row index is within the range of grid rows</summary>
		public static bool WithinRows(this DataGridView grid, int row_index)
		{
			return row_index >= 0 && row_index < grid.RowCount;
		}

		/// <summary>Checks if the given column index is within the range of grid columns</summary>
		public static bool WithinCols(this DataGridView grid, int col_index)
		{
			return col_index >= 0 && col_index < grid.ColumnCount;
		}

		/// <summary>Return this content alignment value adjusted by 'horz' and 'vert'. If non-null, then -1 = Left/Top, 0 = Middle/Centre, +1 = Right/Bottom</summary>
		public static DataGridViewContentAlignment Shift(this DataGridViewContentAlignment align, int? horz = null, int? vert = null)
		{
			// 'align' is a single bit
			int x = 0, y = 0;

			if (horz != null) x = horz.Value + 1;
			else if (Bit.AnySet((int)align ,(int)(DataGridViewContentAlignment.TopLeft  |DataGridViewContentAlignment.MiddleLeft  |DataGridViewContentAlignment.BottomLeft  ))) x = 0;
			else if (Bit.AnySet((int)align ,(int)(DataGridViewContentAlignment.TopCenter|DataGridViewContentAlignment.MiddleCenter|DataGridViewContentAlignment.BottomCenter))) x = 1;
			else if (Bit.AnySet((int)align ,(int)(DataGridViewContentAlignment.TopRight |DataGridViewContentAlignment.MiddleRight |DataGridViewContentAlignment.BottomRight ))) x = 2;

			if (vert != null) y = vert.Value + 1;
			else if (Bit.AnySet((int)align ,(int)(DataGridViewContentAlignment.TopLeft   |DataGridViewContentAlignment.TopCenter   |DataGridViewContentAlignment.TopRight   ))) y = 0;
			else if (Bit.AnySet((int)align ,(int)(DataGridViewContentAlignment.MiddleLeft|DataGridViewContentAlignment.MiddleCenter|DataGridViewContentAlignment.MiddleRight))) y = 1;
			else if (Bit.AnySet((int)align ,(int)(DataGridViewContentAlignment.BottomLeft|DataGridViewContentAlignment.BottomCenter|DataGridViewContentAlignment.BottomRight))) y = 2;

			return (DataGridViewContentAlignment)(1 << (y*3 + x));
		}

		/// <summary>Return the bounds of this cell in DGV space</summary>
		public static Rectangle CellBounds(this DataGridViewCell cell, bool cut_overflow)
		{
			return cell.DataGridView.GetCellDisplayRectangle(cell.ColumnIndex, cell.RowIndex, cut_overflow);
		}

		/// <summary>Temporarily remove the data source from this grid</summary>
		public static Scope PauseBinding(this DataGridView grid)
		{
			var ds = grid.DataSource;
			return Scope.Create(() => grid.DataSource = null, () => grid.DataSource = ds);
		}

		#region Hit Test

		/// <summary>Hit test the grid (includes more useful information than the normal hit test). 'pt' is in grid-relative coordinates</summary>
		public static HitTestInfo HitTestEx(this DataGridView grid, Point pt)
		{
			return new HitTestInfo(grid, pt);
		}
		public static HitTestInfo HitTestEx(this DataGridView grid, int x, int y)
		{
			return new HitTestInfo(grid, new Point(x, y));
		}
		public class HitTestInfo
		{
			public enum EType
			{
				None                = DataGridViewHitTestType.None                ,
				Cell                = DataGridViewHitTestType.Cell                ,
				ColumnHeader        = DataGridViewHitTestType.ColumnHeader        ,
				RowHeader           = DataGridViewHitTestType.RowHeader           ,
				TopLeftHeader       = DataGridViewHitTestType.TopLeftHeader       ,
				HorizontalScrollBar = DataGridViewHitTestType.HorizontalScrollBar ,
				VerticalScrollBar   = DataGridViewHitTestType.VerticalScrollBar   ,
				ColumnDivider,
				RowDivider,
			}

			private readonly DataGridView.HitTestInfo m_info;
			public HitTestInfo(DataGridView grid, Point pt)
			{
				Grid = grid;
				GridPoint = pt;
				m_info = grid.HitTest(pt.X, pt.Y);
				Type = (EType)m_info.Type;

				// Check for clicks on column dividers
				var col = Column;
				if (col != null && m_info.Type == DataGridViewHitTestType.ColumnHeader && grid.AllowUserToResizeColumns)
				{
					if (grid.Cursor == Cursors.SizeWE)
						Type = EType.ColumnDivider;
				}

				// Check for clicks on row dividers
				var row = Row;
				if (row != null && m_info.Type == DataGridViewHitTestType.RowHeader && grid.AllowUserToResizeRows)
				{
					if (grid.Cursor == Cursors.SizeNS)
						Type = EType.RowDivider;
				}
			}

			/// <summary>The grid that was hit tested</summary>
			public DataGridView Grid { get; private set; }

			/// <summary>The grid relative location of where the hit test was performed</summary>
			public Point GridPoint { get; private set; }

			/// <summary>The cell relative location of where the hit test was performed</summary>
			public Point CellPoint { get { return new Point(GridPoint.X - ColumnX, GridPoint.Y - RowY); } }

			/// <summary>What was hit</summary>
			public EType Type
			{
				get;
				private set;
			}

			/// <summary>True if a column or row divider was hit</summary>
			public bool Divider
			{
				get
				{
					var fi = m_info.GetType().GetField("typeInternal", BindingFlags.Instance | BindingFlags.NonPublic);
					var value = fi.GetValue(m_info).ToString();
					return
						value.Equals("RowResizeTop") || value.Equals("RowResizeBottom") ||
						value.Equals("ColumnResizeLeft") || value.Equals("ColumnResizeRight");
				}
			}

			/// <summary>Gets the index of the column that contains the hit test point</summary>
			public int ColumnIndex
			{
				get { return m_info.ColumnIndex; }
			}

			/// <summary>Gets the index of the row that contains the hit test point</summary>
			public int RowIndex
			{
				get { return m_info.RowIndex; }
			}

			/// <summary>Gets the x-coordinate of the column 'ColumnIndex'</summary>
			public int ColumnX
			{
				get { return m_info.ColumnX; }
			}

			/// <summary>Gets the y-coordinate of the row 'RowIndex'</summary>
			public int RowY
			{
				get { return m_info.RowY; }
			}

			/// <summary>The X,Y grid-relative coordinate of the hit cell</summary>
			public Point CellPosition
			{
				get { return new Point(ColumnX, RowY); }
			}

			/// <summary>
			/// Gets the index of the row that is closest to the hit location.
			/// If the mouse is over the top half of a row, then the row index is returned.
			/// If the mouse is over the lower half of the row, then the next row index is returned.</summary>
			public int InsertIndex
			{
				get
				{
					if (Row == null) return RowIndex;
					var over_half = GridPoint.Y - RowY > 0.5f * Row.Height;
					return RowIndex + (over_half ? 1 : 0);
				}
			}

			/// <summary>The y-coordinate of the top of the insert row</summary>
			public int InsertY
			{
				get
				{
					if (Row == null) return RowY;
					var over_half = GridPoint.Y - RowY > 0.5f * Row.Height;
					return RowY + (over_half ? Row.Height : 0);
				}
			}

			/// <summary>Get the hit header cell (or null)</summary>
			public DataGridViewHeaderCell? HeaderCell
			{
				get { return RowIndex == -1 ? Grid.Columns[ColumnIndex].HeaderCell : null; }
			}

			/// <summary>Get the hit data cell (or null)</summary>
			public DataGridViewCell? Cell
			{
				get
				{
					DataGridViewCell? cell;
					return Grid.Within(ColumnIndex, RowIndex, out cell) ? cell : null;
				}
			}

			/// <summary>Get the hit Column (or null)</summary>
			public DataGridViewColumn? Column
			{
				get { return ColumnIndex >= 0 && ColumnIndex < Grid.ColumnCount ? Grid.Columns[ColumnIndex] : null; }
			}

			/// <summary>Get the hit row (or null)</summary>
			public DataGridViewRow? Row
			{
				get { return RowIndex >= 0 && RowIndex < Grid.RowCount ? Grid.Rows[RowIndex] : null; }
			}

			/// <summary>Implicit conversion to 'DataGridView.HitTestInfo'</summary>
			public static implicit operator DataGridView.HitTestInfo(HitTestInfo hit) { return hit.m_info; }
		}

		#endregion

		#region Drag/Drop

		/// <summary>Data used for dragging rows around</summary>
		public class DragDropData :IDisposable
		{
			public DragDropData(DataGridViewRow row, int x, int y)
			{
				Row = row;
				GrabX = x;
				GrabY = y;
				StartRowIndex = row.Index;
				Dragging = false;
				Indicator = new IndicatorCtrl(row.DataGridView.TopLevelControl as Form);
			}
			public void Dispose()
			{
				Indicator?.Close();
				Indicator = null;
			}

			/// <summary>The grid that owns 'Row'</summary>
			public System.Windows.Forms.DataGridView? DataGridView => Row?.DataGridView;

			/// <summary>The row being dragged</summary>
			public DataGridViewRow Row { get; private set; }

			/// <summary>The pixel location of where the row was grabbed (in DGV space)</summary>
			public int GrabX { get; private set; }
			public int GrabY { get; private set; }

			/// <summary>The row index of 'Row' when the drag operation began</summary>
			public int StartRowIndex { get; private set; }

			/// <summary>True once the mouse has moved enough to be detected as a drag operation</summary>
			public bool Dragging { get; set; }

			/// <summary>A form used to indicate where the row will be inserted</summary>
			public IndicatorCtrl? Indicator { get; private set; }
			public class IndicatorCtrl :Form
			{
				public IndicatorCtrl(Form? owner)
				{
					SetStyle(ControlStyles.Selectable, false);
					FormBorderStyle = FormBorderStyle.None;
					StartPosition = FormStartPosition.Manual;
					ShowInTaskbar = false;
					Size = new Size(10,10);
					Ofs = new Size(0, 5);
					Owner = owner;
					Region = Gdi.MakeRegion(Size.Width,Size.Height,  Ofs.Width,Ofs.Height,  Size.Width,0);
					CreateHandle();
				}
				protected override CreateParams CreateParams
				{
					get
					{
						var cp = base.CreateParams;
						cp.ClassStyle |= Win32.CS_DROPSHADOW;
						cp.Style &= ~Win32.WS_VISIBLE;
						cp.ExStyle |= Win32.WS_EX_NOACTIVATE;
						return cp;
					}
				}
				protected override bool ShowWithoutActivation
				{
					get { return true; }
				}
				protected override void OnPaint(PaintEventArgs e)
				{
					base.OnPaint(e);
					e.Graphics.FillRectangle(Brushes.DeepSkyBlue, ClientRectangle);
				}
				public new Point Location
				{
					get { return base.Location + Ofs; }
					set { base.Location = value - Ofs; }
				}
				public Size Ofs { get; private set; }
			}
		}

		/// <summary>
		/// Begin a row drag-drop operation on the grid. Works on RowHeaders.
		/// Attach this method to the 'MouseDown' event on the grid.
		/// Note: Do not attach to 'MouseMove' as well, DoDragDrop handles mouse move
		/// Also, attach 'DragDrop_DoDropMoveRow' to the 'DoDrop' handler and set AllowDrop = true.
		/// See the 'DragDrop' class for more info</summary>
		public static void DragDrop_DragRow(object sender, MouseEventArgs e)
		{
			var grid = (DataGridView)sender;
			var row_count = grid.RowCount - (grid.AllowUserToAddRows ? 1 : 0);

			// Drag by grabbing row headers
			// Only drag if:
			//  A data row header is hit (i.e. not the 'add' row, or other parts of the grid)
			//  The resize cursor isn't visible
			//  No control keys are down
			var grid_pt = grid.PointToClient(Control.MousePosition);
			var hit = grid.HitTest(grid_pt.X, grid_pt.Y);
			if (hit.Type == DataGridViewHitTestType.RowHeader &&
				hit.RowIndex >= 0 && hit.RowIndex < row_count &&
				grid.Cursor != Cursors.SizeNS &&
				Control.ModifierKeys == Keys.None)
			{
				// The grid.MouseDown event calls CellMouseDown (and others) after the drag/drop operation has
				// finished. A consequence is the cell that the drag started from gets 'clicked' causing the selection
				// to change back to that cell. BeginInvoke ensures the drag happens after any selection changes.
				Dispatcher.CurrentDispatcher.BeginInvoke(new Action(() =>
				{
					using (var data = new DragDropData(grid.Rows[hit.RowIndex], grid_pt.X, grid_pt.Y))
						grid.DoDragDrop(data, DragDropEffects.Move|DragDropEffects.Copy|DragDropEffects.Link);
				}));
			}
		}

		/// <summary>
		/// A drag drop function for moving a row in a grid to a new position.
		/// Attach this method to the 'DoDrop' handler.
		/// Also, attach 'DragDrop_DragRow' to the MouseDown event and set AllowDrop = true on the grid.
		/// See the 'DragDrop' class for more info.</summary>
		public static bool DragDrop_DoDropMoveRow(object sender, DragEventArgs args, DragDrop.EDrop mode)
		{
			return DragDrop_DoDropMoveRow(sender, args, mode, (grid, grab, hit) =>
			{
				// Insert 'grab_idx' at 'drop_idx'
				var list = grid.DataSource as IList;
				if (list == null)
					throw new InvalidOperationException("Drag-drop requires a grid with a data source bound to an IList");

				var grab_idx = grab.Row.Index;
				var drop_idx = hit.InsertIndex;
				if (grab_idx != drop_idx)
				{
					// Allow for the index value to change when 'grap_idx' is removed
					if (drop_idx > grab_idx)
						--drop_idx;

					var tmp = list[grab_idx];
					list.RemoveAt(grab_idx);
					list.Insert(drop_idx, tmp);
				}

				// If the list is a binding source, set the current position to the item just moved
				var cm = grid.DataSource as ICurrencyManagerProvider;
				if (cm != null)
					cm.CurrencyManager.Position = drop_idx;
			});
		}
		public static bool DragDrop_DoDropMoveRow(object sender, DragEventArgs args, DragDrop.EDrop mode, Action<DataGridView, DragDropData, HitTestInfo> OnDrop)
		{
			// Must allow move
			if (!args.AllowedEffect.HasFlag(DragDropEffects.Move))
				return false;

			// This method could be hooked up to a Rylogic.Gui.DragDrop so the events could come from anything.
			// Only accept a DGV that contains the row in 'data'
			var grid = sender as DataGridView;
			var data = (DragDropData)args.Data.GetData(typeof(DragDropData));
			if (grid == null || data == null || !ReferenceEquals(grid, data.DataGridView))
				return false;

			var row_count = grid.RowCount - (grid.AllowUserToAddRows ? 1 : 0);
			var scn_pt = new Point(args.X, args.Y);
			var pt = grid.PointToClient(scn_pt); // Find where the mouse is over the grid

			// Check the mouse has moved enough to start dragging
			if (!data.Dragging)
			{
				var distsq = Math_.LengthSq(pt.X - data.GrabX, pt.Y - data.GrabY);
				if (distsq < 25) return false;
				data.Dragging = true;
			}

			// Set the drop effect
			var hit = grid.HitTestEx(pt.X, pt.Y);
			args.Effect =
				hit.Type == HitTestInfo.EType.RowHeader &&
				hit.RowIndex >= 0 && hit.RowIndex < row_count && hit.RowIndex != data.Row.Index
				? DragDropEffects.Move : DragDropEffects.None;
			if (args.Effect != DragDropEffects.Move)
				return true;

			// If this is not the actual drop then just update the indicator position
			if (mode != DragDrop.EDrop.Drop)
			{
				// Ensure the indicator is visible
				if (data.Indicator != null)
				{
					data.Indicator.Visible = true;
					data.Indicator.Location = grid.PointToScreen(new Point(0, hit.InsertY));
				}

				// Auto scroll when at the first or last displayed row
				var idx = hit.InsertIndex;
				if (idx <= grid.FirstDisplayedScrollingRowIndex && idx > 0)
					grid.FirstDisplayedScrollingRowIndex--;
				if (idx >= grid.FirstDisplayedScrollingRowIndex + grid.DisplayedRowCount(false) && idx < row_count)
					grid.FirstDisplayedScrollingRowIndex++;
			}

			// Otherwise, this is the drop
			else
			{
				using (grid.SuspendLayout(false))
				{
					// Handle the drop action
					OnDrop(grid, data, hit);

					grid.Invalidate(true);
				}
			}
			return true;
		}

		#endregion

		#region Filter Columns

		/// <summary>Per-grid object for managing column filters</summary>
		public class ColumnFiltersData :IMessageFilter ,IDisposable
		{
			/// <summary>The grid whose data is to be filtered</summary>
			private DataGridView m_dgv;

			/// <summary>Preserves the state of the DGV</summary>
			private Dictionary<string, object> m_dgv_state;

			/// <summary>The data source that the grid had before filters were used</summary>
			private object? m_original_src;

			public ColumnFiltersData(DataGridView dgv)
			{
				Debug.Assert(dgv.DataSource != null, "Column filters requires a data source");

				m_dgv          = dgv;
				m_dgv_state    = new Dictionary<string, object>();
				m_header_cells = new FilterHeaderCell[dgv.ColumnCount];
				m_original_src = dgv.DataSource;
				BSFilter       = null;
				ShortcutKey    = Keys.F;

				m_dgv.DataSourceChanged += HandleDataSourceChanged;
				HandleDataSourceChanged();
			}
			public void Dispose()
			{
				Enabled = false;
				m_dgv.DataSourceChanged -= HandleDataSourceChanged;
				m_header_cells = Util.DisposeAll(m_header_cells)!;
				BSFilter = null;
			}

			/// <summary>The column header cells corresponding to the columns in the associated grid</summary>
			public FilterHeaderCell[] HeaderCells { get { return m_header_cells; } }
			private FilterHeaderCell[] m_header_cells;

			/// <summary>Contains a BindingSource<> (created from the given DGV) and creates 'Views' of the binding source based on filter predicates</summary>
			private BindingSourceFilter? BSFilter
			{
				get;
				set
				{
					if (field == value) return;
					Util.Dispose(ref field!);
					field = value;
				}
			}
			private class BindingSourceFilter :IDisposable
			{
				/// <summary>The bound data properties for the columns</summary>
				private Dictionary<string,MethodInfo> m_elem_props;

				/// <summary>The method on BindingSource<> for creating views</summary>
				private MethodInfo m_create_view;

				/// <summary>The binding source instance from which we create filtered views</summary>
				private object m_bs;

				public BindingSourceFilter(DataGridView dgv)
				{
					var original_src = dgv.DataSource ?? throw new NullReferenceException("DGV requires a data source ");
					var orig_ty = original_src.GetType();

					// Get the element properties for each column
					var elem_ty = GetElementType(original_src);
					m_elem_props = new Dictionary<string, MethodInfo>();
					foreach (var col in dgv.Columns.OfType<DataGridViewColumn>())
					{
						// Silently ignores 'DataPropertyName' values that aren't valid properties on the element type
						if (!col.DataPropertyName.HasValue()) continue;
						var mi = elem_ty.GetProperty(col.DataPropertyName)?.GetGetMethod();
						if (mi == null) continue;
						m_elem_props[col.DataPropertyName] = mi;
					}

					// Get the binding source type we need to make views from
					var bs_ty = typeof(BindingSource<>).MakeGenericType(elem_ty);

					// If the original source is a BindingSource<>, use it otherwise create a binding source with 'original_src' as it's data source
					m_bs = orig_ty == bs_ty ? original_src : Activator.CreateInstance(bs_ty, original_src, (string?)null);

					// Get the CreateView method on the binding source
					m_create_view = bs_ty.GetMethod(nameof(BindingSource<int>.CreateView), new[] { typeof(Func<,>).MakeGenericType(elem_ty, typeof(bool)) });
				}
				public void Dispose()
				{
					FilteredView = null!;
				}

				/// <summary>The filtered view of the binding source</summary>
				public object FilteredView
				{
					get;
					set
					{
						if (field == value) return;
						if (field != null) Util.Dispose((IDisposable)field);
						field = value;
					}
				} = null!;

				/// <summary>Get the element type from 'original_src'</summary>
				private Type GetElementType(object original_src)
				{
					var src_ty = original_src.GetType();

					// Array/pointer source
					if (src_ty.HasElementType)
						return src_ty.GetElementType();

					// An IList<> source
					var face = src_ty.GetInterfaces().FirstOrDefault(x => x.IsGenericType && x.GetGenericTypeDefinition() == typeof(IList<>));
					if (face != null)
						return face.GetGenericArguments()[0];

					// A non-empty IList source
					var list = original_src as IList;
					if (list != null && list.Count != 0)
						return list[0].GetType();

					// Dunno, but the grid seems to know
					//if (dgv.RowCount != 0 && dgv.Rows[0].DataBoundItem != null)
					//	return dgv.Rows[0].DataBoundItem.GetType();

					throw new Exception($"Could not determine the element type for the data source type {original_src.GetType().Name}");
				}

				/// <summary>Create a view of the data from the binding source using the given filter</summary>
				public void SetFilter(Func<object,bool> filter)
				{
					FilteredView = m_create_view.Invoke(m_bs, new object[] { filter });
				}

				/// <summary>Return the value of property 'prop_name' on 'item' as a string</summary>
				public string GetValue(object item, string prop_name)
				{
					MethodInfo mi;
					return m_elem_props.TryGetValue(prop_name, out mi) ? mi.Invoke(item, null).ToString() : string.Empty;
				}
			}

			/// <summary>Enable/Disable the column filters</summary>
			public bool Enabled
			{
				get { return m_enabled; }
				set
				{
					if (m_enabled == value) return;
					m_enabled = value;
					if (m_enabled)
					{
						// Save the data source that the grid starts with
						m_original_src = m_dgv.DataSource;

						// Preserve the current state of the DGV
						m_dgv_state[nameof(m_dgv.ColumnHeadersHeightSizeMode)]   = m_dgv.ColumnHeadersHeightSizeMode;
						m_dgv_state[nameof(m_dgv.ColumnHeadersHeight)]           = m_dgv.ColumnHeadersHeight;
						m_dgv_state[nameof(m_dgv.ColumnHeadersDefaultCellStyle)] = m_dgv.ColumnHeadersDefaultCellStyle;

						// Set the column header height and text alignment
						m_dgv.ColumnHeadersHeightSizeMode = DataGridViewColumnHeadersHeightSizeMode.DisableResizing;
						m_dgv.ColumnHeadersHeight += FilterHeaderCell.FieldHeight;
						m_dgv.ColumnHeadersDefaultCellStyle = new DataGridViewCellStyle(m_dgv.ColumnHeadersDefaultCellStyle)
						{
							Alignment = m_dgv.ColumnHeadersDefaultCellStyle.Alignment.Shift(vert:-1), // Align to the top of the cell
						};

						// Ensure the array of header cells equals the number of columns
						if (m_header_cells.Length > m_dgv.ColumnCount)
							Util.DisposeRange(m_header_cells, m_dgv.ColumnCount, m_header_cells.Length - m_dgv.ColumnCount);
						Array.Resize(ref m_header_cells, m_dgv.ColumnCount);

						// Create header cells for the current columns in the grid
						foreach (var col in m_dgv.Columns.OfType<DataGridViewColumn>())
						{
							m_header_cells[col.Index] = m_header_cells[col.Index] ?? new FilterHeaderCell(this, col.HeaderCell, OnPatternChanged);
							col.HeaderCell = m_header_cells[col.Index];
						}

						// Shift focus to the filter for the current cell
						if (m_dgv.CurrentCell != null)
							m_header_cells[m_dgv.CurrentCell.ColumnIndex].EditFilter();

						// Update the binding source for the grid to the filtered source
						OnPatternChanged();

						// Start watching for mouse clicks on the filter fields
						Application.AddMessageFilter(this);
					}
					else
					{
						// Stop watching for mouse clicks on the search field
						Application.RemoveMessageFilter(this);

						// Restore the original column header cells
						foreach (var col in m_dgv.Columns.OfType<DataGridViewColumn>())
						{
							var cell = (FilterHeaderCell)col.HeaderCell;
							col.HeaderCell = cell.OriginalHeaderCell;
						}

						// Restore the DGV state
						if (m_dgv.IsHandleCreated)
						{
							m_dgv.ColumnHeadersHeight           = (int)m_dgv_state[nameof(m_dgv.ColumnHeadersHeight)];
							m_dgv.ColumnHeadersHeightSizeMode   = (DataGridViewColumnHeadersHeightSizeMode)m_dgv_state[nameof(m_dgv.ColumnHeadersHeightSizeMode)];
							m_dgv.ColumnHeadersDefaultCellStyle = (DataGridViewCellStyle)m_dgv_state[nameof(m_dgv.ColumnHeadersDefaultCellStyle)];
						}

						// Restore back to the original data source
						m_dgv.DataSource = m_original_src;
					}
				}
			}
			private bool m_enabled;

			/// <summary>The keyboard shortcut that enables/disables filters. Defaults to 'Ctrl+F'</summary>
			public Keys ShortcutKey { get; set; }

			/// <summary>
			/// Pre-filter mouse down messages so we can intercept clicks on the filter field before
			/// the grid handles them as column clicks</summary>
			public bool PreFilterMessage(ref Message m)
			{
				// Watch for mouse down events for our grid
				if (m.HWnd == m_dgv.Handle && m.Msg == Win32.WM_LBUTTONDOWN)
				{
					// If the mouse down position is within a FilterHeaderCell
					var pt = Win32.LParamToPoint(m.LParam);
					var hit = m_dgv.HitTestEx(pt);
					if (hit.Type == HitTestInfo.EType.ColumnHeader && hit.HeaderCell is FilterHeaderCell)
					{
						// If the mouse down position is within the filter field of the header cell
						var filter_cell = (FilterHeaderCell)hit.HeaderCell;
						var bounds = filter_cell.FieldBounds.Shifted(hit.CellPosition);
						if (bounds.Contains(pt))
						{
							// Edit the filter field
							// BeginInvoke so that Editing starts after the message queue has processed the mouse events
							Dispatcher.CurrentDispatcher.BeginInvoke(new Action(filter_cell.EditFilter));
							m_eat_lbuttonup = true;
							return true;
						}
					}
				}
				// Consume the mouse up event after an edit filter click
				if (m.HWnd == m_dgv.Handle && m.Msg == Win32.WM_LBUTTONUP && m_eat_lbuttonup)
				{
					m_eat_lbuttonup = false;
					return true;
				}
				return false;
			}
			private bool m_eat_lbuttonup;

			/// <summary>Handle the filter string changing in one of the FilterHeaderCells</summary>
			private void OnPatternChanged(object? sender = null, EventArgs? args = null)
			{
				// If enabled, use the filtered binding source, otherwise use the original source
				if (Enabled && BSFilter != null)
				{
					try
					{
						// Create a view filter function
						Func<object,bool> filter = item =>
						{
							for (var i = 0; i != m_header_cells.Length; ++i)
							{
								// If the pattern for this column is not valid, assume no filter
								var patn = m_header_cells[i]?.Pattern;
								if (patn == null || !patn.IsValid || !patn.Expr.HasValue()) continue;

								// Read the value from the source
								var val = BSFilter.GetValue(item, m_header_cells[i].OwningColumn.DataPropertyName);
								var str = val.ToString();

								// Filter out the item if it doesn't match the pattern
								if (!patn.IsMatch(str))
									return false;
							}
							return true;
						};

						// Set the view as the data source for the grid
						BSFilter.SetFilter(filter);
						m_dgv.DataSource = BSFilter.FilteredView;
					}
					catch (Exception ex)
					{
						throw new Exception("Failed to create a binding source view with the given column filters", ex);
					}
				}
				else
				{
					m_dgv.DataSource = m_original_src;
				}
			}

			/// <summary>If the data source on the DGV changes, we need to reset the BSFilter</summary>
			private void HandleDataSourceChanged(object? sender = null, EventArgs? e = null)
			{
				// Remember, while filtering is enabled the DGV's data source will be 'm_bs.FilteredView'
				if (BSFilter != null && m_dgv.DataSource == BSFilter.FilteredView)
				{}

				// If we don't currently have a 'bs_filter', and the grid has a data source, then create one
				else if (BSFilter == null && m_dgv.DataSource != null)
					BSFilter = new BindingSourceFilter(m_dgv);

				// Otherwise, if the grid no longer has a source, release our runtime data source
				else if (BSFilter != null && m_dgv.DataSource == null)
					BSFilter = null;
			}

			/// <summary>Custom column header cell for showing the filter string text box</summary>
			public class FilterHeaderCell :DataGridViewColumnHeaderCell
			{
				public const int FieldHeight = 18;

				public FilterHeaderCell(ColumnFiltersData cfd, DataGridViewColumnHeaderCell header_cell, EventHandler? on_pattern_changed = null)
				{
					OriginalHeaderCell = header_cell;
					Pattern = new Pattern(EPattern.Substring, string.Empty) { IgnoreCase = true };
					Value = OriginalHeaderCell.Value;
					EditCtrl = new EditControl(cfd);

					ContextMenuStrip = new ContextMenuStrip();
					ContextMenuStrip.Items.Add2("Substring", null, (s,a) => Pattern.PatnType = EPattern.Substring        );
					ContextMenuStrip.Items.Add2("Wildcard" , null, (s,a) => Pattern.PatnType = EPattern.Wildcard         );
					ContextMenuStrip.Items.Add2("Regex"    , null, (s,a) => Pattern.PatnType = EPattern.RegularExpression);
					ContextMenuStrip.Items.Add2(new ToolStripSeparator());
					ContextMenuStrip.Items.Add2("Properties", null, (s,a) => ShowPatternUI());
					ContextMenuStrip.Opening += (s,a) =>
					{
						((ToolStripMenuItem)ContextMenuStrip.Items[0]).Checked = Pattern.PatnType == EPattern.Substring        ;
						((ToolStripMenuItem)ContextMenuStrip.Items[1]).Checked = Pattern.PatnType == EPattern.Wildcard         ;
						((ToolStripMenuItem)ContextMenuStrip.Items[2]).Checked = Pattern.PatnType == EPattern.RegularExpression;
					};

					if (on_pattern_changed != null)
						PatternChanged += on_pattern_changed;
				}
				protected override void Dispose(bool disposing)
				{
					Pattern = null!;
					Util.Dispose(EditCtrl);
					base.Dispose(disposing);
				}

				/// <summary>The column header cell that this cell replaced</summary>
				public DataGridViewColumnHeaderCell OriginalHeaderCell { get; private set; }

				/// <summary>Returns the cell-relative area of the search field</summary>
				public Rectangle FieldBounds
				{
					get { return new Rectangle(1, Size.Height - FieldHeight - 2, Size.Width - 3, FieldHeight); }
				}

				/// <summary>The search text for this column</summary>
				public Pattern Pattern
				{
					get;
					set
					{
						if (field == value) return;
						if (field != null)
						{
							field.PropertyChanged -= HandlePatternChanged;
						}
						field = value;
						if (field != null)
						{
							field.PropertyChanged += HandlePatternChanged;
						}
						HandlePatternChanged();

						void HandlePatternChanged(object? sender = null, EventArgs? args = null)
						{
							PatternChanged?.Invoke(this, EventArgs.Empty);
							ToolTipText = (Pattern != null && !Pattern.IsValid) ? Pattern.ValidateExpr()?.Message : null;
						}
					}
				} = null!;

				/// <summary>Raised when the pattern expression changes</summary>
				public event EventHandler? PatternChanged;

				/// <summary>The text box for the filter string in this cell</summary>
				public TextBox FilterTextBox
				{
					get { return EditCtrl.TextBox; }
				}

				/// <summary>The control used to edit the filter field</summary>
				private EditControl EditCtrl { get; set; }
				private class EditControl :ToolForm
				{
					private readonly ColumnFiltersData m_cfd;

					public EditControl(ColumnFiltersData cfd) :base()
					{
						FormBorderStyle = FormBorderStyle.None;
						StartPosition = FormStartPosition.Manual;
						ShowInTaskbar = false;
						HideOnClose = true;
						MinimumSize = new Size(1,1);

						m_cfd = cfd;

						var tb = Controls.Add2(new TextBox{ Dock = DockStyle.Fill, AcceptsTab = true, AcceptsReturn = true, Multiline = true });
						tb.KeyDown += HandleKeyDown;
						tb.LostFocus += Hide;
					}
					protected override void Dispose(bool disposing)
					{
						Cell = null;
						base.Dispose(disposing);
					}
					protected override CreateParams CreateParams
					{
						get
						{
							var cp = base.CreateParams;
							cp.ClassStyle &= ~Win32.CS_DROPSHADOW;
							return cp;
						}
					}

					/// <summary>The text box on the edit control</summary>
					public TextBox TextBox
					{
						get { return (TextBox)Controls[0]; }
					}

					/// <summary>The cell being edited</summary>
					public FilterHeaderCell? Cell
					{
						get;
						set
						{
							if (field == value) return;
							if (field != null)
							{
								TextBox.TextChanged -= HandleTextChanged;
								TextBox.Text = string.Empty;
								PinTarget = null;
							}
							field = value;
							if (field != null)
							{
								TextBox.Text = field.Pattern?.Expr ?? string.Empty;
								TextBox.TextChanged += HandleTextChanged;

								// Position the control over the cell
								PinTarget = DGV;
								var cell_bounds = field.CellBounds(false);
								Bounds = DGV.RectangleToScreen(field.FieldBounds.Shifted(cell_bounds.TopLeft()));
							}
						}
					}

					/// <summary>The data grid view that 'Cell' belongs to</summary>
					public System.Windows.Forms.DataGridView? DGV => Cell?.DataGridView;

					/// <summary>Show the edit control within 'cell'</summary>
					public void Show(FilterHeaderCell cell)
					{
						Cell = cell;
						TextBox.Select(TextBox.TextLength, 0);
						base.Show(DGV);
					}
					public override void Hide()
					{
						base.Hide();
						Cell = null;
					}

					/// <summary>Update the cell pattern when the filter text changes</summary>
					private void HandleTextChanged(object sender, EventArgs args)
					{
						Cell.Pattern.Expr = TextBox.Text;
						TextBox.BackColor = Cell.Pattern.IsValid ? SystemColors.Window : Color.LightSalmon;
					}

					/// <summary>Handle keys in the filter field</summary>
					private void HandleKeyDown(object sender, KeyEventArgs args)
					{
						// On Tab, select the next/prev column
						if (args.KeyCode == Keys.Tab)
						{
							var i = Cell.ColumnIndex;
							var next = i+1 < DGV.ColumnCount ? DGV.Columns[i+1].HeaderCell as FilterHeaderCell : null;
							var prev = i-1 > -1              ? DGV.Columns[i-1].HeaderCell as FilterHeaderCell : null;
							if (!args.Shift && next != null) Dispatcher.CurrentDispatcher.BeginInvoke(new Action(next.EditFilter));
							if ( args.Shift && prev != null) Dispatcher.CurrentDispatcher.BeginInvoke(new Action(prev.EditFilter));
						}

						// Close the editing control on these keys
						if (args.KeyCode == Keys.Enter ||
							args.KeyCode == Keys.Escape ||
							args.KeyCode == Keys.Tab)
						{
							Hide();
						}

						// Close the editing control and hide the column filters
						if (args.KeyCode == m_cfd.ShortcutKey && args.Control)
						{
							Hide();
							m_cfd.Enabled = false;
						}
					}
				}

				/// <summary>Edit the value of the filter</summary>
				public void EditFilter()
				{
					DataGridView.EndEdit();
					EditCtrl.Show(this);
				}

				/// <summary>Show a small UI for editing the pattern</summary>
				public void ShowPatternUI()
				{
					var p = new PatternUI { Dock = DockStyle.Fill };
					using (var f = p.FormWrap(title: "Edit Pattern", loc: DataGridView.PointToScreen(this.CellBounds(false).BottomLeft())))
					{
						p.EditPattern(Pattern);
						p.Commit += (s,a) =>
						{
							Pattern = p.Pattern;
							f.Close();
							DataGridView.InvalidateCell(this);
						};
						f.ShowDialog(DataGridView);
					}
				}

				/// <summary>Paint the custom cell</summary>
				protected override void Paint(Graphics graphics, Rectangle clipBounds, Rectangle cell_bounds, int rowIndex, DataGridViewElementStates dataGridViewElementState, object value, object formattedValue, string errorText, DataGridViewCellStyle cellStyle, DataGridViewAdvancedBorderStyle advancedBorderStyle, DataGridViewPaintParts paintParts)
				{
					base.Paint(graphics, clipBounds, cell_bounds, rowIndex, dataGridViewElementState, value, formattedValue, errorText, cellStyle, advancedBorderStyle, paintParts);
		
					// Paint the search string text box
					var font = DataGridView.ColumnHeadersDefaultCellStyle.Font;
					var bounds = FieldBounds.Shifted(cell_bounds.Left, cell_bounds.Top);
					graphics.FillRectangle(Pattern.IsValid ? SystemBrushes.Window : Brushes.LightSalmon, bounds);
					graphics.DrawRectangle(Pens.Black, bounds.Inflated(0,0,-1,-1));
					graphics.DrawString(Pattern.Expr, font, Brushes.Black, bounds.Shifted(0, Math.Max(0, (bounds.Height - font.Height)/2)));
				}
			}
		}

		/// <summary>
		/// Get the column filters associated with this grid.
		/// To remove column filters, just Dispose the returned instance.
		/// Column filters are automatically disposed when the associated grid is disposed</summary>
		public static ColumnFiltersData? ColumnFilters(this DataGridView grid, bool create_if_necessary = false)
		{
			var grid_state = GridState[grid];
			if (grid_state.ColumnFilters != null) return grid_state.ColumnFilters;
			if (create_if_necessary) return grid_state.ColumnFilters = new ColumnFiltersData(grid);
			return null;
		}

		/// <summary>Toggle column filter fields on/off for a grid. Attach this method to KeyDown and ensure 'ColumnFilters(create_if_necessary:true)' has been called</summary>
		public static void ColumnFilters(object sender, EventArgs args)
		{
			var grid = (DataGridView)sender;

			// Get the associated column filters
			var cf = grid.ColumnFilters();
			if (cf == null)
				return; // not enabled

			// If this is a KeyDown handler, check for the required shortcut keys
			if (args is KeyEventArgs ke)
			{
				if (ke.Handled) return; // already handled
				if (!(ke.Control && ke.KeyCode == cf.ShortcutKey)) return; // wrong key shortcut
				ke.Handled = true;
			}

			// Toggle column filter fields
			cf.Enabled = !cf.Enabled;
		}

		#endregion

		#region Virtual Data Source

		/// <summary>An interface for a type that provides data for a grid</summary>
		public interface IDGVVirtualDataSource
		{
			void DGVCellValueNeeded(object sender, DataGridViewCellValueEventArgs args);
		}

		/// <summary>
		/// Attaches 'source' as the data provider for this grid.
		/// Source is connected with weak references so an external reference to 'source' must be held</summary>
		public static void SetVirtualDataSource(this DataGridView grid, IDGVVirtualDataSource source)
		{
			if (source == null)
				throw new ArgumentNullException("source","Source cannot be null");

			var w = new WeakReference(source);
			DataGridViewCellValueEventHandler? handler = null;
			handler = (sender,args) =>
				{
					var src = w.Target as IDGVVirtualDataSource;
					if (src == null) grid.CellValueNeeded -= handler;
					else src.DGVCellValueNeeded(sender, args);
				};

			grid.VirtualMode = true;
			grid.CellValueNeeded += handler;
		}

		#endregion
	}

	/// <summary>Tool strip control extensions</summary>
	public static class ToolStrip_
	{
		/// <summary>ToolStripMenuItem comparer for alphabetical order</summary>
		public static readonly Cmp<ToolStripItem> AlphabeticalOrder = Cmp<ToolStripItem>.From((l,r) => string.Compare(l.Text, r.Text, true));

		/// <summary>RAII scope for temporarily disabling 'Enabled' on this control</summary>
		public static Scope SuspendEnabled(this ToolStripItem ctrl)
		{
			return Scope.Create(
				() => { var e = ctrl.Enabled; ctrl.Enabled = false; return e; },
				e => { ctrl.Enabled = e; });
		}

		/// <summary>Enable/Disable this item and set the tool tip at the same time</summary>
		public static void EnabledWithTT(this ToolStripItem item, bool enabled, string tool_tip)
		{
			item.Enabled = enabled;
			item.ToolTipText = tool_tip;
		}

		/// <summary>Find the first sub-menu item with the given tags (depth first search)</summary>
		public static ToolStripItem? Find(this ToolStripItemCollection items, params string[] tags)
		{
			return Find(items, (IEnumerable<string>)tags);
		}
		public static ToolStripItem? Find(this ToolStripItemCollection items, IEnumerable<string> tags)
		{
			var tag = tags.FirstOrDefault();
			if (tag == null)
				return null;

			// Check each child that matches 'next'
			var rest = tags.Skip(1);
			foreach (var child in items.Find(tag, false))
			{
				if (!rest.Any())
					return child;

				switch (child)
				{
				case ToolStripMenuItem menu:
					{
						var found = menu.DropDownItems.Find(rest);
						if (found != null) return found;
						break;
					}
				case ToolStripDropDownItem dd:
					{
						var found = dd.DropDownItems.Find(rest);
						if (found != null) return found;
						break;
					}
				}
			}
			return null;
		}

		/// <summary>Add and return an item to this collection</summary>
		public static T Add2<T>(this ToolStripItemCollection items, T item) where T:ToolStripItem
		{
			items.Add(item);
			return item;
		}
		public static ToolStripMenuItem Add2(this ToolStripItemCollection items, string text, Image? image, EventHandler on_click)
		{
			return Add2(items, new ToolStripMenuItem(text, image, on_click));
		}

		/// <summary>Insert and return an item into this collection</summary>
		public static T Insert2<T>(this ToolStripItemCollection items, int index, T item) where T:ToolStripItem
		{
			items.Insert(index, item);
			return item;
		}

		/// <summary>Add and return an menu item to this collection in order defined by 'cmp'</summary>
		public static T AddOrdered<T>(this ToolStripItemCollection items, T item, IComparer<ToolStripMenuItem> cmp) where T:ToolStripMenuItem
		{
			var idx = 0;
			for (idx = 0; idx != items.Count; ++idx)
			{
				if (items[idx] is ToolStripMenuItem mi && cmp.Compare(mi, item) > 0)
					break;
			}

			items.Insert(idx, item);
			return item;
		}

		/// <summary>Add and return a menu item in alphabetical order</summary>
		public static T AddOrdered<T>(this ToolStripItemCollection items, T item) where T:ToolStripMenuItem
		{
			return items.AddOrdered(item, AlphabeticalOrder);
		}

		/// <summary>Insert a separator if the previous item is not a separator</summary>
		public static void InsertSeparator(this ToolStripItemCollection items, int index)
		{
			if (items.Count == 0 || index == 0) return;
			if (items[index - 1] is ToolStripSeparator) return;
			items.Insert(index, new ToolStripSeparator());
		}

		/// <summary>Add a separator if the previous item is not a separator</summary>
		public static void AddSeparator(this ToolStripItemCollection items)
		{
			items.InsertSeparator(items.Count);
		}

		/// <summary>Make sure a separator is not the last item</summary>
		public static void TrimSeparator(this ToolStripItemCollection items)
		{
			if (items.Count == 0) return;
			if (!(items[items.Count - 1] is ToolStripSeparator)) return;
			items.RemoveAt(items.Count - 1);
		}

		/// <summary>Hide successive or start/end separators</summary>
		public static void TidySeparators(this ToolStripItemCollection items, bool recursive = true)
		{
			// Note: Not using ToolStripItem.Visible because that also includes the parent's
			// visibility (which during construction is usually false).
			int s, e;

			// Returns true for menu items that should be considered as contiguous separators.
			Func<ToolStripItem, bool> hide = item => item is ToolStripSeparator || !item.Available;

			// Hide starting separators
			for (s = 0; s != items.Count && hide(items[s]); ++s)
				items[s].Available = false;

			// Hide ending separators
			for (e = items.Count; e-- != 0 && hide(items[e]); )
				items[e].Available = false;

			// Hide successive separators
			for (int i = s + 1; i < e; ++i)
			{
				if (!(items[i] is ToolStripSeparator)) continue;

				// Make the first separator in the contiguous sequence visible
				items[i].Available = true;
				for (int j = i + 1; j < e && hide(items[j]); i = j, ++j)
					items[j].Available = false;
			}

			// Tidy sub menus as well
			if (recursive)
			{
				foreach (var item in items.OfType<ToolStripDropDownItem>())
					item.DropDownItems.TidySeparators(recursive);
			}
		}

		/// <summary>Resize this item to fit the available space in the container</summary>
		public static void StretchToFit(this ToolStripItem item, int minimum_width)
		{
			// Notes:
			//  - tool_strip.Stretch = true;
			//  - tool_strip.AutoSize = false;
			//  - tool_strip.Layout += (s,a) => tool_strip_item.StretchToFit(250);

			// Ignore if vertical or on overflow, or no owner
			if (item.IsOnOverflow || item.Owner.Orientation == Orientation.Vertical || item.Owner == null)
				return;

			// Width accumulator
			var width = item.Owner.DisplayRectangle.Width;

			// Subtract the width of the overflow button if it is displayed. 
			if (item.Owner.OverflowButton.Visible)
				width -= item.Owner.OverflowButton.Width + item.Owner.OverflowButton.Margin.Horizontal;

			// Subtract the grip width if visible
			if (item.Owner.GripStyle == ToolStripGripStyle.Visible)
				width -= item.Owner.GripRectangle.Width + item.Owner.GripMargin.Horizontal;

			// Subtract the width of the other items in the container
			foreach (ToolStripItem other in item.Owner.Items)
			{
				if (other.IsOnOverflow) continue;
				if (other == item) continue;
				width -= other.Width + other.Margin.Horizontal;
			}

			// If the available width is less than the default width, use the
			// default width, forcing one or more items onto the overflow menu.
			width = Math.Max(minimum_width, width);

			// Set the new size
			item.Size = new Size(width, item.Height);
		}

		/// <summary>
		/// Restores the tool strip locations in this container and assigns handlers so that
		/// the tool strip locations are saved whenever a tool strip moves, is added, or removed</summary>
		public static void AutoPersistLocations(this ToolStripContainer cont, ToolStripLocations locations, Action<ToolStripLocations> save)
		{
			// Save the layout for 'tsc'
			Action<ToolStripContainer?> persist_locations = tsc =>
			{
				if (tsc == null || !tsc.Visible) return;
				save?.Invoke(tsc.SaveLocations());
			};

			// A handler for saving the TSC layout after a tool strip has had it's location changed programmatically
			EventHandler persist_after_location_changed = (s,a) =>
			{
				if (s is not ToolStrip strip) return;
				var tsc = (ToolStripContainer?)strip.Parent?.Parent;
				if (strip.IsCurrentlyDragging) return; // Don't persist locations during drag operations (the drag handler does that)
				persist_locations(tsc);
			};

			// Restore the locations from persistence data
			cont.LoadLocations(locations);

			// Attach a handler to each child tool strip to save the tool bar locations whenever they move
			foreach (var ts in cont.ToolStrips())
			{
				ts.LocationChanged += persist_after_location_changed;
				ts.EndDrag += persist_after_location_changed;
			}

			// Attach a handler to watch for tool strips added or removed
			cont.ControlAdded += (s,a) =>
			{
				if (a.Control is not ToolStrip ts) return;
				ts.LocationChanged += persist_after_location_changed;
				ts.EndDrag += persist_after_location_changed;
				persist_locations(s as ToolStripContainer);
			};
			cont.ControlRemoved += (s,a) =>
			{
				if (a.Control is not ToolStrip ts) return;
				ts.LocationChanged -= persist_after_location_changed;
				ts.EndDrag -= persist_after_location_changed;
				persist_locations(s as ToolStripContainer);
			};
		}

		/// <summary>Exports location data for this tool strip container</summary>
		public static ToolStripLocations SaveLocations(this ToolStripContainer cont)
		{
			using (cont.MarkAsSaving())
				return new ToolStripLocations(cont);
		}

		/// <summary>Imports location data for this tool strip container</summary>
		public static void LoadLocations(this ToolStripContainer cont, ToolStripLocations data)
		{
			if (data == null) return;
			using (cont.MarkAsLoading())
				data.Apply(cont);
		}

		/// <summary>Returns the Top,Left,Right,Bottom panels</summary>
		public static IEnumerable<ToolStripPanel> Panels(this ToolStripContainer cont)
		{
			yield return cont.TopToolStripPanel;
			yield return cont.LeftToolStripPanel;
			yield return cont.RightToolStripPanel;
			yield return cont.BottomToolStripPanel;
		}

		/// <summary>Returns all contained ToolStrips within the Top,Left,Right,Bottom panels</summary>
		public static IEnumerable<ToolStrip> ToolStrips(this ToolStripContainer cont)
		{
			foreach (var panel in cont.Panels())
			foreach (var ts in panel.Controls.Cast<ToolStrip>())
				yield return ts;
		}

		/// <summary>Returns all contained tool bar items within the Top,Left,Right,Bottom panels</summary>
		public static IEnumerable<ToolStripItem> ToolStripItems(this ToolStripContainer cont)
		{
			foreach (var ts in cont.ToolStrips())
			foreach (var item in ts.Items.Cast<ToolStripItem>())
				yield return item;
		}

		/// <summary>Recursively search the collection removing any items with a Name property equal to 'key'</summary>
		public static void RemoveByKey(this ToolStripItemCollection cont, string[] keys, bool recursive)
		{
			for (int i = cont.Count; i-- != 0;)
			{
				var item = cont[i];
				if (keys.Contains(item.Name))
					cont.RemoveAt(i);
				else if (item is ToolStripMenuItem mi && recursive)
					mi.DropDownItems.RemoveByKey(keys, recursive);
			}
		}
		public static void RemoveByKey(this ToolStripItemCollection cont, string key, bool recursive)
		{
			cont.RemoveByKey(new[]{key}, recursive);
		}

		/// <summary>Remove the tool strip item from it's owner</summary>
		public static void Remove(this ToolStripItem item)
		{
			if (item.Owner == null) return;
			item.Owner.Items.Remove(item);
		}

		/// <summary>A smarter set text that does sensible things with the caret position</summary>
		public static void SetText(this System.Windows.Forms.ToolStripComboBox cb, string text)
		{
			var idx = cb.SelectionStart;
			cb.SelectedText = text;
			cb.SelectionStart = idx + text.Length;
		}

		/// <summary>Set the tool tip for this tool strip item</summary>
		public static void ToolTip(this ToolStripItem ctrl, ToolTip tt, string caption)
		{
			// Don't need 'tt', this method is just for consistency with the other overload
			ctrl.ToolTipText = caption;
		}

		/// <summary>Display the hint balloon. Note, is difficult to get working, use HintBalloon instead.</summary>
		public static void ShowHintBalloon(this ToolStripItem item, ToolTip tt, string msg, int duration = 5000)
		{
			var parent = item.GetCurrentParent();
			if (parent == null) return;
			var pt = item.Bounds.Centre();

			tt.SetToolTip(parent, msg);
			tt.Show(msg, parent, pt, duration);
			SynchronizationContext_.BeginInvokeDelayed(() => tt.SetToolTip(parent, null), TimeSpan.FromMilliseconds(duration));
		}

		/// <summary>Returns the location of this item in screen space</summary>
		public static Point ScreenLocation(this ToolStripItem item)
		{
			var parent = item.GetCurrentParent();
			if (parent == null) return item.Bounds.Location;
			return parent.PointToScreen(item.Bounds.Location);
		}

		/// <summary>Returns the bounds of this item in form space</summary>
		public static Rectangle ParentFormRectangle(this ToolStripItem item)
		{
			var parent = item.GetCurrentParent();
			var top    = parent != null ? parent.TopLevelControl : null;
			var srect  = parent == null ? item.Bounds : parent.RectangleToScreen(item.Bounds);
			return top != null ? top.RectangleToClient(srect) : srect;
		}

		/// <summary>Returns the bounds of this item in screen space</summary>
		public static Rectangle ScreenRectangle(this ToolStripItem item)
		{
			var parent = item.GetCurrentParent();
			if (parent == null) return item.Bounds;
			return parent.RectangleToScreen(item.Bounds);
		}

		/// <summary>Converts a point in item space to screen space</summary>
		public static Point PointToScreen(this ToolStripItem item, Point pt)
		{
			var origin = item.ScreenLocation();
			return pt.Shifted(origin.X, origin.Y);
		}

		/// <summary>
		/// Set a label with text plus optional colours, detail tool tip, callback click handler, display time.
		/// Calling SetStatusMessage() with no msg resets the priority to the given value (default 0).</summary>
		public static void SetStatusMessage(this Label status,
			string? msg = null, string? tt = null, bool bold = false, Color? fr_color = null, Color? bk_color = null,
			TimeSpan? display_time = null, EventHandler? on_click = null, int priority = 0, bool auto_hide = true)
		{
			if (status == null) return;
			SetStatusMessageInternal(new StatusControl_Label(status), msg, tt, bold, fr_color, bk_color, display_time, on_click, priority, auto_hide);
		}

		/// <summary>
		/// Set a status label with text plus optional colours, detail tool tip, callback click handler, display time.
		/// Calling SetStatusMessage() with no msg resets the priority to the given value (default 0).</summary>
		public static void SetStatusMessage(this ToolStripStatusLabel status,
			string? msg = null, string? tt = null, bool bold = false, Color? fr_color = null, Color? bk_color = null,
			TimeSpan? display_time = null, EventHandler? on_click = null, int priority = 0, bool auto_hide = true)
		{
			if (status == null) return;
			SetStatusMessageInternal(new StatusControl_ToolStripStatusLabel(status), msg, tt, bold, fr_color, bk_color, display_time, on_click, priority, auto_hide);
		}

		/// <summary>Set the message to display when the status label has no status to display</summary>
		public static void SetStatusMessageIdle(this Label status, string? msg = null, Color? fr_color = null, Color? bk_color = null)
		{
			if (status == null) return;
			SetStatusMessageIdleInternal(new StatusControl_Label(status), msg, fr_color, bk_color);
		}

		/// <summary>Set the message to display when the status label has no status to display</summary>
		public static void SetStatusMessageIdle(this ToolStripStatusLabel status, string? msg = null, Color? fr_color = null, Color? bk_color = null)
		{
			if (status == null) return;
			SetStatusMessageIdleInternal(new StatusControl_ToolStripStatusLabel(status), msg, fr_color, bk_color);
		}

		/// <summary>The interface required for a control to behave as a status label</summary>
		public interface IStatusControl
		{
			object Tag { get; set; }
			string Text { get; set; }
			string ToolTipText { set; }
			Color ForeColor { set; }
			Color BackColor { set; }
			bool Visible { set; }
			Font Font { get; set; }
			Cursor Cursor { set; }
			event EventHandler Click;
			event EventHandler MouseEnter;
			event EventHandler MouseLeave;
		}
		private class StatusControl_Label :IStatusControl
		{
			private Label m_lbl;
			public StatusControl_Label(Label lbl)
			{
				if (lbl == null) throw new ArgumentNullException(nameof(lbl));
				m_lbl = lbl;
			}

			public object Tag                    { get { return m_lbl.Tag; } set { m_lbl.Tag = value; } }
			public string Text                   { get { return m_lbl.Text; } set { m_lbl.Text = value; } }
			public string ToolTipText            { set { var data = (StatusTagData)m_lbl.Tag; m_lbl.ToolTip(data.m_tt, value); } }
			public Color  BackColor              { set { m_lbl.BackColor = value; } }
			public Color  ForeColor              { set { m_lbl.ForeColor = value; } }
			public bool   Visible                { set { m_lbl.Visible = value; } }
			public Font   Font                   { get { return m_lbl.Font; } set { m_lbl.Font = value; } }
			public Cursor Cursor                 { set { m_lbl.Cursor = value; } }
			public event EventHandler Click      { add { m_lbl.Click += value; }      remove { m_lbl.Click -= value; } }
			public event EventHandler MouseEnter { add { m_lbl.MouseEnter += value; } remove { m_lbl.MouseEnter -= value; } }
			public event EventHandler MouseLeave { add { m_lbl.MouseLeave += value; } remove { m_lbl.MouseLeave -= value; } }
		}
		private class StatusControl_ToolStripStatusLabel :IStatusControl
		{
			private ToolStripStatusLabel m_lbl;
			public StatusControl_ToolStripStatusLabel(ToolStripStatusLabel lbl)
			{
				if (lbl == null) throw new ArgumentNullException(nameof(lbl));
				m_lbl = lbl;
			}

			public object Tag                    { get { return m_lbl.Tag; } set { m_lbl.Tag = value; } }
			public string Text                   { get { return m_lbl.Text; } set { m_lbl.Text = value; } }
			public string ToolTipText            { set { var data = (StatusTagData)m_lbl.Tag; m_lbl.ToolTip(data.m_tt, value); } }
			public Color  BackColor              { set { m_lbl.BackColor = value; } }
			public Color  ForeColor              { set { m_lbl.ForeColor = value; } }
			public bool   Visible                { set { m_lbl.Visible = value; } }
			public Font   Font                   { get { return m_lbl.Font; } set { m_lbl.Font = value; } }
			public Cursor Cursor                 { set { m_lbl.Owner.Cursor = value; } }
			public event EventHandler Click      { add { m_lbl.Click += value; }      remove { m_lbl.Click -= value; } }
			public event EventHandler MouseEnter { add { m_lbl.MouseEnter += value; } remove { m_lbl.MouseEnter -= value; } }
			public event EventHandler MouseLeave { add { m_lbl.MouseLeave += value; } remove { m_lbl.MouseLeave -= value; } }
		}

		/// <summary>Data added to the 'Tag' of a status label when used by the SetStatus function</summary>
		private class StatusTagData
		{
			private IStatusControl m_lbl;
			public string m_idle_msg;
			public Color m_idle_bk_colour;
			public Color m_idle_fr_colour;
			public int m_priority;
			public Timer? m_timer;
			public ToolTip m_tt;
			public EventHandler? m_on_click;

			public StatusTagData(IStatusControl lbl)
			{
				m_lbl            = lbl;
				m_idle_msg       = string.Empty;
				m_idle_fr_colour = SystemColors.ControlText;
				m_idle_bk_colour = SystemColors.Control;
				m_priority       = 0;
				m_timer          = null;
				m_tt             = new ToolTip();
				m_on_click       = null;

				if (m_lbl.Tag is StatusTagData)
					throw new Exception("Status label already has status data");
				if (m_lbl.Tag != null)
					throw new Exception("Status label Tag property already used for non-status data");

				m_lbl.Click += HandleStatusClick;
				m_lbl.MouseEnter += HandleMouseEnter;
				m_lbl.MouseLeave += HandleMouseLeave;
				m_lbl.Tag = this;
			}
			public void HandleStatusClick(object sender, EventArgs args)
			{
				m_on_click?.Invoke(sender, args); // Forward to the user handler
			}
			public void HandleMouseEnter(object sender, EventArgs args)
			{
				var is_link = m_on_click != null;
				if (is_link)
					m_lbl.Cursor = Cursors.Hand;
			}
			public void HandleMouseLeave(object sender, EventArgs args)
			{
				var is_link = m_on_click != null;
				if (is_link)
					m_lbl.Cursor = Cursors.Default;
			}
		}

		/// <summary>
		/// Set the text of a control plus optional colours, detail tool tip, callback click handler, display time.
		/// Calling SetStatusMessage() with no msg resets the priority to the given value (default 0).</summary>
		private static void SetStatusMessageInternal(IStatusControl status,
			string? msg = null, string? tt = null, bool bold = false, Color? fr_color = null, Color? bk_color = null,
			TimeSpan? display_time = null, EventHandler? on_click = null, int priority = 0, bool auto_hide = true)
		{
			if (status == null)
				return;

			// Ensure the status has tag data
			if (status.Tag == null)
				status.Tag = new StatusTagData(status);

			// Get the status data
			var data = (status.Tag as StatusTagData) ?? throw new NullReferenceException("Tag property already used for non-status data");;

			// Ignore the status if it has lower priority than the current
			if (msg.HasValue() && priority < data.m_priority) return;
			data.m_priority = priority;

			// Set the text
			status.Text = msg ?? data.m_idle_msg;

			// Set the tool tip to the detailed message
			status.ToolTipText = tt ?? string.Empty;

			// Set colours
			status.ForeColor = fr_color ?? data.m_idle_fr_colour;
			status.BackColor = bk_color ?? data.m_idle_bk_colour;

			// Hide the status control if it has no value
			status.Visible = !auto_hide || status.Text.HasValue();

			// Choose the font to use
			var font_style = FontStyle.Regular;
			if (bold            ) font_style |= FontStyle.Bold;
			if (on_click != null) font_style |= FontStyle.Underline;
			if (status.Font.Style != font_style)
				status.Font = new Font(status.Font, font_style);

			// If the status message has a timer, dispose it
			// If the status has a display time, set a timer
			Util.Dispose(ref data.m_timer);
			if (display_time != null)
			{
				data.m_timer = new Timer{Enabled = true, Interval = (int)display_time.Value.TotalMilliseconds};
				data.m_timer.Tick += (s,a) =>
					{
						// When the timer fires, if we're still associated with
						// the status message, null out the text and remove ourself
						if (!ReferenceEquals(s, data.m_timer)) return;
						data.m_priority = 0;
						SetStatusMessageInternal(status, msg:data.m_idle_msg);
						Util.Dispose(ref data.m_timer);
					};
			}

			// If a click handler has been provided, subscribe
			data.m_on_click = on_click;
		}

		/// <summary>Set the message to display when the status label has no status to display</summary>
		private static void SetStatusMessageIdleInternal(IStatusControl status, string? msg = null, Color? fr_color = null, Color? bk_color = null)
		{
			if (status == null)
				return;

			// Ensure the status has tag data
			if (status.Tag == null)
			{
				new StatusTagData(status);
			}
			else if (status.Tag is StatusTagData data)
			{
				if (msg      != null) data.m_idle_msg = msg;
				if (fr_color != null) data.m_idle_fr_colour = fr_color.Value;
				if (bk_color != null) data.m_idle_bk_colour = bk_color.Value;
			}
			else
			{
				throw new Exception("Tag property already used for non-status data");
			}
		}

		/// <summary>
		/// Merge the contents of 'rhs' into this menu.
		/// Menus with the same *Name* (not Text) become one menu. Otherwise uses MergeIndex to define the order.
		/// 'choose_rhs' causes the rhs to replace the lhs when items are considered equal</summary>
		public static void Merge(this ToolStrip lhs, ToolStrip rhs, bool choose_rhs, bool permanent = false)
		{
			// Record the layout of 'cont' and 'rhs' if not seen before
			if (!permanent)
			{
				if (!m_ts_layout.ContainsKey(lhs))
					m_ts_layout.Add(lhs, new ToolStripLayout(lhs));
				if (!m_ts_layout.ContainsKey(rhs))
					m_ts_layout.Add(rhs, new ToolStripLayout(rhs));
			}
			DoMerge(lhs, lhs.Items, rhs, rhs.Items, choose_rhs);
		}
		public static void Merge(this ToolStripDropDownItem lhs, ToolStripDropDownItem rhs, bool choose_rhs, bool permanent = false)
		{
			// Record the layout of 'cont' and 'rhs' if not seen before
			if (!permanent)
			{
				if (!m_ts_layout.ContainsKey(lhs))
					m_ts_layout.Add(lhs, new ToolStripLayout(lhs));
				if (!m_ts_layout.ContainsKey(rhs))
					m_ts_layout.Add(rhs, new ToolStripLayout(rhs));
			}
			DoMerge(lhs.Owner, lhs.DropDownItems, rhs.Owner, rhs.DropDownItems, choose_rhs);
		}

		/// <summary>Restore this menu, removing it from any other menus it might be merged into</summary>
		public static void UnMerge(this ToolStrip strip)
		{
			ToolStripLayout layout;
			if (m_ts_layout.TryGetValue(strip, out layout))
			{
				layout.Rebuild();
				m_ts_layout.Remove(strip);
			}
		}
		public static void UnMerge(this ToolStripDropDownItem cont)
		{
			ToolStripLayout layout;
			if (m_ts_layout.TryGetValue(cont, out layout))
			{
				layout.Rebuild();
				m_ts_layout.Remove(cont);
			}
		}

		/// <summary>Keeps a record of drop-down menu layouts</summary>
		private static Dictionary<object, ToolStripLayout> m_ts_layout = new Dictionary<object,ToolStripLayout>();

		/// <summary>Records the layout of a tool-strip</summary>
		internal class ToolStripLayout
		{
			public object m_item;
			public List<ToolStripLayout> m_children;

			public ToolStripLayout(ToolStrip strip)
			{
				m_item = strip;
				m_children = strip.Items.Cast<ToolStripItem>().Select(x => new ToolStripLayout(x)).ToList();
			}
			public ToolStripLayout(ToolStripItem item)
			{
				m_item = item;
				m_children = item is ToolStripDropDownItem ddi
					? ddi.DropDownItems.Cast<ToolStripItem>().Select(x => new ToolStripLayout(x)).ToList()
					: new List<ToolStripLayout>();
			}
			public override string ToString()
			{
				return
					m_item is ToolStrip ts ? ts.Name :
					m_item is ToolStripItem tsi ? $"{tsi.Text} idx:{tsi.MergeIndex}" :
					throw new Exception("m_item is not a tool strip or tool strip item");
			}
			/// <summary>Rebuilds 'm_item' to the stored layout</summary>
			public void Rebuild()
			{
				Rebuild(this);
			}
			private static void Rebuild(ToolStripLayout item)
			{
				item.m_children.ForEach(Rebuild);
				if (item.m_item is ToolStripDropDownItem ddi)
				{
					ddi.DropDownItems.Clear();
					item.m_children.ForEach(c => ddi.DropDownItems.Add((ToolStripItem)c.m_item));
				}
				if (item.m_item is ToolStrip ts)
				{
					ts.Items.Clear();
					item.m_children.ForEach(c => ts.Items.Add((ToolStripItem)c.m_item));
				}
			}
		}

		/// <summary>
		/// Merge item collections into 'lhs'
		/// When two items are considered equal, 'choose_rhs' causes the item from 'rhs' to
		/// replace the item in 'lhs'. Otherwise the item from 'lhs' is used</summary>
		private static void DoMerge(object lhs_owner, ToolStripItemCollection lhs, object rhs_owner, ToolStripItemCollection rhs, bool choose_rhs)
		{
			if (ReferenceEquals(lhs_owner, rhs_owner))
				throw new Exception("Merge menus failed. Cannot merge menu items belonging to the same menu");
			if (lhs.Cast<ToolStripItem>().Any(x => x.Owner != lhs_owner))
				throw new Exception($"Merge menus failed. All items in the collection must belong to {lhs_owner}");
			if (rhs.Cast<ToolStripItem>().Any(x => x.Owner != rhs_owner))
				throw new Exception($"Merge menus failed. All items in the collection must belong to {rhs_owner}");

			// Replace the -1 merge indices by their index position
			// In 'lhs', set the merge index of all items (including separators) because
			// we want to preserve the 'lhs' menu as much as possible
			// In 'rhs', don't set the merge index for separators because we use the default
			// index to indicate that the separator should be ignored for merging
			for (int i = 0; i != lhs.Count; ++i)
				if (lhs[i].MergeIndex == -1)
					lhs[i].MergeIndex = i;
			for (int i = 0; i != rhs.Count; ++i)
				if (rhs[i].MergeIndex == -1 && !(rhs[i] is ToolStripSeparator))
					rhs[i].MergeIndex = i;

			// Sort the items in lhs by MergeIndex
			for (bool sorted = false; !sorted;)
			{
				sorted = true;
				for (int i = 1; i < lhs.Count; ++i)
				{
					if (lhs[i].MergeIndex >= lhs[i-1].MergeIndex) continue;
					lhs.Insert(i-1, lhs[i]);
					sorted = false;
					--i;
				}
			}

			// For each item in 'rhs' look for a match in 'lhs'. If found, merge the item.
			// If not found, insert the item based on MergeIndex.
			var dst = lhs.Cast<ToolStripItem>().ToList();
			var src = rhs.Cast<ToolStripItem>().ToList();
			foreach (var r in src)
			{
				// Ignore separators without a name or merge index
				var sep = r as ToolStripSeparator;
				if (sep != null && !sep.Name.HasValue() && sep.MergeIndex == -1)
					continue;

				// Look for a match in 'lhs'
				var l = r.Name.HasValue()
					? dst.FirstOrDefault(x => x.Name == r.Name)
					: dst.FirstOrDefault(x => x.Text == r.Text);

				// Merge 'l' and 'r'
				if (l != null)
				{
					var ldd = l as ToolStripDropDownItem;
					var rdd = r as ToolStripDropDownItem;

					// If one menu replaces the other, remove all items from the 'replacee'
					if (l.MergeAction == MergeAction.Replace && rdd != null)
						rdd.DropDownItems.Clear();
					if (r.MergeAction == MergeAction.Replace && ldd != null)
						ldd.DropDownItems.Clear();

					// If one menu is marked with remove, then neither menu is added
					if (l.MergeAction == MergeAction.Remove || r.MergeAction == MergeAction.Remove)
					{
						var idx = l.Name.HasValue()
							? lhs.Cast<ToolStripItem>().IndexOf(x => x.Name == r.Name)
							: lhs.Cast<ToolStripItem>().IndexOf(x => x.Text == r.Text);
						if (idx != -1) lhs.RemoveAt(idx);
					}

					// Merge 'r' into 'l', then keep 'l'
					else if (!choose_rhs)
					{
						if (ldd != null && rdd != null && rdd.DropDownItems.Count != 0)
							DoMerge(ldd.DropDown, ldd.DropDownItems, rdd.DropDown, rdd.DropDownItems, false);
					}

					// Otherwise, merge 'l' into 'r', then keep 'r'
					else
					{
						if (ldd != null && rdd != null && ldd.DropDownItems.Count != 0)
							DoMerge(rdd.DropDown, rdd.DropDownItems, ldd.DropDown, ldd.DropDownItems, false);

						// If 'rhs' has more than one item with the same name, it's possible that the first
						// occurrence was merged and replace the item in 'lhs'. The second item needs to replace
						// the first item in lhs. We can't use lhs.IndexOf() because 'l' may no longer be in 'lhs'.
						var idx = l.Name.HasValue()
							? lhs.Cast<ToolStripItem>().IndexOf(x => x.Name == r.Name)
							: lhs.Cast<ToolStripItem>().IndexOf(x => x.Text == r.Text);
						lhs.RemoveAt(idx);
						lhs.Insert(idx, r);
					}
				}

				// No match found, merge using MergeIndex
				else
				{
					var idx = (r.MergeIndex != -1 ? r.MergeIndex : lhs.Count);
					int ins; for (ins = 0; ins != lhs.Count && lhs[ins].MergeIndex < idx; ++ins) {}
					lhs.Insert(ins, r);
				}
			}
		}
	}

	/// <summary>Used to persist control locations and sizes in XML</summary>
	public class ControlLocations
	{
		// Produces this:
		//  <ctrl>
		//    <name/>
		//    <location/>
		//    <size/>
		//    <ctrl/>
		//    <ctrl/>
		//    ...
		//  </ctrl>

		private static class Tag
		{
			public const string Ctrl     = "ctrl";
			public const string Name     = "name";
			public const string Loc      = "loc";
			public const string Size     = "size";
			public const string Children = "children";
		}

		private string m_name;
		private Point  m_location;
		private Size   m_size;
		private Dictionary<string, ControlLocations> m_children;

		public ControlLocations()
		{
			m_name     = string.Empty;
			m_location = Point.Empty;
			m_size     = Size.Empty;
			m_children = new Dictionary<string,ControlLocations>();
		}
		public ControlLocations(ControlLocations rhs)
		{
			m_name     = rhs.m_name;
			m_location = rhs.m_location;
			m_size     = rhs.m_size;
			m_children = new Dictionary<string,ControlLocations>();
			foreach (var kv in rhs.m_children)
				m_children.Add(kv.Key, new ControlLocations(kv.Value));
		}
		public ControlLocations(Control ctrl, int level = 0, int index = 0) :this()
		{
			ReadInternal(ctrl, level, index);
		}
		public ControlLocations(XElement node)
		{
			m_name     = node.Element(Tag.Name).As<string>();
			m_location = node.Element(Tag.Loc ).As<Point>();
			m_size     = node.Element(Tag.Size).As<Size>();
			m_children = new Dictionary<string, ControlLocations>();
			foreach (var child in node.Elements(Tag.Ctrl))
			{
				var loc = new ControlLocations(child);
				m_children.Add(loc.m_name, loc);
			}
		}
		public XElement ToXml(XElement node)
		{
			node.Add(m_name    .ToXml(Tag.Name ,false));
			node.Add(m_location.ToXml(Tag.Loc  ,false));
			node.Add(m_size    .ToXml(Tag.Size ,false));
			foreach (var ch in m_children.Values)
				node.Add(ch.ToXml(Tag.Ctrl, false));
			return node;
		}

		/// <summary>Populate these settings from a control</summary>
		public void Read(Control ctrl)
		{
			ReadInternal(ctrl, 0, 0);
		}

		/// <summary>Apply the stored position data to 'ctrl'</summary>
		public void Apply(Control ctrl, bool layout_on_resume = true)
		{
			var name = UniqueName(ctrl, 0, 0);
			if (m_name != name)
			{
				Debug.WriteLine($"Control locations ignored due to name mismatch.\nControl Name {name} != Layout Data Name {m_name}");
				return;
			}

			//using (ctrl.SuspendLayout(layout_on_resume))
				ApplyInternal(ctrl, 0);
		}

		/// <summary>Recursively populate this object from 'ctrl'</summary>
		private void ReadInternal(Control ctrl, int level, int index)
		{
			m_name     = UniqueName(ctrl, level, index);
			m_location = ctrl.Location;
			m_size     = ctrl.Size;
			m_children = new Dictionary<string,ControlLocations>();
			for (var i = 0; i != ctrl.Controls.Count; ++i)
			{
				var s = new ControlLocations(ctrl.Controls[i], level + 1, i);
				try
				{
					m_children.Add(s.m_name, s);
				}
				catch (ArgumentException ex)
				{
					throw new Exception("A sibling control with this name already exists. All controls must have a unique name in order to save position data", ex);
				}
			}
		}

		// Recursively position 'ctrl' and it's children
		private void ApplyInternal(Control ctrl, int level)
		{
			// ToolStripPanels need to perform a layout before setting the location of child controls
			if (ctrl is ToolStripContainer || ctrl is ToolStripPanel)
				ctrl.PerformLayout();

			// Position the control
			ctrl.Location = m_location;
			if (ctrl.Dock == DockStyle.None && ctrl.Location != m_location && !(ctrl is MenuStrip && ctrl.AutoSize))
				Debug.WriteLine($"ControlLayout: Control {ctrl.Name} location is {ctrl.Location} instead of {m_location}");

			// Set the control size
			ctrl.Size = m_size;
			if (!ctrl.AutoSize && ctrl.Size != m_size)
				Debug.WriteLine($"ControlLayout: Control {ctrl.Name} size is {ctrl.Size} instead of {m_size}");

			// Set the bounds of the child controls
			for (var i = 0; i != ctrl.Controls.Count; ++i)
			{
				var child = ctrl.Controls[i];
				var name = UniqueName(child, level + 1, i);

				ControlLocations s;
				if (m_children.TryGetValue(name, out s))
					s.ApplyInternal(child, level + 1);
			}
		}

		/// <summary>Generates a unique name for 'ctrl'</summary>
		private string UniqueName(Control ctrl, int level, int index)
		{
			return $"{(ctrl.Name.HasValue() ? ctrl.Name : ctrl.GetType().Name)}({level},{index})";
		}

		public override string ToString()
		{
			return $"{m_name} (child count: {m_children.Count})";
		}
	}

	/// <summary>Used to persist control locations and sizes in XML</summary>
	public class ToolStripLocations
	{
		private static class Tag
		{
			public const string Name     = "name";
			public const string Top      = "top";
			public const string Left     = "left";
			public const string Right    = "right";
			public const string Bottom   = "bottom";
		}
		private string m_name;
		private ControlLocations m_top;
		private ControlLocations m_left;
		private ControlLocations m_right;
		private ControlLocations m_bottom;

		public ToolStripLocations()
		{
			m_name      = string.Empty;
			m_top       = new ControlLocations();
			m_left      = new ControlLocations();
			m_right     = new ControlLocations();
			m_bottom    = new ControlLocations();
		}
		public ToolStripLocations(ToolStripLocations rhs)
		{
			m_name   = rhs.m_name;
			m_top    = new ControlLocations(rhs.m_top);
			m_left   = new ControlLocations(rhs.m_left);
			m_right  = new ControlLocations(rhs.m_right);
			m_bottom = new ControlLocations(rhs.m_bottom);
		}
		public ToolStripLocations(ToolStripContainer cont) :this()
		{
			Read(cont);
		}
		public ToolStripLocations(XElement node)
		{
			m_name      = node.Element(Tag.Name    ).As<string>();
			m_top       = node.Element(Tag.Top     ).As<ControlLocations>();
			m_left      = node.Element(Tag.Left    ).As<ControlLocations>();
			m_right     = node.Element(Tag.Right   ).As<ControlLocations>();
			m_bottom    = node.Element(Tag.Bottom  ).As<ControlLocations>();
		}
		public XElement ToXml(XElement node)
		{
			node.Add
			(
				m_name  .ToXml(Tag.Name    ,false),
				m_top   .ToXml(Tag.Top     ,false),
				m_left  .ToXml(Tag.Left    ,false),
				m_right .ToXml(Tag.Right   ,false),
				m_bottom.ToXml(Tag.Bottom  ,false)
			);
			return node;
		}
		public void Read(ToolStripContainer cont)
		{
			m_name      = cont.Name;
			m_top       = new ControlLocations(cont.TopToolStripPanel   );
			m_left      = new ControlLocations(cont.LeftToolStripPanel  );
			m_right     = new ControlLocations(cont.RightToolStripPanel );
			m_bottom    = new ControlLocations(cont.BottomToolStripPanel);
		}
		public void Apply(ToolStripContainer cont)
		{
			// If these locations are for a different container, don't apply.
			if (m_name != cont.Name)
			{
				Debug.WriteLine($"ToolStripContainer locations ignored due to name mismatch - ToolStripContainer Name {cont.Name} != Layout Data Name {m_name}");
				return;
			}

			// ToolStripContainer need to perform a layout before setting the location of child controls
			cont.PerformLayout();

			// Apply the layout to each panel
			m_top   .Apply(cont.TopToolStripPanel   );
			m_left  .Apply(cont.LeftToolStripPanel  );
			m_right .Apply(cont.RightToolStripPanel );
			m_bottom.Apply(cont.BottomToolStripPanel);
		}
	}

	/// <summary>Event args for sorting a DGV using the DataGridView_</summary>
	public class DGVSortEventArgs :EventArgs
	{
		public DGVSortEventArgs(int column_index, SortOrder direction)
		{
			ColumnIndex = column_index;
			Direction  = direction;
		}

		/// <summary>The column to sort</summary>
		public int ColumnIndex { get; private set; }

		/// <summary>The direction to sort it in</summary>
		public SortOrder Direction { get; private set; }
	}

	/// <summary>A helper object for use with Control.Tag</summary>
	public class LoadSaveTag
	{
		public object? OriginalTag;
		public bool Loading;
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Extn;
	using Gui.WinForms;

	[TestFixture] public class TestTSExtns
	{
		[Test] public void MergeMenus()
		{
			var m0 = new MenuStrip{Name = "m0", Text = "m0"};
			{
				var i0 = m0.Items.Add2(new ToolStripMenuItem{Name = "i0", Text = "i0", MergeIndex = 10});
				var i1 = m0.Items.Add2(new ToolStripMenuItem{Name = "i1", Text = "i1", MergeIndex = 20});
				var i2 = m0.Items.Add2(new ToolStripMenuItem{Name = "i2", Text = "i2", MergeIndex = 30});
				var i10 = i1.DropDownItems.Add2(new ToolStripMenuItem{Name = "i10", Text = "i10", MergeIndex = 10});
				var i12 = i1.DropDownItems.Add2(new ToolStripMenuItem{Name = "i12", Text = "i12", MergeIndex = 20});
			}
			var m1 = new MenuStrip{Name = "m1", Text = "m1"};
			{
				var i1 = m1.Items.Add2(new ToolStripMenuItem{Name = "i1", Text = "i1", MergeIndex = 15});
				var i3 = m1.Items.Add2(new ToolStripMenuItem{Name = "i3", Text = "i3", MergeIndex = 35});
				var i10 = i1.DropDownItems.Add2(new ToolStripMenuItem{Name = "i10", Text = "i10", MergeIndex = 15});
				var i11 = i1.DropDownItems.Add2(new ToolStripMenuItem{Name = "i11", Text = "i11", MergeIndex = 15});
			}

			m0.Merge(m1, false);
			Assert.True(m0.Text == "m0");
			Assert.True(m0.Items.Count == 4);
			Assert.True(m0.Items[0].Text == "i0");
			Assert.True(m0.Items[1].Text == "i1");
			Assert.True(m0.Items[2].Text == "i2");
			Assert.True(m0.Items[3].Text == "i3");
			Assert.True(((ToolStripMenuItem)m0.Items[1]).DropDownItems.Count == 3);
			Assert.True(((ToolStripMenuItem)m0.Items[1]).DropDownItems[0].Text == "i10");
			Assert.True(((ToolStripMenuItem)m0.Items[1]).DropDownItems[1].Text == "i11");
			Assert.True(((ToolStripMenuItem)m0.Items[1]).DropDownItems[2].Text == "i12");

			m0.UnMerge();
			Assert.True(m0.Text == "m0");
			Assert.True(m0.Items.Count == 3);
			Assert.True(m0.Items[0].Text == "i0");
			Assert.True(m0.Items[1].Text == "i1");
			Assert.True(m0.Items[2].Text == "i2");
			Assert.True(((ToolStripMenuItem)m0.Items[1]).DropDownItems.Count == 2);
			Assert.True(((ToolStripMenuItem)m0.Items[1]).DropDownItems[0].Text == "i10");
			Assert.True(((ToolStripMenuItem)m0.Items[1]).DropDownItems[1].Text == "i12");
		}
	}
}
#endif