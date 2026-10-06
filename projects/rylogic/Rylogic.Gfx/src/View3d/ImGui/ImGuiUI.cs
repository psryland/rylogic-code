using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using Rylogic.D3D12;
using Rylogic.Interop.Win32;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx.ImGui;

/// <summary>Window behaviour flags. Values match ImGuiWindowFlags.</summary>
[Flags]
public enum EWindowFlags
{
	None = 0,
	NoTitleBar = 1 << 0,
	NoResize = 1 << 1,
	NoMove = 1 << 2,
	NoScrollbar = 1 << 3,
	NoScrollWithMouse = 1 << 4,
	NoCollapse = 1 << 5,
	AlwaysAutoResize = 1 << 6,
	NoBackground = 1 << 7,
	NoSavedSettings = 1 << 8,
	NoMouseInputs = 1 << 9,
	MenuBar = 1 << 10,
	HorizontalScrollbar = 1 << 11,
	NoFocusOnAppearing = 1 << 12,
	NoBringToFrontOnFocus = 1 << 13,
	AlwaysVerticalScrollbar = 1 << 14,
	AlwaysHorizontalScrollbar = 1 << 15,
	NoNavInputs = 1 << 16,
	NoNavFocus = 1 << 17,
	NoNav = NoNavInputs | NoNavFocus,
	NoDecoration = NoTitleBar | NoResize | NoScrollbar | NoCollapse,
	NoInputs = NoMouseInputs | NoNavInputs | NoNavFocus,
}

/// <summary>When a 'SetNextWindow...' value applies. Values match ImGuiCond.</summary>
public enum ECond
{
	Always = 0,
	Once = 1 << 1,
	FirstUseEver = 1 << 2,
	Appearing = 1 << 3,
}

/// <summary>Slider and drag behaviour flags. Values match ImGuiSliderFlags.</summary>
[Flags]
public enum ESliderFlags
{
	None = 0,
	Logarithmic = 1 << 5,
	NoRoundToFormat = 1 << 6,
	NoInput = 1 << 7,
	WrapAround = 1 << 8,
	ClampOnInput = 1 << 9,
	ClampZeroRange = 1 << 10,
	AlwaysClamp = ClampOnInput | ClampZeroRange,
}

/// <summary>Collapsing header flags. Values match ImGuiTreeNodeFlags.</summary>
[Flags]
public enum ETreeNodeFlags
{
	None = 0,
	DefaultOpen = 1 << 5,
}

/// <summary>
/// An immediate-mode Dear ImGui context, implemented by the dynamically loaded 'imgui.dll'.
/// Attach it to a View3D window to draw each completed frame into that window's final overlay pass.
/// Each frame: call 'NewFrame', and only if it returns true, call the widget functions then 'EndFrame', before rendering the window.
/// All calls must be made on the thread that created the context.
/// </summary>
public sealed unsafe class ImGuiUI : IDisposable
{
	/// <summary>Version of the 'imgui.dll' exports that this wrapper expects.</summary>
	internal const uint ApiVersion = 0x00020000U;

	private const string Dll = "imgui";
	private static IntPtr s_module;

	private readonly ErrorCB m_error_cb;
	private IntPtr m_ctx;
	private View3d.Window? m_window;
	private string? m_ini_filename;
	private string? m_error;

	/// <summary>Create a context that receives input from 'hwnd' and renders with the device of 'device'.</summary>
	/// <param name="device">The D3D12 device of the View3D window this context will attach to. The context takes its own reference.</param>
	/// <param name="hwnd">The window that input messages are forwarded from.</param>
	/// <param name="scale">Scale for text and spacing, for example the window's DPI scale.</param>
	public ImGuiUI(DeviceLease device, IntPtr hwnd, float scale = 1f)
	{
		// Load the DLL once, rejecting a build that does not match this wrapper.
		EnsureLoaded();

		// Collect native errors so they can be thrown on the managed side, because exceptions cannot cross the native boundary.
		m_error_cb = (ctx, msg, len) =>
		{
			// Append so that no message is lost when several errors occur before the next check.
			var text = Encoding.UTF8.GetString((byte*)msg, checked((int)len));
			m_error = m_error == null ? text : $"{m_error}\n{text}";
		};

		// The render target format is taken from the attached window, so it is left unknown here.
		// Pin the lease only for creation; the native context takes its own COM reference.
		using var borrowed = device.Borrow();
		var args = new InitArgs
		{
			m_device = borrowed.Handle,
			m_cmd_queue = IntPtr.Zero,
			m_hwnd = hwnd,
			m_rtv_format = 0,
			m_num_frames_in_flight = 0,
			m_font_scale = scale,
		};
		m_ctx = ImGui_Initialise(in args, new ErrorHandler { m_ctx = IntPtr.Zero, m_cb = m_error_cb });
		ThrowIfError();
		if (m_ctx == IntPtr.Zero)
			throw new Exception("Failed to create the ImGui context");
	}

	/// <summary>Detach from the window, wait for the GPU to finish with this context's resources, then release them.</summary>
	public void Dispose()
	{
		// The window's in-flight frames may still reference this context's buffers.
		if (m_ctx == IntPtr.Zero)
			return;

		var window = m_window;
		DetachFromWindow();
		window?.GSyncWait();
		ImGui_Shutdown(m_ctx);
		m_ctx = IntPtr.Zero;
		GC.KeepAlive(m_error_cb);
	}

	/// <summary>Draw this context's completed frames into the final overlay of 'window'. The window must use the device this context was created with.</summary>
	public void AttachToWindow(View3d.Window window)
	{
		// A context draws into at most one window.
		if (m_window != null)
			throw new InvalidOperationException("The ImGui context is already attached to a window");

		var attached = ImGui_AttachToWindow(Ctx, window.Handle);
		ThrowIfError();
		if (!attached)
			throw new Exception("Failed to attach the ImGui context to the window");

		m_window = window;
	}

	/// <summary>Stop drawing into the attached window. Does nothing if not attached.</summary>
	public void DetachFromWindow()
	{
		// The window may draw one more frame from already-recorded command lists. 'Dispose' waits for that.
		if (m_window == null)
			return;

		m_window = null;
		ImGui_DetachFromWindow(Ctx);
		ThrowIfError();
	}

	/// <summary>The file that stores window positions, sizes, and collapsed state. Null disables persistence. Set before the first frame.</summary>
	public string? IniFilename
	{
		get
		{
			return m_ini_filename;
		}
		set
		{
			ImGui_SetIniFilename(Ctx, value);
			m_ini_filename = value;
		}
	}

	/// <summary>Start a frame. Returns false, without starting a frame, until the attached window has rendered once.</summary>
	public bool NewFrame()
	{
		var started = ImGui_NewFrame(Ctx);
		ThrowIfError();
		return started;
	}

	/// <summary>Finish the frame so the attached window draws it in its next render.</summary>
	public void EndFrame()
	{
		ImGui_EndFrame(Ctx);
		ThrowIfError();
	}

	/// <summary>Forward a window message to ImGui. Returns true if ImGui handled the message.</summary>
	public bool WndProc(IntPtr hwnd, int msg, IntPtr wparam, IntPtr lparam)
	{
		return ImGui_WndProc(Ctx, hwnd, (uint)msg, wparam, lparam);
	}

	/// <summary>True when ImGui uses the mouse, so the application should ignore mouse input.</summary>
	public bool WantCaptureMouse => ImGui_WantCaptureMouse(Ctx);

	/// <summary>True when ImGui uses the keyboard, so the application should ignore keyboard input.</summary>
	public bool WantCaptureKeyboard => ImGui_WantCaptureKeyboard(Ctx);

	/// <summary>Use this display size instead of the window's client size, rescaling mouse positions to match. (0, 0) uses the client size.</summary>
	public void SetDisplaySize(float width, float height)
	{
		ImGui_SetDisplaySize(Ctx, width, height);
	}

	// Windows

	/// <summary>Begin a window. Always call 'EndWindow', even when this returns false (collapsed or clipped). 'open' is cleared when the user closes the window.</summary>
	public bool BeginWindow(string name, ref bool open, EWindowFlags flags = EWindowFlags.None)
	{
		fixed (bool* p_open = &open)
			return ImGui_BeginWindow(Ctx, name, p_open, (int)flags);
	}

	/// <summary>Begin a window without a close button. Always call 'EndWindow'.</summary>
	public bool BeginWindow(string name, EWindowFlags flags = EWindowFlags.None)
	{
		return ImGui_BeginWindow(Ctx, name, null, (int)flags);
	}

	/// <summary>End the window begun by 'BeginWindow'.</summary>
	public void EndWindow()
	{
		ImGui_EndWindow(Ctx);
	}

	/// <summary>Set the position of the next window, in display pixels.</summary>
	public void SetNextWindowPos(float x, float y, ECond cond = ECond.Always)
	{
		ImGui_SetNextWindowPos(Ctx, x, y, (int)cond);
	}

	/// <summary>Set the size of the next window, in display pixels. Zero on an axis fits that axis to the content.</summary>
	public void SetNextWindowSize(float width, float height, ECond cond = ECond.Always)
	{
		ImGui_SetNextWindowSize(Ctx, width, height, (int)cond);
	}

	/// <summary>Set the background opacity of the next window.</summary>
	public void SetNextWindowBgAlpha(float alpha)
	{
		ImGui_SetNextWindowBgAlpha(Ctx, alpha);
	}

	// Menus

	/// <summary>Begin the menu bar along the top of the display. Call 'EndMainMenuBar' only if this returns true.</summary>
	public bool BeginMainMenuBar()
	{
		return ImGui_BeginMainMenuBar(Ctx);
	}

	/// <summary>End the menu bar begun by 'BeginMainMenuBar'.</summary>
	public void EndMainMenuBar()
	{
		ImGui_EndMainMenuBar(Ctx);
	}

	/// <summary>Begin a sub-menu. Call 'EndMenu' only if this returns true.</summary>
	public bool BeginMenu(string label, bool enabled = true)
	{
		return ImGui_BeginMenu(Ctx, label, enabled);
	}

	/// <summary>End the sub-menu begun by 'BeginMenu'.</summary>
	public void EndMenu()
	{
		ImGui_EndMenu(Ctx);
	}

	/// <summary>A menu item. Returns true when clicked.</summary>
	public bool MenuItem(string label, string? shortcut = null, bool enabled = true)
	{
		return ImGui_MenuItem(Ctx, label, shortcut, null, enabled);
	}

	/// <summary>A menu item with a check mark that toggles 'selected'. Returns true when clicked.</summary>
	public bool MenuItem(string label, ref bool selected, string? shortcut = null, bool enabled = true)
	{
		fixed (bool* p_selected = &selected)
			return ImGui_MenuItem(Ctx, label, shortcut, p_selected, enabled);
	}

	// Layout

	/// <summary>Place the next item on the same line as the previous one. A negative 'spacing' uses the style spacing.</summary>
	public void SameLine(float offset_from_start_x = 0f, float spacing = -1f)
	{
		ImGui_SameLine(Ctx, offset_from_start_x, spacing);
	}

	/// <summary>A horizontal line.</summary>
	public void Separator()
	{
		ImGui_Separator(Ctx);
	}

	/// <summary>A horizontal line with a label.</summary>
	public void SeparatorText(string label)
	{
		ImGui_SeparatorText(Ctx, label);
	}

	/// <summary>A small vertical gap.</summary>
	public void Spacing()
	{
		ImGui_Spacing(Ctx);
	}

	/// <summary>Set the width of the next widget. A negative width aligns its right edge that far from the window's right edge.</summary>
	public void SetNextItemWidth(float width)
	{
		ImGui_SetNextItemWidth(Ctx, width);
	}

	/// <summary>Push a scope that makes the IDs of following widgets unique. Pair with 'PopID'.</summary>
	public void PushID(string id)
	{
		ImGui_PushID(Ctx, id);
	}

	/// <summary>Pop the scope pushed by 'PushID'.</summary>
	public void PopID()
	{
		ImGui_PopID(Ctx);
	}

	/// <summary>Grey out and ignore input for following widgets while 'disabled' is true. Always pair with 'EndDisabled'.</summary>
	public void BeginDisabled(bool disabled = true)
	{
		ImGui_BeginDisabled(Ctx, disabled);
	}

	/// <summary>End the scope begun by 'BeginDisabled'.</summary>
	public void EndDisabled()
	{
		ImGui_EndDisabled(Ctx);
	}

	/// <summary>A header that shows or hides the content after it. Returns true while open.</summary>
	public bool CollapsingHeader(string label, ETreeNodeFlags flags = ETreeNodeFlags.None)
	{
		return ImGui_CollapsingHeader(Ctx, label, (int)flags);
	}

	// Text

	/// <summary>Plain text.</summary>
	public void Text(string text)
	{
		ImGui_Text(Ctx, text);
	}

	/// <summary>Greyed-out text.</summary>
	public void TextDisabled(string text)
	{
		ImGui_TextDisabled(Ctx, text);
	}

	/// <summary>Text that wraps at the window's right edge.</summary>
	public void TextWrapped(string text)
	{
		ImGui_TextWrapped(Ctx, text);
	}

	/// <summary>Text in a straight (not premultiplied) RGBA colour.</summary>
	public void TextColored(float r, float g, float b, float a, string text)
	{
		ImGui_TextColored(Ctx, r, g, b, a, text);
	}

	/// <summary>Show a tooltip while the previous item is hovered.</summary>
	public void SetItemTooltip(string text)
	{
		ImGui_SetItemTooltip(Ctx, text);
	}

	// Widgets. Each returns true when the user changed the value or clicked.

	/// <summary>A push button. Returns true when clicked.</summary>
	public bool Button(string label)
	{
		return ImGui_Button(Ctx, label);
	}

	/// <summary>A check box. Returns true when toggled.</summary>
	public bool Checkbox(string label, ref bool value)
	{
		fixed (bool* p_value = &value)
			return ImGui_Checkbox(Ctx, label, p_value);
	}

	/// <summary>A slider. 'format' is a printf format for the value, for example "%.2f m".</summary>
	public bool SliderFloat(string label, ref float value, float minimum, float maximum, string format = "%.3f", ESliderFlags flags = ESliderFlags.None)
	{
		fixed (float* p_value = &value)
			return ImGui_SliderFloat(Ctx, label, p_value, minimum, maximum, format, (int)flags);
	}

	/// <summary>An integer slider. 'format' is a printf format for the value.</summary>
	public bool SliderInt(string label, ref int value, int minimum, int maximum, string format = "%d", ESliderFlags flags = ESliderFlags.None)
	{
		fixed (int* p_value = &value)
			return ImGui_SliderInt(Ctx, label, p_value, minimum, maximum, format, (int)flags);
	}

	/// <summary>A value changed by dragging. 'minimum' == 'maximum' means unbounded.</summary>
	public bool DragFloat(string label, ref float value, float speed = 1f, float minimum = 0f, float maximum = 0f, string format = "%.3f", ESliderFlags flags = ESliderFlags.None)
	{
		fixed (float* p_value = &value)
			return ImGui_DragFloat(Ctx, label, p_value, speed, minimum, maximum, format, (int)flags);
	}

	/// <summary>A numeric text box. A non-zero 'step' adds +/- buttons.</summary>
	public bool InputFloat(string label, ref float value, float step = 0f, float step_fast = 0f, string format = "%.3f")
	{
		fixed (float* p_value = &value)
			return ImGui_InputFloat(Ctx, label, p_value, step, step_fast, format, 0);
	}

	/// <summary>A single-line text box. 'max_bytes' is the UTF-8 capacity of the edit buffer, including the terminator.</summary>
	public bool InputText(string label, ref string value, int max_bytes = 256)
	{
		// ImGui edits a null-terminated UTF-8 buffer in place.
		var buffer = new byte[max_bytes];
		var length = Encoding.UTF8.GetBytes(value, 0, value.Length, buffer, 0);
		if (length >= max_bytes)
			throw new ArgumentException("The text does not fit in the edit buffer", nameof(value));

		// Decode the edited text only when it changed.
		fixed (byte* p_buffer = buffer)
		{
			if (!ImGui_InputText(Ctx, label, p_buffer, (UIntPtr)buffer.Length, 0))
				return false;

			var end = Array.IndexOf(buffer, (byte)0);
			value = Encoding.UTF8.GetString(buffer, 0, end);
			return true;
		}
	}

	/// <summary>A drop-down list. 'current' is the selected index.</summary>
	public bool Combo(string label, ref int current, IReadOnlyList<string> items)
	{
		// ImGui takes the items as one string of zero-terminated items, ending with an empty item.
		var sb = new StringBuilder();
		foreach (var item in items)
			sb.Append(item).Append('\0');

		sb.Append('\0');

		// Pass raw bytes because string marshalling stops at the first zero on some runtimes.
		var bytes = Encoding.UTF8.GetBytes(sb.ToString());
		fixed (int* p_current = &current)
		fixed (byte* p_items = bytes)
			return ImGui_Combo(Ctx, label, p_current, p_items, -1);
	}

	/// <summary>A line graph of 'values'. NaN scale limits are found from the data. Zero sizes use defaults.</summary>
	public void PlotLines(string label, float[] values, string? overlay = null, float scale_min = float.NaN, float scale_max = float.NaN, float width = 0f, float height = 0f)
	{
		// ImGui uses FLT_MAX to mean "find from the data".
		fixed (float* p_values = values)
			ImGui_PlotLines(Ctx, label, p_values, values.Length, 0, overlay, float.IsNaN(scale_min) ? float.MaxValue : scale_min, float.IsNaN(scale_max) ? float.MaxValue : scale_max, width, height);
	}

	/// <summary>The native context, which must not be used after disposal.</summary>
	private IntPtr Ctx => m_ctx != IntPtr.Zero ? m_ctx : throw new ObjectDisposedException(nameof(ImGuiUI));

	/// <summary>Throw any error reported by native code since the last check.</summary>
	private void ThrowIfError()
	{
		// Clear before throwing so each error is reported once.
		if (m_error == null)
			return;

		var error = m_error;
		m_error = null;
		throw new Exception($"ImGui: {error}");
	}

	/// <summary>Load 'imgui.dll' and check that its exports match this wrapper.</summary>
	internal static void EnsureLoaded()
	{
		// Every context shares one module.
		if (s_module != IntPtr.Zero)
			return;

		var module = Win32.LoadDll(Dll + ".dll", out var load_error);
		if (module == IntPtr.Zero)
			throw load_error ?? new DllNotFoundException($"Unable to load {Dll}.dll");

		var version = ImGui_ApiVersion();
		if (version != ApiVersion)
			throw new Exception($"{Dll}.dll API version {version:X8} does not match the expected version {ApiVersion:X8}");

		s_module = module;
	}

	#region Native

	/// <summary>Matches 'imgui::InitArgs'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	internal struct InitArgs
	{
		public IntPtr m_device;
		public IntPtr m_cmd_queue;
		public IntPtr m_hwnd;
		public int m_rtv_format;
		public int m_num_frames_in_flight;
		public float m_font_scale;
	}

	/// <summary>Matches 'imgui::ErrorHandler::FuncCB'.</summary>
	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	private delegate void ErrorCB(IntPtr ctx, IntPtr msg, UIntPtr len);

	/// <summary>Matches 'imgui::ErrorHandler'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	private struct ErrorHandler
	{
		public IntPtr m_ctx;
		public ErrorCB m_cb;
	}

	[DllImport(Dll)] private static extern uint ImGui_ApiVersion();
	[DllImport(Dll)] private static extern IntPtr ImGui_Initialise(in InitArgs args, ErrorHandler error_cb);
	[DllImport(Dll)] private static extern void ImGui_Shutdown(IntPtr ctx);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_AttachToWindow(IntPtr ctx, IntPtr window);
	[DllImport(Dll)] private static extern void ImGui_DetachFromWindow(IntPtr ctx);
	[DllImport(Dll)] private static extern void ImGui_SetIniFilename(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string? path);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_NewFrame(IntPtr ctx);
	[DllImport(Dll)] private static extern void ImGui_EndFrame(IntPtr ctx);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_WndProc(IntPtr ctx, IntPtr hwnd, uint msg, IntPtr wparam, IntPtr lparam);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_WantCaptureMouse(IntPtr ctx);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_WantCaptureKeyboard(IntPtr ctx);
	[DllImport(Dll)] private static extern void ImGui_SetDisplaySize(IntPtr ctx, float w, float h);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_BeginWindow(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string name, bool* p_open, int flags);
	[DllImport(Dll)] private static extern void ImGui_EndWindow(IntPtr ctx);
	[DllImport(Dll)] private static extern void ImGui_SetNextWindowPos(IntPtr ctx, float x, float y, int cond);
	[DllImport(Dll)] private static extern void ImGui_SetNextWindowSize(IntPtr ctx, float w, float h, int cond);
	[DllImport(Dll)] private static extern void ImGui_SetNextWindowBgAlpha(IntPtr ctx, float alpha);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_BeginMainMenuBar(IntPtr ctx);
	[DllImport(Dll)] private static extern void ImGui_EndMainMenuBar(IntPtr ctx);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_BeginMenu(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, [MarshalAs(UnmanagedType.U1)] bool enabled);
	[DllImport(Dll)] private static extern void ImGui_EndMenu(IntPtr ctx);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_MenuItem(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, [MarshalAs(UnmanagedType.LPUTF8Str)] string? shortcut, bool* selected, [MarshalAs(UnmanagedType.U1)] bool enabled);
	[DllImport(Dll)] private static extern void ImGui_SameLine(IntPtr ctx, float offset_from_start_x, float spacing);
	[DllImport(Dll)] private static extern void ImGui_Separator(IntPtr ctx);
	[DllImport(Dll)] private static extern void ImGui_SeparatorText(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label);
	[DllImport(Dll)] private static extern void ImGui_Spacing(IntPtr ctx);
	[DllImport(Dll)] private static extern void ImGui_SetNextItemWidth(IntPtr ctx, float width);
	[DllImport(Dll)] private static extern void ImGui_PushID(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string id);
	[DllImport(Dll)] private static extern void ImGui_PopID(IntPtr ctx);
	[DllImport(Dll)] private static extern void ImGui_BeginDisabled(IntPtr ctx, [MarshalAs(UnmanagedType.U1)] bool disabled);
	[DllImport(Dll)] private static extern void ImGui_EndDisabled(IntPtr ctx);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_CollapsingHeader(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, int flags);
	[DllImport(Dll)] private static extern void ImGui_Text(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string text);
	[DllImport(Dll)] private static extern void ImGui_TextDisabled(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string text);
	[DllImport(Dll)] private static extern void ImGui_TextWrapped(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string text);
	[DllImport(Dll)] private static extern void ImGui_TextColored(IntPtr ctx, float r, float g, float b, float a, [MarshalAs(UnmanagedType.LPUTF8Str)] string text);
	[DllImport(Dll)] private static extern void ImGui_SetItemTooltip(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string text);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_Button(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_Checkbox(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, bool* v);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_SliderFloat(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, float* v, float v_min, float v_max, [MarshalAs(UnmanagedType.LPUTF8Str)] string format, int flags);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_SliderInt(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, int* v, int v_min, int v_max, [MarshalAs(UnmanagedType.LPUTF8Str)] string format, int flags);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_DragFloat(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, float* v, float speed, float v_min, float v_max, [MarshalAs(UnmanagedType.LPUTF8Str)] string format, int flags);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_InputFloat(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, float* v, float step, float step_fast, [MarshalAs(UnmanagedType.LPUTF8Str)] string format, int flags);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_InputText(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, byte* buf, UIntPtr buf_size, int flags);
	[DllImport(Dll)][return: MarshalAs(UnmanagedType.U1)] private static extern bool ImGui_Combo(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, int* current_item, byte* items_separated_by_zeros, int popup_max_height_in_items);
	[DllImport(Dll)] private static extern void ImGui_PlotLines(IntPtr ctx, [MarshalAs(UnmanagedType.LPUTF8Str)] string label, float* values, int values_count, int values_offset, [MarshalAs(UnmanagedType.LPUTF8Str)] string? overlay_text, float scale_min, float scale_max, float graph_w, float graph_h);

	#endregion
}

#if PR_UNITTESTS
/// <summary>Checks that the managed declarations match the loaded 'imgui.dll'.</summary>
[TestFixture]
public sealed class TestImGuiUI
{
	/// <summary>The DLL loads with the expected API version, and 'InitArgs' has the native x64 layout.</summary>
	[Test]
	public void AbiMatchesNative()
	{
		// Loading throws if the DLL's API version differs from this wrapper's.
		ImGuiUI.EnsureLoaded();
		Assert.Equal(40, Marshal.SizeOf<ImGuiUI.InitArgs>());
		Assert.Equal(24, (int)Marshal.OffsetOf<ImGuiUI.InitArgs>(nameof(ImGuiUI.InitArgs.m_rtv_format)));
		Assert.Equal(32, (int)Marshal.OffsetOf<ImGuiUI.InitArgs>(nameof(ImGuiUI.InitArgs.m_font_scale)));
	}
}
#endif