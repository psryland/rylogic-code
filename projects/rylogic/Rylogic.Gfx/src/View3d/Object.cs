//#define PR_VIEW3D_CREATE_STACKTRACE
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Maths;
using Rylogic.Utility;
using HObject = System.IntPtr;

namespace Rylogic.Gfx
{
	public sealed partial class View3d
	{
		/// <summary>Object resource wrapper</summary>
		[DebuggerDisplay("{Description}")]
		public class Object :IDisposable, INotifyPropertyChanged
		{
			public delegate VICount EditObjectCB(Span<Vertex> verts, Span<ushort> indices, AddNuggetCB out_nugget);
			public delegate void AddNuggetCB(ref Nugget nugget);

			/// <summary>
			/// Create objects given in an ldr string or file.
			/// If multiple objects are created, the handle returned is to the first object only
			/// 'ldr_script' - an ldr string, or filepath to a file containing ldr script
			/// 'file' - TRUE if 'ldr_script' is a filepath, FALSE if 'ldr_script' is a string containing ldr script
			/// 'context_id' - the context id to create the LdrObjects with
			/// 'async' - if objects should be created by a background thread
			/// 'include_paths' - is a comma separated list of include paths to use to resolve #include directives (or nullptr)
			/// 'module' - if non-zero causes includes to be resolved from the resources in that module</summary>
			public Object()
				:this("*group{}", false, null)
			{}
			public Object(string ldr_script, bool file, Guid? context_id)
				:this(ldr_script, file, context_id, null)
			{}
			public Object(string ldr_script, bool file, Guid? context_id, Includes? includes)
			{
				Owned = true;
				var inc = includes ?? new Includes();
				var ctx = context_id ?? Guid.NewGuid();
				Handle = View3D_ObjectCreateLdrW(ldr_script, file, ref ctx, ref inc);
				if (Handle == HObject.Zero)
					throw new Exception($"Failed to create object from script\r\n{ldr_script.Summary(100)}");
			}
			public Object(byte[] binary, Guid? context_id)
			{
				Owned = true;
				var ctx = context_id ?? Guid.NewGuid();
				using var pin = Marshal_.Pin(binary, GCHandleType.Pinned);
				Handle = View3D_ObjectCreateLdrB(pin.Pointer, binary.LongLength, ref ctx);
				if (Handle == HObject.Zero)
					throw new Exception($"Failed to create object from binary script data");
			}

			/// <summary>Create an object from a P3D file</summary>
			public Object(string name, uint colour, string p3d_filepath, Guid? context_id)
			{
				Owned = true;
				var ctx = context_id ?? Guid.NewGuid();
				Handle = View3D_ObjectCreateP3DFile(name, colour, p3d_filepath, ref ctx);
				if (Handle == HObject.Zero)
					throw new Exception($"Failed to create object from p3d model file: '{p3d_filepath}'");
			}
			public Object(string name, uint colour, byte[] p3d_data, Guid? context_id)
			{
				Owned = true;
				var ctx = context_id ?? Guid.NewGuid();

				using var pin = Marshal_.Pin(p3d_data, GCHandleType.Pinned);
				Handle = View3D_ObjectCreateP3DStream(name, colour, p3d_data.Length, pin.Pointer, ref ctx);
				if (Handle == HObject.Zero)
					throw new Exception($"Failed to create object from p3d model data stream");
			}

			/// <summary>Create from buffer</summary>
			public Object(string name, uint colour, int vcount, int icount, int ncount, Vertex[] verts, ushort[] indices, Nugget[] nuggets, Guid? context_id)
			{
				Owned = true;
				var ctx = context_id ?? Guid.NewGuid();

				// Serialise the verts/indices to a memory buffer
				using var vbuf = Marshal_.Pin(verts, GCHandleType.Pinned);
				using var ibuf = Marshal_.Pin(indices, GCHandleType.Pinned);
				using var nbuf = Marshal_.ArrayToPtr(EHeap.HGlobal, nuggets);
				Handle = View3D_ObjectCreate(name, colour, vcount, icount, ncount, vbuf.Pointer, ibuf.Pointer, nbuf.Value.Ptr, ref ctx);
				if (Handle == HObject.Zero) throw new System.Exception($"Failed to create object '{name}' from provided buffers");
			}

			/// <summary>Create an object via callback</summary>
			public Object(string name, uint colour, int vcount, int icount, int ncount, EditObjectCB edit_cb, Guid? context_id)
			{
				Owned = true;
				var ctx = context_id ?? Guid.NewGuid();
				Handle = View3D_ObjectCreateWithCallback(name, colour, vcount, icount, ncount, new EditCB(edit_cb), ref ctx);
				if (Handle == HObject.Zero) throw new Exception($"Failed to create object '{name}' via edit callback");
			}

			/// <summary>Attach to an existing object handle</summary>
			public Object(HObject handle, bool owned = false)
			{
				Owned = owned;
				Handle = handle;
			}

			/// <summary>Dispose</summary>
			public void Dispose()
			{
				Dispose(true);
				GC.SuppressFinalize(this);
			}
			protected virtual void Dispose(bool _)
			{
				Util.BreakIf(Util.IsGCFinalizerThread, "Disposing in the GC finalizer thread");
				if (Handle == HObject.Zero) return;
				if (Owned) View3D_ObjectDelete(Handle);
				Handle = HObject.Zero;
			}

			/// <summary>The object handle for this object</summary>
			public HObject Handle;

			/// <summary>True if the view3d object is destroyed when this instance is disposed</summary>
			public bool Owned;

			/// <summary>User data attached to the object</summary>
			public object? Tag { get; set; }

			/// <summary>Object name</summary>
			public string Name
			{
				get => View3D_ObjectNameGet(Handle);
				set
				{
					View3D_ObjectNameSet(Handle, value);
					NotifyPropertyChanged(nameof(Name));
				}
			}

			/// <summary>Get the type of Ldr object this is</summary>
			public string Type
			{
				get => View3D_ObjectTypeGet(Handle);
			}

			/// <summary>Get/Set the visibility of this object (set applies to all child objects as well)</summary>
			public bool Visible
			{
				get => VisibleGet(string.Empty);
				set => VisibleSet(value, string.Empty);
			}

			/// <summary>Get/Set the visibility of this object (set applies to all child objects as well)</summary>
			public Colour32 Colour
			{
				get => ColourGet(false, string.Empty);
				set => ColourSet(value, string.Empty);
			}

			/// <summary>Get/Set the reflectivity for this object (set applies to all child objects as well)</summary>
			public float Reflectivity
			{
				get => ReflectivityGet(string.Empty);
				set => ReflectivitySet(value, string.Empty);
			}

			/// <summary>Get/Set wire-frame mode for this object(set applies to all child objects as well)</summary>
			public bool Wireframe
			{
				get => WireframeGet(string.Empty);
				set => WireframeSet(value, string.Empty);
			}

			/// <summary>Get/Set 'show normals' mode for this object(set applies to all child objects as well)</summary>
			public bool ShowNormals
			{
				get => ShowNormalsGet(string.Empty);
				set => ShowNormalsSet(value, string.Empty);
			}

			/// <summary>Get/Set the state flags for this object</summary>
			public ELdrFlags Flags
			{
				get => FlagsGet(string.Empty);
				set
				{
					FlagsSet(~ELdrFlags.None, false);
					FlagsSet(value, true);
				}
			}

			/// <summary>Get/Set sort group for this object</summary>
			public ESortGroup SortGroup
			{
				get => SortGroupGet(string.Empty);
				set => SortGroupSet(value, string.Empty);
			}

			/// <summary>Get/Set the nugget flags for the first nugget for this object</summary>
			public ENuggetFlag NuggetFlags
			{
				get => NuggetFlagsGet(string.Empty);
				set
				{
					NuggetFlagsSet(~ENuggetFlag.None, false, string.Empty);
					NuggetFlagsSet(value, true, string.Empty);
				}
			}

			/// <summary>Get/Set the nugget tint colour for the first nugget for this object</summary>
			public Colour32 NuggetTint
			{
				get => NuggetTintGet(string.Empty);
				set => NuggetTintSet(value, string.Empty);
			}

			/// <summary>The context id that this object belongs to</summary>
			public Guid ContextId
			{
				get => View3D_ObjectContextIdGet(Handle);
			}

			/// <summary>Change the model for this object</summary>
			public void UpdateModel(string ldr_script, EUpdateObject flags = EUpdateObject.All)
			{
				View3D_ObjectUpdate(Handle, ldr_script, flags);
			}

			/// <summary>Modify the model of this object</summary>
			public void Edit(EditObjectCB edit_cb)
			{
				View3D_ObjectEdit(Handle, new EditCB(edit_cb));
			}

			/// <summary>Get/Set the object to parent transform of the root object</summary>
			public m4x4 O2P
			{
				get => O2PGet(null);
				set => O2PSet(value, null);
			}

			/// <summary>Get the model space bounding box of this object</summary>
			public BBox BBoxMS(EBBoxFlags bbox_flags)
			{
				return View3D_ObjectBBoxMS(Handle, bbox_flags);
			}

			/// <summary>Return the object that is the root parent of this object (possibly itself)</summary>
			public Object? Root
			{
				get
				{
					var root = View3D_ObjectGetRoot(Handle);
					return root != HObject.Zero ? new Object(root, false) : null;
				}
			}

			/// <summary>Return the object that is the immediate parent of this object</summary>
			public Object? Parent
			{
				get
				{
					var parent = View3D_ObjectGetParent(Handle);
					return parent != HObject.Zero ? new Object(parent, false) : null;
				}
			}

			/// <summary>Return a child object of this object</summary>
			public Object? Child(string name)
			{
				var child = View3D_ObjectGetChildByName(Handle, name);
				return child != HObject.Zero ? new Object(child, false) : null;
			}
			public Object? Child(int index)
			{
				var child = View3D_ObjectGetChildByIndex(Handle, index);
				return child != HObject.Zero ? new Object(child, false) : null;
			}

			/// <summary>Return the number of child objects of this object</summary>
			public int ChildCount => View3D_ObjectChildCount(Handle);

			/// <summary>Return a list of all the child objects of this object</summary>
			public IList<Object> Children
			{
				get
				{
					if (Handle == HObject.Zero) return Array.Empty<Object>();
					var objects = new List<Object>(capacity: ChildCount);
					bool CB(IntPtr ctx, IntPtr obj) { objects.Add(new Object(obj)); return true; }
					View3D_ObjectEnumChildren(Handle, new EnumObjectsCB { m_cb = CB });
					return objects.ToArray();
				}
			}

			/// <summary>Recursively apply some operation to an object and it's children</summary>
			public bool Apply(Func<Object, bool> func, string? name = null, Object? obj = null)
			{
				if (obj == null)
				{
					obj = this;
				}
				if (name == null)
				{
					if (!func(obj))
						return false;
				}
				else if (name == string.Empty)
				{
					if (!func(obj)) return false;
					foreach (var child in obj.Children)
						if (!Apply(func, name, child))
							return false;
				}
				else
				{
					if ((name[0] == '#' && Regex.IsMatch(obj.Name, name.Substring(1))) || obj.Name == name)
					{
						if (!func(obj))
							return false;
					}
					foreach (var child in obj.Children)
					{
						if (!Apply(func, name, child))
							return false;
					}
				}
				return true;
			}

			/// <summary>Create a new Object that shares the model (but not transform) of this object</summary>
			public Object CreateInstance()
			{
				var handle = View3D_ObjectCreateInstance(Handle);
				if (handle == HObject.Zero)
					throw new Exception("Failed to create object instance");

				return new Object(handle, true);
			}

			// Notes:
			//  - Methods with a 'name' parameter apply an operation on this object
			//    or any of its child objects that match 'name'. If 'name' is null,
			//    then the change is applied to this object only. If 'name' is "",
			//    then the change is applied to this object and all children recursively.
			//	  Otherwise, the change is applied to all child objects that match name.
			//  - If 'name' begins with '#' then the name parameter is treated as a regular
			//    expression.

			/// <summary>
			/// Get/Set the object to world transform for this object or the first child object that matches 'name'.
			/// If 'name' is null, then the state of the root object is returned
			/// If 'name' begins with '#' then the remainder of the name is treated as a regular expression.
			/// Note, setting the o2w for a child object positions the object in world space rather than parent space
			/// (internally the appropriate O2P transform is calculated to put the object at the given O2W location)</summary>
			public m4x4 O2WGet(string? name = null)
			{
				return View3D_ObjectO2WGet(Handle, name);
			}
			public void O2WSet(m4x4 o2w, string? name = null)
			{
				Util.BreakIf(o2w.w.w != 1.0f, "Invalid object transform");
				Util.BreakIf(!Math_.IsFinite(o2w), "Invalid object transform");
				View3D_ObjectO2WSet(Handle, ref o2w, name);
				NotifyPropertyChanged(nameof(O2P));
			}

			/// <summary>
			/// Get/Set the object to parent transform for this object or any of its child objects that match 'name'.
			/// If 'name' is null, then the state change is applied to this object only
			/// If 'name' is "", then the state change is applied to this object and all children recursively
			/// Otherwise, the state change is applied to all child objects that match name.
			/// If 'name' begins with '#' then the remainder of the name is treated as a regular expression</summary>
			public m4x4 O2PGet(string? name = null)
			{
				return View3D_ObjectO2PGet(Handle, name);
			}
			public void O2PSet(m4x4 o2p, string? name = null)
			{
				Util.BreakIf(o2p.w.w != 1.0f, "Invalid object transform");
				Util.BreakIf(!Math_.IsFinite(o2p), "Invalid object transform");
				View3D_ObjectO2PSet(Handle, ref o2p, name);
				NotifyPropertyChanged(nameof(O2P));
			}

			/// <summary>
			/// Get/Set the visibility of this object or any of its child objects that match 'name'.
			/// If 'name' is null, then the state change is applied to this object only
			/// If 'name' is "", then the state change is applied to this object and all children recursively
			/// Otherwise, the state change is applied to all child objects that match name.
			/// If 'name' begins with '#' then the remainder of the name is treated as a regular expression</summary>
			public bool VisibleGet(string? name = null)
			{
				return View3D_ObjectVisibilityGet(Handle, name);
			}
			public void VisibleSet(bool vis, string? name = null)
			{
				var prev = Visible;
				View3D_ObjectVisibilitySet(Handle, vis, name);
				if (prev != Visible)
				{
					NotifyPropertyChanged(nameof(Visible));
					NotifyPropertyChanged(nameof(Flags));
				}
			}

			/// <summary>
			/// Get/Set wireframe mode for this objector any of its child objects that match 'name'.
			/// If 'name' is null, then the state change is applied to this object only
			/// If 'name' is "", then the state change is applied to this object and all children recursively
			/// Otherwise, the state change is applied to all child objects that match name.
			/// If 'name' begins with '#' then the remainder of the name is treated as a regular expression</summary>
			public bool WireframeGet(string? name = null)
			{
				return View3D_ObjectWireframeGet(Handle, name);
			}
			public void WireframeSet(bool vis, string? name = null)
			{
				var prev = Wireframe;
				View3D_ObjectWireframeSet(Handle, vis, name);
				if (prev != Wireframe)
				{
					NotifyPropertyChanged(nameof(Wireframe));
					NotifyPropertyChanged(nameof(Flags));
				}
			}

			/// <summary>
			/// Get/Set 'show normals' mode for this object or any of its child objects that match 'name'.
			/// If 'name' is null, then the state change is applied to this object only
			/// If 'name' is "", then the state change is applied to this object and all children recursively
			/// Otherwise, the state change is applied to all child objects that match name.
			/// If 'name' begins with '#' then the remainder of the name is treated as a regular expression</summary>
			public bool ShowNormalsGet(string? name = null)
			{
				return View3D_ObjectNormalsGet(Handle, name);
			}
			public void ShowNormalsSet(bool vis, string? name = null)
			{
				var prev = ShowNormals;
				View3D_ObjectNormalsSet(Handle, vis, name);
				if (prev != ShowNormals)
				{
					NotifyPropertyChanged(nameof(ShowNormals));
					NotifyPropertyChanged(nameof(Flags));
				}
			}
			/// <summary>
			/// Get/Set the object flags
			/// See LdrObject::Apply for docs on the format of 'name'</summary>
			public ELdrFlags FlagsGet(string? name = null)
			{
				return View3D_ObjectFlagsGet(Handle, name);
			}
			public void FlagsSet(ELdrFlags flags, bool state, string? name = null)
			{
				var prev = Flags;
				View3D_ObjectFlagsSet(Handle, flags, state, name);
				if (prev != Flags)
				{
					NotifyPropertyChanged(nameof(Flags));
					NotifyPropertyChanged(nameof(Wireframe));
					NotifyPropertyChanged(nameof(ShowNormals));
					NotifyPropertyChanged(nameof(Visible));
				}
			}

			/// <summary>
			/// Get/Set the object sort group.
			/// See LdrObject::Apply for docs on the format of 'name'</summary>
			public ESortGroup SortGroupGet(string? name = null)
			{
				return View3D_ObjectSortGroupGet(Handle, name);
			}
			public void SortGroupSet(ESortGroup group, string? name = null)
			{
				var prev = SortGroup;
				View3D_ObjectSortGroupSet(Handle, group, name);
				if (prev != SortGroup)
				{
					NotifyPropertyChanged(nameof(SortGroup));
				}
			}

			/// <summary>
			/// Get/Set the nugget flags for a nugget within this object
			/// See LdrObject::Apply for docs on the format of 'name'</summary>
			public ENuggetFlag NuggetFlagsGet(string? name = null, int index = 0)
			{
				return View3D_ObjectNuggetFlagsGet(Handle, name, index);
			}
			public void NuggetFlagsSet(ENuggetFlag flags, bool state, string? name = null, int index = 0)
			{
				var prev = NuggetFlags;
				View3D_ObjectNuggetFlagsSet(Handle, flags, state, name, index);
				if (prev != NuggetFlags)
				{
					NotifyPropertyChanged(nameof(NuggetFlags));
				}
			}

			/// <summary>
			/// Get/Set the nugget tint colour for a nugget within this object
			/// See LdrObject::Apply for docs on the format of 'name'</summary>
			public Colour32 NuggetTintGet(string? name = null, int index = 0)
			{
				return View3D_ObjectNuggetTintGet(Handle, name, index);
			}
			public void NuggetTintSet(Colour32 tint, string? name = null, int index = 0)
			{
				var prev = NuggetTint;
				View3D_ObjectNuggetTintSet(Handle, tint, name, index);
				if (prev != NuggetTint)
				{
					NotifyPropertyChanged(nameof(NuggetTint));
				}
			}

			/// <summary>
			/// Get/Set the colour of this object or the first child object that matches 'name'.
			/// 'base_colour', if true returns the objects base colour, if false, returns the current colour.
			/// If 'name' is null, then the state change is applied to this object only
			/// If 'name' is "", then the state change is applied to this object and all children recursively
			/// Otherwise, the state change is applied to all child objects that match name.
			/// If 'name' begins with '#' then the remainder of the name is treated as a regular expression</summary>
			public Colour32 ColourGet(bool base_colour, string? name = null)
			{
				return View3D_ObjectColourGet(Handle, base_colour, name);
			}
			public void ColourSet(bool base_colour, Colour32 colour, string? name = null, EColourOp op = EColourOp.Overwrite, float op_value = 0.0f)
			{
				var prev = Colour;
				View3D_ObjectColourSet(Handle, base_colour, colour, name, op, op_value);
				if (prev != Colour)
				{
					NotifyPropertyChanged(nameof(Colour));
				}
			}
			public void ColourSet(Colour32 colour, string? name = null)
			{
				ColourSet(false, colour, name);
			}

			/// <summary>
			/// Reset the colour to the base colour for this object or any of its child objects that match 'name'.
			/// If 'name' is null, then the state change is applied to this object only
			/// If 'name' is "", then the state change is applied to this object and all children recursively
			/// Otherwise, the state change is applied to all child objects that match name.
			/// If 'name' begins with '#' then the remainder of the name is treated as a regular expression</summary>
			public void ResetColour(string? name = null)
			{
				View3D_ObjectResetColour(Handle, name);
				NotifyPropertyChanged(nameof(Colour));
			}

			/// <summary>
			/// Get/Set the reflectivity of this object or the first child object that matches 'name'.
			/// If 'name' is null, then the state change is applied to this object only
			/// If 'name' is "", then the state change is applied to this object and all children recursively
			/// Otherwise, the state change is applied to all child objects that match name.
			/// If 'name' begins with '#' then the remainder of the name is treated as a regular expression</summary>
			public float ReflectivityGet(string? name = null)
			{
				return View3D_ObjectReflectivityGet(Handle, name);
			}
			public void ReflectivitySet(float reflectivity, string? name = null)
			{
				View3D_ObjectReflectivitySet(Handle, reflectivity, name);
				NotifyPropertyChanged(nameof(Reflectivity));
			}

			/// <summary>
			/// Set the texture on this object or any of its child objects that match 'name'.
			/// If 'name' is null, then the state change is applied to this object only
			/// If 'name' is "", then the state change is applied to this object and all children recursively
			/// Otherwise, the state change is applied to all child objects that match name.
			/// If 'name' begins with '#' then the remainder of the name is treated as a regular expression</summary>
			public void SetTexture(Texture tex, string? name = null)
			{
				View3D_ObjectSetTexture(Handle, tex.Handle, name);
			}

			/// <summary>String description of the object</summary>
			public string Description => $"{Name} ChildCount={ChildCount} Vis={Visible} Flags={Flags}";
			public override string ToString()
			{
				return Description;
			}

			/// <summary>Binding support</summary>
			public event PropertyChangedEventHandler? PropertyChanged;
			public void NotifyPropertyChanged(string prop_name)
			{
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
				ObjectChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
			}
			public void Refresh() => NotifyPropertyChanged(string.Empty);

			/// <summary>Raised when any object is changed</summary>
			public static event PropertyChangedEventHandler? ObjectChanged;

			/// <summary>Wrapper for converting EditObjectCB to EditObjectCBInternal</summary>
			unsafe private class EditCB
			{
				private readonly EditObjectCB m_edit_cb;
				private readonly EditObjectCBInternal m_callback;

				public EditCB(EditObjectCB edit_cb)
				{
					m_edit_cb = edit_cb;
					m_callback = new EditObjectCBInternal { m_cb = EditObjectCallback };
				}
				
				private VICount EditObjectCallback(IntPtr ctx, int vcount, int icount, IntPtr vptr, IntPtr iptr, AddNuggetCBInternal out_nugget)
				{
					// Convert IntPtr to a Span<Vertex> (direct memory access)
					var verts = new Span<Vertex>((Vertex*)vptr, vcount);
					var indices = new Span<ushort>((ushort*)iptr, icount);
					void AddNugget(ref Nugget nug) => out_nugget.m_cb(out_nugget.m_ctx, ref nug);
					return m_edit_cb(verts, indices, AddNugget);
				}

				public static implicit operator EditObjectCBInternal(EditCB cb) => cb.m_callback;
			}

			#region Equals
			public static bool operator ==(Object? lhs, Object? rhs)
			{
				return ReferenceEquals(lhs, rhs) || Equals(lhs, rhs);
			}
			public static bool operator !=(Object? lhs, Object? rhs)
			{
				return !(lhs == rhs);
			}
			public bool Equals(Object? rhs)
			{
				return rhs != null && Handle == rhs.Handle;
			}
			public override bool Equals(object? rhs)
			{
				return Equals(rhs as Object);
			}
			public override int GetHashCode()
			{
				return Handle.GetHashCode();
			}
			#endregion
		}
	}
}
