using System;
using System.Runtime.InteropServices;
using System.Text;
using Rylogic.Interop.Win32;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>Fixed-layout P/Invoke surface for the versioned native Physics ABI.</summary>
internal static unsafe class Native
{
	internal const string Dll = "physics";
	internal const uint ApiVersion = 0x00010000U;
	internal const uint StructVersion = 1U;
	private static IntPtr m_module;

	/// <summary>Load the configuration-appropriate native runtime before the first P/Invoke.</summary>
	internal static void EnsureLoaded()
	{
		if (m_module != IntPtr.Zero)
			return;

		m_module = Win32.LoadDll(Dll + ".dll", out var load_error);
		if (m_module == IntPtr.Zero)
			throw load_error ?? new DllNotFoundException($"Unable to load {Dll}.dll.");
	}

	[UnmanagedFunctionPointer(CallingConvention.StdCall)]
	internal delegate void ReportErrorFn(IntPtr context, [MarshalAs(UnmanagedType.LPStr)] string message, [MarshalAs(UnmanagedType.LPStr)] string filepath, int line, long position);

	[StructLayout(LayoutKind.Sequential)]
	internal struct ReportErrorCallback
	{
		internal IntPtr m_context;
		internal ReportErrorFn m_callback;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct EngineConfig
	{
		internal NativeHeader m_header;
		internal int m_max_collision_pairs;
		internal int m_sleeping_enabled;
		internal float m_sleep_velocity_threshold_linear;
		internal float m_sleep_velocity_threshold_angular;
		internal float m_sleep_delay_seconds;
		internal int m_solver_iterations;
		internal int m_push_out_iterations;
		internal float m_broadphase_aabb_margin;
		internal float m_contact_sort_propagation_scale;
		internal int m_contact_sort_shock_iterations;
		internal float m_contact_sort_shock_alignment;
		internal float m_contact_sort_shock_min_strength;
		internal float m_contact_sort_shock_decay;
		internal float m_penetration_slop;
		internal float m_velocity_baumgarte;
		internal float m_position_slop;
		internal float m_position_baumgarte;
		internal float m_contact_slop_scale;
		internal float m_support_contact_slop_scale;
		internal float m_warm_start_scale;
		internal float m_deep_penetration_threshold;
		internal float m_deep_penetration_range;
		internal float m_deep_penetration_baumgarte_min;
		internal float m_deep_penetration_baumgarte_max;
		internal int m_selective_refresh_passes;
		internal int m_selective_refresh_max_pairs;
		internal int m_selective_refresh_body_limit;
		internal int m_selective_refresh_contact_limit;
		internal int m_selective_refresh_solver_iterations;
		internal int m_selective_refresh_position_iterations;
		internal float m_selective_refresh_bias_scale;
		internal float m_selective_refresh_restitution_scale;
		internal int m_selective_refresh_adaptive_body_limit;
		internal int m_selective_refresh_adaptive_solver_iterations;
		internal int m_selective_refresh_support_only;
		internal int m_selective_refresh_resolve_support_only;
		internal float m_selective_refresh_depth_slop;
		internal float m_selective_refresh_support_depth_slop;
		internal float m_selective_refresh_closing_speed_slop;
		internal float m_selective_refresh_support_alignment;
		internal float m_selective_refresh_aabb_margin;

		/// <summary>Convert caller-facing options into the exact ABI layout.</summary>
		internal static EngineConfig From(EngineOptions options)
		{
			return new EngineConfig
			{
				m_header = NativeHeader.Create<EngineConfig>(),
				m_max_collision_pairs = options.MaxCollisionPairs,
				m_sleeping_enabled = options.SleepingEnabled ? 1 : 0,
				m_sleep_velocity_threshold_linear = options.SleepVelocityThresholdLinear,
				m_sleep_velocity_threshold_angular = options.SleepVelocityThresholdAngular,
				m_sleep_delay_seconds = options.SleepDelaySeconds,
				m_solver_iterations = options.SolverIterations,
				m_push_out_iterations = options.PushOutIterations,
				m_broadphase_aabb_margin = options.BroadphaseAabbMargin,
				m_contact_sort_propagation_scale = options.ContactSortPropagationScale,
				m_contact_sort_shock_iterations = options.ContactSortShockIterations,
				m_contact_sort_shock_alignment = options.ContactSortShockAlignment,
				m_contact_sort_shock_min_strength = options.ContactSortShockMinStrength,
				m_contact_sort_shock_decay = options.ContactSortShockDecay,
				m_penetration_slop = options.PenetrationSlop,
				m_velocity_baumgarte = options.VelocityBaumgarte,
				m_position_slop = options.PositionSlop,
				m_position_baumgarte = options.PositionBaumgarte,
				m_contact_slop_scale = options.ContactSlopScale,
				m_support_contact_slop_scale = options.SupportContactSlopScale,
				m_warm_start_scale = options.WarmStartScale,
				m_deep_penetration_threshold = options.DeepPenetrationThreshold,
				m_deep_penetration_range = options.DeepPenetrationRange,
				m_deep_penetration_baumgarte_min = options.DeepPenetrationBaumgarteMin,
				m_deep_penetration_baumgarte_max = options.DeepPenetrationBaumgarteMax,
				m_selective_refresh_passes = options.SelectiveRefreshPasses,
				m_selective_refresh_max_pairs = options.SelectiveRefreshMaxPairs,
				m_selective_refresh_body_limit = options.SelectiveRefreshBodyLimit,
				m_selective_refresh_contact_limit = options.SelectiveRefreshContactLimit,
				m_selective_refresh_solver_iterations = options.SelectiveRefreshSolverIterations,
				m_selective_refresh_position_iterations = options.SelectiveRefreshPositionIterations,
				m_selective_refresh_bias_scale = options.SelectiveRefreshBiasScale,
				m_selective_refresh_restitution_scale = options.SelectiveRefreshRestitutionScale,
				m_selective_refresh_adaptive_body_limit = options.SelectiveRefreshAdaptiveBodyLimit,
				m_selective_refresh_adaptive_solver_iterations = options.SelectiveRefreshAdaptiveSolverIterations,
				m_selective_refresh_support_only = options.SelectiveRefreshSupportOnly ? 1 : 0,
				m_selective_refresh_resolve_support_only = options.SelectiveRefreshResolveSupportOnly ? 1 : 0,
				m_selective_refresh_depth_slop = options.SelectiveRefreshDepthSlop,
				m_selective_refresh_support_depth_slop = options.SelectiveRefreshSupportDepthSlop,
				m_selective_refresh_closing_speed_slop = options.SelectiveRefreshClosingSpeedSlop,
				m_selective_refresh_support_alignment = options.SelectiveRefreshSupportAlignment,
				m_selective_refresh_aabb_margin = options.SelectiveRefreshAabbMargin,
			};
		}

		/// <summary>Convert the exact ABI layout into caller-facing engine options.</summary>
		internal EngineOptions ToPublic()
		{
			return new EngineOptions
			{
				MaxCollisionPairs = m_max_collision_pairs,
				SleepingEnabled = m_sleeping_enabled != 0,
				SleepVelocityThresholdLinear = m_sleep_velocity_threshold_linear,
				SleepVelocityThresholdAngular = m_sleep_velocity_threshold_angular,
				SleepDelaySeconds = m_sleep_delay_seconds,
				SolverIterations = m_solver_iterations,
				PushOutIterations = m_push_out_iterations,
				BroadphaseAabbMargin = m_broadphase_aabb_margin,
				ContactSortPropagationScale = m_contact_sort_propagation_scale,
				ContactSortShockIterations = m_contact_sort_shock_iterations,
				ContactSortShockAlignment = m_contact_sort_shock_alignment,
				ContactSortShockMinStrength = m_contact_sort_shock_min_strength,
				ContactSortShockDecay = m_contact_sort_shock_decay,
				PenetrationSlop = m_penetration_slop,
				VelocityBaumgarte = m_velocity_baumgarte,
				PositionSlop = m_position_slop,
				PositionBaumgarte = m_position_baumgarte,
				ContactSlopScale = m_contact_slop_scale,
				SupportContactSlopScale = m_support_contact_slop_scale,
				WarmStartScale = m_warm_start_scale,
				DeepPenetrationThreshold = m_deep_penetration_threshold,
				DeepPenetrationRange = m_deep_penetration_range,
				DeepPenetrationBaumgarteMin = m_deep_penetration_baumgarte_min,
				DeepPenetrationBaumgarteMax = m_deep_penetration_baumgarte_max,
				SelectiveRefreshPasses = m_selective_refresh_passes,
				SelectiveRefreshMaxPairs = m_selective_refresh_max_pairs,
				SelectiveRefreshBodyLimit = m_selective_refresh_body_limit,
				SelectiveRefreshContactLimit = m_selective_refresh_contact_limit,
				SelectiveRefreshSolverIterations = m_selective_refresh_solver_iterations,
				SelectiveRefreshPositionIterations = m_selective_refresh_position_iterations,
				SelectiveRefreshBiasScale = m_selective_refresh_bias_scale,
				SelectiveRefreshRestitutionScale = m_selective_refresh_restitution_scale,
				SelectiveRefreshAdaptiveBodyLimit = m_selective_refresh_adaptive_body_limit,
				SelectiveRefreshAdaptiveSolverIterations = m_selective_refresh_adaptive_solver_iterations,
				SelectiveRefreshSupportOnly = m_selective_refresh_support_only != 0,
				SelectiveRefreshResolveSupportOnly = m_selective_refresh_resolve_support_only != 0,
				SelectiveRefreshDepthSlop = m_selective_refresh_depth_slop,
				SelectiveRefreshSupportDepthSlop = m_selective_refresh_support_depth_slop,
				SelectiveRefreshClosingSpeedSlop = m_selective_refresh_closing_speed_slop,
				SelectiveRefreshSupportAlignment = m_selective_refresh_support_alignment,
				SelectiveRefreshAabbMargin = m_selective_refresh_aabb_margin,
			};
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct ShapeCommon
	{
		internal NativeHeader m_header;
		internal m4x4 m_shape_to_root;
		internal int m_material_id;
		internal uint m_flags;

		/// <summary>Create the common shape prefix with the size of its complete enclosing description.</summary>
		internal static ShapeCommon From<T>(ShapeOptions? options)
		{
			options ??= new ShapeOptions();
			return new ShapeCommon
			{
				m_header = NativeHeader.Create<T>(),
				m_shape_to_root = options.ShapeToRoot,
				m_material_id = options.MaterialId,
				m_flags = options.Flags,
			};
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct SphereShape
	{
		internal ShapeCommon m_common;
		internal float m_radius;
		internal int m_hollow;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct BoxShape
	{
		internal ShapeCommon m_common;
		internal v4 m_dimensions;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct LineShape
	{
		internal ShapeCommon m_common;
		internal float m_length;
		internal float m_radius;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct TriangleShape
	{
		internal ShapeCommon m_common;
		internal v4 m_a;
		internal v4 m_b;
		internal v4 m_c;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct BodyDesc
	{
		internal NativeHeader m_header;
		internal ShapeHandle m_shape;
		internal m4x4 m_object_to_world;
		internal BodyInertia m_inertia;
		internal SpatialVector m_momentum;
		internal v4 m_gravity;
		internal ulong m_user_tag;
		internal EMotionType m_motion_type;
		internal EMassMode m_mass_mode;
		internal float m_mass_or_density;
		internal EBodyFlags m_flags;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct BodyState
	{
		internal NativeHeader m_header;
		internal BodyHandle m_body;
		internal ShapeHandle m_shape;
		internal m4x4 m_object_to_world;
		internal BodyInertia m_inertia;
		internal SpatialVector m_momentum;
		internal SpatialVector m_velocity;
		internal SpatialVector m_force;
		internal v4 m_gravity;
		internal ulong m_user_tag;
		internal EMotionType m_motion_type;
		internal EBodyFlags m_flags;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct MaterialValue
	{
		internal NativeHeader m_header;
		internal int m_id;
		internal float m_static_friction;
		internal float m_normal_elasticity;
		internal float m_tangential_elasticity;
		internal float m_torsional_elasticity;
		internal float m_density;

		/// <summary>Convert a public material into its versioned ABI representation.</summary>
		internal static MaterialValue From(Material material)
		{
			return new MaterialValue
			{
				m_header = NativeHeader.Create<MaterialValue>(),
				m_id = material.m_id,
				m_static_friction = material.m_static_friction,
				m_normal_elasticity = material.m_normal_elasticity,
				m_tangential_elasticity = material.m_tangential_elasticity,
				m_torsional_elasticity = material.m_torsional_elasticity,
				m_density = material.m_density,
			};
		}

		/// <summary>Convert a versioned ABI material to its caller-facing value.</summary>
		internal Material ToPublic()
		{
			return new Material(m_id, m_static_friction, m_normal_elasticity, m_tangential_elasticity, m_torsional_elasticity, m_density);
		}
	}

	/// <summary>Throw a managed exception containing the native thread-local error message.</summary>
	internal static void Check(EStatus status)
	{
		if (status == EStatus.Success)
			return;

		uint required;
		Physics_LastError(null, 0, out required);
		var bytes = new byte[Math.Max(required, 1)];
		fixed (byte* ptr = bytes)
		{
			Physics_LastError(ptr, (uint)bytes.Length, out required);
		}
		var count = Array.IndexOf(bytes, (byte)0);
		if (count < 0)
			count = bytes.Length;

		var message = Encoding.UTF8.GetString(bytes, 0, count);
		throw new PhysicsException(status, string.IsNullOrWhiteSpace(message) ? $"Native physics call failed with status {status}." : message);
	}

	[DllImport(Dll)] internal static extern uint Physics_ApiVersion();
	[DllImport(Dll)] internal static extern IntPtr Physics_Initialise(ReportErrorCallback callback);
	[DllImport(Dll)] internal static extern void Physics_Shutdown(IntPtr context);
	[DllImport(Dll)] private static extern EStatus Physics_LastError(byte* buffer, uint capacity, out uint required);
	[DllImport(Dll)] internal static extern EStatus Physics_StructSize(int struct_id, out uint size);

	[DllImport(Dll)] internal static extern EStatus Physics_EngineCreate(IntPtr context, EngineConfig* config, IntPtr external_device, out ulong engine);
	[DllImport(Dll)] internal static extern EStatus Physics_EngineDestroy(ulong engine);
	[DllImport(Dll)] internal static extern void Physics_EngineAbandon(ulong engine);
	[DllImport(Dll)] internal static extern EStatus Physics_EngineDeviceLeaseAcquire(ulong engine, out IntPtr device);
	[DllImport(Dll)] internal static extern EStatus Physics_EngineConfigGet(ulong engine, EngineConfig* config);
	[DllImport(Dll)] internal static extern EStatus Physics_EngineConfigSet(ulong engine, EngineConfig* config);
	[DllImport(Dll)] internal static extern EStatus Physics_MaterialGet(ulong engine, int material_id, MaterialValue* material);
	[DllImport(Dll)] internal static extern EStatus Physics_MaterialSet(ulong engine, MaterialValue* material);

	[DllImport(Dll)] internal static extern EStatus Physics_ShapeCreateSphere(ulong engine, SphereShape* desc, out ulong shape);
	[DllImport(Dll)] internal static extern EStatus Physics_ShapeCreateBox(ulong engine, BoxShape* desc, out ulong shape);
	[DllImport(Dll)] internal static extern EStatus Physics_ShapeCreateLine(ulong engine, LineShape* desc, out ulong shape);
	[DllImport(Dll)] internal static extern EStatus Physics_ShapeCreateTriangle(ulong engine, TriangleShape* desc, out ulong shape);
	[DllImport(Dll)] internal static extern EStatus Physics_ShapeCreatePolytope(ulong engine, ShapeCommon* common, v4* points, uint point_count, out ulong shape);
	[DllImport(Dll)] internal static extern EStatus Physics_ShapeCreateCompound(ulong engine, ShapeCommon* common, ShapeHandle* children, uint child_count, out ulong shape);
	[DllImport(Dll)] internal static extern EStatus Physics_ShapeDestroy(ulong engine, ulong shape);

	[DllImport(Dll)] internal static extern EStatus Physics_BodyCreate(ulong engine, BodyDesc* desc, out ulong body);
	[DllImport(Dll)] internal static extern EStatus Physics_BodyDestroy(ulong engine, ulong body);
	[DllImport(Dll)] internal static extern EStatus Physics_BodyStateGet(ulong engine, ulong body, BodyState* state);
	[DllImport(Dll)] internal static extern EStatus Physics_BodyStateSet(ulong engine, ulong body, BodyState* state);
	[DllImport(Dll)] internal static extern EStatus Physics_CommandsApply(ulong engine, BodyCommand* commands, uint command_count);
	[DllImport(Dll)] internal static extern EStatus Physics_BeginStep(ulong engine, float elapsed_seconds, double absolute_time_seconds, BodyCommand* commands, uint command_count);
	[DllImport(Dll)] internal static extern EStatus Physics_CompleteStep(ulong engine);
	[DllImport(Dll)] internal static extern EStatus Physics_Step(ulong engine, float elapsed_seconds, double absolute_time_seconds, BodyCommand* commands, uint command_count);
	[DllImport(Dll)] internal static extern EStatus Physics_SnapshotCopy(ulong engine, BodySnapshot* snapshots, uint capacity, out uint required);
	[DllImport(Dll)] internal static extern EStatus Physics_EventsCopy(ulong engine, PhysicsEvent* events, uint capacity, out uint required);
	[DllImport(Dll)] internal static extern EStatus Physics_DiagnosticsGet(ulong engine, Diagnostics* diagnostics);
	[DllImport(Dll)] internal static extern EStatus Physics_CheckpointSize(ulong engine, out ulong required);
	[DllImport(Dll)] internal static extern EStatus Physics_CheckpointWrite(ulong engine, void* buffer, ulong capacity, out ulong written);
	[DllImport(Dll)] internal static extern EStatus Physics_CheckpointRead(ulong engine, void* buffer, ulong size);

	[DllImport("kernel32.dll")] internal static extern uint GetCurrentThreadId();
}
