using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>Atmosphere records and entry points in the versioned native Physics ABI.</summary>
internal static unsafe partial class Native
{
	[StructLayout(LayoutKind.Sequential)]
	internal struct AtmosphereDesc
	{
		internal NativeHeader m_header;
		internal Vector4i m_cell_count;
		internal v4 m_origin;
		internal float m_dx;
		internal float m_lid_z;
		internal float m_first_layer_thickness;
		internal float* m_floor_heights;
		internal byte* m_active_columns;
		internal fixed int m_boundaries[6];
		internal fixed float m_wall_drag[6];
		internal float m_reference_temperature;
		internal float m_lapse_rate;
		internal float m_min_temperature;
		internal float m_gravity;
		internal float m_floor_exchange_rate;
		internal float m_lid_temperature;
		internal float m_lid_relaxation_rate;
		internal int m_pressure_vcycles;
		internal int m_pressure_pre_smooth;
		internal int m_pressure_post_smooth;
		internal int m_pressure_coarse_smooth;
		internal int m_open_edge_band;
		internal float m_vorticity_confinement;
		internal float m_vertical_viscosity;
		internal int m_tracer_count;
		internal uint m_tracer_seed;
		internal float m_tracer_max_age;
		internal float m_tracer_ground_density;
		internal float m_tracer_break_density;
		internal float m_tracer_upper_density;
		internal float m_tracer_break_height;
		private int m_reserved;

		/// <summary>Convert managed creation options into the exact native layout. Pinned arrays must stay pinned until creation returns.</summary>
		internal static AtmosphereDesc From(AtmosphereOptions options, float* floor_heights, byte* active_columns)
		{
			var result = new AtmosphereDesc
			{
				m_header = NativeHeader.Create<AtmosphereDesc>(),
				m_cell_count = new Vector4i(options.CellCountX, options.CellCountY, options.CellCountZ, 0),
				m_origin = new v4(options.OriginX, options.OriginY, options.OriginZ, 1.0f),
				m_dx = options.CellSize,
				m_lid_z = options.LidZ,
				m_first_layer_thickness = options.FirstLayerThickness,
				m_floor_heights = floor_heights,
				m_active_columns = active_columns,
				m_reference_temperature = options.ReferenceTemperature,
				m_lapse_rate = options.LapseRate,
				m_min_temperature = options.MinTemperature,
				m_gravity = options.Gravity,
				m_floor_exchange_rate = options.FloorExchangeRate,
				m_lid_temperature = options.LidTemperature,
				m_lid_relaxation_rate = options.LidRelaxationRate,
				m_pressure_vcycles = options.PressureVCycles,
				m_pressure_pre_smooth = options.PressurePreSmooth,
				m_pressure_post_smooth = options.PressurePostSmooth,
				m_pressure_coarse_smooth = options.PressureCoarseSmooth,
				m_open_edge_band = options.OpenEdgeBand,
				m_vorticity_confinement = options.VorticityConfinement,
				m_vertical_viscosity = options.VerticalViscosity,
				m_tracer_count = options.TracerCount,
				m_tracer_seed = options.TracerSeed,
				m_tracer_max_age = options.TracerMaxAge,
				m_tracer_ground_density = options.TracerGroundDensity,
				m_tracer_break_density = options.TracerBreakDensity,
				m_tracer_upper_density = options.TracerUpperDensity,
				m_tracer_break_height = options.TracerBreakHeight,
			};

			// The side arrays are indexed by EAtmosphereSide in both the managed options and the native record.
			for (var i = 0; i != 6; ++i)
			{
				result.m_boundaries[i] = (int)options.Boundaries[i];
				result.m_wall_drag[i] = options.WallDrag[i];
			}
			return result;
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct AtmosphereStepDesc
	{
		internal NativeHeader m_header;
		internal float m_dt;
		internal int m_heat_source_count;
		internal AtmosphereHeatSource* m_heat_sources;
		internal int m_probe_count;
		private int m_reserved;
		internal v4* m_probes;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct AtmosphereStats
	{
		internal NativeHeader m_header;
		internal float m_max_speed;
		internal float m_max_divergence;
		internal float m_rms_divergence;
		internal float m_mean_top_temperature;
		internal float m_peak_vertical_velocity;
		private int m_reserved;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct AtmosphereTracerSlot
	{
		internal IntPtr m_resource;
		internal int m_slot;
		internal uint m_count;
		internal uint m_stride;
		private uint m_reserved;
	}

	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereCreate(ulong engine, AtmosphereDesc* desc, out ulong atmosphere);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereDestroy(ulong engine, ulong atmosphere);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereBeginStep(ulong engine, ulong atmosphere, AtmosphereStepDesc* step);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmospherePollStep(ulong engine, ulong atmosphere, out int idle);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereProbesCopy(ulong engine, ulong atmosphere, AtmosphereProbeSample* samples, uint capacity, out uint required);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereCompleteStep(ulong engine, ulong atmosphere);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereFloorsSet(ulong engine, ulong atmosphere, float* floor_heights, int count);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereFloorTemperaturesSet(ulong engine, ulong atmosphere, float* floor_temperatures, int count);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereOutsideAirSet(ulong engine, ulong atmosphere, AtmosphereOutsideAir* outside_air, int count);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereTracersCopy(ulong engine, ulong atmosphere, AtmosphereTracerParticle* particles, uint capacity, out uint required);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereTracersAcquire(ulong engine, ulong atmosphere, AtmosphereTracerSlot* slot);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereTracersRelease(ulong engine, ulong atmosphere, int slot, IntPtr fence, ulong value);
	[DllImport(Dll)] internal static extern EStatus Physics_AtmosphereCellStatesCopy(ulong engine, ulong atmosphere, AtmosphereCellState* cells, uint capacity, out uint required, AtmosphereStats* stats);
}
