using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Xml.Linq;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Maths;

namespace LDraw
{
	public class SettingsData : SettingsBase<SettingsData>
	{
		public SettingsData()
		{
			RecentFiles = string.Empty;
			Profiles = [new SettingsProfile { Name = SettingsProfile.DefaultProfileName }];
			AutoSaveOnChanges = true;
		}
		public SettingsData(string filepath)
			: base(filepath, ESettingsLoadFlags.ThrowOnError)
		{
			AutoSaveOnChanges = true;
		}

		/// <inheritdoc/>
		public override string Version => "v2.0";

		/// <summary>Recently loaded files</summary>
		public string RecentFiles
		{
			get => get<string>(nameof(RecentFiles));
			set => set(nameof(RecentFiles), value);
		}

		/// <summary>Saved configurations</summary>
		public List<SettingsProfile> Profiles
		{
			get => get<List<SettingsProfile>>(nameof(Profiles));
			private set => set(nameof(Profiles), value);
		}
	}

	/// <summary>Per Scene settings</summary>
	[DebuggerDisplay("{Name,nq}")]
	public class SettingsProfile :SettingsSet<SettingsProfile>
	{
		public SettingsProfile()
		{
			Name = "Profile";
			FontName = "Consolas";
			FontSize = 10.0;
			AutoRefresh = false;
			ResetOnLoad = true;
			ReloadChangedScripts = null;
			ClearErrorLogOnReload = true;
			CheckForChangesPollPeriodS = 1.0;
			IncludePaths = Array.Empty<string>();
			TextEditorPath = string.Empty;
			TextEditorArguments = "--reuse-window --goto \"{file}:{line}\"";
			StreamingPort = 1976;
			SceneState = new List<SceneStateData>();
			UILayout = null;
		}
		public SettingsProfile(SettingsProfile rhs)
			:base(rhs)
		{}

		/// <summary>Name of the default profile</summary>
		public static string DefaultProfileName => "Default Profile";

		/// <summary>The name of this profile</summary>
		public string Name
		{
			get => get<string>(nameof(Name));
			set => set(nameof(Name), value);
		}
		
		/// <summary>The font to use in scripts UIs</summary>
		public string FontName
		{
			get => get<string>(nameof(FontName));
			set => set(nameof(FontName), value);
		}

		/// <summary>The font size</summary>
		public double FontSize
		{
			get => get<double>(nameof(FontSize));
			set => set(nameof(FontSize), value);
		}

		/// <summary>Auto reload script sources when changes are detected</summary>
		public bool AutoRefresh
		{
			get => get<bool>(nameof(AutoRefresh));
			set => set(nameof(AutoRefresh), value);
		}

		/// <summary>True if the scene should auto range after loading files</summary>
		public bool ResetOnLoad
		{
			get => get<bool>(nameof(ResetOnLoad));
			set => set(nameof(ResetOnLoad), value);
		}

		/// <summary>Where scripts changed externally are automatically reloaded. Null means prompt</summary>
		public bool? ReloadChangedScripts
		{
			get => get<bool?>(nameof(ReloadChangedScripts));
			set => set(nameof(ReloadChangedScripts), value);
		}

		/// <summary>Clear the error log when source data is reloaded</summary>
		public bool ClearErrorLogOnReload
		{
			get => get<bool>(nameof(ClearErrorLogOnReload));
			set => set(nameof(ClearErrorLogOnReload), value);
		}

		/// <summary>The period between checking for changed files</summary>
		public double CheckForChangesPollPeriodS
		{
			get => get<double>(nameof(CheckForChangesPollPeriodS));
			set => set(nameof(CheckForChangesPollPeriodS), value);
		}

		/// <summary>Includes paths to use when resolving includes in script files</summary>
		public string[] IncludePaths
		{
			get => get<string[]>(nameof(IncludePaths));
			set => set(nameof(IncludePaths), value);
		}

		/// <summary>Path to the text editor executable used for opening source files from error logs</summary>
		public string TextEditorPath
		{
			get => get<string>(nameof(TextEditorPath));
			set => set(nameof(TextEditorPath), value);
		}

		/// <summary>Command line arguments pattern for the text editor. Use {file} and {line} as placeholders.</summary>
		public string TextEditorArguments
		{
			get => get<string>(nameof(TextEditorArguments));
			set => set(nameof(TextEditorArguments), value);
		}

		/// <summary>The port to listen to for incoming streaming connections</summary>
		public int StreamingPort
		{
			get => get<int>(nameof(StreamingPort));
			set => set(nameof(StreamingPort), value);
		}

		/// <summary>Per Scene settings</summary>
		public List<SceneStateData> SceneState
		{
			get => get<List<SceneStateData>>(nameof(SceneState));
			private set => set(nameof(SceneState), value);
		}

		/// <summary>Layout state of the main UI</summary>
		public XElement? UILayout
		{
			get => get<XElement>(nameof(UILayout));
			set => set(nameof(UILayout), value);
		}
	}

	/// <summary>Per Scene settings</summary>
	public class SceneStateData :SettingsSet<SceneStateData>
	{
		public SceneStateData()
		{
			Name = string.Empty;
			ViewPreset = EViewPreset.Current;
			AlignDirection = EAlignDirection.None;
			Chart = new ChartControl.OptionsData
			{
				BackgroundColour = Colour32.Gray,
				ShowAxes = false,
				ShowGridLines = false,
				FocusPointVisible = true,
				OriginPointVisible = false,
				NavigationMode = ChartControl.ENavMode.Scene3D,
				LockAspect = 1.0,
			};
			Lighting = new LightData();
			RayTracing = new RayTracingData();
		}

		/// <summary>The name of the scene that this state data belongs to</summary>
		public string Name
		{
			get => get<string>(nameof(Name));
			set => set(nameof(Name), value);
		}

		/// <summary>Pre-set view directions</summary>
		public EViewPreset ViewPreset
		{
			get => get<EViewPreset>(nameof(ViewPreset));
			set => set(nameof(ViewPreset), value);
		}

		/// <summary>Directions to align the camera up-axis to</summary>
		public EAlignDirection AlignDirection
		{
			get => get<EAlignDirection>(nameof(AlignDirection));
			set => set(nameof(AlignDirection), value);
		}

		/// <summary>Options for scene behaviour, common to all scenes</summary>
		public ChartControl.OptionsData Chart
		{
			get => get<ChartControl.OptionsData>(nameof(Chart));
			set => set(nameof(Chart), value);
		}

		/// <summary>Light source settings for this scene</summary>
		public LightData Lighting
		{
			get => get<LightData>(nameof(Lighting));
			set => set(nameof(Lighting), value);
		}

		/// <summary>Ray tracing settings for this scene</summary>
		public RayTracingData RayTracing
		{
			get => get<RayTracingData>(nameof(RayTracing));
			set => set(nameof(RayTracing), value);
		}
	}

	/// <summary>Per-scene ray tracing settings</summary>
	public class RayTracingData :SettingsSet<RayTracingData>
	{
		public RayTracingData()
		{
			Enabled = false;
			ReflectionsEnabled = true;
			CausticsEnabled = true;
			MaxReflectionBounces = 1;
		}

		/// <summary>True if ray tracing is enabled for this scene</summary>
		public bool Enabled
		{
			get => get<bool>(nameof(Enabled));
			set => set(nameof(Enabled), value);
		}

		/// <summary>True if ray traced reflections are enabled for this scene</summary>
		public bool ReflectionsEnabled
		{
			get => get<bool>(nameof(ReflectionsEnabled));
			set => set(nameof(ReflectionsEnabled), value);
		}

		/// <summary>True if ray traced caustics are enabled for this scene</summary>
		public bool CausticsEnabled
		{
			get => get<bool>(nameof(CausticsEnabled));
			set => set(nameof(CausticsEnabled), value);
		}

		/// <summary>Maximum number of ray traced reflection bounces</summary>
		public int MaxReflectionBounces
		{
			get => get<int>(nameof(MaxReflectionBounces));
			set => set(nameof(MaxReflectionBounces), Math.Clamp(value, 1, 4));
		}

		/// <summary>Convert the persisted feature settings to View3D feature flags</summary>
		public View3d.ERayTracingFeature ToView3dFeatures()
		{
			var features = View3d.ERayTracingFeature.None;
			if (ReflectionsEnabled)
				features |= View3d.ERayTracingFeature.Reflections;
			if (CausticsEnabled)
				features |= View3d.ERayTracingFeature.Caustics;

			return features;
		}

		/// <summary>Convert the persisted ray tracing settings to View3D ray tracing properties</summary>
		public View3d.RayTracingProps ToRayTracingProps()
		{
			return new View3d.RayTracingProps
			{
				Features = ToView3dFeatures(),
				MaxReflectionBounces = MaxReflectionBounces,
			};
		}

		/// <summary>Copy View3D feature flags into the persisted feature settings</summary>
		public void FromView3dFeatures(View3d.ERayTracingFeature features)
		{
			switch (features)
			{
			case View3d.ERayTracingFeature.None:
			case View3d.ERayTracingFeature.Reflections:
			case View3d.ERayTracingFeature.Caustics:
			case View3d.ERayTracingFeature.All:
				{
					ReflectionsEnabled = features.HasFlag(View3d.ERayTracingFeature.Reflections);
					CausticsEnabled = features.HasFlag(View3d.ERayTracingFeature.Caustics);
					break;
				}
			default:
				{
					throw new ArgumentOutOfRangeException(nameof(features), features, "Unknown ray tracing feature flags");
				}
			}
		}

		/// <summary>Copy View3D ray tracing properties into the persisted ray tracing settings</summary>
		public void FromRayTracingProps(View3d.RayTracingProps props)
		{
			FromView3dFeatures(props.Features);
			MaxReflectionBounces = props.MaxReflectionBounces;
		}
	}

	/// <summary>Per-scene light source settings (mirrors View3d.LightInfo as a SettingsSet for persistence)</summary>
	public class LightData :SettingsSet<LightData>
	{
		public LightData()
			: this(View3d.LightInfo.Directional(-v4.ZAxis, camera_relative: true))
		{
		}
		public LightData(View3d.LightInfo info)
		{
			Position = info.Position;
			Direction = info.Direction;
			Type = info.Type;
			AmbientColour = info.AmbientColour;
			DiffuseColour = info.DiffuseColour;
			SpecularColour = info.SpecularColour;
			SpecularPower = info.SpecularPower;
			Range = info.Range;
			Falloff = info.Falloff;
			InnerAngle = info.InnerAngle;
			OuterAngle = info.OuterAngle;
			CastShadow = info.CastShadow;
			CameraRelative = info.CameraRelative;
			On = info.On;
		}

		public v4 Position
		{
			get => get<v4>(nameof(Position));
			set => set(nameof(Position), value);
		}
		public v4 Direction
		{
			get => get<v4>(nameof(Direction));
			set => set(nameof(Direction), value);
		}
		public View3d.ELight Type
		{
			get => get<View3d.ELight>(nameof(Type));
			set => set(nameof(Type), value);
		}
		public Colour32 AmbientColour
		{
			get => get<Colour32>(nameof(AmbientColour));
			set => set(nameof(AmbientColour), value);
		}
		public Colour32 DiffuseColour
		{
			get => get<Colour32>(nameof(DiffuseColour));
			set => set(nameof(DiffuseColour), value);
		}
		public Colour32 SpecularColour
		{
			get => get<Colour32>(nameof(SpecularColour));
			set => set(nameof(SpecularColour), value);
		}
		public float SpecularPower
		{
			get => get<float>(nameof(SpecularPower));
			set => set(nameof(SpecularPower), value);
		}
		public float Range
		{
			get => get<float>(nameof(Range));
			set => set(nameof(Range), value);
		}
		public float Falloff
		{
			get => get<float>(nameof(Falloff));
			set => set(nameof(Falloff), value);
		}
		public float InnerAngle
		{
			get => get<float>(nameof(InnerAngle));
			set => set(nameof(InnerAngle), value);
		}
		public float OuterAngle
		{
			get => get<float>(nameof(OuterAngle));
			set => set(nameof(OuterAngle), value);
		}
		public float CastShadow
		{
			get => get<float>(nameof(CastShadow));
			set => set(nameof(CastShadow), value);
		}
		public bool CameraRelative
		{
			get => get<bool>(nameof(CameraRelative));
			set => set(nameof(CameraRelative), value);
		}
		public bool On
		{
			get => get<bool>(nameof(On));
			set => set(nameof(On), value);
		}

		/// <summary>Convert to a native LightInfo struct</summary>
		public View3d.LightInfo ToLightInfo()
		{
			return new View3d.LightInfo
			{
				Position = Position,
				Direction = Direction,
				Type = Type,
				AmbientColour = AmbientColour,
				DiffuseColour = DiffuseColour,
				SpecularColour = SpecularColour,
				SpecularPower = SpecularPower,
				Range = Range,
				Falloff = Falloff,
				InnerAngle = InnerAngle,
				OuterAngle = OuterAngle,
				CastShadow = CastShadow,
				CameraRelative = CameraRelative,
				On = On,
			};
		}

		/// <summary>Update from a native LightInfo struct (per-field; equal values short-circuit)</summary>
		public void FromLightInfo(View3d.LightInfo info)
		{
			Position = info.Position;
			Direction = info.Direction;
			Type = info.Type;
			AmbientColour = info.AmbientColour;
			DiffuseColour = info.DiffuseColour;
			SpecularColour = info.SpecularColour;
			SpecularPower = info.SpecularPower;
			Range = info.Range;
			Falloff = info.Falloff;
			InnerAngle = info.InnerAngle;
			OuterAngle = info.OuterAngle;
			CastShadow = info.CastShadow;
			CameraRelative = info.CameraRelative;
			On = info.On;
		}
	}

	/// <summary>Extensions</summary>
	public static class SettingsData_
	{
		/// <summary>Access the scene state data for a scene by name</summary>
		public static SceneStateData get(this IList<SceneStateData> container, string name)
		{
			var ssd = container.FirstOrDefault(x => x.Name == name);
			ssd ??= container.Add2(new SceneStateData { Name = name });
			return ssd;
		}
	}
}
