using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Xml.Linq;
using LDraw.MCP;
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
			MCP = new McpSettingsData();
			AutoSaveOnChanges = true;
		}
		public SettingsData(string filepath)
			: base(filepath, ESettingsLoadFlags.ThrowOnError)
		{
			if (!has(nameof(MCP)))
				MCP = new McpSettingsData();

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

		/// <summary>Model Context Protocol server settings</summary>
		public McpSettingsData MCP
		{
			get => get<McpSettingsData>(nameof(MCP));
			private set => set(nameof(MCP), value);
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

		/// <summary>Access the persisted state for 'name', creating it when the scene is new</summary>
		public SceneStateData GetOrAddSceneState(string name)
		{
			var scene_state = SceneState.FirstOrDefault(x => x.Name == name);
			if (scene_state != null)
				return scene_state;

			// Parent dynamically added state so its changes participate in profile notifications and auto-save.
			scene_state = new SceneStateData { Name = name, Parent = this };
			SceneState.Add(scene_state);
			NotifySettingChanged(nameof(SceneState));
			return scene_state;
		}

		/// <summary>Remove persisted state for a scene that no longer exists</summary>
		public void RemoveSceneState(SceneStateData scene_state)
		{
			if (!SceneState.Remove(scene_state))
				return;

			scene_state.Parent = null;
			NotifySettingChanged(nameof(SceneState));
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
			Ambient = new Colour32(0xFF808080);
			Shadows = new ShadowData();
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

		/// <summary>Main light (light 0) settings for this scene</summary>
		public LightData Lighting
		{
			get => get<LightData>(nameof(Lighting));
			set => set(nameof(Lighting), value);
		}

		/// <summary>Scene-wide ambient light colour for this scene</summary>
		public Colour32 Ambient
		{
			get => get<Colour32>(nameof(Ambient));
			set => set(nameof(Ambient), value);
		}

		/// <summary>Shadow rendering settings for this scene</summary>
		public ShadowData Shadows
		{
			get => get<ShadowData>(nameof(Shadows));
			set => set(nameof(Shadows), value);
		}

		/// <summary>Ray tracing settings for this scene</summary>
		public RayTracingData RayTracing
		{
			get => get<RayTracingData>(nameof(RayTracing));
			set => set(nameof(RayTracing), value);
		}
	}

	/// <summary>Per-scene shadow rendering settings. See View3d.ShadowSettings</summary>
	public class ShadowData :SettingsSet<ShadowData>
	{
		public ShadowData()
		{
			FromShadowSettings(View3d.ShadowSettings.Default());
		}

		/// <summary>Width and height of the square shadow atlas (in pixels)</summary>
		public int AtlasSize
		{
			get => get<int>(nameof(AtlasSize));
			set => set(nameof(AtlasSize), value);
		}

		/// <summary>Requested size of each directional light cascade view (in pixels)</summary>
		public int DirectionalResolution
		{
			get => get<int>(nameof(DirectionalResolution));
			set => set(nameof(DirectionalResolution), value);
		}

		/// <summary>Largest size of a spot light shadow view (in pixels)</summary>
		public int SpotResolution
		{
			get => get<int>(nameof(SpotResolution));
			set => set(nameof(SpotResolution), value);
		}

		/// <summary>Largest size of each point light cube face shadow view (in pixels)</summary>
		public int PointResolution
		{
			get => get<int>(nameof(PointResolution));
			set => set(nameof(PointResolution), value);
		}

		/// <summary>The maximum number of lights that cast shadows</summary>
		public int MaxShadowLights
		{
			get => get<int>(nameof(MaxShadowLights));
			set => set(nameof(MaxShadowLights), value);
		}

		/// <summary>The number of cascades for directional lights, in [1,4]</summary>
		public int CascadeCount
		{
			get => get<int>(nameof(CascadeCount));
			set => set(nameof(CascadeCount), value);
		}

		/// <summary>Distance from the camera beyond which directional lights cast no shadows. Zero means fit to the shadow casters</summary>
		public float ShadowDistance
		{
			get => get<float>(nameof(ShadowDistance));
			set => set(nameof(ShadowDistance), value);
		}

		/// <summary>Cascade split distribution in [0,1]. 0 = even spacing, 1 = logarithmic spacing</summary>
		public float CascadeSplitBlend
		{
			get => get<float>(nameof(CascadeSplitBlend));
			set => set(nameof(CascadeSplitBlend), value);
		}

		/// <summary>Width of the shadow edge filter (in shadow texels). Either 5 or 7</summary>
		public int FilterSize
		{
			get => get<int>(nameof(FilterSize));
			set => set(nameof(FilterSize), value);
		}

		/// <summary>Constant depth bias (in units of the smallest depth step)</summary>
		public int DepthBias
		{
			get => get<int>(nameof(DepthBias));
			set => set(nameof(DepthBias), value);
		}

		/// <summary>Depth bias scaled by the depth slope of each triangle</summary>
		public float SlopeBias
		{
			get => get<float>(nameof(SlopeBias));
			set => set(nameof(SlopeBias), value);
		}

		/// <summary>Receiver offset along the surface normal (in shadow texels)</summary>
		public float NormalBias
		{
			get => get<float>(nameof(NormalBias));
			set => set(nameof(NormalBias), value);
		}

		/// <summary>Re-render shadow views only when their content changes</summary>
		public bool CacheViews
		{
			get => get<bool>(nameof(CacheViews));
			set => set(nameof(CacheViews), value);
		}

		/// <summary>Convert the persisted settings to View3D shadow settings</summary>
		public View3d.ShadowSettings ToShadowSettings()
		{
			return new View3d.ShadowSettings
			{
				AtlasSize = AtlasSize,
				DirectionalResolution = DirectionalResolution,
				SpotResolution = SpotResolution,
				PointResolution = PointResolution,
				MaxShadowLights = MaxShadowLights,
				CascadeCount = CascadeCount,
				ShadowDistance = ShadowDistance,
				CascadeSplitBlend = CascadeSplitBlend,
				FilterSize = FilterSize,
				DepthBias = DepthBias,
				SlopeBias = SlopeBias,
				NormalBias = NormalBias,
				CacheViews = CacheViews,
			};
		}

		/// <summary>Copy View3D shadow settings into the persisted settings</summary>
		public void FromShadowSettings(View3d.ShadowSettings settings)
		{
			AtlasSize = settings.AtlasSize;
			DirectionalResolution = settings.DirectionalResolution;
			SpotResolution = settings.SpotResolution;
			PointResolution = settings.PointResolution;
			MaxShadowLights = settings.MaxShadowLights;
			CascadeCount = settings.CascadeCount;
			ShadowDistance = settings.ShadowDistance;
			CascadeSplitBlend = settings.CascadeSplitBlend;
			FilterSize = settings.FilterSize;
			DepthBias = settings.DepthBias;
			SlopeBias = settings.SlopeBias;
			NormalBias = settings.NormalBias;
			CacheViews = settings.CacheViews;
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
			DiffuseColour = info.DiffuseColour;
			SpecularColour = info.SpecularColour;
			SpecularPower = info.SpecularPower;
			Intensity = info.Intensity;
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
		public float Intensity
		{
			get => get<float>(nameof(Intensity));
			set => set(nameof(Intensity), value);
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
				DiffuseColour = DiffuseColour,
				SpecularColour = SpecularColour,
				SpecularPower = SpecularPower,
				Intensity = Intensity,
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
			DiffuseColour = info.DiffuseColour;
			SpecularColour = info.SpecularColour;
			SpecularPower = info.SpecularPower;
			Intensity = info.Intensity;
			Range = info.Range;
			Falloff = info.Falloff;
			InnerAngle = info.InnerAngle;
			OuterAngle = info.OuterAngle;
			CastShadow = info.CastShadow;
			CameraRelative = info.CameraRelative;
			On = info.On;
		}
	}

}
