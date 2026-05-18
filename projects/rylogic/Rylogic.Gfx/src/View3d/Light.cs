//#define PR_VIEW3D_CREATE_STACKTRACE
using System;
using System.ComponentModel;
using System.Xml.Linq;
using Rylogic.Extn;
using Rylogic.Maths;

namespace Rylogic.Gfx
{
	public sealed partial class View3d
	{
		/// <summary>Reference type wrapper of a view3d light</summary>
		public class Light :INotifyPropertyChanged
		{
			public Light()
			{
				m_info = LightInfo.Ambient();
			}
			public Light(LightInfo light)
			{
				m_info = light;
			}
			public Light(Colour32 ambient, Colour32 diffuse, Colour32 specular, double spec_power = 1000.0, v4? direction = null, v4? position = null, double intensity = 1.0)
				:this(
					direction != null ? LightInfo.Directional(direction.Value, ambient, diffuse, specular, (float)spec_power, 0, intensity: (float)intensity) :
					position  != null ? LightInfo.Point(position.Value, ambient, diffuse, specular, (float)spec_power, 0, intensity: (float)intensity) :
					LightInfo.Ambient(ambient, (float)intensity))
			{}
			public Light(XElement node)
				:this()
			{
				On             = node.Element(nameof(On)            ).As<bool>(On);
				Type           = node.Element(nameof(Type)          ).As<ELight>(Type);
				Position       = node.Element(nameof(Position)      ).As<v4>(Position);
				Direction      = node.Element(nameof(Direction)     ).As<v4>(Direction);
				Ambient        = node.Element(nameof(Ambient)       ).As<Colour32>(Ambient);
				Diffuse        = node.Element(nameof(Diffuse)       ).As<Colour32>(Diffuse);
				Specular       = node.Element(nameof(Specular)      ).As<Colour32>(Specular);
				SpecularPower  = node.Element(nameof(SpecularPower) ).As<double>(SpecularPower);
				Intensity      = node.Element(nameof(Intensity)     ).As<double>(Intensity);
				InnerAngle     = node.Element(nameof(InnerAngle)    ).As<double>(InnerAngle);
				OuterAngle     = node.Element(nameof(OuterAngle)    ).As<double>(OuterAngle);
				Range          = node.Element(nameof(Range)         ).As<double>(Range);
				Falloff        = node.Element(nameof(Falloff)       ).As<double>(Falloff);
				CastShadow     = node.Element(nameof(CastShadow)    ).As<double>(CastShadow);
				CameraRelative = node.Element(nameof(CameraRelative)).As<bool>(CameraRelative);
			}
			public XElement ToXml(XElement node)
			{
				node.Add2(nameof(Type          ), Type          , false);
				node.Add2(nameof(Position      ), Position      , false);
				node.Add2(nameof(Direction     ), Direction     , false);
				node.Add2(nameof(Ambient       ), Ambient       , false);
				node.Add2(nameof(Diffuse       ), Diffuse       , false);
				node.Add2(nameof(Specular      ), Specular      , false);
				node.Add2(nameof(SpecularPower ), SpecularPower , false);
				node.Add2(nameof(Intensity     ), Intensity     , false);
				node.Add2(nameof(InnerAngle    ), InnerAngle    , false);
				node.Add2(nameof(OuterAngle    ), OuterAngle    , false);
				node.Add2(nameof(Range         ), Range         , false);
				node.Add2(nameof(Falloff       ), Falloff       , false);
				node.Add2(nameof(CastShadow    ), CastShadow    , false);
				node.Add2(nameof(On            ), On            , false);
				node.Add2(nameof(CameraRelative), CameraRelative, false);
				return node;
			}

			/// <summary>The View3d Light data</summary>
			public LightInfo Data
			{
				get => m_info;
				set => SetProp(ref m_info, value, nameof(Data));
			}
			private LightInfo m_info;

			/// <summary>Whether the light is active or not</summary>
			public bool On
			{
				get => m_info.On;
				set => SetProp(ref m_info.On, value, nameof(On));
			}

			/// <summary>The type of light source</summary>
			public ELight Type
			{
				get => m_info.Type;
				set => SetProp(ref m_info.Type, value, nameof(Type));
			}

			/// <summary>The position of the light source</summary>
			public v4 Position
			{
				get => m_info.Position;
				set => SetProp(ref m_info.Position, value, nameof(Position));
			}

			/// <summary>The direction of the light source</summary>
			public v4 Direction
			{
				get => m_info.Direction;
				set => SetProp(ref m_info.Direction, Math_.Normalise(value, -v4.ZAxis), nameof(Direction));
			}

			/// <summary>The colour of the ambient component of the light</summary>
			public Colour32 Ambient
			{
				get => m_info.AmbientColour;
				set => SetProp(ref m_info.AmbientColour, value, nameof(Ambient));
			}

			/// <summary>The colour of the diffuse component of the light</summary>
			public Colour32 Diffuse
			{
				get => m_info.DiffuseColour;
				set => SetProp(ref m_info.DiffuseColour, value, nameof(Diffuse));
			}

			/// <summary>The colour of the specular component of the light</summary>
			public Colour32 Specular
			{
				get => m_info.SpecularColour;
				set => SetProp(ref m_info.SpecularColour, value, nameof(Specular));
			}

			/// <summary>The specular power</summary>
			public double SpecularPower
			{
				get => m_info.SpecularPower;
				set => SetProp(ref m_info.SpecularPower, (float)value, nameof(SpecularPower));
			}

			/// <summary>The light intensity scale</summary>
			public double Intensity
			{
				get => m_info.Intensity;
				set => SetProp(ref m_info.Intensity, (float)Math.Max(0.0, value), nameof(Intensity));
			}

			/// <summary>The inner spot light cone angle (in degrees)</summary>
			public double InnerAngle
			{
				get => Math_.RadiansToDegrees(m_info.InnerAngle);
				set
				{
					var angle = Math_.DegreesToRadians(Math_.Clamp(value, 0, 180));
					SetProp(ref m_info.InnerAngle, (float)angle, nameof(InnerAngle));
				}
			}

			/// <summary>The outer spot light cone angle (in degrees)</summary>
			public double OuterAngle
			{
				get => Math_.RadiansToDegrees(m_info.OuterAngle);
				set
				{
					var angle = Math_.DegreesToRadians(Math_.Clamp(value, 0, 180));
					SetProp(ref m_info.OuterAngle, (float)angle, nameof(OuterAngle));
				}
			}

			/// <summary>The range of the light</summary>
			public double Range
			{
				get => m_info.Range;
				set => SetProp(ref m_info.Range, (float)value, nameof(Range));
			}

			/// <summary>The attenuation of the light with distance</summary>
			public double Falloff
			{
				get => m_info.Falloff;
				set => SetProp(ref m_info.Falloff, (float)value, nameof(Falloff));
			}

			/// <summary>The maximum distance from the light source in which objects cast shadows</summary>
			public double CastShadow
			{
				get => m_info.CastShadow;
				set => SetProp(ref m_info.CastShadow, (float)value, nameof(CastShadow));
			}

			/// <summary>Whether the light moves with the camera or not</summary>
			public bool CameraRelative
			{
				get => m_info.CameraRelative;
				set => SetProp(ref m_info.CameraRelative, value, nameof(CameraRelative));
			}

			/// <summary>Notify property value changed</summary>
			public event PropertyChangedEventHandler? PropertyChanged;
			private void SetProp<T>(ref T prop, T value, string name)
			{
				if (Equals(prop, value)) return;
				prop = value;
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
			}

			/// <summary>Implicit conversion to the value type</summary>
			public static implicit operator LightInfo(Light light) { return light.m_info; }
		}
	}
}
