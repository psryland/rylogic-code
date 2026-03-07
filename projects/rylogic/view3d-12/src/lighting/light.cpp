//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/lighting/light.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"

namespace pr::rdr12
{
	Light::Light()
		:m_position(v4::Origin())
		,m_direction(0, 0, -1, 0)
		,m_type(ELight::Directional)
		,m_ambient(0.25f, 0.25f, 0.25f, 0.0f)
		,m_diffuse(0.25f, 0.25f, 0.25f, 1.0f)
		,m_specular(0.1f, 0.1f, 0.1f, 0.0f)
		,m_specular_power(1000.0f)
		,m_range(100.0f)
		,m_falloff(0.0f)
		,m_inner_angle(constants<float>::tau_by_4)
		,m_outer_angle(constants<float>::tau_by_4)
		,m_cast_shadow(0.0f)
		,m_cam_relative(false)
		,m_on(true)
	{}

	// Return true if this light is in a valid state
	bool Light::IsValid() const
	{
		switch (m_type)
		{
			case ELight::Ambient:     return true;
			case ELight::Point:       return m_position.w == 1.0f;
			case ELight::Spot:        return m_direction != v4::Zero();
			case ELight::Directional: return m_direction != v4::Zero();
			default: return false;
		}
	}

	// Returns a light to world transform appropriate for this light type and facing 'centre'
	m4x4 Light::LightToWorld(v4 centre, float centre_dist, m4x4 const& c2w) const
	{
		auto pos = m_cam_relative ? c2w * m_position : m_position;
		auto dir = m_cam_relative ? c2w * m_direction : m_direction;
		auto preferred_up = m_cam_relative ? c2w.y : v4::YAxis();
		centre_dist = centre_dist != 0 ? centre_dist : 1.0f;
		switch (m_type)
		{
			case ELight::Directional: return m4x4::LookAt(centre - centre_dist * dir, centre, Perpendicular(dir, preferred_up));
			case ELight::Point:       return m4x4::LookAt(pos, centre, Perpendicular(centre - pos, preferred_up));
			case ELight::Spot:        return m4x4::LookAt(pos, centre, Perpendicular(centre - pos, preferred_up));
			default:                  return m4x4::Identity();
		}
	}

	// Returns a projection transform appropriate for this light type
	m4x4 Light::Projection(float zn, float zf, float w, float h, float focus_dist) const
	{
		auto s = zn / focus_dist;
		switch (m_type)
		{
			case ELight::Directional: return m4x4::ProjectionOrthographic(w, h, zn, zf, true);
			case ELight::Point:       return m4x4::ProjectionPerspective(w * s, h * s, zn, zf, true);
			case ELight::Spot:        return m4x4::ProjectionPerspective(w * s, h * s, zn, zf, true);
			default:                  return m4x4::Identity();
		}
	}
	m4x4 Light::ProjectionFOV(float zn, float zf, float aspect, float fovY, float focus_dist) const
	{
		auto height = 2.0f * focus_dist * tan(fovY * 0.5f);
		switch (m_type)
		{
			case ELight::Directional: return m4x4::ProjectionOrthographic(height * aspect, height, zn, zf, true);
			case ELight::Point:       return m4x4::ProjectionPerspectiveFOV(fovY, aspect, zn, zf, true);
			case ELight::Spot:        return m4x4::ProjectionPerspectiveFOV(fovY, aspect, zn, zf, true);
			default:                  return m4x4::Identity();
		}
	}

	enum class ELightKW
	{
		#define PR_ENUM(x)\
		x(Pos  ,= rdr12::ldraw::HashI("Pos" ))\
		x(Dir  ,= rdr12::ldraw::HashI("Dir" ))\
		x(Type ,= rdr12::ldraw::HashI("Type"))\
		x(Amb  ,= rdr12::ldraw::HashI("Amb" ))\
		x(Diff ,= rdr12::ldraw::HashI("Diff"))\
		x(Spec ,= rdr12::ldraw::HashI("Spec"))\
		x(SPwr ,= rdr12::ldraw::HashI("SPwr"))\
		x(Ang0 ,= rdr12::ldraw::HashI("Ang0"))\
		x(Ang1 ,= rdr12::ldraw::HashI("Ang1"))\
		x(Rng  ,= rdr12::ldraw::HashI("Rng" ))\
		x(FOff ,= rdr12::ldraw::HashI("FOff"))\
		x(Shdw ,= rdr12::ldraw::HashI("Shdw"))\
		x(On   ,= rdr12::ldraw::HashI("On"  ))\
		x(CRel ,= rdr12::ldraw::HashI("CRel"))
		PR_ENUM_MEMBERS2(PR_ENUM)
	};
	PR_ENUM_REFLECTION2(ELightKW, PR_ENUM);
	#undef PR_ENUM

	// Get/Set light settings
	std::string Light::Settings() const
	{
		std::stringstream out;
		out << "  *" << ELightKW::Pos  << "{" << m_position.xyz << "}\n"
			<< "  *" << ELightKW::Dir  << "{" << m_direction.xyz << "}\n"
			<< "  *" << ELightKW::Type << "{" << m_type << "}\n"
			<< std::hex
			<< "  *" << ELightKW::Amb  << "{" << m_ambient.argb << "}\n"
			<< "  *" << ELightKW::Diff << "{" << m_diffuse.argb << "}\n"
			<< "  *" << ELightKW::Spec << "{" << m_specular.argb << "}\n"
			<< std::dec
			<< "  *" << ELightKW::SPwr << "{" << m_specular_power << "}\n"
			<< "  *" << ELightKW::Ang0 << "{" << m_inner_angle << "}\n"
			<< "  *" << ELightKW::Ang1 << "{" << m_outer_angle << "}\n"
			<< "  *" << ELightKW::Rng  << "{" << m_range << "}\n"
			<< "  *" << ELightKW::FOff << "{" << m_falloff << "}\n"
			<< "  *" << ELightKW::Shdw << "{" << m_cast_shadow << "}\n"
			<< "  *" << ELightKW::On   << "{" << m_on << "}\n"
			<< "  *" << ELightKW::CRel << "{" << m_cam_relative << "}\n"
			;
		return out.str();
	}
	void Light::Settings(std::string_view settings)
	{
		try
		{
			// Parse the settings for light, if no errors are found update *this
			Light light;

			// Parse the settings
			mem_istream<char> src(settings);
			ldraw::TextReader reader(src, {});
			for (ELightKW kw; reader.NextKeyword(kw); ) switch (kw)
			{
				case ELightKW::Pos:  light.m_position = reader.Vector3f().w1(); break;
				case ELightKW::Dir:  light.m_direction = reader.Vector3f().w0(); break;
				case ELightKW::Type: light.m_type = reader.Enum<ELight>(); break;
				case ELightKW::Amb:  light.m_ambient = reader.Int<uint32_t>(16); break;
				case ELightKW::Diff: light.m_diffuse = reader.Int<uint32_t>(16); break;
				case ELightKW::Spec: light.m_specular = reader.Int<uint32_t>(16); break;
				case ELightKW::SPwr: light.m_specular_power = reader.Real<float>(); break;
				case ELightKW::Ang0: light.m_inner_angle = reader.Real<float>(); break;
				case ELightKW::Ang1: light.m_outer_angle = reader.Real<float>(); break;
				case ELightKW::Rng:  light.m_range = reader.Real<float>(); break;
				case ELightKW::FOff: light.m_falloff = reader.Real<float>(); break;
				case ELightKW::Shdw: light.m_cast_shadow = reader.Real<float>(); break;
				case ELightKW::On:   light.m_on = reader.Bool(); break;
				case ELightKW::CRel: light.m_cam_relative = reader.Bool(); break;
			}
			*this = light;
		}
		catch (std::exception const& e)
		{
			throw std::invalid_argument(e.what());
		}
	}
}