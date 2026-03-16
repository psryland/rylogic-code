//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "pr/view3d-12/ldraw/ldraw_serialiser.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/ldraw/ldraw_reader_binary.h"
#include "pr/view3d-12/ldraw/ldraw_commands.h"
#include "pr/view3d-12/compute/gpu_job.h"
#include "pr/view3d-12/lighting/light.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/model_desc.h"
#include "pr/view3d-12/model/model_generator.h"
#include "pr/view3d-12/model/model_tree.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/model/animation.h"
#include "pr/view3d-12/model/skeleton.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/sampler/sampler_desc.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/shaders/shader_arrow_head.h"
#include "pr/view3d-12/shaders/shader_point_sprites.h"
#include "pr/view3d-12/shaders/shader_thick_line.h"
#include "pr/view3d-12/texture/texture_desc.h"

#include "pr/str/extract.h"
#include "pr/geometry/convex_hull.h"
#include "pr/geometry/index_buffer.h"
#include "pr/container/byte_data.h"
#include "pr/script/forward.h"
#include "pr/storage/csv.h"

namespace pr::rdr12::ldraw
{
	// Notes:
	//  - Error Handling:
	//    Don't assume the report error callback will throw, try to continue or fail gracefully.

	using Guid = pr::Guid;
	using VCont = pr::vector<v4>;

	// Static random number generator (replaces the old maths g_rng)
	inline std::default_random_engine& g_rng()
	{
		static std::default_random_engine s_rng(std::random_device{}());
		return s_rng;
	}

	using NCont = pr::vector<v4>;
	using ICont = geometry::IdxBuf;
	using CCont = pr::vector<Colour32>;
	using TCont = pr::vector<v2>;
	using GCont = pr::vector<NuggetDesc>;
	using Font = ModelGenerator::Font;
	using TextFormat = ModelGenerator::TextFormat;
	using TextLayout = ModelGenerator::TextLayout;
	using ObjectLookup = ParseResult::ObjectLookup;

	enum class EFlags
	{
		None = 0,
		ExplicitName = 1 << 0,
		ExplicitColour = 1 << 1,
		_flags_enum = 0,
	};

	// Template prototype for ObjectCreators
	struct IObjectCreator;
	template <ELdrObject ObjType> struct ObjectCreator;

	// Cached geometry buffers
	#pragma region Buffer Pool / Cache
	struct alignas(16) Buffers
	{
		VCont m_verts;
		ICont m_index;
		NCont m_norms;
		CCont m_color;
		TCont m_texts;
		GCont m_nugts;
		
		// Resize all buffers to zero
		void Reset()
		{
			m_verts.resize(0);
			m_norms.resize(0);
			m_index.resize(0, sizeof(uint16_t));
			m_color.resize(0);
			m_texts.resize(0);
			m_nugts.resize(0);
		}
	};
	struct Cache
	{
		using BuffersPtr = std::unique_ptr<Buffers>;

		BuffersPtr m_bptr;
		VCont& m_verts;
		ICont& m_index;
		NCont& m_norms;
		CCont& m_color;
		TCont& m_texts;
		GCont& m_nugts;

		Cache()
			: m_bptr(GetFromPool())
			, m_verts(m_bptr->m_verts)
			, m_index(m_bptr->m_index)
			, m_norms(m_bptr->m_norms)
			, m_color(m_bptr->m_color)
			, m_texts(m_bptr->m_texts)
			, m_nugts(m_bptr->m_nugts)
		{
		}
		~Cache()
		{
			m_bptr->Reset();
			ReturnToPool(std::move(m_bptr));
		}
		Cache(Cache&&) = delete;
		Cache(Cache const&) = delete;
		Cache operator =(Cache&&) = delete;
		Cache operator =(Cache const&) = delete;

		void Reset()
		{
			m_bptr->Reset();
		}

	private:

		// Global buffers
		inline static std::vector<BuffersPtr> g_buffer_pool;
		inline static std::mutex g_buffer_pool_mutex;

		static BuffersPtr GetFromPool()
		{
			std::lock_guard<std::mutex> lock(g_buffer_pool_mutex);
			if (!g_buffer_pool.empty())
			{
				auto ptr = std::move(g_buffer_pool.back());
				g_buffer_pool.pop_back();
				return ptr;
			}
			return BuffersPtr(new Buffers()); // using make_unique causes a crash in x64 release here
		}
		static void ReturnToPool(BuffersPtr&& bptr)
		{
			std::lock_guard<std::mutex> lock(g_buffer_pool_mutex);
			g_buffer_pool.push_back(std::move(bptr));
		}
	};
	#pragma endregion

	// Helper object for passing parameters between parsing functions
	struct ParseParams
	{
		// Notes:
		// - Ldr object can be created in a background thread. So there is a separate command list.
		using system_clock = std::chrono::system_clock;
		using time_point = std::chrono::time_point<system_clock>;
		using FontStack = pr::vector<Font>;
		using GpuJob = GpuJob<D3D12_COMMAND_LIST_TYPE_DIRECT>;

		Renderer&       m_rdr;
		ParseResult&    m_result;
		ObjectCont&     m_objects;
		ObjectLookup&   m_lookup;
		ResourceFactory m_factory;
		ReportErrorCB   m_report_error;
		Guid            m_context_id;
		Cache           m_cache;
		ELdrObject      m_type;
		LdrObject*      m_parent;
		IObjectCreator* m_parent_creator;
		FontStack       m_font;
		ParseProgressCB m_progress_cb;
		time_point      m_last_progress_update;
		EFlags          m_flags;
		bool&           m_cancel;
		std::stop_token m_stop_token;

		ParseParams(Renderer& rdr, ParseResult& result, Guid const& context_id, ReportErrorCB error_cb, ParseProgressCB progress_cb, bool& cancel, std::stop_token stop_token = {})
			: m_rdr(rdr)
			, m_result(result)
			, m_objects(result.m_objects)
			, m_lookup(result.m_lookup)
			, m_factory(rdr)
			, m_report_error(error_cb)
			, m_context_id(context_id)
			, m_cache()
			, m_type(ELdrObject::Unknown)
			, m_parent()
			, m_parent_creator()
			, m_font(1)
			, m_progress_cb(progress_cb)
			, m_last_progress_update(system_clock::now())
			, m_flags(EFlags::None)
			, m_cancel(cancel)
			, m_stop_token(std::move(stop_token))
		{}
		ParseParams(ParseParams& pp, ObjectCont& objects, LdrObject* parent, IObjectCreator* parent_creator)
			: m_rdr(pp.m_rdr)
			, m_result(pp.m_result)
			, m_objects(objects)
			, m_lookup(pp.m_lookup)
			, m_factory(pp.m_rdr)
			, m_report_error(pp.m_report_error)
			, m_context_id(pp.m_context_id)
			, m_cache()
			, m_type(ELdrObject::Unknown)
			, m_parent(parent)
			, m_parent_creator(parent_creator)
			, m_font(pp.m_font)
			, m_progress_cb(pp.m_progress_cb)
			, m_last_progress_update(pp.m_last_progress_update)
			, m_flags(pp.m_flags)
			, m_cancel(pp.m_cancel)
			, m_stop_token(pp.m_stop_token)
		{}
		ParseParams(ParseParams&&) = delete;
		ParseParams(ParseParams const&) = delete;
		ParseParams& operator = (ParseParams&&) = delete;
		ParseParams& operator = (ParseParams const&) = delete;

		// Report an error in the script
		void ReportError(EParseError code, Location const& loc, std::string_view msg)
		{
			if (m_report_error == nullptr)
				return;

			//if (msg.empty()) msg = To<std::string>::ToStringA(result);
			m_report_error(code, loc, msg);
		}

		// Give a progress update
		void ReportProgress(IReader& reader, int r = 0)
		{
			using namespace std::chrono;

			// Reduce report progress frequency based on external loop counter
			if ((r % 0xfff) != 0)
				return;

			// Check for stop_token cancellation
			if (m_stop_token.stop_requested())
			{
				m_cancel = true;
				return;
			}

			// Callback provided?
			if (m_progress_cb == nullptr)
				return;

			// Limit callbacks to once every X seconds.
			if (system_clock::now() - m_last_progress_update < milliseconds(200))
				return;

			// Call the callback with the freshly minted object.
			// If the callback returns false, abort parsing.
			m_cancel = !m_progress_cb(m_context_id, m_result, reader.Loc(), false);
			m_last_progress_update = system_clock::now();
		}
	};

	// Info on a texture in an ldr object
	struct TextureInfo
	{
		m4x4 m_t2s = m4x4::Identity();
		std::filesystem::path m_filepath = {"#white"};
		TextureDesc m_tdesc = {};
		SamDesc m_sdesc = {SamDesc::LinearWrap()};
		bool m_has_alpha = false;
	};

	// Information on a animation ldr object
	struct RootAnimInfo
	{
		EAnimStyle m_style = EAnimStyle::Once;
		double m_period = {1.0}; // Seconds
		v4 m_vel = {}; // Linear velocity of the animation in m/s
		v4 m_acc = {}; // Linear velocity of the animation in m/s
		v4 m_avel = {}; // Angular velocity of the animation in rad/s
		v4 m_aacc = {}; // Angular velocity of the animation in rad/s
	};

	// Forward declare the recursive object parsing function
	bool ParseLdrObject(ELdrObject type, IReader& reader, ParseParams& pp);

	#pragma region Parse Common Elements

	// Parse command blocks
	void ParseCommands(IReader& reader, ParseParams& pp, ParseResult& out)
	{
		// Parse a command block
		auto start_location = reader.Loc();
		auto section = reader.SectionScope();
		for (EKeyword kw; reader.NextKeyword(kw);)
		{
			if (kw != EKeyword::Data)
			{
				pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), "Unknown keyword in Command block");
				continue;
			}

			out.m_commands.pad_to(16);

			// Read the command id, then parse the associated command
			auto id = reader.Enum<ECommandId>();
			switch (id)
			{
				case ECommandId::Invalid:
				{
					break;
				}
				case ECommandId::AddToScene:
				{
					out.m_commands.push_back(Command_AddToScene{
						.m_id = id,
						.m_scene_id = reader.Int<int>(),
					});
					break;
				}
				case ECommandId::CameraToWorld:
				{
					out.m_commands.push_back(Command_CameraToWorld{
						.m_id = id,
						.m_c2w = reader.Matrix4x4(),
					});
					break;
				}
				case ECommandId::CameraPosition:
				{
					out.m_commands.push_back(Command_CameraPosition{
						.m_id = id,
						.m_pos = reader.Vector3f().w1(),
					});
					break;
				}
				case ECommandId::ObjectToWorld:
				{
					auto cmd = Command_ObjectToWorld{
						.m_id = id,
						.m_object_name = {},
						.m_o2w = {},
					};
					auto obj_name = reader.Identifier<string32>();
					memcpy(&cmd.m_object_name[0], obj_name.c_str(), std::min(_countof(cmd.m_object_name) - 1, obj_name.size()));
					cmd.m_o2w = reader.Matrix4x4();
					out.m_commands.push_back(cmd);
					break;
				}
				case ECommandId::Render:
				{
					out.m_commands.push_back(Command_Render{
						.m_id = id,
						.m_scene_id = reader.Int<int>(),
					});
					break;
				}
				default:
				{
					pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), "Unsupported command");
					break;
				}
			}
		}
	}

	// Parse a camera description
	void ParseCamera(IReader& reader, ParseParams& pp, ParseResult& out)
	{
		auto section = reader.SectionScope();
		for (EKeyword kw; reader.NextKeyword(kw);)
		{
			switch (kw)
			{
				case EKeyword::O2W:
				{
					auto c2w = m4x4::Identity();
					reader.Transform(c2w);
					out.m_cam.CameraToWorld(c2w);
					out.m_cam_fields |= ECamField::C2W;
					break;
				}
				case EKeyword::LookAt:
				{
					auto lookat = reader.Vector3f().w1();
					m4x4 c2w = out.m_cam.CameraToWorld();
					out.m_cam.LookAt(c2w.pos, lookat, c2w.y);
					out.m_cam_fields |= ECamField::C2W;
					out.m_cam_fields |= ECamField::Focus;
					break;
				}
				case EKeyword::Align:
				{
					auto align = reader.Vector3f().w0();
					out.m_cam.Align(align);
					out.m_cam_fields |= ECamField::Align;
					break;
				}
				case EKeyword::Aspect:
				{
					auto aspect = reader.Real<float>();
					out.m_cam.Aspect(aspect);
					out.m_cam_fields |= ECamField::Align;
					break;
				}
				case EKeyword::FovX:
				{
					auto fovX = reader.Real<float>();
					out.m_cam.FovX(fovX);
					out.m_cam_fields |= ECamField::FovY;
					break;
				}
				case EKeyword::FovY:
				{
					auto fovY = reader.Real<float>();
					out.m_cam.FovY(fovY);
					out.m_cam_fields |= ECamField::FovY;
					break;
				}
				case EKeyword::Fov:
				{
					auto fov = reader.Vector2f();
					out.m_cam.Fov(fov.x, fov.y);
					out.m_cam_fields |= ECamField::Aspect;
					out.m_cam_fields |= ECamField::FovY;
					break;
				}
				case EKeyword::Near:
				{
					auto near_ = reader.Real<float>();
					out.m_cam.Near(near_, true);
					out.m_cam_fields |= ECamField::Near;
					break;
				}
				case EKeyword::Far:
				{
					auto far_ = reader.Real<float>();
					out.m_cam.Far(far_, true);
					out.m_cam_fields |= ECamField::Far;
					break;
				}
				case EKeyword::Orthographic:
				{
					out.m_cam.Orthographic(true);
					out.m_cam_fields |= ECamField::Ortho;
					break;
				}
				default:
				{
					pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("Keyword '{}' is not valid within *Camera", EKeyword_::ToStringA(kw)));
					break;
				}
			}
		}
	}

	// Parse a font description
	void ParseFont(IReader& reader, ParseParams& pp, Font& font)
	{
		font.m_underline = false;
		font.m_strikeout = false;
		auto section = reader.SectionScope();
		for (EKeyword kw; reader.NextKeyword(kw);)
		{
			switch (kw)
			{
				case EKeyword::Name:
				{
					font.m_name = Widen(reader.String<string32>());
					break;
				}
				case EKeyword::Size:
				{
					font.m_size = reader.Real<float>();
					break;
				}
				case EKeyword::Colour:
				{
					font.m_colour = reader.Int<uint32_t>(16);
					break;
				}
				case EKeyword::Weight:
				{
					font.m_weight = s_cast<DWRITE_FONT_WEIGHT>(reader.Int<int>(10));
					break;
				}
				case EKeyword::Style:
				{
					auto ident = reader.Identifier<string32>();
					if (str::EqualI(ident, "normal")) font.m_style = DWRITE_FONT_STYLE_NORMAL;
					if (str::EqualI(ident, "italic")) font.m_style = DWRITE_FONT_STYLE_ITALIC;
					if (str::EqualI(ident, "oblique")) font.m_style = DWRITE_FONT_STYLE_OBLIQUE;
					break;
				}
				case EKeyword::Stretch:
				{
					font.m_stretch = s_cast<DWRITE_FONT_STRETCH>(reader.Int<int>(10));
					break;
				}
				case EKeyword::Underline:
				{
					font.m_underline = reader.IsSectionEnd() ? true : reader.Bool();
					break;
				}
				case EKeyword::Strikeout:
				{
					font.m_strikeout = reader.IsSectionEnd() ? true : reader.Bool();
					break;
				}
				default:
				{
					pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("Keyword '{}' is not valid within *Font", EKeyword_::ToStringA(kw)));
					break;
				}
			}
		}
	}

	// Parse a root animation description
	void ParseRootAnimation(IReader& reader, ParseParams& pp, RootAnimInfo& anim_info)
	{
		auto section = reader.SectionScope();
		for (EKeyword kw; reader.NextKeyword(kw);)
		{
			switch (kw)
			{
				case EKeyword::Style:
				{
					anim_info.m_style = reader.Enum<EAnimStyle>();
					break;
				}
				case EKeyword::Period:
				{
					anim_info.m_period = reader.Real<double>();
					break;
				}
				case EKeyword::Velocity:
				{
					anim_info.m_vel = reader.Vector3f().w0();
					break;
				}
				case EKeyword::Accel:
				{
					anim_info.m_acc = reader.Vector3f().w0();
					break;
				}
				case EKeyword::AngVelocity:
				{
					anim_info.m_avel = reader.Vector3f().w0();
					break;
				}
				case EKeyword::AngAccel:
				{
					anim_info.m_aacc = reader.Vector3f().w0();
					break;
				}
				default:
				{
					pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("Keyword '{}' is not valid within *RootAnimation", EKeyword_::ToStringA(kw)));
					break;
				}
			}
		}
	}

	// Parse a texture description
	void ParseTexture(IReader& reader, ParseParams& pp, TextureInfo& tex)
	{
		auto section = reader.SectionScope();
		for (EKeyword kw; reader.NextKeyword(kw);)
		{
			switch (kw)
			{
				case EKeyword::FilePath:
				{
					tex.m_filepath = reader.String<std::filesystem::path>();
					break;
				}
				case EKeyword::O2W:
				{
					reader.Transform(tex.m_t2s);
					break;
				}
				case EKeyword::Addr:
				{
					tex.m_sdesc.AddressU = s_cast<D3D12_TEXTURE_ADDRESS_MODE>(reader.Enum<EAddrMode>());
					tex.m_sdesc.AddressV = reader.IsSectionEnd() ? tex.m_sdesc.AddressU : s_cast<D3D12_TEXTURE_ADDRESS_MODE>(reader.Enum<EAddrMode>());
					tex.m_sdesc.AddressW = reader.IsSectionEnd() ? tex.m_sdesc.AddressV : s_cast<D3D12_TEXTURE_ADDRESS_MODE>(reader.Enum<EAddrMode>());
					break;
				}
				case EKeyword::Filter:
				{
					tex.m_sdesc.Filter = s_cast<D3D12_FILTER>(reader.Enum<EFilter>());
					break;
				}
				case EKeyword::Alpha:
				{
					tex.m_has_alpha = reader.IsSectionEnd() ? true : reader.Bool();;
					break;
				}
				default:
				{
					pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("Keyword '{}' is not valid within *Texture", EKeyword_::ToStringA(kw)));
					break;
				}
			}
		}
	}

	// Parse keywords that can appear in any section. Returns true if the keyword was recognised.
	bool ParseProperties(IReader& reader, ParseParams& pp, EKeyword kw, LdrObject* obj)
	{
		switch (kw)
		{
			case EKeyword::Name:
			{
				obj->m_name = reader.Identifier<string32>();
				pp.m_flags = SetBits(pp.m_flags, EFlags::ExplicitName, true);
				return true;
			}
			case EKeyword::Colour:
			{
				obj->m_base_colour = reader.Int<uint32_t>(16);
				pp.m_flags = SetBits(pp.m_flags, EFlags::ExplicitColour, true);
				return true;
			}
			case EKeyword::O2W:
			case EKeyword::Txfm:
			{
				reader.Transform(obj->m_o2p);
				obj->Flags(ELdrFlags::NonAffine, !IsAffine(obj->m_o2p));
				return true;
			}
			case EKeyword::GroupColour:
			{
				obj->m_grp_colour = reader.Int<uint32_t>(16);
				return true;
			}
			case EKeyword::Reflectivity:
			{
				obj->m_env = reader.Real<float>();
				return true;
			};
			case EKeyword::RandColour:
			{
				obj->m_base_colour = RandomRGB(g_rng(), 0.5f, 1.0f);
				return true;
			}
			case EKeyword::Font:
			{
				ParseFont(reader, pp, pp.m_font.back());
				return true;
			}
			case EKeyword::RootAnimation:
			{
				RootAnimInfo anim_info = {};
				ParseRootAnimation(reader, pp, anim_info);
				obj->m_root_anim.m_simple = { rdr12::New<rdr12::RootAnimation>(), true };
				obj->m_root_anim.m_simple->m_style = anim_info.m_style;
				obj->m_root_anim.m_simple->m_period = anim_info.m_period;
				obj->m_root_anim.m_simple->m_vel = anim_info.m_vel;
				obj->m_root_anim.m_simple->m_acc = anim_info.m_acc;
				obj->m_root_anim.m_simple->m_avel = anim_info.m_avel;
				obj->m_root_anim.m_simple->m_aacc = anim_info.m_aacc;
				obj->Flags(ELdrFlags::Animated, true);
				return true;
			}
			case EKeyword::Hidden:
			{
				auto hide = reader.IsSectionEnd() ? true : reader.Bool();
				obj->Flags(ELdrFlags::Hidden, hide);
				return true;
			}
			case EKeyword::Wireframe:
			{
				auto wire = reader.IsSectionEnd() ? true : reader.Bool();
				obj->Flags(ELdrFlags::Wireframe, wire);
				return true;
			}
			case EKeyword::NoZTest:
			{
				obj->Flags(ELdrFlags::NoZTest, true);
				return true;
			}
			case EKeyword::NoZWrite:
			{
				obj->Flags(ELdrFlags::NoZWrite, true);
				return true;
			}
			case EKeyword::ScreenSpace:
			{
				// Use a magic number to signal screen space mode to the ApplyState function
				obj->m_screen_space = Sub((multicast::IMultiCast*)1, 0);
				return true;
			}
			default:
			{
				return false;
			}
		}
	}

	// Apply the states such as colour,wireframe,etc to the objects renderer model
	void ApplyObjectState(LdrObject* obj)
	{
		// Set colour on 'obj' (so that render states are set correctly)
		obj->Colour(true, obj->m_base_colour);

		// Apply the group colour of 'obj' to all children
		if (obj->m_grp_colour != 0)
			obj->Colour(false, obj->m_grp_colour, "", EColourOp::Multiply);

		// If flagged as hidden, hide
		if (AllSet(obj->Flags(), ELdrFlags::Hidden))
			obj->Visible(false);

		// If flagged as wireframe, set wireframe
		if (AllSet(obj->Flags(), ELdrFlags::Wireframe))
			obj->Wireframe(true);

		// If NoZTest
		if (AllSet(obj->Flags(), ELdrFlags::NoZTest))
		{
			// Don't test against Z, and draw above all objects
			obj->m_pso.Set<EPipeState::DepthEnable>(FALSE);
			obj->m_sko.Group(ESortGroup::PostAlpha);
		}

		// If NoZWrite
		if (AllSet(obj->Flags(), ELdrFlags::NoZWrite))
		{
			// Don't write to Z and draw behind all objects
			obj->m_pso.Set<EPipeState::DepthWriteMask>(D3D12_DEPTH_WRITE_MASK_ZERO);
			obj->m_sko.Group(ESortGroup::PreOpaques);
		}

		// If flagged as screen space rendering mode
		if (obj->m_screen_space)
			obj->ScreenSpace(true);
	}

	#pragma endregion

	#pragma region Creation helpers
	
	namespace creation
	{
		// Direction:
		//  - Prefer these 'creation' objects. Many of the 'ParseXYZ' functions above could be objects in here

		// Get/Create a texture for a 2D Point sprite
		static Texture2DPtr PointStyleTexture(EPointStyle style, ParseParams& pp)
		{
			using TDrawOnIt = std::function<void(D2D1Context&, ID2D1SolidColorBrush*, ID2D1SolidColorBrush*)>;
			auto CreatePointStyleTexture = [&](ParseParams& pp, RdrId id, iv2 const& sz, char const* name, TDrawOnIt draw) -> Texture2DPtr
			{
				ResDesc tdesc = ResDesc::Tex2D(Image(sz.x, sz.y, nullptr, DXGI_FORMAT_B8G8R8A8_UNORM), 1, EUsage::RenderTarget | EUsage::SimultaneousAccess).heap_flags(D3D12_HEAP_FLAG_SHARED);
				TextureDesc desc = TextureDesc(id, tdesc).name(name);
				auto tex = pp.m_factory.CreateTexture2D(desc);

				// Get a D2D device context to draw on
				auto dc = tex->GetD2DeviceContext();

				// Create the brushes
				D3DPtr<ID2D1SolidColorBrush> fr_brush;
				D3DPtr<ID2D1SolidColorBrush> bk_brush;
				auto fr = D3DCOLORVALUE{ 1.f, 1.f, 1.f, 1.f };
				auto bk = D3DCOLORVALUE{ 0.f, 0.f, 0.f, 0.f };
				Check(dc->CreateSolidColorBrush(fr, fr_brush.address_of()));
				Check(dc->CreateSolidColorBrush(bk, bk_brush.address_of()));

				// Draw the spot
				dc->BeginDraw();
				dc->Clear(&bk);
				draw(dc, fr_brush.get(), bk_brush.get());
				Check(dc->EndDraw());
				return tex;
			};

			iv2 sz(256,256);
			switch (style)
			{
				case EPointStyle::Square:
				{
					// No texture needed for square style
					return nullptr;
				}
				case EPointStyle::Circle:
				{
					ResourceStore::Access store(pp.m_rdr);
					auto id = hash::HashArgs("PointStyleCircle", sz);
					return store.FindTexture<Texture2D>(id, [&]
					{
						auto w0 = sz.x * 0.5f;
						auto h0 = sz.y * 0.5f;
						return CreatePointStyleTexture(pp, id, sz, "PointStyleCircle", [=](auto& dc, auto fr, auto) { dc->FillEllipse({ {w0, h0}, w0, h0 }, fr); });
					});
				}
				case EPointStyle::Triangle:
				{
					ResourceStore::Access store(pp.m_rdr);
					auto id = hash::HashArgs("PointStyleTriangle", sz);
					return store.FindTexture<Texture2D>(id, [&]
					{
						D3DPtr<ID2D1PathGeometry> geom;
						D3DPtr<ID2D1GeometrySink> sink;
						Check(pp.m_rdr.D2DFactory()->CreatePathGeometry(geom.address_of()));
						Check(geom->Open(sink.address_of()));

						auto w0 = 1.0f * sz.x;
						auto h0 = 0.5f * sz.y * (float)tan(DegreesToRadians(60.0f));
						auto h1 = 0.5f * (sz.y - h0);

						sink->BeginFigure({ w0, h1 }, D2D1_FIGURE_BEGIN_FILLED);
						sink->AddLine({ 0.0f * w0, h1 });
						sink->AddLine({ 0.5f * w0, h0 + h1 });
						sink->EndFigure(D2D1_FIGURE_END_CLOSED);
						Check(sink->Close());

						return CreatePointStyleTexture(pp, id, sz, "PointStyleTriangle", [&geom](auto& dc, auto fr, auto) { dc->FillGeometry(geom.get(), fr, nullptr); });
					});
				}
				case EPointStyle::Star:
				{
					ResourceStore::Access store(pp.m_rdr);
					auto id = hash::HashArgs("PointStyleStar", sz);
					return store.FindTexture<Texture2D>(id, [&]
					{
						D3DPtr<ID2D1PathGeometry> geom;
						D3DPtr<ID2D1GeometrySink> sink;
						Check(pp.m_rdr.D2DFactory()->CreatePathGeometry(geom.address_of()));
						Check(geom->Open(sink.address_of()));

						auto w0 = 1.0f * sz.x;
						auto h0 = 1.0f * sz.y;

						sink->BeginFigure({ 0.5f * w0, 0.0f * h0 }, D2D1_FIGURE_BEGIN_FILLED);
						sink->AddLine({ 0.4f * w0, 0.4f * h0 });
						sink->AddLine({ 0.0f * w0, 0.5f * h0 });
						sink->AddLine({ 0.4f * w0, 0.6f * h0 });
						sink->AddLine({ 0.5f * w0, 1.0f * h0 });
						sink->AddLine({ 0.6f * w0, 0.6f * h0 });
						sink->AddLine({ 1.0f * w0, 0.5f * h0 });
						sink->AddLine({ 0.6f * w0, 0.4f * h0 });
						sink->EndFigure(D2D1_FIGURE_END_CLOSED);
						Check(sink->Close());

						return CreatePointStyleTexture(pp, id, sz, "PointStyleStar", [&geom](auto& dc, auto fr, auto) { dc->FillGeometry(geom.get(), fr, nullptr); });
					});
				}
				case EPointStyle::Annulus:
				{
					ResourceStore::Access store(pp.m_rdr);
					auto id = hash::HashArgs("PointStyleAnnulus", sz);
					return store.FindTexture<Texture2D>(id, [&]
					{
						auto w0 = sz.x * 0.5f;
						auto h0 = sz.y * 0.5f;
						auto w1 = sz.x * 0.4f;
						auto h1 = sz.y * 0.4f;
						return CreatePointStyleTexture(pp, id, sz, "PointStyleAnnulus", [=](auto& dc, auto fr, auto bk)
						{
							dc->SetPrimitiveBlend(D2D1_PRIMITIVE_BLEND_COPY);
							dc->FillEllipse({ {w0, h0}, w0, h0 }, fr);
							dc->FillEllipse({ {w0, h0}, w1, h1 }, bk);
						});
					});
				}
				default:
				{
					throw std::runtime_error("Unknown point style");
				}
			}
		}

		// Support for objects with a texture
		struct Textured
		{
			Texture2DPtr m_texture;
			SamplerPtr m_sampler;
			SamDesc m_def_sdesc;

			explicit Textured(SamDesc def_sdesc)
				: m_texture()
				, m_sampler()
				, m_def_sdesc(def_sdesc)
			{}
			bool ParseKeyword(IReader& reader, ParseParams& pp, EKeyword kw)
			{
				switch (kw)
				{
					case EKeyword::Texture:
					{
						TextureInfo tex_info;
						ParseTexture(reader, pp, tex_info);

						// Create the texture
						try
						{
							auto desc = TextureDesc(AutoId, {}).has_alpha(tex_info.m_has_alpha);
							m_texture = pp.m_factory.CreateTexture2D(tex_info.m_filepath, tex_info.m_tdesc);
							m_texture->m_t2s = tex_info.m_t2s;
						}
						catch (std::exception const& e)
						{
							pp.ReportError(EParseError::NotFound, reader.Loc(), std::format("Failed to create texture {}\n{}", tex_info.m_filepath.string(), e.what()));
						}

						// Create the sampler
						try
						{
							auto desc = SamplerDesc(tex_info.m_sdesc.Id(), tex_info.m_sdesc);
							m_sampler = pp.m_factory.CreateSampler(desc);
						}
						catch(std::exception const& e)
						{
							pp.ReportError(EParseError::NotFound, reader.Loc(), std::format("Failed to create sampler for texture {}\n{}", tex_info.m_filepath.string(), e.what()));
						}
						return true;
					}
					case EKeyword::Video:
					{
						auto filepath = reader.String<string32>();
						if (!filepath.empty())
						{
							//todo
							//' // Load the video texture
							//' try
							//' {
							//' 	vid = pp.m_rdr.m_tex_mgr.CreateVideoTexture(AutoId, filepath.c_str());
							//' }
							//' catch (std::exception const& e)
							//' {
							//' 	pp.ReportError(EScriptResult::ValueNotFound, reader.Location(), std::format("failed to create video {}\n{}", filepath, e.what()));
							//' }
						}
						return true;
					}
					default:
					{
						return false;
					}
				}
			}
		};

		// Support for objects with a main axis
		struct MainAxis
		{
			m4x4 m_o2w;
			AxisId m_main_axis; // The natural main axis of the object
			AxisId m_align;     // The axis we want the main axis to be aligned to

			MainAxis(pr::AxisId main_axis = pr::AxisId::PosZ, pr::AxisId align = pr::AxisId::PosZ)
				:m_o2w(m4x4::Identity())
				,m_main_axis(main_axis)
				,m_align(align)
			{}
			bool ParseKeyword(IReader& reader, ParseParams& pp, EKeyword kw)
			{
				switch (kw)
				{
					case EKeyword::AxisId:
					{
						auto align = pr::AxisId(reader.Int<int>(10));
						if (!pr::AxisId::IsValid(align))
						{
							pp.ReportError(EParseError::InvalidValue, reader.Loc(), "AxisId must be +/- 1, 2, or 3 (corresponding to the positive or negative X, Y, or Z axis)");
							return false;
						}

						m_align = align;
						m_o2w = m4x4::Transform(m_main_axis.m_axis, align, v4::Origin());
						return true;
					}
					default:
					{
						return false;
					}
				}
			}

			// True if the main axis is not equal to the desired align axis
			bool RotationNeeded() const
			{
				return m_main_axis.m_axis != m_align.m_axis;
			}

			// Returns the rotation from 'main_axis' to 'axis'
			m4x4 const& O2W() const
			{
				return m_o2w;
			}

			// Returns a pointer to a rotation from 'main_axis' to 'axis'. Returns null if identity
			m4x4 const* O2WPtr() const
			{
				return RotationNeeded() ? &m_o2w : nullptr;
			}

			// Apply main axis transform
			void BakeTransform(std::span<v4> verts)
			{
				for (auto& v : verts)
					v = m_o2w * v;
			}

			explicit operator bool() const
			{
				return m_main_axis.m_axis != m_align.m_axis;
			}
		};

		// Support baked in transforms
		struct BakeTransform
		{
			m4x4 m_o2w = m4x4::Zero();
			bool ParseKeyword(IReader& reader, ParseParams&, EKeyword kw)
			{
				switch (kw)
				{
					case EKeyword::BakeTransform:
					{
						m_o2w = m4x4::Identity();
						reader.Transform(m_o2w);
						return true;
					}
					default:
					{
						return false;
					}
				}
			}
			m4x4 const* O2WPtr() const
			{
				// Returns a pointer to a rotation from 'main_axis' to 'axis'. Returns null if identity
				return m_o2w.w.w != 0 ? &m_o2w : nullptr;
			}
			explicit operator bool() const
			{
				return m_o2w.w.w != 0;
			}
		};

		// Support for generate normals
		struct GenNorms
		{
			float m_smoothing_angle = -1.0f;
			bool ParseKeyword(IReader& reader, ParseParams&, EKeyword kw)
			{
				switch (kw)
				{
					case EKeyword::GenerateNormals:
					{
						m_smoothing_angle = reader.Real<float>();
						m_smoothing_angle = pr::DegreesToRadians(m_smoothing_angle);
						return true;
					}
					default:
					{
						return false;
					}
				}
			}
			void Generate(ParseParams& pp)
			{
				if (m_smoothing_angle < 0.0f)
					return;

				auto& verts = pp.m_cache.m_verts;
				auto& index = pp.m_cache.m_index;
				auto& normals = pp.m_cache.m_norms;
				auto& nuggets = pp.m_cache.m_nugts;

				// Can't generate normals per nugget because nuggets may share vertices.
				// Generate normals for all vertices (verts used by lines only will have zero-normals)
				normals.resize(verts.size());

				// Generate normals for the nuggets containing faces
				for (auto& nug : nuggets)
				{
					// Not face topology...
					if (nug.m_topo != ETopo::TriList)
						continue;

					// If the nugget doesn't have an 'irange' then assume one index per vert
					auto icount = isize(index);
					auto iptr = index.begin<int>();

					// The number of indices in this nugget
					if (nug.m_irange != Range::Reset())
					{
						icount = isize(nug.m_irange);
						iptr += s_cast<int>(nug.m_irange.begin());
					}

					// Not sure if this works... needs testing
					geometry::GenerateNormals(icount, iptr, m_smoothing_angle, 0,
						[&](int i)
						{
							return verts[i];
						},
						[&](int new_idx, int orig_idx, v4 norm)
						{
							if (new_idx >= isize(verts))
							{
								verts.resize(new_idx + 1, verts[orig_idx]);
								normals.resize(new_idx + 1, normals[orig_idx]);
							}
							normals[new_idx] = norm;
						},
						[&](int i0, int i1, int i2)
						{
							*iptr++ = i0;
							*iptr++ = i1;
							*iptr++ = i2;
						}
					);

					// Geometry has normals now
					nug.m_geom |= EGeom::Norm;
				}
			}
			explicit operator bool() const
			{
				return m_smoothing_angle >= 0;
			}
		};

		// Support for smoothed lines
		struct SmoothLine
		{
			bool m_enabled = false;
			bool ParseKeyword(IReader& reader, ParseParams&, EKeyword kw)
			{
				switch (kw)
				{
					case EKeyword::Smooth:
					{
						m_enabled = reader.IsSectionEnd() ? true : reader.Bool();
						return true;
					}
					default:
					{
						return false;
					}
				}
			}
			VCont InterpolateVerts(std::span<v4 const> verts)
			{
				VCont out;

				if (verts.size() < 3)
				{
					out.assign(verts.begin(), verts.end());
					return out;
				}

				// Create a cubic spline from the points and raster it
				auto spline = math::CubicSpline<float>::FromPoints(verts, math::ECurveTopology::Continuous3, math::CurveType<float>::Bezier);
				out.resize(verts.size() * 10);

				auto rastered = math::Raster(spline, spline.Time0(), spline.Time1(), std::span<v4>{out});
				out.resize(rastered.size());

				return out;
			}
			explicit operator bool() const
			{
				return m_enabled;
			}
		};

		// Support for think lines
		struct ThickLine
		{
			float m_width = 0.0f;
			bool ParseKeyword(IReader& reader, ParseParams&, EKeyword kw)
			{
				switch (kw)
				{
					case EKeyword::Width:
					{
						m_width = reader.IsSectionEnd() ? 0.0f : reader.Real<float>();
						return true;
					}
					default:
					{
						return false;
					}
				}
			}
			ShaderPtr CreateShader(Renderer& rdr, ELineStyle line_style) const
			{
				return
					line_style == ELineStyle::LineSegments ? static_cast<ShaderPtr>(Shader::Create<shaders::ThickLineListGS>(rdr, m_width)) :
					line_style == ELineStyle::LineStrip ? static_cast<ShaderPtr>(Shader::Create<shaders::ThickLineStripGS>(rdr, m_width)) :
					throw std::runtime_error(std::format("Unsupported line style: {}", ELineStyle_::ToStringA(line_style)));

			}
			void ConvertNuggets(Renderer& rdr, ELineStyle line_style, LdrObject* obj)
			{
				auto shdr = CreateShader(rdr, line_style);
				for (auto& nug : obj->m_model->m_nuggets)
				{
					nug.m_topo = line_style == ELineStyle::LineSegments ? ETopo::LineList : ETopo::LineStripAdj;
					nug.m_shdr_overlays.push_back({ shdr, ERenderStep::RenderForward });
				}
			}
			explicit operator bool() const
			{
				return m_width != 0.0f;
			}
		};

		// Support for point sprites
		struct PointSprite
		{
			EPointStyle m_style = EPointStyle::Square;
			v2 m_size = {};
			bool m_depth = false;

			void CreateNugget(LdrObject* obj, ParseParams& pp, Range vrange = Range::Reset())
			{
				// Remember to 'obj->m_model->DeleteNuggets()' first if you need too
				auto shdr = Shader::Create<shaders::PointSpriteGS>(pp.m_rdr, m_size, m_depth);
				obj->m_model->CreateNugget(pp.m_factory, NuggetDesc(ETopo::PointList, EGeom::Vert | EGeom::Colr | EGeom::Tex0)
					.use_shader_overlay(ERenderStep::RenderForward, shdr)
					.tex_diffuse(PointStyleTexture(m_style, pp))
					.flags(ENuggetFlag::RangesCanOverlap)
					.vrange(vrange)
				);
			}
			explicit operator bool() const
			{
				return m_size != v2::Zero();
			}
		};

		// Support for arrow heads
		struct ArrowHeads
		{
			EArrowType m_style = EArrowType::Line;
			Colour32 m_colour = Colour32White;
			v2 m_size = {};
			bool m_depth = false;

			void Parse(IReader& reader, ParseParams& pp)
			{
				auto section = reader.SectionScope();
				for (EKeyword kw; reader.NextKeyword(kw);)
				{
					switch (kw)
					{
						case EKeyword::Style:
						{
							m_style = reader.Enum<EArrowType>();
							break;
						}
						case EKeyword::Colour:
						{
							m_colour = reader.Int<uint32_t>(16);
							break;
						}
						case EKeyword::Size:
						{
							m_size.x = reader.IsSectionEnd() ? 0.0f : reader.Real<float>();
							m_size.y = reader.IsSectionEnd() ? m_size.x : reader.Real<float>();
							break;
						}
						case EKeyword::Depth:
						{
							m_depth = reader.IsSectionEnd() ? true : reader.Bool();
							break;
						}
						default:
						{
							pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("Keyword '{}' is not valid within *Arrow", EKeyword_::ToStringA(kw)));
							break;
						}
					}
				}
			}
			explicit operator bool() const
			{
				return m_style != EArrowType::Line;
			}
		};

		// Information on data point markers
		struct DataPoints
		{
			EPointStyle m_style = EPointStyle::Square;
			Colour32 m_colour = Colour32White;
			v2 m_size = {};
			bool m_depth = false;

			void Parse(IReader& reader, ParseParams& pp)
			{
				auto section = reader.SectionScope();
				for (EKeyword kw; reader.NextKeyword(kw);)
				{
					switch (kw)
					{
						case EKeyword::Style:
						{
							m_style = reader.Enum<EPointStyle>();
							break;
						}
						case EKeyword::Colour:
						{
							m_colour = reader.Int<uint32_t>(16);
							break;
						}
						case EKeyword::Size:
						{
							m_size.x = reader.IsSectionEnd() ? 0.0f : reader.Real<float>();
							m_size.y = reader.IsSectionEnd() ? m_size.x : reader.Real<float>();
							break;
						}
						case EKeyword::Depth:
						{
							m_depth = reader.IsSectionEnd() ? true : reader.Bool();
							break;
						}
						default:
						{
							pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("Keyword '{}' is not valid within *DataPoints", EKeyword_::ToStringA(kw)));
							break;
						}
					}
				}
			}
			explicit operator bool() const
			{
				return m_size != v2::Zero();
			}
		};

		// Support for parametric ranges
		struct Parametrics
		{
			pr::vector<int> m_index;
			pr::vector<v2> m_para;
			bool m_per_item_parametrics;

			Parametrics()
				: m_index()
				, m_para()
				, m_per_item_parametrics()
			{}
			bool ParseKeyword(IReader& reader, ParseParams&, EKeyword kw)
			{
				switch (kw)
				{
					case EKeyword::PerItemParametrics:
					{
						m_per_item_parametrics = reader.IsSectionEnd() ? true : reader.Bool();
						return true;
					}
					case EKeyword::Parametrics:
					{
						// Expect tuples of (item index, [t0, t1])
						for (; !reader.IsSectionEnd(); )
						{
							auto idx = reader.Int<int>();
							auto para = reader.Vector2f();
							Add(idx, para);
						}
						return true;
					}
					default:
					{
						return false;
					}
				}
			}
			void Add(int idx, v2 para)
			{
				m_index.push_back(idx);
				m_para.push_back(para);
			}
			void MoveEndpoints(ELineStyle line_style, std::span<v4> verts, ParseParams& pp, Location const& loc)
			{
				for (int i = 0, iend = isize(m_index); i != iend; ++i)
				{
					auto idx = m_index[i];
					auto para = m_para[i];
					switch (line_style)
					{
						case ELineStyle::LineSegments:
						{
							if (idx >= isize(verts) / 2)
							{
								pp.ReportError(EParseError::IndexOutOfRange, loc, std::format("Index {} is out of range (max={})", idx, isize(verts) / 2));
								return;
							}

							auto& p0 = verts[idx * 2 + 0];
							auto& p1 = verts[idx * 2 + 1];
							auto dir = p1 - p0;
							auto pt = p0;
							p0 = pt + para.x * dir;
							p1 = pt + para.y * dir;
							break;
						}
						case ELineStyle::LineStrip:
						{
							if (idx >= isize(verts) - 1)
							{
								pp.ReportError(EParseError::IndexOutOfRange, loc, std::format("Index {} is out of range (max={})", idx, isize(verts) - 1));
								return;
							}

							auto& p0 = verts[idx + 0];
							auto& p1 = verts[idx + 1];
							auto dir = p1 - p0;
							auto pt = p0;
							p0 = pt + para.x * dir;
							p1 = pt + para.y * dir;
							break;
						}
						default:
						{
							pp.ReportError(EParseError::InvalidValue, loc, std::format("Parametrics not support for line style {}", ELineStyle_::ToStringA(line_style)));
							return;
						}
					}
				}
			}
			explicit operator bool() const
			{
				return !m_index.empty();
			}
		};

		// Support for dashed lines
		struct DashedLines
		{
			v2 m_dash = { 1, 0 }; // x = "on" length, y = "off" length.
			bool ParseKeyword(IReader& reader, ParseParams&, EKeyword kw)
			{
				switch (kw)
				{
					case EKeyword::Dashed:
					{
						m_dash = reader.Vector2f();
						return true;
					}
					default:
					{
						return false;
					}
				}
			}
			VCont CreateSegments(ELineStyle& line_style, std::span<v4 const> verts, ParseParams& pp, Location const& loc)
			{
				VCont out;
				VCont const& in = verts;
				out.reserve(1024);

				// Convert each line segment to dashed lines
				switch (line_style)
				{
					case ELineStyle::LineSegments:
					{
						assert("Expected line segments to be vertex pairs" && (ssize(in) & 1) == 0);

						auto t = 0.0f;
						for (int64_t i = 0, iend = ssize(in); i != iend; i += 2)
						{
							auto d = in[i + 1] - in[i];
							auto len = Length(d);

							// Emit pairs of verts for each "on" section
							for (; t < len; t += m_dash.x + m_dash.y)
							{
								out.push_back(in[i] + d * Clamp(t, 0.0f, len) / len);
								out.push_back(in[i] + d * Clamp(t + m_dash.x, 0.0f, len) / len);
							}
							t -= len + m_dash.x + m_dash.y;
						}
						break;
					}
					case ELineStyle::LineStrip:
					{
						assert("Expected a line strip with at last two points" && ssize(in) >= 2);

						auto t = 0.0f;
						for (int64_t i = 1, iend = ssize(in); i != iend; ++i)
						{
							auto d = in[i] - in[i - 1];
							auto len = Length(d);

							// Emit dashes over the length of the line segment
							for (; t < len; t += m_dash.x + m_dash.y)
							{
								out.push_back(in[i - 1] + d * Clamp(t, 0.0f, len) / len);
								out.push_back(in[i - 1] + d * Clamp(t + m_dash.x, 0.0f, len) / len);
							}
							t -= len + m_dash.x + m_dash.y;
						}

						line_style = ELineStyle::LineSegments;
						break;
					}
					default:
					{
						pp.ReportError(EParseError::InvalidValue, loc, std::format("Dashed lines not support for line style {}", ELineStyle_::ToStringA(line_style)));
						return {};
					}
				}
				return out;
			}
			explicit operator bool() const
			{
				return m_dash != v2(1, 0);
			}
		};

		// Information on a key-frame animation
		// Shared playback modifiers used by both *Animation and *Montage
		struct AnimModifiers
		{
			EAnimStyle m_style = EAnimStyle::NoAnimation;
			EAnimFlags m_flags = EAnimFlags::None;
			float m_stretch = {1.0}; // aka playback speed scale
			float m_bias = {0.0};
			bool m_hide_when_not_animating = false;

			// Parse playback modifier keywords. Returns true if the keyword was handled.
			bool ParseKeyword(IReader& reader, EKeyword kw)
			{
				switch (kw)
				{
					case EKeyword::Style:
					{
						m_style = reader.Enum<EAnimStyle>();
						return true;
					}
					case EKeyword::TimeBias:
					{
						m_bias = reader.Real<float>();
						return true;
					}
					case EKeyword::Stretch:
					{
						m_stretch = reader.Real<float>();
						return true;
					}
					case EKeyword::HideWhenNotAnimating:
					{
						m_hide_when_not_animating = reader.IsSectionEnd() ? true : reader.Bool();
						return true;
					}
					case EKeyword::NoRootTranslation:
					{
						m_flags = SetBits(m_flags, EAnimFlags::NoRootTranslation, reader.IsSectionEnd() ? true : reader.Bool());
						return true;
					}
					case EKeyword::NoRootRotation:
					{
						m_flags = SetBits(m_flags, EAnimFlags::NoRootRotation, reader.IsSectionEnd() ? true : reader.Bool());
						return true;
					}
					default:
					{
						return false;
					}
				}
			}
			explicit operator bool() const
			{
				return m_style != EAnimStyle::NoAnimation;
			}
		};

		struct KeyFrameAnimInfo
		{
			AnimModifiers m_mods;
			FrameRange m_frame_range = { 0, std::numeric_limits<int>::max() };
			TimeRange m_time_range = { 0, std::numeric_limits<double>::max() }; // Seconds
			std::optional<float> m_frame_rate = {};

			void Parse(IReader& reader, ParseParams& pp)
			{
				// Set a default and indicate that an *Animation block was found
				m_mods.m_style = EAnimStyle::Once;

				auto section = reader.SectionScope();
				for (EKeyword kw; reader.NextKeyword(kw);)
				{
					switch (kw)
					{
						case EKeyword::Frame:
						{
							auto frame = reader.Int<int>();
							m_frame_range = { frame, frame };
							break;
						}
						case EKeyword::FrameRate:
						{
							m_frame_rate = reader.Real<float>();
							break;
						}
						case EKeyword::FrameRange:
						{
							auto beg = reader.Int<int>();
							auto end = reader.Int<int>();
							m_frame_range = { beg, std::max(end, beg + 1) };
							break;
						}
						case EKeyword::TimeRange:
						{
							auto t0 = reader.Real<float>();
							auto t1 = reader.Real<float>();
							m_time_range = { t0, std::max(t1, t0) };
							break;
						}
						default:
						{
							if (m_mods.ParseKeyword(reader, kw))
								break;

							if (auto const* tr = dynamic_cast<TextReader const*>(&reader))
								pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("Keyword '{}' is not valid within *Animation", tr->m_keyword.c_str()));
							else
								pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("Keyword '{}' is not valid within *Animation", EKeyword_::ToStringA(kw)));
							break;
						}
					}
				}
			}
			explicit operator bool() const
			{
				return m_mods.m_style != EAnimStyle::NoAnimation;
			}
		};

		// Description of a montage — multiple animation sources with cherry-picked frames
		struct MontageInfo
		{
			AnimModifiers m_mods;
			vector<std::filesystem::path> m_source_paths;
			vector<std::unique_ptr<std::istream>> m_source_streams;
			vector<KeyRef> m_key_refs;
			vector<float> m_durations;
			vector<m4x4> m_per_frame_o2w;
			bool m_has_data = false;

			void Parse(IReader& reader, ParseParams& pp)
			{
				m_has_data = true;
				m_mods.m_style = EAnimStyle::Once;

				auto section = reader.SectionScope();
				for (EKeyword kw; reader.NextKeyword(kw);)
				{
					switch (kw)
					{
						case EKeyword::AnimSource:
						{
							auto path = reader.String<std::filesystem::path>();
							auto stream = reader.PathResolver.OpenStream(path, IPathResolver::EFlags::Binary);
							m_source_paths.push_back(std::move(path));
							m_source_streams.push_back(std::move(stream));
							break;
						}
						case EKeyword::Frame:
						{
							auto fsection = reader.SectionScope();
							auto source_index = reader.Int<int>();
							auto frame_index = reader.Int<int>();
							auto duration = 1.0f;
							auto o2w = m4x4::Identity();

							// Check for optional *O2W within the frame block
							for (EKeyword fkw; reader.NextKeyword(fkw);)
							{
								switch (fkw)
								{
									case EKeyword::Period:
									{
										duration = reader.Real<float>();
										break;
									}
									case EKeyword::O2W:
									{
										reader.Transform(o2w);
										break;
									}
								}
							}

							m_key_refs.push_back({ source_index, frame_index });
							m_durations.push_back(duration);
							m_per_frame_o2w.push_back(o2w);
							break;
						}
						default:
						{
							if (m_mods.ParseKeyword(reader, kw))
								break;

							pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("Keyword '{}' is not valid within *Montage", EKeyword_::ToStringA(kw)));
							break;
						}
					}
				}
			}
			explicit operator bool() const
			{
				return m_has_data;
			}
		};
	}

	#pragma endregion

	#pragma region ObjectCreator

	// Base class for all object creators
	struct IObjectCreator
	{
		ParseParams& m_pp;
		VCont& m_verts;
		ICont& m_indices;
		NCont& m_normals;
		CCont& m_colours;
		TCont& m_texs;
		GCont& m_nuggets;

		IObjectCreator(ParseParams& pp)
			: m_pp(pp)
			, m_verts  (pp.m_cache.m_verts)
			, m_indices(pp.m_cache.m_index)
			, m_normals(pp.m_cache.m_norms)
			, m_colours(pp.m_cache.m_color)
			, m_texs   (pp.m_cache.m_texts)
			, m_nuggets(pp.m_cache.m_nugts)
		{}
		virtual ~IObjectCreator() = default;

		// Create an LdrObject from 'reader'
		virtual LdrObjectPtr Parse(IReader& reader)
		{
			// Notes:
			//  - Not using an output iterator style callback because model
			//    instancing relies on the map from object to model.
			auto start_location = reader.Loc();
			auto section = reader.SectionScope();

			// Read the object attributes: name, colour, instance
			auto obj = LdrObjectPtr(new LdrObject(m_pp.m_type, m_pp.m_parent, m_pp.m_context_id), true);

			// Read the description of the model
			for (EKeyword kw; !m_pp.m_cancel && reader.NextKeyword(kw);)
			{
				// Let the object creator have the first go with the keyword
				if (ParseKeyword(reader, kw))
					continue;

				// Is the keyword a common object property?
				if (ParseProperties(reader, m_pp, kw, obj.get()))
					continue;

				// Recursively parse child objects
				ParseParams pp(m_pp, obj->m_child, obj.get(), this);
				if (ParseLdrObject((ELdrObject)kw, reader, pp))
					continue;

				// Unknown token
				m_pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), "Unknown keyword");
				continue;
			}

			// Complete the model for 'm_obj'
			CreateModel(obj.get(), start_location);

			return obj;
		}
		virtual bool ParseKeyword(IReader&, EKeyword)
		{
			return false;
		}
		virtual void CreateModel(LdrObject*, Location const&)
		{
			// Don't return a model from this method, because the overrides are also configuring 'obj'
			// It's doesn't make sense to separate this from the returned model.
		}
	};

	#pragma endregion

	#pragma region Sprite Objects

	// ELdrObject::Point
	template <> struct ObjectCreator<ELdrObject::Point> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::PointSprite m_sprite;
		bool m_per_item_colour;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_sprite()
			, m_per_item_colour()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						m_verts.push_back(reader.Vector3f().w1());
						if (m_per_item_colour)
							m_colours.push_back(reader.Int<uint32_t>(16));
					}
					return true;
				}
				case EKeyword::PerItemColour:
				{
					m_per_item_colour = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				case EKeyword::Size:
				{
					m_sprite.m_size.x = reader.Real<float>();
					m_sprite.m_size.y = reader.IsSectionEnd() ? m_sprite.m_size.x : reader.Real<float>();
					return true;
				}
				case EKeyword::Style:
				{
					m_sprite.m_style = reader.Enum<EPointStyle>();
					return true;
				}
				case EKeyword::Depth:
				{
					m_sprite.m_depth = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				default:
				{
					return
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// No points = no model
			if (m_verts.empty())
				return;

			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours);
			obj->m_model = ModelGenerator::Points(m_pp.m_factory, m_verts, &opts);
			obj->m_model->m_name = obj->TypeAndName();

			// Use Point sprites
			if (m_sprite)
			{
				obj->m_model->DeleteNuggets();
				m_sprite.CreateNugget(obj, m_pp);
			}
		}
	};

	#pragma endregion

	#pragma region Line Objects

	// ELdrObject::Line
	template <> struct ObjectCreator<ELdrObject::Line> :IObjectCreator
	{
		// Notes:
		//  - Each *Data {} block is one segment.
		//  - Each segment captures the current line style, arrow type, etc. So segments can be different types.
		//  - Segments are used for strip-cuts or disjoint splines.
		//  ///- All segments must be the same line style because they are turned into one model
		//  - Arrow type applies to each segment
		//  - Smooth and Splines are orthogonal, Splines are how the data points are given, smooth is used to sub-sample lines.
		//  - One colour per line element
		struct Segment
		{
			ELineStyle m_style = ELineStyle::LineSegments; // The type of line this is
			creation::ThickLine m_thick;                   // Thick line support for the segment
			creation::DashedLines m_dashed;                // Dashed line support for the segment
			creation::Parametrics m_parametric;            // Parametric values to apply to the segment elements
			creation::ArrowHeads m_arrow_heads;            // The arrow heads to add to the segment
			creation::DataPoints m_data_points;            // Point sprites for the verts of the line
			creation::SmoothLine m_smooth;                 // Smoothing support for the segment
			int m_vcount = 0;                              // Number of verts added due to this line segment
			int m_ccount = 0;                              // Number of colours added due to this line segment
			int m_count = 0;                               // Line elements count
		};
		vector<Segment, 1> m_segments; // Segments that make up the line
		vector<Vert, 2> m_arrow_heads; // Buffer of arrow head instances
		vector<Vert, 2> m_data_points; // Buffer of data point locations
		Segment m_current;             // The current state read from the script
		bool m_per_item_parametrics;   // True if each element is expected to have parametric values
		bool m_per_item_colour;        // True if each element is expected to have a colour

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_segments()
			, m_arrow_heads()
			, m_data_points()
			, m_current(ELineStyle::LineSegments)
			, m_per_item_parametrics()
			, m_per_item_colour()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Style:
				{
					m_current.m_style = reader.Enum<ELineStyle>();
					return true;
				}
				case EKeyword::PerItemColour:
				{
					m_per_item_colour = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				case EKeyword::PerItemParametrics:
				{
					m_per_item_parametrics  = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				case EKeyword::Arrow:
				{
					m_current.m_arrow_heads = creation::ArrowHeads{};
					m_current.m_arrow_heads.Parse(reader, m_pp);
					return true;
				}
				case EKeyword::DataPoints:
				{
					m_current.m_data_points = creation::DataPoints{};
					m_current.m_data_points.Parse(reader, m_pp);
					return true;
				}
				case EKeyword::Width:
				{
					m_current.m_thick.m_width = reader.IsSectionEnd() ? 0.0f : reader.Real<float>();
					return true;
				}
				case EKeyword::Dashed:
				{
					m_current.m_dashed = creation::DashedLines{};
					m_current.m_dashed.m_dash = reader.Vector2f();
					return true;
				}
				case EKeyword::Smooth:
				{
					m_current.m_smooth.m_enabled = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				case EKeyword::Data:
				{
					ReadSegmentData(reader);
					return true;
				}
				default:
				{
					return
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void ReadSegmentData(IReader& reader)
		{
			Segment segment = m_current;
			switch (segment.m_style)
			{
				// Read pairs of points, each pair is a line segment
				case ELineStyle::LineSegments:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						m_verts.push_back(reader.Vector3f().w1());
						m_verts.push_back(reader.Vector3f().w1());
						segment.m_vcount += 2;
						if (m_per_item_colour)
						{
							Colour32 col = reader.Int<uint32_t>(16);
							m_colours.push_back(col);
							m_colours.push_back(col);
							segment.m_ccount += 2;
						}
						if (m_per_item_parametrics)
						{
							auto para = reader.Vector2f();
							segment.m_parametric.Add(segment.m_count, para);
						}
						++segment.m_count;
					}
					break;
				}

				// Read single points, each point is a continuation of a line strip. Use separate *Data sections to create strip cuts.
				case ELineStyle::LineStrip:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						m_verts.push_back(reader.Vector3f().w1());
						segment.m_vcount += 1;
						if (m_per_item_colour)
						{
							m_colours.push_back(reader.Int<uint32_t>(16));
							segment.m_ccount += 1;
						}
						if (m_per_item_parametrics)
						{
							auto para = reader.Vector2f();
							segment.m_parametric.Add(segment.m_count, para);
						}
						++segment.m_count;
					}
					break;
				}

				// Read pairs of points, each pair is a (pt, pt + dir) line segment
				case ELineStyle::Direction:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						auto p = reader.Vector3f().w1();
						auto d = reader.Vector3f().w0();
						m_verts.push_back(p);
						m_verts.push_back(p + d);
						segment.m_vcount += 2;
						if (m_per_item_colour)
						{
							Colour32 col = reader.Int<uint32_t>(16);
							m_colours.push_back(col);
							m_colours.push_back(col);
							segment.m_ccount += 2;
						}
						if (m_per_item_parametrics)
						{
							auto para = reader.Vector2f();
							segment.m_parametric.Add(segment.m_count, para);
						}
						++segment.m_count;
					}
					segment.m_style = ELineStyle::LineSegments;
					break;
				}

				// Read control points in sets of 4
				case ELineStyle::BezierSpline:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						auto p0 = reader.Vector3f().w1();
						auto p1 = reader.Vector3f().w1();
						auto p2 = reader.Vector3f().w1();
						auto p3 = reader.Vector3f().w1();
						(void)p0, p1, p2, p3;
						// Todo: Fill 'm_verts' with the rendered spline
						// m_verts.push_back(p0);
						// m_verts.push_back(p1);
						// m_verts.push_back(p2);
						// m_verts.push_back(p3);
						// if (m_per_item_colour)
						// {
						// 	Colour32 col = reader.Int<uint32_t>(16);
						// 	m_colours.push_back(col);
						// 	m_colours.push_back(col);
						// }
						// if (m_per_item_parametrics)
						// {
						// 	auto para = reader.Vector2f();
						// 	segment.m_parametric.Add(isize(m_verts) / 2 - 1, para);
						// }
					}
					
					break;
				}

				case ELineStyle::HermiteSpline:
				{
					break;
				}
				case ELineStyle::BSplineSpline:
				{
					break;
				}
				case ELineStyle::CatmullRom:
				{
					break;
				}
				default:
				{
					m_pp.ReportError(EParseError::InvalidValue, reader.Loc(), "Unknown line style");
					break;
				}
			}

			// Only add segments containing data
			if (segment.m_vcount != 0)
				m_segments.push_back(segment);
		}
		std::tuple<int, int, int> ProcessSegments(Location const& loc)
		{
			// If a segments needs to change it's verts, it should remove them from 'm_verts'
			// and insert the new verts at 'm_verts.begin() + vcount'

			int vcount = 0;
			int ccount = 0;
			int ncount = 0;

			// Process each segment
			for (auto& segment : m_segments)
			{
				// Copy the data points to a separate buffer because later steps can change them.
				if (segment.m_data_points)
				{
					auto verts = m_verts.span(vcount, segment.m_vcount);
					auto segment_idx = s_cast<int>(&segment - m_segments.data());
					for (auto const& v : verts)
					{
						m_data_points.push_back(Vert{
							.m_vert = v,
							.m_diff = pr::Colour(segment.m_data_points.m_colour),
							.m_norm = v4{segment.m_data_points.m_size, 0, 0},
							.m_tex0 = {},
							.m_idx0 = {segment_idx, 0},
						});
					}
				}

				// Clip lines to parametric values
				if (segment.m_parametric)
				{
					auto verts = m_verts.span(vcount, segment.m_vcount);
					segment.m_parametric.MoveEndpoints(segment.m_style, verts, m_pp, loc);
				}

				// Smooth the points
				if (segment.m_smooth && segment.m_style == ELineStyle::LineStrip)
				{
					// Convert the points of this segment into a Bezier cubic spline
					CubicSpline<float> spline = CubicSpline<float>::FromPoints(m_verts.span(vcount, segment.m_vcount), ECurveTopology::Continuous3, CurveType<float>::Bezier);

					// Raster the spline into a new buffer
					VCont spline_point_buf(50);
					auto spline_points = Raster(spline, spline.Time0(), spline.Time1(), std::span<v4>{spline_point_buf});

					// Replace the verts with the smoothed verts
					m_verts.erase(m_verts.begin() + vcount, m_verts.begin() + vcount + segment.m_vcount);
					m_verts.insert(m_verts.begin() + vcount, begin(spline_points), end(spline_points));
					segment.m_vcount = isize(spline_points);
				}

				// If the line has arrow heads, add them to 'arrow_heads'
				if (segment.m_arrow_heads)
				{
					auto verts = m_verts.span(vcount, segment.m_vcount);
					auto colrs = m_colours.span(ccount, segment.m_ccount);
					auto segment_idx = s_cast<int>(&segment - m_segments.data());
					auto size = segment.m_arrow_heads.m_size =
						segment.m_arrow_heads.m_size != v2::Zero() ? segment.m_arrow_heads.m_size :
						segment.m_thick.m_width != 0 ? v2{ segment.m_thick.m_width * 2 } :
						v2{ 8.0f };

					switch (segment.m_style)
					{
						case ELineStyle::LineSegments:
						{
							// Add arrow heads for each line segment
							for (int i = 0; i != segment.m_vcount; i += 2)
							{
								auto elem = verts.subspan(i, 2);
								if (AllSet(segment.m_arrow_heads.m_style, EArrowType::Fwd))
								{
									m_arrow_heads.push_back(Vert{
										.m_vert = elem[1],
										.m_diff = pr::Colour(colrs.empty() ? Colour32White : colrs.last<1>()[0]),
										.m_norm = Normalise(elem[1] - elem[0]),
										.m_tex0 = size,
										.m_idx0 = {segment_idx, 0},
									});
								}
								if (AllSet(segment.m_arrow_heads.m_style, EArrowType::Back))
								{
									m_arrow_heads.push_back(Vert{
										.m_vert = elem[0],
										.m_diff = pr::Colour(colrs.empty() ? Colour32White : colrs.first<1>()[0]),
										.m_norm = Normalise(elem[0] - elem[1]),
										.m_tex0 = size,
										.m_idx0 = {segment_idx, 0},
									});
								}
							}
							break;
						}
						case ELineStyle::LineStrip:
						{
							if (AllSet(segment.m_arrow_heads.m_style, EArrowType::Fwd))
							{
								auto head = verts.last<2>();
								m_arrow_heads.push_back(Vert{
									.m_vert = head[1],
									.m_diff = pr::Colour(colrs.empty() ? Colour32White : colrs.last<1>()[0]),
									.m_norm = Normalise(head[1] - head[0]),
									.m_tex0 = size,
									.m_idx0 = {segment_idx, 0},
								});
							}
							if (AllSet(segment.m_arrow_heads.m_style, EArrowType::Back))
							{
								auto tail = verts.first<2>();
								m_arrow_heads.push_back(Vert{
									.m_vert = tail[0],
									.m_diff = pr::Colour(colrs.empty() ? Colour32White : colrs.first<1>()[0]),
									.m_norm = Normalise(tail[0] - tail[1]),
									.m_tex0 = size,
									.m_idx0 = {segment_idx, 0},
								});
							}
							break;
						}
						default:
						{
							throw std::runtime_error(std::format("Unsupported line style: {}", ELineStyle_::ToStringA(segment.m_style)));
						}
					}
				}

				// Convert lines to dashed lines
				if (segment.m_dashed)
				{
					auto verts = m_verts.span(vcount, segment.m_vcount);
					auto new_verts = segment.m_dashed.CreateSegments(segment.m_style, verts, m_pp, loc);

					// Replace the verts with the dashed verts
					m_verts.erase(begin(m_verts) + vcount, begin(m_verts) + vcount + segment.m_vcount);
					m_verts.insert(begin(m_verts) + vcount, begin(new_verts), end(new_verts));
					segment.m_vcount = isize(new_verts);
				}

				// The thick line strip shader uses LineAdj which requires an extra first and last vert
				if (segment.m_thick && segment.m_style == ELineStyle::LineStrip)
				{
					if (segment.m_vcount != 0)
					{
						auto verts = m_verts.span(vcount, segment.m_vcount);
						auto tail = verts.first<1>()[0];
						auto head = verts.last<1>()[0];
						m_verts.insert(begin(m_verts) + vcount, tail);
						m_verts.insert(begin(m_verts) + vcount + segment.m_vcount, head);
						segment.m_vcount += 2;
					}
					if (segment.m_ccount != 0)
					{
						auto colrs = m_colours.span(ccount, segment.m_ccount);
						auto tail = colrs.first<1>()[0];
						auto head = colrs.last<1>()[0];
						m_colours.insert(begin(m_colours) + ccount, tail);
						m_colours.insert(begin(m_colours) + ccount + segment.m_ccount, head);
						segment.m_ccount += 2;
					}
				}

				vcount += segment.m_vcount;
				ccount += segment.m_ccount;
				ncount += 1;
			}
			for (auto _ : group_by(m_arrow_heads, [](Vert const& v) { return v.m_idx0.x; }))
				++ncount;
			for (auto _ : group_by(m_data_points, [](Vert const& v) { return v.m_idx0.x; }))
				++ncount;

			return { vcount, ccount, ncount };
		}
		void CreateModel(LdrObject* obj, Location const& loc) override
		{
			// No segments = no model
			if (m_segments.empty())
				return;

			auto [vcount, ccount, ncount] = ProcessSegments(loc);

			ModelGenerator::Cache<Vert> cache{ 0, 0, 0, isizeof<uint16_t>() };
			cache.m_vcont.reserve(vcount);
			cache.m_ncont.reserve(ncount);

			vcount = 0;
			ccount = 0;

			constexpr auto cc = [](Colour32 c, bool& has_alpha) -> pr::Colour { has_alpha |= HasAlpha(c); return pr::Colour(c); };
			constexpr auto bb = [](v4 v, BBox& bbox) { Grow(bbox, v); return v; };

			// Combine all segments into one model
			for (auto& segment : m_segments)
			{
				// The spans associated with 'segment'
				auto verts = m_verts.span(vcount, segment.m_vcount);
				auto colours = m_colours.cspan(ccount, segment.m_ccount);

				// Append to the cache
				auto vofs = cache.m_vcont.size();
				cache.m_vcont.resize(vofs + segment.m_vcount, {});
				auto vptr = cache.m_vcont.data() + vofs;

				// Colours
				auto col = segment.m_smooth
					? CreateLerpRepeater(colours, segment.m_vcount, Colour32White)
					: CreateRepeater(colours, segment.m_vcount, Colour32White);

				auto has_alpha = false;

				// Append to the model buffer
				for (auto const& v : verts)
					SetPC(*vptr++, bb(v, cache.m_bbox), cc(*col++, has_alpha));

				// Add a nugget for this line segment
				auto topo =
					segment.m_style == ELineStyle::LineSegments ? ETopo::LineList :
					segment.m_style == ELineStyle::LineStrip ? ETopo::LineStrip :
					throw std::runtime_error(std::format("Unsupported line style: {}", ELineStyle_::ToStringA(segment.m_style)));

				NuggetDesc nugget = NuggetDesc(topo, EGeom::Vert | EGeom::Colr)
					.vrange(vcount, vcount + segment.m_vcount)
					.alpha_geom(has_alpha);

				// Use the thick line shader
				if (segment.m_thick)
				{
					auto shdr = segment.m_thick.CreateShader(m_pp.m_rdr, segment.m_style);
					nugget.use_shader_overlay(ERenderStep::RenderForward, shdr);
					if (segment.m_style == ELineStyle::LineStrip)
						nugget.topo(ETopo::LineStripAdj);
				}

				cache.m_ncont.push_back(nugget);
				vcount += segment.m_vcount;
				ccount += segment.m_ccount;
			}

			// Add geometry and a nugget for the arrow heads
			if (!m_arrow_heads.empty())
			{
				cache.m_vcont.insert(end(cache.m_vcont), begin(m_arrow_heads), end(m_arrow_heads));

				// Arrow heads for difference chunks can be different styles, depth
				for (auto [b, e] : group_by(m_arrow_heads, [](Vert const& v) { return v.m_idx0.x; }))
				{
					auto const& arrow_heads = m_segments[b->m_idx0.x].m_arrow_heads;
					auto beg = static_cast<int>(b - m_arrow_heads.data());
					auto end = static_cast<int>(e - m_arrow_heads.data());
					auto has_alpha = std::ranges::any_of(m_arrow_heads, [](Vert const& ah) { return HasAlpha(ah.m_diff); });
					auto size = arrow_heads.m_size;
					auto depth = arrow_heads.m_depth;

					// Add a nugget for this style
					auto arw_shdr = Shader::Create<shaders::ArrowHeadGS>(m_pp.m_rdr, size, depth);
					cache.m_ncont.push_back(NuggetDesc(ETopo::PointList, EGeom::Vert | EGeom::Colr)
						.use_shader_overlay(ERenderStep::RenderForward, arw_shdr)
						.vrange(vcount + beg, vcount + end)
						.flags(ENuggetFlag::GeometryHasAlpha, has_alpha)
					);
				}

				vcount += isize(m_arrow_heads);
			}

			// Add geometry and a nugget for the data points
			if (!m_data_points.empty())
			{
				cache.m_vcont.insert(end(cache.m_vcont), begin(m_data_points), end(m_data_points));

				// Data points for difference chunks can be different styles
				for (auto [b, e] : group_by(m_data_points, [](Vert const& v) { return v.m_idx0.x; }))
				{
					auto const& data_points = m_segments[b->m_idx0.x].m_data_points;
					auto beg = static_cast<int>(b - m_data_points.data());
					auto end = static_cast<int>(e - m_data_points.data());
					auto style = data_points.m_style;
					auto size = data_points.m_size;
					auto depth = data_points.m_depth;
					auto has_alpha = std::ranges::any_of(m_data_points, [](Vert const& x) { return HasAlpha(x.m_diff); });

					// Add a nugget for this style
					auto pt_shdr = Shader::Create<shaders::PointSpriteGS>(m_pp.m_rdr, size, depth);
					cache.m_ncont.push_back(NuggetDesc(ETopo::PointList, EGeom::Vert | EGeom::Colr | EGeom::Tex0)
						.use_shader_overlay(ERenderStep::RenderForward, pt_shdr)
						.tex_diffuse(creation::PointStyleTexture(style, m_pp))
						.vrange(vcount + beg, vcount + end)
						.flags(ENuggetFlag::GeometryHasAlpha, has_alpha)
					);
				}

				vcount += isize(m_data_points);
			}

			// Create the line model
			obj->m_model = ModelGenerator::Create(m_pp.m_factory, cache);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::LineBox
	template <> struct ObjectCreator<ELdrObject::LineBox> :IObjectCreator
	{
		creation::DashedLines m_dashed;
		creation::ThickLine m_thick;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_dashed()
			, m_thick()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override 
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					auto dim = v4(reader.Real<float>()).w0();
					if (!reader.IsSectionEnd()) dim.y = reader.Real<float>();
					if (!reader.IsSectionEnd()) dim.z = reader.Real<float>();
					dim *= 0.5f;

					m_verts.push_back(v4(-dim.x, -dim.y, -dim.z, 1.0f));
					m_verts.push_back(v4(+dim.x, -dim.y, -dim.z, 1.0f));
					m_verts.push_back(v4(+dim.x, +dim.y, -dim.z, 1.0f));
					m_verts.push_back(v4(-dim.x, +dim.y, -dim.z, 1.0f));
					m_verts.push_back(v4(-dim.x, -dim.y, +dim.z, 1.0f));
					m_verts.push_back(v4(+dim.x, -dim.y, +dim.z, 1.0f));
					m_verts.push_back(v4(+dim.x, +dim.y, +dim.z, 1.0f));
					m_verts.push_back(v4(-dim.x, +dim.y, +dim.z, 1.0f));

					uint16_t const idx[] = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };
					m_indices.resize(0, sizeof(uint16_t));
					m_indices.append<uint16_t>(idx);
					return true;
				}
				default:
				{
					return
						m_thick.ParseKeyword(reader, m_pp, kw) ||
						m_dashed.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const& loc) override
		{
			// No points = no model
			if (m_verts.empty())
				return;

			auto line_style = ELineStyle::LineSegments;

			// Convert lines to dashed lines
			if (m_dashed)
				m_dashed.CreateSegments(line_style, m_verts, m_pp, loc);

			m_nuggets.push_back(NuggetDesc(ETopo::LineList, EGeom::Vert|EGeom::Colr));

			// Create the model
			auto cdata = MeshCreationData()
				.verts  (m_verts)
				.indices(m_indices)
				.colours(m_colours)
				.nuggets(m_nuggets);
			obj->m_model = ModelGenerator::Mesh(m_pp.m_factory, cdata);
			obj->m_model->m_name = obj->TypeAndName();

			// Use thick lines
			if (m_thick)
				m_thick.ConvertNuggets(m_pp.m_rdr, line_style, obj);
		}
	};

	// ELdrObject::Grid
	template <> struct ObjectCreator<ELdrObject::Grid> :IObjectCreator
	{
		creation::DashedLines m_dashed;
		creation::MainAxis m_axis;
		creation::ThickLine m_thick;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_dashed()
			, m_axis()
			, m_thick()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					auto dim = reader.Vector2f();
					auto div = reader.Vector2f();

					auto step = dim / div;
					for (float i = -dim.x / 2; i <= dim.x / 2; i += step.x)
					{
						m_verts.push_back(v4(i, -dim.y / 2, 0, 1));
						m_verts.push_back(v4(i, dim.y / 2, 0, 1));
					}
					for (float i = -dim.y / 2; i <= dim.y / 2; i += step.y)
					{
						m_verts.push_back(v4(-dim.x / 2, i, 0, 1));
						m_verts.push_back(v4(dim.x / 2, i, 0, 1));
					}
					return true;
				}
				default:
				{
					return
						m_thick.ParseKeyword(reader, m_pp, kw) ||
						m_dashed.ParseKeyword(reader, m_pp, kw) ||
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const& loc) override
		{
			// Validate
			if (m_verts.empty())
				return;

			auto line_style = ELineStyle::LineSegments;

			// Convert lines to dashed lines
			if (m_dashed)
				m_dashed.CreateSegments(line_style, m_verts, m_pp, loc);

			// Apply main axis transform
			if (m_axis)
				m_axis.BakeTransform(m_verts);

			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours);
			obj->m_model = ModelGenerator::Lines(m_pp.m_factory, isize(m_verts) / 2, m_verts, &opts);
			obj->m_model->m_name = obj->TypeAndName();

			// Use thick lines
			if (m_thick)
				m_thick.ConvertNuggets(m_pp.m_rdr, line_style, obj);
		}
	};

	// ELdrObject::CoordFrame
	template <> struct ObjectCreator<ELdrObject::CoordFrame> :IObjectCreator
	{
		vector<m4x4> m_basis;
		creation::ThickLine m_thick;
		float m_scale;
		bool m_lh;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_basis()
			, m_thick()
			, m_scale(1.0f)
			, m_lh(false)
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					auto o2w = m4x4::Identity();
					reader.Transform(o2w);
					m_basis.push_back(o2w);
					return true;
				}
				case EKeyword::Scale:
				{
					m_scale = reader.Real<float>();
					return true;
				}
				case EKeyword::LeftHanded:
				{
					m_lh = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				default:
				{
					return
						m_thick.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			if (m_basis.empty())
				m_basis.push_back(m4x4::Identity());
			
			for (auto const& o2w : m_basis)
			{
				// Scale doesn't use the *o2w scale because that is recursive
				auto origin = o2w * v4::Origin();
				auto xaxis = o2w * v4{ m_scale, 0, 0, 1 };
				auto yaxis = o2w * v4{ 0, m_scale, 0, 1 };
				auto zaxis = o2w * v4{ 0, 0, (m_lh ? -1 : +1) * m_scale, 1 };
				m_verts.push_back(origin);
				m_verts.push_back(xaxis);
				m_verts.push_back(origin);
				m_verts.push_back(yaxis);
				m_verts.push_back(origin);
				m_verts.push_back(zaxis);
				m_colours.push_back(Colour32Red);
				m_colours.push_back(Colour32Red);
				m_colours.push_back(Colour32Green);
				m_colours.push_back(Colour32Green);
				m_colours.push_back(Colour32Blue);
				m_colours.push_back(Colour32Blue);
			}

			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours);
			obj->m_model = ModelGenerator::Lines(m_pp.m_factory, int(m_verts.size() / 2), m_verts, &opts);
			obj->m_model->m_name = obj->TypeAndName();

			// Use thick lines
			if (m_thick)
				m_thick.ConvertNuggets(m_pp.m_rdr, ELineStyle::LineSegments, obj);
		}
	};

	// ELdrObject::Chart
	template <> struct ObjectCreator<ELdrObject::Chart> :IObjectCreator
	{
		// Notes:
		//  - 'm_data' may be a fully populated NxM table, or a jagged array.
		//  - If jagged, then 'm_index' will be non-empty and 'm_dim' will be the bounding dimensions of the table.
		//  - If non-jagged, then 'm_index' will be empty, and 'm_dim' represents the dimensions of the table.

		using Index = std::vector<int>;         // The index for accessing rows in the data table
		using Table = std::vector<double>;      // The source data loaded into memory

		Table m_data;        // A 2D table of data (row major, i.e. rows are contiguous)
		Index m_index;       // The offset into 'm_data' for the start if each row (if jagged) else empty.
		iv2   m_dim;         // Table dimensions (columns, rows) or bounds of the table dimensions.

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_data()
			, m_index()
			, m_dim()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Dim:
				{
					m_dim.x = reader.Int<int>(10);
					m_dim.y = reader.IsSectionEnd() ? 0 : reader.Int<int>(10);
					return true;
				}
				case EKeyword::Data:
				{
					// Read data till the end of the section
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						auto value = reader.Real<double>();
						m_data.push_back(value);
					}
					
					// Infer the data dimensions if not given
					if (m_dim.x == 0) m_dim.x = 1;
					if (m_dim.y == 0) m_dim.y = (isize(m_data) + m_dim.x - 1) / m_dim.x;

					// Immediate data is not jagged.
					m_index.resize(0);
					return true;
				}
				case EKeyword::FilePath:
				{
					// Source is a file containing data
					auto filepath = reader.String<std::filesystem::path>();
					std::ifstream file(filepath);
					ParseDataStream(file);
					return true;
				}
				default:
				{
					return IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		template <typename Stream> void ParseDataStream(Stream& stream)
		{
			using namespace pr::csv;
			using namespace pr::str;

			m_data.clear();
			m_index.clear();
			m_dim = iv2::Zero();

			// Read CSV data up to the section close
			m_data.reserve(100); Loc loc;
			for (Row row; Read(stream, row, loc); row.resize(0))
			{
				// Trim trailing empty values and empty rows
				if (row.size() == 1 && Trim(row[0], IsWhiteSpace<wchar_t>, false, true).empty())
					row.pop_back();
				if (!row.empty() && Trim(row.back(), IsWhiteSpace<wchar_t>, false, true).empty())
					row.pop_back();
				if (row.empty())
					continue;

				// Convert the row to values. Stop at the first element that fails to parse as a value
				auto row_count = 0;
				for (auto& item : row)
				{
					double value;
					if (!ExtractRealC(value, item.c_str())) break;
					m_data.push_back(value);
					++row_count;
				}

				// Skip rows with no data
				if (row_count == 0)
					continue;

				// Assume the table is non-jagged until we find a different number of items in a row
				if (!m_index.empty()) // Table is jagged already
				{
					m_index.push_back(m_index.back() + row_count);
					m_dim.x = max(m_dim.x, row_count);
					m_dim.y++;
				}
				else if (m_dim.x == row_count) // Table is not jagged (yet), row length is the same
				{
					m_dim.y++;
				}
				else if (m_dim.x == 0) // Table is empty, set the row length
				{
					m_dim.x = row_count;
					m_dim.y = 1;
				}
				else // Row length has changed, convert to jagged
				{
					m_index.reserve(m_dim.y + 1);

					// Fill 'm_index' with rows of length 'm_dim.x'
					m_index.push_back(0);
					for (int i = 0; i != m_dim.y; ++i)
						m_index.push_back(m_index.back() + m_dim.x);

					m_index.push_back(m_index.back() + row_count);
					m_dim.x = max(m_dim.x, row_count);
					m_dim.y++;
				}
			}
			
			// If this is jagged data, then 'm_index' should have 'm_dim.y + 1' items
			// with the last value == to the number of elements in the data table.
			assert(m_index.empty() || (int)m_index.size() == m_dim.y + 1);
			assert(m_index.empty() || m_index.back() == (int)m_data.size());
		}
		void CreateModel(LdrObject*, Location const&) override
		{
			// The chart does not contain a model. Instead, nested 'Series'
			// objects form the models, based on the data in this object.
		}
	};

	// ELdrObject::Series
	template <> struct ObjectCreator<ELdrObject::Series> :IObjectCreator
	{
		struct DataIter
		{
			enum class EType
			{
				None = 0,
				Row         = 1 << 0,
				Column      = 1 << 1,
				Index       = 1 << 2,
				Value       = 1 << 3,
				IndexRow    = Row | Index,
				DataRow     = Row | Value,
				IndexColumn = Column | Index,
				DataColumn  = Column | Value,
				_flags_enum = 0,
			};

			eval::IdentHash m_arghash; // The hash of the argument name
			EType m_type;              // Iterator type
			iv2 m_idx0;                // The virtual coordinate of the iterator position
			iv2 m_step;                // The amount to advance 'idx' by with each iteration
			iv2 m_max;                 // Where iteration stops.

			DataIter(std::string_view name, iv2 max)
				: m_arghash(eval::hashname(name))
				, m_type(EType::None)
				, m_idx0(0, 0)
				, m_step(0, 0)
				, m_max(max)
			{
				// Convert a name like "C32" or "R21" into an iterator into 'm_data'
				// Format is '(C|R)(#|<number>)'. E.g. C#, R#, C0, C23, R23, R2
				if (name.size() < 2)
					throw std::runtime_error("Data iterator name is empty");

				switch (std::toupper(name[0]))
				{
					case 'C': m_type |= EType::Column; break;
					case 'R': m_type |= EType::Row; break;
					default: throw std::runtime_error("Data iterator must start with 'C' or 'R'");
				}

				m_type |= std::toupper(name[1]) == 'I' ? EType::Index : EType::Value;

				int idx = 0;
				if (AllSet(m_type, EType::Value) && !str::ExtractIntC(idx, 10, script::StringSrc(name.substr(1))))
					throw std::runtime_error(std::format("Series data references should contain an index: '{}'", name));

				m_idx0 = AllSet(m_type, EType::Column) ? iv2{ idx, 0 } : iv2{ 0, idx };
				m_step = AllSet(m_type, EType::Column) ? iv2{ 0, 1 } : iv2{ 1, 0 };
			}
		};

		// Default colours to use for each series
		static inline Colour32 const colours[] =
		{
			0xFF70ad47, 0xFF4472c4, 0xFFed7d31,
			0xFF264478, 0xFF9e480e, 0xFFffc000,
			0xFF9e480e, 0xFF636363,

			//0xFF0000FF, 0xFF00FF00, 0xFFFF0000,
			//0xFF0000A0, 0xFF00A000, 0xFFA00000,
			//0xFF000080, 0xFF008000, 0xFF800000,
			//0xFF00FFFF, 0xFFFFFF00, 0xFFFF00FF,
			//0xFF00A0A0, 0xFFA0A000, 0xFFA000A0,
			//0xFF008080, 0xFF808000, 0xFF800080,
		};

		ObjectCreator<ELdrObject::Chart> const* m_chart;
		eval::Expression m_xaxis;
		eval::Expression m_yaxis;
		pr::vector<DataIter> m_xiter;
		pr::vector<DataIter> m_yiter;
		creation::ThickLine m_thick;
		creation::DashedLines m_dashed;
		creation::SmoothLine m_smooth;
		creation::DataPoints m_data_points;
		
		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_chart()
			, m_xaxis()
			, m_yaxis()
			, m_xiter()
			, m_yiter()
			, m_thick()
			, m_dashed()
			, m_smooth()
			, m_data_points()
		{}
		LdrObjectPtr Parse(IReader& reader) override
		{
			// Find the ancestor chart creator
			for (auto const* parent = m_pp.m_parent_creator; parent; parent = parent->m_pp.m_parent_creator)
			{
				if (parent->m_pp.m_type != ELdrObject::Chart) continue;
				m_chart = static_cast<ObjectCreator<ELdrObject::Chart> const*>(parent);
				break;
			}
			if (!m_chart)
			{
				m_pp.ReportError(EParseError::NotFound, {}, "Series objects must be children of a Chart object");
				return nullptr; // Not possible to carry on without a chart
			}
			return IObjectCreator::Parse(reader);
		}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::XAxis:
				{
					m_xaxis = eval::Compile(reader.String<string32>());
					for (auto& name : m_xaxis.m_arg_names)
						m_xiter.push_back(DataIter{name, m_chart->m_dim});
					
					return true;
				}
				case EKeyword::YAxis:
				{
					m_yaxis = eval::Compile(reader.String<string32>());
					for (auto& name : m_yaxis.m_arg_names)
						m_yiter.push_back(DataIter{name, m_chart->m_dim});

					return true;
				}
				case EKeyword::DataPoints:
				{
					m_data_points.Parse(reader, m_pp);
					return true;
				}
				default:
				{
					return
						m_thick.ParseKeyword(reader, m_pp, kw) ||
						m_dashed.ParseKeyword(reader, m_pp, kw) ||
						m_smooth.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const& loc) override
		{
			// Determine the index of this series within the chart
			int child_index = 0;
			for (auto& child : m_pp.m_parent->m_child)
				child_index += int(child->m_type == ELdrObject::Series);

			// Generate a name if none given
			if (!AllSet(m_pp.m_flags, EFlags::ExplicitName))
				obj->m_name = std::format("Series {}", child_index);

			// Assign a colour if none given
			if (!AllSet(m_pp.m_flags, EFlags::ExplicitColour))
				obj->m_base_colour = colours[child_index % _countof(colours)];

			auto& verts = m_pp.m_cache.m_verts;
			verts.clear();

			// Merge the args from both expressions
			auto args = eval::ArgSet{};
			args.add(m_xaxis.m_args);
			args.add(m_yaxis.m_args);

			// Iterate over the data points
			for (int i = 0;;++i)
			{
				// Initialise the expression arguments for 'i'
				bool in_range = false;
				for (auto& iter : m_xiter)
					args.set(iter.m_arghash, GetValue(iter, i, in_range));
				for (auto& iter : m_yiter)
					args.set(iter.m_arghash, GetValue(iter, i, in_range));
				if (!in_range)
					break;

				// Evaluate the data point at 'i'
				auto x = m_xaxis(args);
				auto y = m_yaxis(args);
				verts.push_back(v4{static_cast<float>(x.db()), static_cast<float>(y.db()), 0, 1});
			}
				
			// Create a plot from the points
			if (verts.empty())
				return;

			auto line_style = ELineStyle::LineStrip;

			// If we're showing data points, save the verts that represent actual data
			VCont data_verts;
			if (m_data_points)
				data_verts = verts;

			// Convert the points into a spline if smooth is specified
			if (m_smooth)
				m_smooth.InterpolateVerts(verts);

			// Convert lines to dashed lines
			if (m_dashed)
				m_dashed.CreateSegments(line_style, verts, m_pp, loc);

			// The thick line strip shader uses LineAdj which requires an extra first and last vert
			if (line_style == ELineStyle::LineStrip && m_thick.m_width != 0.0f)
			{
				verts.insert(std::begin(verts), verts.front());
				verts.insert(std::end(verts), verts.back());
			}

			auto opts = ModelGenerator::CreateOptions().colours({&obj->m_base_colour, 1});
			obj->m_model = ModelGenerator::LineStrip(m_pp.m_factory, isize(verts) - 1, verts, &opts);
			obj->m_model->m_name = obj->TypeAndName();

			// Use thick lines
			if (m_thick)
				m_thick.ConvertNuggets(m_pp.m_rdr, line_style, obj);

			// Add data points as a child object
			if (m_data_points)
			{
				ParseParams pp(m_pp, obj->m_child, obj, this);
				
				LdrObjectPtr data_points(new LdrObject(ELdrObject::Point, obj, obj->m_context_id), true);
				data_points->m_name = "DataPoints";

				opts = ModelGenerator::CreateOptions().colours({&data_points->m_base_colour, 1});
				data_points->m_model = ModelGenerator::Points(m_pp.m_factory, data_verts, &opts);
				data_points->m_model->m_name = data_points->TypeAndName();
				data_points->m_model->DeleteNuggets();

				// Add a nugget for the data points
				auto shdr = Shader::Create<shaders::PointSpriteGS>(m_pp.m_rdr, m_data_points.m_size, m_data_points.m_depth);
				data_points->m_model->CreateNugget(pp.m_factory, NuggetDesc(ETopo::PointList, EGeom::Vert | EGeom::Colr | EGeom::Tex0)
					.use_shader_overlay(ERenderStep::RenderForward, shdr)
					.tex_diffuse(creation::PointStyleTexture(m_data_points.m_style, pp))
					.flags(ENuggetFlag::RangesCanOverlap)
					.tint(m_data_points.m_colour)
				);

				// Add the object to the parent
				ApplyObjectState(data_points.get());
				obj->m_child.push_back(data_points);
			}
		}
		double GetValue(DataIter const& iter, int i, bool& in_range) const
		{
			// Points outside the data set are considered zeros
			auto idx = iter.m_idx0 + i * iter.m_step;
			auto is_within = idx.x >= 0 && idx.x < iter.m_max.x && idx.y >= 0 && idx.y < iter.m_max.y;
			if (!is_within)
				return 0.0;

			// 'iter' still points to valid data
			in_range |= true;

			// If the iterator is just the row or column index
			switch (iter.m_type)
			{
				case DataIter::EType::IndexColumn:
				{
					return idx.y;
				}
				case DataIter::EType::IndexRow:
				{
					return idx.x;
				}
				case DataIter::EType::DataColumn:
				case DataIter::EType::DataRow:
				{
					// Not jagged
					if (m_chart->m_index.empty())
						return m_chart->m_data[idx.y * m_chart->m_dim.x + idx.x];

					// If 'm_data' is a jagged array, get the number of values on the current row
					auto num_on_row = m_chart->m_index[idx.y + 1] - m_chart->m_index[idx.y];
					return idx.x < num_on_row ? m_chart->m_data[m_chart->m_index[idx.y] + idx.x] : 0.0;
				}
				default:
				{
					throw std::runtime_error("Unknown iterator type");
				}
			}
		}
	};

	#pragma endregion

	#pragma region Shapes2d

	// ELdrObject::Circle
	template <> struct ObjectCreator<ELdrObject::Circle> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::MainAxis m_axis;
		v2 m_dim;
		int m_facets;
		bool m_solid;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_axis()
			, m_dim()
			, m_facets(40)
			, m_solid()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					m_dim.x = reader.Real<float>();
					m_dim.y = reader.IsSectionEnd() ? m_dim.x : reader.Real<float>();
					if (Abs(m_dim) != m_dim)
					{
						m_pp.ReportError(EParseError::InvalidValue, reader.Loc(), "Circle dimensions contain a negative value");
						m_dim = Abs(m_dim);
					}

					return true;
				}
				case EKeyword::Solid:
				{
					m_solid = reader.IsSectionEnd() ? true : reader.Bool();;
					return true;
				}
				case EKeyword::Facets:
				{
					m_facets = reader.Int<int>(10);
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Ellipse(m_pp.m_factory, m_dim.x, m_dim.y, m_solid, m_facets, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// LdrObject::Pie
	template <> struct ObjectCreator<ELdrObject::Pie> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::MainAxis m_axis;
		v2 m_scale;
		v2 m_ang;
		v2 m_rad;
		int m_facets;
		bool m_solid;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_axis()
			, m_scale(v2::One())
			, m_ang()
			, m_rad()
			, m_facets(40)
			, m_solid()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					m_ang = reader.Vector2f();
					m_rad = reader.Vector2f();
					m_ang.x = DegreesToRadians(m_ang.x);
					m_ang.y = DegreesToRadians(m_ang.y);
					return true;
				}
				case EKeyword::Solid:
				{
					m_solid = reader.IsSectionEnd() ? true : reader.Bool();;
					return true;
				}
				case EKeyword::Scale:
				{
					m_scale = reader.Vector2f();
					return true;
				}
				case EKeyword::Facets:
				{
					m_facets = reader.Int<int>(10);
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Pie(m_pp.m_factory, m_scale.x, m_scale.y, m_ang.x, m_ang.y, m_rad.x, m_rad.y, m_solid, m_facets, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Rect
	template <> struct ObjectCreator<ELdrObject::Rect> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::MainAxis m_axis;
		v2 m_dim;
		float m_corner_radius;
		int m_facets;
		bool m_solid;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_axis()
			, m_dim()
			, m_corner_radius()
			, m_facets(40)
			, m_solid()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					m_dim.x = reader.Real<float>();
					m_dim.y = reader.IsSectionEnd() ? m_dim.x : reader.Real<float>();
					m_dim *= 0.5f;

					if (Abs(m_dim) != m_dim)
					{
						m_pp.ReportError(EParseError::InvalidValue, reader.Loc(), "Rect dimensions contain a negative value");
						m_dim = Abs(m_dim);
					}
					return true;
				}
				case EKeyword::CornerRadius:
				{
					m_corner_radius = reader.Real<float>();
					return true;
				}
				case EKeyword::Facets:
				{
					m_facets = reader.Int<int>(10);
					m_facets *= 4;
					return true;
				}
				case EKeyword::Solid:
				{
					m_solid = reader.IsSectionEnd() ? true : reader.Bool();;
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::RoundedRectangle(m_pp.m_factory, m_dim.x, m_dim.y, m_corner_radius, m_solid, m_facets, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Polygon
	template <> struct ObjectCreator<ELdrObject::Polygon> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::MainAxis m_axis;
		pr::vector<v2> m_poly;
		bool m_per_item_colour;
		bool m_solid;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_axis()
			, m_poly()
			, m_per_item_colour()
			, m_solid()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					for (; !reader.IsSectionEnd(); )
					{
						m_poly.push_back(reader.Vector2f());
						if (m_per_item_colour)
							m_colours.push_back(reader.Int<uint32_t>(16));
					}
					return true;
				}
				case EKeyword::Solid:
				{
					m_solid = reader.IsSectionEnd() ? true : reader.Bool();;
					return true;
				}
				case EKeyword::PerItemColour:
				{
					m_per_item_colour = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Check the polygon winding order
			if (geometry::PolygonArea(m_poly) < 0)
			{
				std::ranges::reverse(m_poly);
				std::ranges::reverse(m_colours);
			}

			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Polygon(m_pp.m_factory, m_poly, m_solid, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	#pragma endregion

	#pragma region Quads

	// ELdrObject::Triangle
	template <> struct ObjectCreator<ELdrObject::Triangle> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::MainAxis m_axis;
		bool m_per_item_colour;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_axis()
			, m_per_item_colour()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);

						v4 pt[3] = {};
						Colour32 col[3] = {};
						for (int i = 0; i != 3; ++i)
						{
							pt[i] = reader.Vector3f().w1();
							if (m_per_item_colour)
								col[i] = reader.Int<uint32_t>(16);
						}

						m_verts.push_back(pt[0]);
						m_verts.push_back(pt[1]);
						m_verts.push_back(pt[2]);
						m_verts.push_back(pt[2]); // create a degenerate
						if (m_per_item_colour)
						{
							m_colours.push_back(col[0]);
							m_colours.push_back(col[1]);
							m_colours.push_back(col[2]);
							m_colours.push_back(col[2]);
						}
					}
					return true;
				}
				case EKeyword::PerItemColour:
				{
					m_per_item_colour = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Validate
			if (m_verts.empty() || (isize(m_verts) % 4) != 0)
				return;

			// Apply the axis id rotation
			if (m_axis)
				m_axis.BakeTransform(m_verts);

			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Quad(m_pp.m_factory, isize(m_verts) / 4, m_verts, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Quad
	template <> struct ObjectCreator<ELdrObject::Quad> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::MainAxis m_axis;
		bool m_per_item_colour;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_axis()
			, m_per_item_colour()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);

						v4 pt[4] = {};
						Colour32 col[4] = {};
						for (int i = 0; i != 4; ++i)
						{
							pt[i] = reader.Vector3f().w1();
							if (m_per_item_colour)
								col[i] = reader.Int<uint32_t>(16);
						}

						m_verts.push_back(pt[0]);
						m_verts.push_back(pt[1]);
						m_verts.push_back(pt[2]);
						m_verts.push_back(pt[3]);
						if (m_per_item_colour)
						{
							m_colours.push_back(col[0]);
							m_colours.push_back(col[1]);
							m_colours.push_back(col[2]);
							m_colours.push_back(col[3]);
						}
					}
					return true;
				}
				case EKeyword::PerItemColour:
				{
					m_per_item_colour = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Validate
			if (m_verts.empty() || (isize(m_verts) % 4) != 0)
				return;

			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Quad(m_pp.m_factory, isize(m_verts) / 4, m_verts, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Plane
	template <> struct ObjectCreator<ELdrObject::Plane> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::MainAxis m_axis_id;
		v2 m_dim;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicWrap())
			, m_dim()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					m_dim.x = reader.Real<float>();
					m_dim.y = reader.Real<float>();
					m_dim *= 0.5f;
					return true;
				}
				default:
				{
					return
						m_axis_id.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			if (m_dim == v2::Zero())
				return;

			v4 verts[4] = {
				v4{-m_dim.x, -m_dim.y, 0, 1},
				v4{+m_dim.x, -m_dim.y, 0, 1},
				v4{-m_dim.x, +m_dim.y, 0, 1},
				v4{+m_dim.x, +m_dim.y, 0, 1},
			};

			// Create the model
			auto opts = ModelGenerator::CreateOptions().bake(m_axis_id.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Quad(m_pp.m_factory, 1, verts, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Ribbon
	template <> struct ObjectCreator<ELdrObject::Ribbon> :IObjectCreator
	{
		// Notes:
		//  - Defaulting to 'clamp' because ribbons use the first row of the 2D texture and extrude it.
		//    This doesn't work with 'wrap' or 'border' modes.

		creation::Textured m_tex;
		creation::MainAxis m_axis;
		creation::SmoothLine m_smooth;
		float m_width;
		bool m_per_item_colour;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_axis()
			, m_smooth()
			, m_width(10.0f)
			, m_per_item_colour()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						m_verts.push_back(reader.Vector3f().w1());
						if (m_per_item_colour)
							m_colours.push_back(reader.Int<uint32_t>(16));
					}
					return true;
				}
				case EKeyword::Width:
				{
					m_width = reader.IsSectionEnd() ? 0.0f : reader.Real<float>();
					return true;
				}
				case EKeyword::PerItemColour:
				{
					m_per_item_colour = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				default:
				{
					return
						m_smooth.ParseKeyword(reader, m_pp, kw) ||
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Validate
			if (isize(m_verts) < 2)
				return;

			// Smooth the points
			if (m_smooth)
				m_smooth.InterpolateVerts(m_verts);

			v4 normal = m_axis.m_align.m_axis;
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::QuadStrip(m_pp.m_factory, isize(m_verts) - 1, m_verts, m_width, { &normal, 1 }, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	#pragma endregion

	#pragma region Shapes3d

	// ELdrObject::Box
	template <> struct ObjectCreator<ELdrObject::Box> :IObjectCreator
	{
		creation::Textured m_tex;
		v4 m_dim;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_dim()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					m_dim.x = reader.IsSectionEnd() ? 1.0f : reader.Real<float>();
					m_dim.y = reader.IsSectionEnd() ? m_dim.x : reader.Real<float>();
					m_dim.z = reader.IsSectionEnd() ? m_dim.y : reader.Real<float>();
					m_dim *= 0.5f;
					return true;
				}
				default:
				{
					return
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Box(m_pp.m_factory, m_dim, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::BoxList
	template <> struct ObjectCreator<ELdrObject::BoxList> :IObjectCreator
	{
		creation::Textured m_tex;
		vector<BBox, 4> m_boxes;
		bool m_per_item_colour;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_boxes()
			, m_per_item_colour()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::PerItemColour:
				{
					m_per_item_colour = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				case EKeyword::Data:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						auto dim = reader.Vector3f().w0();
						auto xyz = reader.Vector3f().w1();
						m_boxes.push_back(BBox(xyz, Abs(dim) * 0.5f));
						if (m_per_item_colour)
							m_colours.push_back(reader.Int<uint32_t>(16));
					}
					return true;
				}
				default:
				{
					return
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Validate
			if (m_boxes.empty())
				return;

			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::BoxList(m_pp.m_factory, m_boxes, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::FrustumWH
	template <> struct ObjectCreator<ELdrObject::FrustumWH> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::MainAxis m_axis;
		v4 m_pt[8];
		float m_width, m_height;
		float m_near, m_far;
		float m_view_plane;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_axis()
			, m_pt()
			, m_width(1.0f)
			, m_height(1.0f)
			, m_near(0.0f)
			, m_far(1.0f)
			, m_view_plane(0.0f)
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					m_width = reader.Real<float>();
					m_height = reader.Real<float>();
					m_near = reader.Real<float>();
					m_far = reader.Real<float>();
					return true;
				}
				case EKeyword::ViewPlaneZ:
				{
					m_view_plane = reader.Real<float>();
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Construct pointing down -z, then rotate the points based on axis id.
			// Do this because frustums are commonly used for camera views and cameras point down -z.
			// If the near plane is given, but no view plane, assume the near plane is the view plane.
			float n = m_near, f = m_far;
			auto vp = m_view_plane != 0 ? m_view_plane : m_near != 0 ? m_near : 1.0f;
			auto w = 0.5f * m_width  / vp;
			auto h = 0.5f * m_height / vp;

			m_pt[0] = v4(-f*w, -f*h, -f, 1.0f);
			m_pt[1] = v4(+f*w, -f*h, -f, 1.0f);
			m_pt[2] = v4(-f*w, +f*h, -f, 1.0f);
			m_pt[3] = v4(+f*w, +f*h, -f, 1.0f);
			m_pt[4] = v4(-n*w, -n*h, -n, 1.0f);
			m_pt[5] = v4(+n*w, -n*h, -n, 1.0f);
			m_pt[6] = v4(-n*w, +n*h, -n, 1.0f);
			m_pt[7] = v4(+n*w, +n*h, -n, 1.0f);

			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Boxes(m_pp.m_factory, 1, m_pt, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::FrustumFA
	template <> struct ObjectCreator<ELdrObject::FrustumFA> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::MainAxis m_axis;
		v4 m_pt[8];
		float m_fovY, m_aspect;
		float m_near, m_far;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_axis()
			, m_pt()
			, m_fovY(constants<float>::tau_by_8)
			, m_aspect(1.0f)
			, m_near(0.0f)
			, m_far(1.0f)
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					m_fovY = reader.Real<float>();
					m_aspect = reader.Real<float>();
					m_near = reader.Real<float>();
					m_far = reader.Real<float>();
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Construct pointing down -z, then rotate the points based on axis id.
			// Do this because frustums are commonly used for camera views and cameras point down -z.
			float h = Tan(DegreesToRadians(m_fovY * 0.5f));
			float w = m_aspect * h;
			float n = m_near, f = m_far;
			
			m_pt[0] = v4(-f*w, -f*h, -f, 1.0f);
			m_pt[1] = v4(+f*w, -f*h, -f, 1.0f);
			m_pt[2] = v4(-f*w, +f*h, -f, 1.0f);
			m_pt[3] = v4(+f*w, +f*h, -f, 1.0f);
			m_pt[4] = v4(-n*w, -n*h, -n, 1.0f);
			m_pt[5] = v4(+n*w, -n*h, -n, 1.0f);
			m_pt[6] = v4(-n*w, +n*h, -n, 1.0f);
			m_pt[7] = v4(+n*w, +n*h, -n, 1.0f);

			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Boxes(m_pp.m_factory, 1, m_pt, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Sphere
	template <> struct ObjectCreator<ELdrObject::Sphere> :IObjectCreator
	{
		creation::Textured m_tex;
		v4 m_dim;
		int m_facets;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicWrap())
			, m_dim()
			, m_facets(3)
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					m_dim.x = reader.Real<float>();
					m_dim.y = reader.IsSectionEnd() ? m_dim.x : reader.Real<float>();
					m_dim.z = reader.IsSectionEnd() ? m_dim.y : reader.Real<float>();
					return true;
				}
				case EKeyword::Facets:
				{
					m_facets = reader.Int<int>(10);
					return true;
				}
				default:
				{
					return
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Geosphere(m_pp.m_factory, m_dim, m_facets, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Cylinder
	template <> struct ObjectCreator<ELdrObject::Cylinder> :IObjectCreator
	{
		creation::MainAxis m_axis;
		creation::Textured m_tex;
		v4 m_dim; // x,y = radius, z = height
		v2 m_scale;
		int m_layers;
		int m_wedges;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_axis()
			, m_tex(SamDesc::AnisotropicClamp())
			, m_dim()
			, m_scale(v2::One())
			, m_layers(1)
			, m_wedges(20)
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					m_dim.z = reader.Real<float>();
					m_dim.x = reader.Real<float>();
					m_dim.y = reader.IsSectionEnd() ? m_dim.x : reader.Real<float>();
					return true;
				}
				case EKeyword::Facets:
				{
					auto facets = reader.Vector2i(10);
					m_layers = facets.x;
					m_wedges = facets.y;
					return true;
				}
				case EKeyword::Scale:
				{
					m_scale = reader.Vector2f();
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Cylinder(m_pp.m_factory, m_dim.x, m_dim.y, m_dim.z, m_scale.x, m_scale.y, m_wedges, m_layers, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Cone
	template <> struct ObjectCreator<ELdrObject::Cone> :IObjectCreator
	{
		creation::MainAxis m_axis;
		creation::Textured m_tex;
		v4 m_dim; // x,y = radius, z = height
		v2 m_scale;
		int m_layers;
		int m_wedges;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_axis()
			, m_tex(SamDesc::AnisotropicClamp())
			, m_dim()
			, m_scale(v2::One())
			, m_layers(1)
			, m_wedges(20)
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					auto a = reader.Real<float>();
					auto h0 = reader.Real<float>();
					auto h1 = reader.Real<float>();

					a = DegreesToRadians(a);

					m_dim.z = h1 - h0;
					m_dim.x = h0 * Tan(a);
					m_dim.y = h1 * Tan(a);
					return true;
				}
				case EKeyword::Facets:
				{
					auto facets = reader.Vector2i(10);
					m_layers = facets.x;
					m_wedges = facets.y;
					return true;
				}
				case EKeyword::Scale:
				{
					m_scale = reader.Vector2f();
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_axis.O2WPtr()).tex_diffuse(m_tex.m_texture, m_tex.m_sampler);
			obj->m_model = ModelGenerator::Cylinder(m_pp.m_factory, m_dim.x, m_dim.y, m_dim.z, m_scale.x, m_scale.y, m_wedges, m_layers, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Tube
	template <> struct ObjectCreator<ELdrObject::Tube> :IObjectCreator
	{
		enum class ECSType
		{
			Invalid,
			Round,
			Square,
			Polygon,
		};

		pr::vector<v2> m_cs;           // 2d cross section
		v2 m_radius;                   // X,Y radii for implicit cross sections
		ECSType m_cs_type;             // Cross section type
		int m_cs_facets;               // The number of divisions for Round cross sections
		bool m_cs_smooth;              // True if outward normals for the tube are smoothed
		bool m_per_item_colour;        // Colour per vertex
		bool m_closed;                 // True if the tube end caps should be filled in
		creation::SmoothLine m_smooth; // True if the extrusion path is smooth

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_cs()
			, m_radius(1.0f)
			, m_cs_type(ECSType::Square)
			, m_cs_facets(20)
			, m_cs_smooth(false)
			, m_per_item_colour()
			, m_closed(false)
			, m_smooth()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					// Parse the extrusion path
					for (; !reader.IsSectionEnd();)
					{
						auto pt = reader.Vector3f().w1();
						auto col = m_per_item_colour ? Colour32(reader.Int<uint32_t>(16)) : Colour32White;

						// Ignore degenerates
						if (m_verts.empty() || !FEql(m_verts.back(), pt))
						{
							m_verts.push_back(pt);
							if (m_per_item_colour)
								m_colours.push_back(col);
						}
					}
					return true;
				}
				case EKeyword::CrossSection:
				{
					ParseCrossSection(reader);
					return true;
				}
				case EKeyword::Closed:
				{
					m_closed = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				case EKeyword::PerItemColour:
				{
					m_per_item_colour = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				default:
				{
					return
						m_smooth.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void ParseCrossSection(IReader& reader)
		{
			auto section = reader.SectionScope();
			for (EKeyword kw; reader.NextKeyword(kw);)
			{
				switch (kw)
				{
					case EKeyword::Round:
					{
						// Elliptical cross section, expect 1 or 2 radii to follow
						m_radius.x = reader.Real<float>();
						m_radius.y = reader.IsSectionEnd() ? m_radius.x : reader.Real<float>();
						m_cs_smooth = true;
						m_cs_type = ECSType::Round;
						break;
					}
					case EKeyword::Square:
					{
						// Square cross section, expect 1 or 2 radii to follow
						m_radius.x = reader.Real<float>();
						m_radius.y = reader.IsSectionEnd() ? m_radius.x : reader.Real<float>();
						m_cs_smooth = false;
						m_cs_type = ECSType::Square;
						break;
					}
					case EKeyword::Polygon:
					{
						// Create the cross section, expect X,Y pairs
						for (; !reader.IsSectionEnd();)
							m_cs.push_back(reader.Vector2f());

						m_cs_type = ECSType::Polygon;
						break;
					}
					case EKeyword::Facets:
					{
						m_cs_facets = reader.Int<int>(10);
						break;
					}
					case EKeyword::Smooth:
					{
						m_cs_smooth = true;
						break;
					}
					default:
					{
						m_pp.ReportError(EParseError::InvalidValue, reader.Loc(), std::format("Cross Section keyword {} is not supported", EKeyword_::ToStringA(kw)));
						break;
					}
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const& loc) override
		{
			// If no cross section or extrusion path is given
			if (isize(m_verts) < 2)
				return;

			// Create the cross section for implicit profiles
			switch (m_cs_type)
			{
				case ECSType::Round:
				{
					for (auto i = 0; i != m_cs_facets; ++i)
						m_cs.push_back(v2(
							m_radius.x * Cos(float(constants<double>::tau) * i / m_cs_facets),
							m_radius.y * Sin(float(constants<double>::tau) * i / m_cs_facets)));
					break;
				}
				case ECSType::Square:
				{
					// Create the cross section
					m_cs.push_back(v2(-m_radius.x, -m_radius.y));
					m_cs.push_back(v2(+m_radius.x, -m_radius.y));
					m_cs.push_back(v2(+m_radius.x, +m_radius.y));
					m_cs.push_back(v2(-m_radius.x, +m_radius.y));
					break;
				}
				case ECSType::Polygon:
				{
					if (m_cs.empty())
					{
						m_pp.ReportError(EParseError::DataMissing, loc, std::format("Tube object '{}' description incomplete", obj->TypeAndName()));
						return;
					}

					// Ensure a positive area
					if (geometry::PolygonArea(m_cs) < 0)
						std::reverse(std::begin(m_cs), std::end(m_cs));

					break;
				}
				default:
				{
					m_pp.ReportError(EParseError::DataMissing, loc, std::format("Tube object '{}' description incomplete. No style given.", obj->TypeAndName()));
					return;
				}
			}

			// Smooth the tube centre line
			if (m_smooth)
				m_smooth.InterpolateVerts(m_verts);

			// Create the model
			auto opts = ModelGenerator::CreateOptions().colours(m_colours);
			obj->m_model = ModelGenerator::Extrude(m_pp.m_factory, m_cs, m_verts, m_closed, m_cs_smooth, &opts);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Mesh
	template <> struct ObjectCreator<ELdrObject::Mesh> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::GenNorms m_gen_norms;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_gen_norms(-1.0f)
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Verts:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						m_verts.push_back(reader.Vector3f().w1());
					}
					return true;
				}
				case EKeyword::Normals:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						m_normals.push_back(reader.Vector3f().w0());
					}
					return true;
				}
				case EKeyword::Colours:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						m_colours.push_back(pr::Colour(reader.Int<uint32_t>(16)));
					}
					return true;
				}
				case EKeyword::TexCoords:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						m_texs.push_back(reader.Vector2f());
					}
					return true;
				}
				case EKeyword::Lines:
				case EKeyword::LineList:
				case EKeyword::LineStrip:
				{
					auto is_strip = kw == EKeyword::LineStrip;
					auto nug = NuggetDesc(is_strip ? ETopo::LineStrip : ETopo::LineList, EGeom::Vert |
						(!m_colours.empty() ? EGeom::Colr : EGeom::None))
						.vrange(Range::Reset())
						.irange(Range(m_indices.size(), m_indices.size()))
						.tex_diffuse(m_tex.m_texture)
						.sam_diffuse(m_tex.m_sampler);

					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						auto idx = reader.Int<uint16_t>(10);
						m_indices.push_back(idx);
						nug.m_vrange.grow(idx);
						++nug.m_irange.m_end;
					}

					m_nuggets.push_back(nug);
					return true;
				}
				case EKeyword::Faces:
				case EKeyword::TriList:
				case EKeyword::TriStrip:
				{
					auto is_strip = kw == EKeyword::TriStrip;
					auto nug = NuggetDesc(is_strip ? ETopo::TriStrip : ETopo::TriList, EGeom::Vert |
						(!m_normals.empty() ? EGeom::Norm : EGeom::None) |
						(!m_colours.empty() ? EGeom::Colr : EGeom::None) |
						(!m_texs.empty() ? EGeom::Tex0 : EGeom::None))
						.vrange(Range::Reset())
						.irange(Range(m_indices.size(), m_indices.size()))
						.tex_diffuse(m_tex.m_texture)
						.sam_diffuse(m_tex.m_sampler);

					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						auto idx = reader.Int<uint16_t>(10);
						m_indices.push_back(idx);
						nug.m_vrange.grow(idx);
						++nug.m_irange.m_end;
					}

					m_nuggets.push_back(nug);
					return true;
				}
				case EKeyword::Tetra:
				{
					auto nug = NuggetDesc(ETopo::TriList, EGeom::Vert |
						(!m_normals.empty() ? EGeom::Norm : EGeom::None) |
						(!m_colours.empty() ? EGeom::Colr : EGeom::None) |
						(!m_texs.empty() ? EGeom::Tex0 : EGeom::None))
						.vrange(Range::Reset())
						.irange(Range(m_indices.size(), m_indices.size()))
						.tex_diffuse(m_tex.m_texture)
						.sam_diffuse(m_tex.m_sampler);

					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);

						uint16_t const idx[] = {
							reader.Int<uint16_t>(10),
							reader.Int<uint16_t>(10),
							reader.Int<uint16_t>(10),
							reader.Int<uint16_t>(10),
						};
						m_indices.push_back(idx[0]);
						m_indices.push_back(idx[1]);
						m_indices.push_back(idx[2]);
						m_indices.push_back(idx[0]);
						m_indices.push_back(idx[2]);
						m_indices.push_back(idx[3]);
						m_indices.push_back(idx[0]);
						m_indices.push_back(idx[3]);
						m_indices.push_back(idx[1]);
						m_indices.push_back(idx[3]);
						m_indices.push_back(idx[2]);
						m_indices.push_back(idx[1]);

						nug.m_vrange.grow(idx[0]);
						nug.m_vrange.grow(idx[1]);
						nug.m_vrange.grow(idx[2]);
						nug.m_vrange.grow(idx[3]);
						nug.m_irange.m_end += 12;
					}

					m_nuggets.push_back(nug);
					return true;
				}
				default:
				{
					return
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						m_gen_norms.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const& loc) override
		{
			// Validate
			if (m_indices.empty() || m_verts.empty())
			{
				m_pp.ReportError(EParseError::DataMissing, loc, "Mesh object description incomplete");
				return;
			}
			if (!m_colours.empty() && m_colours.size() != m_verts.size())
			{
				m_pp.ReportError(EParseError::DataMissing, loc, std::format("Mesh objects with colours require one colour per vertex. {} required, {} given.", isize(m_verts), isize(m_colours)));
				return;
			}
			if (!m_normals.empty() && m_normals.size() != m_verts.size())
			{
				m_pp.ReportError(EParseError::DataMissing, loc, std::format("Mesh objects with normals require one normal per vertex. {} required, {} given.", isize(m_verts), isize(m_normals)));
				return;
			}
			if (!m_texs.empty() && m_texs.size() != m_verts.size())
			{
				m_pp.ReportError(EParseError::DataMissing, loc, std::format("Mesh objects with texture coordinates require one coordinate per vertex. {} required, {} given.", isize(m_verts), isize(m_normals)));
				return;
			}
			for (auto& nug : m_nuggets)
			{
				// Check the index range is valid
				if (nug.m_vrange.m_beg < 0 || nug.m_vrange.m_end > isize(m_verts))
				{
					m_pp.ReportError(EParseError::InvalidValue, loc, std::format("Mesh object with face, line, or tetra section contains indices out of range (section index: {}).", int(&nug - &m_nuggets[0])));
					return;
				}

				// Set the nugget's 'has_alpha' value now we know the indices are valid
				if (!m_colours.empty())
				{
					for (auto i : nug.m_irange.enumerate())
					{
						auto ii = static_cast<uint64_t>(m_indices[s_cast<size_t>(i)]);
						if (!HasAlpha(m_colours[s_cast<size_t>(ii)]))
							continue;

						nug.m_nflags = SetBits(nug.m_nflags, ENuggetFlag::GeometryHasAlpha, true);
						break;
					}
				}
			}

			// Generate normals if needed
			m_gen_norms.Generate(m_pp);

			// Create the model
			auto cdata = MeshCreationData()
				.verts(m_verts)
				.indices(m_indices)
				.nuggets(m_nuggets)
				.colours(m_colours)
				.normals(m_normals)
				.tex(m_texs);
			obj->m_model = ModelGenerator::Mesh(m_pp.m_factory, cdata);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::ConvexHull
	template <> struct ObjectCreator<ELdrObject::ConvexHull> :IObjectCreator
	{
		creation::Textured m_tex;
		creation::GenNorms m_gen_norms;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_tex(SamDesc::AnisotropicClamp())
			, m_gen_norms(0.0f)
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Verts:
				{
					for (int r = 1; !reader.IsSectionEnd() && !m_pp.m_cancel; ++r)
					{
						m_pp.ReportProgress(reader, r);
						m_verts.push_back(reader.Vector3f().w1());
					}
					return true;
				}
				default:
				{
					return
						m_tex.ParseKeyword(reader, m_pp, kw) ||
						m_gen_norms.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Validate
			if (m_verts.size() < 2)
				return;

			auto vcount = isize(m_verts);
			auto icount = 6 * (isize(m_verts) - 2);
			auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

			// Allocate space for the face indices
			m_indices.resize(icount, idx_stride);

			// Find the convex hull
			size_t num_verts = 0, num_faces = 0;
			if (idx_stride == sizeof(uint32_t))
				hull::ConvexHull(m_verts, m_indices.span<uint32_t>(), num_verts, num_faces);
			else
				hull::ConvexHull(m_verts, m_indices.span<uint16_t>(), num_verts, num_faces);

			m_verts.resize(num_verts);
			m_indices.resize(3 * num_faces, idx_stride);

			// Create a nugget for the hull
			m_nuggets.push_back(NuggetDesc(ETopo::TriList, EGeom::Vert).tex_diffuse(m_tex.m_texture).sam_diffuse(m_tex.m_sampler));

			// Generate normals if needed
			m_gen_norms.Generate(m_pp);

			// Create the model
			auto cdata = MeshCreationData()
				.verts(m_verts)
				.indices(m_indices)
				.nuggets(m_nuggets)
				.colours(m_colours)
				.normals(m_normals)
				.tex(m_texs);
			obj->m_model = ModelGenerator::Mesh(m_pp.m_factory, cdata);
			obj->m_model->m_name = obj->TypeAndName();
		}
	};

	// ELdrObject::Model
	template <> struct ObjectCreator<ELdrObject::Model> :IObjectCreator
	{
		struct ModelOut : ModelGenerator::IModelOut
		{
			ObjectCreator<ELdrObject::Model>& m_self;
			geometry::ESceneParts m_parts;
			LdrObject* m_obj;

			ModelOut(ObjectCreator<ELdrObject::Model>& self, geometry::ESceneParts parts, LdrObject* obj)
				: m_self(self)
				, m_parts(parts)
				, m_obj(obj)
			{
			}

			// IModelOut functions
			geometry::ESceneParts Parts() const override
			{
				return m_parts;
			}
			rdr12::FrameRange FrameRange() const override
			{
				// The frame range of animation data to return
				return m_self.m_anim_info ? m_self.m_anim_info.m_frame_range : ModelGenerator::IModelOut::FrameRange();
			}
			bool ModelFilter(std::string_view model_name) const override
			{
				return m_self.m_model_parts.empty() || m_self.m_model_parts.contains(model_name);
			}
			bool SkeletonFilter(std::string_view skeleton_name) const override
			{
				return m_self.m_skel_parts.empty() || m_self.m_skel_parts.contains(skeleton_name);
			}
			EResult Model(ModelTree&& tree) override
			{
				ModelTreeToLdr(m_obj, tree);
				return EResult::Continue;
			}
			EResult Skeleton(SkeletonPtr&& skel) override
			{
				m_self.m_skels.push_back(skel);
				return EResult::Continue;
			}
			EResult Animation(KeyFrameAnimationPtr&& anim) override
			{
				// If the script didn't ask for animation data, then ignore animation
				if (!m_self.m_anim_info && !m_self.m_montage)
					return EResult::Stop;

				// Otherwise, record the animation
				m_self.m_model_anim = std::move(anim);
				return EResult::Stop;
			}
		};
		struct AnimSourceOut : ModelGenerator::IModelOut
		{
			SkeletonPtr m_skel;
			KeyFrameAnimationPtr m_anim;

			geometry::ESceneParts Parts() const override
			{
				return geometry::ESceneParts::AnimationOnly;
			}
			EResult Skeleton(SkeletonPtr&& skel) override
			{
				m_skel = std::move(skel); return EResult::Continue;
			}
			EResult Animation(KeyFrameAnimationPtr&& anim) override
			{
				m_anim = std::move(anim); return EResult::Stop;
			}
		};

		std::filesystem::path m_filepath;
		std::unique_ptr<std::istream> m_file_stream;
		std::unordered_set<string32> m_model_parts;
		std::unordered_set<string32> m_skel_parts;
		creation::KeyFrameAnimInfo m_anim_info;
		creation::MontageInfo m_montage;
		KeyFrameAnimationPtr m_model_anim; // The model's own animation, stored for montage source index 0
		creation::GenNorms m_gen_norms;
		creation::BakeTransform m_bake;
		vector<SkeletonPtr, 1> m_skels;
		bool m_ignore_materials;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_filepath()
			, m_file_stream()
			, m_model_parts()
			, m_skel_parts()
			, m_anim_info()
			, m_montage()
			, m_model_anim()
			, m_gen_norms()
			, m_bake()
			, m_skels()
			, m_ignore_materials()
		{
		}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::FilePath:
				{
					// Ask the include handler to turn the filepath into a stream.
					// Load the stream in binary mode. The model loading functions can convert binary to text if needed.
					m_filepath = reader.String<std::filesystem::path>();
					m_file_stream = reader.PathResolver.OpenStream(m_filepath, IPathResolver::EFlags::Binary);
					return true;
				}
				case EKeyword::NoMaterials:
				{
					m_ignore_materials = reader.IsSectionEnd() ? true : reader.Bool();
					return true;
				}
				case EKeyword::Parts:
				{
					auto section = reader.SectionScope();
					for (; !reader.IsSectionEnd();)
						m_model_parts.insert(reader.String<string32>());

					return true;
				}
				case EKeyword::Animation:
				{
					m_anim_info.Parse(reader, m_pp);
					return true;
				}
				case EKeyword::Montage:
				{
					m_montage.Parse(reader, m_pp);
					return true;
				}
				default:
				{
					return
						m_bake.ParseKeyword(reader, m_pp, kw) ||
						m_gen_norms.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const& loc) override
		{
			using namespace pr::geometry;
			using namespace std::filesystem;

			// Validate
			if (m_filepath.empty())
			{
				return;
			}
			if (!m_file_stream)
			{
				m_pp.ReportError(EParseError::NotFound, loc, "Failed to open the model file");
				return;
			}
			if (m_anim_info && m_montage)
			{
				m_pp.ReportError(EParseError::InvalidValue, loc, "*Animation and *Montage cannot both be specified in the same *Model");
				return;
			}

			// Determine the format from the file extension
			auto format = GetModelFormat(m_filepath);
			if (format == EModelFileFormat::Unknown)
			{
				auto msg = std::format("Model file '{}' is not supported.\nSupported Formats: ", m_filepath.string());
				for (auto f : Enum<EModelFileFormat>::Members()) msg.append(Enum<EModelFileFormat>::ToStringA(f)).append(" ");
				m_pp.ReportError(EParseError::InvalidValue, loc, msg);
				return;
			}

			// Attach a texture filepath resolver
			vector<path> search_paths = {
				m_filepath.string().append(".textures"),
				m_filepath.parent_path(),
			};
			AutoSub sub = m_pp.m_rdr.ResolveFilepath += [&](auto&, ResolvePathArgs& args)
			{
				// Look in a folder with the same name as the model
				auto resolved = filesys::ResolvePath<vector<path>>(args.filepath, search_paths, nullptr, false, nullptr);
				if (!exists(resolved)) return;
				args.filepath = resolved;
				args.handled = true;
			};

			// Select the parts of the model to load
			auto parts = geometry::ESceneParts::ModelOnly;
			if (m_anim_info || m_montage) parts = SetBits(parts, geometry::ESceneParts::All, true);
			if (m_ignore_materials) parts = SetBits(parts, geometry::ESceneParts::Materials, false);

			// Create the models
			ModelOut out(*this, parts, obj);
			auto opts = ModelGenerator::CreateOptions().colours(m_colours).bake(m_bake.O2WPtr());
			ModelGenerator::LoadModel(format, m_pp.m_factory, *m_file_stream, out, &opts);

			// If animation data is specified, load the model's animation sources and build the animation
			if (m_anim_info)
				CreateAnimation(obj, loc);

			// If a montage is specified, load animation sources and build the montage animation
			if (m_montage)
				CreateMontage(obj, loc);
		}

		// Create an animation for the model using the loaded animation sources and the animation info
		void CreateAnimation(LdrObject* obj, Location const& loc)
		{
			// Find the associated skeleton
			auto const& skeleton = get_if(m_skels, [&](SkeletonPtr skel) { return skel->Id() == m_model_anim->m_skel_id; });
			if (!skeleton)
			{
				m_pp.ReportError(EParseError::InvalidValue, loc, "The skeleton required by the animation is not found");
				return;
			}

			// Overrite the frame rate if given
			if (m_anim_info.m_frame_rate)
			{
				m_model_anim->m_native_frame_rate = *m_anim_info.m_frame_rate;
				m_model_anim->m_native_duration = (m_model_anim->key_count() - 1) / m_model_anim->m_native_frame_rate;
			}

			// The animator to run the animation
			auto animator = AnimatorPtr{ rdr12::New<Animator_KeyFrameAnimation>(m_model_anim), true };

			// The time/frame range in the anim info is the portion of the animation to use during playback
			auto time_range = TimeRange{ 0, animator->Duration() };

			// Create an animator that uses the animation and a pose for it to animate
			PosePtr pose{ rdr12::New<Pose>(m_pp.m_factory, skeleton, animator, m_anim_info.m_mods.m_style, m_anim_info.m_mods.m_flags, time_range, m_anim_info.m_mods.m_stretch, m_anim_info.m_mods.m_bias), true };

			// Set the pose for each model in the hierarchy.
			obj->Apply([&](LdrObject* ob)
			{
				ob->m_pose = pose;
				ob->Flags(ELdrFlags::Animated, m_anim_info.m_mods.m_style != EAnimStyle::NoAnimation);
				ob->Flags(ELdrFlags::HideWhenNotAnimating, m_anim_info.m_mods.m_hide_when_not_animating);
				return true;
			}, "");
		}

		// Load animation sources from the montage and build the composite animation
		void CreateMontage(LdrObject* obj, Location const& loc)
		{
			using namespace pr::geometry;
			using namespace std::filesystem;

			// No frames, no montage
			if (m_montage.m_key_refs.empty())
				return;

			// The model's own animation is always source index 0 for montages, so it must be loaded for the montage to work
			if (!m_model_anim)
			{
				m_pp.ReportError(EParseError::InvalidValue, loc, "*Montage requires a model with an animation (source index 0)");
				return;
			}

			// Find the associated skeleton
			auto const& skeleton = get_if(m_skels, [&](SkeletonPtr skel) { return skel->Id() == m_model_anim->m_skel_id; });
			if (!skeleton)
			{
				m_pp.ReportError(EParseError::InvalidValue, loc, "The skeleton required by the model's animation (source index 0) is not found");
				return;
			}

			// Load the animations from the source animation files
			vector<KeyFrameAnimationPtr> source_anims;
			source_anims.push_back(std::move(m_model_anim)); // The model's own animation is source index 0
			for (int si = 0; si != isize(m_montage.m_source_paths); ++si) // Add the additional animation sources from the montage (indices 1, 2, ...)
			{
				auto const& source_path = m_montage.m_source_paths[si];
				auto& stream = m_montage.m_source_streams[si];
				if (!stream)
				{
					m_pp.ReportError(EParseError::NotFound, loc, std::format("Failed to open animation source '{}'", source_path.string()));
					return;
				}

				auto source_format = GetModelFormat(source_path);
				if (source_format == EModelFileFormat::Unknown)
				{
					m_pp.ReportError(EParseError::InvalidValue, loc, std::format("Unsupported animation source format '{}'", source_path.string()));
					return;
				}

				AnimSourceOut out;
				ModelGenerator::LoadModel(source_format, m_pp.m_factory, *stream, out);

				if (!out.m_anim)
				{
					m_pp.ReportError(EParseError::InvalidValue, loc, std::format("No animation found in source '{}'", source_path.string()));
					return;
				}
				if (!out.m_skel || !skeleton->IsCompatible(*out.m_skel.get()))
				{
					m_pp.ReportError(EParseError::InvalidValue, loc, std::format("Skeleton in '{}' is not compatible with the model skeleton", source_path.string()));
					return;
				}

				source_anims.push_back(std::move(out.m_anim));
			}

			// Validate source indices in frame entries
			for (int i = 0; i != isize(m_montage.m_key_refs); ++i)
			{
				auto const& key_refs = m_montage.m_key_refs;
				if (key_refs[i].source_index < 0 || key_refs[i].source_index >= isize(source_anims))
				{
					m_pp.ReportError(EParseError::InvalidValue, loc, std::format("*Frame source index {} is out of range (0..{})", key_refs[i].source_index, isize(source_anims) - 1));
					return;
				}
			}

			// Create kinematic key frame animation from the multi-source frames
			auto kkfa = KinematicKeyFrameAnimationPtr{ rdr12::New<KinematicKeyFrameAnimation>(skeleton->Id()), true };
			kkfa->Populate(
				source_anims,
				m_montage.m_key_refs,
				m_montage.m_durations,
				m_montage.m_per_frame_o2w
			);

			// Create animator and pose
			AnimatorPtr animator = AnimatorPtr{ rdr12::New<Animator_InterpolatedAnimation>(kkfa), true };
			auto time_range = TimeRange{ 0, animator->Duration() };

			PosePtr pose{ rdr12::New<Pose>(m_pp.m_factory, skeleton, animator, m_montage.m_mods.m_style, m_montage.m_mods.m_flags, time_range, m_montage.m_mods.m_stretch, m_montage.m_mods.m_bias), true };

			// Set the pose for each model in the hierarchy
			obj->Apply([&](LdrObject* o)
			{
				o->m_pose = pose;
				o->Flags(ELdrFlags::Animated, m_montage.m_mods.m_style != EAnimStyle::NoAnimation);
				o->Flags(ELdrFlags::HideWhenNotAnimating, m_montage.m_mods.m_hide_when_not_animating);
				return true;
			}, "");
		}
	};

	// ELdrObject::Equation
	template <> struct ObjectCreator<ELdrObject::Equation> :IObjectCreator
	{
		using VCPair = struct { float m_value; Colour32 m_colour; };
		using ColourBands = pr::vector<VCPair>;

		// An axis for the space that the equation is plotted in
		struct Axis
		{
			float m_min;
			float m_max;
			ColourBands m_col;

			Axis()
				: m_min(limits<float>::max())
				, m_max(limits<float>::lowest())
				, m_col()
			{
			}
			bool limited() const
			{
				return m_min <= m_max;
			}
			float centre() const
			{
				return (m_min + m_max) * 0.5f;
			}
			float radius() const
			{
				return Abs(m_max - m_min) * 0.5f;
			}
			VCPair clamp(float value) const
			{
				VCPair vc = { value, Colour32White };

				// Clamp the range
				if (m_min <= m_max)
				{
					vc.m_value = Clamp(value, m_min, m_max);
					if (vc.m_value != value)
						vc.m_colour.a = 0;
				}

				// Interpolate the colour
				if (!m_col.empty() && vc.m_value == value)
				{
					int i = 0, iend = static_cast<int>(m_col.size());
					for (; i != iend && vc.m_value >= m_col[i].m_value; ++i) {}
					if (i == 0)
						vc.m_colour = m_col.front().m_colour;
					else if (i == iend)
						vc.m_colour = m_col.back().m_colour;
					else
					{
						auto f = Frac(m_col[i - 1].m_value, vc.m_value, m_col[i].m_value);
						vc.m_colour = Lerp(m_col[i - 1].m_colour, m_col[i].m_colour, f);
					}
				}

				return vc;
			}
			static Axis Parse(IReader& reader, ParseParams& pp)
			{
				Axis axis;
				auto section = reader.SectionScope();
				for (EKeyword kw; reader.NextKeyword(kw); )
				{
					switch (kw)
					{
						case EKeyword::Range:
						{
							axis.m_min = reader.Real<float>();
							axis.m_max = reader.Real<float>();
							break;
						}
						case EKeyword::Colours:
						{
							for (; !reader.IsSectionEnd(); )
							{
								VCPair vcpair = {
									.m_value = reader.Real<float>(),
									.m_colour = reader.Int<uint32_t>(16),
								};
								axis.m_col.push_back(vcpair);
							}
							sort(axis.m_col, [](auto& l, auto& r) { return l.m_value < r.m_value; });
							break;
						}
						default:
						{
							pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("Keyword '{}' is not valid within *Axis", EKeyword_::ToStringA(kw)));
							break;
						}
					}
				}
				return axis;
			}
		};

		// Data stored in the object's user data to help with plotting
		struct Extras
		{
			std::array<Axis, 3> m_axis;
			float m_weight;
			Extras()
				: m_axis()
				, m_weight(0.5f)
			{}
			bool has_alpha() const
			{
				for (auto& axis : m_axis)
				{
					for (auto& col : axis.m_col)
					{
						if (col.m_colour.a == 0xFF) continue;
						return true;
					}
				}
				return false;
			}
			BBox clamp_range(BBox range) const
			{
				for (int i = 0, iend = static_cast<int>(m_axis.size()); i != iend; ++i)
				{
					auto& axis = m_axis[i];
					if (!axis.limited()) continue;
					range.m_centre[i] = axis.centre();
					range.m_radius[i] = axis.radius();
				}
				return range;
			}
		};

		// Used to prevent unnecessary recreations of the equation model
		struct Cache
		{
			BBox m_range; // The last size of the rendered equation
			Cache()
				:m_range(BBox::Reset())
			{}
		};

		eval::Expression m_eq;
		eval::ArgSet m_args;
		int m_resolution;
		Extras m_extras;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_eq()
			, m_args()
			, m_resolution(10000)
			, m_extras()
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					// Compile the equation
					try
					{
						auto equation = reader.String<string32>();
						m_eq = eval::Compile(equation);
					}
					catch (std::exception const& ex)
					{
						m_pp.ReportError(EParseError::InvalidValue, reader.Loc(), std::format("Equation expression is invalid: {}", ex.what()));
					}
					return true;
				}
				case EKeyword::Resolution:
				{
					m_resolution = reader.Int<int>(10);
					m_resolution = std::clamp(m_resolution, 8, 0xFFFF);
					return true;
				}
				case EKeyword::Param:
				{
					auto variable = reader.String<string32>();
					auto value = reader.Real<double>();
					m_args.add(variable, value);
					return true;
				}
				case EKeyword::Weight:
				{
					m_extras.m_weight = reader.Real<float>();
					m_extras.m_weight = std::clamp(m_extras.m_weight, -1.0f, +1.0f);
					return true;
				}
				case EKeyword::XAxis:
				{
					m_extras.m_axis[0] = Axis::Parse(reader, m_pp);
					return true;
				}
				case EKeyword::YAxis:
				{
					m_extras.m_axis[1] = Axis::Parse(reader, m_pp);
					return true;
				}
				case EKeyword::ZAxis:
				{
					m_extras.m_axis[2] = Axis::Parse(reader, m_pp);
					return true;
				}
				default:
				{
					return IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			using namespace pr::geometry;

			// Validate
			if (!m_eq)
				return;

			// Apply any constants
			m_eq.m_args.add(m_args);

			// Update the model before each render so the range depends on the visible area at the focus point
			obj->OnAddToScene += UpdateModel;

			// Choose suitable 'vcount, icount' based on the equation dimension and resolution
			int vcount, icount, dim = m_eq.m_args.unassigned_count();
			switch (dim)
			{
				case 1: vcount = icount = m_resolution; break;
				case 2: vcount = m_resolution; icount = 2 * m_resolution; break;
				case 3: vcount = icount = m_resolution; break;
				default: throw std::runtime_error(std::format("Unsupported equation dimension: {}", dim));
			}

			// Store the expression in the object user data
			obj->m_user_data.get<eval::Expression>() = m_eq;
			obj->m_user_data.get<Extras>() = m_extras;
			obj->m_user_data.get<Cache>() = Cache{};

			// Create buffers for a dynamic model
			ModelDesc mdesc = ModelDesc()
				.vbuf(ResDesc::VBuf<Vert>(vcount, {}))
				.ibuf(ResDesc::IBuf<uint32_t>(icount, {}))
				.bbox(BBox::Reset())
				.name(obj->TypeAndName());

			// Create the model
			obj->m_model = m_pp.m_factory.CreateModel(mdesc);
			obj->m_model->m_name = obj->TypeAndName();
		}

		// Generate the model based on the visible range
		static void UpdateModel(LdrObject& ob, Scene const& scene)
		{
			// Notes:
			//  - This code attempts to give the effect of an infinite function or surface by creating graphics
			//    within the view frustum as the camera moves. It evaluates the equation within a cube centred
			//    on the focus point.
			//  - no back-face culling
			//  - only update the model when the camera moves by ? distance.
			//  - functions can have infinities and divide by zero
			//  - set the bbox to match the view volume so that auto range doesn't zoom out to infinity

			auto& model = *ob.m_model.get();
			auto& equation = ob.m_user_data.get<eval::Expression>();
			auto& extras = ob.m_user_data.get<Extras>();
			auto& cache = ob.m_user_data.get<Cache>();
			auto init = model.m_nuggets.empty();

			// Find the range to plot the equation over
			auto& cam = scene.m_cam;
			auto fp = cam.FocusPoint();
			auto area = cam.ViewRectAtDistance(cam.FocusDist());

			// Determine the interval to plot within. Default to a sphere around the focus point.
			auto range = BBox(fp, v4(area.x, area.x, area.x, 0));
			range = extras.clamp_range(range);

			// Only update the model if necessary
			if (init || !IsWithin(cache.m_range, range))
			{
				// Functions can have infinities and divide by zeros. Set the bbox
				// to match the view volume so that auto-range doesn't zoom out to infinity.
				model.m_bbox.reset();
				model.m_bbox = BBox::Unit();

				// Update the model by evaluating the equation
				switch (equation.m_args.unassigned_count())
				{
					case 1: // Line plot
					{
						LinePlot(model, range, equation, extras, init);
						break;
					}
					case 2: // Surface plot
					{
						SurfacePlot(model, range, equation, extras, init);
						break;
					}
					case 3:
					{
						CloudPlot(model, range, equation, extras, init);
						break;
					}
					default:
					{
						assert(false && "Unsupported equation dimension");
						break;
					}
				}

				// Save the range last rendered
				cache.m_range = range;
			}

			// Update object colour, visibility, etc
			ApplyObjectState(&ob);
		}
		static void LinePlot(Model& model, BBox range, eval::Expression const& equation, Extras const& extras, bool init)
		{
			// Notes:
			//  - 'range' is the independent variable range. For line plots, only 'x' is used
			//  - 'extras.m_axis' contains the bounds on output values and colour gradients.
			auto vcount = s_cast<int>(model.m_vcount);
			auto icount = s_cast<int>(model.m_icount);
			auto count = std::min(vcount, icount);
			
			(void)equation, extras, range;
			#if 0 //todo
			// Populate verts
			{
				Lock vlock;
				model.MapVerts(vlock, EMap::WriteDiscard);
				auto vout = vlock.ptr<Vert>();
				for (int i = 0; i != count; ++i)
				{
					// 'f' is a point in the range [-1.0,+1.0]. Rescale to the range.
					// 'weight' controls the density of points near the range centre.
					auto f = Lerp(-1.0f, +1.0f, (1.0f * i) / (count - 1));
					auto x = range.m_centre.x + f * range.m_radius.x * Pow(Abs(f), extras.m_weight);
					auto [y,col] = extras.m_axis[1].clamp(static_cast<float>(equation(x).db()));
					SetPC(*vout, v4(x, y, 0, 1), col);
					++vout;
				}
			}

			// Populate faces
			if (init)
			{
				Lock ilock;
				model.MapIndices(ilock, EMap::WriteDiscard);
				auto iout = ilock.ptr<uint32_t>();
				for (int i = 0; i != count; ++i)
				{
					*iout = static_cast<uint32_t>(i);
					++iout;
				}
			}
			#endif

			// Populate nuggets
			if (init)
			{
				// Create a nugget
				NuggetDesc n = {};
				n.m_topo = ETopo::LineStrip;
				n.m_geom = EGeom::Vert;
				n.m_vrange = Range(0, count);
				n.m_irange = Range(0, count);
				n.m_nflags = extras.has_alpha() ? ENuggetFlag::GeometryHasAlpha : ENuggetFlag::None;
				model.DeleteNuggets();

				ResourceFactory factory(model.rdr());
				model.CreateNugget(factory, n);
			}
		}
		static void SurfacePlot(Model& model, BBox range, eval::Expression const& equation, Extras const& extras, bool init)
		{
			// Notes:
			//  - 'range' is the independent variable range. For surface plots, 'x' and 'y' are used.
			//  - 'extras.m_axis' contains the bounds on output values and colour gradients.

			// Determine the largest hex patch that can be made with the available model size:
			//  i.e. solve for the minimum value for 'rings' in:
			//'    vcount = ArithmeticSum(0, 6, rings) + 1;
			//'    icount = ArithmeticSum(0, 12, rings) + 2*rings;
			// ArithmeticSum := (n + 1) * (a0 + an) / 2, where an = (a0 + n * step)
			//    3r^2 + 3r + 1-vcount = 0  =>  r = (-3 +/- sqrt(-3 + 12*vcount)) / 6
			//    6r^2 + 8r - icount = 0    =>  r = (-8 +/- sqrt(64 + 24*icount)) / 12
			auto vrings = (-3 + sqrt(-3 + 12 * model.m_vcount)) / 6;
			auto irings = (-8 + sqrt(64 + 24 * model.m_icount)) / 12;
			auto rings = static_cast<int>(std::min(vrings, irings));
			auto dx_step = range.SizeX() * 1e-5f;
			auto dy_step = range.SizeY() * 1e-5f;

			auto [nv, ni] = geometry::HexPatchSize(rings);
			assert(nv <= (int)model.m_vcount);
			assert(ni <= (int)model.m_icount);

			ResourceFactory factory(model.rdr());
			auto update_v = model.UpdateVertices(factory.CmdList(), factory.UploadBuffer());
			auto update_i = model.UpdateIndices(factory.CmdList(), factory.UploadBuffer());
			auto vout = update_v.ptr<Vert>();
			auto iout = update_i.ptr<uint32_t>();

			auto props = geometry::HexPatch(rings,
				[&](v4 pos, Colour32, v4, v2)
				{
					// Evaluate the function at points around the focus point, but shift them back
					// so the focus point is centred around (0,0,0), then set the o2w transform

					// 'pos' is a point in the range [-1.0,+1.0]. Rescale to the range.
					// 'weight' controls the density of points near the range centre since 'len_sq' is on [0,1].
					auto dir = pos.w0();
					auto len_sq = LengthSq(dir);
					auto weight = Lerp(extras.m_weight, 1.0f, len_sq);
					auto pt = range.Centre() + dir * range.Radius() * weight;
					
					// Evaluate the equation at 'pt' to get z = f(x,y) and the colour.
					auto [z,col] = extras.m_axis[2].clamp(static_cast<float>(equation(pt.x, pt.y).db()));

					// Evaluate the normal at 'pt'. Want to choose a 'd' value that is proportional to the density of points at 'pt'
					auto dx = dx_step * weight; // this isn't right, 'd' should be the smallest step that produces an accurate normal...
					auto dy = dy_step * weight;

					// Evaluate the function at four points around (x,y) to get the height 'h'
					auto h = equation(v4(pt.x - dx, pt.x + dx, pt.x, pt.x), v4(pt.y, pt.y, pt.y - dy, pt.y + dy)).v4();
					auto n = Cross(v4(2 * dx, 0, h.y - h.x, 0), v4(0, 2 * dy, h.w - h.z, 0));
					auto norm = Normalise(n, v4::Zero());

					SetPCNT(*vout++, v4(pt.x, pt.y, z, 1), pr::Colour(col), norm, v2::Zero());
				},
				[&](auto idx)
				{
					*iout++ = idx;
				});

			assert(vout - update_v.ptr<Vert>() == nv);
			assert(iout - update_i.ptr<uint32_t>() == ni);
			update_v.Commit();
			update_i.Commit();

			// Generate nuggets if initialising
			if (init)
			{
				model.CreateNugget(factory, NuggetDesc(ETopo::TriStrip, props.m_geom).vrange(Range(0, nv)).irange(Range(0, ni)).alpha_geom(extras.has_alpha()));
			}
		}
		static void CloudPlot(Model& model, BBox range, eval::Expression const& equation, Extras const& extras, bool init)
		{
			(void)model, range, equation, extras, init;
			throw std::runtime_error("Plots of 3 independent variables are not supported");
		}
	};

	#pragma endregion

	#pragma region Special Objects

	// ELdrObject::Group
	template <> struct ObjectCreator<ELdrObject::Group> :IObjectCreator
	{
		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
		{}
	};

	// ELdrObject::LightSource
	template <> struct ObjectCreator<ELdrObject::LightSource> :IObjectCreator
	{
		Light m_light;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_light()
		{
			m_light.m_on = true;
		}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Style:
				{
					auto ident = reader.Identifier<string32>();
					switch (HashI(ident.c_str()))
					{
						case HashI("Directional"): m_light.m_type = ELight::Directional; break;
						case HashI("Point"): m_light.m_type = ELight::Point; break;
						case HashI("Spot"): m_light.m_type = ELight::Spot; break;
						default: m_pp.ReportError(EParseError::InvalidValue, reader.Loc(), std::format("{} is an unknown light type", ident));
					}
					return true;
				}
				case EKeyword::Ambient:
				{
					m_light.m_ambient = reader.Int<uint32_t>(16);
					return true;				
				}
				case EKeyword::Diffuse:
				{
					m_light.m_diffuse = reader.Int<uint32_t>(16);
					return true;
				}
				case EKeyword::Specular:
				{
					m_light.m_specular = reader.Int<uint32_t>(16);
					m_light.m_specular_power = reader.Real<float>();
					return true;
				}
				case EKeyword::Range:
				{
					m_light.m_range = reader.Real<float>();
					m_light.m_falloff = reader.Real<float>();
					return true;
				}
				case EKeyword::Cone:
				{
					m_light.m_inner_angle = reader.Real<float>(); // in degrees
					m_light.m_outer_angle = reader.Real<float>(); // in degrees
					return true;
				}
				case EKeyword::CastShadow:
				{
					m_light.m_cast_shadow = reader.Real<float>();
					return true;
				}
				default:
				{
					return IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// Assign the light data as user data
			obj->m_user_data.get<Light>() = m_light;
		}
	};

	// ELdrObject::Text
	template <> struct ObjectCreator<ELdrObject::Text> :IObjectCreator
	{
		enum class EType
		{
			Full3D,
			Billboard3D,
			Billboard,
			ScreenSpace,
		};
		using TextFmtCont = pr::vector<TextFormat>;

		wstring32 m_text;
		EType m_type;
		TextFmtCont m_fmt;
		TextLayout m_layout;
		creation::MainAxis m_axis;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_text()
			, m_type(EType::Full3D)
			, m_fmt()
			, m_layout()
			, m_axis(pr::AxisId::PosZ)
		{}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					auto text = reader.String<string32>();
					m_text.append(Widen(text));

					// Record the formatting state
					m_fmt.push_back(TextFormat(int(m_text.size() - text.size()), isize(text), m_pp.m_font.back()));
					return true;
				}
				case EKeyword::CString:
				{
					auto text = reader.String<string32>('\\');
					m_text.append(Widen(text));

					// Record the formatting state
					m_fmt.push_back(TextFormat(int(m_text.size() - text.size()), isize(text), m_pp.m_font.back()));
					return true;
				}
				case EKeyword::NewLine:
				{
					m_text.append(L"\n");
					return true;
				}
				case EKeyword::ScreenSpace:
				{
					m_type = EType::ScreenSpace;
					return true;
				}
				case EKeyword::Billboard:
				{
					m_type = EType::Billboard;
					return true;
				}
				case EKeyword::Billboard3D:
				{
					m_type = EType::Billboard3D;
					return true;
				}
				case EKeyword::BackColour:
				{
					m_layout.m_bk_colour = reader.Int<uint32_t>(16);
					return true;
				}
				case EKeyword::Format:
				{
					auto ident = reader.Identifier<string32>();
					switch (HashI(ident.c_str()))
					{
						case HashI("Left"): m_layout.m_align_h = DWRITE_TEXT_ALIGNMENT_LEADING; break;
						case HashI("CentreH"): m_layout.m_align_h = DWRITE_TEXT_ALIGNMENT_CENTER; break;
						case HashI("Right"): m_layout.m_align_h = DWRITE_TEXT_ALIGNMENT_TRAILING; break;
						default: m_pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("{} is not a valid horizontal alignment value", ident));
					}
					ident = reader.Identifier<string32>();
					switch (HashI(ident.c_str()))
					{
						case HashI("Top"): m_layout.m_align_v = DWRITE_PARAGRAPH_ALIGNMENT_NEAR; break;
						case HashI("CentreV"): m_layout.m_align_v = DWRITE_PARAGRAPH_ALIGNMENT_CENTER; break;
						case HashI("bottom"): m_layout.m_align_v = DWRITE_PARAGRAPH_ALIGNMENT_FAR; break;
						default: m_pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("{} is not a valid vertical alignment value", ident));
					}
					ident = reader.Identifier<string32>();
					switch (HashI(ident.c_str()))
					{
						case HashI("Wrap"): m_layout.m_word_wrapping = DWRITE_WORD_WRAPPING_WRAP; break;
						case HashI("NoWrap"): m_layout.m_word_wrapping = DWRITE_WORD_WRAPPING_NO_WRAP; break;
						case HashI("WholeWord"): m_layout.m_word_wrapping = DWRITE_WORD_WRAPPING_WHOLE_WORD; break;
						case HashI("Character"): m_layout.m_word_wrapping = DWRITE_WORD_WRAPPING_CHARACTER; break;
						case HashI("EmergencyBreak"): m_layout.m_word_wrapping = DWRITE_WORD_WRAPPING_EMERGENCY_BREAK; break;
						default: m_pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), std::format("{} is not a valid word wrapping value", ident));
					}
					return true;
				}
				case EKeyword::Anchor:
				{
					m_layout.m_anchor = reader.Vector2f();
					return true;
				}
				case EKeyword::Padding:
				{
					auto padding = reader.Vector4f();
					m_layout.m_padding.left = padding.x;
					m_layout.m_padding.top = padding.y;
					m_layout.m_padding.right = padding.z;
					m_layout.m_padding.bottom = padding.w;
					return true;
				}
				case EKeyword::Dim:
				{
					m_layout.m_dim = reader.Vector2f();
					return true;
				}
				default:
				{
					return
						m_axis.ParseKeyword(reader, m_pp, kw) ||
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const&) override
		{
			// If an explicit colour was given for the text object, apply it to the font colour
			// for any text ranges that still use the default font colour. The font colour is baked
			// into the text texture, so the object tint colour can't brighten it (black * tint = black).
			// After applying the colour to the font, reset the object tint to white so that the
			// texture renders without colour distortion.
			if (AllSet(m_pp.m_flags, EFlags::ExplicitColour))
			{
				auto default_colour = Font().m_colour;
				for (auto& fmt : m_fmt)
				{
					if (fmt.m_font.m_colour == default_colour)
						fmt.m_font.m_colour = obj->m_base_colour;
				}
				obj->m_base_colour = Colour32White;
			}

			// Create a quad containing the text
			obj->m_model = ModelGenerator::Text(m_pp.m_factory, m_text, m_fmt, m_layout, 1.0, m_axis.m_align.m_axis);
			obj->m_model->m_name = obj->TypeAndName();

			// Create the model
			switch (m_type)
			{
				// Text is a normal 3D object
				case EType::Full3D:
				{
					break;
				}
				// Position the text quad so that it always faces the camera but scales with distance
				case EType::Billboard3D:
				{
					// Do not include in scene bounds calculations because we're scaling
					// this model at a point that the bounding box calculation can't see.
					obj->Flags(ELdrFlags::SceneBoundsExclude, true, "");

					// Update the rendering 'i2w' transform on add-to-scene
					obj->OnAddToScene += [](LdrObject& ob, Scene const& scene)
					{
						// The size of the text texture is the text metrics size / 96.0.
						auto c2w = scene.m_cam.CameraToWorld();
						auto w2c = scene.m_cam.WorldToCamera();
						auto w = 1.0f * scene.m_viewport.ScreenW;
						auto h = 1.0f * scene.m_viewport.ScreenH;
						#if PR_DBG
						if (w < 1.0f || h < 1.0f)
							throw std::runtime_error("Invalid viewport size");
						#endif

						// Create a camera with an aspect ratio that matches the viewport
						// This handles the case where main camera X/Y are not using the same resolution.
						auto& main_camera = static_cast<Camera const&>(scene.m_cam);
						auto  text_camera = main_camera; text_camera.Aspect(w / h);
						auto fd = main_camera.FocusDist();

						// Get the scaling factors from 'main_camera' to 'text_camera'
						auto viewarea_camera = main_camera.ViewRectAtDistance(fd);
						auto viewarea_txtcam = text_camera.ViewRectAtDistance(fd);

						// Scale the X,Y coordinates in camera space
						auto pt_cs = w2c * ob.m_i2w.pos;
						pt_cs.x *= viewarea_txtcam.x / viewarea_camera.x;
						pt_cs.y *= viewarea_txtcam.y / viewarea_camera.y;
						auto pt_ws = c2w * pt_cs;

						// Position facing the camera
						ob.m_i2w = m4x4(c2w.rot, pt_ws) * ob.m_i2w.scale();
						ob.m_c2s = text_camera.CameraToScreen();
					};
					break;
				}
				// Position the text quad so that it always faces the camera and has the same size
				case EType::Billboard:
				{
					// Do not include in scene bounds calculations because we're scaling
					// this model at a point that the bounding box calculation can't see.
					obj->Flags(
						ELdrFlags::BBoxExclude |
						ELdrFlags::SceneBoundsExclude |
						ELdrFlags::HitTestExclude |
						ELdrFlags::ShadowCastExclude, true, "");

					// Scale up the view port to reduce floating point precision noise.
					constexpr int ViewPortSize = 1024;

					// Screen space uses a standard normalised orthographic projection
					obj->m_c2s = m4x4::ProjectionOrthographic(float(ViewPortSize), float(ViewPortSize), -0.01f, 1, true);

					// Update the rendering 'i2w' transform on add-to-scene
					obj->OnAddToScene += [](LdrObject& ob, Scene const& scene)
					{
						auto& main_camera = static_cast<Camera const&>(scene.m_cam);
						auto c2w = main_camera.CameraToWorld();
						auto w2c = main_camera.WorldToCamera();
						auto w = 1.0f * scene.m_viewport.ScreenW;
						auto h = 1.0f * scene.m_viewport.ScreenH;
						#if PR_DBG
						if (w < 1.0f || h < 1.0f)
							throw std::runtime_error("Invalid viewport size");
						#endif

						// Convert the world space position into a screen space position
						auto pt_ss = w2c * ob.m_i2w.pos;
						auto viewarea = main_camera.ViewRectAtDistance(abs(pt_ss.z));
						pt_ss.x *= ViewPortSize / viewarea.x;
						pt_ss.y *= ViewPortSize / viewarea.y;
						pt_ss.z = static_cast<float>(main_camera.NormalisedDistance(pt_ss.z));

						// The text quad has a scale of 100pt == 1m. For screen space make this 100pt * 96/72 == 133px
						constexpr float m_to_px = 133.0f;

						// Scale the object from physical pixels to normalised screen space
						auto scale = m4x4::Scale(m_to_px * ViewPortSize / w, m_to_px * ViewPortSize / h, 1, v4::Origin());

						// Construct the 'i2w' using the screen space position
						ob.m_i2w = c2w * m4x4::Translation(pt_ss.x, pt_ss.y, pt_ss.z) * scale * ob.m_i2w.scale().w1();
					};
					break;
				}
				// Position the text quad in screen space.
				case EType::ScreenSpace:
				{
					// Do not include in scene bounds calculations because we're scaling
					// this model at a point that the bounding box calculation can't see.
					obj->Flags(
						ELdrFlags::BBoxExclude |
						ELdrFlags::SceneBoundsExclude |
						ELdrFlags::HitTestExclude |
						ELdrFlags::ShadowCastExclude, true, "");

					// Scale up the view port to reduce floating point precision noise.
					constexpr int ViewPortSize = 1024;

					// Screen space uses a standard normalised orthographic projection
					obj->m_c2s = m4x4::ProjectionOrthographic(float(ViewPortSize), float(ViewPortSize), -0.01f, 1, true);

					// Update the rendering 'i2w' transform on add-to-scene.
					obj->OnAddToScene += [](LdrObject& ob, Scene const& scene)
					{
						// The 'ob.m_i2w' is a normalised screen space position
						// (-1,-1,-0) is the lower left corner on the near plane,
						// (+1,+1,-1) is the upper right corner on the far plane.
						auto& main_camera = static_cast<Camera const&>(scene.m_cam);
						auto c2w = main_camera.CameraToWorld();
						auto w = 1.0f * scene.m_viewport.ScreenW;
						auto h = 1.0f * scene.m_viewport.ScreenH;
						#if PR_DBG
						if (w < 1.0f || h < 1.0f)
							throw std::runtime_error("Invalid viewport size");
						#endif

						// Convert the position given in the ldr script as 2D screen space
						// Note: pt_ss.z should already be the normalised distance from the camera
						auto pt_ss = ob.m_i2w.pos;
						pt_ss.x *= 0.5f * ViewPortSize;
						pt_ss.y *= 0.5f * ViewPortSize;

						// The text quad has a scale of 100pt == 1m. For screen space make this 100pt * 96/72 == 133px
						constexpr float m_to_px = 133.0f;

						// Scale the object from physical pixels to normalised screen space
						auto scale = m4x4::Scale(m_to_px * ViewPortSize / w, m_to_px * ViewPortSize / h, 1, v4::Origin());

						// Convert 'i2w', which is 'i2c' in the ldr script, into an actual 'i2w'
						ob.m_i2w = c2w * m4x4::Translation(pt_ss.x, pt_ss.y, pt_ss.z) * scale * ob.m_i2w.scale();
					};
					break;
				}
				default:
				{
					throw std::runtime_error(std::format("Unknown Text mode: {}", int(m_type)));
				}
			}
		}
	};

	// ELdrObject::Instance
	template <> struct ObjectCreator<ELdrObject::Instance> :IObjectCreator
	{
		string32 m_source;
		creation::KeyFrameAnimInfo m_anim_info;
		creation::MontageInfo m_montage;
		std::unordered_map<Pose const*, PosePtr> m_pose_map;

		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
			, m_source()
			, m_anim_info()
			, m_montage()
			, m_pose_map()
		{
		}
		bool ParseKeyword(IReader& reader, EKeyword kw) override
		{
			switch (kw)
			{
				case EKeyword::Data:
				{
					// The object name of the source to instance
					m_source = reader.Identifier<string32>(true);
					return true;
				}
				case EKeyword::Animation:
				{
					m_anim_info.Parse(reader, m_pp);
					return true;
				}
				case EKeyword::Montage:
				{
					m_montage.Parse(reader, m_pp);
					return true;
				}
				default:
				{
					return
						IObjectCreator::ParseKeyword(reader, kw);
				}
			}
		}
		void CreateModel(LdrObject* obj, Location const& loc) override
		{
			// Ignore empty instances
			if (m_source.empty())
				return;

			// *Animation and *Montage are mutually exclusive
			if (m_anim_info && m_montage)
			{
				m_pp.ReportError(EParseError::InvalidValue, loc, "*Animation and *Montage cannot both be specified in the same *Instance");
				return;
			}

			LdrObject* source = nullptr;
			std::string_view addr{ m_source };
			string32 path;

			// If 'addr' starts with a '.' then it's a relative address.
			if (addr.front() == '.')
			{
				// Navigate up through parents...
				LdrObject* src = obj;
				for (; src != nullptr && !addr.empty() && addr.front() == '.'; addr.remove_prefix(1))
					src = src->m_parent;

				//... and down through children
				for (; src != nullptr && !addr.empty(); )
				{
					// Find the child matching 'segment'
					auto segment = addr.substr(0, index_of(addr, '.'));
					auto it = find_if(src->m_child, [=](LdrObjectPtr& c) { return std::string_view{ c->m_name } == segment; });
					src = it != std::end(src->m_child) ? it->get() : nullptr;
					if (src == nullptr)
						break;

					// Advance to the next segment
					addr = addr.substr(segment.size());
					if (!addr.empty()) addr.remove_prefix(1);
				}
				source = src;
			}
			else
			{
				// Find the source object in the lookup
				if (auto it = m_pp.m_lookup.find(hash::Hash(addr)); it != std::end(m_pp.m_lookup))
					source = it->second;
			}

			// Instance source not found?
			if (source == nullptr)
			{
				m_pp.ReportError(EParseError::NotFound, loc, "Object not found. Can't create an instance.");
				return;
			}

			// Create an LdrObject instance for each nested object
			RecursiveCreate(obj, source, false);

			// Clone the pose if animation info is given
			if (m_anim_info && source->m_pose != nullptr)
				CreateAnimation(obj, source, loc);

			// Build a montage animation for this instance
			if (m_montage && source->m_pose != nullptr)
				CreateMontage(obj, source, loc);
		}

		// Recursively create an instance by copying the model and properties of 'source' into 'obj' and doing the same for each child.
		void RecursiveCreate(LdrObject* obj, LdrObject const* source, bool copy_props)
		{
			obj->m_model = source->m_model;
			if (copy_props)
			{
				obj->m_o2p = source->m_o2p;
				obj->m_base_colour = source->m_base_colour;
				obj->m_grp_colour = source->m_grp_colour;
				obj->m_root_anim = source->m_root_anim;
				obj->m_screen_space = source->m_screen_space;
				obj->m_flags_local = source->m_flags_local;
				obj->m_flags_recursive = source->m_flags_recursive;
			}

			for (auto source_child : source->m_child)
			{
				LdrObjectPtr child(new LdrObject(ELdrObject::Instance, obj, obj->m_context_id), true);
				RecursiveCreate(child.get(), source_child.get(), true);
				child->m_name = source_child->m_name;
				obj->m_child.push_back(std::move(child));
			}
		}

		// Build an animation for the instance by cloning the source object's pose and applying the animation mods
		void CreateAnimation(LdrObject* obj, LdrObject const* source, Location const&)
		{
			// Clamp the time range to the frame range
			auto time_range = ToTimeRange(m_anim_info.m_frame_range, source->m_pose->m_animator->FrameRate());
			time_range = Intersect(time_range, source->m_pose->m_time_range);
			time_range = Intersect(time_range, m_anim_info.m_time_range);

			// Clone the animator because it can have cached state for efficient animation sampling
			AnimatorPtr animator = source->m_pose->m_animator->Clone();

			// Create a new pose for the instance using the source pose's skeleton and the cloned animator
			PosePtr pose{ rdr12::New<Pose>(m_pp.m_factory, source->m_pose->m_skeleton, animator, m_anim_info.m_mods.m_style, m_anim_info.m_mods.m_flags, time_range, m_anim_info.m_mods.m_stretch, m_anim_info.m_mods.m_bias), true };

			// Set the pose for each model in the hierarchy.
			obj->Apply([&](LdrObject* obj)
			{
				obj->m_pose = pose;
				obj->Flags(ELdrFlags::Animated, m_anim_info.m_mods.m_style != EAnimStyle::NoAnimation);
				obj->Flags(ELdrFlags::HideWhenNotAnimating, m_anim_info.m_mods.m_hide_when_not_animating);
				return true;
			}, "");
		}

		// Build a montage animation for the instance using the source object's animation as source index 0
		void CreateMontage(LdrObject* obj, LdrObject const* source, Location const& loc)
		{
			using namespace pr::geometry;
			using namespace std::filesystem;

			// No frames, no montage
			if (m_montage.m_key_refs.empty())
				return;

			// The source's pose must use a KeyFrameAnimation-based animator so we can access the raw animation data
			auto* kfa_animator = dynamic_cast<Animator_KeyFrameAnimation const*>(source->m_pose->m_animator.get());
			if (!kfa_animator || !kfa_animator->m_anim)
			{
				m_pp.ReportError(EParseError::InvalidValue, loc, "*Montage on *Instance requires the source object to have a key-frame animation (source index 0)");
				return;
			}

			auto const& skeleton = source->m_pose->m_skeleton;
			auto const& model_anim = kfa_animator->m_anim;

			// Collect animation sources: index 0 is the source object's animation
			vector<KeyFrameAnimationPtr> source_anims;
			source_anims.push_back(model_anim);

			// Load additional animation sources from the montage
			for (int si = 0; si != isize(m_montage.m_source_paths); ++si)
			{
				auto const& source_path = m_montage.m_source_paths[si];
				auto& stream = m_montage.m_source_streams[si];
				if (!stream)
				{
					m_pp.ReportError(EParseError::NotFound, loc, std::format("Failed to open animation source '{}'", source_path.string()));
					return;
				}

				auto source_format = GetModelFormat(source_path);
				if (source_format == EModelFileFormat::Unknown)
				{
					m_pp.ReportError(EParseError::InvalidValue, loc, std::format("Unsupported animation source format '{}'", source_path.string()));
					return;
				}

				// Reuse the AnimSourceOut type from the *Model creator
				struct AnimSourceOut : ModelGenerator::IModelOut
				{
					SkeletonPtr m_skel;
					KeyFrameAnimationPtr m_anim;

					geometry::ESceneParts Parts() const override
					{
						return geometry::ESceneParts::AnimationOnly;
					}
					EResult Skeleton(SkeletonPtr&& skel) override
					{
						m_skel = std::move(skel); return EResult::Continue;
					}
					EResult Animation(KeyFrameAnimationPtr&& anim) override
					{
						m_anim = std::move(anim); return EResult::Stop;
					}
				};

				AnimSourceOut out;
				ModelGenerator::LoadModel(source_format, m_pp.m_factory, *stream, out);

				if (!out.m_anim)
				{
					m_pp.ReportError(EParseError::InvalidValue, loc, std::format("No animation found in source '{}'", source_path.string()));
					return;
				}
				if (!out.m_skel || !skeleton->IsCompatible(*out.m_skel.get()))
				{
					m_pp.ReportError(EParseError::InvalidValue, loc, std::format("Skeleton in '{}' is not compatible with the source skeleton", source_path.string()));
					return;
				}

				source_anims.push_back(std::move(out.m_anim));
			}

			// Validate source indices in frame entries
			for (int i = 0; i != isize(m_montage.m_key_refs); ++i)
			{
				auto const& key_refs = m_montage.m_key_refs;
				if (key_refs[i].source_index < 0 || key_refs[i].source_index >= isize(source_anims))
				{
					m_pp.ReportError(EParseError::InvalidValue, loc, std::format("*Frame source index {} is out of range (0..{})", key_refs[i].source_index, isize(source_anims) - 1));
					return;
				}
			}

			// Create kinematic key frame animation from the multi-source frames
			auto kkfa = KinematicKeyFrameAnimationPtr{ rdr12::New<KinematicKeyFrameAnimation>(skeleton->Id()), true };
			kkfa->Populate(
				source_anims,
				m_montage.m_key_refs,
				m_montage.m_durations,
				m_montage.m_per_frame_o2w
			);
			auto time_range = TimeRange{ 0, kkfa->duration() };

			// Create animator and pose
			AnimatorPtr animator = AnimatorPtr{ rdr12::New<Animator_InterpolatedAnimation>(kkfa), true };

			// Create a new pose for the instance using the source pose's skeleton and the montage animator
			PosePtr pose{ rdr12::New<Pose>(m_pp.m_factory, skeleton, animator, m_montage.m_mods.m_style, m_montage.m_mods.m_flags, time_range, m_montage.m_mods.m_stretch, m_montage.m_mods.m_bias), true };

			// Set the pose for each model in the hierarchy
			obj->Apply([&](LdrObject* o)
			{
				o->m_pose = pose;
				o->Flags(ELdrFlags::Animated, m_montage.m_mods.m_style != EAnimStyle::NoAnimation);
				o->Flags(ELdrFlags::HideWhenNotAnimating, m_montage.m_mods.m_hide_when_not_animating);
				return true;
			}, "");
		}
	};

	// ELdrObject::Unknown
	template <> struct ObjectCreator<ELdrObject::Unknown> : IObjectCreator
	{
		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
		{}
	};

	// ELdrObject::Custom
	template <> struct ObjectCreator<ELdrObject::Custom> : IObjectCreator
	{
		ObjectCreator(ParseParams& pp)
			: IObjectCreator(pp)
		{}
	};

	#pragma endregion

	// Reads a single ldr object from a script adding object (+ children) to 'pp.m_objects'.
	// Returns true if an object was read or false if the next keyword is unrecognised
	bool ParseLdrObject(ELdrObject type, IReader& reader, ParseParams& pp)
	{
		// Push a font onto the font stack, so that fonts are scoped to object declarations
		auto font_scope = Scope<void>(
			[&]{ pp.m_font.push_back(pp.m_font.back()); },
			[&]{ pp.m_font.pop_back(); });

		// Parse the object
		LdrObjectPtr obj = {};
		switch (type)
		{
			#define PR_LDRAW_PARSE_OBJECTS(name)\
			case ELdrObject::name:\
			{\
				pp.m_type = ELdrObject::name;\
				ObjectCreator<ELdrObject::name> creator(pp);\
				obj = creator.Parse(reader);\
				break;\
			}
			PR_LDRAW_OBJECTS(PR_LDRAW_PARSE_OBJECTS)
			#undef PR_LDRAW_PARSE_OBJECTS
			default: return false;
		}

		// If an object was created add it to the parse results
		if (obj != nullptr)
		{
			// Apply properties to the object
			// This is done after objects are parsed so that recursive properties can be applied
			ApplyObjectState(obj.get());

			// Add to the lookup
			pp.m_lookup[hash::Hash(obj->FullName())] = obj.get();

			// Add the object to the container
			pp.m_objects.push_back(obj);
		}

		// Reset the memory pool for the next object
		pp.m_cache.Reset();

		// Report progress
		pp.ReportProgress(reader);

		return true;
	}

	// Reads all ldr objects from a script. 'add_cb' is 'void function(int object_index)'
	template <std::invocable<int> AddCB>
	void ParseLdrObjects(IReader& reader, ParseParams& pp, AddCB add_cb)
	{
		// Loop over keywords in the script
		for (EKeyword kw; !pp.m_cancel && !pp.m_stop_token.stop_requested() && reader.NextKeyword(kw);)
		{
			switch (kw)
			{
				case EKeyword::Commands:
				{
					ParseCommands(reader, pp, pp.m_result);
					break;
				}
				case EKeyword::Camera:
				{
					ParseCamera(reader, pp, pp.m_result);
					break;
				}
				case EKeyword::Wireframe:
				{
					pp.m_result.m_wireframe = reader.IsSectionEnd() ? true : reader.Bool();
					break;
				}
				case EKeyword::Font:
				{
					ParseFont(reader, pp, pp.m_font.back());
					break;
				}
				default:
				{
					// Save the current number of objects
					auto object_count = isize(pp.m_objects);

					// Assume the keyword is an object and start parsing
					if (!ParseLdrObject(static_cast<ELdrObject>(kw), reader, pp))
					{
						pp.ReportError(EParseError::UnknownKeyword, reader.Loc(), "Expected an object declaration");
						break;
					}

					assert("Objects removed but 'ParseLdrObject' didn't fail" && isize(pp.m_objects) > object_count);

					// Call the callback with the freshly minted object.
					add_cb(object_count);
					break;
				}
			}
		}
	}

	// Parse the ldr script in 'reader' adding the results to 'out'.
	// This function can be called from any thread (main or worker) and may be called concurrently by multiple threads.
	// There is synchronisation in the renderer for creating/allocating models. The calling thread must control the
	// lifetimes of the script reader, the parse output, and the 'store' container it refers to.
	ParseResult Parse(Renderer& rdr, IReader& reader, Guid const& context_id, std::stop_token stop_token)
	{
		ParseResult out;

		// Give initial and final progress updates
		auto exit = Scope<void>(
			[&] { !reader.Progress || reader.Progress(context_id, out, reader.Loc(), false); }, // Give an initial progress update
			[&] { !reader.Progress || reader.Progress(context_id, out, reader.Loc(), true); }); // Give a final progress update

		// Parse the script
		bool cancel = false;
		ParseParams pp(rdr, out, context_id, reader.ReportError, reader.Progress, cancel, std::move(stop_token));
		ParseLdrObjects(reader, pp, [&](int){});
		return out;
	}
	ParseResult Parse(Renderer& rdr, std::string_view ldr_script, Guid const& context_id, std::stop_token stop_token)
	{
		mem_istream<char> src{ ldr_script };
		rdr12::ldraw::TextReader reader(src, {});
		return Parse(rdr, reader, context_id, std::move(stop_token));
	}
	ParseResult Parse(Renderer& rdr, std::wstring_view ldr_script, Guid const& context_id, std::stop_token stop_token)
	{
		mem_istream<wchar_t> src{ ldr_script };
		rdr12::ldraw::TextReader reader(src, {});
		return Parse(rdr, reader, context_id, std::move(stop_token));
	}
	ParseResult Parse(Renderer& rdr, std::span<std::byte const> data, Guid const& context_id, std::stop_token stop_token)
	{
		mem_istream<char> src{ data.data(), data.size() };
		rdr12::ldraw::BinaryReader reader(src, {});
		return Parse(rdr, reader, context_id, std::move(stop_token));
	}
	ParseResult ParseFile(Renderer& rdr, std::filesystem::path ldr_filepath, Guid const& context_id, std::stop_token stop_token)
	{
		if (ldr_filepath.extension() == ".ldr")
		{
			std::ifstream src{ ldr_filepath };
			rdr12::ldraw::TextReader reader(src, ldr_filepath);
			return Parse(rdr, reader, context_id, std::move(stop_token));
		}
		if (ldr_filepath.extension() == ".bdr")
		{
			std::ifstream src{ ldr_filepath, std::ios::binary };
			rdr12::ldraw::BinaryReader reader(src, ldr_filepath);
			return Parse(rdr, reader, context_id, std::move(stop_token));
		}
		return {};
	}

	// Create an ldr object from creation data.
	LdrObjectPtr Create(Renderer& rdr, ELdrObject type, MeshCreationData const& cdata, Guid const& context_id)
	{
		LdrObjectPtr obj(new LdrObject(type, nullptr, context_id), true);

		// Create the model
		ResourceFactory factory(rdr);
		obj->m_model = ModelGenerator::Mesh(factory, cdata);
		obj->m_model->m_name = obj->TypeAndName();
		return obj;
	}

	// Create an ldr object from a p3d model.
	LdrObjectPtr CreateP3D(Renderer& rdr, ELdrObject type, std::filesystem::path const& p3d_filepath, Guid const& context_id)
	{
		LdrObjectPtr obj(new LdrObject(type, nullptr, context_id), true);

		struct ModelOut :ModelGenerator::IModelOut
		{
			LdrObject* m_obj;
			ModelOut(LdrObject* obj) :m_obj(obj) {}
			virtual EResult Model(ModelTree&& tree) override
			{
				ModelTreeToLdr(m_obj, tree);
				return EResult::Continue;
			}
		} model_out = { obj.get() };

		// Create the model
		ResourceFactory factory(rdr);
		std::ifstream src(p3d_filepath, std::ios::binary);
		ModelGenerator::LoadP3DModel(factory, src, model_out);
		return obj;
	}
	LdrObjectPtr CreateP3D(Renderer& rdr, ELdrObject type, std::span<std::byte const> p3d_data, Guid const& context_id)
	{
		LdrObjectPtr obj(new LdrObject(type, nullptr, context_id), true);

		struct ModelOut :ModelGenerator::IModelOut
		{
			LdrObject* m_obj;
			ModelOut(LdrObject* obj) :m_obj(obj) {}
			virtual EResult Model(ModelTree&& tree) override
			{
				ModelTreeToLdr(m_obj, tree);
				return EResult::Continue;
			}
		} model_out = { obj.get() };

		// Create the model
		ResourceFactory factory(rdr);
		mem_istream<char> src(p3d_data.data(), p3d_data.size());
		ModelGenerator::LoadP3DModel(factory, src, model_out);
		return obj;
	}

	// Create an instance of an existing ldr object.
	LdrObjectPtr CreateInstance(LdrObject const* existing)
	{
		LdrObjectPtr obj(new LdrObject(existing->m_type, nullptr, existing->m_context_id), true);

		// Use the same model
		obj->m_model = existing->m_model;
		obj->m_name = existing->m_name;
		obj->m_base_colour = existing->m_base_colour;

		// Recursively create instances of the child objects
		for (auto& child : existing->m_child)
			obj->m_child.push_back(CreateInstance(child.get()));

		return obj;
	}

	// Create an ldr object using a callback to populate the model data.
	// Objects created by this method will have dynamic usage and are suitable for updating every frame via the 'Edit' function.
	LdrObjectPtr CreateEditCB(Renderer& rdr, ELdrObject type, int vcount, int icount, int ncount, EditObjectCB edit_cb, Guid const& context_id)
	{
		LdrObjectPtr obj(new LdrObject(type, nullptr, context_id), true);

		// Create buffers for a dynamic model
		ModelDesc mdesc = ModelDesc()
			.vbuf(ResDesc::VBuf<Vert>(vcount, {}))
			.ibuf(ResDesc::IBuf<uint16_t>(icount, {}))
			.bbox(BBox::Reset())
			.name(obj->TypeAndName());

		// Create the model
		ResourceFactory factory(rdr);
		obj->m_model = factory.CreateModel(mdesc);

		// Create dummy nuggets
		NuggetDesc nug(ETopo::PointList, EGeom::Vert);
		nug.m_nflags = SetBits(nug.m_nflags, ENuggetFlag::RangesCanOverlap, true);
		for (int i = ncount; i-- != 0;)
			obj->m_model->CreateNugget(factory, nug);

		// Initialise it via the callback
		edit_cb(obj->m_model.get(), rdr);
		return obj;
	}

	// Modify the geometry of an LdrObject
	void Edit(Renderer& rdr, LdrObject* object, EditObjectCB edit_cb)
	{
		edit_cb(object->m_model.get(), rdr);
	}

	// Update 'object' with info from 'reader'. 'flags' describes the properties of 'object' to update
	void Update(Renderer& rdr, LdrObject* object, IReader& reader, EUpdateObject flags)
	{
		bool cancel = false;

		// Parsing parameters
		ParseResult result;
		ParseParams pp(rdr, result, object->m_context_id, reader.ReportError, reader.Progress, cancel);
	
		// Parse 'reader' for the new model
		ParseLdrObjects(reader, pp, [&](int object_index)
		{
			// Want the first root level object
			auto& rhs = result.m_objects[object_index];
			if (rhs->m_parent != nullptr)
				return true;

			// Swap the bits we want from 'rhs'
			// Note: we can't swap everything then copy back the bits we want to keep
			// because LdrObject is reference counted and isn't copyable. This is risky
			// though, if new members are added I'm bound to forget to consider them here :-/

			// RdrInstance
			if (AllSet(flags, EUpdateObject::Model))
			{
				std::swap(object->m_model, rhs->m_model);
				std::swap(object->m_sko, rhs->m_sko);
				std::swap(object->m_pso, rhs->m_pso);
				std::swap(object->m_iflags, rhs->m_iflags);
			}
			if (AllSet(flags, EUpdateObject::Transform))
				std::swap(object->m_i2w, rhs->m_i2w);
			if (AllSet(flags, EUpdateObject::Colour))
				std::swap(object->m_colour, rhs->m_colour);

			// LdrObject
			std::swap(object->m_type, rhs->m_type);
			if (AllSet(flags, EUpdateObject::Name))
				std::swap(object->m_name, rhs->m_name);
			if (AllSet(flags, EUpdateObject::Transform))
				std::swap(object->m_o2p, rhs->m_o2p);
			if (AllSet(flags, EUpdateObject::Flags))
				std::swap(object->m_flags_local, rhs->m_flags_local);
			if (AllSet(flags, EUpdateObject::Flags))
				std::swap(object->m_flags_recursive, rhs->m_flags_recursive);
			if (AllSet(flags, EUpdateObject::Animation))
				std::swap(object->m_root_anim, rhs->m_root_anim);
			if (AllSet(flags, EUpdateObject::GroupColour))
				std::swap(object->m_grp_colour, rhs->m_grp_colour);
			if (AllSet(flags, EUpdateObject::Reflectivity))
				std::swap(object->m_env, rhs->m_env);
			if (AllSet(flags, EUpdateObject::Colour))
				std::swap(object->m_base_colour, rhs->m_base_colour);

			// Transfer the child objects
			if (AllSet(flags, EUpdateObject::Children))
			{
				object->RemoveAllChildren();
				while (!rhs->m_child.empty())
				{
					auto child = rhs->RemoveChild(0);
					object->AddChild(child);
				}
			}
			else
				ApplyObjectState(object);

			// Only want one object
			return false;
		});
	}

	// Remove all objects from 'objects' that have a context id matching one in 'incl' and not in 'excl'
	// If 'incl' is empty, all are assumed included. If 'excl' is empty, none are assumed excluded.
	// 'excl' is considered after 'incl' so if any context ids are in both arrays, they will be excluded.
	void Remove(ObjectCont& objects, std::span<Guid const> incl, std::span<Guid const> excl)
	{
		erase_if_unstable(objects, [=](auto& ob)
		{
			if (!incl.empty() && !contains(incl, ob->m_context_id)) return false; // not in the doomed list
			if (!excl.empty() &&  contains(excl, ob->m_context_id)) return false; // saved by exclusion
			return true;
		});
	}

	// Remove 'obj' from 'objects'
	void Remove(ObjectCont& objects, LdrObject* obj)
	{
		erase_first_unstable(objects, [=](auto& ob){ return ob == obj; });
	}

	// Copy properties from 'src' to 'out' based on 'fields'
	void CopyCamera(Camera const& src, ECamField fields, Camera& out)
	{
		if (AllSet(fields, ECamField::C2W))
			out.CameraToWorld(src.CameraToWorld());
		if (AllSet(fields, ECamField::Focus))
			out.FocusDist(src.FocusDist());
		if (AllSet(fields, ECamField::Align))
			out.Align(src.Align());
		if (AllSet(fields, ECamField::Aspect))
			out.Aspect(src.Aspect());
		if (AllSet(fields, ECamField::FovY))
			out.FovY(src.FovY());
		if (AllSet(fields, ECamField::Near))
			out.Near(true, src.Near(true));
		if (AllSet(fields, ECamField::Far))
			out.Far(true, src.Far(true));
		if (AllSet(fields, ECamField::Ortho))
			out.Orthographic(src.Orthographic());
	}

	// Convert a model tree into a tree of LdrObjects
	void ModelTreeToLdr(LdrObject* root, std::span<ModelTreeNode const> tree)
	{
		if (tree.empty())
			return;

		// Count the number of roots.
		auto num_roots = pr::count_if(tree, [](ModelTreeNode const& m) { return m.m_level == 0; });
		if (num_roots == 0)
			throw std::runtime_error("Model tree has no roots");

		using Parent = struct { LdrObject* obj; int level; };
		pr::vector<Parent> ancestors;

		// Single root models have 'root' as the root.
		if (num_roots == 1)
		{
			root->m_model = tree[0].m_model;
			ancestors.push_back({ root, 0 });
			tree = tree.subspan<1>();
		}

		// Multi-root models have 'root' as dummy root (or Group)
		else
		{
			root->m_model = nullptr;
			ancestors.push_back({ root, -1 });
		}

		// Recurse
		for (auto& node : tree)
		{
			for (; node.m_level <= ancestors.back().level; )
				ancestors.pop_back();

			auto& parent = ancestors.back();

			// Create an LdrObject for each model
			LdrObjectPtr obj(new LdrObject(ELdrObject::Model, parent.obj, root->m_context_id), true);
			obj->m_name = node.m_name;
			obj->m_model = node.m_model;
			obj->m_o2p = node.m_o2p;

			// Add 'obj' as the current leaf node
			parent.obj->m_child.push_back(obj);
			ancestors.push_back({ obj.get(), node.m_level });
		}
	}

	// IReader ------------------------------------------------------------------------------------

	// Reads a transform accumulatively. 'o2w' must be a valid initial transform
	m4x4& IReader::Transform(m4x4& o2w)
	{
		assert(IsFinite(o2w) && "A valid 'o2w' must be passed to this function as it pre-multiplies the transform with the one read from the script");

		auto p2w = m4x4::Identity();
		auto affine = IsAffine(o2w);
		auto section = SectionScope();

		// Parse the transform
		for (EKeyword kw; NextKeyword(kw);)
		{
			switch (kw)
			{
				case EKeyword::M4x4:
				{
					auto m = Matrix4x4();
					if (m.w.w == 0 && m == m4x4::Zero())
					{
						ReportError(EParseError::InvalidValue, Loc(), "Invalid transform.");
						m = m4x4::Identity();
					}
					if (m.w.w != 1 && affine)
					{
						ReportError(EParseError::InvalidValue, Loc(), "Invalid transform. Specify 'NonAffine' if M4x4 is intentionally non-affine.");
						m = m4x4::Identity();
					}
					p2w = m * p2w;
					break;
				}
				case EKeyword::M3x3:
				{
					auto m = m4x4{ Matrix3x3(), v4::Origin() };
					p2w = m * p2w;
					break;
				}
				case EKeyword::Pos:
				{
					auto m = m4x4{ m3x4::Identity(), Vector3f().w1() };
					p2w = m * p2w;
					break;
				}
				case EKeyword::Align:
				{
					auto axis_id = pr::AxisId(Int<int>(10));
					auto direction = Vector3f().w0();

					v4 axis = axis_id;
					if (axis == v4::Zero())
					{
						ReportError(EParseError::InvalidValue, Loc(), "axis_id must one of \xc2\xb1""1, \xc2\xb1""2, \xc2\xb1""3");
						axis = v4::ZAxis();
					}

					p2w = m4x4::Transform(axis, direction, v4::Origin()) * p2w;
					break;
				}
				case EKeyword::LookAt:
				{
					auto point = Vector3f().w1();
					p2w = m4x4::LookAt(o2w.pos, point, o2w.y) * p2w;
					break;
				}
				case EKeyword::Quat:
				{
					auto v = Vector4f();
					auto q = quat(v.x, v.y, v.z, v.w);
					p2w = m4x4::Transform(q, v4::Origin()) * p2w;
					break;
				}
				case EKeyword::QuatPos:
				{
					auto v = Vector4f();
					auto q = quat(v.x, v.y, v.z, v.w);
					auto p = Vector3f().w1();
					p2w = m4x4::Transform(q, p) * p2w;
					break;
				}
				case EKeyword::Rand4x4:
				{
					auto centre = Vector3f().w1();
					auto radius = Real<float>();
					p2w = m4x4{Random<m3x4>(g_rng()), Random<v4>(g_rng(), centre, radius).w1()} * p2w;
					break;
				}
				case EKeyword::RandPos:
				{
					auto centre = Vector3f().w1();
					auto radius = Real<float>();
					p2w = m4x4::Translation(Random<v4>(g_rng(), centre, radius).w1()) * p2w;
					break;
				}
				case EKeyword::RandOri:
				{
					auto m = m4x4{Random<m3x4>(g_rng()), v4::Origin()};
					p2w = m * p2w;
					break;
				}
				case EKeyword::Euler:
				{
					auto angles = Vector3f().w0();
					p2w = m4x4::TransformDeg(angles.x, angles.y, angles.z, v4::Origin()) * p2w;
					break;
				}
				case EKeyword::Scale:
				{
					v4 scale = {};
					scale.x = Real<float>();
					scale.y = IsSectionEnd() ? scale.x : Real<float>();
					scale.z = IsSectionEnd() ? scale.y : Real<float>();
					p2w = m4x4::Scale(scale.x, scale.y, scale.z, v4::Origin()) * p2w;
					break;
				}
				case EKeyword::Transpose:
				{
					p2w = Transpose(p2w);
					break;
				}
				case EKeyword::Inverse:
				{
					p2w = IsOrthonormal(p2w) ? InvertOrthonormal(p2w) : Invert(p2w);
					break;
				}
				case EKeyword::Normalise:
				{
					p2w.x = Normalise(p2w.x);
					p2w.y = Normalise(p2w.y);
					p2w.z = Normalise(p2w.z);
					break;
				}
				case EKeyword::Orthonormalise:
				{
					p2w = Orthonorm(p2w);
					break;
				}
				case EKeyword::NonAffine:
				{
					affine = false;
					break;
				}
				default:
				{
					ReportError(EParseError::UnexpectedToken, Loc(), std::format("{} is not a valid Transform keyword", EKeyword_::ToStringA(kw)));
					break;
				}
			}
		}
		
		if (affine && !IsAffine(p2w))
		{
			ReportError(EParseError::UnexpectedToken, Loc(), "Transform is not affine. If non-affine is intended, use *NonAffine {}");
			p2w = m4x4::Identity();
		}

		// Pre-multiply the object to world transform
		o2w = p2w * o2w;
		return o2w;
	}

	// ParseResult ------------------------------------------------------------------------------------

	ParseResult::ParseResult()
		: m_objects()
		, m_lookup()
		, m_cam()
		, m_cam_fields()
		, m_wireframe()
	{
	}
	void ParseResult::reset()
	{
		m_objects.resize(0);
		m_lookup.clear();
		m_cam = {};
		m_cam_fields = {};
		m_wireframe = {};
	}
	size_t ParseResult::count() const
	{
		return m_objects.size();
	}
	LdrObjectPtr ParseResult::operator[](size_t index) const
	{
		return m_objects[index];
	}
	ParseResult& ParseResult::operator += (ParseResult const& rhs)
	{
		m_objects.insert(end(m_objects), begin(rhs.m_objects), end(rhs.m_objects));

		// The lookup maps names to objects, duplicate names will replace
		// earlier objects with the same name. It's up to the script writer
		// to prevent that if they need to refer to objects by name.
		for (auto& p : rhs.m_lookup)
			m_lookup[p.first] = p.second;

		m_commands.append(rhs.m_commands);

		CopyCamera(rhs.m_cam, rhs.m_cam_fields, m_cam);
		
		m_wireframe |= rhs.m_wireframe;

		return *this;
	}
	ParseResult::operator bool() const
	{
		return !m_objects.empty() || !m_lookup.empty() || !m_commands.empty();
	}
}