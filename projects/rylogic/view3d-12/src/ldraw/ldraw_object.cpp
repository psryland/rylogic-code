//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/model/animation.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/utility/diagnostics.h"
#include "pr/view3d-12/utility/pipe_state.h"
#include "pr/view3d-12/utility/update_resource.h"
#include "pr/view3d-12/utility/wrappers.h"

namespace pr::rdr12::ldraw
{
	#if PR_DBG
	#define PR_LDR_CALLSTACKS 0
	struct LeakedLdrObjects
	{
		struct RecentlyDeceased { string32 m_name; LdrObject const* ptr; };
		std::unordered_set<LdrObject const*> m_ldr_objects;
		std::deque<RecentlyDeceased> m_obituaries;
		std::string m_call_stacks;
		std::mutex m_mutex;

		LeakedLdrObjects()
			:m_ldr_objects()
			, m_mutex()
		{
		}
		~LeakedLdrObjects()
		{
			if (m_ldr_objects.empty()) return;

			size_t const msg_max_len = 1000;
			std::string msg = "Leaked LdrObjects detected:\n";
			for (auto ldr : m_ldr_objects)
			{
				msg.append(ldr->TypeAndName()).append("\n");
				if (msg.size() > msg_max_len)
				{
					msg.resize(msg_max_len - 3);
					msg.append("...");
					break;
				}
			}

			PR_ASSERT(1, m_ldr_objects.empty(), msg.c_str());
		}
		void add(LdrObject const* ldr)
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_ldr_objects.insert(ldr);
		}
		void remove(LdrObject const* ldr)
		{
			#if PR_LDR_CALLSTACKS
			#pragma message(PR_LINK "WARNING: ************************************************** PR_LDR_CALLSTACKS enabled")
			m_call_stacks.append(FmtS("[%p] %s\n", ldr, ldr->TypeAndName().c_str()));
			pr::DumpStack([&](auto& sym, auto& file, int line) { m_call_stacks.append(FmtS("%s(%d): %s\n", file.c_str(), line, sym.c_str())); }, 2U, 50U);
			m_call_stacks.append("\n");
			#endif

			std::lock_guard<std::mutex> lock(m_mutex);
			m_ldr_objects.erase(ldr);
			m_obituaries.push_front(RecentlyDeceased{ .m_name = ldr->m_name, .ptr = ldr });
			for (; m_obituaries.size() > 20; m_obituaries.pop_back()) {}
		}
		void check(LdrObject const* ldr)
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			if (m_ldr_objects.contains(ldr)) return;
			if (auto ded = std::ranges::find_if(m_obituaries, [=](RecentlyDeceased const& rd) { return rd.ptr == ldr; }); ded != end(m_obituaries))
				throw std::runtime_error(std::format("Use of recently deleted object {}", ded->m_name));
		}
	} g_ldr_object_tracker;
	#endif

	// Validate an ldr object pointer
	void Validate(LdrObject const* object)
	{
		if (object == nullptr)
			throw std::runtime_error("object pointer is null");

		#if PR_DBG
		g_ldr_object_tracker.check(object);
		#endif
	}

	LdrObject::LdrObject(ELdrObject type, LdrObject* parent, Guid const& context_id)
		: RdrInstance()
		, m_o2p(m4x4::Identity())
		, m_type(type)
		, m_parent(parent)
		, m_child() // Populated by the ParseParams object (it holds a reference as 'm_objects')
		, m_name()
		, m_context_id(context_id)
		, m_base_colour(Colour32White)
		, m_grp_colour()
		, m_root_anim()
		, m_bbox_instance()
		, m_screen_space()
		, m_flags_local(ELdrFlags::None)
		, m_flags_recursive(ELdrFlags::Invalidated)
		, m_user_data()
	{
		m_i2w = m4x4::Identity();
		m_colour = m_base_colour;
		PR_EXPAND(PR_DBG, g_ldr_object_tracker.add(this));
	}
	LdrObject::~LdrObject()
	{
		PR_EXPAND(PR_DBG, g_ldr_object_tracker.remove(this));
	}

	// Return the declaration name of this object
	string32 LdrObject::TypeAndName() const
	{
		return string32(ELdrObject_::ToStringA(m_type)) + " " + m_name;
	}

	// Return the full name (address) of this object
	string32 LdrObject::FullName() const
	{
		string32 parent = m_parent ? m_parent->FullName() : "";
		return parent.append(parent.empty() ? "" : ".").append(m_name);
	}

	
	// Add 'obj' recursively to the scene for renderering
	void LdrObject::AddToScene(Scene& scene, m4x4 const& p2w, ELdrFlags parent_flags)
	{
		// Notes:
		//  - The 'OnAddToScene' event can change 'm_i2w'. The local 'i2w' variable is used so that parenting is unaffected.
		//  - The decision to include or remove root motion for animated models is handled in the pose/animator. The 'i2w' is
		//    always the animation origin from the instance's point of view. That's why there is no 'if (m_pose) ...'
		//  - Simple root animation is just a modification to the 'i2w' transform, it's orthogonal to animation.
		//  - A model can have a 'm_m2root' transform that locates the model in root space. This is mainly for multi-part models.

		// Set the instance to world.
		auto i2w = p2w * m_o2p;
		if (m_root_anim) i2w *= m_root_anim.RootToAnim();
		if (m_model) i2w *= m_model->m_m2root;
		m_i2w = i2w;

		// Combine recursive flags
		auto flags = Flags() | (parent_flags & (ELdrFlags::Hidden | ELdrFlags::Wireframe | ELdrFlags::NonAffine | ELdrFlags::HideWhenNotAnimating));
		PR_ASSERT(PR_DBG, AllSet(flags, ELdrFlags::NonAffine) || IsAffine(m_i2w), "Invalid instance transform");

		// Allow the object to change it's transform just before rendering
		OnAddToScene(*this, scene);
		PR_ASSERT(PR_DBG, AllSet(flags, ELdrFlags::NonAffine) || IsAffine(m_i2w), "Invalid instance transform");

		// Decide if this object should be added to the scene
		auto is_hidden =
			AllSet(flags, ELdrFlags::Hidden) ||
			(AllSet(flags, ELdrFlags::Animated | ELdrFlags::HideWhenNotAnimating) && !IsAnimating());

		// Add the instance to the scene draw list
		if (m_model && !is_hidden)
		{
			// Could add occlusion culling here...
			scene.AddInstance(*this);
		}

		// Rinse and repeat for all children
		for (auto& child : m_child)
			child->AddToScene(scene, i2w, flags);
	}

	// Recursively add this object using 'bbox_model' instead of its
	// actual model, located and scaled to the transform and bbox of this object
	void LdrObject::AddBBoxToScene(Scene& scene, m4x4 const& p2w, ELdrFlags parent_flags)
	{
		// Set the instance to world for this object
		auto i2w = p2w * m_o2p;
		if (m_root_anim) i2w *= m_root_anim.RootToAnim();
		if (m_pose) i2w *= m_pose->RootToAnim(); // Apply the animation's root offset to the bounding box
		if (m_model) i2w *= m_model->m_m2root;

		// Combine recursive flags
		auto flags = Flags() | (parent_flags & (ELdrFlags::Hidden | ELdrFlags::Wireframe | ELdrFlags::NonAffine));
		PR_ASSERT(PR_DBG, AllSet(flags, ELdrFlags::NonAffine) || IsAffine(i2w), "Invalid instance transform");

		auto is_hidden =
			AnySet(flags, ELdrFlags::Hidden | ELdrFlags::SceneBoundsExclude) ||
			(AllSet(flags, ELdrFlags::Animated | ELdrFlags::HideWhenNotAnimating) && !IsAnimating());

		// Add the bbox instance to the scene draw list
		if (m_model && !is_hidden)
		{
			// Find the object to world for the bbox
			ResourceFactory factory(scene.rdr());
			m_bbox_instance.m_model = factory.CreateModel(EStockModel::BBoxModel);
			m_bbox_instance.m_i2w = i2w * BBoxTransform(m_model->m_bbox);
			scene.AddInstance(m_bbox_instance);
		}

		// Rinse and repeat for all children
		for (auto& child : m_child)
			child->AddBBoxToScene(scene, i2w, flags);
	}

	// Get the first child object of this object that matches 'name' (see Apply)
	LdrObject const* LdrObject::Child(char const* name) const
	{
		LdrObject const* obj = nullptr;
		Apply([&](LdrObject const* o) { obj = o; return false; }, name);
		return obj;
	}
	LdrObject* LdrObject::Child(char const* name)
	{
		return const_call(Child(name));
	}

	// Get a child object of this object by index
	LdrObject const* LdrObject::Child(int index) const
	{
		if (index < 0 || index >= isize(m_child)) throw std::runtime_error(std::format("LdrObject child index ({}) out of range [0,{})", index, isize(m_child)));
		return m_child[index].get();
	}
	LdrObject* LdrObject::Child(int index)
	{
		return const_call(Child(index));
	}

	// Get/Set the object to world transform of this object or the first child object matching 'name' (see Apply)
	m4x4 LdrObject::O2W(char const* name) const
	{
		auto obj = Child(name);
		if (obj == nullptr)
			return m4x4::Identity();

		// Combine parent transforms back to the root
		auto o2w = obj->m_o2p;
		for (auto p = obj->m_parent; p != nullptr; p = p->m_parent)
		{
			o2w = p->m_o2p * o2w;
		}

		return o2w;
	}
	void LdrObject::O2W(m4x4 const& o2w, char const* name)
	{
		Apply([&](LdrObject* o)
		{
			o->m_o2p = o->m_parent ? InvertOrthonormal(o->m_parent->O2W()) * o2w : o2w;
			assert(FEql(o->m_o2p.w.w, 1.0f) && "Invalid instance transform");
			return true;
		}, name);
	}

	// Get/Set the object to parent transform of this object or child objects matching 'name' (see Apply)
	m4x4 LdrObject::O2P(char const* name) const
	{
		auto obj = Child(name);
		return obj ? obj->m_o2p : m4x4::Identity();
	}
	void LdrObject::O2P(m4x4 const& o2p, char const* name)
	{
		assert(FEql(o2p.w.w, 1.0f) && "Invalid instance transform");
		assert(IsFinite(o2p) && "Invalid instance transform");
		Apply([&](LdrObject* o)
		{
			o->m_o2p = o2p;
			return true;
		}, name);
	}

	// Get/Set the animation time of this object or child objects matching 'name' (see Apply)
	float LdrObject::AnimTime(char const* name) const
	{
		auto obj = Child(name);
		return obj ? s_cast<float>(obj->m_root_anim.m_time_s) : 0.0f;
	}
	void LdrObject::AnimTime(float time_s, char const* name)
	{
		Apply([&](LdrObject* o)
		{
			// Set the time for the root animation
			o->m_root_anim.AnimTime(time_s);

			// Set the time for any skinned model animation
			if (o->m_pose)
				o->m_pose->AnimTime(time_s);

			return true;
		}, name);
	}

	// Get/Set the visibility of this object or child objects matching 'name' (see Apply)
	bool LdrObject::Visible(char const* name) const
	{
		auto obj = Child(name);
		return obj ? !AllSet(obj->Flags(), ELdrFlags::Hidden) : false;
	}
	void LdrObject::Visible(bool visible, char const* name)
	{
		Flags(ELdrFlags::Hidden, !visible, name);
	}

	// Get/Set the render mode for this object or child objects matching 'name' (see Apply)
	bool LdrObject::Wireframe(char const* name) const
	{
		auto obj = Child(name);
		return obj ? AllSet(obj->Flags(), ELdrFlags::Wireframe) : false;
	}
	void LdrObject::Wireframe(bool wireframe, char const* name)
	{
		Flags(ELdrFlags::Wireframe, wireframe, name);
	}

	// Get/Set the visibility of normals for this object or child objects matching 'name' (see Apply)
	bool LdrObject::Normals(char const* name) const
	{
		auto obj = Child(name);
		return obj ? AllSet(obj->Flags(), ELdrFlags::Normals) : false;
	}
	void LdrObject::Normals(bool show, char const* name)
	{
		Flags(ELdrFlags::Normals, show, name);
	}

	// Get/Set screen space rendering mode for this object and all child objects
	bool LdrObject::ScreenSpace() const
	{
		auto obj = Child("");
		return obj ? static_cast<bool>(obj->m_screen_space) : false;
	}
	void LdrObject::ScreenSpace(bool screen_space)
	{
		Apply([=](LdrObject* o)
		{
			if (screen_space)
			{
				static constexpr int ViewPortSize = 2;

				// Do not include in scene bounds calculations because we're scaling
				// this model at a point that the bounding box calculation can't see.
				o->Flags(ELdrFlags::SceneBoundsExclude, true);

				// Update the rendering 'i2w' transform on add-to-scene.
				o->m_screen_space = o->OnAddToScene += [](LdrObject& ob, Scene const& scene)
				{
					// The 'ob.m_i2w' is a normalised screen space position
					// (-1,-1,-0) is the lower left corner on the near plane,
					// (+1,+1,-1) is the upper right corner on the far plane.
					auto w = float(scene.m_viewport.Width);
					auto h = float(scene.m_viewport.Height);
					auto c2w = scene.m_cam.CameraToWorld();

					// Screen space uses a standard normalised orthographic projection
					ob.m_c2s = w >= h
						? m4x4::ProjectionOrthographic(float(ViewPortSize) * w / h, float(ViewPortSize), -0.01f, 1.01f, true)
						: m4x4::ProjectionOrthographic(float(ViewPortSize), float(ViewPortSize) * h / w, -0.01f, 1.01f, true);

					// Scale the object to normalised screen space
					auto scale = w >= h
						? m4x4::Scale(0.5f * ViewPortSize * (w / h), 0.5f * ViewPortSize, 1, v4::Origin())
						: m4x4::Scale(0.5f * ViewPortSize, 0.5f * ViewPortSize * (h / w), 1, v4::Origin());

					// Scale the X,Y position so that positions are still in normalised screen space
					ob.m_i2w.pos.x *= w >= h ? (w / h) : 1.0f;
					ob.m_i2w.pos.y *= w >= h ? 1.0f : (h / w);

					// Convert 'i2w', which is being interpreted as 'i2c', into an actual 'i2w'
					ob.m_i2w = c2w * ob.m_i2w * scale;
				};
			}
			else
			{
				o->m_c2s = m4x4::Zero();
				o->Flags(ELdrFlags::SceneBoundsExclude, false);
				o->OnAddToScene -= o->m_screen_space;
			}
			return true;
		}, "");
	}

	// Get/Set meta behaviour flags for this object or child objects matching 'name' (see Apply)
	ELdrFlags LdrObject::Flags(char const* name) const
	{
		// Mainly used to allow non-user objects to be added to a scene
		// and not affect the bounding box of the scene
		auto obj = Child(name);
		return obj ? obj->m_flags_local : ELdrFlags::None;
	}
	void LdrObject::Flags(ELdrFlags flags, bool state, char const* name)
	{
		Apply([=](LdrObject* o)
		{
			// Apply flag changes
			o->m_flags_local = SetBits(o->m_flags_local, flags, state);

			// Hidden
			if (o->m_model != nullptr)
			{
				// Ldraw doesn't add instances that are hidden. Don't set the nugget's
				// hidden flag, because hidden Ldraw objects may still be instanced.
			}

			// Wireframe
			if (AllSet(o->Flags(), ELdrFlags::Wireframe))
			{
				o->m_pso.Set<EPipeState::FillMode>(D3D12_FILL_MODE_WIREFRAME);
			}
			else
			{
				o->m_pso.Clear<EPipeState::FillMode>();
			}

			// No Z Test
			if (AllSet(o->Flags(), ELdrFlags::NoZTest))
			{
				// Don't test against Z, and draw above all objects
				o->m_pso.Set<EPipeState::DepthEnable>(FALSE);
				o->m_sko.Group(ESortGroup::PostAlpha);
			}
			else
			{
				o->m_pso.Set<EPipeState::DepthEnable>(TRUE);
				o->m_sko = SKOverride();
			}

			// If NoZWrite
			if (AllSet(o->Flags(), ELdrFlags::NoZWrite))
			{
				// Don't write to Z and draw behind all objects
				o->m_pso.Set<EPipeState::DepthWriteMask>(D3D12_DEPTH_WRITE_MASK_ZERO);
				o->m_sko.Group(ESortGroup::PreOpaques);
			}
			else
			{
				o->m_pso.Set<EPipeState::DepthWriteMask>(D3D12_DEPTH_WRITE_MASK_ALL);
				o->m_sko = SKOverride();
			}

			// Normals
			if (o->m_model != nullptr)
			{
				auto show_normals = AllSet(o->Flags(), ELdrFlags::Normals);
				ShowNormals(o->m_model.get(), show_normals);
			}

			// Shadow cast
			{
				auto vampire = AllSet(o->Flags(), ELdrFlags::ShadowCastExclude);
				o->m_iflags = SetBits(o->m_iflags, EInstFlag::ShadowCastExclude, vampire);
			}

			// Non-Affine
			{
				auto non_affine = AllSet(o->Flags(), ELdrFlags::NonAffine);
				o->m_iflags = SetBits(o->m_iflags, EInstFlag::NonAffine, non_affine);
			}

			// Animated
			if (AllSet(o->Flags(), ELdrFlags::Animated))
			{
				InvalidateRecursiveFlags();
			}
			return true;
		}, name);
	}

	// Flags that propagate to the parent (i.e., if the child has it, then I have it)
	ELdrFlags LdrObject::RecursiveFlags() const
	{
		// Lazy update recursive flags
		if (AllSet(m_flags_recursive, ELdrFlags::Invalidated))
		{
			// Read flags from children
			m_flags_recursive = ELdrFlags::None;
			for (auto const& child : m_child)
				m_flags_recursive = SetBits(m_flags_recursive, child->RecursiveFlags(), true);

			// Apply flags that propagate to the parent
			if (AllSet(Flags(), ELdrFlags::Animated))
			{
				m_flags_recursive = SetBits(m_flags_recursive, ELdrFlags::Animated, true);
			}
		}
		return m_flags_recursive;
	}
	void LdrObject::InvalidateRecursiveFlags()
	{
		// Invalidate all parent recursive flags
		for (auto p = this; p != nullptr; p = p->m_parent)
			p->m_flags_recursive = SetBits(p->m_flags_recursive, ELdrFlags::Invalidated, true);
	}

	// Get/Set the render group for this object or child objects matching 'name' (see Apply)
	ESortGroup LdrObject::SortGroup(char const* name) const
	{
		auto obj = Child(name);
		return obj ? obj->m_sko.Group() : ESortGroup::Default;
	}
	void LdrObject::SortGroup(ESortGroup grp, char const* name)
	{
		Apply([=](LdrObject* o)
		{
			o->m_sko.Group(grp);
			return true;
		}, name);
	}

	// Get/Set the nugget flags for this object or child objects matching 'name' (see Apply)
	ENuggetFlag LdrObject::NuggetFlags(char const* name, int index) const
	{
		auto obj = Child(name);
		if (obj == nullptr || obj->m_model == nullptr)
			return ENuggetFlag::None;

		if (index >= static_cast<int>(obj->m_model->m_nuggets.size()))
			throw std::runtime_error("nugget index out of range");

		auto nug = obj->m_model->m_nuggets.begin();
		for (int i = 0; i != index; ++i, ++nug) {}
		return nug->m_nflags;
	}
	void LdrObject::NuggetFlags(ENuggetFlag flags, bool state, char const* name, int index)
	{
		Apply([=](LdrObject* obj)
		{
			if (obj->m_model != nullptr)
			{
				auto nug = obj->m_model->m_nuggets.begin();
				for (int i = 0; i != index; ++i, ++nug) {}
				nug->m_nflags = SetBits(nug->m_nflags, flags, state);
			}
			return true;
		}, name);
	}

	// Get/Set the nugget flags for this object or child objects matching 'name' (see Apply)
	Colour32 LdrObject::NuggetTint(char const* name, int index) const
	{
		auto obj = Child(name);
		if (obj == nullptr || obj->m_model == nullptr)
			return Colour32White;

		if (index >= static_cast<int>(obj->m_model->m_nuggets.size()))
			throw std::runtime_error("nugget index out of range");

		auto nug = obj->m_model->m_nuggets.begin();
		for (int i = 0; i != index; ++i, ++nug) {}
		return nug->m_tint;
	}
	void LdrObject::NuggetTint(Colour32 tint, char const* name, int index)
	{
		Apply([=](LdrObject* obj)
		{
			if (obj->m_model != nullptr)
			{
				auto nug = obj->m_model->m_nuggets.begin();
				for (int i = 0; i != index; ++i, ++nug) {}
				nug->m_tint = tint;
			}
			return true;
		}, name);
	}

	// Get/Set the colour of this object or child objects matching 'name' (see Apply)
	// For 'Get', the colour of the first object to match 'name' is returned
	// For 'Set', the object base colour is not changed, only the tint colour
	Colour32 LdrObject::Colour(bool base_colour, char const* name) const
	{
		Colour32 col;
		Apply([&](LdrObject const* o)
		{
			col = base_colour ? o->m_base_colour : o->m_colour;
			return false; // stop at the first match
		}, name);
		return col;
	}
	void LdrObject::Colour(bool base_colour, Colour32 colour, char const* name, EColourOp op, float op_value)
	{
		Apply([=](LdrObject* o)
		{
			auto& obj_colour = base_colour ? o->m_base_colour : o->m_colour;
			switch (op)
			{
				case EColourOp::Overwrite:
					obj_colour = colour;
					break;
				case EColourOp::Add:
					obj_colour = o->m_base_colour + colour;
					break;
				case EColourOp::Subtract:
					obj_colour = o->m_base_colour - colour;
					break;
				case EColourOp::Multiply:
					obj_colour = o->m_base_colour * colour;
					break;
				case EColourOp::Lerp:
					obj_colour = Lerp(o->m_base_colour, colour, op_value);
					break;
			}

			// If the base colour was updated, update the instance colour as well
			if (base_colour)
				o->m_colour = o->m_base_colour;

			// Update the nugget alpha states
			if (o->m_model != nullptr)
			{
				auto tint_has_alpha = HasAlpha(o->m_colour);
				for (auto& nug : o->m_model->m_nuggets)
				{
					nug.m_nflags = SetBits(nug.m_nflags, ENuggetFlag::TintHasAlpha, tint_has_alpha);
					nug.UpdateAlphaStates();
				}
			}

			return true;
		}, name);
	}

	// Restore the colour to the initial colour for this object or child objects matching 'name' (see Apply)
	void LdrObject::ResetColour(char const* name)
	{
		Apply([=](LdrObject* o)
		{
			o->m_colour = o->m_base_colour;
			if (o->m_model == nullptr) return true;

			auto has_alpha = HasAlpha(o->m_colour);
			for (auto& nug : o->m_model->m_nuggets)
			{
				nug.m_nflags = SetBits(nug.m_nflags, ENuggetFlag::TintHasAlpha, has_alpha);
				nug.UpdateAlphaStates();
			}

			return true;
		}, name);
	}

	// Get/Set the reflectivity of this object or child objects matching 'name' (see Apply)
	float LdrObject::Reflectivity(char const* name) const
	{
		float env;
		Apply([&](LdrObject const* o)
		{
			env = o->m_env;
			return false; // stop at the first match
		}, name);
		return env;
	}
	void LdrObject::Reflectivity(float reflectivity, char const* name)
	{
		Apply([=](LdrObject* o)
		{
			o->m_env = reflectivity;
			return true;
		}, name);
	}

	// Set the texture on this object or child objects matching 'name' (see Apply).
	// Note for 'difference-mode' drawlist management: if the object is currently in one or more drawlists
	// (i.e. added to a scene) it will need to be removed and re-added so that the sort order is correct.
	void LdrObject::SetTexture(Texture2D* tex, char const* name)
	{
		Apply([=](LdrObject* o)
		{
			if (o->m_model == nullptr) return true;
			for (auto& nug : o->m_model->m_nuggets)
			{
				nug.m_tex_diffuse = Texture2DPtr(tex, true);
				nug.UpdateAlphaStates();
			}

			return true;
		}, name);
	}

	// Set the sampler on the nuggets of this object or child objects matching 'name' (see Apply).
	// Note for 'difference-mode' drawlist management: if the object is currently in one or more drawlists
	// (i.e. added to a scene) it will need to be removed and re-added so that the sort order is correct.
	void LdrObject::SetSampler(Sampler* sam, char const* name)
	{
		Apply([=](LdrObject* o)
		{
			if (o->m_model == nullptr) return true;
			for (auto& nug : o->m_model->m_nuggets)
			{
				nug.m_sam_diffuse = SamplerPtr(sam, true);
			}

			return true;
		}, name);
	}
	// Return the bounding box for this object in model space
	// To convert this to parent space multiply by 'm_o2p'
	// e.g. BBoxMS() for "*Box { 1 2 3 *o2w{*rand} }" will return bb.m_centre = origin, bb.m_radius = (1,2,3)
	BBox LdrObject::BBoxMS(EBBoxFlags bbox_flags, std::function<bool(LdrObject const&)> pred, m4x4 const* p2w_, ELdrFlags parent_flags) const
	{
		auto& p2w = p2w_ ? *p2w_ : m4x4::Identity();

		auto i2w = p2w;
		if (m_root_anim) i2w *= m_root_anim.RootToAnim();
		if (m_pose) i2w *= m_pose->RootToAnim();
		if (m_model) i2w = i2w * m_model->m_m2root;

		// Combine recursive flags
		auto ldr_flags = Flags() | (parent_flags & (ELdrFlags::BBoxExclude | ELdrFlags::NonAffine));

		// Start with the bbox for this object
		auto bbox = BBox::Reset();
		if (m_model != nullptr && !AnySet(ldr_flags, ELdrFlags::BBoxExclude) && pred(*this)) // Get the bbox from the graphics model
		{
			if (m_model->m_bbox.valid())
			{
				if (IsAffine(i2w))
					Grow(bbox, i2w * m_model->m_bbox);
				else
					Grow(bbox, MulNonAffine(i2w, m_model->m_bbox));
			}
		}
		if (AllSet(bbox_flags, EBBoxFlags::IncludeChildren)) // Add the bounding boxes of the children
		{
			for (auto& child : m_child)
			{
				auto c2w = i2w * child->m_o2p;
				auto cbbox = child->BBoxMS(bbox_flags, pred, &c2w, ldr_flags);
				if (cbbox.valid())
					Grow(bbox, cbbox);
			}
		}
		return bbox;
	}
	BBox LdrObject::BBoxMS(EBBoxFlags flags) const
	{
		return BBoxMS(flags, [](LdrObject const&) { return true; });
	}

	// Return the bounding box for this object in world space.
	// If this is a top level object, this will be equivalent to 'm_o2p * BBoxMS()'
	// If not then, then the returned bbox will be transformed to the top level object space
	BBox LdrObject::BBoxWS(EBBoxFlags flags, std::function<bool(LdrObject const&)> pred) const
	{
		// Get the o2w for this object
		m4x4 o2w = m_o2p;
		for (LdrObject* parent = m_parent; parent; parent = parent->m_parent)
		{
			auto o2p = parent->m_o2p;
			if (parent->m_root_anim) o2p *= parent->m_root_anim.RootToAnim();
			if (parent->m_pose) o2p *= parent->m_pose->RootToAnim();
			o2w = o2p * o2w; //??
		}

		return BBoxMS(flags, pred, &o2w);
	}
	BBox LdrObject::BBoxWS(EBBoxFlags flags) const
	{
		return BBoxWS(flags, [](LdrObject const&) { return true; });
	}

	// Add/Remove 'child' as a child of this object
	void LdrObject::AddChild(LdrObjectPtr& child)
	{
		PR_ASSERT(PR_DBG, child->m_parent != this, "child is already a child of this object");
		PR_ASSERT(PR_DBG, child->m_parent == nullptr, "child already has a parent");
		child->m_parent = this;
		m_child.push_back(child);
	}
	LdrObjectPtr LdrObject::RemoveChild(LdrObjectPtr& child)
	{
		PR_ASSERT(PR_DBG, child->m_parent == this, "child is not a child of this object");
		auto idx = index_of(m_child, child);
		return RemoveChild(idx);
	}
	LdrObjectPtr LdrObject::RemoveChild(size_t i)
	{
		PR_ASSERT(PR_DBG, i >= 0 && i < m_child.size(), "child index out of range");
		auto child = m_child[i];
		m_child.erase(std::begin(m_child) + i);
		child->m_parent = nullptr;
		return child;
	}
	void LdrObject::RemoveAllChildren()
	{
		while (!m_child.empty())
			RemoveChild(0);
	}

	// True if this object or any child objects are currently animating
	bool LdrObject::IsAnimating() const
	{
		if (!AllSet(RecursiveFlags(), ELdrFlags::Animated))
			return false;
		
		return Apply([](LdrObject const* obj)
		{
			if (obj->m_root_anim)
				return false; // stop, found an animating object

			if (obj->m_pose && obj->m_pose->IsAnimating())
				return false; // stop, found an animating object

			return true; // continue
		}, "") == false;
	}

	// Called when there are no more references to this object
	void LdrObject::RefCountZero(RefCount<LdrObject>* doomed)
	{
		delete static_cast<LdrObject*>(doomed);
	}
	long LdrObject::AddRef() const
	{
		return RefCount<LdrObject>::AddRef();
	}
	long LdrObject::Release() const
	{
		return RefCount<LdrObject>::Release();
	}
}
