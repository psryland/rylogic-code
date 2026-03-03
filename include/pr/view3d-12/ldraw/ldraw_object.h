//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/animation.h"
#include "pr/view3d-12/model/animator.h"

namespace pr::rdr12::ldraw
{
	// An instance type for line drawer stock objects
	struct StockInstance
	{
		#define PR_RDR_INST(x)\
		x(m4x4     ,m_i2w   ,EInstComp::I2WTransform)\
		x(ModelPtr ,m_model ,EInstComp::ModelPtr    )
		PR_RDR12_INSTANCE_MEMBERS(StockInstance, PR_RDR_INST);
		#undef PR_RDR_INST
	};

	// An instance type for object bounding boxes
	struct BBoxInstance
	{
		#define PR_RDR_INST(x)\
		x(m4x4     ,m_i2w   ,EInstComp::I2WTransform   )\
		x(ModelPtr ,m_model ,EInstComp::ModelPtr       )
		PR_RDR12_INSTANCE_MEMBERS(BBoxInstance, PR_RDR_INST);
		#undef PR_RDR_INST
	};

	// The base instance type for LdrObjects
	struct RdrInstance
	{
		// Note: don't use 'm_i2w' to control the object transform, use m_o2p in the LdrObject instead
		#define PR_RDR_INST(x) \
		x(m4x4       ,m_i2w    ,EInstComp::I2WTransform       )/*     16 bytes, align 16 */\
		x(m4x4       ,m_c2s    ,EInstComp::C2SOptional        )/*     16 bytes, align 16 */\
		x(ModelPtr   ,m_model  ,EInstComp::ModelPtr           )/* 4 or 8 bytes, align 8 */\
		x(PosePtr    ,m_pose   ,EInstComp::PosePtr            )/* 4 or 8 bytes, align 8 */\
	 	x(PipeStates ,m_pso    ,EInstComp::PipeStates         )/*    104 bytes, align 8 */\
		x(Colour32   ,m_colour ,EInstComp::TintColour32       )/*      4 bytes, align 4 */\
		x(float      ,m_env    ,EInstComp::EnvMapReflectivity )/*      4 bytes, align 4 */\
		x(EInstFlag  ,m_iflags ,EInstComp::Flags              )/*      4 bytes, align 4 */\
		x(SKOverride ,m_sko    ,EInstComp::SortkeyOverride    )/*      8 bytes, align 4 */
		PR_RDR12_INSTANCE_MEMBERS(RdrInstance , PR_RDR_INST);
		#undef PR_RDR_INST
	};
	
	// Root animation
	struct RootAnimation
	{
		// Notes:
		//  - This type is akin to the 'Pose' type.
		//    It's a runtime instance of the simple animation data
		//    Each LdrObject has one of these with it's own time value

		RootAnimationPtr m_simple = {};
		double m_time_s = {};

		// Set the animation time
		void AnimTime(double time_s)
		{
			m_time_s = time_s;
		}

		// Returns the root relative to the origin (i.e., the root motion) at the given time
		m4x4 RootToAnim(double time_s) const
		{
			return m_simple ? m_simple->EvaluateAtTime(time_s) : m4x4::Identity();
		}
		m4x4 RootToAnim() const
		{
			return RootToAnim(m_time_s);
		}

		// Has animation?
		explicit operator bool() const
		{
			return m_simple != nullptr;
		}
	};

	// An LDraw object
	struct LdrObject
		:RdrInstance
		,RefCounted<LdrObject>
	{
		// Notes:
		//  - Try not to use the RdrInstance members for things other than rendering
		//    they can temporarily have different models/transforms/etc during rendering of
		//    object bounding boxes etc.
		m4x4              m_o2p;             // Object to parent transform (or object to world if this is a top level object)
		ELdrObject        m_type;            // Object type
		LdrObject*        m_parent;          // The parent of this object, nullptr for top level instances.
		ObjectCont        m_child;           // A container of pointers to child instances
		string32          m_name;            // A name for the object
		GUID              m_context_id;      // The id of the context this instance was created in
		Colour32          m_base_colour;     // The original colour of this object
		Colour32          m_grp_colour;      // A Colour multiplier that is applied to all child objects
		RootAnimation     m_root_anim;       // Animation of the model root position
		BBoxInstance      m_bbox_instance;   // Used for rendering the bounding box for this instance
		Sub               m_screen_space;    // True if this object should be rendered in screen space
		ELdrFlags         m_flags_local;     // Property flags controlling meta behaviour of this object only. DON'T USE DIRECTLY, use Flags()
		mutable ELdrFlags m_flags_recursive; // Recursive property flags of this object and all children. DON'T USE DIRECTLY, use RecursiveFlags()
		UserData          m_user_data;       // User data

		LdrObject(ELdrObject type, LdrObject* parent, Guid const& context_id);
		~LdrObject();

		// Return the type and name of this object
		string32 TypeAndName() const;

		// Return the full name (address) of this object
		string32 FullName() const;

		// Called just prior to this object being added to a scene.
		// Allows handlers to change the object's 'i2w' transform, visibility, etc.
		EventHandler<LdrObject&, Scene const&, true> OnAddToScene;

		// Recursively add this object and its children to a scene
		void AddToScene(Scene& scene, m4x4 const& p2w = m4x4::Identity(), ELdrFlags parent_flags = ELdrFlags::None);

		// Recursively add the bounding box instance for this object using 'bbox_model'
		// located and scaled to the transform and box of this object
		void AddBBoxToScene(Scene& scene, m4x4 const& p2w = m4x4::Identity(), ELdrFlags parent_flags = ELdrFlags::None);

		// Apply an operation on this object or any of its child objects that match 'name'.
		// 'func' should have a signature: 'bool func(LdrObject* obj);' returning false to 'quick-out'.
		// 'name' == nullptr: apply to this object only,
		// 'name' == "": apply to this object and all children recursively.
		// 'name' == "foo": apply to all child objects named 'foo'.
		// 'name' == "#regex": apply to all child objects whose name matches the regex.
		// 'obj' is a recursion parameter, callers should use 'nullptr'
		// Returns 'true' if 'func' always returns 'true'.
		template <typename TFunc> bool Apply(TFunc func, char const* name = nullptr, LdrObject* obj = nullptr) const
		{
			if (obj == nullptr)
			{
				// The 'const-ness' of this function depends on 'func'
				obj = const_cast<LdrObject*>(this);
			}
			if (name == nullptr)
			{
				if (!func(obj))
					return false;
			}
			else if (name[0] == '\0')
			{
				if (!func(obj)) return false;
				for (auto& child : obj->m_child)
					if (!Apply(func, name, child.m_ptr))
						return false;
			}
			else
			{
				if ((name[0] == '#' && std::regex_match(std::string(obj->m_name.c_str()), std::regex(&name[1]))) || obj->m_name == name)
				{
					if (!func(obj))
						return false;
				}
				for (auto& child : obj->m_child)
				{
					if (!Apply(func, name, child.m_ptr))
						return false;
				}
			}
			return true;
		}

		// Get the first child object of this object that matches 'name' (see Apply)
		LdrObject const* Child(char const* name) const;
		LdrObject* Child(char const* name);

		// Get a child object of this object by index
		LdrObject const* Child(int index) const;
		LdrObject* Child(int index);

		// Notes:
		//  - O2W,O2P are for unanimated object positions. This is because get/set should be symmetric.
		//  - To find the animated O2W/O2P at an animation time, access the 'm_root_anim' and 'm_pose' members.
		//    'm_root_anim' can be simply applied to the o2p transform, (see AddToScene() for reference).
		//    Use 'O2W() * m_root_anim.RootToWorld() * m_pose->RootToAnim()' to find the animated root bone position.

		// Get/Set the object to world transform of this object or the first child object matching 'name' (see Apply)
		// Notes: It is more efficient to set O2P.
		m4x4 O2W(char const* name = nullptr) const;
		void O2W(m4x4 const& o2w, char const* name = nullptr);

		// Get/Set the object to parent transform of this object or child objects matching 'name' (see Apply)
		m4x4 O2P(char const* name = nullptr) const;
		void O2P(m4x4 const& o2p, char const* name = nullptr);

		// Get/Set the animation time of this object or child objects matching 'name' (see Apply)
		float AnimTime(char const* name = nullptr) const;
		void AnimTime(float time_s, char const* name = nullptr);

		// Get/Set the visibility of this object or child objects matching 'name' (see Apply)
		bool Visible(char const* name = nullptr) const;
		void Visible(bool visible, char const* name = nullptr);

		// Get/Set the render mode for this object or child objects matching 'name' (see Apply)
		bool Wireframe(char const* name = nullptr) const;
		void Wireframe(bool wireframe, char const* name = nullptr);

		// Get/Set the visibility of normals for this object or child objects matching 'name' (see Apply)
		bool Normals(char const* name = nullptr) const;
		void Normals(bool wireframe, char const* name = nullptr);

		// Get/Set screen space rendering mode for this object (and all child objects)
		bool ScreenSpace() const;
		void ScreenSpace(bool screen_space3);

		// Get/Set meta behaviour flags for this object or child objects matching 'name' (see Apply)
		ELdrFlags Flags(char const* name = nullptr) const;
		void Flags(ELdrFlags flags, bool state, char const* name = nullptr);

		// Flags that propagate to the parent (i.e., if the child has it, then I have it)
		ELdrFlags RecursiveFlags() const;
		void InvalidateRecursiveFlags();

		// Get/Set the render group for this object or child objects matching 'name' (see Apply)
		ESortGroup SortGroup(char const* name = nullptr) const;
		void SortGroup(ESortGroup grp, char const* name = nullptr);

		// Get/Set the nugget flags for this object or child objects matching 'name' (see Apply)
		ENuggetFlag NuggetFlags(char const* name, int index) const;
		void NuggetFlags(ENuggetFlag flags, bool state, char const* name, int index);

		// Get/Set the nugget tint colour for this object or child objects matching 'name' (see Apply)
		Colour32 NuggetTint(char const* name, int index) const;
		void NuggetTint(Colour32 tint, char const* name, int index);

		// Get/Set the colour of this object or child objects matching 'name' (see Apply)
		// For 'Get', the colour of the first object to match 'name' is returned
		// For 'Set', the object base colour is not changed, only the instance colour
		Colour32 Colour(bool base_colour, char const* name = nullptr) const;
		void Colour(bool base_colour, Colour32 colour, char const* name = nullptr, EColourOp op = EColourOp::Overwrite, float op_value = 0.0f);

		// Restore the colour to the initial colour for this object or child objects matching 'name' (see Apply)
		void ResetColour(char const* name = nullptr);

		// Get/Set the reflectivity of this object or child objects matching 'name' (see Apply)
		float Reflectivity(char const* name = nullptr) const;
		void Reflectivity(float reflectivity, char const* name = nullptr);

		// Set the texture on this object or child objects matching 'name' (see Apply).
		// Note for difference mode drawlist management, if the object is currently in one or more drawlists
		// (i.e. added to a scene) it will need to be removed and re-added so that the sort order is correct.
		void SetTexture(Texture2D* tex, char const* name = nullptr);

		// Set the sampler on the nuggets of this object or child objects matching 'name' (see Apply).
		// Note for 'difference-mode' drawlist management: if the object is currently in one or more drawlists
		// (i.e. added to a scene) it will need to be removed and re-added so that the sort order is correct.
		void SetSampler(Sampler* sam, char const* name = nullptr);

		// Return the bounding box for this object in model space
		// To convert this to parent space multiply by 'm_o2p'
		// e.g. BBoxMS() for "*Box { 1 2 3 *o2w{*rand} }" will return bb.m_centre = origin, bb.m_radius = (1,2,3)
		BBox BBoxMS(EBBoxFlags bbox_flags, std::function<bool(LdrObject const&)> pred, m4x4 const* p2w = nullptr, ELdrFlags parent_flags = ELdrFlags::None) const;
		BBox BBoxMS(EBBoxFlags bbox_flags) const;

		// Return the bounding box for this object in world space.
		// If this is a top level object, this will be equivalent to 'm_o2p * BBoxMS()'
		// If not then, then the returned bbox will be transformed to the top level object space
		BBox BBoxWS(EBBoxFlags bbox_flags, std::function<bool(LdrObject const&)> pred) const;
		BBox BBoxWS(EBBoxFlags bbox_flags) const;

		// Add/Remove 'child' as a child of this object
		void AddChild(LdrObjectPtr& child);
		LdrObjectPtr RemoveChild(LdrObjectPtr& child);
		LdrObjectPtr RemoveChild(size_t i);
		void RemoveAllChildren();

		// True if this object or any child objects are currently animating
		bool IsAnimating() const;

		// Predicate for matching this object by context id
		struct MatchId
		{
			GUID m_id;
			MatchId(GUID const& id) :m_id(id) {}
			bool operator()(LdrObject const& obj) const { return obj.m_context_id == m_id; }
			bool operator()(LdrObject const* obj) const { return obj && obj->m_context_id == m_id; }
		};

		// Called when there are no more references to this object
		static void RefCountZero(RefCount<LdrObject>* doomed);
		long AddRef() const;
		long Release() const;
	};

	// Validate an ldr object pointer
	void Validate(LdrObject const* object);
}
