//*****************************************
//*****************************************
// Bouncing Sphere
// 1) Create a static box
// 2) Create a dynamic sphere
// 3) Add both to a brute force broadphase
// 4) 
#include "test.h"
#include "pr/common/stdvector.h"
#include "pr/common/fmt.h"
#include "pr/common/console.h"
#include "pr/common/testbed3d.h"
#include "pr/common/timers.h"
#include "pr/maths/stringconversion.h"
#include "pr/linedrawer/ldr_helper2.h"


#include "pr/geometry/primitive.h"
#include "pr/geometry/mesh_tools.h"
#include "pr/physics/physics.h"

extern int max_loop_count;
extern int refine_edge_count;
extern int get_opposing_edge_count;

namespace TestPhysics
{
	using namespace pr;
	using namespace pr::ph;
	const char scene_script[] =
		"										\
		*Window									\
		{										\
			*Bounds 0 0 900 900					\
			*ClientArea 0 0 900 900				\
			*BackColour FF3000A0				\
		}										\
		*Viewport								\
		{										\
			*Rect 0.0 0.0 1.0 1.0				\
		}										\
		*Camera									\
		{										\
			*Position 0 1 20					\
			*LookAt 0 0 0						\
			*Up 0 1 0							\
			*NearPlane 0.1						\
			*FarPlane 1000.0 					\
			*FOV 0.785398						\
			*Aspect 1							\
		}										\
		*CameraController						\
		{										\
			*Keyboard							\
			*LinAccel 0.2						\
			*MaxLinVel 1000.0					\
			*RotAccel 0.03						\
			*MaxRotVel 20.0						\
			*Scale 1							\
		}										\
		*Light									\
		{										\
			*Ambient 0.1 0.1 0.1 0.0			\
			*Diffuse 1.0 1.0 1.0 1.0			\
			*Specular 0.2 0.2 0.2 0.0			\
			*SpecularPower 100.0				\
			*Direction -1.0 -2.0 -2.0			\
		}										\
		*Light									\
		{										\
			*Ambient 0.1 0.1 0.1 0.0			\
			*Diffuse 1.0 0.0 0.0 1.0			\
			*Specular 0.2 0.2 0.2 0.0			\
			*SpecularPower 100.0				\
			*Direction 1.0 -2.0 2.0				\
		}										\
		";

	// ********************************************************************
	// A Renderer instance 
	PR_RDR_DECLARE_INSTANCE_TYPE4
	(
		Instance, 
		pr::rdr::Model*, m_model,             pr::rdr::instance::ECpt_ModelPtr,
		pr::m4x4*,       m_instance_to_world, pr::rdr::instance::ECpt_I2WTransformPtr,
		pr::Colour32,    m_colour,            pr::rdr::instance::ECpt_TintColour32,
		rdr::rs::Block,  m_render_state,      pr::rdr::instance::ECpt_RenderState
	);

	// My first physics objects
	struct Thing
	{
		Thing()
		{
			m_i2w.identity();
			m_graphic_inst.m_instance_to_world = &m_i2w;
			m_shadow_i2w.identity();
			m_shadow_i2w.y.y = 0.0f;
			m_shadow_inst.m_instance_to_world = &m_shadow_i2w;
		}
		m4x4				m_i2w;
		Instance			m_graphic_inst;
		m4x4				m_shadow_i2w;
		Instance			m_shadow_inst;

		// Physics members
		ph::Rigidbody		m_physics_inst;		// Shape and motion properties of the physics object
	};

	// ********************************************************************

	class Main
	{
	public:
		Main();
		void Run();
	
	private:
		ph::Settings GetPhysicsEngineSettings();

	private:
		typedef std::list<TBinaryData> TModelData;
		TestBed3d			m_3dtestbed;
		TModelData			m_model_data;		// Storage for the rigid body data

		ph::Engine			m_engine;			// This is the physics world
		ph::BPBruteForce	m_broadphase;		// The broadphase system to use
		Thing				m_ball0;			// A dynamic physics object
		Thing				m_ball1;			// A dynamic physics object
		ph::TerrainPlane	m_terrain;			// The terrain system for the world
		Thing				m_ground;			// The ground plane
	};

	//*****
	// Return the settings to use for the physics engine
	ph::Settings Main::GetPhysicsEngineSettings()
	{
		ph::Settings settings;
		settings.m_broadphase	= &m_broadphase;
		settings.m_terrain		= &m_terrain;
		return settings;
	}

	//*****
	Main::Main()
	:m_3dtestbed(scene_script)
	,m_engine(GetPhysicsEngineSettings())
	{
		// Create a dynamic sphere and add it to the engine
		{
			using namespace pr::geom::unit_sphere;
			float sphere_radius = 1.0f;

			// Mass properties for the shape
			MassProperties mp;
			v4 model_to_inertial;
			TBinaryData& model_data = (m_model_data.push_back(TBinaryData()), m_model_data.back());

			// Create a shape builder to create the collision shape with it
			// The is serialised into a TBinaryData buffer
			ShapeBuilder shape_builder;
			shape_builder.AddShape(ShapeSphere::make(1.0f, m4x4Identity, 0, 0));
			Shape* shape = shape_builder.BuildShape(model_data, mp, model_to_inertial, EShapeHierarchy_Single);
			
			// This object contains the data we need to make a rigidbody
			RigidbodySettings rb_settings;
			rb_settings.m_shape = shape;
			rb_settings.m_mass_properties = mp;
			
			rb_settings.m_object_to_world = m_ball0.m_i2w;
			rb_settings.m_object_to_world.pos.x = -5.0f;
			rb_settings.m_object_to_world.pos.y = 5.0f + sphere_radius; // Should hit the ground in 1 sec. S = So + VoT + 0.5AT^2
			rb_settings.m_lin_velocity.set(2.0f, 0.0f, 0.0f, 0.0f);
			rb_settings.m_ang_velocity.set(0.0f, maths::pi, 0.0f, 0.0f);
		
			// Construct a rigid body
			m_ball0.m_physics_inst.Create(rb_settings);

			rb_settings.m_object_to_world = m_ball1.m_i2w;
			rb_settings.m_object_to_world.pos.x = 5.0f;
			rb_settings.m_object_to_world.pos.y = 5.0f + sphere_radius; // Should hit the ground in 1 sec. S = So + VoT + 0.5AT^2
			rb_settings.m_lin_velocity.set(-2.0f, 0.0f, 0.0f, 0.0f);
			rb_settings.m_ang_velocity.set( 0.0f, maths::pi, 0.0f, 0.0f);

			// Construct a rigid body
			m_ball1.m_physics_inst.Create(rb_settings);

			// Add the rigid body to the physics engine
			m_engine.Register(m_ball0.m_physics_inst);
			//m_engine.Register(m_ball1.m_physics_inst);

			// Create 3d testbed geometry for the sphere
			m_ball0.m_graphic_inst.m_model = m_3dtestbed.CreateModel(num_vertices, vertices, num_indices, indices, Scale4x4(sphere_radius, v4Origin));
			m_ball0.m_graphic_inst.m_colour.set(0.6f, 0.0f, 0.0f, 1.0f);
			m_ball0.m_graphic_inst.m_render_state.SetRenderState(D3DRS_FILLMODE, D3DFILL_WIREFRAME);
			m_ball0.m_shadow_inst.m_model = m_ball0.m_graphic_inst.m_model;
			m_ball0.m_shadow_inst.m_colour.set(0.0f, 0.0f, 0.0f, 1.0f);

			m_ball1.m_graphic_inst.m_model = m_3dtestbed.CreateModel(num_vertices, vertices, num_indices, indices, Scale4x4(sphere_radius, v4Origin));
			m_ball1.m_graphic_inst.m_colour.set(0.6f, 0.0f, 0.0f, 1.0f);
			m_ball1.m_graphic_inst.m_render_state.SetRenderState(D3DRS_FILLMODE, D3DFILL_WIREFRAME);
			m_ball1.m_shadow_inst.m_model = m_ball1.m_graphic_inst.m_model;
			m_ball1.m_shadow_inst.m_colour.set(0.0f, 0.0f, 0.0f, 1.0f);

			// Add the graphic instance to the 3d test bed
			m_3dtestbed.AddInstance(m_ball0.m_graphic_inst);
			m_3dtestbed.AddInstance(m_ball0.m_shadow_inst);

			m_3dtestbed.AddInstance(m_ball1.m_graphic_inst);
			m_3dtestbed.AddInstance(m_ball1.m_shadow_inst);
		}

		// Create the ground plane
		{
			using namespace pr::geom::unit_plane;
			m_ground.m_graphic_inst.m_model = m_3dtestbed.CreateModel(num_vertices, vertices, num_indices, indices, Rotation4x4(v4XAxis, -maths::pi_by_2, v4Origin) * Scale4x4(100.0f, v4Origin));
			m_ground.m_graphic_inst.m_colour.set(0.0f, 0.6f, 0.0f, 1.0f);
			m_3dtestbed.AddInstance(m_ground.m_graphic_inst);
		}
	}

	void Main::Run()
	{
		float now = 0.0f;
		float step_time = 1.0f / 60.0f;
		pr::Console cons;
		while( GetAsyncKeyState(VK_ESCAPE) == 0 )
		{
			m_3dtestbed.ReadInput();
			float power = 0.4f;
			if( GetAsyncKeyState(VK_SHIFT) != 0 ) power = 2.0f;
			if( GetAsyncKeyState('S')      != 0 ) m_ball0.m_physics_inst.ApplyWSTwist  (v4::make( 0.00f, 0.0f,  power, 0.0f));
			if( GetAsyncKeyState('F')      != 0 ) m_ball0.m_physics_inst.ApplyWSTwist  (v4::make( 0.00f, 0.0f, -power, 0.0f));
			if( GetAsyncKeyState('W')      != 0 ) m_ball0.m_physics_inst.ApplyWSTwist  (v4::make( 0.0f, -power, 0.00f, 0.0f));
			if( GetAsyncKeyState('R')      != 0 ) m_ball0.m_physics_inst.ApplyWSTwist  (v4::make( 0.00f, power, 0.00f, 0.0f));
			if( GetAsyncKeyState('E')      != 0 ) m_ball0.m_physics_inst.ApplyWSTwist  (v4::make(-power, 0.00f, 0.00f, 0.0f));
			if( GetAsyncKeyState('D')      != 0 ) m_ball0.m_physics_inst.ApplyWSTwist  (v4::make( power, 0.00f, 0.00f, 0.0f));
			if( GetAsyncKeyState(VK_SPACE) != 0 ) m_ball0.m_physics_inst.ApplyWSImpulse(v4::make(0.0f, power * 30.0f, 0.0f, 0.0f));

			//m_3dtestbed.GetCamera().LookAt(m_ball0.m_i2w.pos);
			//m_ground.m_i2w.pos.x = m_3dtestbed.GetCamera().GetPosition().x;
			//m_ground.m_i2w.pos.z = m_3dtestbed.GetCamera().GetPosition().z;
			m_ball0.m_shadow_i2w.pos = m_ball0.m_i2w.pos;
			m_ball0.m_shadow_i2w.pos.y = 0.01f;
			m_ball1.m_shadow_i2w.pos = m_ball1.m_i2w.pos;
			m_ball1.m_shadow_i2w.pos.y = 0.01f;
			m_3dtestbed.Present();
			
			// Info
			v4 lin_vel = m_ball0.m_physics_inst.Velocity();
			v4 ang_vel = m_ball0.m_physics_inst.AngVelocity();
			cons.Write(0, 0, Fmt("Time: %2.2f    ", now));
			cons.Write(0, 1, Fmt("Lin Vel: %3.2f %3.2f %3.2f     ", lin_vel.x, lin_vel.y, lin_vel.z));
			cons.Write(0, 2, Fmt("Ang Vel: %3.2f %3.2f %3.2f     ", ang_vel.x, ang_vel.y, ang_vel.z));

			m_engine.Step(step_time);
			now += step_time;

			Sleep(DWORD(step_time * 1010));
		}
		_getch();
	}

	typedef pr::ph::ShapePolytopeHelper TPolyHelper;

	// ********************************************************************
	//void Run() 	{ srand(0); Main m; m.Run(); }
	void Run()
	{
		//v4 verts[] = 
		//{
		//	{0.0f, 0.0f, 0.0f, 1.0f},
		//	{1.0f, 0.0f, 0.0f, 1.0f},
		//	{0.0f, 1.0f, 0.0f, 1.0f},
		//	{1.0f, 1.0f, 0.0f, 1.0f},
		//	{0.0f, 0.0f, 1.0f, 1.0f},
		//	{1.0f, 0.0f, 1.0f, 1.0f},
		//	{0.0f, 1.0f, 1.0f, 1.0f},
		//	{1.0f, 1.0f, 1.0f, 1.0f}
		//};
		v4 verts[] = 
		{
			{0.141392f, -0.501572f, -0.306192f, 1.000000f},
			{0.329813f, 0.079867f, 0.741508f, 1.000000f},
			{0.890561f, 0.865047f, -0.789605f, 1.000000f},
			{0.115940f, -0.693533f, 0.600513f, 1.000000f},
			{-0.854976f, 0.790277f, 0.744621f, 1.000000f},
			{-0.223426f, 0.644826f, 0.097568f, 1.000000f},
			{-0.098117f, 0.343913f, -0.437239f, 1.000000f},
			{0.964293f, 0.898923f, 0.456099f, 1.000000f},
			{-0.455245f, 0.189612f, -0.736076f, 1.000000f},
			{0.201025f, -0.604114f, -0.025300f, 1.000000f},
			{-0.540147f, -0.289346f, -0.043123f, 1.000000f},
			{0.896298f, 0.383587f, 0.342265f, 1.000000f},
			{-0.524949f, 0.965148f, 0.027131f, 1.000000f},
			{0.796075f, -0.528367f, -0.031281f, 1.000000f},
			{0.074984f, -0.347087f, 0.141575f, 1.000000f},
			{-0.303690f, -0.326945f, -0.661550f, 1.000000f},
			{-0.561449f, -0.419660f, 0.845637f, 1.000000f},
			{0.614002f, 0.479354f, 0.300882f, 1.000000f},
			{-0.459822f, 0.373394f, 0.533738f, 1.000000f},
			{0.443526f, -0.871151f, 0.718131f, 1.000000f}
		};
		ShapeBox		boxA;		boxA		.set(v4::make(5.0f, 0.25f, 5.0f, 0.0f), m4x4Identity, 0, 0);
		ShapeCylinder	cylinderA;	cylinderA	.set(1.0f, 0.5f, m4x4Identity, 0, 0);
		ShapeSphere		sphereA;	sphereA		.set(1.0f, m4x4Identity, 0, 0);
		TPolyHelper		polyA;		polyA		.set(verts, sizeof(verts)/sizeof(verts[0]), m4x4Identity, 0, 0);

		ShapeBox		boxB;		boxB		.set(v4::make(0.5f, 0.5f, 0.5f, 0.0f), m4x4Identity, 0, 0);
		ShapeCylinder	cylinderB;	cylinderB	.set(1.0f, 0.5f, m4x4Identity, 0, 0);
		ShapeSphere		sphereB;	sphereB		.set(0.2f, m4x4Identity, 0, 0);
		TPolyHelper		polyB;		polyB		.set(verts, sizeof(verts)/sizeof(verts[0]), m4x4Identity, 0, 0);

		Shape& shapeA = 
			polyA.get().m_base;
			//sphereA.m_base;
			//boxA.m_base;
			//cylinderA.m_base;
		//Shape& shapeB =
			//polyB.get().m_base;
			//sphereB.m_base;
			//boxB.m_base;
			//cylinderB.m_base;
		
		ContactManifold contact_manifold;

		//int max_loops = 0;
		//int i_for_max_loops = 0;
		//int max_opposing = 0;
		//int i_for_max_opposing = 0;
		//int max_refine = 0;
		//int i_for_max_refine = 0;

		int i = 199, i_end = 10000;
		for( i = 0; i != i_end; ++i )
		//for( ; i != i_end; ++i )
		{
			srand(i);
			m4x4 a2w = m4x4Random(v4::make( 0.0f, 0.0f, 0.0f, 1.0f), 1.0f);
			//m4x4 b2w = m4x4Random(v4::make( 01.5f, 0.0f, 0.0f, 1.0f), 1.0f);
			//m4x4 a2w = m4x4Identity;
			//m4x4 b2w = m4x4Identity;//Rotation4x4(maths::half_pi, maths::pi, 0.0f);
			//b2w = Rotation4x4(b2w, maths::half_pi, 0.0f, 0.0f);
			//b2w.pos.x = 0.2f;//0.0f;//
			//b2w.pos.y = 0.2f;
			//b2w.pos.z = 0.01f;
			//b2w.pos.z = -0.000001f;//0.0f;//
			//b2w.pos.x = 0.8f;
			//b2w.pos.y = 0.3333333f;
			////b2w.pos.z = 0.3333333f;
//boxA.set(v4::make(0.000000f, 0.000000f, 0.000000f, 0.0f), m4x4Identity, 0, 0.0f);
//boxB.set(v4::make(0.500000f, 0.500000f, 0.500000f, 0.0f), m4x4Identity, 0, 0.0f);
//m4x4 a2w = {
//0.966496f,      0.256653f,      0.003893f,      0.000000f,
//-0.256430f,     0.964758f,      0.059033f,      0.000000f,
//0.011396f,      -0.058054f,     0.998248f,      0.000000f,
//-0.014495f,     0.084455f,      0.002463f,      1.000000f
//};
//m4x4 b2w = {
//0.995262f,      0.096948f,      -0.007383f,     0.000000f,
//-0.091673f,     0.960984f,      0.260972f,      0.000000f,
//0.032396f,      -0.259059f,     0.965318f,      0.000000f,
//-0.049017f,     1.315711f,      0.065578f,      1.000000f
//};

			m4x4 w2a		= GetInverseFast(a2w);
			ph::Ray ray;
			//ray.m_point		= w2a * v4Random3(v4::make(0.0f, -5.0f, 0.0f, 1.0f), 1.0f, 1.0f);
			//ray.m_direction	= w2a * v4RandomNormal3(0.0f) * FRand(1.0f, 10.0f);
			ray.m_point		= w2a * v4::make(-1.0f, 1.0f, 0.49999f, 1.0f);
			ray.m_direction	= w2a * v4::make( 2.0f, -2.0f, 0.0f, 0.0f);
			ray.m_thickness	= 0.3f;
			
			RayCastResult result, result_;
			PR_UNUSED(result_);
			if( ph::RayCast(ray, shape_cast<ShapePolytope>(shapeA), result) )
			{
				PR_ASSERT(1, RayCastBruteForce(ray, shape_cast<ShapePolytope>(shapeA), result_));
				PR_ASSERT(1, FEql(result.m_t0, result_.m_t0));
				PR_ASSERT(1, FEql(result.m_t1, result_.m_t1));
				PR_ASSERT(1, result.m_t0 == 0.0f || FEql3(result.m_normal, result_.m_normal));
				//Sleep(1000);
				printf("%d\n", i);
			}
			else
			{
				PR_ASSERT(1, !RayCastBruteForce(ray, shape_cast<ShapePolytope>(shapeA), result_));
				++i_end;
			}



			////v4 normal;
			//LDR_OUTPUT(1, ldr::PhCollisionScene(shapeA, a2w, shapeB, b2w);)

			//contact_manifold.m_contact.clear();
			//GetNearestPoints(shapeA, a2w, shapeB, b2w, contact_manifold, 0);
			//Contact const& contact = contact_manifold.Deepest();
			//LDR_OUTPUT(1, StartFile("C:/DeleteMe/Features.pr_script");)
			//LDR_OUTPUT(1, ldr::Box("PtA", "FFFF0000", contact.m_pointA + a2w.pos, 0.05f);)
			//LDR_OUTPUT(1, ldr::Box("PtB", "FF0000FF", contact.m_pointB + b2w.pos, 0.05f);)
			//LDR_OUTPUT(1, ldr::LineD("norm", "FFFFFF00", contact.m_pointB + b2w.pos, contact.m_normal * contact.m_depth);)
			//LDR_OUTPUT(1, ldr::LineD("norm", "FFFFFF00", contact.m_pointA + a2w.pos, contact.m_normal * -contact.m_depth);)
			//LDR_OUTPUT(1, EndFile();)
			a2w = a2w;


			//contact_manifold.m_contact.clear();
			//bool collide1 = Collide(shapeA, a2w, shapeB, b2w, contact_manifold, 0);
			////bool collide1 = CollideGJK(shapeA, a2w, shapeB, b2w, contact_manifold);
			//if( collide1 )
			//{
			//	LDR_OUTPUT(1, ldr::phContactManifold(a2w, b2w, contact_manifold);)

			//	PR_ASSERT(1, CollideBruteForce(shapeA, a2w, shapeB, b2w, normal, true)); normal;
			//	
			//	printf("In collision [%d]: \n\tGet Opposing %d\n\tRefine edge %d\n", i, get_opposing_edge_count, refine_edge_count);
			//	
			//	if( max_loop_count > max_loops             ) { max_loops    = max_loop_count; i_for_max_loops = i; }
			//	if( get_opposing_edge_count > max_opposing ) { max_opposing = get_opposing_edge_count; i_for_max_opposing = i; }
			//	if( refine_edge_count > max_refine         ) { max_refine   = refine_edge_count; i_for_max_refine = i; }
			//}
			//else
			//{
			//	PR_ASSERT(1, !CollideBruteForce(shapeA, a2w, shapeB, b2w, normal, false));
			//	printf("Not in collision\n");
			//}

			//contact_manifold.m_contact.clear();
			//bool collide2 = Collide   (shapeA, a2w, shapeB, b2w, contact_manifold);
			//if( collide2 )
			//{
			//	LDR_OUTPUT(1, ldr::phContactManifold(a2w, b2w, contact_manifold);)

			//	//PR_ASSERT(1, CollideBruteForce(shapeA, a2w, shapeB, b2w, normal));
			//	//LDR_OUTPUT(1, ldr::LineD("TrueNormal", "FFFFFFFF", contact_manifold.m_contact.front().m_pointA + a2w.pos, normal);)
			//	printf("In collision\n");
			//}
			//else
			//{
			//	//PR_ASSERT(1, !CollideBruteForce(shapeA, a2w, shapeB, b2w, normal));
			//	printf("Not in collision\n");
			//}
			//Sleep(500);
		}

		printf("Test done. Please the any key\n");
		_getch();
	}
	// ********************************************************************
}//namespace TestPhysics


	//// Make a static ground box
	//void Main::CreateStaticBox()
	//{
	//	ph::ShapeBuilder shape_builder;

	//	// Make the static box collision shape
	//	shape_builder.Begin();
	//	shape_builder.AddBox(10.0f, 0.5f, 10.0f, 0, m4x4Identity);
	//	m_ground.m_shape = shape_builder.BuildShape(m_model_data);
	//	m_ground.m_i2w.Identity();
	//	{
	//		using namespace pr::geometry;
	//		m4x4 scale = m4x4Identity;
	//		scale.x.x = 10.0f;
	//		scale.y.y = 0.5f;
	//		scale.z.z = 10.0f;
	//		m_ground.m_graphic_inst.m_model = CreateModel(unit_cube::num_vertices, unit_cube::vertices, unit_cube::num_indices, unit_cube::indices, scale);
	//	}
	//}


//	//void LDRDrawInstances(Thing* instance, uint num_instances);
//
////PSR...		// Do things to the instances
////PSR...		uint T = 1000;
////PSR...		for( uint t = 0; t < T; ++t )
////PSR...		{
////PSR...			LDRDrawInstances(obj, 2);
////PSR...
////PSR...			// Apply impulses/forces
////PSR...
////PSR...			// Update the object positions
////PSR...			obj[0].Update();
////PSR...			obj[1].Update();
////PSR...			
////PSR...			// Loop over the potentially colliding pairs
////PSR...			Thing *objA, *objB;
////PSR...			g_dom.FirstOverlap();
////PSR...			while( g_dom.GetOverlap(objA, objB) )
////PSR...			{
////PSR...				CollisionData data;
////PSR...				data.m_objA	= &objA->m_physics;
////PSR...				data.m_objB	= &objB->m_physics;
////PSR...				data.m_contact= NULL;
////PSR...
////PSR...				g_engine.CollisionDetection(data);
////PSR...				g_engine.ResolveCollision(data);
////PSR...
////PSR...				objA->Update();
////PSR...				objB->Update();
////PSR...			}
////PSR...		}
////	}
////
////	Primitive::Type GetRandomPrimitiveType()
////	{
////		switch( URand(Primitive::Box, Primitive::NumberOf) )
////		{
////		case Primitive::Box:		return Primitive::Box;
////		case Primitive::Cylinder:	return Primitive::Cylinder;
////		case Primitive::Sphere:		return Primitive::Sphere;
////		default: PR_ASSERT(PR_DBG_COMMON, false); return Primitive::Box;
////		}
////	}
////
////	void LDRDrawInstances(Thing* instance, uint num_instances)
////	{
////		ldr::StartFile("C:\\Physics.txt");
////		for( uint i = 0; i < num_instances; ++i )
////		{
////			ldr::PhInst(Fmt("Thing_%d", i).c_str(), "FFFFFFFF", instance[i].m_physics);
////			ldr::BoundingBox(Fmt("Thing_%d_BBox", i).c_str(), "7FFF0000", instance[i].m_physics.m_physics_object->m_bbox);
////		}
////		ldr::EndFile();
////	}

//PSR...		// Object 1
//PSR...		Primitive primitive;
//PSR...		primitive.m_type				= Primitive::Box;
//PSR...		primitive.m_radius[0]			= 1.0f;
//PSR...		primitive.m_radius[1]			= 0.3f;
//PSR...		primitive.m_radius[2]			= 1.2f;
//PSR...		primitive.m_material_index		= 0;
//PSR...		//prim_to_model		.Identity();
//PSR...		//prim_to_model[3]	.Set(0.0f, 0.0f, 0.0f, 1.0f);
//PSR...		//prim_to_model		.Rotation4x4(v3YAxis, D3DX_PI/4.0f);
//PSR...		//prim_to_model[3]	.Set(4.0f, 0.0f, -2.0f, 1.0f);
//PSR...		prim_to_model		.Rotation4x4(v3::RandomNormal(), FRand(0.0f, 2.0f * D3DX_PI));
//PSR...		prim_to_model[3]	.Set(v3::Random(0.1f, 1.5f), 1.0f);
//PSR...
//PSR...		object_builder.Begin();
//PSR...		object_builder.AddPrimitive(primitive);
//PSR...		object_builder.End();
//PSR...
//PSR...		// Object 2
//PSR...		primitive.m_type					= Primitive::Box;
//PSR...		primitive.m_radius[0]				= 1.0f;
//PSR...		primitive.m_radius[1]				= 0.7f;
//PSR...		primitive.m_radius[2]				= 2.0f;
//PSR...		primitive.m_material_index			= 0;
//PSR...		//prim_to_model		.Identity();
//PSR...		//prim_to_model[3]	.Set(0.0f, 0.0f, 0.0f, 1.0f);
//PSR...		//prim_to_model		.Rotation4x4(v3YAxis, D3DX_PI/4.0f);
//PSR...		//prim_to_model		*= m4x4(v3ZAxis, 0.27f + D3DX_PI/4.0f, v4Origin);
//PSR...		//prim_to_model[3]	.Set(10.0f, 0.0f, 10.5f, 1.0f);
//PSR...		prim_to_model		.Rotation4x4(v3::RandomNormal(), FRand(0.0f, 2.0f * D3DX_PI));
//PSR...		prim_to_model[3]	.Set(v3::Random(0.1f, 1.5f), 1.0f);
//PSR...
//PSR...		object_builder.Begin();
//PSR...		object_builder.AddPrimitive(primitive);
//PSR...		object_builder.End();
//PSR...
//PSR...		PhysicsObjectList object_list;
//PSR...		object_builder.ExportPhysicsObjectList(object_list);
//PSR...
//PSR...		PhysicsEngine engine;
//PSR...		PhysicsEngineSettings pesettings;
//PSR...		pesettings.m_max_collision_groups	= 1;
//PSR...		pesettings.m_max_contact_points		= 10;
//PSR...		pesettings.m_max_physics_materials	= 1;
//PSR...		engine.Initialise(pesettings);
//PSR...		engine.CollisionGroup(0, 0)			= ZerothOrderCollision;
//PSR...		
//PSR...		srand(0);
//PSR...		int num_tests = 100;
//PSR...		for( int test = 0; test < num_tests; ++test )
//PSR...		{
//PSR...			// Instances of physics objects
//PSR...			Instance obj1;
//PSR...			obj1.m_physics_object			= object_list.GetObject(0);
//PSR...			obj1.m_collision_group			= 0;
//PSR...			obj1.m_ang_velocity				.Zero();
//PSR...			obj1.m_velocity					.Zero();
//PSR...			//obj1.m_object_to_world			.Identity();
//PSR...			//obj1.m_object_to_world[3]		.Set(0.0f, 0.0f, 0.0f, 1.0f);
//PSR...			//obj1.m_object_to_world			.Rotation4x4(v3XAxis, D3DX_PI/2.0f);
//PSR...			//obj1.m_object_to_world[3]		.Set(0.2f, 0.0f, 0.0f, 1.0f);
//PSR...			obj1.m_object_to_world			.Rotation4x4(v3::RandomNormal(), FRand(0.0f, 2.0f * D3DX_PI));
//PSR...			obj1.m_object_to_world[3]		.Set(v3::Random(0.1f, 1.5f), 1.0f);
//PSR...			
//PSR...			Instance obj2;
//PSR...			obj2.m_physics_object			= object_list.GetObject(1);
//PSR...			obj2.m_collision_group			= 0;
//PSR...			obj2.m_ang_velocity				.Zero();
//PSR...			obj2.m_velocity					.Zero();
//PSR...			//obj2.m_object_to_world			.Identity();
//PSR...			//obj2.m_object_to_world[3]		.Set(5.0f, 5.0f, 8.0f, 1.0f);
//PSR...			//obj2.m_object_to_world			.Rotation4x4(v3XAxis, D3DX_PI/4.0f);
//PSR...			//obj2.m_object_to_world[3]		.Set(1.9f, 1.0f, 0.0f, 1.0f);
//PSR...			obj2.m_object_to_world			.Rotation4x4(v3::RandomNormal(), FRand(0.0f, 2.0f * D3DX_PI));
//PSR...			obj2.m_object_to_world[3]		.Set(v3::Random(0.1f, 1.5f), 1.0f);
//PSR...
//PSR...			CollisionParams params;
//PSR...			CollisionResult result;
//PSR...
//PSR...			params.m_objA			= &obj1;
//PSR...			params.m_objB			= &obj2;
//PSR...			params.m_contact		= NULL;
//PSR...			params.m_max_contacts	= 0;
//PSR...			engine.CollisionDetection(params, result);
//PSR...
//PSR...			printf("Test: %d  Num contacts: %d\n", test, result.m_num_contacts);
//PSR...			if( test + 1 < num_tests ) Sleep(10000);
//PSR...		}
