//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Procedural fork lightning: the shape of a bolt from a storm cloud to the ground, and how its light changes over the strike.
#pragma once
#include "pr/game-systems/forward.h"

namespace pr::game_systems
{
	// One point on a lightning bolt's channel. The channel is a tree: each node joins its parent with a straight segment.
	struct LightningNode
	{
		// Position in the sky frame (Z up, w = 1).
		v4 m_pos;

		// Seconds after the strike starts when the leader reaches this node. The origin is reached at time 0.
		float m_arrival;

		// Brightness relative to the main channel, in (0,1]. Branches are dimmer the further they are from the main channel.
		float m_brightness;

		// Width of the glowing channel, in world units.
		float m_width;

		// Index of the node this node grows from, or -1 for the origin.
		int m_parent;

		// True when the node is inside the cloud, where the channel is seen dimly through the cloud.
		bool m_in_cloud;
	};

	// How a lightning bolt looks at one moment of its strike.
	struct LightningLight
	{
		// Brightness of the channel. 1 is the full brightness of the main return stroke; 0 when the bolt is not visible.
		float m_glow;

		// Progress of the fade after the first return stroke's hold, in [0,1]. Branches dim and the channel narrows as it increases.
		float m_fade;

		// How strongly the flash dazzles a viewer, in [0,1]. 1 for a moment after each strong return stroke, then it decays quickly.
		float m_dazzle;

		// Brightness of the light the return strokes shed where the bolt touches the ground, in [0,1]. Each stroke flashes to its strength,
		// holds for StrokeHold, and decays over StrokeDecay, so the light is brief and flickers with the strokes. 0 before the first return stroke.
		float m_flash;

		// True while the leader is still growing toward the ground, before the first return stroke.
		bool m_leader;
	};

	// The light a lightning strike sheds where its bolt touches the ground.
	struct LightningGroundLight
	{
		// The bolt's ground point in the scene frame (w = 1).
		v4 m_position;

		// LightningLight::m_flash, in (0,1].
		float m_brightness;
	};

	// A procedural cloud-to-ground lightning bolt. The same inputs always make the same bolt.
	// The bolt is one bright, thick main channel, a few dim branches spreading sideways inside the cloud, and many faint tendrils below the cloud
	// that reach down and out from the main channel and fork again, up to four levels deep, each level dimmer and thinner than the last.
	// A strike has three phases: a dim leader grows from inside the cloud toward the ground, a bright return stroke lights the whole channel
	// when the leader touches the ground, and the channel then fades and narrows, with up to two more return strokes making it flicker.
	struct LightningBolt
	{
		// The most return strokes in one strike.
		static constexpr int StrokesMax = 3;

		// Seconds that a return stroke holds the bolt at the stroke's full brightness and width before it starts to decay. The first stroke's hold
		// also delays the channel's fade.
		static constexpr float StrokeHold = 0.06f;

		// Seconds that one return stroke's extra brightness takes to fall to about 37% after its hold.
		static constexpr float StrokeDecay = 0.06f;

		// Seconds that a return stroke fully dazzles a viewer, and the decay time of the dazzle after that.
		static constexpr float DazzleHold = 0.1f;
		static constexpr float DazzleDecay = 0.07f;

		// The channel tree. Node 0 is the origin inside the cloud. The main channel ends at the ground node.
		std::vector<LightningNode> m_nodes;

		// Where the bolt starts inside the cloud, where it leaves the cloud base, and where it strikes the ground, all in the sky frame.
		v4 m_origin;
		v4 m_exit;
		v4 m_ground;

		// Seconds from the start of the strike until the leader touches the ground and the first return stroke begins.
		float m_leader_time;

		// The start time (seconds after the start of the strike) and relative strength of each return stroke. The first stroke has strength 1.
		int m_stroke_count;
		float m_stroke_times[StrokesMax];
		float m_stroke_strengths[StrokesMax];

		// Seconds the strike lasts. The bolt is invisible after this.
		float m_duration;

		// Grow a bolt that strikes 'ground' (sky frame) from a cloud whose base is at height 'cloud_base' (sky frame Z), using 'seed' for every random choice.
		// 'cloud_base' must be above the ground point.
		LightningBolt(v4 const& ground, float cloud_base, uint32_t seed);

		// The bolt's light 'age' seconds after the strike starts.
		LightningLight LightAt(float age) const;
	};

	struct LightningShader;

	// A lightning bolt drawn in a scene: the bolt, its model, and when it started. Create, update and release on the renderer owner thread.
	struct LightningStrike
	{
		// The bolt's model. Its positions are in the sky frame, so the instance transform rotates the sky frame into the scene.
		struct Instance
		{
			#define PR_RDR_INST(x)\
			x(m4x4       , m_i2w  , EInstComp::I2WTransform)\
			x(ModelPtr   , m_model, EInstComp::ModelPtr)\
			x(SKOverride , m_sko  , EInstComp::SortkeyOverride)
			PR_RDR12_INSTANCE_MEMBERS(Instance, PR_RDR_INST);
			#undef PR_RDR_INST
		};

		LightningBolt m_bolt;

		// The time on the owner's lightning clock when the strike started, in seconds.
		double m_start_time;

		Instance m_inst;

		// The bolt's shader. The model's material owns it, so it lives as long as the model.
		LightningShader* m_shader;

		// Create the model that draws 'bolt' using 'factory', for a strike that starts at 'start_time' on the owner's lightning clock.
		// The caller flushes the factory to the GPU before the strike is drawn.
		LightningStrike(ResourceFactory& factory, LightningBolt bolt, double start_time);

		// The bolt's light 'age' seconds after the strike starts, which is also how the bolt will be drawn until the next call.
		LightningLight Show(float age);
	};
}
