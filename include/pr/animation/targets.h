//*****************************************************************************
// Procedural animation - targets and layers
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
#pragma once
#include "pr/animation/forward.h"
#include "pr/animation/rig.h"
#include "pr/animation/target_path.h"

namespace pr::animation
{
	// The model-space goal for one rig chain. Its meaning depends on the chain's role:
	//  - Spine: 'base' is the first bone's frame (pelvis) and 'frame' is the last bone's frame (chest).
	//  - Leg, Arm: 'frame' is the end bone's frame (foot, hand) and 'pole' is the direction the middle joint bends towards (unit, w = 0).
	//  - Neck: 'frame.pos' is the point the last bone faces.
	//  - Tail, Digit: unused. Tails are posed by 'Locomotion'.
	struct Target
	{
		xform frame = xform::Identity();
		xform base = xform::Identity();
		v4 pole = {};
	};

	// Write into 'out' one target per chain of 'rig' that reproduces the pose whose model-space bone transforms are 'model'
	inline void PoseTargets(Rig const& rig, std::span<xform const> model, std::span<Target> out)
	{
		// Read each chain's goal from the pose
		pr_assert(model.size() == rig.bones.size() && out.size() == rig.chains.size() && "One transform per bone and one target per chain are required");
		for (int c = 0, cend = static_cast<int>(rig.chains.size()); c != cend; ++c)
		{
			// The rest-pose directions turn with the bone that carries them
			auto const& chain = rig.chains[c];
			auto first = chain.bones.front();
			auto last = chain.bones.back();
			auto& target = out[c];
			target.frame = model[last];
			target.base = model[first];
			target.pole = Rotate(model[first].rot * ~rig.rest_model[first].rot, chain.pole);

			// A neck faces a point ahead along its forward direction. A distant point keeps the facing steady when the head moves slightly.
			if (chain.role == EChainRole::Neck)
				target.frame.pos += Rotate(model[last].rot * ~rig.rest_model[last].rot, chain.forward) * 100.0f * rig.bones[last].length;
		}
	}
	inline std::vector<Target> PoseTargets(Rig const& rig, std::span<xform const> model)
	{
		// Allocate the targets, then fill them
		std::vector<Target> targets(rig.chains.size());
		PoseTargets(rig, model, targets);
		return targets;
	}

	// Blend from target 'a' (frac = 0) to target 'b' (frac = 1)
	inline Target Blend(Target const& a, Target const& b, float frac)
	{
		// Positions blend linearly, rotations by the shortest arc
		auto blend = [frac](xform const& l, xform const& r)
		{
			return xform(Lerp(l.pos, r.pos, frac), Slerp(l.rot, r.rot, frac));
		};
		return Target{
			.frame = blend(a.frame, b.frame),
			.base = blend(a.base, b.base),
			.pole = Normalise(Lerp(a.pole, b.pole, frac), b.pole),
		};
	}

	// A set of chain targets that overrides those beneath it, by a weight that fades in and out
	struct Layer
	{
		std::vector<int> channels;     // The chain indices that this layer overrides
		std::vector<Target> targets;   // The target for each channel (parallel to 'channels')
		Fade weight;                   // The layer's influence in [0,1]
	};

	// Combine 'base' (one target per chain) with 'layers', applied in order, into 'out'
	inline void ApplyLayers(std::span<Target const> base, std::span<Layer const> layers, std::span<Target> out)
	{
		// Each layer blends its channels over the result so far
		pr_assert(base.size() == out.size() && "One output target per chain is required");
		std::copy(base.begin(), base.end(), out.begin());
		for (auto const& layer : layers)
		{
			// A zero-weight layer has no effect
			pr_assert(layer.channels.size() == layer.targets.size() && "One target per channel is required");
			auto w = layer.weight.Value();
			if (w == 0)
				continue;

			for (int i = 0, iend = static_cast<int>(layer.channels.size()); i != iend; ++i)
				out[layer.channels[i]] = Blend(out[layer.channels[i]], layer.targets[i], w);
		}
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::animation::tests
{
	PRUnitTestClass(TargetLayerTests)
	{
		PRUnitTestMethod(LayersOverrideChannels, Quick)
		{
			// A full-weight layer replaces its channel, a half-weight layer blends, and other channels are untouched
			auto rig = TestRig();
			auto base = PoseTargets(rig, rig.rest_model);
			auto hand = Target{ .frame = xform(v4(1, 1, 1, 1), quat(v4::ZAxis(), 1.0f)), .pole = v4(0, 0, 1, 0) };
			auto look = Target{ .frame = xform(v4(5, 0, 1, 1), Identity<quat>()) };

			std::vector<Layer> layers(2);
			layers[0] = Layer{ .channels = { 3 }, .targets = { hand }, .weight = Fade(1.0f) };
			layers[1] = Layer{ .channels = { 1 }, .targets = { look }, .weight = Fade(0.5f) };

			std::vector<Target> out(base.size());
			ApplyLayers(base, layers, out);
			PR_EXPECT(FEqlAbsolute(out[3].frame.pos, hand.frame.pos, 1e-6f));
			PR_EXPECT(FEqlOrientation(out[3].frame.rot, hand.frame.rot, 1e-5f));
			PR_EXPECT(FEqlAbsolute(out[1].frame.pos, Lerp(base[1].frame.pos, look.frame.pos, 0.5f), 1e-5f));
			PR_EXPECT(FEqlAbsolute(out[2].frame.pos, base[2].frame.pos, 1e-6f));
		}
	};
}
#endif
