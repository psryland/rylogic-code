#! "net10.0-windows"
#r "../../rylogic/Rylogic.Core/bin/Debug/net10.0-windows/Rylogic.Core.dll"
#r "../../rylogic/Rylogic.Gfx/bin/Debug/net10.0-windows/Rylogic.Gfx.dll"
#nullable enable

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Scripting.Hosting;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.LDraw;
using Rylogic.Maths;
using Rylogic.Utility;

var Spacing = 1f;
var Interval = 2f;
var SampleCount = 8;
var SubStepCount = 20;
var VelScale = 0.05f;
var AvlScale = 0.4f;
var BoxDim = 0.1f * new v4(1, 2, 3, 0);
var Colour0 = new Colour32(0xFFFF2020);
var Colour1 = new Colour32(0xFF20FF20);
var VelColour = new Colour32(0xFFFF00FF);
var AvlColour = new Colour32(0xFFFFFF00);
var OriColour = new Colour32(0xFF00FFFF);
var MeasAvlColour = new Colour32(0xFF0000FF);
var rng = new Random(1);

var data =
	TestCase_Random();
	//TestCase_Rotating(0.3f, 0.6f);
	//TestCase_FromKnownParameters(new(){
	//	vel = new v3(0.3f, 0, 0),
	//	acl = new v3(0, 0, 0),
	//	avl = new v3(0, 0, 0.1f),
	//	aac = new v3(0, 0.08f, 0),
	//});

{
	var builder = new Builder();
	var box = builder.Box("Box").dim(BoxDim).hide();
	
	// Inputs
	var grp_inputs = builder.Group("Inputs");
	var grp_inputs_boxes = grp_inputs.Group($"Boxes");
	var grp_inputs_vel   = grp_inputs.Group($"Velocities").hide(true);
	var grp_inputs_avl   = grp_inputs.Group($"Angular_Velocities").hide(false);

	// Eval gfx
	var grp_eval          = builder.Group("Eval");
	var grp_eval_boxes    = grp_eval.Group($"Boxes").hide(true);
	var grp_eval_vel      = grp_eval.Group($"Velocities").hide(true);
	var grp_eval_avl      = grp_eval.Group($"Angular_Velocities").hide(true);
	var grp_eval_avl_meas = grp_eval.Group($"Measured_Angular_Velocities").hide(false);
	var grp_eval_samples  = grp_eval.Point("Samples").depth(false).size(20f).style(EPointStyle.Circle);

	// Bone path
	var grp_path = builder.Line("Path").strip(data[0].pos.w1, Colour0).width(3f);

	// Paths showing orientations
	var grp_ori = builder.Group("Orientations").hide(true);
	var grp_ori_path = grp_ori.Line("Path", OriColour).strip(data[0].pos.w1 + Math_.LogMap(data[0].ori), Colour0).width(3f);
	var grp_ori_pts = grp_ori.Point("Points", OriColour).size(20f);

	var grp_avl_outp_path = builder.Line("Output_Avl", AvlColour).strip(data[0].pos.w1 + AvlScale * data[0].avl.w0, Colour0);
	var grp_avl_meas_path = builder.Line("Measured_Avl", MeasAvlColour).strip(data[0].pos.w1 + AvlScale * data[0].avl.w0, Colour0);
	
	for (int i = 0; i != data.Count; ++i)
	{
		var sample0 = data[(i + 0) % data.Count];
		var sample1 = data[(i + 1) % data.Count];

		var P = new InterpolateVector(sample0.pos, sample0.vel, sample1.pos, sample1.vel, Interval);
		var R = new InterpolateRotation(sample0.ori, sample0.avl, sample1.ori, sample1.avl, Interval);

		// Inputs
		{
			grp_inputs_boxes.Instance("Box", Colour0.Alpha(0.5)).ori(sample0.ori).pos(sample0.pos);
			grp_inputs_vel.Arrow("Vel", Colour0.LerpRGB(VelColour, 0.8)).start(-VelScale * sample0.vel).line_to(VelScale * sample0.vel).pos(sample0.pos);
			grp_inputs_avl.Arrow("Avl", MeasAvlColour).width(5f).start(-AvlScale * sample0.avl).line_to(AvlScale * sample0.avl).pos(sample0.pos);

			grp_ori.Arrow($"Ori_{i}", OriColour).start(v4.Origin).line_to(Math_.LogMap(sample0.ori)).pos(sample0.pos);
			grp_ori_path.line_to(sample0.pos.w1 + Math_.LogMap(sample0.ori));
			grp_ori_pts.pt(sample0.pos.w1 + Math_.LogMap(sample0.ori));

			grp_avl_outp_path.line_to(sample0.pos.w1 + AvlScale * sample0.avl.w0);
			grp_avl_meas_path.line_to(sample0.pos.w1 + AvlScale * sample0.avl.w0);
		}

		if ((i + 1) % data.Count == 0)
			break;

		// Interpolate
		//if (false)
		{
			Key Sample(int j)
			{
				if (j <= 0)
					return sample0;
				if (j >= SubStepCount + 1)
					return sample1;
				
				var time = (float)Math_.Frac(0, j, SubStepCount + 1) * Interval;
				return new Key
				{
					pos = P.Eval(time),
					vel = P.EvalDerivative(time),
					ori = R.Eval(time),
					avl = R.EvalDerivative(time),
				};
			}

			var step = Interval / (SubStepCount + 1f);
			for (int j = 0; j != SubStepCount; ++j)
			{
				var sample_p1 = Sample(j + 0);
				var sample_c0 = Sample(j + 1);
				var sample_n1 = Sample(j + 2);

				grp_eval_boxes.Instance("Box", Colour1.Alpha(0.5)).ori(sample_c0.ori).pos(sample_c0.pos);
				grp_eval_vel.Arrow("Vel", VelColour.LerpRGB(Colour1, 0.2)).start(-VelScale * sample_c0.vel).line_to(VelScale * sample_c0.vel).pos(sample_c0.pos);
				grp_eval_avl.Arrow("Avl", AvlColour.LerpRGB(Colour1, 0.2)).start(-AvlScale * sample_c0.avl).line_to(AvlScale * sample_c0.avl).pos(sample_c0.pos);
				grp_eval_samples.pt(sample_c0.pos, Colour1);

				grp_path.line_to(sample_c0.pos, Colour1);

				grp_ori.Arrow($"Ori_{i}_{j}", OriColour).start(v4.Origin).line_to(Math_.LogMap(sample_c0.ori)).pos(sample_c0.pos);
				grp_ori_path.line_to(sample_c0.pos.w1 + Math_.LogMap(sample_c0.ori));
				grp_ori_pts.pt(sample_c0.pos.w1 + Math_.LogMap(sample_c0.ori));

				// Output Avl
				grp_avl_outp_path.line_to(sample_c0.pos.w1 + AvlScale * sample_c0.avl.w0);

				// Measured Avl
				var avl = 2 * Math_.LogMap(sample_n1.ori * ~sample_p1.ori).xyz / (2*step);
				grp_eval_avl_meas.Arrow("MeasuredVel", MeasAvlColour).start(-AvlScale * avl).line_to(AvlScale * avl).pos(sample_c0.pos);
				grp_avl_meas_path.line_to(sample_c0.pos.w1 + AvlScale * avl.w0);

			}
			grp_path.line_to(sample1.pos, Colour1);
		}
	}

	builder.Save(FP("Scene"), ESaveFlags.Pretty); // | ESaveFlags.Binary
}

//************************************************************************

public class Key
{
	public int frame = 0;

	public Quat ori = Quat.Identity;
	public v3 avl = v3.Zero;
	public v3 aac = v3.Zero;

	public v3 pos = v3.Zero;
	public v3 vel = v3.Zero;
	public v3 acl = v3.Zero;
}

// Random points and velocities
List<Key> TestCase_Random()
{
	List<Key> keys = [];
	for (int i = 0; i != SampleCount + 5; ++i)
		keys.Add(new() { frame = i, pos = v3.Random3(v3.Zero, 10f, rng) });

	CalculateDynamics(keys, Interval, generate_orientations: true);

	keys.RemoveRange(0, 2);
	keys.RemoveRange(keys.Count - 2, 2);

	return keys;
}

// Generate keys from known dynamics
List<Key> TestCase_FromKnownParameters(Key key0)
{
	List<Key> keys = [];
	var prev = key0.ori;

	for (int i = 0; i != SampleCount; ++i)
	{
		var time = Interval * i;

		var pos = key0.pos + key0.vel * time + 0.5f * key0.acl * time * time;
		var vel = key0.vel + key0.acl * time;
		var acl = key0.acl;

		var ori = Math_.RotationAt(time, key0.ori, key0.avl.w0, key0.aac.w0);
		var avl = key0.avl + key0.aac * time;
		var aac = key0.aac;

		Debug.Assert(avl.Length < Math_.TauBy2F, "Must rotate less that 180° between frames");

		if (Math_.Dot(prev.xyzw, ori.xyzw) < 0)
			ori = -ori;

		keys.Add(new()
		{
			pos = pos,
			vel = vel,
			acl = acl,

			ori = ori,
			avl = avl,
			aac = aac,
		});

		prev = ori;
	}

	return keys;
}

// Rotating and translating
List<Key> TestCase_Rotating(float vel, float avl)
{
	List<Key> keys = [];

	for (int i = 0; i != SampleCount; ++i)
	{
		var time = Interval * i;
		keys.Add(new()
		{
			pos = new v3(vel * time, 0, 0),
			ori = Math_.ExpMap(0.5f * new v3(avl, 0, avl) * time),
		});
	}

	CalculateDynamics(keys, dt: Interval, generate_orientations: false);
	return keys;
}

// Set the dynamics by central difference
void CalculateDynamics(List<Key> keys, float dt, bool generate_orientations = false)
{
	var _2dt = 2.0f * dt;

	{// Linear
		// Assume positions have been set

		// Velocities
		for (int i = 1; i != keys.Count - 1; ++i)
		{
			keys[i].vel = (keys[i + 1].pos - keys[i - 1].pos) / _2dt;
		}
		keys[0].vel = keys[1].vel;
		keys[^1].vel = keys[^2].vel;

		// Accelerations
		for (int i = 1; i != keys.Count - 1; ++i)
		{
			keys[i].acl = (keys[i + 1].vel - keys[i - 1].vel) / _2dt;
		}
		keys[0].acl = v3.Zero;
		keys[^1].acl = v3.Zero;
	}

	{// Angular

		// Orientations
		if (generate_orientations)
		{
			for (int i = 0; i != keys.Count; ++i)
				keys[i].ori = new Quat(v4.ZAxis, keys[i].vel.w0);
		}

		// Orientations - shortest arc
		for (int i = 1; i != keys.Count; ++i)
		{
			if (Math_.Dot(keys[i].ori.xyzw, keys[i - 1].ori.xyzw) > 0) continue;
			keys[i].ori = -keys[i].ori;
		}

		// Ang velocity
		for (int i = 1; i != keys.Count - 1; ++i)
		{
			var half_omega = Math_.LogMap(keys[i + 1].ori * ~keys[i - 1].ori);
			keys[i].avl = 2 * half_omega.xyz / _2dt;
		}
		keys[0].avl = keys[1].avl;
		keys[^1].avl = keys[^2].avl;

		// Ang acceleration
		for (int i = 1; i != keys.Count - 1; ++i)
		{
			keys[i].aac = (keys[i + 1].avl - keys[i - 1].avl) / _2dt;
		}
		keys[0].aac = keys[1].aac;
		keys[^1].aac = keys[^2].aac;
	}
}

//************************************************************************

public readonly struct CubicCurve3
{
	// Notes:
	//  - A general cubic curve is given by a parametric matrix equation:
	//                           [x x x x] [P0]
	//       P(t) =  [1 t t² t³] [x x x x] [P1]
	//                           [x x x x] [P2]
	//                           [x x x x] [P3]
	//
	//  - Different spline types come from different matrix values
	//    e.g.
	//                    [+1 +0 +0 +0] [P0]            [+1 +0 +0 +0] [P0]
	//     Cubic Bezier = [-3 +3 +0 +0] [P1]  Hermite = [+0 +1 +0 +0] [V0]
	//                    [+3 -6 +3 +0] [P2]            [-3 -2 +3 -1] [P1]
	//                    [-1 +3 -3 +1] [P3]            [+2 +1 -2 +1] [V1]
	//
	//  - A hermite curve can be expressed as a Bezier curve using:
	//     [p0, p1, p2, p3] <=> [x0, x0 + v0/3, x1 - v1/3, x1]
	//    For rotations that's:
	//     [q0, q1, q2, q3] <=> [q0, q0*exp(w0/3), q1*~exp(w1/3), q1]

	public static readonly m4x4 Bezier = new(
		new(+1, +0, +0, +0),
		new(-3, +3, +0, +0),
		new(+3, -6, +3, +0),
		new(-1, +3, -3, +1)
	);
	public static readonly m4x4 Hermite = new(
		new(+1, +0, +0, +0),
		new(+0, +1, +0, +0),
		new(-3, -2, +3, -1),
		new(+2, +1, -2, +1)
	);
	public static readonly m4x4 Trajectory = new(
		new(+1, +0, +0, +0),
		new(+0, +1, +0, +0),
		new(+0, +0, +1/2f, +0),
		new(+0, +0, +0, +1/6f)
	);

	readonly m4x4 m_coeff;
	public CubicCurve3(v3 p0, v3 p1, v3 p2, v3 p3, m4x4 coeff)
	{
		m4x4 p = new(p0.w0, p1.w0, p2.w0, p3.w0);
		m_coeff = p * coeff;
	}
	public readonly v3 Eval(float t)
	{
		t = Math.Clamp(t, 0, 1);
		var time = new v4(1, t, t * t, t * t * t);
		var p = m_coeff * time;
		return p.xyz;
	}
	public readonly v3 EvalDerivative(float t)
	{
		t = Math.Clamp(t, 0, 1);
		var time = new v4(0, 1, 2 * t, 3 * t * t);
		var p = m_coeff * time;
		return p.xyz;
	}
	public readonly v3 EvalDerivative2(float t)
	{
		t = Math.Clamp(t, 0, 1);
		var time = new v4(0, 0, 2, 6 * t);
		var p = m_coeff * time;
		return p.xyz;
	}
	public readonly v3 EvalDerivative3()
	{
		var time = new v4(0, 0, 0, 6);
		var p = m_coeff * time;
		return p.xyz;
	}
}
public readonly struct InterpolateVector(v3 x0, v3 v0, v3 x1, v3 v1, float interval)
{
	readonly CubicCurve3 m_p = new(
		x0 - x1,
		v0 * interval,
		v3.Zero,
		v1 * interval,
		CubicCurve3.Hermite
	);
	readonly v3 m_x1 = x1;
	readonly float m_interval = interval;

	public readonly v3 Eval(float t)
	{
		return m_x1 + m_p.Eval(t / m_interval);
	}
	public readonly v3 EvalDerivative(float t)
	{
		return m_p.EvalDerivative(t / m_interval);
	}
	public readonly v3 EvalDerivative2(float t)
	{
		return m_p.EvalDerivative2(t / m_interval);
	}
}
public readonly struct InterpolateRotation(Quat q0, v3 w0, Quat q1, v3 w1, float interval)
{
	readonly CubicCurve3 m_p = new(
	 	Math_.LogMap(~q1 * q0).xyz,
		Tangent(~q1 * q0, Math_.Rotate(~q1, w0) * interval, interval * 0.01f),
	 	v3.Zero,
	 	Tangent(Quat.Identity, Math_.Rotate(~q1, w1) * interval, interval * 0.01f),
		CubicCurve3.Hermite
	);
	readonly Quat m_q1 = q1;
	readonly float m_interval = interval;

	public readonly Quat Eval(float t)
	{
		return m_q1 * Math_.ExpMap(m_p.Eval(t / m_interval));
	}
	public readonly v3 EvalDerivative(float t)
	{
		// To calculate 'W' from log_q and log_q`:
		// Say:
		//   u = log(q), r = |u| = angle / 2
		//   q = [qv, qw] = [(u/r) * sin(r), cos(r)] = [u*f(r), cos(r)]
		//     where f(r) = sin(r) / r
		// Also:
		//   u  == m_p.Eval(t)
		//   u` == m_p.EvalDerivative(t)
		//   r` == Dot(u`, u) / r  (where r > 0) (i.e. tangent amount in direction of u)
		//
		// Differentiating:
		//   f`(r) = (r*cos(r) - sin(r)) / r² (product rule)
		//   q` = [qv`, qw`] = [u`*qv + u*qv`*r`, -sin(r)*r`]
		// Also:
		//   q` = 0.5 x [w,0] x q  (quaternion derivative)
		//    => [w,0] = 2*(q` x ~q) = 2*(qw*qv` - qw`*qv - Cross(qv`, qv)) (expanded quaternion multiply)
		//
		// For small 'r' can use expansion for sin:
		//   f(r) = sin(r)/r ~= 1 - r²/6 +...
		//   f`(r) = -r/3 + ...
		// For really small 'r' use:
		//   W ~= 2 * u`  (comes from: if q = [u, 1] => q` ~= [u`, 0]
		const float TinyAngle = 1e-5f;

		var u = m_p.Eval(t / m_interval);
		var u_dot = m_p.EvalDerivative(t / m_interval);
		var r = u.Length;

		var omega = 2 * u_dot;
		if (r > TinyAngle)
		{
			// Derivative of angle
			var r_dot = Math_.Dot(u, u_dot) / r;
			var sin_r = Math.Sin(r);
			var cos_r = Math.Cos(r);

			// Derivative of axis
			var f = sin_r / r;
			var f_dot = (r * cos_r - sin_r) / (r * r);

			// q
			var qv = u * f; // vector part
			var qw = cos_r; // scalar part

			// q`
			var qw_dot = -sin_r * r_dot;
			var qv_dot = u_dot * f + u * (f_dot * r_dot);

			// Vector part of (q` * ~q): vw = qw*qv` - qw`*qv - qv` x qv
			omega = 1.0f * (qw * qv_dot - qw_dot * qv - Math_.Cross(qv_dot, qv));
		}

		return Math_.Rotate(m_q1, omega);
	}

	// Returns the tangent vector in SO(3)
	static v3 Tangent(Quat q, v3 w, float eps)
	{
		var q_p1 = Math_.ExpMap(-0.5f * eps * w) * q;
		var q_n1 = Math_.ExpMap(+0.5f * eps * w) * q;
		var tangent = (Math_.LogMap(q_n1).xyz - Math_.LogMap(q_p1).xyz) / (2 * eps);
		return tangent;
	}
}

//************************************************************************

string FP(string filename)
{
	return Path.Join(Util.ThisDirectory(), filename);
}
