//***************************************************
// Spline
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

using System;
using System.Collections.Generic;
using System.Diagnostics;
using Rylogic.Common;

namespace Rylogic.Maths
{
	// x = start position
	// y = start control point. tangent = y - x
	// z = end control point.   tangent = w - z
	// w = end position
	public struct Spline
	{
		private m4x4 m;
		public enum CtrlPt { Start, SCtrl, ECtrl, End };

		// Construct a spline from 4 control points
		public Spline(v4 start, v4 start_ctrl, v4 end_ctrl, v4 end) :this(new m4x4(start, start_ctrl, end_ctrl, end)) {}
		public Spline(v4[] spline) :this(spline[(int)CtrlPt.Start], spline[(int)CtrlPt.SCtrl], spline[(int)CtrlPt.ECtrl], spline[(int)CtrlPt.End]) {}
		public Spline(m4x4 spline)
		{
			Debug.Assert(spline.x.w == 1.0f && spline.y.w == 1.0f && spline.z.w == 1.0f && spline.w.w == 1.0f, "Splines are constructed from 4 positions");
			m = spline;
		}

		/// <summary>Interpretation of the control points</summary>
		public v4 Point0   { get { return m.x; }       set { m.x = value; } }
		public v4 Ctrl0    { get { return m.y; }       set { m.y = value; } }
		public v4 Ctrl1    { get { return m.z; }       set { m.z = value; } }
		public v4 Point1   { get { return m.w; }       set { m.w = value; } }
		public v4 Forward0 { get { return m.y - m.x; } set { m.y = m.x + value; } }
		public v4 Forward1 { get { return m.w - m.z; } set { m.z = m.w - value; } }

		/// <summary>Return the position along the spline at 'time'</summary>
		public v4 Position(float time)
		{
			var blend = new v4(
				(1.0f - time) * (1.0f - time) * (1.0f - time),
				3.0f * time * (1.0f - time) * (1.0f - time),
				3.0f * time * time * (1.0f - time),
				time * time * time);
			return (m * blend).w1;
		}

		/// <summary>
		/// Return the tangent along the spline at 'time'
		/// Notes about velocity:
		/// A spline from (0,0,0) to (1,0,0) with control points at (1/3,0,0) and (2/3,0,0) will
		/// have a constant velocity of (1,0,0) over the full length of the spline.</summary>
		public v4 Velocity(float time)
		{
			var dblend = new v4( // the derivative of blend
				3.0f * (time - 1.0f) * (1.0f - time),
				3.0f * (1.0f - time) * (1.0f - 3.0f * time),
				3.0f * time * (2.0f - 3.0f * time),
				3.0f * time * time);
			return (m * dblend).w0;
		}

		/// <summary>Return the acceleration along the spline at 'time'</summary>
		public v4 Acceleration(float time)
		{
			var ddblend = new v4( // the 2nd derivative of blend
				6.0f * (1.0f - time),
				6.0f * (3.0f * time - 2.0f),
				6.0f * (1.0f - 3.0f * time),
				6.0f * time);
			return (m * ddblend).w0;
		}

		/// <summary>
		/// Return an object to world transform for a position along the spline
		/// 'axis' is the axis id that will lie along the tangent of the spline
		/// By default, the z axis is aligned to the spline with Y as up.</summary>
		public m4x4 O2W(float time)
		{
			return O2W(time, 2, v4.YAxis);
		}

		/// <summary>
		/// Return an object to world transform for a position along the spline
		/// 'axis' is the axis id that will lie along the tangent of the spline
		/// By default, the z axis is aligned to the spline with Y as up.</summary>
		public m4x4 O2W(float time, int axis, v4 up)
		{
			return Math_.TxfmFromDir(axis, Velocity(time), Position(time), up);
		}

		/// <summary>Implicit conversion to/from m4x4</summary>
		public static implicit operator Spline(m4x4 s) { return new Spline(s); }
		public static implicit operator m4x4(Spline s) { return s.m; }
		public m4x4 ToM4x4() { return m; }

		// Equality operators
		//bool operator == (Spline const& lhs, Spline const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) == 0; }
		//bool operator != (Spline const& lhs, Spline const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) != 0; }
		//bool operator <  (Spline const& lhs, Spline const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) <  0; }
		//bool operator >  (Spline const& lhs, Spline const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) >  0; }
		//bool operator <= (Spline const& lhs, Spline const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) <= 0; }
		//bool operator >= (Spline const& lhs, Spline const& rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) >= 0; }

		/// <summary>
		/// Split 'spline' at 't' to produce two new splines
		/// Given the 4 control points P0,P1,P2,P3 of 'spline'
		/// the position is given by:
		///  P4 = lerp(P0,P1,t); P5 = lerp(P1,P2,t); P6 = lerp(P2,P3,t);
		///  P7 = lerp(P4,P5,t); P8 = lerp(P5,P6,t);
		///  P9 = lerp(P7,P8,t);
		/// The two resulting splines 'lhs' and 'rhs' are:
		///  lhs = P0,P4,P7,P9;  rhs = P9,P8,P6,P3</summary>
		public Tuple<Spline,Spline> Split(float t)
		{
			var lhs = new Spline();
			var rhs = new Spline();
			lhs.m.x = m.x;                             // P0
			lhs.m.y = Math_.Lerp(m.x, m.y, t);         // P4
			var P5  = Math_.Lerp(m.y, m.z, t);         // P5
			rhs.m.z = Math_.Lerp(m.z, m.w, t);         // P6
			rhs.m.w = m.w;                             // P3
			lhs.m.z = Math_.Lerp(lhs.m.y, P5, t);      // P7
			rhs.m.y = Math_.Lerp(P5, rhs.m.z, t);      // P8
			rhs.m.x = Math_.Lerp(lhs.m.z, rhs.m.y, t); // P9
			lhs.m.w = rhs.m.x;
			return Tuple.Create(lhs,rhs);
		}

		/// <summary>Return the length of a spline from t0 to t1</summary>
		public float Length(float t0, float t1, float tol = Math_.TinyF)
		{
			// Recursive lambda
			float Len(Spline s, float t)
			{
				float poly_length = (s.m.y - s.m.x).Length + (s.m.z - s.m.y).Length + (s.m.w - s.m.z).Length;
				float chord_length = (s.m.w - s.m.x).Length;
				if (poly_length - chord_length < tol)
					return (poly_length + chord_length) * 0.5f;

				var split = s.Split(0.5f);
				return Len(split.Item1, tol) + Len(split.Item2, tol);
			}

			// Trim 'spline' to the region of interest
			var clipped = this;
			if (t0 != 0f) clipped = clipped.Split(t0).Item2;
			if (t1 != 1f) clipped = clipped.Split(t1).Item1;
			return Len(clipped, tol);
		}

		/// <summary>
		/// Convert a collection of points into a collection of splines.
		/// Generates a spline from each set of three points in 'points'</summary>
		public static IEnumerable<Spline> CreateSplines(IEnumerable<v4> points, ETopo topo)
		{
			// No points, no splines
			var iter = points.GetIterator();
			if (iter.AtEnd)
				yield break;
			
			v4 p0,p1,p2,p3;

			// One point, no splines
			p0 = iter.CurrentThenNext();
			if (iter.AtEnd)
				yield break;

			// Two points, straight line
			p1 = iter.CurrentThenNext();
			if (iter.AtEnd)
				yield return new Spline(p0, p1, p0, p1);

			// Three points, degenerate control point
			p2 = iter.CurrentThenNext();
			if (iter.AtEnd)
				yield return new Spline(p0, (p0 + p1) / 2f, (p1 + p2) / 2f, p2);

			// Generate the stream of splines
			switch (topo)
			{
				case ETopo.Continuous3:
				{
					// Generate splines from a sliding window of three points
					for (bool first = true, last = false; ; first = false)
					{
						var sp = first ? p0 : (p0 + p1)/2f;
						var sc = p1;
						var ec = p1;
						var ep = last ? p2 : (p2 + p1)/2f;

						// Yield a spline
						yield return new Spline(sp, sc, ec, ep);
						if (last) break;

						// Slide the window
						p0 = p1; p1 = p2; p2 = iter.CurrentThenNext();
						last = iter.AtEnd;
					}
					break;
				}
				case ETopo.Continuous4:
				{
					p3 = iter.CurrentThenNext();
					for (; ; )
					{
						// Yield a spline
						yield return new Spline(p0, p1, p2, p3);
						if (iter.AtEnd) break;

						// Slide the window. An exception means, the wrong number of points
						p0 = p3;
						p1 = iter.CurrentThenNext();
						p2 = iter.CurrentThenNext();
						p3 = iter.CurrentThenNext();
					}
					break;
				}
				case ETopo.Disjoint3:
				{
					// Generate splines from each set of three points
					for (; ; )
					{
						// Yield a spline
						yield return new Spline(p0, (p0 + p1)/2f, (p1 + p2)/2f, p2);
						if (iter.AtEnd) break;

						// Slide the window. An exception means, the wrong number of points
						p0 = iter.CurrentThenNext();
						p1 = iter.CurrentThenNext();
						p2 = iter.CurrentThenNext();
					}
					break;
				}
				case ETopo.Disjoint4:
				{
					// Generate splines from each set of four points
					p3 = iter.CurrentThenNext();
					for (;;)
					{
						// Yield a spline
						yield return new Spline(p0, p1, p2, p3);
						if (iter.AtEnd) break;

						// Slide the window. An exception means, the wrong number of points
						p0 = iter.CurrentThenNext();
						p1 = iter.CurrentThenNext();
						p2 = iter.CurrentThenNext();
						p3 = iter.CurrentThenNext();
					}
					break;
				}
				default:
				{
					throw new Exception($"Unsupported spline topology: {topo}");
				}
			}
		}
		public enum ETopo
		{
			/// <summary>3 points per spline. Sliding window of 3 points per spline</summary>
			Continuous3,

			/// <summary>4 points per spline. Last point is the first point of the next spline</summary>
			Continuous4,

			/// <summary>3 points per spline. Each set of 3 points is a separate spline</summary>
			Disjoint3,

			/// <summary>4 points per spline. Each set of 4 points is a separate spline</summary>
			Disjoint4,
		}
		///// <summary>Fill a container of points with a rastered version of this spline</summary>
		//void Raster(pr::Spline const& spline, Cont& points, int max_points, float tol = pr::maths::tinyf)
		//{
		//	{
		//		namespace spline
		//		{
		//			struct Elem // Linked list element type used to form a priority queue
		//			{
		//				Elem* m_next; Spline const* m_spline; int m_ins; float m_err;
		//				Elem(Spline const& s, int ins) :m_next(0) ,m_spline(&s) ,m_ins(ins) ,m_err(pr::Length3(s.y - s.x) + pr::Length3(s.z - s.y) + pr::Length3(s.w - s.z) - pr::Length3(s.w - s.x)) {}
		//			};

		//			// Insert 'elem' into the priority queue of which 'queue' is the head
		//			inline Elem* QInsert(Elem* queue, Elem& elem)
		//			{
		//				if (queue == 0 || queue->m_err < elem.m_err) { elem.m_next = queue; return &elem; }
		//				Elem* i; for (i = queue; i->m_next && i->m_next->m_err > elem.m_err; i = i->m_next) {}
		//				elem.m_next = i->m_next; // 'i->m_next' has an error less than 'elem' (i.m_next might be 0)
		//				i->m_next = &elem;
		//				return queue;
		//			}

		//			// Breadth-first recursive raster of this spline
		//			template <typename Cont> void Raster(Cont& points, Elem* queue, int& pts_remaining, float tol)
		//			{
		//				// Pop the top spline segment from the queue or we've run out of points
		//				Elem const& elem = *queue;
		//				queue = queue->m_next;

		//				// Stop if the error is less than 'tol'
		//				if (elem.m_err < tol || pts_remaining <= 0)
		//					return;

		//				// Subdivide the spline segment and insert the mid-point into 'points'
		//				Spline Lhalf, Rhalf;                       // Spline seqments for each half
		//				Split(*elem.m_spline, 0.5f, Lhalf, Rhalf); // splits the spline at t=0.5
		//				points.insert(points.begin() + elem.m_ins, Lhalf.Point1());

		//				// Increment the insert position for all elements after 'elem.m_ins'
		//				for (Elem* i = queue; i != 0; i = i->m_next)
		//					i->m_ins += (i->m_ins > elem.m_ins);

		//				// Insert both halves into the queue
		//				Elem lhs(Lhalf, elem.m_ins), rhs(Rhalf, elem.m_ins+1);  // Create queue elements for each half
		//				queue = QInsert(queue, lhs);
		//				queue = QInsert(queue, rhs);
		//				Raster(points, queue, --pts_remaining, tol); // continue recursively
		//			}
		//		}
		//	}
		
		//	impl::spline::Elem elem(spline, 1);
		//	points.push_back(spline.Point0());
		//	points.push_back(spline.Point1());
		//	int pts_remaining = max_points - 2;
		//	impl::spline::Raster(points, &elem, pts_remaining, tol);
		//}
	}

	public static partial class Math_
	{
		/// <summary>Find a point along a string of splines</summary>
		public static v4 Lerp(double frac, params Spline[] splines)
		{
			var len = splines.Length;
			if (len == 0)
				throw new Exception("No splines to interpolate");

			// Scale 'frac' up to the number of splines
			var fidx = frac * len;
			if (fidx <= 0.0) return splines[0].Position(0);
			if (fidx >= len) return splines[len - 1].Position(1);

			// Interpolate along the 'idx'th spline
			var idx = (int)fidx;
			return splines[idx].Position((float)fidx - idx);
		}
	}
}
