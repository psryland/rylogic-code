using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows.Threading;
using Rylogic.Extn;
using Rylogic.Maths;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	public sealed class LinkOptimiser :IDisposable
	{
		private readonly ChartControl m_diagram;
		private readonly Diagram_.Options m_opts;
		private readonly List<Connector> m_conns;
		private DispatcherTimer m_timer;
		private int m_issue;

		public LinkOptimiser(ChartControl diagram, Diagram_.Options opts)
		{
			m_diagram = diagram;
			m_opts = opts;
			m_conns = new List<Connector>();
			m_timer = new DispatcherTimer(TimeSpan.FromMilliseconds(100), DispatcherPriority.Background, HandleTick, Dispatcher.CurrentDispatcher);
			m_issue = 0;

			// Handler
			void HandleTick(object? sender, EventArgs e) => Step();
		}
		public void Dispose()
		{
			Running = false;
		}

		/// <summary>Set the Link Optimiser running</summary>
		public bool Running
		{
			get => m_timer.IsEnabled;
			set
			{
				if (Running == value) return;
				if (value) m_timer.Start();
				else m_timer.Stop();
			}
		}

		/// <summary>Step the force simulation</summary>
		public void Step()
		{
			// Update the collection of nodes and connectors whenever the Elements collection changes
			if (m_issue != m_diagram.Elements.IssueNumber)
				Init();

			// Get the world to camera transform and create a set of 2D line segments
			// for the connectors by projecting into the camera view plane.
			var w2c = m_diagram.Scene.Camera.W2O;
			var pairs = m_conns.ToDictionary(x => x, x => new AnchorPair(x, w2c));

			// For each connector, look for a shorter pair of anchors that doesn't cross any other connectors.
			// If already crossing, choose the shortest pair that doesn't cross
			var possibles = new List<AnchorPair>();
			foreach (var conn in m_conns)
			{
				var node0 = conn.Node0;
				var node1 = conn.Node1;

				// Ignore dangling connectors
				if (node0 == null || node1 == null)
					continue;

				// Get the anchor locations that can be used (note, the existing anchors are included if not mixed)
				var anchors0 = node0.AnchorPointsAvailable(m_opts.Relink.AnchorSharingMode, conn.End0).ToList();
				var anchors1 = node1.AnchorPointsAvailable(m_opts.Relink.AnchorSharingMode, conn.End1).ToList();
				if (conn.Anc0.Type != Connector.EEnd.Mixed && !anchors0.Contains(conn.Anc0)) anchors0.Add(conn.Anc0);
				if (conn.Anc1.Type != Connector.EEnd.Mixed && !anchors1.Contains(conn.Anc1)) anchors1.Add(conn.Anc1);

				// Get the number of crossings that 'conn' currently has
				var initial_crossing_count = CountCrossings(new AnchorPair(conn, w2c), pairs.Values, 5);
				var crossing_count = initial_crossing_count;

				// Generate the set of possible connectors.
				possibles.Clear();
				foreach (var anc0 in anchors0)
				{
					foreach (var anc1 in anchors1)
					{
						// Project the segment between anc0 and anc1 into the camera focus plane
						var possible = new AnchorPair(conn, anc0, anc1, w2c);
						var count = CountCrossings(possible, pairs.Values, crossing_count + 1);
						if (count < crossing_count)
						{
							crossing_count = count;
							possibles.Clear();
						}
						if (count == crossing_count)
						{
							possibles.Add(possible);
						}
					}
				}

				// None possible?
				if (possibles.Count == 0)
					continue;

				// Look for a shorter pair. Use hysteresis to prevent snapping back and forth.
				var best = possibles.MinBy(x => x.LengthSq);
				if (crossing_count == initial_crossing_count && best.LengthSq > pairs[conn].LengthSq * 0.999)
					continue;

				// Update the connector
				pairs[conn] = best;
				conn.Anc0 = best.Anc0;
				conn.Anc1 = best.Anc1;
				conn.Invalidate();
			}
		}

		/// <summary>Get the connectors and nodes available for relinking</summary>
		private void Init()
		{
			m_conns.Clear();

			// Re-link the selected connectors or all connectors if nothing is selected
			m_issue = m_diagram.Elements.IssueNumber;
			m_conns.AddRange(m_diagram.Elements.OfType<Connector>());
		}

		/// <summary>Return the number of connectors that 'possible' crosses. Give up if more than 'max'</summary>
		private int CountCrossings(AnchorPair possible, IEnumerable<AnchorPair> existing, int max)
		{
			int count = 0;
			foreach (var pair in existing)
			{
				// Don't test against self
				if (ReferenceEquals(possible.Conn, pair.Conn) && possible.Anc0 == pair.Anc0 && possible.Anc1 == pair.Anc1)
					continue;

				// If 'possible' shares one anchor with 'pair' then it's a crossing (depending on sharing mode)
				var is_crossing =
					(possible.Anc0 == pair.Anc0 && !CanShare(possible.Conn.End0, pair.Anc0.Type)) ||
					(possible.Anc0 == pair.Anc1 && !CanShare(possible.Conn.End0, pair.Anc1.Type)) ||
					(possible.Anc1 == pair.Anc0 && !CanShare(possible.Conn.End1, pair.Anc0.Type)) ||
					(possible.Anc1 == pair.Anc1 && !CanShare(possible.Conn.End1, pair.Anc1.Type)) ||
					CrossingSegments(possible, pair);
				if (is_crossing && ++count == max)
					break;
			}
			return count;

			// True if 'lhs' and 'rhs' can share an anchor point
			bool CanShare(Connector.EEnd lhs, Connector.EEnd rhs)
			{
				return m_opts.Relink.AnchorSharingMode switch
				{
					Node.EAnchorSharing.ShareAll => true,
					Node.EAnchorSharing.ShareSameOnly => lhs == rhs,
					Node.EAnchorSharing.NoSharing => false,
					_ => throw new Exception("Unknown anchor sharing mode"),
				};
			}
			static bool CrossingSegments(AnchorPair lhs, AnchorPair rhs)
			{
				Geometry.ClosestPoint(lhs.Pt0, lhs.Pt1, rhs.Pt0, rhs.Pt1, out var t0, out var t1);
				return t0 > 0 && t0 < 1 && t1 > 0 && t1 < 1;
			}
		}

		/// <summary></summary>
		[DebuggerDisplay("{Description,nq}")]
		private struct AnchorPair
		{
			public Connector Conn;
			public AnchorPoint Anc0;
			public AnchorPoint Anc1;
			public v2 Pt0;
			public v2 Pt1;

			public AnchorPair(Connector conn, AnchorPoint anc0, AnchorPoint anc1, v2 pt0, v2 pt1)
			{
				Conn = conn;
				Anc0 = anc0;
				Anc1 = anc1;
				Pt0 = pt0;
				Pt1 = pt1;
			}
			public AnchorPair(Connector conn, AnchorPoint anc0, AnchorPoint anc1, m4x4 w2c)
				: this(conn, anc0, anc1, (w2c * anc0.LocationWS).xy, (w2c * anc1.LocationWS).xy)
			{ }
			public AnchorPair(Connector conn, m4x4 w2c)
				: this(conn, conn.Anc0, conn.Anc1, w2c)
			{ }

			/// <summary></summary>
			public float LengthSq => (Pt1 - Pt0).LengthSq;

			/// <summary></summary>
			public string Description => $"[{Anc0.Description}] -> [{Anc1.Description}]";
		}
	}
}
