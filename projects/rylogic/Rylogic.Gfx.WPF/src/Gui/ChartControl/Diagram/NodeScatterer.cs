using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows.Threading;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Maths;

namespace Rylogic.Gui.WPF.ChartDiagram
{
	/// <summary>Helper class for scattering nodes within the diagram</summary>
	public sealed class NodeScatterer :IDisposable
	{
		private readonly ChartControl m_diagram;
		private readonly Diagram_.Options m_opts;
		private readonly List<Link> m_springs;
		private readonly List<Body> m_body;
		private DispatcherTimer m_timer;
		private readonly Random m_rng;
		private int m_issue;

		public NodeScatterer(ChartControl diagram, Diagram_.Options opts)
		{
			m_diagram = diagram;
			m_opts = opts;
			m_rng = new Random();
			m_springs = new List<Link>();
			m_body = new List<Body>();
			m_timer = new DispatcherTimer(TimeSpan.FromMilliseconds(10), DispatcherPriority.Background, Step, Dispatcher.CurrentDispatcher);

			Init();

			// Start the sim
			m_timer.Start();
		}
		public void Dispose()
		{
			m_timer.Stop();
		}

		/// <summary>True when the forces balance</summary>
		public bool Equalibrium { get; private set; }

		/// <summary>Create the bodies, springs, and charges from the nodes in the chart</summary>
		private void Init()
		{
			m_issue = m_diagram.Elements.IssueNumber;

			// Create a dynamics object for each node
			var nodes = m_diagram.Elements.OfType<Node>()
				.Where(x => x.Chart != null)
				.ToDictionary(x => x, x => new Body(x));

			// Find the unique set of inter-node connections
			var conns = nodes.Values.SelectMany(x => x.Node.Connectors)
				.Where(x => x.Chart != null)                                            // is in the diagram
				.Where(x => !x.Dangling)                                                // both ends connected
				.Where(x => nodes.ContainsKey(x.Node0!) && nodes.ContainsKey(x.Node1!)) // both ends are in the diagram
				.Distinct(Eql<Connector>.From(Connector.HasSameNodes));                 // only one per pair of nodes

			// Cache the nodes and springs
			m_body.Assign(nodes.Values);
			m_springs.Assign(conns.Select(x => new Link(nodes[x.Node0!], nodes[x.Node1!])));
		}

		/// <summary>Step the force simulation</summary>
		private void Step(object? sender, EventArgs e)
		{
			// Recreate the network of bodies, springs, and charges whenever the Elements collection changes
			if (m_issue != m_diagram.Elements.IssueNumber)
				Init();

			// On each tick, step the scattering integrator and invalidate the display
			const float dt = 0.05f;
			CalculateForces();
			Integrate(dt);

			// Apply the changes in body positions to the nodes
			foreach (var body in m_body)
			{
				// Treat selected nodes as immoveable
				if (body.Node.Selected)
				{
					body.Vel = v4.Zero;
					body.Pos = body.Node.O2W.pos;
				}
				else
				{
					body.Node.O2W = m4x4.Translation(body.Pos);
				}
			}

			// Redraw the chart
			m_diagram.Invalidate();
		}

		/// <summary>Enumerates all pairs of bodies that experience a repulsive force between them</summary>
		private IEnumerable<Link> Charges
		{
			get
			{
				if (m_body.Count < 2) yield break;
				for (int i = 0; i != m_body.Count - 1; ++i)
					for (int j = i + 1; j != m_body.Count; ++j)
						yield return new Link(m_body[i], m_body[j]);
			}
		}

		/// <summary>Accumulate forces on the bodies</summary>
		private void CalculateForces()
		{
			// This is just a balance between a force that is linear with distance
			// and a force that is quadratic with distance. Since spring simulations
			// easily become unstable, use a modified spring force function.

			// Determine coulomb forces
			foreach (var link in Charges)
			{
				var body0 = link.Body0;
				var body1 = link.Body1;

				// Find the minimum separation and the current separation
				Separation(body0, body1, out var sep, out var min_sep);

				// A coulomb force (F = kQq/r^2) is quadratically proportional to separation distance.
				// To handle nodes of unknown sizes, set all nodes to have the same charge, regardless of size
				// but make the separation equal to the distance between nearest points. Use charge of 1.
				var dist = Math_.Max(sep.Length - min_sep, 1.0);
				var coulumb = m_opts.Scatter.CoulombConstant / (dist * dist);

				// Add the forces to each node
				var f = coulumb * Math_.Normalise(sep, v4.Zero);
				Debug.Assert(Math_.IsFinite(f));
				body0.Force -= f;
				body1.Force += f;
			}

			// Determine spring forces
			foreach (var link in m_springs)
			{
				var body0 = link.Body0;
				var body1 = link.Body1;

				// Find the minimum separation and the current separation
				Separation(body0, body1, out var sep, out var min_sep);

				// A spring force (F = -Kx) is linearly proportional to the deviation from the rest length.
				// To handle nodes of unknown sizes, set the rest length to 'min_sep'. To stop the
				// simulation blowing up, make the force zero when less than 'min_sep' and a constant
				// when above some maximum separation.
				var dist = Math_.Clamp(sep.Length - min_sep, -10 * min_sep, 10 * min_sep);
				var spring = -m_opts.Scatter.SpringConstant * dist;

				// Add the forces to each node
				var f = spring * Math_.Normalise(sep, v4.Zero);
				Debug.Assert(Math_.IsFinite(f));
				body0.Force -= f;
				body1.Force += f;
			}

			// Friction forces
			foreach (var body in m_body)
			{
				var friction = m_opts.Scatter.FrictionConstant * body.Vel;
				body.Force -= friction;
			}
		}

		/// <summary>Advance the simulation</summary>
		private void Integrate(float dt)
		{
			Equalibrium = true;

			// Apply forces (2nd order integrator)
			const float mass = 1f;
			foreach (var body in m_body)
			{
				var a = body.Force / mass;
				var v = body.Vel + 0.5f * dt * a;

				Equalibrium &= a.LengthSq < Math_.Sqr(m_opts.Scatter.Equilibrium);

				body.Pos += v * dt + 0.5f * a * Math_.Sqr(dt);
				body.Vel += a;
				body.Force = v4.Zero;
			}

			// Ensure the centroid is always at (0,0,0)
			var centre = v4.Zero;
			foreach (var body in m_body)
				centre += body.Pos.w0;
			centre /= m_body.Count;
			foreach (var body in m_body)
				body.Pos -= centre;
		}

		/// <summary>Return the separation vector between the two bodies and the minimum distance in that direction to separate the bodies.</summary>
		private void Separation(Body b0, Body b1, out v4 sep, out float min_dist)
		{
			var vec = b1.Pos - b0.Pos;
			var size = b1.Size + b0.Size;

			// Treat nodes as AABBs. This function needs to be tolerant of overlapping nodes.
			// In 2D, separate the nodes in the camera view plane. In 3D separate the 3d boxes.
			switch (m_diagram.Options.NavigationMode)
			{
				case ChartControl.ENavMode.Scene3D:
				{
					// 'pen' is the penetration depth. Positive for penetration.
					var pen = size - Math_.Abs(vec);

					// The separating axis is the axis with the minimum penetration depth.
					var i = Math_.MinElementIndex(pen.xyz);
					if (pen[i] < 0) // Not overlapping
					{
						sep = vec;
						min_dist = size.Length;
					}
					else
					{
						// Push out of penetration first
						sep = v4.Zero;
						sep[i] = vec[i];
						min_dist = size[i];
					}

					// Sep can be zero if the nodes are 2D. The zero-dimension will be the minimum penetration
					if (sep.LengthSq < Math_.TinyF)
						sep = v4.Random3N(0f, m_rng);

					break;
				}
				case ChartControl.ENavMode.Chart2D:
				{
					// Assume the AABBs are aligned with the camera (so 'size' does not need rotating)
					sep = m_diagram.Camera.W2O * vec;
					sep.z = 0f;

					if (sep.LengthSq < Math_.TinyF)
						sep = v4.Random2N(0f, 0f, m_rng);

					// Find the minimum distance along 'sep' needed to separate the bodies
					min_dist = Math_.Dot(0.5f * size.xy, Math_.Abs(sep.xy)) / sep.xy.Length;
					sep = m_diagram.Camera.O2W * sep;
					break;
				}
				default:
				{
					throw new Exception("Unsupported navigation mode");
				}
			}
		}

		/// <summary>Wrapper of 'Node'</summary>
		[DebuggerDisplay("{Node}")]
		private class Body
		{
			public Body(Node node)
			{
				Node = node;
				LinkCount = node.Connectors
					.Where(x => !x.Dangling && !x.Loop)
					.Select(x => x.OtherNode(node))
					.Distinct().Count();

				Pos = node.O2W.pos;
				Vel = v4.Zero;
				Force = v4.Zero;
			}

			/// <summary>The wrapped node</summary>
			public Node Node;

			/// <summary>The number of unique nodes this node is connected to</summary>
			public int LinkCount;

			/// <summary>The width/height of the node</summary>
			public v4 Size => Node.Size;

			/// <summary>The position of the node during scattering</summary>
			public v4 Pos;

			/// <summary>The velocity of the node during scattering</summary>
			public v4 Vel;

			/// <summary>Scattering force</summary>
			public v4 Force;
		}
		private class Link
		{
			public Link(Body body0, Body body1)
			{
				Body0 = body0;
				Body1 = body1;
			}

			/// <summary>The connected bodies</summary>
			public Body Body0;
			public Body Body1;
		}
	}
}
