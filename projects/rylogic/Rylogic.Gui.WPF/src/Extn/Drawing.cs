using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Media;
using Rylogic.Extn;
using Rylogic.Maths;

namespace Rylogic.Gui.WPF
{
	public static class Drawing_
	{
		// Notes:
		//  - System.Drawing is available in .NET core, System.Windows is not.
		//    So use System.Drawing primitive types for shared code. System.Windows
		//    primitive types are used by WPF however.

		public static System.Drawing.Point ToPointI(this Point pt)
		{
			return new System.Drawing.Point((int)Math.Round(pt.X), (int)Math.Round(pt.Y));
		}
		public static System.Drawing.PointF ToPointF(this Point pt)
		{
			return new System.Drawing.PointF((float)pt.X, (float)pt.Y);
		}
		public static Point ToPointD(this System.Drawing.Point pt)
		{
			return new Point(pt.X, pt.Y);
		}
		public static Point ToPointD(this System.Drawing.PointF pt)
		{
			return new Point(pt.X, pt.Y);
		}
		public static Point ToPointD(this v2 pt)
		{
			return new Point(pt.x, pt.y);
		}
		public static Vector ToVectorD(this v2 pt)
		{
			return new Vector(pt.x, pt.y);
		}
		public static Size ToSizeD(this v2 pt)
		{
			return new Size(pt.x, pt.y);
		}
		public static Point ToNormalisedScreenSpace(Point pt, double screen_width, double screen_height)
		{
			return new Point(pt.X / screen_width - 0.5, 0.5 - pt.Y / screen_height);
		}
		public static System.Drawing.Rectangle ToRectI(this Rect rect)
		{
			return new System.Drawing.Rectangle(
				(int)Math.Round(rect.X), (int)Math.Round(rect.Y),
				(int)Math.Round(rect.Width), (int)Math.Round(rect.Height));
		}
		public static System.Drawing.RectangleF ToRectF(this Rect rect)
		{
			return new System.Drawing.RectangleF((float)rect.X, (float)rect.Y, (float)rect.Width, (float)rect.Height);
		}
		public static Rect ToRectD(this System.Drawing.Rectangle rect)
		{
			return new Rect(rect.X, rect.Y, rect.Width, rect.Height);
		}
		public static Rect ToRectD(this System.Drawing.RectangleF rect)
		{
			return new Rect(rect.X, rect.Y, rect.Width, rect.Height);
		}

		/// <summary>Add XML serialisation support for graphics types</summary>
		public static XmlConfig SupportSystemDrawingCommonTypes(this XmlConfig cfg)
		{
			Xml_.ToMap[typeof(System.Drawing.Drawing2D.Matrix)] = (obj, node) =>
			{
				var mat = (System.Drawing.Drawing2D.Matrix)obj;
				node.Add(string.Join(" ", mat.Elements));
				return node;
			};
			Xml_.AsMap[typeof(System.Drawing.Drawing2D.Matrix)] = (elem, type, instance) =>
			{
				var v = float_.ParseArray(elem.Value);
				return new System.Drawing.Drawing2D.Matrix(v[0], v[1], v[2], v[3], v[4], v[5]);
			};
			return cfg;
		}

		/// <summary>Debugging check that an object is frozen</summary>
		[Conditional("DEBUG")]
		public static void CheckIsFrozen(Freezable f)
		{
			if (f == null || f.IsFrozen) return;
			Debug.WriteLine($"Performance warning: Not frozen: {f}");
		}
	}

	public static class Size_
	{
		/// <summary>Zero size</summary>
		public static Size Zero => new(0, 0);

		/// <summary>Infinite size</summary>
		public static Size Infinity => new(double.PositiveInfinity, double.PositiveInfinity);

		/// <summary>Convert to v2</summary>
		public static v2 ToV2(this Size s)
		{
			return new v2((float)s.Width, (float)s.Height);
		}

		/// <summary>Compare sizes for approximate equality</summary>
		public static bool FEql(Size lhs, Size rhs)
		{
			return 
				Math_.FEql(lhs.Width, rhs.Width) &&
				Math_.FEql(lhs.Height, rhs.Height);
		}
		public static bool FEqlRelative(Size lhs, Size rhs, double tol)
		{
			return
				Math_.FEqlRelative(lhs.Width, rhs.Width, tol) &&
				Math_.FEqlRelative(lhs.Height, rhs.Height, tol);
		}

		/// <summary>The component-wise minimum of two sizes</summary>
		public static Size Min(Size lhs, Size rhs)
		{
			return new Size(
				Math.Min(lhs.Width, rhs.Width),
				Math.Min(lhs.Height, rhs.Height));
		}

		/// <summary>The component-wise maximum of two sizes</summary>
		public static Size Max(Size lhs, Size rhs)
		{
			return new Size(
				Math.Max(lhs.Width, rhs.Width),
				Math.Max(lhs.Height, rhs.Height));
		}
	}

	public static class Rect_
	{
		/// <summary>Zero size</summary>
		public static Rect Zero => new(0, 0, 0, 0);

		/// <summary>Construct from LTRB</summary>
		public static Rect FromLTRB(double left, double top, double right, double bottom)
		{
			return new Rect(left, top, right - left, bottom - top);
		}

		/// <summary>Return the centre position of the rectangle</summary>
		public static Point Centre(this Rect rect)
		{
			return new Point(rect.Left + rect.Width * 0.5, rect.Top + rect.Height * 0.5);
		}

		/// <summary>Left/Centre point</summary>
		public static Point LeftCentre(this Rect rect)
		{
			return new Point(rect.Left, (rect.Top + rect.Bottom) * 0.5);
		}

		/// <summary>Right/Centre point</summary>
		public static Point RightCentre(this Rect rect)
		{
			return new Point(rect.Right, (rect.Top + rect.Bottom) * 0.5);
		}

		/// <summary>Top/Centre point</summary>
		public static Point TopCentre(this Rect rect)
		{
			return new Point((rect.Left + rect.Right) * 0.5, rect.Top);
		}

		/// <summary>Bottom/Centre point</summary>
		public static Point BottomCentre(this Rect rect)
		{
			return new Point((rect.Left + rect.Right) * 0.5, rect.Bottom);
		}

		/// <summary>Reduces the size of this rectangle by excluding the area 'x'. The result must be a rectangle or an exception is thrown</summary>
		public static Rect Subtract(this Rect r, Rect x)
		{
			// If 'x' has no area, then subtraction is identity
			if (x.Width * x.Height <= 0)
				return r;

			// If the rectangles do not overlap. Right/Bottom is not considered 'in' the rectangle
			if (r.Left >= x.Right || r.Right <= x.Left || r.Top >= x.Bottom || r.Bottom <= x.Top)
				return r;

			// If 'x' completely covers 'r' then the result is empty
			if (x.Left <= r.Left && x.Right >= r.Right && x.Top <= r.Top && x.Bottom >= r.Bottom)
				return Rect.Empty;

			// If 'x' spans 'r' horizontally
			if (x.Left <= r.Left && x.Right >= r.Right)
			{
				// If the top edge of 'r' is aligned with the top edge of 'x', or within 'x'
				// then the top edge of the resulting rectangle is 'x.Bottom'.
				if (x.Top <= r.Top) return new Rect(r.Left, x.Bottom, r.Right - r.Left, r.Bottom - x.Bottom);
				if (x.Bottom >= r.Bottom) return new Rect(r.Left, r.Top, r.Right - r.Left, x.Top - r.Top);
				throw new Exception("The result of subtracting rectangle 'x' does not result in a rectangle");
			}

			// If 'x' spans 'r' vertically
			if (x.Top <= r.Top && x.Bottom >= r.Bottom)
			{
				if (x.Left <= r.Left) return new Rect(x.Right, r.Top, r.Right - x.Right, r.Bottom - r.Top);
				if (x.Right >= r.Right) return new Rect(r.Left, r.Top, x.Left - r.Left, r.Bottom - r.Top);
				throw new Exception("The result of subtracting rectangle 'x' does not result in a rectangle");
			}

			throw new Exception("The result of subtracting rectangle 'x' does not result in a rectangle");
		}

		/// <summary>Reduces the size of this rectangle by thickness 'x'</summary>
		public static Rect ShrinkBy(this Rect r, Thickness x)
		{
			var dw = Math.Min(r.Width, x.Left + x.Right);
			var dh = Math.Min(r.Height, x.Top + x.Bottom);
			return new Rect(
				r.Left + x.Left,
				r.Top + x.Top,
				r.Width - dw,
				r.Height - dh);
		}

		/// <summary>Return the difference between two rectangles as a difference</summary>
		public static Thickness ToThickness(Rect l, Rect r)
		{
			return new Thickness(
				r.Left - l.Left,
				r.Top - l.Top,
				l.Right - r.Right,
				l.Bottom - r.Bottom);
		}

		/// <summary>Compare sizes for approximate equality</summary>
		public static bool FEql(Rect lhs, Rect rhs)
		{
			return
				Math_.FEql(lhs.X, rhs.X) &&
				Math_.FEql(lhs.Y, rhs.Y) &&
				Math_.FEql(lhs.Width, rhs.Width) &&
				Math_.FEql(lhs.Height, rhs.Height);
		}
		public static bool FEqlRelative(Rect lhs, Rect rhs, double tol)
		{
			return
				Math_.FEqlRelative(lhs.X, rhs.X, tol) &&
				Math_.FEqlRelative(lhs.Y, rhs.Y, tol) &&
				Math_.FEqlRelative(lhs.Width, rhs.Width, tol) &&
				Math_.FEqlRelative(lhs.Height, rhs.Height, tol);
		}
	}

	public static class Point_
	{
		/// <summary>(0,0)</summary>
		public static Point Zero => new();

		/// <summary>Infinite vector</summary>
		public static Point Infinity => new(double.PositiveInfinity, double.PositiveInfinity);

		/// <summary>Convert to v2</summary>
		public static v2 ToV2(this Point p)
		{
			return new v2((float)p.X, (float)p.Y);
		}

		/// <summary>Convert to Vector</summary>
		public static Vector ToVectorD(this Point p)
		{
			return new Vector(p.X, p.Y);
		}

		/// <summary>True if any component of the point is NaN</summary>
		public static bool IsNaN(Point v)
		{
			return double.IsNaN(v.X) || double.IsNaN(v.Y);
		}

		/// <summary>Compare sizes for approximate equality</summary>
		public static bool FEql(Point lhs, Point rhs)
		{
			return
				Math_.FEql(lhs.X, rhs.X) &&
				Math_.FEql(lhs.Y, rhs.Y);
		}
		public static bool FEqlRelative(Point lhs, Point rhs, double tol)
		{
			return
				Math_.FEqlRelative(lhs.X, rhs.X, tol) &&
				Math_.FEqlRelative(lhs.Y, rhs.Y, tol);
		}
	}

	public static class Vector_
	{
		/// <summary>Infinite vector</summary>
		public static Vector Infinity => new(double.PositiveInfinity, double.PositiveInfinity);

		/// <summary>Convert to v2</summary>
		public static v2 ToV2(this Vector v)
		{
			return new v2((float)v.X, (float)v.Y);
		}

		/// <summary>Convert to Point</summary>
		public static Point ToPointD(this Vector v)
		{
			return new Point(v.X, v.Y);
		}

		/// <summary>True if any component of the vector is NaN</summary>
		public static bool IsNaN(Vector v)
		{
			return double.IsNaN(v.X) || double.IsNaN(v.Y);
		}

		/// <summary>Compare sizes for approximate equality</summary>
		public static bool FEql(Vector lhs, Vector rhs)
		{
			return
				Math_.FEql(lhs.X, rhs.X) &&
				Math_.FEql(lhs.Y, rhs.Y);
		}
		public static bool FEqlRelative(Vector lhs, Vector rhs, double tol)
		{
			return
				Math_.FEqlRelative(lhs.X, rhs.X, tol) &&
				Math_.FEqlRelative(lhs.Y, rhs.Y, tol);
		}
	}

	public static class Transform_
	{
		/// <summary>Convert a matrix to a WPF media transform</summary>
		public static MatrixTransform ToTransform(this m4x4 o2w)
		{
			return new MatrixTransform(o2w.x.x, o2w.x.y, o2w.y.x, o2w.y.y, o2w.pos.x, o2w.pos.y);
		}
	}
}
