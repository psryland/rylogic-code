using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media;
using ICSharpCode.AvalonEdit;
using ICSharpCode.AvalonEdit.CodeCompletion;
using ICSharpCode.AvalonEdit.Document;
using ICSharpCode.AvalonEdit.Editing;

namespace Rylogic.LDrawVisualiser.Core
{
	/// <summary>Provides C# autocomplete suggestions for the LDraw Builder API</summary>
	public static class CompletionProvider
	{
		/// <summary>Get member completions after a '.' character</summary>
		public static IEnumerable<CSharpCompletionData> GetMemberCompletions(TextEditor editor, int offset)
		{
			// Find the identifier before the dot
			var identifier = GetPrecedingIdentifier(editor.Document, offset - 1);

			// Builder factory methods (after 'b.' or 'builder.')
			if (IsBuilderVariable(identifier))
				return BuilderMethods;

			// Common Ldr object methods (fluent chain after .Box(), .Sphere(), etc.)
			if (IsLdrObjectType(identifier) || IsFluentChainContext(editor.Document, offset))
				return LdrObjectMethods;

			// After 'vars.' — no completions (debug variables are dynamic)
			if (string.Equals(identifier, "vars", StringComparison.OrdinalIgnoreCase))
				return [];

			// Default: show common members
			return CommonMembers;
		}

		/// <summary>Get keyword completions at word start</summary>
		public static IEnumerable<CSharpCompletionData> GetKeywordCompletions(TextEditor editor, int offset)
		{
			// Get the partial word being typed
			var partial = GetCurrentWord(editor.Document, offset);
			if (string.IsNullOrEmpty(partial))
				return [];

			return AllKeywords.Where(k => k.Text.StartsWith(partial, StringComparison.OrdinalIgnoreCase));
		}

		private static string GetPrecedingIdentifier(TextDocument doc, int dot_offset)
		{
			if (dot_offset <= 0)
				return string.Empty;

			// Walk backward from just before the dot
			var end = dot_offset;
			var pos = end - 1;
			while (pos >= 0 && (char.IsLetterOrDigit(doc.GetCharAt(pos)) || doc.GetCharAt(pos) == '_'))
				pos--;

			return doc.GetText(pos + 1, end - pos - 1);
		}

		private static string GetCurrentWord(TextDocument doc, int offset)
		{
			var pos = offset - 1;
			while (pos >= 0 && (char.IsLetterOrDigit(doc.GetCharAt(pos)) || doc.GetCharAt(pos) == '_'))
				pos--;

			return doc.GetText(pos + 1, offset - pos - 1);
		}

		private static bool IsBuilderVariable(string name) =>
			name is "b" or "builder" or "Builder";

		private static bool IsLdrObjectType(string name) =>
			name.StartsWith("Ldr", StringComparison.Ordinal) ||
			LdrTypeNames.Contains(name);

		private static bool IsFluentChainContext(TextDocument doc, int offset)
		{
			// Check if we're in a fluent chain by looking for common method names before the dot
			var preceding = GetPrecedingIdentifier(doc, offset - 1);
			return FluentMethodNames.Contains(preceding);
		}

		// Builder factory methods
		private static readonly CSharpCompletionData[] BuilderMethods =
		[
			new("Box", "Box(name?, colour?) — Create a box object"),
			new("Chart", "Chart(name?, colour?) — Create a chart object"),
			new("Circle", "Circle(name?, colour?) — Create a circle object"),
			new("Cone", "Cone(name?, colour?) — Create a cone object"),
			new("ConvexHull", "ConvexHull(name?, colour?) — Create a convex hull"),
			new("CoordFrame", "CoordFrame(name?, colour?) — Create a coordinate frame"),
			new("Cylinder", "Cylinder(name?, colour?) — Create a cylinder object"),
			new("Equation", "Equation(name?, colour?) — Create an equation surface"),
			new("Frustum", "Frustum(name?, colour?) — Create a frustum object"),
			new("Grid", "Grid(name?, colour?) — Create a grid object"),
			new("Group", "Group(name?, colour?) — Create a group container"),
			new("Instance", "Instance(name?, colour?) — Create an instance reference"),
			new("LightSource", "LightSource(name?, colour?) — Create a light source"),
			new("Line", "Line(name?, colour?) — Create a line object"),
			new("LineBox", "LineBox(name?, colour?) — Create a wireframe box"),
			new("Mesh", "Mesh(name?, colour?) — Create a mesh object"),
			new("Model", "Model(name?, colour?) — Create an FBX/OBJ model"),
			new("Pie", "Pie(name?, colour?) — Create a pie/arc shape"),
			new("Plane", "Plane(name?, colour?) — Create an infinite plane"),
			new("Point", "Point(name?, colour?) — Create a point sprite"),
			new("Polygon", "Polygon(name?, colour?) — Create a polygon"),
			new("Quad", "Quad(name?, colour?) — Create a quad"),
			new("Rect", "Rect(name?, colour?) — Create a rectangle"),
			new("Ribbon", "Ribbon(name?, colour?) — Create a ribbon strip"),
			new("Sphere", "Sphere(name?, colour?) — Create a sphere object"),
			new("Text", "Text(name?, colour?) — Create a text label"),
			new("Triangle", "Triangle(name?, colour?) — Create a triangle"),
			new("Tube", "Tube(name?, colour?) — Create a tube/pipe"),
			new("Clear", "Clear(count?) — Reset the builder"),
			new("ToString", "ToString() — Generate the LDraw script string"),
		];

		// Fluent Ldr object methods (available on all LdrXxx types)
		private static readonly CSharpCompletionData[] LdrObjectMethods =
		[
			new("o2w", "o2w(m4x4) — Set object-to-world transform"),
			new("pos", "pos(x, y, z) — Set position"),
			new("ori", "ori(dir, axis) — Set orientation"),
			new("name", "name(string) — Set the object name"),
			new("colour", "colour(uint) — Set the object colour (AARRGGBB)"),
			new("box", "box(sx, sy, sz) — Set box dimensions"),
			new("sphere", "sphere(radius) — Set sphere radius"),
			new("cylinder", "cylinder(height, radius) — Set cylinder dimensions"),
			new("line", "line(x0,y0,z0, x1,y1,z1) — Set line endpoints"),
			new("point", "point(x, y, z) — Add a point"),
			new("width", "width(w) — Set line/ribbon width"),
			new("font", "font() — Set text font properties"),
			new("bake", "bake() — Bake transform into geometry"),
			new("wireframe", "wireframe(bool) — Set wireframe rendering"),
			new("hidden", "hidden(bool) — Set visibility"),
		];

		private static readonly CSharpCompletionData[] CommonMembers =
		[
			new("ToString", "ToString() — Convert to string"),
			new("Length", "Length — Array/string length"),
		];

		// All top-level keyword completions
		private static readonly CSharpCompletionData[] AllKeywords =
		[
			new("var", "var — Implicitly typed local variable"),
			new("vars", "vars — Access debugger variables (dynamic proxy)"),
			new("new", "new — Create new instance"),
			new("return", "return — Return a value"),
			new("Builder", "Builder — LDraw scene builder class"),
			new("true", "true — Boolean literal"),
			new("false", "false — Boolean literal"),
			new("null", "null — Null reference"),
			new("if", "if — Conditional statement"),
			new("else", "else — Else branch"),
			new("for", "for — Loop statement"),
			new("foreach", "foreach — Iterate over collection"),
			new("while", "while — While loop"),
			new("Math", "Math — System.Math class"),
			new("Convert", "Convert — Type conversion utilities"),
			new("string", "string — String type"),
			new("float", "float — Single-precision floating point"),
			new("double", "double — Double-precision floating point"),
			new("int", "int — 32-bit integer"),
		];

		private static readonly HashSet<string> LdrTypeNames = new(StringComparer.Ordinal)
		{
			"LdrBox", "LdrChart", "LdrCircle", "LdrCone", "LdrConvexHull",
			"LdrCoordFrame", "LdrCylinder", "LdrEquation", "LdrFrustum",
			"LdrGrid", "LdrGroup", "LdrInstance", "LdrLightSource", "LdrLine",
			"LdrLineBox", "LdrMesh", "LdrModel", "LdrPie", "LdrPlane",
			"LdrPoint", "LdrPolygon", "LdrQuad", "LdrRect", "LdrRibbon",
			"LdrSphere", "LdrText", "LdrTriangle", "LdrTube",
		};

		private static readonly HashSet<string> FluentMethodNames = new(StringComparer.OrdinalIgnoreCase)
		{
			"box", "sphere", "cylinder", "cone", "line", "point",
			"name", "colour", "o2w", "pos", "ori", "width", "bake",
			"Box", "Sphere", "Cylinder", "Cone", "Line", "Point",
			"Circle", "Grid", "Group", "Mesh", "Model", "Quad", "Rect",
			"Triangle", "Tube", "Ribbon", "Text", "Frustum", "Plane",
		};
	}

	/// <summary>A simple completion data item for AvalonEdit</summary>
	public class CSharpCompletionData : ICompletionData
	{
		public CSharpCompletionData(string text, string? description = null)
		{
			Text = text;
			Description = description;
		}

		public ImageSource? Image => null;
		public string Text { get; }
		public object Content => Text;
		public object? Description { get; }
		public double Priority => 1.0;

		public void Complete(TextArea text_area, ISegment completion_segment, EventArgs e)
		{
			text_area.Document.Replace(completion_segment, Text);
		}
	}
}
