using System;
using System.Collections.Generic;
using System.Linq.Expressions;
using System.Text.RegularExpressions;
using Rylogic.Maths;

namespace Rylogic.Common
{
	public class ExprLambda
	{
		/// <summary>Convert a maths equation of one variable into a lambda expression</summary>
		public static Func<double, double> Create(string expr, string variable_name = "x")
		{
			var vars = new Vars(variable_name);
			var result = new Expression[1];
			if (!string.IsNullOrEmpty(expr))
			{
				var i = 0;
				if (!Eval(expr, ref i, vars, result, 0, ETok.None))
					throw new Exception("Expression syntax error");
			}
			else
			{
				result[0] = Expression.Constant(0.0);
			}
			
			// Compile the lambda
			return Expression.Lambda<Func<double, double>>(result[0], vars[variable_name]).Compile();
		}

		/// <summary>Recursive expression to expression tree</summary>
		private static bool Eval(string expr, ref int i, Vars vars, Expression[] result, int ridx, ETok parent_op, bool l2r = true)
		{
			// Each time round the loop should result in a value.
			// Operation tokens result in recursive calls.
			bool follows_value = false;
			for (; i != expr.Length;)
			{
				var tok = Token(expr, ref i, out var val, follows_value, vars);
				follows_value = true;

				// If the next token has lower precedence than the parent operation
				// then return to allow the parent op to evaluate
				var prec0 = Precedence(tok);
				var prec1 = Precedence(parent_op);
				if (prec0 < prec1)
					return true;
				if (prec0 == prec1 && l2r)
					return true;

				switch (tok)
				{
					case ETok.None:
					{
						return i == expr.Length;
					}
					case ETok.Value:
					{
						result[ridx] = Expression.Constant(val);
						break;
					}
					case ETok.Variable:
					{
						result[ridx] = (Expression?)val ?? throw new Exception("Variable not found");
						break;
					}
					case ETok.Add:
					{
						++i;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.Add(result[ridx], rhs[0]);
						break;
					}
					case ETok.Sub:
					{
						++i;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.Subtract(result[ridx], rhs[0]);
						break;
					}
					case ETok.Mul:
					{
						++i;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.Multiply(result[ridx], rhs[0]);
						break;
					}
					case ETok.Div:
					{
						++i;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.Divide(result[ridx], rhs[0]);
						break;
					}
					case ETok.Mod:
					{
						++i;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.Modulo(result[ridx], rhs[0]);
						break;
					}
					case ETok.UnaryAdd:
					{
						++i;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok, false)) return false;
						result[ridx] = Expression.UnaryPlus(rhs[0]);
						break;
					}
					case ETok.UnaryMinus:
					{
						++i;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok, false)) return false;
						result[ridx] = Expression.Negate(rhs[0]);
						break;
					}
					case ETok.Comp:
					{
						++i;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok, false)) return false;
						result[ridx] = Expression.OnesComplement(rhs[0]);
						break;
					}
					case ETok.Not:
					{
						++i;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok, false)) return false;
						result[ridx] = Expression.Not(rhs[0]);
						break;
					}
					case ETok.LogOR:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.OrElse(result[ridx], rhs[0]);
						break;
					}
					case ETok.LogAND:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.AndAlso(result[ridx], rhs[0]);
						break;
					}
					case ETok.BitOR:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.Or(result[ridx], rhs[0]);
						break;
					}
					case ETok.BitXOR:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.ExclusiveOr(result[ridx], rhs[0]);
						break;
					}
					case ETok.BitAND:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.And(result[ridx], rhs[0]);
						break;
					}
					case ETok.LogEql:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.Equal(result[ridx], rhs[0]);
						break;
					}
					case ETok.LogNEql:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.NotEqual(result[ridx], rhs[0]);
						break;
					}
					case ETok.LogLT:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.LessThan(result[ridx], rhs[0]);
						break;
					}
					case ETok.LogLTEql:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.LessThanOrEqual(result[ridx], rhs[0]);
						break;
					}
					case ETok.LogGT:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.GreaterThan(result[ridx], rhs[0]);
						break;
					}
					case ETok.LogGTEql:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.GreaterThanOrEqual(result[ridx], rhs[0]);
						break;
					}
					case ETok.LeftShift:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.LeftShift(result[ridx], rhs[0]);
						break;
					}
					case ETok.RightShift:
					{
						i += 2;
						var rhs = new Expression[1];
						if (!Eval(expr, ref i, vars, rhs, 0, tok)) return false;
						result[ridx] = Expression.RightShift(result[ridx], rhs[0]);
						break;
					}
					case ETok.Fmod:
					{
						i += 4;
						var args = new Expression[2];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null || args[1] == null) throw new Exception("insufficient parameters for 'fmod'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.IEEERemainder), null, args[0], args[1]);
						break;
					}
					case ETok.Ceil:
					{
						i += 4;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'ceil'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Ceiling), null, args[0]);
						break;
					}
					case ETok.Floor:
					{
						i += 5;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'floor'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Floor), null, args[0]);
						break;
					}
					case ETok.Round:
					{
						i += 5;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'round'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Round), null, args[0]);
						break;
					}
					case ETok.Min:
					{
						i += 3;
						var args = new Expression[2];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null || args[1] == null) throw new Exception("insufficient parameters for 'min'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Min), null, args[0], args[1]);
						break;
					}
					case ETok.Max:
					{
						i += 3;
						var args = new Expression[2];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null || args[1] == null) throw new Exception("insufficient parameters for 'max'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Max), null, args[0], args[1]);
						break;
					}
					case ETok.Clamp:
					{
						i += 5;
						var args = new Expression[3];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null || args[1] == null || args[2] == null) throw new Exception("insufficient parameters for 'clamp'");
						result[ridx] = Expression.Call(typeof(Math_), nameof(Math_.Clamp), new[] { args[0].Type }, args[0], args[1], args[2]);
						break;
					}
					case ETok.Abs:
					{
						i += 3;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'abs'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Abs), null, args[0]);
						break;
					}
					case ETok.Sin:
					{
						i += 3;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'sin'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Sin), null, args[0]);
						break;
					}
					case ETok.Cos:
					{
						i += 3;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'cos'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Cos), null, args[0]);
						break;
					}
					case ETok.Tan:
					{
						i += 3;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'tan'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Tan), null, args[0]);
						break;
					}
					case ETok.ASin:
					{
						i += 4;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'asin'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Asin), null, args[0]);
						break;
					}
					case ETok.ACos:
					{
						i += 4;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'acos'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Acos), null, args[0]);
						break;
					}
					case ETok.ATan:
					{
						i += 4;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'atan'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Atan), null, args[0]);
						break;
					}
					case ETok.ATan2:
					{
						i += 5;
						var args = new Expression[2];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null || args[1] == null) throw new Exception("insufficient parameters for 'atan2'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Atan2), null, args[0], args[1]);
						break;
					}
					case ETok.SinH:
					{
						i += 4;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'sinh'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Sinh), null, args[0]);
						break;
					}
					case ETok.CosH:
					{
						i += 4;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'cosh'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Cosh), null, args[0]);
						break;
					}
					case ETok.TanH:
					{
						i += 4;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'tanh'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Tanh), null, args[0]);
						break;
					}
					case ETok.Exp:
					{
						i += 3;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'exp'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Exp), null, args[0]);
						break;
					}
					case ETok.Log:
					{
						i += 3;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'log'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Log), null, args[0]);
						break;
					}
					case ETok.Log10:
					{
						i += 5;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'log10'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Log10), null, args[0]);
						break;
					}
					case ETok.Pow:
					{
						i += 3;
						var args = new Expression[2];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null || args[1] == null) throw new Exception("insufficient parameters for 'pow'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Pow), null, args[0], args[1]);
						break;
					}
					case ETok.Sqr:
					{
						i += 3;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'sqr'");
						result[ridx] = Expression.Call(typeof(Math_), nameof(Math_.Sqr), null, args[0]);
						break;
					}
					case ETok.Sqrt:
					{
						i += 4;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'sqrt'");
						result[ridx] = Expression.Call(typeof(Math), nameof(Math.Sqrt), null, args[0]);
						break;
					}
					case ETok.Len2:
					{
						i += 4;
						var args = new Expression[2];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null || args[1] == null) throw new Exception("insufficient parameters for 'len2'");
						result[ridx] = Expression.Call(typeof(Math_), nameof(Math_.Length0), null, args[0], args[1]);
						break;
					}
					case ETok.Len3:
					{
						i += 4;
						var args = new Expression[3];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null || args[1] == null || args[2] == null) throw new Exception("insufficient parameters for 'len3'");
						result[ridx] = Expression.Call(typeof(Math_), nameof(Math_.Length0), null, args[0], args[1], args[2]);
						break;
					}
					case ETok.Len4:
					{
						i += 4;
						var args = new Expression[4];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null || args[1] == null || args[2] == null || args[3] == null) throw new Exception("insufficient parameters for 'len4'");
						result[ridx] = Expression.Call(typeof(Math_), nameof(Math_.Length0), null, args[0], args[1], args[2], args[3]);
						break;
					}
					case ETok.Deg:
					{
						i += 3;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'deg'");
						result[ridx] = Expression.Call(typeof(Math_), nameof(Math_.RadiansToDegrees), null, args[0]);
						break;
					}
					case ETok.Rad:
					{
						i += 3;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, args, 0, tok)) return false;
						if (args[0] == null) throw new Exception("insufficient parameters for 'rad'");
						result[ridx] = Expression.Call(typeof(Math_), nameof(Math_.DegreesToRadians), null, args[0]);
						break;
					}
					case ETok.Comma:
					{
						if (ridx + 1 == result.Length)
							throw new Exception("too many parameters");

						++i;
						++ridx;
						follows_value = false;
						break;
					}
					case ETok.OpenParenthesis:
					{
						// Parent op is 'None' because it has the lowest precedence
						++i;
						if (!Eval(expr, ref i, vars, result, ridx, ETok.None)) return false;
						break;
					}
					case ETok.CloseParenthesis:
					{
						// Wait for the parent op to be the 'Open Parenthesis'
						if (parent_op == ETok.None) ++i;
						return true;
					}
					case ETok.If:
					{
						++i;
						var args = new Expression[2];
						if (!Eval(expr, ref i, vars, args, 0, ETok.None)) return false;
						if (args[0] == null || args[1] == null) throw new Exception("incomplete if-else");
						result[ridx] = Expression.Condition(result[ridx], args[0], args[1]);
						break;
					}
					case ETok.Else:
					{
						++i;
						var args = new Expression[1];
						if (!Eval(expr, ref i, vars, result, ++ridx, ETok.Else)) return false;
						++ridx;
						return true;
					}
					default:
					{
						throw new Exception($"unknown expression token: {tok}");
					}
				}
			}
			return true;
		}

		/// <summary>
		/// Extract a token from 'expr'
		/// If the token is a value then 'expr' is advanced past the value
		/// if it's an operator it isn't. This is so that operator precedence works
		/// 'follows_value' should be true if the preceding expression evaluates to a value</summary>
		private static ETok Token(string expr, ref int i, out object? val, bool follows_value, Vars vars)
		{
			val = null;

			// Skip any leading whitespace
			for (; i != expr.Length && char.IsWhiteSpace(expr[i]); ++i) { }
			if (i == expr.Length) return ETok.None;

			// String compare helper
			static bool cmp(string lhs, int idx, string rhs, int len) => string.Compare(lhs, idx, rhs, 0, len, true) == 0;

			// Look for an operator
			switch (char.ToLowerInvariant(expr[i]))
			{
			// Convert Add/Sub to unary plus/minus by looking at the previous expression
			// If the previous expression evaluates to a value then Add/Sub are binary expressions
			case '+': return follows_value ? ETok.Add : ETok.UnaryAdd;
			case '-': return follows_value ? ETok.Sub : ETok.UnaryMinus;
			case '*': return ETok.Mul;
			case '/': return ETok.Div;
			case '%': return ETok.Mod;
			case '~': return ETok.Comp;
			case ',': return ETok.Comma;
			case '^': return ETok.BitXOR;
			case '(': return ETok.OpenParenthesis;
			case ')': return ETok.CloseParenthesis;
			case '?': return ETok.If;
			case ':': return ETok.Else;
			case '<':
				if (cmp(expr, i, "<<", 2)) return ETok.LeftShift;
				if (cmp(expr, i, "<=", 2)) return ETok.LogLTEql;
				return ETok.LogLT;
			case '>':
				if (cmp(expr, i, ">>", 2)) return ETok.RightShift;
				if (cmp(expr, i, ">=", 2)) return ETok.LogGTEql;
				return ETok.LogGT;
			case '|':
				if (cmp(expr, i, "||", 2)) return ETok.LogOR;
				return ETok.BitOR;
			case '&':
				if (cmp(expr, i, "&&", 2)) return ETok.LogAND;
				return ETok.BitAND;
			case '=':
				if (cmp(expr, i, "==", 2)) return ETok.LogEql;
				break;
			case '!':
				if (cmp(expr, i, "!=", 2)) return ETok.LogNEql;
				return ETok.Not;
			case 'a':
				if (cmp(expr, i, "abs", 3)) return ETok.Abs;
				if (cmp(expr, i, "asin", 4)) return ETok.ASin;
				if (cmp(expr, i, "acos", 4)) return ETok.ACos;
				if (cmp(expr, i, "atan2", 5)) return ETok.ATan2;
				if (cmp(expr, i, "atan", 4)) return ETok.ATan;
				break;
			case 'c':
				if (cmp(expr, i, "clamp", 5)) return ETok.Clamp;
				if (cmp(expr, i, "ceil", 4)) return ETok.Ceil;
				if (cmp(expr, i, "cosh", 4)) return ETok.CosH;
				if (cmp(expr, i, "cos", 3)) return ETok.Cos;
				break;
			case 'd':
				if (cmp(expr, i, "deg", 3)) return ETok.Deg;
				break;
			case 'e':
				if (cmp(expr, i, "exp", 3)) return ETok.Exp;
				break;
			case 'f':
				if (cmp(expr, i, "floor", 5)) return ETok.Floor;
				if (cmp(expr, i, "fmod", 3)) return ETok.Fmod;
				if (cmp(expr, i, "false", 5)) { i += 5; val = 0.0; return ETok.Value; }
				break;
			case 'l':
				if (cmp(expr, i, "log10", 5)) return ETok.Log10;
				if (cmp(expr, i, "log", 3)) return ETok.Log;
				if (cmp(expr, i, "len2", 4)) return ETok.Len2;
				if (cmp(expr, i, "len3", 4)) return ETok.Len3;
				if (cmp(expr, i, "len4", 4)) return ETok.Len4;
				break;
			case 'm':
				if (cmp(expr, i, "min", 3)) return ETok.Min;
				if (cmp(expr, i, "max", 3)) return ETok.Max;
				break;
			case 'p':
				if (cmp(expr, i, "pow", 3)) return ETok.Pow;
				if (cmp(expr, i, "phi", 3)) { i += 3; val = Math_.Phi; return ETok.Value; }
				if (cmp(expr, i, "pi", 2)) { i += 2; val = Math_.Tau / 2.0; return ETok.Value; }
				break;
			case 'r':
				if (cmp(expr, i, "round", 5)) return ETok.Round;
				if (cmp(expr, i, "rad", 3)) return ETok.Rad;
				break;
			case 's':
				if (cmp(expr, i, "sinh", 4)) return ETok.SinH;
				if (cmp(expr, i, "sin", 3)) return ETok.Sin;
				if (cmp(expr, i, "sqrt", 4)) return ETok.Sqrt;
				if (cmp(expr, i, "sqr", 3)) return ETok.Sqr;
				break;
			case 't':
				if (cmp(expr, i, "tanh", 4)) return ETok.TanH;
				if (cmp(expr, i, "tan", 3)) return ETok.Tan;
				if (cmp(expr, i, "tau", 3)) { i += 3; val = Math_.Tau; return ETok.Value; }
				if (cmp(expr, i, "true", 4)) { i += 4; val = 1.0; return ETok.Value; }
				break;
			default:
				break;
			}

			// If it's not an operator, try extracting a variable name
			// Variables must be identifiers which are a letter or '_', followed by letters, digits, or '_'
			if (char.IsLetter(expr[i]) || expr[i] == '_')
			{
				int j; for (j = i + 1; j != expr.Length && (char.IsLetterOrDigit(expr[j]) || expr[j] == '_'); ++j) { }
				var variable = expr.Substring(i, j - i);
				if (vars.TryGetValue(variable, out var parm))
				{
					val = parm;
					i += variable.Length;
					return ETok.Variable;
				}
			}
			// If not a variable, look for an operand
			else
			{
				var m = Regex.Match(expr.Substring(i), @"^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?");
				if (m.Success)
				{
					val = double.Parse(m.Value);
					i += m.Value.Length;
					return ETok.Value;
				}
			}
			return ETok.None;
		}

		/// <summary>
		/// Returns the precedence of a token
		/// How to work out precedence:
		///   NewOp = the op whose precedence you want to know
		///   RhsOp = an op in this list
		/// Ask, "If I encounter RhsOp next after NewOp, should NewOp go on hold while
		///  RhsOp is evaluated, or should I stop and evaluate up to NewOp before carrying on".
		///  Also the visa versa case.
		/// If NewOp should go on hold, then it has lower precedence (i.e. NewOp < RhsOp)
		/// If NewOp needs evaluating, then RhsOp has lower precedence (i.e. RhsOp > NewOp)</summary>
		private static int Precedence(ETok tok)
		{
			return tok switch
			{
				ETok.None => 0,
				ETok.Comma => 20,
				ETok.If => 30,
				ETok.Else => 30,
				ETok.LogOR => 40,
				ETok.LogAND => 50,
				ETok.BitOR => 60,
				ETok.BitXOR => 70,
				ETok.BitAND => 80,
				ETok.LogEql => 90,
				ETok.LogNEql => 90,
				ETok.LogLT => 100,
				ETok.LogLTEql => 100,
				ETok.LogGT => 100,
				ETok.LogGTEql => 100,
				ETok.LeftShift => 110,
				ETok.RightShift => 110,
				ETok.Add => 120,
				ETok.Sub => 120,
				ETok.Mul => 130,
				ETok.Div => 130,
				ETok.Mod => 130,
				ETok.UnaryAdd => 140,
				ETok.UnaryMinus => 140,
				ETok.Comp => 140,
				ETok.Not => 140,
				ETok.Fmod => 200,
				ETok.Abs => 200,
				ETok.Ceil => 200,
				ETok.Floor => 200,
				ETok.Round => 200,
				ETok.Min => 200,
				ETok.Max => 200,
				ETok.Clamp => 200,
				ETok.Sin => 200,
				ETok.Cos => 200,
				ETok.Tan => 200,
				ETok.ASin => 200,
				ETok.ACos => 200,
				ETok.ATan => 200,
				ETok.ATan2 => 200,
				ETok.SinH => 200,
				ETok.CosH => 200,
				ETok.TanH => 200,
				ETok.Exp => 200,
				ETok.Log => 200,
				ETok.Log10 => 200,
				ETok.Pow => 200,
				ETok.Sqr => 200,
				ETok.Sqrt => 200,
				ETok.Len2 => 200,
				ETok.Len3 => 200,
				ETok.Len4 => 200,
				ETok.Deg => 200,
				ETok.Rad => 200,
				ETok.OpenParenthesis => 300,
				ETok.CloseParenthesis => 300,
				ETok.Value => 1000,
				ETok.Variable => 1000,
				_ => throw new Exception($"Unknown token: {tok}"),
			};
		}

		/// <summary>Expression tokens</summary>
		private enum ETok
		{
			None,
			If,
			Else,
			Comma,
			LogOR,
			LogAND,
			BitOR,
			BitXOR,
			BitAND,
			LogEql,
			LogNEql,
			LogLT,
			LogLTEql,
			LogGT,
			LogGTEql,
			LeftShift,
			RightShift,
			Add,
			Sub,
			Mul,
			Div,
			Mod,
			UnaryAdd,
			UnaryMinus,
			Comp,
			Not,
			Fmod,
			Abs,
			Ceil,
			Floor,
			Round,
			Min,
			Max,
			Clamp,
			Sin,
			Cos,
			Tan,
			ASin,
			ACos,
			ATan,
			ATan2,
			SinH,
			CosH,
			TanH,
			Exp,
			Log,
			Log10,
			Pow,
			Sqr,
			Sqrt,
			Len2,
			Len3,
			Len4,
			Deg,
			Rad,
			OpenParenthesis,
			CloseParenthesis,
			Value,
			Variable,
		};

		/// <summary>Map from variable name to parameter expression</summary>
		private class Vars :Dictionary<string, ParameterExpression>
		{
			public Vars(params string[] variables)
			{
				foreach (var variable in variables)
					Add(variable, Expression.Parameter(typeof(double), variable));
			}
		}
	}
}


#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;
	using Rylogic.Common;

	[TestFixture]
	public class TestExprLamdba
	{
		[Test]
		public void ExprToLambda()
		{
			{
				var func = ExprLambda.Create("3 + 4");
				var res = func(0);
				Assert.Equal(7.0, res);
			}
			{
				var func = ExprLambda.Create("-x - 2*+x - x + 2/x % 3");
				var res = func(4);
				Assert.Equal(-15.5, res);
			}
			{
				var func = ExprLambda.Create("sqr(sqrt(x)*-abs(x%2)/15.0-tan(TAU/-6*x))");
				var res = func(12.34);
				Assert.Equal(0.08542317787694452, res);
			}
		}
	}
}
#endif
