using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Controls;
using Rylogic.Common;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public class SliderFunction :Slider
	{
		// Usage:
		//   <gui:SliderFunction
		//       Minimum = "-2"
		//       Maximum="5"
		//       FunctionExpr="pow(10, x)"
		//       InverseExpr="x > 0 ? log10(x) : 1"
		//       ValueFn="{Binding Light.SpecularPower}"
		//       />

		public SliderFunction()
		{
			Function = x => x;
			Inverse = x => x;
		}
		protected override void OnValueChanged(double oldValue, double newValue)
		{
			if (m_in_value_changed != 0) return;
			using (Scope.Create(() => ++m_in_value_changed, () => --m_in_value_changed))
			{
				try { ValueFn = Function(newValue); }
				catch (Exception ex)
				{
					Debug.WriteLine($"SliderFunction 'Function' threw: {ex.Message}");
				}
			}
			base.OnValueChanged(oldValue, newValue);
		}
		private int m_in_value_changed;

		/// <summary>The function that converts from slider value to output value</summary>
		public Func<double, double> Function { get; set; }

		/// <summary>The function that converts from output values to slider values</summary>
		public Func<double, double> Inverse { get; set; }

		/// <summary>The slider value evaluated function output</summary>
		public double ValueFn
		{
			get => (double)GetValue(ValueFnProperty);
			set => SetValue(ValueFnProperty, value);
		}
		private void ValueFn_Changed()
		{
			if (m_in_value_changed != 0) return;
			using (Scope.Create(() => ++m_in_value_changed, () => --m_in_value_changed))
			{
				try { Value = Inverse(ValueFn); }
				catch (Exception ex)
				{
					Debug.WriteLine($"SliderFunction 'Inverse' threw: {ex.Message}");
				}
			}
		}
		public static readonly DependencyProperty ValueFnProperty = Gui_.DPRegister<SliderFunction>(nameof(ValueFn), 0.0, Gui_.EDPFlags.TwoWay);

		/// <summary>The function that converts from slider value to output value</summary>
		public string FunctionExpr
		{
			get => (string)GetValue(FunctionExprProperty);
			set => SetValue(FunctionExprProperty, value);
		}
		private void FunctionExpr_Changed()
		{
			Function = ExprLambda.Create(FunctionExpr) ?? (x => x);
		}
		public static readonly DependencyProperty FunctionExprProperty = Gui_.DPRegister<SliderFunction>(nameof(FunctionExpr), string.Empty, Gui_.EDPFlags.None);

		/// <summary>The function that converts from output values to slider values</summary>
		public string InverseExpr
		{
			get => (string)GetValue(InverseExprProperty);
			set => SetValue(InverseExprProperty, value);
		}
		private void InverseExpr_Changed()
		{
			Inverse = ExprLambda.Create(InverseExpr) ?? (x => x);
		}
		public static readonly DependencyProperty InverseExprProperty = Gui_.DPRegister<SliderFunction>(nameof(InverseExpr), string.Empty, Gui_.EDPFlags.None);
	}
}
