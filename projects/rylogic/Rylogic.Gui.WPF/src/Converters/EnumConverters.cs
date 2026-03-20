using System;
using System.Collections;
using System.Globalization;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Markup;
using System.Windows.Media;
using System.Xaml;
using Rylogic.Attrib;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF.Converters
{
	// Notes:
	//  - [Flags] enum => use 'HasFlag' in BoolConverters

	/// <summary></summary>
	public class EnumToDesc : MarkupExtension, IValueConverter
	{
		// Notes:
		//  - Convert an enum to its description attribute
		public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
		{
			if (value is IEnumerable collection)
				return collection.Cast<object>().OfType<Enum>().Select(x => x.Desc());
			if (value is Enum en)
				return en.Desc();
			return null;
		}
		public object? ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
		{
			throw new NotSupportedException();
		}
		public override object ProvideValue(IServiceProvider serviceProvider)
		{
			return this;
		}
	}

	/// <summary></summary>
	public class EnumValues : MarkupExtension, IValueConverter
	{
		// Use:
		//  <ComboBox
		//     ItemsSource="{Binding MyProp, Converter={conv:EnumValues}, Mode=OneTime}"
		//     ItemTemplate= "{conv:EnumValues+ToDesc}"  <-- Remove this if your enum doesn't use [Desc()] attributes
		//     SelectedItem="{Binding MyProp}"
		//     />
		public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
		{
			if (value == null)
				throw new ArgumentNullException();
			if (!value.GetType().IsEnum)
				throw new ArgumentException("Expected an enum property");

			var values = Enum.GetValues(value.GetType());
			return values;
		}
		public object? ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
		{
			throw new NotSupportedException();
		}
		public override object ProvideValue(IServiceProvider serviceProvider)
		{
			return this;
		}

		/// <summary>Helper for mapping enum values to their description</summary>
		public class ToDesc : MarkupExtension
		{
			public override object ProvideValue(IServiceProvider serviceProvider)
			{
				var dt = new DataTemplate();
				dt.VisualTree = new FrameworkElementFactory(typeof(TextBlock));
				dt.VisualTree.SetValue(TextBlock.TextProperty, new Binding(".") { Converter = new EnumToDesc() });
				return dt;
			}
		}
	}

	/// <summary>Select between options based on an enum.</summary>
	public class EnumSelect :MarkupExtension, IValueConverter
	{
		// Notes:
		//  - Parameter should be 'case0:value0|case1:value1|case2:value2|default_value'
		//    where 'value' is a string that is convertable to 'targetType'.
		//  - Any value with no 'case' is assumed to be the default value.
		//    The default case should always be last

		public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
		{
			if (value is not Enum e)
				return null;
			if (parameter is not string s)
				return null;

			try
			{
				// Look for a matching case
				foreach (var pair in s.Split(new[] { '|' }, StringSplitOptions.RemoveEmptyEntries))
				{
					var case_value = pair.Split(':');

					// Default case?
					if (case_value.Length == 1)
						return InterpretValue(case_value[0]);

					// Valid 'case:value' pair?
					if (case_value.Length != 2)
						continue;

					// Case name is not a member of the enum
					if (!Enum.IsDefined(value.GetType(), case_value[0]))
						continue;

					// Case matches 'value'
					var cas = (Enum)Enum.Parse(value.GetType(), case_value[0]);
					var is_flags = e.GetType().HasAttribute<FlagsAttribute>();
					var is_match = is_flags ? e.HasFlag(cas) : Equals(cas, value);
					if (!is_match)
						continue;

					// Interpret the value to the target type
					return InterpretValue(case_value[1]);
				}
				return null;
			}
			catch
			{
				// Handle this silently so that runtime editing XML works
				return null;
			}

			// Convert the string to the target type
			object? InterpretValue(string val)
			{
				// Special case conversions not handled by 'ConvertTo'
				if (targetType == typeof(Colour32))
				{
					return Colour32.Parse(val);
				}
				if (targetType == typeof(Color))
				{
					return Colour32.Parse(val).ToMediaColor();
				}
				if (targetType == typeof(Brush))
				{
					return Colour32.Parse(val).ToMediaBrush();
				}
				if (targetType == typeof(Thickness))
				{
					var i = (int?)Util.ConvertTo(val, typeof(int)) ?? 0;
					return new Thickness(i);
				}
				if (targetType == typeof(ImageSource))
				{
					// 'val' should be the name of a static resource. The problem is we don't have the control that
					// this binding converter is being used on, so we don't know which window to search for resources.
					// 'TryFindResource' only searches upwards through the parents, so really we need a pointer to the
					// FrameworkElement that the converter is being used on. So far, I don't know how to get that reference,
					// so instead, just search all the UserControls. A better solution is needed ...

					if (m_host != null && m_host.TryFindResource(val) is object res0) //does this work??
					{
						return res0;
					}

					// Search App.xaml resources first.
					var res = Application.Current.TryFindResource(val);
					if (res == null)
					{
						foreach (var win in Application.Current.Windows.Cast<Window>())
						{
							res ??= win
								.AllVisualChildren().OfType<UserControl>()
								.Select(x => x.TryFindResource(val))
								.FirstOrDefault(x => x != null);
							if (res != null)
								break;
						}
					}
					return res;
				}
				return Util.ConvertTo(val, targetType);
			}
		}
		public object? ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
		{
			throw new NotSupportedException();
		}
		public override object ProvideValue(IServiceProvider serviceProvider)
		{
			//if (serviceProvider.GetService(typeof(IProvideValueTarget)) is IProvideValueTarget target &&
			//	target.TargetObject is FrameworkElement host)
			//	m_host = host;
			if (serviceProvider.GetService(typeof(IRootObjectProvider)) is IRootObjectProvider target &&
				target.RootObject is FrameworkElement host)
				m_host = host;

			return this;
		}
		private FrameworkElement? m_host;
	}

	/// <summary>Convert between int values in enum values</summary>
	public class EnumToInt :MarkupExtension, IValueConverter
	{
		// Use:
		//  <ComboBox
		//     SelectedIndex="{Binding MyEnumValue, Converter={conv:EnumToInt}}"
		//     />
		public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
		{
			if (value is not Enum en)
				throw new ArgumentException("Expected an enum property");

			return Util.ConvertTo(value, typeof(int));
		}
		public object? ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
		{
			if (!Enum.IsDefined(targetType, value))
				throw new ArgumentException("Expected an enum property");

			return Util.ConvertTo(value, targetType);
		}
		public override object ProvideValue(IServiceProvider serviceProvider)
		{
			return this;
		}
	}

	/// <summary></summary>
	public class EnumFlags : MarkupExtension, IValueConverter
	{
		// Use:
		//    <ListBox
		//        ItemsSource = "{Binding MyEnumFlagsProp, Converter={conv:EnumFlags}}"  - This returns a collection of the single bit flags
		//        >
		//        <ListBox.Resources>
		//            <gui:DataContextRef x:Key="ListCtx" Ctx="{Binding .}"/> - This captures the ListBox DataContext
		//        </ListBox.Resources>
		//        <ListBox.ItemTemplate>
		//            <DataTemplate>
		//                <CheckBox
		//                    Content = "{Binding .}"  - This binds the check box content to the list item (i.e. a single bit value)
		//                    IsChecked="{conv:EnumFlagSet BitField={Binding Source={StaticResource ListCtx}, Path=Ctx.MyEnumFlagsProp, Mode=TwoWay}, Bits={Binding .}}"
		//                         ^- This binds the list item to a converter that sets/clears the bit in 'MyEnumFlagsProp' 
		//                    />
		//            </DataTemplate>
		//        </ListBox.ItemTemplate>
		//    </ListBox>
		public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
		{
			if (value?.GetType() is not Type ty)
				throw new ArgumentNullException();
			if (!ty.IsEnum || !ty.HasAttribute<FlagsAttribute>())
				throw new ArgumentException("Expected a flags enum property");

			// Convert the flags into a list of the single bit values
			return Enum.GetValues(value.GetType())
				.ConvertTo<long>()
				.Where(x => Bit.IsPowerOfTwo(x))
				.ConvertTo(ty);
		}
		public object? ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
		{
			throw new NotSupportedException();
		}
		public override object ProvideValue(IServiceProvider serviceProvider)
		{
			return this;
		}
	}
	[ContentProperty(nameof(Binding))]
	public class EnumFlagSet : MarkupExtension ,IMultiValueConverter
	{
		/// <summary>The binding to the property that will have bits set/cleared</summary>
		public Binding BitField { get; set; } = null!;

		/// <summary>The binding to the bit value</summary>
		public Binding Bits { get; set; } = null!;

		public override object ProvideValue(IServiceProvider service_provider)
		{
			var mb = new MultiBinding { Mode = BindingMode.TwoWay };
			mb.Bindings.Add(BitField);
			mb.Bindings.Add(Bits);
			mb.Converter = this;
			return mb.ProvideValue(service_provider);
		}
		public object Convert(object[] values, Type targetType, object parameter, CultureInfo culture)
		{
			if (values.Length != 2)
				return false;
			if (values[0] == DependencyProperty.UnsetValue ||
				values[1] == DependencyProperty.UnsetValue)
				return false;

			m_lhs = Util.ConvertTo<long>(values[0]); // bitfield
			m_rhs = Util.ConvertTo<long>(values[1]); // bits
			var set = Bit.AllSet(m_lhs, m_rhs);
			return set;
		}
		public object[] ConvertBack(object value, Type[] target_types, object parameter, CultureInfo culture)
		{
			var set = (bool)value;
			var val = new object[2]
			{
				Util.ConvertTo(Bit.SetBits(m_lhs, m_rhs, set), target_types[0])!, // bitfield
				Util.ConvertTo(m_rhs, target_types[1])!, // bits
			};
			return val;
		}
		private long m_lhs;
		private long m_rhs;
	}


	[ContentProperty(nameof(Binding))]
	public class BindingEx : MarkupExtension
	{
		// Notes:
		//  - This helper wraps a multi binding into an object that looks like a normal binding but with a bindable parameter.
		// Use:
		//   <Setter Property="Visibility">
		//     <Setter.Value>
		//       <conv:BindingEx
		//           Binding = "{Binding Tag, RelativeSource={RelativeSource Mode=FindAncestor, AncestorType={x:Type UserControl}"
		//           Converter="{StaticResource AccessLevelToVisibilityConverter}"
		//           ConverterParameterBinding="{Binding RelativeSource={RelativeSource Mode=Self}, Path=Tag}"
		//           />
		//   	</Setter.Value>
		//   </Setter>
		public BindingEx()
		{ }
		public BindingEx(string path)
		{
			Binding = new Binding(path);
		}
		public BindingEx(Binding binding)
		{
			Binding = binding;
		}
		public override object ProvideValue(IServiceProvider serviceProvider)
		{
			// Create the internal 'MultiBinding' and add 'Binding' and 'ConverterParameter' to it
			var multi_binding = new MultiBinding();
			
			// Add the main binding
			Binding.Mode = Mode;
			multi_binding.Bindings.Add(Binding);

			// Add the converter parameter
			if (ConverterParameter != null)
			{
				ConverterParameter.Mode = BindingMode.OneWay;
				multi_binding.Bindings.Add(ConverterParameter);
			}

			var adapter = new MultiValueConverterAdapter
			{
				Converter = Converter
			};
			multi_binding.Converter = adapter;
			return multi_binding.ProvideValue(serviceProvider);
		}
		public Binding Binding { get; set; } = null!;
		public BindingMode Mode { get; set; } = BindingMode.Default;
		public IValueConverter Converter { get; set; } = null!;
		public Binding ConverterParameter { get; set; } = null!;


		[ContentProperty(nameof(Converter))]
		private class MultiValueConverterAdapter : IMultiValueConverter
		{
			public IValueConverter Converter { get; set; } = null!;

			private object m_last_parameter = null!;

			public object Convert(object[] values, Type targetType, object parameter, CultureInfo culture)
			{
				if (Converter == null) return values[0]; // Required for VS design-time
				if (values.Length > 1) m_last_parameter = values[1];
				return Converter.Convert(values[0], targetType, m_last_parameter, culture);
			}

			public object[] ConvertBack(object value, Type[] targetTypes, object parameter, CultureInfo culture)
			{
				if (Converter == null) return new object[] { value }; // Required for VS design-time

				return new object[] { Converter.ConvertBack(value, targetTypes[0], m_last_parameter, culture) };
			}
		}
	}
}
