using System;
using System.ComponentModel;
using System.Reflection;
using System.Windows.Input;

namespace Rylogic.Gui.WPF
{
	/// <summary>Command base class for 'ICommand'</summary>
	public abstract class Command : ICommand, INotifyPropertyChanged
	{
		// Notes:
		//
		// - Context Menu 'Command' binding not working? Create your context menu like this:
		//        <ContextMenu
		//            DataContext="{Binding PlacementTarget.DataContext, RelativeSource={RelativeSource Self}}"
		//            >
		//            <MenuItem
		//                Header="MyCmd..."
		//                Command="{Binding MyCmd}"  <-- this should hanld the 'CanExecute'
		//                IsEnabled="{Binding MyCmd.Available}"
		//            />
		//        </ContextMenu>
		//    The reason is that context menus are not in the same visual tree as the what they're attached to
		//    so they don't automatically pick up the correct DataContext.
		//

		/// <summary>No op command</summary>
		public static readonly Command NoOp = new Command<object>(new object(), null, null);

		/// <summary>Can execute changed</summary>
		public event EventHandler? CanExecuteChanged;
		public void NotifyCanExecuteChanged()
		{
			CanExecuteChanged?.Invoke(this, EventArgs.Empty);
			NotifyPropertyChanged(nameof(Available));
		}

		/// <summary>True if the command is available</summary>
		public virtual bool CanExecute(object? _) => true;
		public bool CanExecute() => CanExecute(null);
		public bool Available => CanExecute();

		/// <summary>Execute the command</summary>
		public virtual void Execute(object? _) { }
		public void Execute() => Execute(null);

		/// <summary></summary>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));

		/// <summary>Construct a command from a delegate</summary>
		public static Command<TOwner> Create<TOwner>(TOwner owner, Action execute)
		{
			return new Command<TOwner>(owner, (o, p) => execute(), null);
		}
		public static Command<TOwner> Create<TOwner>(TOwner owner, Action<object?> execute)
		{
			return new Command<TOwner>(owner, (_, p) => execute(p), null);
		}
		public static Command<TOwner> Create<TOwner>(TOwner owner, Action<TOwner, object?> execute)
		{
			return new Command<TOwner>(owner, (o, p) => execute(o,p), null);
		}
		public static Command<TOwner> Create<TOwner>(TOwner owner, Action execute, Func<bool> can_execute)
		{
			return new Command<TOwner>(owner, (o, p) => execute(), (o, p) => can_execute());
		}
		public static Command<TOwner> Create<TOwner>(TOwner owner, Action<object?> execute, Func<object?, bool> can_execute)
		{
			return new Command<TOwner>(owner, (_, p) => execute(p), (_, p) => can_execute(p));
		}
	}

	/// <summary>Base class for WPF commands</summary>
	public class Command<TOwner> : Command
	{
		public Command(TOwner owner)
			: this(owner, null, null)
		{ }
		public Command(TOwner owner, Action<TOwner, object?>? execute, Func<TOwner, object?, bool>? can_execute)
		{
			Owner = owner;
			m_execute = execute;
			m_can_execute = can_execute;
		}

		/// <summary>The owner</summary>
		protected TOwner Owner { get; }

		/// <summary>True if the command is available</summary>
		public override bool CanExecute(object? parameter) => m_can_execute?.Invoke(Owner, parameter) ?? (m_execute != null);
		private readonly Func<TOwner, object?, bool>? m_can_execute;

		/// <summary>Execute the command</summary>
		public override void Execute(object? parameter) => m_execute?.Invoke(Owner, parameter);
		private readonly Action<TOwner, object?>? m_execute;
	}

	/// <summary>A wrapper that invokes a command on 'data_context'</summary>
	public class ReflectedCommand :Command
	{
		private readonly object? m_command;
		private readonly MethodInfo? m_execute;
		private readonly MethodInfo? m_can_execute;

		public ReflectedCommand(object data_context, string command)
		{
			// Locate the command within the data context. Silently ignore missing commands
			var command_pi = data_context.GetType().GetProperty(command);
			if (command_pi == null || !typeof(ICommand).IsAssignableFrom(command_pi.PropertyType))
				return;

			// Find the Execute and CanExecute methods
			m_command = command_pi.GetValue(data_context);
			m_execute = command_pi.PropertyType.GetMethod(nameof(ICommand.Execute), new Type[] { typeof(object) });
			m_can_execute = command_pi.PropertyType.GetMethod(nameof(ICommand.CanExecute), new Type[] { typeof(object) });
		}
		public override bool CanExecute(object? parameter)
		{
			if (m_command == null || m_can_execute == null) return false;
			return (bool?)m_can_execute.Invoke(m_command, new object?[] { parameter }) ?? false;
		}
		public override void Execute(object? parameter)
		{
			if (m_command == null || m_execute == null) return;
			m_execute.Invoke(m_command, new object?[] { parameter });
		}
	}
}
