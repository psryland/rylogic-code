namespace Rylogic.Gui.WPF
{
	public partial class ContextMenus
	{
		// Notes:
		//  - To access these CMenu resources from other assemblies use:
		//       // File: App.xaml
		//       xmlns:gui="clr-namespace:Rylogic.Gui.WPF;assembly=Rylogic.Gui.WPF"
		//       ...
		//       <Application.Resources>
		//           <ResourceDictionary>
		//               <ResourceDictionary.MergedDictionaries>
		//                   <x:Static Member="gui:ContextMenus.Instance"/>
		//               </ResourceDictionary.MergedDictionaries>
		//           </ResourceDictionary>
		//       </Application.Resources>
		//  - Using resources in this way allows:
		//       - x:Shared="false" elements,
		//       - path independence in the containing assembly
		//
		// - Context Menu 'IsEnabled' binding not working? Create your context menu like this:
		//    <ContextMenu
		//        DataContext="{Binding PlacementTarget.DataContext, RelativeSource={RelativeSource Self}}"
		//        >
		//        <MenuItem
		//            Header="Add Source..."
		//            Command="{Binding AddSource}"
		//        />
		//    </ContextMenu>
		//    The reason is that context menus are not in the same visual tree as the what they're attached to
		//    so they don't automatically pick up the correct DataContext.
		//
		public static ContextMenus Instance { get; } = new ContextMenus();
		public ContextMenus() { InitializeComponent(); }
	}
}
