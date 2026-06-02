namespace LDraw
{
	public static class CmdLine
	{
		// Order by longest option first
		public const string SettingsPath = "-s";
		public const string Portable     = "-p";
		public const string ShowHelp     = "-h";
		public const string ShowHelp2    = "/?";

		/// <summary>Help string</summary>
		public static string Help
		{
			get
			{
				return
					"LDraw Command Line:\n" +
					"   Syntax: LDraw.exe [options] [script_filepath]\n" +
					"\n" +
					"   Options:\n" +
					"   -s <filepath> : Load/Save application settings to the given filepath.\n" +
					"   -p : Run LDraw in portable mode.\n" +
					"   -h or /? : Show this help message\n" +
					"";
			}
		}
	}
}
