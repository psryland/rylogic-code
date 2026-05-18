# Rylogic Lib project organisation

This repository contains the Rylogic Ltd code.

## Project structure

	rylogic_code
	|-- art
	|   |-- icons       - Art assets
	|   |-- pngs        - Art assets
	|   |-- 3d models   - Model assets
	|   |-- etc...
	|
	|-- build           - Project files and property sheets
	|   |-- props       - Property sheets
	|   |-- target      - msbuild target files
	|   |-- etc...      - Miscellaneous files
	|
	|-- include         - Public headers and interfaces. Users of this library should add this directory as an include path.
	|   |-- pr            All rylogic library code uses includes relative to '/rylogic_code/include', e.g. #include "pr/common/..."
	|       |-- common
	|       |-- etc...
	|
	|-- projects        - C/C++/C# Projects
	|   |-- apps        - Applications/ideas some complete, most half baked
	|   |   |-- LDraw
	|   |   |-- RyLogViewer
	|   |   |-- Rylogic.TextAligner
	|   |   |-- etc...
	|   |-- rylogic     - Rylogic core libraries and assemblies
	|   |   |-- Rylogic.Core
	|   |   |-- Rylogic.Gui.WPF
	|   |   |-- view3d
	|   |   |-- etc...
	|   |-- tests       - Unit test projects and other tests
	|   |-- tools       - Helper utility projects
	|
	|-- typescript      - Typescript Projects
	|   |-- Rylogic.TextAligner
	|   |-- etc...
	|
	|-- script            - Scripts used in the build process.
	|   |-- UserVars.csx  - User variables required to build the library.
	|   |-- UserVars.json - Local customisation of user variables.
	|   |-- Tools.csx     - Helper functions
	|   |-- etc...
	|
	|-- sdk             - Third party libraries and source
	|-- tools           - Handy binaries
	|-- miscellaneous   - HowTos, binary templates, licenses, and other random stuff
	|
	|-- obj             - Generated directory containing all object files for native projects
	|-- lib             - Generated directory containing compiled libraries
	|-- bin             - Generated directory containing compiled executables
	|
    |-- Rylogic.sln - "Everything" solution file

## Building

This project is only used on windows. Compiling requires MSBuild, dotnet, dotnet-script.
Follow these steps to build:

- Pull to a clean directory,
- Ensure the 'dotnet-script' tool is installed (use `dotnet tool install -g dotnet-script`),
- Customise _/script/UserVars.csx_ or _/script/UserVars.json_ as needed,
- Run `dotnet-script ./script/Build.csx` to build projects from the command line, or, open _Rylogic.sln_ in Visual Studio.

For faster local Visual Studio builds, copy `Directory.Build.user.props.template` to `Directory.Build.user.props` and enable the opt-in settings there.

This repo is actively developed, often refactored, and frequently broken. It is public so that the source for my released projects is publicly available.

## License

- [licence](miscellaneous/licenses/license.txt)
