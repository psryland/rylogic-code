//**********************************************
// P3D Graphics Tool
//  Copyright (c) Rylogic Ltd 2019
//**********************************************

#include "src/forward.h"
#include "src/commands/model_io.h"
#include "src/commands/remove_degenerates.h"
#include "src/commands/generate_normals.h"
//#include "cex/src/NEW_COMMAND.h"

using namespace pr;
using namespace pr::geometry;
using namespace std::literals;
using namespace std::filesystem;

struct Main
{
	std::unique_ptr<p3d::File> m_model;
	path m_base_dir;
	path m_infile;
	int m_verbosity;

	Main()
		:m_model()
		,m_base_dir()
		,m_infile()
		,m_verbosity(1)
	{}

	// Show the main help
	void ShowHelp() const
	{
		std::cout <<
			"\n"
			"-------------------------------------------------------------\n"
			"  P3D Graphics Tool \n"
			"   Copyright (c) Rylogic 2019 \n"
			"   Version: v1.0\n"
			"-------------------------------------------------------------\n"
			"\n"
			"  This tool is used to generate and modify p3d format geometry models.\n"
			"  It can be driven via script or command line parameters:\n"
			"  Syntax:\n"
			"     p3d.exe script.ldr\n"
			"     p3d.exe [ordered sequence of commands]\n"
			"\n"
			"  ** NOTE: ORDER OF PARAMETERS IS IMPORTANT **\n"
			"  i.e. you probably want -fo as the LAST option\n"
			"\n"
			"  Commands:\n"
			"    -verbosity <level>\n"
			"        Set the level of feedback from this tool (0 .. 3).\n"
			"\n"
			"    -fi <filepath>\n"
			"        Load a model into memory.\n"
			"        Supported formats: p3d, 3ds, stl (so far)\n"
			"\n"
			"    -fo <filepath> [flags] [Code|Ldr]\n"
			"        Export a p3d format model file.\n"
			"        <flags> is any combination of the following separated by ':' characters:\n"
			"        Code - Optional. Output model as C++ code\n"
			"        Ldr - Optional. Output model as Ldr script\n"
			"\n"
			"        Vertex Formats:\n"
			"            Verts32Bit - Use 32-bit floats for position data (default). Size/Vert = 12 bytes (float[3])\n"
			"            Verts16Bit - Use 16-bit floats for position data. Size/Vert = 6 bytes (half_t[3])\n"
			"\n"
			"        Normal Formats:\n"
			"            Norms32Bit - Use 32-bit floats for normal data (default). Size/Norm = 12 bytes (float[3])\n"
			"            Norms16Bit - Use 16-bit floats for normal data. Size/Norm = 6 bytes (half[3])\n"
			"            NormsPack32 - Pack each normal into 32bits.  Size/Norm = 4 bytes (uint32_t)\n"
			"\n"
			"        Colour Formats:\n"
			"            Colours32Bit - Use 32-bit AARRGGBB colours (default).  Size/Colour = 4 bytes (uint32_t)\n"
			"\n"
			"        UV Formats:\n"
			"            UVs32Bit - Use 32-bit floats for UV data. Size/UV = 8 bytes (float[2])\n"
			"            UVs16Bit - Use 16-bit floats for UV data. Size/UV = 4 bytes (half[2])\n"
			"\n"
			"        Index Formats:\n"
			"            IdxSrc - Don't convert indices, use the input stride (default)\n"
			"            Idx32Bit - Use 32-bit integers for index data. Size/Index = 4 bytes (uint32_t)\n"
			"            Idx16Bit - Use 16-bit integers for index data. Size/Index = 2 bytes (uint16_t)\n"
			"            Idx8Bit - Use 8-bit integers for index data. Size/Index = 1 byte (uint8_t)\n"
			"            IdxNBit - Use variable length integers for index data.\n"
			"\n"
			"    -RemoveDegenerates [<Tolerance>:<NormalSmoothingAngle>:<ColourDistance>:<UVDistance>]\n"
			"        Simplify a model by removing degenerate verticies.\n"
			"        Parameters can be omitted, in which case defaults are used. e.g.  -RemoveDegenerates 30:::0.001\n"
			"        <Tolerance> - Vertex position quantisation value: [0,32) (default is 10 = 1<<10 = 1024).\n"
			"        <NormalSmoothingAngle> - Vertices with normals different by more than this angle (deg)\n"
			"             are not degenerate. (default normals ignored)\n"
			"        <ColourDistance> - Vertices with colours different by more than this distance are not\n"
			"             degenerate. (default colours ignored)\n"
			"        <UVDistance> - Vertices with  UVs different by more than this distance are not degenerate.\n"
			"            (default UVs ignored)\n"
			"\n"
			"    -GenerateNormals [<SmoothingAngle>]\n"
			"        Generate normals from face data within the model.\n"
			"        SmoothingAngle -  All faces within the smoothing angle of each other are smoothed.\n"
			"\n"
			"    -Transform <o2w>\n"
			"        Apply a transform to the model.\n"
			"        <o2w> - A 4x4 matrix given as pr script. e.g '*euler{20 30 20} *pos{0 1 0}'\n"
			"\n"
			// NEW_COMMAND - add a help string
			"\n"
			;
	}

	// Convert the command line into a script source
	std::unique_ptr<script::Src> ParseCommandLine(int argc, wchar_t* argv[])
	{
		using namespace pr::script;

		// No arguments given
		if (argc <= 1)
			return nullptr;

		// If the command line is a script filepath, return a file source
		if (!cmdline::IsOption(argv[1]))
		{
			// If the only argument is a filepath, assume a script file
			auto script_filepath = path(filesys::ResolvePath(argv[1]));
			if (script_filepath.empty())
				return nullptr;

			if (!exists(script_filepath))
				throw std::runtime_error(Fmt("Script '%S' does not exist", argv[1]));

			m_base_dir = script_filepath.parent_path();
			return std::unique_ptr<FileSrc>(new FileSrc(script_filepath));
		}
		else
		{
			// Otherwise, convert the command line parameters into a script
			struct Parser :cmdline::IOptionReceiver<wchar_t>
			{
				std::string m_script;
				int m_verbosity;
				bool m_ends_with_fileout;

				Parser()
					:m_script()
					,m_verbosity()
					,m_ends_with_fileout()
				{}

				// Read the options
				bool CmdLineOption(std::wstring const& option, TArgIter& arg, TArgIter arg_end) override
				{
					m_ends_with_fileout = false;
					std::ostringstream ss;
					for (;;)
					{
						if (str::EqualI(option, "-verbosity"))
						{
							if (arg == arg_end || IsOption(*arg) || !str::ExtractIntC(m_verbosity, 10, arg->c_str()) || m_verbosity < 0 || m_verbosity > 3)
								throw std::runtime_error("Verbosity level must be in the range [0..3]");

							ss << "*Verbosity {" << m_verbosity << "}\n";
							break;
						}
						if (str::EqualI(option, "-fi"))
						{
							ss << "*fi {" << Narrow(*arg++) << "}\n";
							break;
						}
						if (str::EqualI(option, "-fo"))
						{
							ss << "*fo {" << Narrow(*arg++) << " ";
							if (arg != arg_end)
							{
								if (false) {}
								else if (str::EqualI(*arg, "code")) ss << "*Code";
								else if (str::EqualI(*arg, "ldr" )) ss << "*Ldr";
								else ss << "*Flags {" << Narrow(*arg++) << "}\n";
							}
							ss << "}\n";
							m_ends_with_fileout = true;
							break;
						}
						if (str::EqualI(option, "-RemoveDegenerates"))
						{
							ss << "*RemoveDegenerates {";
							if (!IsOption(*arg))
							{
								int field = 0;
								str::Split(*arg++, ":", [&](auto sub, int)
								{
									switch (field++)
									{
										case 0: if (!sub.empty()) ss << "*Quantisation {" << Narrow(sub) << "}"; break;
										case 1: if (!sub.empty()) ss << "*NormalSmoothingAngle {" << Narrow(sub) << "}"; break;
										case 2: if (!sub.empty()) ss << "*ColourDistance {" << Narrow(sub) << "}"; break;
										case 3: if (!sub.empty()) ss << "*UVDistance {" << Narrow(sub) << "}"; break;
										default: throw std::runtime_error(FmtS("RemoveDegenerates - too many parameter fields. Expected %d", field - 1));
									}
								});
							}
							ss << "}\n";
							break;
						}
						if (str::EqualI(option, "-GenerateNormals"))
						{
							ss << "*GenerateNormals {";
							for (; arg != arg_end && !IsOption(*arg); ++arg)
							{
								if (int i; str::ExtractIntC(i, 10, arg->c_str())) { ss << "*SmoothingAngle {" << i << "}"; continue; }
								throw std::runtime_error(FmtS("GenerateNormals - unknown argument:  %s", arg->c_str()));
							}
							ss << "}\n";
							break;
						}
						if (str::EqualI(option, "-Transform"))
						{
							ss << "*Transform {" << Narrow(*arg) << "}\n";
							break;
						}
						if (str::EqualI(option, "--help"))
						{
							return false;
						}
						if (str::EqualI(option, "-h"))
						{
							return false;
						}
						if (str::EqualI(option, "/?"))
						{
							return false;
						}

						// NEW_COMMAND
						throw std::runtime_error(FmtS("Unknown command line option: %S", option.c_str()));
					}
					m_script = ss.str();
					return true;
				}
			};

			Parser p;
			EnumCommandLine<wchar_t>(argc, argv, p);
			if (p.m_script.empty())
				throw std::runtime_error("Invalid command line");

			// Warn if -fo is not the last operation
			if (!p.m_ends_with_fileout)
				std::cout << "WARNING: The command sequence does not end with a file output command (-fo).\n";

			// Dump the script
			if (p.m_verbosity >= 3)
				std::cout << "Command Script:\n" << p.m_script << "\n";

			// Create a string source
			m_base_dir = current_path();
			return std::unique_ptr<StringSrc>(new StringSrc(p.m_script, StringSrc::EFlags::BufferLocally));
		}
	}

	// Main program run
	int Run(int argc, wchar_t* argv[])
	{
		//NEW_COMMAND - Test the new command
		//if (!args.empty()) printf("warning: debugging overriding arguments");
		//args = R("-input blah -msg "Type a value> ")";
		//args = R("-shcopy "c:/deleteme/SQ.bin,c:/deleteme/TheList.txt" "c:/deleteme/cexitime/" -title "Testing shcopy")";
		//args = R"(-clip -lwr -bkslash "C:/blah" "Boris" "F:\\Jef/wan")";
		//args = R"(-p3d -fi "\dump\test.3ds" -verbosity 5 -remove_degenerates 4096 -gen_normals 40)";
		//args = R"(-p3d -fi R:\localrepo\PC\vrex\res\pelvis.3ds -verbosity 3 -remove_degenerates 16384 -gen_normals 40)";

		try
		{
			// Get the script source from the command line
			auto src = ParseCommandLine(argc, argv);
			if (src == nullptr)
			{
				ShowHelp();
				return -1;
			}

			// Execute the script
			script::Reader reader(*src);
			for (char kw[32]; reader.NextKeywordS(kw);)
			{
				if (str::EqualI(kw, "verbosity"))
				{
					reader.IntS(m_verbosity, 10);
					continue;
				}
				if (str::EqualI(kw, "fi"))
				{
					ImportFile(reader);
					continue;
				}
				if (str::EqualI(kw, "fo"))
				{
					ExportFile(reader);
					continue;
				}
				if (str::EqualI(kw, "RemoveDegenerates"))
				{
					RemoveDegenerates(reader);
					continue;
				}
				if (str::EqualI(kw, "GenerateNormals"))
				{
					GenerateNormals(reader);
					continue;
				}
				if (str::EqualI(kw, "Transform"))
				{
					Transform(reader);
					continue;
				}
				// NEW_COMMAND
				throw std::runtime_error(FmtS("Unknown command: %s (line: %d)", kw, reader.Location().Line()));
			}
		}
		catch (std::exception const& ex)
		{
			std::wcerr << ex.what() << std::endl;
			return -1;
		}

		// Run the script
		return 0;
	}

	// Import a geometry model file
	void ImportFile(script::Reader& reader)
	{
		// Read the file name
		std::string infile;
		reader.CStringS(infile);

		// Resolve the file
		m_infile = path(infile);
		m_infile = m_infile.is_relative() ? m_base_dir / m_infile : m_infile;

		// Import the file
		if (!exists(m_infile))
		{
			if (m_verbosity >= 1)
				std::cout << "Could not locate '" << infile << "'. Does the file exist?" << std::endl;

			m_model = nullptr;
		}
		else
		{
			if (m_verbosity >= 1)
				std::cout << "Loading '" << m_infile << "'." << std::endl;

			auto extn = m_infile.extension().string();
			m_model =
				str::EqualI(extn, ".p3d") ? CreateFromP3D(m_infile) :
				str::EqualI(extn, ".3ds") ? CreateFrom3DS(m_infile) :
				str::EqualI(extn, ".stl") ? CreateFromSTL(m_infile) :
				nullptr;
			if (m_model == nullptr)
				throw std::runtime_error("Model format '"s + extn + "' is not supported");
		}
	}

	// Export a p3d model file
	void ExportFile(script::Reader& reader) const
	{
		std::string outfile, extn = "p3d";
		p3d::EFlags p3d_flags = p3d::EFlags::Default;

		// Generate an output filepath based on 'infile'
		if (reader.IsSectionStart())
		{
			// Parse the optional *fo section
			reader.SectionStart();

			for (;!reader.IsSectionEnd(); )
			{
				// If a filepath is given, read it
				if (!reader.IsKeyword())
				{
					reader.CString(outfile);
					continue;
				}
				if (char kw[32]; reader.NextKeywordS(kw))
				{
					// Parse optional keywords
					if (str::EqualI(kw, "Code"))
					{
						extn = "cpp";
						continue;
					}
					if (str::EqualI(kw, "Ldr"))
					{
						extn = "ldr";
						continue;
					}
					if (str::EqualI(kw, "Flags"))
					{
						using namespace pr::geometry::p3d;
						std::string flagstr;
						reader.StringS(flagstr);

						// Parse the flags
						uint32_t flags = s_cast<uint32_t>(p3d::EFlags::Default);
						str::Split(flagstr, ":", [&](std::string_view flag, int)
						{
							if (false) {}
							else if (str::EqualI(flag, "Verts32Bit"))   flags = SetBits(flags, Flags::Mask << Flags::VertsOfs  , (uint32_t)EVertFormat  ::Verts32Bit   << Flags::VertsOfs);
							else if (str::EqualI(flag, "Verts16Bit"))   flags = SetBits(flags, Flags::Mask << Flags::VertsOfs  , (uint32_t)EVertFormat  ::Verts16Bit   << Flags::VertsOfs);
							else if (str::EqualI(flag, "Norms32Bit"))   flags = SetBits(flags, Flags::Mask << Flags::NormsOfs  , (uint32_t)ENormFormat  ::Norms32Bit   << Flags::NormsOfs);
							else if (str::EqualI(flag, "Norms16Bit"))   flags = SetBits(flags, Flags::Mask << Flags::NormsOfs  , (uint32_t)ENormFormat  ::Norms16Bit   << Flags::NormsOfs);
							else if (str::EqualI(flag, "NormsPack32"))  flags = SetBits(flags, Flags::Mask << Flags::NormsOfs  , (uint32_t)ENormFormat  ::NormsPack32  << Flags::NormsOfs);
							else if (str::EqualI(flag, "Colours32Bit")) flags = SetBits(flags, Flags::Mask << Flags::ColoursOfs, (uint32_t)EColourFormat::Colours32Bit << Flags::ColoursOfs);
							else if (str::EqualI(flag, "UVs32Bit"))     flags = SetBits(flags, Flags::Mask << Flags::UVsOfs    , (uint32_t)EUVFormat    ::UVs32Bit     << Flags::UVsOfs);
							else if (str::EqualI(flag, "UVs16Bit"))     flags = SetBits(flags, Flags::Mask << Flags::UVsOfs    , (uint32_t)EUVFormat    ::UVs16Bit     << Flags::UVsOfs);
							else if (str::EqualI(flag, "IdxSrc"))       flags = SetBits(flags, Flags::Mask << Flags::IndexOfs  , (uint32_t)EIndexFormat ::IdxSrc       << Flags::IndexOfs);
							else if (str::EqualI(flag, "Idx32Bit"))     flags = SetBits(flags, Flags::Mask << Flags::IndexOfs  , (uint32_t)EIndexFormat ::Idx32Bit     << Flags::IndexOfs);
							else if (str::EqualI(flag, "Idx16Bit"))     flags = SetBits(flags, Flags::Mask << Flags::IndexOfs  , (uint32_t)EIndexFormat ::Idx16Bit     << Flags::IndexOfs);
							else if (str::EqualI(flag, "Idx8Bit"))      flags = SetBits(flags, Flags::Mask << Flags::IndexOfs  , (uint32_t)EIndexFormat ::Idx8Bit      << Flags::IndexOfs);
							else if (str::EqualI(flag, "IdxNBit"))      flags = SetBits(flags, Flags::Mask << Flags::IndexOfs  , (uint32_t)EIndexFormat ::IdxNBit      << Flags::IndexOfs);
							else std::cout << "Unknown output flag '" << flag << "' ignored" << std::endl;
						});
						p3d_flags = s_cast<p3d::EFlags>(flags);
						continue;
					}
				}
			}

			reader.SectionEnd();
		}

		// If there is no model, then there's nothing to export. (We still need to parse the script tho)
		if (m_model == nullptr)
			return;

		// Resolve the output file path
		auto outpath =
			outfile.empty() ? path(m_infile) :
			path(outfile).is_relative() ? m_base_dir / outfile :
			path(outfile);

		outpath = outpath.replace_extension(extn);
		if (m_verbosity >= 1)
			std::cout << "Writing '" << outpath << "'..." << std::endl;

		// Ensure the output directory exists
		if (!exists(outpath.parent_path()) && !create_directories(outpath.parent_path()))
			throw std::runtime_error("Failed to create directory: "s + outpath.parent_path().string());

		// Determine the output format from the extn
		extn = outpath.extension().string();
		str::EqualI(extn, ".p3d") ? WriteP3d(m_model, outpath, p3d_flags) :
		str::EqualI(extn, ".ldr") ? WriteLdr(m_model, outpath, "\t") :
		str::EqualI(extn, ".cpp") ? WriteCpp(m_model, outpath, "\t") :
		throw std::runtime_error("Unsupported output file format: "s + extn);

		if (m_verbosity >= 3)
		{
			for (auto& mesh : m_model->m_scene.m_meshes)
			{
				std::cout
					<< "  Mesh: " << mesh.m_name.c_str() << "\n"
					<< "    V Count: " << mesh.vcount() << "\n"
					<< "    I Count: " << mesh.icount() << "\n"
					<< "    N Count: " << mesh.ncount() << "\n";
			}
		}
		if (m_verbosity >= 1)
			std::cout << "'" << outpath << "' saved." << std::endl;
	}

	// Remove degenerate verts from the model
	void RemoveDegenerates(script::Reader& reader) const
	{
		if (m_model == nullptr)
			return;

		auto quantisation = 10;
		auto normal_smoothing_angle = -1.0f;
		auto colour_distance = -1.0f;
		auto uv_distance = -1.0f;

		// Read parameters
		reader.SectionStart();
		for (char kw[32]; reader.NextKeywordS(kw);)
		{
			if (str::EqualI(kw, "Quantisation"))
			{
				reader.IntS(quantisation, 10);
				continue;
			}
			if (str::EqualI(kw, "NormalSmoothingAngle"))
			{
				reader.RealS(normal_smoothing_angle);
				continue;
			}
			if (str::EqualI(kw, "ColourDistance"))
			{
				reader.RealS(colour_distance);
				continue;
			}
			if (str::EqualI(kw, "UVDistance"))
			{
				reader.RealS(uv_distance);
				continue;
			}
		}
		reader.SectionEnd();

		// Remove the degenerates
		RemoveDegenerateVerts(*m_model, quantisation, normal_smoothing_angle, colour_distance, uv_distance, m_verbosity);
	}

	// Generate normals for the model
	void GenerateNormals(script::Reader& reader) const
	{
		if (m_model == nullptr)
			return;

		auto smoothing_angle = 10.0f;

		// Read parameters
		reader.SectionStart();
		for (char kw[32]; reader.NextKeywordS(kw);)
		{
			if (str::EqualI(kw, "SmoothingAngle"))
			{
				reader.RealS(smoothing_angle);
				continue;
			}
		}
		reader.SectionEnd();

		// Remove the degenerates
		GenerateVertNormals(*m_model, smoothing_angle, m_verbosity);
	}

	// Apply a transform to the model
	void Transform(script::Reader& reader) const
	{
		if (m_model == nullptr)
			return;

		// Read the object to world transform
		auto o2w = m4x4::Identity();
		reader.TransformS(o2w);
		
		// Create a normals to world transform
		auto n2w = o2w;
		n2w.x = Normalise(n2w.x);
		n2w.y = Normalise(n2w.y);
		n2w.z = Normalise(n2w.z);

		if (m_verbosity >= 2)
			std::cout << "  Applying transform to model" << std::endl;
		if (m_verbosity >= 3)
			std::cout << "    Position transform: " << o2w << std::endl;
		if (m_verbosity >= 3)
			std::cout << "    Normal transform: " << n2w << std::endl;

		for (auto& mesh : m_model->m_scene.m_meshes)
		{
			auto bbox = BBox::Reset();
			for (auto& pos : mesh.m_vert)
			{
				pos = o2w * static_cast<v4>(pos);
				Grow(bbox, pos);
			}
			for (auto& norm : mesh.m_norm)
			{
				norm = n2w * static_cast<v4>(norm);
			}
			mesh.m_bbox = bbox;
		}
	}
};

int __cdecl wmain(int argc, wchar_t* argv[])
{
	Main m;
	return m.Run(argc, argv);
}