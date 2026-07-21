//*****************************************
// Zip Compression
//	Copyright (c) Rylogic 2019
//*****************************************
// This code was originally based on the 'miniz' library but has
// been heavily refactored to make use of modern C++ language features.
// See the file end for copyright notices.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <chrono>

namespace pr::storage::zip
{
	template <typename TAlloc = std::allocator<void>>
	class ZipArchiveA
	{
		// Notes:
		//  - Zip File Format Reference: https://pkware.cachefly.net/webdocs/casestudies/APPNOTE.TXT
		//  - Deflate algorithm reference: https://www.w3.org/Graphics/PNG/RFC-1951#algorithm
		//  - ZLIB compressed data format spec: https://tools.ietf.org/html/rfc1950
		//  - Huffman coding description: https://rosettacode.org/wiki/Huffman_coding
		//
		// Zip Format:
		//   [Zip File Record] [0 - N)
		//       [Local Directory Header]
		//       [Item Name]
		//       [Extra Data]
		//       [Compressed File Data]
		//   [Central Directory Record] [0 - N)
		//       [Central Directory Header]
		//       [Item Name]
		//       [Extra Data]
		//       [Item Comment]
		//   [End of Central Directory Locator]
		//
		// Use cases:
		//  1) Read an archive and extract it's contents:
		//       ZipArchive z(mem);                        // Construct from in-memory zip
		//       ZipArchive z(filepath);                   // Construct from file
		//       z.Extract(item_name, file);
		//       z.Extract(item_name, mem);
		//       z.Add(item_name, file);                   // Error - ReadOnly
		//       z.Remove(item_name);                      // Error - ReadOnly
		//       z.Close();                                // No effect since read only
		//
		//  2) Create new or modify existing archive
		//       ZipArchive z;                             // Archive created in internal memory. Initially empty. Completed with 'Close' or destruction.
		//       ZipArchive z(ostream, EMode::Writeable);  // Archive written to 'ostream'. Initially empty. Completed with 'Close' or destruction.
		//       ZipArchive z(filepath, EMode::Writeable); // Archive written to 'filepath'. Completed with 'Close' or destruction.
		//       z.Add(item_name, file);
		//       z.Add(item_name, mem);
		//       z.Remove(item_name);                      // Error unless using internal memory archive
		//       z.Extract(item_name, file);
		//       z.Close();                                // Flushes to 'filepath' or 'ostream'
		//

	public:

		using allocator_t = TAlloc;
		using alloc_traits_t = typename std::allocator_traits<allocator_t>;
		template <typename U> using allocator_u = typename alloc_traits_t::template rebind_alloc<U>;
		template <typename T> using vector_t = typename std::vector<T, allocator_u<T>>;
		template <typename T> using string_t = typename std::basic_string<T, std::char_traits<T>, allocator_u<T>>;
		template <typename T> using span_t = typename std::basic_string_view<T>;
		template <typename T> using ifstream_t = typename std::basic_ifstream<T>;
		template <typename T> using ofstream_t = typename std::basic_ofstream<T>;
		using MSDosTimestamp = struct { uint16_t time, date; };
		static_assert(sizeof(uint16_t) == 2);
		static_assert(sizeof(uint32_t) == 4);
		static_assert(sizeof(uint64_t) == 8);

		// The mode this archive is in
		enum class EMode
		{
			ReadOnly = 0,
			Writeable = 1,
		};

		// Archive flags
		enum class EZipFlags
		{
			None = 0,

			// Opening an archive normally scans through the records in the 
			// file to find the end of central directory record. This might
			// be slow for archives containing many files. Scan from end
			// searches from the end of the archive data, but is volnerable
			// to archives containing malicious data in the archive comment.
			ScanFromEnd = 1 << 0,

			// Used when searching for items by name
			IgnoreCase = 1 << 1,

			// Used when searching for items by name
			IgnorePath = 1 << 2,

			// Used when adding and extracting items. Does not calculate or check CRC's.
			IgnoreCrc = 1 << 3,

			// Used when opening an archive. Generates a hash table of zip entry names to
			// offsets allowing for faster access to contained files. Combine with 'IgnoreCase' and 'IgnorePath'
			FastNameLookup = 1 << 4,

			// Used in 'Extract' to copy data without decompressing it
			CompressedData = 1 << 5,
		};

		// Compression methods
		enum class EMethod :uint16_t
		{
			None = 0,
			Shrunk = 1,
			Reduce1 = 2,
			Reduce2 = 3,
			Reduce3 = 4,
			Reduce4 = 5,
			Implode = 6,
			Reserved_for_Tokenizing_Compression_Algorithm = 7,
			Deflate = 8,
			Deflate64 = 9,
			PKWARE_Data_Compression_Library_Imploding = 10,
			Reserved_by_PKWARE_1 = 11,
			BZIP2 = 12,
			Reserved_by_PKWARE_2 = 13,
			LZMA = 14,
			Reserved_by_PKWARE_3 = 15,
			IBM_CMPSC = 16,
			Reserved_by_PKWARE_4 = 17,
			IBM_TERSE = 18,
			IBM_LZ77 = 19, // z Architecture (PFS)
			JPEG_Variant = 96,
			WavPack = 97,
			PPMd = 98, // version I, Rev 1
			AE_x = 99, // encryption marker (see APPENDIX E)
		};

		// Compression levels: 0-9 are the standard zlib-style levels, 10 is best possible compression (not zlib compatible, and may be very slow), MZ_DEFAULT_COMPRESSION=MZ_DEFAULT_LEVEL.
		enum class ECompressionLevel
		{
			None = 0,
			Fastest = 1,
			Default = 6,
			Best = 9,
			Uber = 10,
		};

		// Zip file header flags
		enum class EBitFlags :uint16_t
		{
			None = 0,
			Encrypted             = 1 << 0,
			CompressionFlagBit1   = 1 << 1, 
			CompressionFlagBit2   = 1 << 2,
			DescriptorUsedMask    = 1 << 3,
			Reserved1             = 1 << 4,
			PatchFile             = 1 << 5,
			StrongEncrypted       = 1 << 6,
			CurrentlyUnused1      = 1 << 7,
			CurrentlyUnused2      = 1 << 8,
			CurrentlyUnused3      = 1 << 9,
			CurrentlyUnused4      = 1 << 10,
			Utf8                  = 1 << 11, // filename and comment encoded using UTF-8
			ReservedPKWARE1       = 1 << 12,
			CDEncrypted           = 1 << 13, // Used when encrypting the Central Directory to indicate selected data values in the Local Header are masked to hide their actual values.
			ReservedPKWARE2       = 1 << 14,
			ReservedPKWARE3       = 1 << 15,
		};

		#pragma pack(push, 1)

		// Local directory header
		struct LDH
		{
			// Notes:
			//  - The 'Extra' in this header is not necessarily the same as the 'Extra' in CDH

			static uint32_t const Signature = 0x04034b50; // PK34

			uint32_t Sig;
			uint16_t Version;
			EBitFlags BitFlags;
			EMethod  Method;
			uint16_t FileTime;
			uint16_t FileDate;
			uint32_t Crc;
			uint32_t CompressedSize;
			uint32_t UncompressedSize;
			uint16_t NameSize;
			uint16_t ExtraSize;

			// Following this header:
			// char    Name[NameSize];
			// uint8_t Extra[ExtraSize];
			// uint8_t Data[CompressedSize];

			LDH() = default;
			LDH(int item_name_size, int extra_size, int64_t uncompressed_size, int64_t compressed_size, uint32_t uncompressed_crc32, EMethod method, EBitFlags bit_flags, MSDosTimestamp dos_timestamp)
				:Sig(Signature)
				,Version(VersionFor(method))
				,BitFlags(bit_flags)
				,Method(method)
				,FileTime(dos_timestamp.time)
				,FileDate(dos_timestamp.date)
				,Crc(uncompressed_crc32)
				,CompressedSize(s_cast<uint32_t>(compressed_size))
				,UncompressedSize(s_cast<uint32_t>(uncompressed_size))
				,NameSize(s_cast<uint16_t>(item_name_size))
				,ExtraSize(s_cast<uint16_t>(extra_size))
			{}
			size_t Size() const
			{
				return sizeof(LDH) + NameSize + ExtraSize + CompressedSize;
			}
			size_t ItemDataOffset() const
			{
				return sizeof(LDH) + NameSize + ExtraSize;
			}
			std::string_view ItemName() const
			{
				return std::string_view(reinterpret_cast<char const*>(this + 1), NameSize);
			}
			span_t<uint8_t> Extra() const
			{
				return span_t<uint8_t>(reinterpret_cast<uint8_t const*>(this + 1) + NameSize, ExtraSize);
			}
			span_t<uint8_t> Data() const
			{
				return span_t<uint8_t>(reinterpret_cast<uint8_t const*>(this + 1) + NameSize + ExtraSize, CompressedSize);
			}
		};
		static_assert(sizeof(LDH) == 30);
		static_assert(std::is_trivially_copyable_v<LDH>);

		// Central directory header
		struct CDH
		{
			// Notes:
			//  - The 'Extra' in this header is not necessarily the same as the 'Extra' in LDH

			static uint32_t const Signature = 0x02014b50;// PK12

			uint32_t Sig;
			uint16_t VersionMadeBy;
			uint16_t VersionNeeded;
			EBitFlags BitFlags;
			EMethod  Method;
			uint16_t FileTime;
			uint16_t FileDate;
			uint32_t Crc;
			uint32_t CompressedSize;
			uint32_t UncompressedSize;
			uint16_t NameSize;
			uint16_t ExtraSize;
			uint16_t CommentSize;
			uint16_t DiskNumberStart;
			uint16_t InternalAttributes;
			uint32_t ExternalAttributes;
			uint32_t LocalHeaderOffset;

			// Following this header:
			// char    ItemName[NameSize];
			// uint8_t Extra[ExtraSize];
			// uint8_t ItemComment[ItemCommentSize];

			CDH() = default;
			CDH(int name_size, int extra_size, int comment_size, int64_t uncompressed_size, int64_t compressed_size, uint32_t uncompressed_crc32, EMethod method, EBitFlags bit_flags, MSDosTimestamp dos_timestamp, int64_t local_header_ofs, uint32_t ext_attributes, uint16_t int_attributes)
				:Sig(Signature)
				,VersionMadeBy()
				,VersionNeeded(VersionFor(method))
				,BitFlags(bit_flags)
				,Method(method)
				,FileTime(dos_timestamp.time)
				,FileDate(dos_timestamp.date)
				,Crc(uncompressed_crc32)
				,CompressedSize(s_cast<uint32_t>(compressed_size))
				,UncompressedSize(s_cast<uint32_t>(uncompressed_size))
				,NameSize(s_cast<uint16_t>(name_size))
				,ExtraSize(s_cast<uint16_t>(extra_size))
				,CommentSize(s_cast<uint16_t>(comment_size))
				,DiskNumberStart()
				,InternalAttributes(int_attributes)
				,ExternalAttributes(ext_attributes)
				,LocalHeaderOffset(s_cast<uint32_t>(local_header_ofs))
			{}
			size_t Size() const
			{
				return sizeof(CDH) + NameSize + ExtraSize + CommentSize;
			}
			std::string_view ItemName() const
			{
				return std::string_view(reinterpret_cast<char const*>(this + 1), NameSize);
			}
			span_t<uint8_t> Extra() const
			{
				return span_t<uint8_t>(reinterpret_cast<uint8_t const*>(this + 1) + NameSize, ExtraSize);
			}
			std::string_view Comment() const
			{
				return std::string_view(reinterpret_cast<char const*>(this + 1) + NameSize + ExtraSize, CommentSize);
			}
			bool IsDirectory() const
			{
				// Notes:
				//  - Some old zips have directory entries which have compressed deflate data that inflates to 0 bytes.
				//    These entries claim to uncompress to 512 bytes in the headers.
				//  - Most/all zip writers (hopefully) set DOS file/directory attributes in the low 16-bits, so check for
				//    the DOS directory flag and ignore the source OS ID in the created by field.
				return
					NameSize != 0 && *(reinterpret_cast<char const*>(this + 1) + NameSize - 1) == '/' ||
					has_flag(ExternalAttributes, DOSSubDirectoryFlag);
			}
			std::filesystem::file_time_type Time() const
			{
				return DosTimeToFSTime(MSDosTimestamp{FileTime, FileDate});
			}
		};
		static_assert(sizeof(CDH) == 46);
		static_assert(std::is_trivially_copyable_v<CDH>);

		// End of central directory
		struct ECD
		{
			static uint32_t const Signature = 0x06054b50; // PK56

			uint32_t Sig;                 // Magic number signature
			uint16_t DiskNumber;          // The number of this disk in a multi-disk archive
			uint16_t CDirDiskNumber;      // The disk containing the start of the central directory
			uint16_t NumEntriesOnDisk;    // The number of entries on this disk
			uint16_t TotalEntries;        // The number of entries on the central directory
			uint32_t CDirSize;            // The central directory size
			uint32_t CDirOffset;          // Offset to the start of central directory, relative to the 'CDirDiskNumber' disk
			uint16_t CommentSize;         // ZIP comment

			// Following this header:
			//   char Comment[CommentLength];

			ECD() = default;
			ECD(int disk_number, int cdir_disk_number, int num_entries_on_disk, int total_entries, int64_t cdir_size, int64_t cdir_offset, int comment_size)
				:Sig(Signature)
				,DiskNumber(s_cast<uint16_t>(disk_number))
				,CDirDiskNumber(s_cast<uint16_t>(cdir_disk_number))
				,NumEntriesOnDisk(s_cast<uint16_t>(num_entries_on_disk))
				,TotalEntries(s_cast<uint16_t>(total_entries))
				,CDirSize(s_cast<uint32_t>(cdir_size))
				,CDirOffset(s_cast<uint32_t>(cdir_offset))
				,CommentSize(s_cast<uint16_t>(comment_size))
			{}
		};
		static_assert(sizeof(ECD) == 22);
		static_assert(std::is_trivially_copyable_v<ECD>);

		#pragma pack(pop)

	private:

		// Constants
		static size_t const LZDictionarySize = 0x8000;
		static uint32_t const DOSSubDirectoryFlag = 0x10;
		static uint32_t const InitialCrc = 0;

		// Standard library compliant allocator 
		TAlloc m_alloc;

		// The mode this archive was opened as
		EMode m_mode;

		// Construction flags
		EZipFlags m_flags;

		// The byte alignment of entries in the archive
		int m_entry_alignment;

		// In-memory copy of the central directory
		vector_t<uint8_t> m_cdir;

		// Byte offsets into 'm_cdir' to the start of each entry header
		vector_t<uint32_t> m_cdir_index;

		// A lookup table from entry name hash to central directory index
		using name_hash_index_pair_t = struct { uint64_t name_hash; int index; };
		vector_t<name_hash_index_pair_t> m_cdir_lookup;

		// The byte offset to where the item data ends and the central directory starts
		int64_t m_cdir_offset;

		// The comment associated with the whole archive
		string_t<char> m_comment;

		// The filepath for the zip archive
		std::filesystem::path m_filepath;

		// Only one of the following "zip" members should contain data
		ifstream_t<char> mutable m_istream;  // Readonly input zip stream
		ofstream_t<char> m_ostream;          // Writeable output zip stream
		span_t<uint8_t> m_imem;              // Readonly input in-memory zip
		vector_t<uint8_t> m_omem;            // Writeable output in-memory zip

		// Read/Write functions that change depending on whether the archive is in memory or a file on disk.
		using read_func_t = void(*)(ZipArchiveA const& me, int64_t file_ofs, void* buf, int64_t n);
		using write_func_t = void(*)(ZipArchiveA& me, int64_t file_ofs, void const* buf, int64_t n);
		read_func_t m_read;
		write_func_t m_write;

		// Construct an empty archive
		ZipArchiveA(EMode mode, EZipFlags flags, int entry_alignment)
			:m_alloc()
			,m_mode(mode)
			,m_flags(flags)
			,m_entry_alignment(entry_alignment)
			,m_cdir()
			,m_cdir_index()
			,m_cdir_lookup()
			,m_cdir_offset()
			,m_comment()
			,m_filepath()
			,m_istream()
			,m_ostream()
			,m_imem()
			,m_omem()
			,m_read(nullptr)
			,m_write(nullptr)
		{
			// Ensure user specified entry alignment is a power of 2.
			if (m_entry_alignment != 0 && (m_entry_alignment & (m_entry_alignment - 1)) != 0)
				throw std::runtime_error("Zip archive entry alignment must be a power of 2");
		}

	public:

		// Construct an archive in internal memory. Writeable.
		explicit ZipArchiveA(EZipFlags flags = EZipFlags::None, int entry_alignment = 0)
			:ZipArchiveA(EMode::Writeable, flags, entry_alignment)
		{
			Reset();
		}

		// Construct from an archive in memory. ReadOnly.
		explicit ZipArchiveA(void const* archive, size_t size, EZipFlags flags = EZipFlags::None, int entry_alignment = 0)
			:ZipArchiveA(span_t<uint8_t>(static_cast<uint8_t const*>(archive), size), flags, entry_alignment)
		{}
		explicit ZipArchiveA(span_t<uint8_t> archive, EZipFlags flags = EZipFlags::None, int entry_alignment = 0)
			:ZipArchiveA(EMode::ReadOnly, flags, entry_alignment)
		{
			using namespace std::literals;

			// Read/Write handlers
			m_read = &IO::ReadIMem;
			m_write = &IO::NoWrite; 

			// Parse the central directory
			m_imem = archive;
			auto archive_size = s_cast<int64_t>(m_imem.size());
			ReadCentralDirectory(archive_size);
		}

		// Construct from a zip archive file. Read or write
		explicit ZipArchiveA(std::filesystem::path const& filepath, EMode mode = EMode::ReadOnly, EZipFlags flags = EZipFlags::None, int entry_alignment = 0)
			:ZipArchiveA(mode, flags, entry_alignment)
		{
			using namespace std::literals;
			
			m_filepath = filepath;
			switch (mode)
			{
				case EMode::ReadOnly:
				{
					// File must exist
					if (!std::filesystem::exists(filepath))
						throw std::runtime_error("Zip archive file '"s + filepath.string() + "' does not exist.");

					// Read/Write handlers
					m_read = &IO::ReadIStream;
					m_write = &IO::NoWrite;

					// Read the central directory
					m_istream = IO::OpenForReading(m_filepath);
					auto archive_size = s_cast<int64_t>(std::filesystem::file_size(filepath));
					ReadCentralDirectory(archive_size);
					break;
				}
				case EMode::Writeable:
				{
					// Read/Write handlers
					m_read = &IO::ReadIStream;
					m_write = &IO::WriteOStream;

					// If the file exists, read the central directory into memory, then
					// truncate the file at the central directory offset and allow appending.
					if (std::filesystem::exists(m_filepath))
					{
						// Read the central directory into memory
						m_istream = IO::OpenForReading(m_filepath);
						auto archive_size = s_cast<int64_t>(std::filesystem::file_size(m_filepath));
						ReadCentralDirectory(archive_size);
						m_istream.close();

						// Trucate the central directory from the file
						std::filesystem::resize_file(m_filepath, m_cdir_offset);
					}

					m_ostream = IO::OpenForWriting(m_filepath);
					m_istream = IO::OpenForReading(m_filepath);
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown ZipArchive mode.");
				}
			}
		}

		// Deep copy an archive
		ZipArchiveA(ZipArchiveA const& rhs, EMode mode)
			:ZipArchiveA(mode, rhs.m_flags, rhs.m_entry_alignment)
		{
			m_cdir = rhs.m_cdir;
			m_cdir_index = rhs.m_cdir_index;
			m_cdir_lookup = rhs.m_cdir_lookup;
			m_cdir_offset = rhs.m_cdir_offset;
			m_comment = rhs.m_comment;
			m_filepath = rhs.m_filepath;

			// If readonly, create a 'view' of 'rhs'
			// If writeable, make a copy of 'rhs' in memory
			switch (mode)
			{
				case EMode::ReadOnly:
				{
					m_read = &IO::NoRead;
					m_write = &IO::NoWrite;

					// In-memory, readonly
					if (!rhs.m_imem.empty())
					{
						m_imem = rhs.m_imem;
						m_read = &IO::ReadIMem;
					}
					// Input stream, readonly
					else if (rhs.m_istream.good())
					{
						m_istream = IO::OpenForReading(m_filepath);
						m_read = &IO::ReadIStream;
					}
					// In-memory, writeable
					else if (!rhs.m_omem.empty())
					{
						m_imem = span_t<uint8_t>(rhs.m_omem.data(), rhs.m_omem.size());
						m_read = &IO::ReadOStream;
					}
					// Output stream, writeable
					else if (rhs.m_ostream.good())
					{
						m_istream = IO::OpenForReading(m_filepath);
						m_read = &IO::ReadIStream;
					}
					// Empty in-memory, writeable
					else
					{
						Reset();
					}
					break;
				}
				case EMode::Writeable:
				{
					m_read = &IO::ReadOMem;
					m_write = &IO::WriteOMem;

					// In-memory, readonly
					if (!rhs.m_imem.empty())
					{
						append(m_omem, rhs.m_imem);
					}
					// Input stream, readonly
					else if (rhs.m_istream.good())
					{
						m_omem.resize(rhs.m_cdir_offset);
						rhs.m_read(rhs, 0, m_omem.data(), m_omem.size());
					}
					// In-memory, writeable
					else if (!rhs.m_omem.empty())
					{
						m_omem = rhs.m_omem;
					}
					// Output stream, writeable
					else if (rhs.m_ostream.good())
					{
						auto ifile = IO::OpenForReading(m_filepath);
						m_omem << ifile.rdbuf();
					}
					// Empty in-memory, writeable
					else
					{
						Reset();
					}
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown archive mode");
				}
			}
		}

		// Move/Copy
		ZipArchiveA(ZipArchiveA&& rhs)
			:ZipArchiveA(rhs.m_mode, rhs.m_flags, rhs.m_entry_alignment)
		{
			m_cdir = std::move(rhs.m_cdir);
			m_cdir_index = std::move(rhs.m_cdir_index);
			m_cdir_lookup = std::move(rhs.m_cdir_lookup);
			m_cdir_offset = rhs.m_cdir_offset;
			m_comment = std::move(rhs.m_comment);
			m_filepath = std::move(rhs.m_filepath);
			m_istream = std::move(rhs.m_istream);
			m_ostream = std::move(rhs.m_ostream);
			m_imem = rhs.m_imem;
			m_omem = std::move(rhs.m_omem);
			m_read = rhs.m_read;
			m_write = rhs.m_write;
			
			rhs.Reset();
		}
		ZipArchiveA(ZipArchiveA const&) = delete;

		// Save writeable archive on destruction
		~ZipArchiveA()
		{
			Close();
		}

		// The number of items in the archive
		int Count() const
		{
			return int(m_cdir_index.size());
		}

		// Gets/Sets the comment associated with the whole archive
		std::string_view Comment() const
		{
			return m_comment;
		}
		void Comment(std::string_view comment)
		{
			if (comment.size() > 0xFFFF)
				throw std::runtime_error("Comment is too long. Maximum length is 65535 bytes");
			
			m_comment = comment;
		}

		// Return the central directory header entry for 'index'
		CDH const& CDirEntry(int index) const
		{
			using namespace std::literals;
			if (index < 0 || index >= Count())
				throw std::runtime_error("Entry index ("s + std::to_string(index) + ") out of range ("s + std::to_string(Count()) + ")"s);

			return *reinterpret_cast<CDH const*>(m_cdir.data() + m_cdir_index[index]);
		}

		// Return the local directory header entry for 'index'
		LDH LDirEntry(int index) const
		{
			return LDirEntry(CDirEntry(index));
		}
		LDH LDirEntry(CDH const& cdh) const
		{
			LDH ldh = {};

			// Read the local header from the archive source
			m_read(*this, cdh.LocalHeaderOffset, &ldh, sizeof(ldh));
			if (ldh.Sig != LDH::Signature)
				throw std::runtime_error("Item header structure is invalid. Signature mismatch");

			return ldh;
		}

		// Retrieves the name of an archive entry.
		std::string_view Name(int index) const
		{
			return CDirEntry(index).ItemName();
		}

		// Retrieves the extra data associated with an archive entry
		span_t<uint8_t> Extra(int index) const
		{
			return CDirEntry(index).Extra();
		}

		// Retrieves the comment associated with an archive entry
		std::string_view Comment(int index) const
		{
			return CDirEntry(index).Comment();
		}

		// Determines if an archive file entry is a directory entry.
		bool ItemIsDirectory(int index) const
		{
			return CDirEntry(index).IsDirectory();
		}

		// Searches the archive's central directory for an entry matching 'item_name' and 'item_comment' (if not null).
		// Valid flags: EZipFlags::IgnoreCase | EZipFlags::IgnorePath. Returns -1 if the file cannot be found.
		int IndexOf(std::string_view item_name, std::string_view item_comment = "") const
		{
			return IndexOf(item_name, item_comment, m_flags);
		}
		int IndexOf(std::string_view item_name, std::string_view item_comment, EZipFlags flags) const
		{
			if (!ValidateItemName(item_name))
				throw std::runtime_error("Item name is invalid or too long");
			if (!ValidateItemComment(item_comment))
				throw std::runtime_error("Archive comment is invalid or too long");

			// See if the lookup hash table is available.
			// Check the flags used to create the cache are the same as the flags provided here.
			if (!m_cdir_lookup.empty() && m_flags == flags)
			{
				// Get the range of items that match 'name'
				auto hash = Hash(item_name.data(), flags);
				auto matches = std::equal_range(begin(m_cdir_lookup), end(m_cdir_lookup), hash);
				for (auto p = matches.first; p != matches.second; ++p)
				{
					// Find a matching item name
					auto name = Name(p->index);
					if (Compare(item_name, name, flags) != 0)
						continue;

					// Check matching comment
					if (!item_comment.empty())
					{
						auto comment = Comment(p->index);
						if (Compare(item_comment, comment) != 0)
							continue;
					}

					// Found it
					return p->index;
				}
			}

			// Otherwise, fall back to a linear search
			else
			{
				for (int i = 0; i != Count(); ++i)
				{
					// Find a matching item name
					auto name = Name(i);
					if (Compare(item_name, name, flags) != 0)
						continue;

					// Check matching comment
					if (!item_comment.empty())
					{
						auto comment = Comment(i);
						if (Compare(item_comment, comment) != 0)
							continue;
					}

					// Found it.
					return i;
				}
			}
			
			// Not found
			return -1;
		}

		// Add already compressed data.
		// 'item_name' is the entry name for the data to be added.
		// 'buf' is the already compressed data.
		// 'method' is the method that was used to compressed the data.
		// 'uncompressed_size' is the original size of the data.
		// 'uncompressed_crc' is the crc of the uncompressed data.
		void AddAlreadyCompressed(span_t<uint8_t> buf, std::string_view item_name, size_t uncompressed_size, uint32_t uncompressed_crc32, EMethod method = EMethod::Deflate, span_t<uint8_t> extra = {}, std::string_view item_comment = "")
		{
			// Sanity checks
			if (m_mode == EMode::ReadOnly)
				throw std::runtime_error("ZipArchive is readonly");
			if (!ValidateItemName(item_name))
				throw std::runtime_error("Archive name is invalid or too long");
			if (!ValidateItemComment(item_comment))
				throw std::runtime_error("Item comment is invalid or too long");
			if (buf.size() > 0xFFFFFFFF || uncompressed_size > 0xFFFFFFFF)
				throw std::runtime_error("Data too large. Zip64 is not supported");
			if (Count() >= 0xFFFF)
				throw std::runtime_error("Too many files added.");
			if (uncompressed_size == 0)
				throw std::runtime_error("Uncompressed data size must be provided when adding already compressed data.");

			// Calculate offsets
			auto item_ofs = m_cdir_offset;
			auto num_alignment_padding_bytes = CalcAlignmentPadding();
			auto ldh_ofs = item_ofs + num_alignment_padding_bytes; // Record the header offset for later
			auto cdh_ofs = s_cast<uint32_t>(m_cdir.size());
			assert(is_aligned(ldh_ofs) && "local header offset should be aligned");

			// Overflow check
			if (item_ofs + num_alignment_padding_bytes + sizeof(LDH) + item_name.size() + extra.size() + buf.size() + // Local header plus data
				m_cdir.size() + sizeof(CDH) + item_name.size() + extra.size() + item_comment.size() +          // Central header
				sizeof(ECD) > 0xFFFFFFFF)                                                                             // Footer
				throw std::runtime_error("Zip too large. Zip64 is not supported");

			EBitFlags bit_flags = 0;
			uint16_t int_attributes = 0;
			uint32_t ext_attributes = 0;

			// Record the current time so the item can be date stamped.
			// Do this before compressing just in case compression takes a while
			auto dos_timestamp = FSTimeToDosTime(std::filesystem::file_time_type::now());

			// Reserve space for the entry in the central directory
			m_cdir.reserve(m_cdir.size() + sizeof(CDH) + item_name.size() + extra.size() + item_comment.size());
			m_cdir_index.reserve(m_cdir_index.size() + 1);
			if (has_flag(m_flags, EZipFlags::FastNameLookup))
				m_cdir_lookup.reserve(m_cdir_lookup.size() + 1);

			// Write zeros for padding
			WriteZeros(item_ofs, num_alignment_padding_bytes);
			item_ofs += num_alignment_padding_bytes;

			// Write the local directory header
			LDH ldh(item_name.size(), extra.size(), uncompressed_size, buf.size(), uncompressed_crc32, method, bit_flags, dos_timestamp.time, dos_timestamp);
			m_write(*this, item_ofs, &ldh, sizeof(ldh));
			item_ofs += sizeof(LDH);

			// Write the item name
			m_write(*this, item_ofs, item_name.data(), item_name.size());
			item_ofs += item_name.size();

			// Write the extra data
			m_write(*this, item_ofs, extra.data(), extra.size());
			item_ofs += extra.size();

			// Write the item data
			m_write(*this, item_ofs, buf.data(), buf.size());
			item_ofs += buf.size();

			// Add an entry to the central directory
			CDH cdh(item_name.size(), extra.size(), item_comment.size(), uncompressed_size, buf.size(), uncompressed_crc32, method, bit_flags, dos_timestamp, ldh_ofs, ext_attributes, int_attributes);
			append(m_cdir, &cdh, &cdh + 1);
			append(m_cdir, item_name);
			append(m_cdir, extra);
			append(m_cdir, item_comment);

			// Add the entry to the index
			m_cdir_index.push_back(cdh_ofs);
			if (has_flag(m_flags, EZipFlags::FastNameLookup))
			{
				auto hash = Hash(item_name, m_flags);
				m_cdir_lookup.push_back(name_hash_index_pair_t{ hash, int(m_cdir_index.size() - 1) });
				std::sort(begin(m_cdir_lookup), end(m_cdir_lookup));
			}

			// Move the cdir offset
			m_cdir_offset = item_ofs;
		}

		// Compresses and adds the contents of a memory buffer to the archive.
		// To add a directory entry, call this method with an archive name ending in a forward slash and an empty buffer.
		// 'item_name' is the entry name for the data to be added.
		// 'buf' is the uncompressed data to be compressed and added.
		// 'method' is the method to use to compressed the data.
		// 'uncompressed_size' is the original size of the data.
		// 'uncompressed_crc' is the crc of the uncompressed data.
		void AddBytes(span_t<uint8_t> buf, std::string_view item_name, EMethod method = EMethod::Deflate, span_t<uint8_t> extra = {}, std::string_view item_comment = "", ECompressionLevel level = ECompressionLevel::Default, EZipFlags flags = EZipFlags::None)
		{
			// Sanity checks
			if (m_mode == EMode::ReadOnly)
				throw std::runtime_error("ZipArchive is readonly");
			if (!ValidateItemName(item_name))
				throw std::runtime_error("Archive name is invalid or too long");
			if (!ValidateItemComment(item_comment))
				throw std::runtime_error("Item comment is invalid or too long");
			if (buf.size() > 0xFFFFFFFF)
				throw std::runtime_error("Data too large. Zip64 is not supported");
			if (level < ECompressionLevel::None || level > ECompressionLevel::Uber)
				throw std::runtime_error("Compression level out of range");
			if (Count() >= 0xFFFF)
				throw std::runtime_error("Too many files added.");
			if (has_flag(flags, EZipFlags::CompressedData))
				throw std::runtime_error("Use the 'AddAlreadyCompressed' function to add compressed data.");

			// Don't compress if too small
			if (buf.size() <= 3)
				level = ECompressionLevel::None;

			// Calculate offsets
			auto item_ofs = m_cdir_offset;
			auto num_alignment_padding_bytes = CalcAlignmentPadding();
			auto ldh_ofs = item_ofs + num_alignment_padding_bytes; // Record the header offset for later
			auto cdh_ofs = s_cast<uint32_t>(m_cdir.size());
			assert(is_aligned(ldh_ofs) && "local header offset should be aligned");

			EBitFlags bit_flags = EBitFlags::None;
			uint16_t int_attributes = 0;
			uint32_t ext_attributes = 0;
			int64_t compressed_size = 0;
			uint32_t crc32 = InitialCrc;

			// If the name has a directory divider at the end, set the directory bit
			if (item_name.back() == '/')
			{
				// Set DOS Subdirectory attribute bit.
				ext_attributes |= DOSSubDirectoryFlag;

				// Subdirectories cannot contain data.
				if (!buf.empty())
					throw std::runtime_error("Sub-directories cannot contain data.");
			}

			// Record the current time so the item can be date stamped. Do this before compressing just in case compression takes a while
			auto dos_timestamp = FSTimeToDosTime(std::filesystem::file_time_type::clock::now());

			// Reserve space for the entry in the central directory
			m_cdir.reserve(m_cdir.size() + sizeof(CDH) + item_name.size() + extra.size() + item_comment.size());
			m_cdir_index.reserve(m_cdir_index.size() + 1);
			if (has_flag(m_flags, EZipFlags::FastNameLookup))
				m_cdir_lookup.reserve(m_cdir_lookup.size() + 1);

			// Write zeros for padding
			WriteZeros(item_ofs, num_alignment_padding_bytes);
			item_ofs += num_alignment_padding_bytes;

			// Write a dummy local directory header. This will be overwritten once the data has been compressed
			WriteZeros(item_ofs, sizeof(LDH));
			item_ofs += sizeof(LDH);

			// Write the item name
			m_write(*this, item_ofs, item_name.data(), item_name.size());
			item_ofs += item_name.size();

			// Write the extra data
			m_write(*this, item_ofs, extra.data(), extra.size());
			item_ofs += extra.size();

			// Calculate the uncompressed crc
			if (!has_flag(flags, EZipFlags::IgnoreCrc))
				crc32 = Crc(buf.data(), buf.size());

			// Add the data
			auto item_ofs_beg = item_ofs;
			if (level == ECompressionLevel::None)
			{
				// Write the raw data
				m_write(*this, item_ofs, buf.data(), buf.size());
				item_ofs += buf.size();
				
				// Record the stats
				compressed_size = item_ofs - item_ofs_beg;
				method = EMethod::None;
			}
			else
			{
				Deflate algo = {};

				// Compress into a local buffer and periodically flush to the output
				algo.Compress(buf.data(), buf.size(), [&](auto const& out)
				{
					m_write(*this, item_ofs, out.data(), out.size());
					item_ofs += out.size();
				});

				// Record the stats
				compressed_size = item_ofs - item_ofs_beg;
				method = EMethod::Deflate;
			}

			auto item_name_size = static_cast<int>(item_name.size());
			auto extra_size = static_cast<int>(extra.size());
			auto item_comment_size = static_cast<int>(item_comment.size());
			auto buf_size = static_cast<int64_t>(buf.size());

			// Write the local directory header now that we have the compressed size
			LDH ldh(item_name_size, extra_size, buf_size, compressed_size, crc32, method, bit_flags, dos_timestamp);
			m_write(*this, ldh_ofs, &ldh, sizeof(ldh));

			// Add an entry to the central directory
			CDH cdh(item_name_size, extra_size, item_comment_size, buf_size, compressed_size, crc32, method, bit_flags, dos_timestamp, ldh_ofs, ext_attributes, int_attributes);
			append(m_cdir, &cdh, &cdh + 1);
			append(m_cdir, item_name);
			append(m_cdir, extra);
			append(m_cdir, item_comment);

			// Add the entry to the index
			m_cdir_index.push_back(cdh_ofs);
			if (has_flag(m_flags, EZipFlags::FastNameLookup))
			{
				auto hash = Hash(item_name, m_flags);
				m_cdir_lookup.push_back(name_hash_index_pair_t{ hash, int(m_cdir_index.size() - 1) });
				std::sort(begin(m_cdir_lookup), end(m_cdir_lookup));
			}

			// Move the cdir offset
			m_cdir_offset = item_ofs;
		}
		void AddBytes(void const* data, size_t len, std::string_view item_name, EMethod method = EMethod::Deflate, span_t<uint8_t> extra = {}, std::string_view item_comment = "", ECompressionLevel level = ECompressionLevel::Default, EZipFlags flags = EZipFlags::None)
		{
			auto buf = span_t<uint8_t>(static_cast<uint8_t const*>(data), len);
			AddBytes(buf, item_name, method, extra, item_comment, level, flags);
		}
		void AddBytes(std::vector<uint8_t> const& data, std::string_view item_name, EMethod method = EMethod::Deflate, span_t<uint8_t> extra = {}, std::string_view item_comment = "", ECompressionLevel level = ECompressionLevel::Default, EZipFlags flags = EZipFlags::None)
		{
			auto buf = span_t<uint8_t>(data.data(), data.size());
			AddBytes(buf, item_name, method, extra, item_comment, level, flags);
		}
		void AddString(std::string_view data, std::string_view item_name, EMethod method = EMethod::Deflate, span_t<uint8_t> extra = {}, std::string_view item_comment = "", ECompressionLevel level = ECompressionLevel::Default, EZipFlags flags = EZipFlags::None)
		{
			auto buf = span_t<uint8_t>(reinterpret_cast<uint8_t const*>(data.data()), data.size());
			AddBytes(buf, item_name, method, extra, item_comment, level, flags);
		}

		// Compresses and adds the contents of a disk file to an archive.
		// To add a directory entry, call this method with an archive name ending in a forward slash and an empty buffer.
		// 'item_name' is the entry name for the data to be added.
		// 'buf' is the uncompressed data to be compressed and added.
		// 'method' is the method to use to compressed the data.
		// 'uncompressed_size' is the original size of the data.
		// 'uncompressed_crc' is the crc of the uncompressed data.
		void AddFile(std::filesystem::path const& src_filepath, std::string_view item_name = "", EMethod method = EMethod::Deflate, span_t<uint8_t> extra = {}, std::string_view item_comment = "", ECompressionLevel level = ECompressionLevel::Default, EZipFlags flags = EZipFlags::None)
		{
			using namespace std::literals;

			// Default the item name to the filename
			auto filename = src_filepath.filename().string();
			if (item_name.empty())
				item_name = filename;

			// Sanity checks
			if (m_mode == EMode::ReadOnly)
				throw std::runtime_error("ZipArchive is readonly");
			if (!ValidateItemName(item_name))
				throw std::runtime_error("Archive name is invalid or too long");
			if (!ValidateItemComment(item_comment))
				throw std::runtime_error("Item comment is invalid or too long");
			if (!std::filesystem::exists(src_filepath))
				throw std::runtime_error("Path '"s + src_filepath.string() + "' does not exist");
			if (std::filesystem::is_directory(src_filepath))
				throw std::runtime_error("Path '"s + src_filepath.string() + "' is not a file");
			if (std::filesystem::file_size(src_filepath) > 0xFFFFFFFF)
				throw std::runtime_error("File '"s + src_filepath.string() + "' is too large. Zip64 is not supported");
			if (level < ECompressionLevel::None || level > ECompressionLevel::Uber)
				throw std::runtime_error("Compression level out of range");
			if (has_flag(flags, EZipFlags::CompressedData))
				throw std::runtime_error("Use the 'AddAlreadyCompressed' function to add compressed data.");
			if (Count() >= 0xFFFF)
				throw std::runtime_error("Too many files added.");

			// Open the source file
			auto src_file = IO::OpenForReading(src_filepath);
			if (!src_file.good())
				throw std::runtime_error("Failed to open file '"s + src_filepath.string() + "'");

			// Calculate offsets
			auto item_ofs = m_cdir_offset;
			auto num_alignment_padding_bytes = CalcAlignmentPadding();
			auto ldh_ofs = item_ofs + num_alignment_padding_bytes; // Record the header offset for later
			auto cdh_ofs = s_cast<uint32_t>(m_cdir.size());
			assert(is_aligned(ldh_ofs) && "local header offset should be aligned");

			EBitFlags bit_flags = EBitFlags::None;
			uint16_t int_attributes = 0;
			uint32_t ext_attributes = 0;
			int64_t compressed_size = 0;
			int64_t uncompressed_size = static_cast<int64_t>(std::filesystem::file_size(src_filepath));
			uint32_t crc32 = InitialCrc;

			// Don't compress if too small
			if (uncompressed_size <= 3)
				level = ECompressionLevel::None;

			// Record the current time so the item can be date stamped.
			// Do this before compressing just in case compression takes a while
			auto dos_timestamp = FSTimeToDosTime(std::filesystem::file_time_type::clock::now());

			// Reserve space for the entry in the central directory
			m_cdir.reserve(m_cdir.size() + sizeof(CDH) + item_name.size() + extra.size() + item_comment.size());
			m_cdir_index.reserve(m_cdir_index.size() + 1);
			if (has_flag(m_flags, EZipFlags::FastNameLookup))
				m_cdir_lookup.reserve(m_cdir_lookup.size() + 1);

			// Write zeros for padding
			WriteZeros(item_ofs, num_alignment_padding_bytes);
			item_ofs += num_alignment_padding_bytes;

			// Write a dummy local directory header. This will be overwritten once the data has been compressed
			WriteZeros(ldh_ofs, sizeof(LDH));
			item_ofs += sizeof(LDH);

			// Write the item name
			m_write(*this, item_ofs, item_name.data(), item_name.size());
			item_ofs += item_name.size();

			// Write the extra data
			m_write(*this, item_ofs, extra.data(), extra.size());
			item_ofs += extra.size();

			// Calculate the uncompressed crc
			if (!has_flag(flags, EZipFlags::IgnoreCrc))
				crc32 = Crc(src_file);

			// Write the compressed data
			auto item_ofs_beg = item_ofs;
			if (level == ECompressionLevel::None)
			{
				// Read from the file in blocks
				std::array<char, 4096> buf = {};
				for (; src_file.good();)
				{
					auto n = src_file.read(buf.data(), buf.size()).gcount();
					m_write(*this, item_ofs, buf.data(), static_cast<size_t>(n));
					item_ofs += n;
				}
				
				// Record the stats
				compressed_size = item_ofs - item_ofs_beg;
				method = EMethod::None;
			}
			else
			{
				Deflate algo = {};
				
				src_file.seekg(0);
				auto src = std::istream_iterator<uint8_t>(src_file);

				// Compress into a local buffer and periodically flush to the output
				algo.Compress(src, uncompressed_size, [&](auto const& out)
				{
					m_write(*this, item_ofs, out.data(), out.size());
					item_ofs += out.size();
				});

				// Record the stats
				compressed_size = item_ofs - item_ofs_beg;
				method = EMethod::Deflate;
			}

			// Write the local directory header now that we have the compressed size
			LDH ldh(static_cast<int>(item_name.size()), static_cast<int>(extra.size()), uncompressed_size, compressed_size, crc32, method, bit_flags, dos_timestamp);
			m_write(*this, ldh_ofs, &ldh, sizeof(ldh));

			// Add an entry to the central directory
			CDH cdh(static_cast<int>(item_name.size()), static_cast<int>(extra.size()), static_cast<int>(item_comment.size()), uncompressed_size, compressed_size, crc32, method, bit_flags, dos_timestamp, ldh_ofs, ext_attributes, int_attributes);
			append(m_cdir, &cdh, &cdh + 1);
			append(m_cdir, item_name);
			append(m_cdir, extra);
			append(m_cdir, item_comment);

			// Add the entry to the index
			m_cdir_index.push_back(cdh_ofs);
			if (has_flag(m_flags, EZipFlags::FastNameLookup))
			{
				auto hash = Hash(item_name, m_flags);
				m_cdir_lookup.push_back(name_hash_index_pair_t{ hash, int(m_cdir_index.size() - 1) });
				std::sort(begin(m_cdir_lookup), end(m_cdir_lookup));
			}

			// Move the cdir offset
			m_cdir_offset = item_ofs;
		}

		// Extract all items in the archive to the given directory
		// 'progress' is a callback of the number of items extracted. Sig: bool progress_cb(CDH const& info, int index, int count)
		void ExtractAll(std::filesystem::path const& directory) const
		{
			ExtractAll(directory, [](CDH const&, int, int) {});
		}
		template <typename ProgressCB>
		void ExtractAll(std::filesystem::path const& directory, ProgressCB progress) const
		{
			// Create the output directory if it doesn't exist
			if (!std::filesystem::exists(directory))
				std::filesystem::create_directories(directory);

			// Create the subdirectories included in the archive
			for (int i = 0, iend = Count(); i != iend; ++i)
			{
				auto& info = CDirEntry(i);
				if (!info.IsDirectory())
					continue;

				auto rel_path = ItemPath(info.ItemName());
				std::filesystem::create_directories(directory / rel_path);
			}

			// Extract each archive item
			for (int i = 0, iend = Count(); i != iend; ++i)
			{
				auto& info = CDirEntry(i);
				if (info.IsDirectory())
					continue;

				auto path = directory / ItemPath(info.ItemName());

				progress(info, i, iend);
				Extract(info.ItemName(), path);
				progress(info, i + 1, iend);
			}
		}

		// Extracts an archive entry to disk and restores its last accessed and modified times.
		// 'item_name' is the entry name for the item to extract.
		// 'dst_filepath' is where to write the extracted file.
		// 'index' is the item's index within the archive.
		// This function only extracts files, not archive directory records.
		void Extract(std::string_view item_name, std::filesystem::path const& dst_filepath) const
		{
			Extract(item_name, dst_filepath, m_flags);
		}
		void Extract(std::string_view item_name, std::filesystem::path const& dst_filepath, EZipFlags flags) const
		{
			auto index = IndexOf(item_name, "", flags);
			return index >= 0 && index < Count()
				? Extract(index, dst_filepath, flags)
				: throw std::runtime_error("Archive item not found");
		}
		void Extract(int index, std::filesystem::path const& dst_filepath) const
		{
			Extract(index, dst_filepath, m_flags);
		}
		void Extract(int index, std::filesystem::path const& dst_filepath, EZipFlags flags) const
		{
			// Create the destination file
			std::ofstream outfile(dst_filepath, std::ios::binary);
			Extract(index, outfile, flags);
			outfile.close();

			// Set the file time on the extracted file to match the times recorded in the archive
			auto stat = CDirEntry(index);
			std::filesystem::last_write_time(dst_filepath, stat.Time());
		}

		// Extracts an archive entry to a stream.
		// 'item_name' is the entry name for the item to extract.
		// 'out' is where to write the extracted file.
		// 'index' is the item's index within the archive.
		// This function only extracts files, not archive directory records.
		template <typename Elem = uint8_t> void Extract(std::string_view item_name, std::basic_ostream<Elem> & out) const
		{
			Extract(item_name, out, m_flags);
		}
		template <typename Elem = uint8_t> void Extract(std::string_view item_name, std::basic_ostream<Elem>& out, EZipFlags flags) const
		{
			auto index = IndexOf(item_name, "", flags);
			return index >= 0 && index < Count()
				? Extract(index, out, flags)
				: throw std::runtime_error("Archive item not found");
		}
		template <typename Elem = uint8_t> void Extract(int index, std::basic_ostream<Elem> & out) const
		{
			void Extract(index, out, m_flags);
		}
		template <typename Elem = uint8_t> void Extract(int index, std::basic_ostream<Elem>& out, EZipFlags flags) const
		{
			Extract(index, [](void* ctx, uint64_t ofs, void const* buf, size_t n)
			{
				auto& out = *static_cast<std::basic_ostream<Elem>*>(ctx);
				out.seekp(ofs);
				out.write(static_cast<Elem const*>(buf), n / sizeof(Elem));
			}, &out, flags);
		}

		// Extracts a archive entry using a callback function to output the uncompressed data.
		// OutputCB = void(void* ctx, uint64_t output_buffer_ofs, void const* buf, size_t len)
		// OutputCB is expected to copy 'buf[0:len) to '&somewhere[output_buffer_ofs]'.
		// OutputCB should throw if not all bytes can be copied.
		// 'output_buffer_ofs' is a convenience for output streams that do not have an internal 'file' pointer.
		template <typename OutputCB> void Extract(std::string_view item_name, OutputCB callback, void* ctx) const
		{
			Extract(item_name, callback, ctx, m_flags);
		}
		template <typename OutputCB> void Extract(std::string_view item_name, OutputCB callback, void* ctx, EZipFlags flags) const
		{
			auto index = IndexOf(item_name, "", flags);
			Extract(index, callback, ctx, flags);
		}
		template <typename OutputCB> void Extract(int index, OutputCB callback, void* ctx) const
		{
			Extract(index, callback, ctx, m_flags);
		}
		template <typename OutputCB> void Extract(int index, OutputCB callback, void* ctx, EZipFlags flags) const
		{
			using namespace std::literals;

			// Empty file, or a directory (but not always a directory - I've seen odd zips with directories that have compressed data which inflates to 0 bytes)
			auto const& cdh = CDirEntry(index);
			if (cdh.CompressedSize == 0 || cdh.IsDirectory())
				return;

			// Encryption and patch files are not supported.
			if (has_flag(cdh.BitFlags, EBitFlags::PatchFile))
				throw std::runtime_error("Patch files are not supported");
			if (has_flag(cdh.BitFlags, EBitFlags::Encrypted))
				throw std::runtime_error("Encrypted files are not supported");

			// This function only supports stored and deflate.
			if (cdh.Method != EMethod::Deflate && cdh.Method != EMethod::None && !has_flag(flags, EZipFlags::CompressedData))
				throw std::runtime_error("Unsupported compression method type: "s + std::to_string(int(cdh.Method)));

			// Read and parse the local directory entry.
			LDH ldh = LDirEntry(cdh);
			if (ldh.Sig != LDH::Signature)
				throw std::runtime_error("Item header structure is invalid. Signature mismatch");

			// Check the item size is within range
			if (int64_t(cdh.LocalHeaderOffset + ldh.Size()) > m_cdir_offset)
				throw std::runtime_error("Archive corrupt. Indicated item size exceeds the available data");

			// From input memory stream
			if (!m_imem.empty())
				return ExtractFromMemory(callback, ctx, cdh, ldh, flags);
			if (m_istream.good())
				return ExtractFromFile(callback, ctx, cdh, ldh, flags);
			
			throw std::runtime_error("Input data stream not available");
		}

		// Finalise the zip archive. Use this overload for writeable file archives
		void Save()
		{
			// Is there a use case for writing a readonly archive?
			if (m_mode != EMode::Writeable)
				throw std::runtime_error("Only writeable archives can be saved");
			if (m_filepath.empty())
				throw std::runtime_error("Parameterless save can only be used with file archives opened in Writeable mode");
			if (!m_ostream.good())
				throw std::runtime_error("Output file stream is in an error state");

			// Write the central directory and end marker record
			WriteCentralDirectory();
		}
		
		// Write the zip archive to a file. Use this overload for saving in-memory archives
		void Save(std::filesystem::path const& filepath)
		{
			using namespace std::literals;

			// Is there a use case for writing a readonly archive?
			if (m_mode != EMode::Writeable)
				throw std::runtime_error("Only writeable archives can be saved");
			if (m_omem.empty())
				throw std::runtime_error("Save method can only be used with in-memory archives opened in Writeable mode");

			// Write the central directory and end marker record
			WriteCentralDirectory();

			// Copy the archive to a file
			std::basic_ofstream<uint8_t> ofile(filepath, std::ios::binary);
			if (!ofile.write(m_omem.data(), m_omem.size()).good())
				throw std::runtime_error("Saving archive '"s + filepath.string() + "' failed.");
		}

		// Close the archive
		void Close()
		{
			// If the archive is a writeable file archive, save any changes to disk
			// In memory archives will just be dropped.
			if (m_mode == EMode::Writeable && !m_filepath.empty() && m_ostream.good())
				Save();

			Reset();
		}

		// Reset the archive to that of an empty writeable one
		void Reset()
		{
			m_mode = EMode::Writeable;
			m_cdir.resize(0);
			m_cdir_index.resize(0);
			m_cdir_lookup.resize(0);
			m_cdir_offset = 0;
			m_filepath = "";
			m_comment = "";
			m_imem = span_t<uint8_t>{};
			m_omem.resize(0);
			m_istream.close();
			m_ostream.close();

			// Read/Write handlers
			m_read = &IO::ReadOMem;
			m_write = &IO::WriteOMem;
		}

	private:

		// Find the byte offset to the end of central directory record
		int64_t FindECDOffset(int64_t archive_size) const
		{
			int64_t ofs = 0;

			// Search backwards from the end of the data
			if (has_flag(m_flags, EZipFlags::ScanFromEnd))
			{
				std::array<uint8_t, 4096> buf = {};
				for (uint32_t sig = 0;;)
				{
					// Read a chunk from the end of the archive
					auto n = std::min<int64_t>(buf.size(), ofs);
					m_read(*this, ofs - n, buf.data(), n);
					ofs -= n;

					// Search backwards for the ECD marker
					for (; n-- != 0;)
					{
						sig = (sig << 8) | buf[static_cast<size_t>(n)];
						if (sig == ECD::Signature) break;
					}
					if (ofs == 0 && n == -1)
						throw std::runtime_error("Invalid zip. Central directory header not found");
					if (n == -1)
						continue;

					// Found the CDH end marker at '@buf[n]', move 'ofs' to the start of the ECD.
					ofs += n;
					break;
				}
			}

			// Traverse the data forwards
			else
			{
				for (uint32_t sig = 0; sig != ECD::Signature;)
				{
					m_read(*this, ofs, &sig, sizeof(sig));
					switch (sig)
					{
						case LDH::Signature:
						{
							LDH ldh = {};
							m_read(*this, ofs, &ldh, sizeof(ldh));
							ofs += ldh.Size();
							break;
						}
						case CDH::Signature:
						{
							CDH cdh = {};
							m_read(*this, ofs, &cdh, sizeof(cdh));
							ofs += cdh.Size();
							break;
						}
						case ECD::Signature:
						{
							// Found it
							break;
						}
						default:
						{
							throw std::runtime_error("Corrupt zip archive. Record signature is invalid");
						}
					}
					if (ofs >= archive_size)
						throw std::runtime_error("Corrupt zip archive. End of Central Directory record not found");
				}
			}

			return ofs;
		}

		// Read the top level directory structure contained in the zip and populate our state variables
		void ReadCentralDirectory(int64_t archive_size)
		{
			// Read and verify the end of central directory record.
			ECD ecd = {};
			auto ofs = FindECDOffset(archive_size);
			m_read(*this, ofs, &ecd, sizeof(ecd));
			if (ecd.Sig != ECD::Signature)
				throw std::runtime_error("Invalid zip. Central directory end marker not found");
			if (ecd.TotalEntries != ecd.NumEntriesOnDisk || ecd.DiskNumber > 1)
				throw std::runtime_error("Invalid zip. Archives that span multiple disks are not supported");
			if (ecd.CDirSize < ecd.TotalEntries * sizeof(CDH))
				throw std::runtime_error("Invalid zip. Central directory size is invalid");
			if (ecd.CDirOffset + ecd.CDirSize > archive_size)
				throw std::runtime_error("Invalid zip. Central directory size exceeds archive size");

			// Read the central directory into memory.
			m_cdir_offset = ecd.CDirOffset;
			m_cdir.resize(ecd.CDirSize);
			m_cdir_index.resize(ecd.TotalEntries);
			m_read(*this, ecd.CDirOffset, m_cdir.data(), m_cdir.size());

			// Read the archive comment
			m_comment.resize(ecd.CommentSize);
			m_read(*this, ofs + sizeof(ECD), m_comment.data(), ecd.CommentSize);

			// Populate the index of offsets into the central directory
			auto p = m_cdir.data();
			for (size_t n = ecd.CDirSize, i = 0; i != ecd.TotalEntries; ++i)
			{
				auto const& cdh = *reinterpret_cast<CDH const*>(p);

				// Sanity checks
				if (n < sizeof(CDH) || cdh.Sig != CDH::Signature)
					throw std::runtime_error("Invalid zip. Central directory header corrupt");
				if ((cdh.UncompressedSize != 0 && cdh.CompressedSize == 0) || cdh.UncompressedSize == 0xFFFFFFFF || cdh.CompressedSize == 0xFFFFFFFF)
					throw std::runtime_error("Invalid zip. Compressed and Decompressed sizes are invalid");
				if (cdh.Method == EMethod::None && cdh.UncompressedSize != cdh.CompressedSize)
					throw std::runtime_error("Invalid zip. Header indicates no compression, but compressed and decompressed sizes differ");
				if (cdh.DiskNumberStart != ecd.DiskNumber && cdh.DiskNumberStart != 1)
					throw std::runtime_error("Unsupported zip. Archive spans multiple disks");
				if (cdh.LocalHeaderOffset + sizeof(LDH) + cdh.CompressedSize > size_t(archive_size))
					throw std::runtime_error("Invalid zip. Item size value exceeds actual data size");
				if (cdh.Size() > n)
					throw std::runtime_error("Invalid zip. Computed header size does not agree header end signature location");
				if (!ValidateItemName(cdh.ItemName()))
					throw std::runtime_error("Invalid zip. Central directory item name is invalid");

				m_cdir_index[i] = s_cast<uint32_t>(p - m_cdir.data());
				n -= cdh.Size();
				p += cdh.Size();
			}

			// Generate a lookup table from name (hashed) to index
			if (has_flag(m_flags, EZipFlags::FastNameLookup))
			{
				m_cdir_lookup.reserve(ecd.TotalEntries);
				for (int i = 0, iend = int(m_cdir_index.size()); i != iend; ++i)
				{
					auto name = Name(i);
					auto hash = Hash(name, m_flags);
					m_cdir_lookup.push_back(name_hash_index_pair_t{ hash, i });
				}
				std::sort(begin(m_cdir_lookup), end(m_cdir_lookup));
			}
		}

		// Finalise an archive by writing the central directory headers and end marker record
		void WriteCentralDirectory()
		{
			auto ofs = m_cdir_offset;

			// Write the central directory headers
			m_write(*this, ofs, m_cdir.data(), m_cdir.size());
			ofs += m_cdir.size();

			// Write the end marker record
			auto total_entries = s_cast<uint16_t>(Count());
			auto comment_size = s_cast<uint16_t>(m_comment.size());
			auto cdir_size = s_cast<uint32_t>(m_cdir.size());
			auto cdir_offset = s_cast<uint32_t>(m_cdir_offset);
			ECD ecd(0, 0, total_entries, total_entries, cdir_size, cdir_offset, comment_size);
			m_write(*this, ofs, &ecd, sizeof(ECD));
			ofs += sizeof(ECD);

			// Write the archive comment
			m_write(*this, ofs, m_comment.data(), comment_size);
			ofs += comment_size;
		}

		// Return the required padding needed to align an item in the archive
		int CalcAlignmentPadding() const
		{
			if (m_entry_alignment == 0) return 0;
			auto n = static_cast<int>(m_cdir_offset) & (m_entry_alignment - 1);
			return (m_entry_alignment - n) & (m_entry_alignment - 1);
		}

		// Write zeros into the output
		void WriteZeros(int64_t ofs, size_t count)
		{
			static char const zeros[1024] = {};
			while (count)
			{
				auto sz = std::min<size_t>(sizeof(zeros), count);
				m_write(*this, ofs, zeros, sz);
				ofs += sz;
				count -= sz;
			}
		}

		// Extract from a zip archive in memory
		template <typename OutputCB>
		void ExtractFromMemory(OutputCB callback, void* ctx, CDH const& cdh, LDH const& ldh, EZipFlags flags) const
		{
			using namespace std::literals;
			if (m_imem.empty())
				throw std::runtime_error("There is no in-memory archive");

			int64_t item_ofs = cdh.LocalHeaderOffset + ldh.ItemDataOffset();
			uint32_t crc32 = InitialCrc;
			uint64_t ofs = 0;

			// The item was stored uncompressed or the caller has requested the compressed data.
			if (cdh.Method == EMethod::None || has_flag(flags, EZipFlags::CompressedData))
			{
				// Zip64 check
				if constexpr (sizeof(size_t) == sizeof(uint32_t))
					if (cdh.CompressedSize > 0xFFFFFFFF)
						throw std::runtime_error("Item is too large. Zip64 is not supported");

				// Calculate the crc if the call was not just for the compressed data
				if (!has_flag(flags, EZipFlags::CompressedData) && !has_flag(flags, EZipFlags::IgnoreCrc))
					crc32 = Crc(m_imem.data() + item_ofs, cdh.CompressedSize, crc32);

				// Send the data directly to the callback
				callback(ctx, ofs, m_imem.data() + item_ofs, size_t(cdh.CompressedSize));

				// All data sent
				item_ofs += cdh.CompressedSize;
				ofs += cdh.CompressedSize;

				// CRC check
				if (!has_flag(flags, EZipFlags::CompressedData) && !has_flag(flags, EZipFlags::IgnoreCrc) && cdh.Crc != crc32)
					throw std::runtime_error("CRC check failure");

				return;
			}

			// Data is compressed, inflate before passing to callback
			if (cdh.Method == EMethod::Deflate)
			{
				// Decompress into a temporary buffer. The minimum buffer size must be 'LZDictionarySize'
				// because Deflate uses references to earlier bytes, up to an LZ dictionary size prior.
				Deflate algo = {};
				algo.Decompress(m_imem.data() + item_ofs, cdh.CompressedSize, [&](auto const& out)
				{
					// Check for overflow
					if (ofs + out.size() > ldh.UncompressedSize)
						throw std::runtime_error("Output buffer overflow");

					// Update the crc
					if (!has_flag(flags, EZipFlags::IgnoreCrc))
						crc32 = Crc(out.data(), out.size(), crc32);

					// Push the buffered data out to the callback
					callback(ctx, ofs, out.data(), out.size());
					ofs += out.size();
				});

				// CRC check
				if (!has_flag(flags, EZipFlags::IgnoreCrc) && cdh.Crc != crc32)
					throw std::runtime_error("CRC check failure");

				return;
			}

			throw std::runtime_error("Unsupported compression method:"s + std::to_string(int(cdh.Method)));
		}

		// Extract from a zip archive file
		template <typename OutputCB>
		void ExtractFromFile(OutputCB callback, void* ctx, CDH const& cdh, LDH const& ldh, EZipFlags flags) const
		{
			using namespace std::literals;
			if (!m_istream.good())
				throw std::runtime_error("There is no archive file");

			int64_t item_ofs = cdh.LocalHeaderOffset + ldh.ItemDataOffset();
			uint32_t crc32 = InitialCrc;
			uint64_t ofs = 0;

			// The item was stored uncompressed or the caller has requested the compressed data.
			if (cdh.Method == EMethod::None || has_flag(flags, EZipFlags::CompressedData))
			{
				// Zip is a file. Read chunks into a temporary buffer
				std::array<uint8_t, 4096> buf = {};
				for (size_t remaining = cdh.CompressedSize; remaining != 0;)
				{
					// Read chunk
					auto n = std::min<size_t>(buf.size(), remaining);
					m_read(*this, item_ofs, buf.data(), n);

					// Calculate the crc if the call was not just for the compressed data
					if (!has_flag(flags, EZipFlags::CompressedData) && !has_flag(flags, EZipFlags::IgnoreCrc))
						crc32 = Crc(buf.data(), n, crc32);

					// Send the data directly to the callback
					callback(ctx, ofs, buf.data(), n);

					// Accumulate
					remaining -= n;
					item_ofs += n;
					ofs += n;
				}

				// Check the CRC
				if (!has_flag(flags, EZipFlags::CompressedData) && !has_flag(flags, EZipFlags::IgnoreCrc) && cdh.Crc != crc32)
					throw std::runtime_error("CRC check failure");

				return;
			}

			// Data is compressed, inflate before passing to callback
			if (cdh.Method == EMethod::Deflate)
			{
				m_istream.seekg(item_ofs);
				auto src = std::istream_iterator<uint8_t>(m_istream);

				Deflate algo = {};
				algo.Decompress(src, cdh.CompressedSize, [&](auto const& out)
				{
					// Check for overflow
					if (ofs + out.size() > ldh.UncompressedSize)
						throw std::runtime_error("Output buffer overflow");

					// Update the CRC
					if (!has_flag(flags, EZipFlags::IgnoreCrc))
						crc32 = Crc(out.data(), out.size(), crc32);

					// Push the buffered data out to the callback
					callback(ctx, ofs, out.data(), out.size());
					ofs += out.size();
				});

				// Check the CRC
				if (!has_flag(flags, EZipFlags::IgnoreCrc) && cdh.Crc != crc32)
					throw std::runtime_error("CRC check failure");

				return;
			}

			throw std::runtime_error("Unsupported compression method:"s + std::to_string(int(cdh.Method)));
		}

		// Lexicographically compare strings
		template <bool IgnorePath = false, bool IgnoreCase = false> static int Compare(std::string_view lhs, std::string_view rhs)
		{
			// One range empty => the empty range is less. Both ranges empty => equal
			if (lhs.empty() || rhs.empty())
				return int(rhs.empty()) - int(lhs.empty());

			// The ranges to compare
			auto lhs_beg = begin(lhs);
			auto rhs_beg = begin(rhs);
			auto lhs_end = end(lhs);
			auto rhs_end = end(rhs);

			// Exclude everything prior to the last '/', '\\', ':' character
			if constexpr (IgnorePath)
			{
				auto PathDivider = [](char c) { return c == '/' || c == '\\' || c == ':'; };

				auto p = lhs_end;
				for (p = lhs_end; p-- != lhs_beg && !PathDivider(*p);) {}
				lhs_beg = p + int(p != lhs_beg);
				for (p = rhs_end; p-- != rhs_beg && !PathDivider(*p);) {}
				rhs_beg = p + int(p != rhs_beg);
			}

			// Compare ordinal
			for (;lhs_beg != lhs_end && rhs_beg != rhs_end;)
			{
				int c = IgnoreCase
					? std::tolower(*lhs_beg++) - std::tolower(*rhs_beg++)
					: *lhs_beg++ - *rhs_beg++;

				if (c != 0)
					return c;
			}

			return int(rhs_beg == rhs_end) - int(lhs_beg == lhs_end);
		}
		static int Compare(std::string_view lhs, std::string_view rhs, EZipFlags flags = EZipFlags::None)
		{
			auto ignore_path = has_flag(flags, EZipFlags::IgnorePath);
			auto ignore_case = has_flag(flags, EZipFlags::IgnoreCase);
			return
				!ignore_path && !ignore_case ? Compare<false, false>(lhs, rhs) :
				!ignore_path &&  ignore_case ? Compare<false, true>(lhs, rhs) :
				 ignore_path && !ignore_case ? Compare<true, false>(lhs, rhs) :
				 ignore_path &&  ignore_case ? Compare<true, true>(lhs, rhs) :
				throw std::runtime_error("");
		}
		static bool Equals(std::string_view lhs, std::string_view rhs, EZipFlags flags = EZipFlags::None)
		{
			return Compare(lhs, rhs, flags) == 0;
		}

		// Generate a hash of 'name' based on 'flags'
		static uint64_t Hash(std::string_view name, EZipFlags flags)
		{
			if (name.empty())
				return 0;

			// Hash from end to start so that 'IgnorePath' quick outs at the first path divider
			uint64_t hash = 0;
			auto end = rend(name);
			auto p = rbegin(name);
			for (p += *p == '/'; p != end; ++p) // Skip the last '/' for sub-directories
			{
				hash = Hash64CT(has_flag(flags, EZipFlags::IgnoreCase) ? std::tolower(*p) : *p, hash);
				if (has_flag(flags, EZipFlags::IgnorePath) && (*p == '/' || *p == '\\' || *p == ':'))
					break;
			}
			return hash;
		}

		// Return the zip version required for the given compression method
		static uint16_t VersionFor(EMethod method)
		{
			switch (method)
			{
			default: throw std::runtime_error("Unsupported compression method. Version needed unknown");
			case EMethod::None: return 0x0000;
			case EMethod::Deflate: return 0x0014;
			}
		}

		// Validate an archive item name
		static bool ValidateItemName(std::string_view item_name)
		{
			// Archive names are stored as relative paths. Reject rooted, drive-relative, and parent-directory components so
			// extraction can only occur beneath the caller's chosen destination root.
			if (item_name.empty())
				return false;
			if (item_name.size() > 0xFFFF)
				return false;
			if (item_name[0] == '/')
				return false;
			if (item_name.find('\\') != std::string_view::npos)
				return false;
			if (item_name.find(':') != std::string_view::npos)
				return false;

			auto path = std::filesystem::path(item_name);
			if (path.has_root_name() || path.has_root_directory() || path.is_absolute())
				return false;
			for (auto const& part : path)
			{
				if (part == "." || part == "..")
					return false;
			}
			return true;
		}

		// Turn a validated archive item name into a filesystem path relative to the extraction root.
		static std::filesystem::path ItemPath(std::string_view item_name)
		{
			if (!ValidateItemName(item_name))
				throw std::runtime_error("Archive item name is invalid or too long");
			return std::filesystem::path(item_name);
		}

		// Validate an archive item comment
		static bool ValidateItemComment(std::string_view item_comment)
		{
			if (item_comment.size() > 0xFFFF)
				return false;
			return true;
		}

		// Convert a MS DOS file time to a standard filesystem time
		static std::filesystem::file_time_type DosTimeToFSTime(MSDosTimestamp dos_timestamp)
		{
			using namespace std::filesystem;
			using namespace std::chrono;

			// DOS date/time: https://docs.microsoft.com/en-us/windows/desktop/api/winbase/nf-winbase-dosdatetimetofiletime
			struct tm tm = {};
			tm.tm_isdst  = -1;
			tm.tm_year   = ((dos_timestamp.date >> 9) & 127) + 1980 - 1900;
			tm.tm_mon    = ((dos_timestamp.date >> 5) & 15) - 1;
			tm.tm_mday   = ((dos_timestamp.date     ) & 31);
			tm.tm_hour   = (dos_timestamp.time >> 11) & 31;
			tm.tm_min    = (dos_timestamp.time >>  5) & 63;
			tm.tm_sec    = (dos_timestamp.time <<  1) & 62;
			auto time    = mktime(&tm);

			#if defined (_MSC_VER)
			// In VS, 'file_time_type' is in 100s of nanoseconds since 1601-01-01 (i.e. FILETIME).
			// For whatever reason, 'from_time_t' isn't defined, so
			return file_time_type(file_time_type::duration(static_cast<long long>(time) * 10000000LL + 116444736000000000LL));
			#else
			return file_time_type::clock::from_time_t(time);
			#endif
		}

		// Convert a standard filesystem time to a MS DOS time and date
		static MSDosTimestamp FSTimeToDosTime(std::filesystem::file_time_type last_mod_time)
		{
			using namespace std::filesystem;
			using namespace std::chrono;

			// Convert the file time to a std::time_t
			#if defined (_MSC_VER)
			auto filetime = last_mod_time.time_since_epoch().count();
			auto time = std::time_t(filetime - 116444736000000000LL) / 10000000LL;
			#else
			auto time = file_time_type::clock::to_time_t(last_mod_time);
			#endif

			// Convert a time to a local time
			auto LocalTime = [](time_t time, tm& tm)
			{
				#ifdef _MSC_VER
				return localtime_s(&tm, &time) == 0;
				#else
				tm = *localtime(&time);
				return true;
				#endif
			};

			// Convert the time_t to a 'tm' in local time
			struct tm tm = {};
			MSDosTimestamp dos_timestamp = {};
			if (LocalTime(time, tm))
			{
				dos_timestamp.time = static_cast<uint16_t>((tm.tm_hour << 11) + (tm.tm_min << 5) + (tm.tm_sec >> 1));
				dos_timestamp.date = static_cast<uint16_t>(((tm.tm_year + 1900 - 1980) << 9) + ((tm.tm_mon + 1) << 5) + tm.tm_mday);
			}
			return dos_timestamp;
		}

		// Accumulate the crc of given data.
		static uint32_t Crc(void const* ptr, int64_t buf_len, uint32_t crc = InitialCrc)
		{
			// Karl Malbrain's compact CRC-32. See "A compact CCITT crc16 and crc32 C implementation that balances
			// processor cache usage against speed": http://www.geocities.com/malbrain/
			static const uint32_t s_crc32[16] =
			{
				0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
				0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
			};

			if (ptr == nullptr) return crc;
			auto p = static_cast<uint8_t const*>(ptr);

			crc = ~crc;
			while (buf_len--)
			{
				auto b = *p++;
				crc = (crc >> 4) ^ s_crc32[(crc & 0xF) ^ (b & 0xF)];
				crc = (crc >> 4) ^ s_crc32[(crc & 0xF) ^ (b >> 4)];
			}
			return ~crc;
		}
		static uint32_t Crc(std::ifstream& ifile, uint32_t crc = InitialCrc)
		{
			if (!ifile.good())
				throw std::runtime_error("Calculating CRC checksum for input file stream failed. File stream in error state");

			// Preserve the current file pointer position
			auto fpos = ifile.tellg();
			if (!ifile.seekg(0))
				throw std::runtime_error("Calculating CRC checksum for input file stream failed. Seek to start failed");

			// Read from the file in blocks
			std::array<char, 4096> buf = {};
			for (; ifile.good();)
			{
				auto n = ifile.read(buf.data(), buf.size()).gcount();
				crc = Crc(buf.data(), n, crc);
			}

			// Restore the file pointer position
			ifile.clear();
			ifile.seekg(fpos);
			return crc;
		}

		// Return 'value' with 'length' bits reversed
		template <typename TInt>
		static TInt ReverseBits(TInt value, int length)
		{
			assert(length <= sizeof(TInt) * 8);

			TInt reversed = 0;
			for (; length-- != 0;)
			{
				reversed = (reversed << 1) | (value & 1);
				value >>= 1;
			}
			return reversed;
		}

		// CompileTime accumulative hash 
		static uint64_t const FNV_prime64 = 1099511628211ULL;
		static uint64_t const FNV_offset_basis64 = 14695981039346656037ULL;
		static constexpr uint64_t Hash64CT(uint64_t ch, uint64_t h = FNV_offset_basis64)
		{
			return Mul64(h ^ ch, FNV_prime64);
		}
		static constexpr uint64_t Mul64(uint64_t a, uint64_t b)
		{
			// 64 bit multiply without a warning...
			auto ffffffff = uint32_t(-1);
			auto ab = Lo32(a) * Lo32(b);
			auto aB = Lo32(a) * Hi32(b);
			auto Ab = Hi32(a) * Lo32(b);
			auto hi = ((((Hi32(ab) + aB) & ffffffff) + Ab) & ffffffff) << 32;
			auto lo = ab & ffffffff;
			return hi + lo;
		}
		static constexpr uint64_t Hi32(uint64_t x) { return (x >> 32) & uint32_t(-1); }
		static constexpr uint64_t Lo32(uint64_t x) { return (x      ) & uint32_t(-1); }
		static_assert(Mul64(0x1234567887654321, 0x1234567887654321) == 0x290D0FCAD7A44A41, "Compile time multiply failed");

		// Return the length of available data for a stream
		template <typename Elem>
		static std::streamsize length(std::basic_istream<Elem> const& stream)
		{
			auto pos = stream.tellg();
			auto len = stream.seekg(0, std::ios::end).tellg() - pos;
			stream.seekg(pos);
			return len;
		}

		// Append bytes to a vector of bytes
		static void append(vector_t<uint8_t>& vec, void const* beg, void const* end)
		{
			vec.insert(vec.end(), static_cast<uint8_t const*>(beg), static_cast<uint8_t const*>(end));
		}
		static void append(vector_t<uint8_t>& vec, span_t<uint8_t> bytes)
		{
			vec.insert(vec.end(), bytes.begin(), bytes.end());
		}
		static void append(vector_t<uint8_t>& vec, std::string_view str)
		{
			vec.insert(vec.end(), str.data(), str.data() + str.size());
		}

		// Helper for detecting data lost when casting
		template <typename T, typename U>
		static constexpr T s_cast(U x)
		{
			assert(static_cast<U>(static_cast<T>(x)) == x && "Cast loses data");
			return static_cast<T>(x);
		}

		// True if 'ofs' is an aligned offset in the output stream
		template <typename T>
		bool is_aligned(T ofs) const
		{
			if (m_entry_alignment == 0) return true;
			return (ofs & (m_entry_alignment - 1)) == 0;
		}

		// True if (value & flags) == flags;
		template <typename T>
		static constexpr bool has_flag(T value, T flags)
		{
			if constexpr (std::is_enum_v<T>)
			{
				using ut = typename std::underlying_type<T>::type;
				return (ut(value) & ut(flags)) == ut(flags);
			}
			else
				return (value & flags) == flags;
		}

		// Integer divide by 3 with round up
		static constexpr int Div3(int x) { return (x + 2) / 3; }

		// Less operator for name_hash_index_pair_t (used when searching 'm_cdir_lookup')
		friend constexpr bool operator < (name_hash_index_pair_t const& lhs, name_hash_index_pair_t const& rhs)
		{
			return lhs.name_hash < rhs.name_hash;
		}
		friend constexpr bool operator < (name_hash_index_pair_t const& lhs, uint64_t rhs)
		{
			return lhs.name_hash < rhs;
		}
		friend constexpr bool operator < (uint64_t lhs, name_hash_index_pair_t const& rhs)
		{
			return lhs < rhs.name_hash;
		}

		// Static IO functions
		struct IO
		{
			static void NoRead(ZipArchiveA const&, int64_t, void*, int64_t)
			{
				throw std::runtime_error("Archive is write-only");
			}
			static void ReadIMem(ZipArchiveA const& me, int64_t ofs, void* buf, int64_t n)
			{
				using namespace std::literals;

				if (ofs < 0 || ofs + n > static_cast<int64_t>(me.m_imem.size()))
					throw std::runtime_error("Out of bounds read at offset "s + std::to_string(ofs) + " from in-memory zip archive");

				std::memcpy(buf, me.m_imem.data() + ofs, static_cast<size_t>(n));
			};
			static void ReadOMem(ZipArchiveA const& me, int64_t ofs, void* buf, int64_t n)
			{
				using namespace std::literals;

				if (ofs < 0 || ofs + n > static_cast<int64_t>(me.m_omem.size()))
					throw std::runtime_error("Out of bounds read at offset "s + std::to_string(ofs) + " from internal in-memory zip archive");

				std::memcpy(buf, me.m_omem.data() + ofs, static_cast<size_t>(n));
			}
			static void ReadIStream(ZipArchiveA const& me, int64_t ofs, void* buf, int64_t n)
			{
				using namespace std::literals;

				if (!me.m_istream.good())
					throw std::runtime_error("File read at offset "s + std::to_string(ofs) + " failed. Stream in error state.");
				if (!me.m_istream.seekg(ofs).good())
					throw std::runtime_error("File read at offset "s + std::to_string(ofs) + " failed. Seek failed.");
				if (int64_t c = me.m_istream.read(static_cast<char*>(buf), n).gcount(); c != n)
					throw std::runtime_error("File read at offset "s + std::to_string(ofs) + " failed. Stream truncated.");
			}

			static void NoWrite(ZipArchiveA&, int64_t, void const*, int64_t)
			{
				throw std::runtime_error("Archive is read-only");
			}
			static void WriteOMem(ZipArchiveA& me, int64_t ofs, void const* buf, int64_t n)
			{
				me.m_omem.resize(std::max<size_t>(size_t(ofs + n), me.m_omem.size()));
				std::memcpy(me.m_omem.data() + ofs, buf, static_cast<size_t>(n));
			}
			static void WriteOStream(ZipArchiveA& me, int64_t ofs, void const* buf, int64_t n)
			{
				using namespace std::literals;

				if (!me.m_ostream.good())
					throw std::runtime_error("File write at offset "s + std::to_string(ofs) + " failed. Stream in error state.");
				if (!me.m_ostream.seekp(ofs).good())
					throw std::runtime_error("File write at offset "s + std::to_string(ofs) + " failed. Seek failed.");
				if (!me.m_ostream.write(static_cast<char const*>(buf), n).good())
					throw std::runtime_error("File write at offset "s + std::to_string(ofs) + " failed.");
			}

			static ifstream_t<char> OpenForReading(std::filesystem::path const& filepath)
			{
				using namespace std::literals;

				ifstream_t<char> ifile(filepath, std::ios::binary, _SH_DENYNO);
				if (!ifile.good())
					throw std::runtime_error("File '"s + filepath.string() + "' could not be opened for reading.");

				std::noskipws(ifile);
				return std::move(ifile);
			}
			static ofstream_t<char> OpenForWriting(std::filesystem::path const& filepath)
			{
				using namespace std::literals;

				ofstream_t<char> ofile(filepath, std::ios::binary|std::ios::ate, _SH_DENYWR);
				if (!ofile.good())
					throw std::runtime_error("File '"s + filepath.string() + "' could not be opened for writing.");

				std::noskipws(ofile);
				return std::move(ofile);
			}
		};

	private:

		struct Deflate
		{
			// Notes:
			//  - Implements the DEFLATE compression algorthim
			//  - Compression format:
			//       https://en.wikipedia.org/wiki/DEFLATE
			//       https://www.w3.org/Graphics/PNG/RFC-1951
			//       https://zlib.net/feldspar.html <- explanation of DEFLATE
			//
			// Algorithm:
			//  The compressor terminates a block when it determines that starting a new block with fresh trees
			//  would be useful, or when the block size fills up the compressor's block buffer.
			//
			//  The compressor uses a chained hash table to find duplicated strings, using a hash function that
			//  operates on 3-byte sequences. At any given point during compression, let XYZ be the next 3 input
			//  bytes to be examined (not necessarily all different, of course). First, the compressor examines
			//  the hash chain for XYZ. If the chain is empty, the compressor simply writes out X as a literal
			//  byte and advances one byte in the input. If the hash chain is not empty, indicating that the
			//  sequence XYZ (or, if we are unlucky, some other 3 bytes with the same hash function value) has
			//  occurred recently, the compressor compares all strings on the XYZ hash chain with the actual
			//  input data sequence starting at the current point, and selects the longest match.
			//
			//  The compressor searches the hash chains starting with the most recent strings, to favor small
			//  distances and thus take advantage of the Huffman encoding. The hash chains are singly linked.
			//  There are no deletions from the hash chains; the algorithm simply discards matches that are too
			//  old. To avoid a worst-case situation, very long hash chains are arbitrarily truncated at a certain
			//  length, determined by a run-time parameter.
			//
			//  To improve overall compression, the compressor optionally defers the selection of matches ("lazy matching"):
			//    after a match of length N has been found, the compressor searches for a longer match starting at
			//    the next input byte. If it finds a longer match, it truncates the previous match to a length of one
			//    (thus producing a single literal byte) and then emits the longer match. Otherwise, it emits the
			//    original match, and, as described above, advances N bytes before continuing.
			//
			//  Run-time parameters also control this "lazy match" procedure. If compression ratio is most important,
			//  the compressor attempts a complete second search regardless of the length of the first match. In the
			//  normal case, if the current match is "long enough", the compressor reduces the search for a longer
			//  match, thus speeding up the process. If speed is most important, the compressor inserts new strings
			//  in the hash table only when no match was found, or when the match is not "too long". This degrades
			//  the compression ratio but saves time since there are both fewer insertions and fewer searches.

			// Bit shift register. Could also use uint32_t if on 32-bit
			using bit_buf_t = uint64_t;

			// The maximum size of a block
			static int64_t const MaxBlockSize = 64 * 1024;

			// Huffman table sizes
			static int64_t const LitTableSize = 288;
			static int64_t const DstTableSize = 32;
			static int64_t const DynTableSize = 19;
			static int64_t const MaxTableSize = std::max({ LitTableSize, DstTableSize, DynTableSize });

			// The compressor defaults to 128 dictionary probes per dictionary search. 0=Huffman only, 1=Huffman+LZ (fastest/crap compression), 4095=Huffman+LZ (slowest/best compression).
			static uint32_t const DefaultProbes = 0x080;
			static uint32_t const MaxProbesMask = 0xFFF;
			static int const MaxSupportedHuffCodeSize = 32;
			static int const StaticBlockSizeThreshold = 5;
			static int const DynamicBlockSizeThreshold = 48;
			static int const MinMatchLength = 3;
			static int const MaxMatchLength = 258;
			static int const MaxDeferCount = 8;

			// Signals the end of a block
			static uint16_t const BlockTerminator = 0x0100;

			// Flags used in 'Decompress()'
			enum class EDecompressFlags
			{
				None = 0,

				// If set, the input has a valid zlib header and ends with an Adler32 checksum (i.e. a zlib stream). Otherwise, the input is a raw deflate stream.
				ExpectZlibHeader = 1 << 0,
			};

			// Flags used by 'Compress()'
			enum class ECompressFlags
			{
				None = 0,
				
				// If set, the compressor outputs a zlib header before the deflate data, and the Adler-32 of the source data at the end. Otherwise, you'll get raw deflate data.
				WriteZLibHeader = 1 << 0,

				// If set, the first best match is used. Matching is not deferred
				GreedyMatching = 1 << 1,

				// Only look for RLE matches (i.e. matches with a distance of 1)
				RLEMatches = 1 << 2,

				// Disable usage of optimized Huffman tables.
				ForceAllStaticBlocks = 1 << 3,

				// Only use raw (uncompressed) deflate blocks.
				ForceAllRawBlocks = 1 << 4,
			};

			// A decoded zlib header
			struct ZLibHeader
			{
				// The ZLib header is two bytes, interpretted as:
				//  CMF
				//   bits [0..3] = CM - Compression method
				//   bits [4..7] = CINFO - Compression info
				//  FLG
				//   bits [0..4] = FCHECK - Check bits for CMF and FLG
				//   bits [5..5] = FDICT - Preset dictionary
				//   bits [6..7] = FLEVEL - Compression level
				// FCHECK is used to ensure ((CMF<<8)|FLG) is a multiple of 31

				// See: https://tools.ietf.org/html/rfc1950
				uint8_t CMF;
				uint8_t FLG;

				ZLibHeader(uint8_t cmf, uint8_t flg)
					:CMF(cmf)
					,FLG(flg)
				{
					// Header checksum
					auto fcheck = CMF * 256 + FLG;
					if ((fcheck % 31) != 0)
						throw std::runtime_error("ZLIB header invalid. FCHECK failed.");
				}

				// Compression method
				EMethod Method() const
				{
					return EMethod(CMF & 0xF);
				}

				// Deflate compression window size
				uint32_t DeflateWindowSize() const
				{
					if (Method != EMethod::Deflate)
						throw std::runtime_error("ZLIB header LZ77 Window size is only valid when the compression method is DEFLATE");
					
					auto log_sz = ((CMF >> 4) & 0xF);
					if (log_sz > 7)
						throw std::runtime_error("ZLIB header invalid. ZLIB header CINFO field is greater than 7.");

					return 1 << (log_sz + 8);
				}

				// True if a preset dictionary immediately follows the ZLIB header
				bool PresetDictionary() const
				{
					// If set, a DICT dictionary identifier is present immediately after the FLG byte.
					// The dictionary is a sequence of bytes which are initially fed to the compressor
					// without producing any compressed output. DICT is the Adler-32 checksum of this
					// sequence of bytes (see the definition of ADLER32 below). The decompressor can
					// use this identifier to determine which dictionary has been used by the compressor.
					return (FLG & (1 << 5)) != 0;
				}

				// The compression level
				uint32_t CompressionLevel() const
				{
					// 0 = Compressor used fastest algorithm 
					// 1 = Compressor used fast algorithm 
					// 2 = Compressor used default algorithm 
					// 2 = Compressor used maximum/slowest algorithm
					return (FLG >> 6) & 0x3;
				}
			};

			// Functor for calculating the Adler32 checksum
			struct AdlerChecksum
			{
				inline static constexpr uint32_t AdlerMod = 65521;
				uint32_t a, b;

				AdlerChecksum()
					:a(1)
					,b(0)
				{}
				uint32_t checksum() const
				{
					return (b << 16) | a;
				}
				uint8_t operator()(uint8_t byte)
				{
					a = (a + byte) % AdlerMod;
					b = (b + a) % AdlerMod;
					return byte;
				}
			};

			// Shift register used to stream data in/out during Compress()/Decompress()
			bit_buf_t m_bit_buf; // bits in => MSB...LSB => bits out
			int m_num_bits; // The current number of bits in the shift register

			Deflate()
				:m_bit_buf()
				,m_num_bits()
			{}

			// Decompress a stream of bytes from 'stream' and write the decompressed stream to 'output'.
			// 'Src' should have uint8_t pointer-like symmantics.
			// 'stream' is the input stream of compressed data
			// 'length' is the length of data available via 'stream'
			// 'output' is called periodically to return the compressed data. Signature: void flush(span_t<uint8_t> out)
			// 'flags' control the decompression behaviour
			// 'adler_checksum' is used to return the checksum value. Only valid for ZLib data.
			//   Callers should use the AdlerChecksum helper with their output iterator to calculate
			//   the checksum then compare it to the 'adler_checksum' value.
			template <typename Src, typename FlushCB>
			void Decompress(Src stream, int64_t length, FlushCB output, EDecompressFlags const flags = EDecompressFlags::None, uint32_t* adler_checksum = nullptr)
			{
				using namespace std::literals;

				m_num_bits = 0;
				m_bit_buf = 0;

				SrcIter<Src> src(stream, length);
				Out<FlushCB> out(output);

				// Parse the ZLIB header
				if (has_flag(flags, EDecompressFlags::ExpectZlibHeader))
				{
					auto cmf = *src; ++src; // Compression method an flags
					auto flg = *src; ++src; // More flags
					auto zhdr = ZLibHeader(cmf, flg);
					if (zhdr.Method() != EMethod::Deflate)
						throw std::runtime_error("ZLIB header indicates a compression method other than 'DEFLATE'. Not supported.");
					if (zhdr.PresetDictionary())
						throw std::runtime_error("ZLIB header contains a preset dictionary. Not supported.");
				}

				// A Deflate stream consists of a series of blocks. Each block is preceded by a 3-bit header:
				// First bit: Last-block-in-stream marker:
				//  1: this is the last block in the stream.
				//  0: there are more blocks to process after this one.
				// Second and third bits: Encoding method used for this block type:
				//  00: a stored/raw/literal section, between 0 and 0xFFFF bytes in length.
				//  01: a static Huffman compressed block, using a pre-agreed Huffman tree.
				//  10: a compressed block complete with the Huffman table supplied.
				//  11: reserved, don't use.
				for (bool more = true; more;)
				{
					// Read the block header, and see if this is the last block
					auto hdr = GetBits<uint32_t>(src, 3);
					more = (hdr & 1) == 0;

					// Read the block type and prepare the huff tables based on type
					auto type = static_cast<EBlock>(hdr >> 1);
					switch (type)
					{
					// A stored/raw/literal section, between 0 and 65,535 bytes in length.
					case EBlock::Literal:
						{
							// Skip bits up to the next byte boundary
							void(GetBits<uint32_t>(src, m_num_bits & 7));

							// The length and two's complement of length of uncompressed data follows.
							auto a0 = static_cast<uint16_t>(GetByte(src));
							auto a1 = static_cast<uint16_t>(GetByte(src));
							auto len = a0 | (a1 << 8); 

							auto b0 = static_cast<uint16_t>(GetByte(src));
							auto b1 = static_cast<uint16_t>(GetByte(src));
							auto nlen = b0 | (b1 << 8);

							if (len != ~nlen)
								throw std::runtime_error("DEFLATE uncompressed block has an invalid length");

							// Copy bytes directly to the output stream
							for (; len-- != 0; ++out)
								*out = GetByte(src);

							break;
						}

					// A static Huffman compressed block, using pre-agreed symbol and distance tables
					case EBlock::Static:
						{
							// Initialise the literal/lengths table
							HuffLookupTable lit_table(LitTableSize);
							memset(&lit_table.m_bit_lengths[  0], 8, 144);
							memset(&lit_table.m_bit_lengths[144], 9, 112);
							memset(&lit_table.m_bit_lengths[256], 7,  24);
							memset(&lit_table.m_bit_lengths[280], 8,   8);
							lit_table.Populate(15);

							// Initialise the distance table
							HuffLookupTable dst_table(DstTableSize);
							memset(&dst_table.m_bit_lengths[0], 5, dst_table.m_alphabet_count);
							dst_table.Populate(15);

							// Decompress the block
							ReadBlock(src, lit_table, dst_table, out);
							break;
						}

					// A compressed block complete with the Huffman table supplied.
					case EBlock::Dynamic:
						{
							// See the Alphabet description in WriteBlock
							auto num_lit_codes = GetBits<uint8_t>(src, 5) + 257; // number of literal/length codes (- 257)
							auto num_dst_codes = GetBits<uint8_t>(src, 5) + 1;   // number of distance codes (- 1)
							auto num_bit_lengths = GetBits<int>(src, 4) + 4;     // number of bit length codes (- 4)

							// Read the bit lengths into 'dyn_table'.
							HuffLookupTable dyn_table(DynTableSize);
							for (int i = 0; i != num_bit_lengths; ++i) dyn_table.m_bit_lengths[s_swizzle[i]] = GetBits<uint8_t>(src, 3);
							dyn_table.Populate(7);

							// Decompress the code bit length values
							std::array<uint8_t, LitTableSize + DstTableSize + 137> bit_lengths = {};
							for (int i = 0, iend = num_lit_codes + num_dst_codes; i != iend;)
							{
								// The table of bit lengths is run length encoded
								auto sym = HuffDecode(src, dyn_table);
								if (sym < 16)
								{
									// 'sym' < 16 means it is a literal bit length value
									bit_lengths[i] = static_cast<uint8_t>(sym);
									++i;
								}
								else
								{
									// The first symbol cannot be a reference to an earlier location
									if (i == 0 && sym == 16)
										throw std::runtime_error("Dynamic Huffman table is corrupt.");

									// Read the length of the LZ encoded code size
									auto len = "\03\03\013"[sym - 16] + GetBits<uint32_t>(src, "\02\03\07"[sym - 16]); // length + num_extra
									memset(&bit_lengths[i], sym == 16 ? bit_lengths[s_cast<size_t>(i) - 1] : 0, len);
									if ((i += len) > iend)
										throw std::runtime_error("Dynamic Huffman table is corrupt.");
								}
							}

							// Generate the literal/length and distance tables from the bit length values
							HuffLookupTable lit_table(num_lit_codes);
							HuffLookupTable dst_table(num_dst_codes);
							memcpy(&lit_table.m_bit_lengths[0], &bit_lengths[            0], num_lit_codes);
							memcpy(&dst_table.m_bit_lengths[0], &bit_lengths[num_lit_codes], num_dst_codes);
							lit_table.Populate(15);
							dst_table.Populate(15);

							// Decompress the block
							ReadBlock(src, lit_table, dst_table, out);
							break;
						}

					// reserved, don't use.
					case EBlock::Reserved:
					default:
						{
							throw std::runtime_error("DEFLATE stream contains an invalid block header");
						}
					}
				}

				// ZLib streams contain the Alder32 CRC after the data.
				if (has_flag(flags, EDecompressFlags::ExpectZlibHeader))
				{
					// Skip bits up to the next byte boundary
					void(GetBits<uint32_t>(src, m_num_bits & 7));

					// Read the expected Alder32 value
					uint32_t tail_adler32 = 1;
					for (int i = 0; i != 4; ++i)
						tail_adler32 = (tail_adler32 << 8) | GetByte(src);

					// Return the checksum
					if (adler_checksum != nullptr)
						*adler_checksum = tail_adler32;
				}
			}

			// Compress a stream of bytes from 'stream' and write the compressed stream to 'output'.
			// 'Src' should have uint8_t pointer-like symmantics.
			// 'stream' is the input stream to be compressed.
			// 'length' is the number of bytes available from 'stream'
			// 'output' is called periodically to return the decompressed data. Signature: void flush(span_t<uint8_t> out)
			// 'flags' controls the compression output.
			// 'probe_count' controls the level of compression and must be a value in the range [0,4096) where 0=Huffman only, 1=Huffman+LZ (fastest/crap compression), 4095=Huffman+LZ (slowest/best compression).
			template <typename Src, typename FlushCB>
			void Compress(Src stream, int64_t length, FlushCB output, ECompressFlags const flags = ECompressFlags::None, int probe_count = DefaultProbes)
			{
				using namespace std::literals;

				m_bit_buf = 0;
				m_num_bits = 0;

				LZDictionary dict = {};
				LZBuffer lz_buffer = {};
				SymCount lit_counts(LitTableSize);
				SymCount dst_counts(DstTableSize);
				SrcIter<Src> src(stream, length), src_end;
				Out<FlushCB> out(output);
				RingBuffer<Range, MaxDeferCount> deferred;
				int defer_count = 0;

				// Write the ZLib header for DEFLATE
				if (has_flag(flags, ECompressFlags::WriteZLibHeader) && length != 0)
				{
					PutByte(out, 0x78);
					PutByte(out, 0x01);
				}

				// Handle raw block output as a special case
				if (has_flag(flags, ECompressFlags::ForceAllRawBlocks))
				{
					for (auto remaining = length; remaining != 0;)
					{
						// Header + Data <= MaxBlockSize
						auto const max = MaxBlockSize - 5;
						auto len = static_cast<uint16_t>(std::min(remaining, max));
						remaining -= len;

						// Write block header (1 byte)
						PutBits(out, len != max, 1);           // Write 1 for "last block"
						PutBits(out, int(EBlock::Literal), 2); // Write block type
						PutBits(out, 0, 5);                    // Align to next byte

						// Write length (4 bytes)
						PutBits(out, len, 16);
						PutBits(out, ~len, 16);

						// Write raw data (<= max bytes)
						for (; len-- != 0; ++src)
							PutByte(out, *src);
					}
					assert(src == src_end);
				}

				// Add a literal byte to 'lz_buffer' and count frequencies of the byte values
				auto RecordLiteral = [&](uint8_t lit)
				{
					lz_buffer.add(lit);
					lit_counts[lit]++;
				};

				// Add a (length,distance) pair to 'lz_buffer' and count frequencies of the length and distance values
				auto RecordMatch = [&](ptrdiff_t pos, Range match)
				{
					// 'match' is an absolute range. It needs to be converted to be relative to 'pos'.
					match.pos = pos - match.pos;
					assert(match.len >= MinMatchLength && "Match is too small");
					assert(match.pos >= 1 && match.pos <= LZDictionarySize && "Match is not within the dictionary");
					lz_buffer.add(match);

					// Count frequency of matches of this length
					auto s = s_len_sym[match.len - MinMatchLength];
					lit_counts[s]++;

					// Count frequency of matches at this distance
					// Note: since a relative position of 0 is invalid, pos - 1 is stored in 'lz_buffer'
					auto dist = match.pos - 1;
					auto d = dist <= 0x1FF
						? s_small_dist_sym[(dist >> 0) & 0x1FF]
						: s_large_dist_sym[(dist >> 8) & 0x07F];
					dst_counts[d]++;
				};

				// Record the best match of the buffered matches. (m0 is the index of the first queued match)
				auto FlushDeferQueue = [&](ptrdiff_t m0)
				{
					// Find the best of the deferred matches based on compression ratio
					auto best = 0;
					auto best_ratio = 0.0;
					for (int i = 0; i != defer_count; ++i)
					{
						auto& m = deferred[m0 + i];
						auto cost = i + (m.len >= MinMatchLength ? 3 : 1); // storage cost = 1 literal byte per skipped match +3 bytes if the match is valid or +1 literal byte if not valid
						auto value = i + m.len;                            // data represented count
						auto ratio = value / double(cost);                 // bytes represented / bytes stored
						if (ratio > best_ratio)
						{
							best_ratio = ratio;
							best = i;
						}
					}

					// Record the best match
					auto p = m0 + best; // The position that the best match is relative to
					for (auto i = m0; i != p; ++i) RecordLiteral(dict[i]); // Write literals for each skipped deferred match
					RecordMatch(p, deferred[p]);

					// Reset the defer queue
					defer_count = 0;

					// Return the next byte to be considered
					return p + deferred[p].len;
				};

				// Consume all bytes from 'stream'
				ptrdiff_t pos = 0;
				for (; src != src_end || pos != dict.m_size; )
				{
					// Push up to 'MaxMatchLength' bytes into the dictionary
					for (; src != src_end && dict.m_size - pos < MaxMatchLength; ++src)
						dict.Push(*src);

					// Find the longest match for the current position
					auto match =
						dict.m_size - pos < MinMatchLength ? Range(pos, 1) :
						has_flag(flags, ECompressFlags::RLEMatches) ? dict.RLEMatch(pos) :
						dict.Match(pos, probe_count);

					// Greedy matching, record the first best match as we find it
					if (has_flag(flags, ECompressFlags::GreedyMatching))
					{
						// If there is no suitable match...
						if (match.len < MinMatchLength) 
						{
							// Write a literal byte
							RecordLiteral(dict[pos]);
							++pos;
						}
						else // A match was found...
						{
							RecordMatch(pos, match);
							pos = match.end();
						}
					}
					else
					{
						// Deferred matches:
						// For any match, defer recording it until we know it's the best match for
						// the current position. A better match may exist that starts within 'match'.
						// E.g. consider the sequence: 1234567 0123 01234567
						//  matching group 3 will find group 2, but if we defer by one byte, a better
						//  match is found in group 1. This generalises on the number of bytes to defer
						//  by. Consider:  234567 0123 01234567. Deferring by 2 bytes finds the better
						//  match.
						// Algorithm:
						//   Let Q = [M0, M1, ...] be a queue of deferred matches.
						//   If the queue is empty, push the match onto the queue (this is M0)
						//   If the current position is spanned by M0, push the match on the queue
						//   If not, select the best match from the buffered matches (best = highest
						//   compression ratio = bytes-represented / bytes-stored). Advance 'pos' and 
						//   reset the queue.
						auto m0 = pos - defer_count;

						// No match and no deferred matches, write a literal
						if (defer_count == 0 && match.len < MinMatchLength)
						{
							RecordLiteral(dict[pos]);
							++pos;
						}
						else
						{
							// Push the match onto the queue
							deferred[pos] = match;
							++defer_count;
							++pos;

							// Flush the defer queue when: 
							if (defer_count == MaxDeferCount ||               // the queue is full
								match.len == MaxMatchLength ||                // a best possible match is found
								!deferred[m0].contains(pos) ||                // the first deferred match no longer spans 'pos'
								pos - deferred[m0].pos == LZDictionarySize || // the first match is about to roll out of the dictionary ring buffer
								src == src_end ||                             // the end of the source data has been reached
								false)
							{
								auto next_pos = FlushDeferQueue(m0);
								assert(next_pos >= pos && "'pos' should never go backwards");
								pos = next_pos;
							}
						}
					}

					// Write a block when 'lz_buffer' is full
					static size_t const MinSpaceRequired = 1 + (MaxDeferCount-1) + 3; // 1 byte for flags, MaxDeferCount-1 literal bytes, +3 bytes for (length,distance)
					if (LZBuffer::Size - lz_buffer.size() < MinSpaceRequired)
					{
						// Add the block terminator
						lit_counts[BlockTerminator] = 1;
						WriteBlock(out, lz_buffer, dict, pos, lit_counts, dst_counts, flags, false);

						// Reset the compression buffer and symbol counts
						lz_buffer.reset();
						lit_counts.reset();
						dst_counts.reset();
					}
				}

				// Write any remaining data
				lit_counts[BlockTerminator] = 1;
				WriteBlock(out, lz_buffer, dict, pos, lit_counts, dst_counts, flags, true);

				// Write the ZLib footer
				if (has_flag(flags, ECompressFlags::WriteZLibHeader) && length != 0)
				{
					// Calculate the checksum on the source input
					auto s = stream;
					AdlerChecksum adler = {};
					for (auto l = length; l-- != 0; ++s)
						adler(*s);

					// Align to the next byte
					if (m_num_bits != 0)
						PutBits(out, 0, 8 - m_num_bits);

					// Write the adler checksum (bit endian)
					auto checksum = adler.checksum();
					PutByte(out, (checksum >> 24) & 0xFF);
					PutByte(out, (checksum >> 16) & 0xFF);
					PutByte(out, (checksum >>  8) & 0xFF);
					PutByte(out, (checksum >>  0) & 0xFF);
				}
			}

		private:

			// Compressed block types
			enum class EBlock
			{
				Literal  = 0,
				Static   = 1,
				Dynamic  = 2,
				Reserved = 3,
			};

			// Represents the interval [pos, pos + len)
			struct Range
			{
				ptrdiff_t pos;
				ptrdiff_t len;

				Range() = default;
				Range(ptrdiff_t start, ptrdiff_t count)
					:pos(start)
					,len(count)
				{}

				// True if 'x' is within the interval [pos, pos + len)
				bool contains(ptrdiff_t x) const
				{
					return x >= begin() && x < end();
				}
				bool contains(Range x) const
				{
					return x.begin() >= begin() && x.end() <= end();
				}

				// Reduce the range to [pos + count, pos + len - count)
				void move_beg(ptrdiff_t count = 1)
				{
					pos += count;
					len -= count;
					return *this;
				}

				// Extend the range to [pos, pos + len + count)
				void move_end(ptrdiff_t count = 1)
				{
					len += count;
					return *this;
				}

				// The range begin/end
				ptrdiff_t begin() const
				{
					return pos;
				}
				ptrdiff_t end() const
				{
					return pos + len;
				}
			};

			// Fixed buffer for counting symbol frequencies
			struct SymCount
			{
				std::array<uint16_t, MaxTableSize> m_data;
				int m_alphabet_count;

				SymCount()
					:SymCount(0)
				{}
				SymCount(int alphabet_count)
					:m_data()
					,m_alphabet_count(alphabet_count)
				{
					assert(alphabet_count <= MaxTableSize);
				}
				int size() const
				{
					return m_alphabet_count;
				}
				void reset()
				{
					std::memset(m_data.data(), 0, static_cast<size_t>(sizeof(uint16_t) * m_alphabet_count));
				}
				uint16_t operator[](int idx) const
				{
					assert(idx < m_alphabet_count);
					return m_data[idx];
				}
				uint16_t& operator[](int idx)
				{
					assert(idx < m_alphabet_count);
					return m_data[idx];
				}
				operator span_t<uint16_t>() const
				{
					return span_t<uint16_t>(m_data.data(), static_cast<size_t>(m_alphabet_count));
				}
			};

			// A ring buffer of 'T' with fixed size 'Size' and optional "tail" of length 'Extend'
			template <typename T, size_t Size, size_t Extend = 0>
			struct RingBuffer
			{
				// This is basically a normal ring buffer of size 'Size'. There is an additional
				// 'Extend' bytes duplicated at the end so that sequences of 'Extend' bytes are contiguous.
				static_assert((Size & (Size - 1)) == 0, "RingBuffer size must be a power of 2");
				static_assert(Extend <= Size, "Size must be large enough to contain 'Extend' bytes");
				static_assert(std::is_trivially_copyable_v<T>, "Designed for POD types only");
				static int const Capacity = Size;
				static int const Mask = Size - 1;

				std::vector<T> m_buf;
				bool m_extend_required; // Dirty flag for when values are modified in the range [0, Extend)

				RingBuffer()
					:m_buf(Size + Extend)
					,m_extend_required()
				{}
				RingBuffer(uint8_t fill)
					:RingBuffer()
				{
					memset(&m_buf[0], fill, m_buf.size() * sizeof(T));
				}

				// The maximum size of the ring buffer
				constexpr int capacity() const
				{
					return Capacity;
				}

				// Ring buffer array access
				T operator [](ptrdiff_t idx) const
				{
					return m_buf[idx & Mask];
				}
				T& operator [](ptrdiff_t idx)
				{
					if constexpr (Extend != 0)
						m_extend_required |= idx >= 0 && idx < Extend;
					
					return m_buf[idx & Mask];
				}

				// Return a pointer into the buffer that is valid for at least 'Extend' values
				T const* ptr(ptrdiff_t ofs) const
				{
					if constexpr (Extend != 0)
						const_cast<RingBuffer*>(this)->extend();
					
					return &m_buf[ofs & Mask];
				}

			private:

				// Replicate the first 'Extend' value at the end of the buffer
				void extend()
				{
					if (!m_extend_required) return;
					memcpy(&m_buf[Size], &m_buf[0], Extend * sizeof(T));
					m_extend_required = false;
				}
			};

			// An iterator wrapper for 'src' pointers
			template <typename Src>
			struct SrcIter
			{
				using value_type = typename std::iterator_traits<Src>::value_type;

				Src m_ptr;     // Iterator to underlying sequence
				int64_t m_len; // Remaining count

				SrcIter()
					:m_ptr()
					,m_len()
				{}
				SrcIter(Src src, int64_t len)
					:m_ptr(src)
					,m_len(len)
				{}
				value_type operator *() const
				{
					return m_len != 0 ? *m_ptr : value_type{};
				}
				SrcIter& operator ++()
				{
					if (m_len != 0)
					{
						++m_ptr;
						--m_len;
					}
					return *this;
				}

				friend bool operator == (SrcIter lhs, SrcIter rhs)
				{
					return lhs.m_ptr == rhs.m_ptr || (lhs.m_len == 0 && rhs.m_len == 0);
				}
				friend bool operator != (SrcIter lhs, SrcIter rhs)
				{
					return !(lhs == rhs);
				}
				friend ptrdiff_t operator - (SrcIter lhs, SrcIter rhs)
				{
					// Remember, 'm_len' is length remaining so iterators with 'm_len == 0' are end iterators
					return lhs.m_len - rhs.m_len;
				}
			};

			// An output helper
			template <typename FlushCB>
			struct Out
			{
				// This class is sort-of pointer like, but not copyable.
				using ringbuf_t = RingBuffer<uint8_t, MaxBlockSize>;
				ringbuf_t m_buf; // Ring buffer of output bytes
				ptrdiff_t m_beg; // Number of bytes flushed
				ptrdiff_t m_end; // Where new bytes are added
				FlushCB m_flush; // Called to flush data to the caller

				explicit Out(FlushCB flush)
					:m_buf()
					,m_beg()
					,m_end()
					,m_flush(flush)
				{
					static_assert(ringbuf_t::Capacity >= LZDictionarySize, "Output buffer must have a history of 'LZDictionarySize' bytes");
				}
				~Out()
				{
					if (m_end - m_beg != 0)
						flush();
				}
				Out(Out const&) = delete;
				Out& operator =(Out const&) = delete;
				Out& operator ++()
				{
					++m_end;
					if (m_end - m_beg == m_buf.capacity())
						flush();

					return *this;
				}
				uint8_t& operator*()
				{
					return m_buf[m_end];
				}
				uint8_t operator[](ptrdiff_t idx) const
				{
					// Remember 'idx' is a relative index, relative to 'm_end'
					assert(abs(idx) < m_buf.capacity() && "Can only access bytes within the ring buffer");
					assert(m_end + idx >= 0 && "index out of bounds");
					return m_buf[m_end + idx];
				}
				void flush()
				{
					auto count = m_end - m_beg;
					auto toend = m_buf.capacity() - int(m_beg & ringbuf_t::Mask);
					if (count <= toend)
					{
						m_flush(span_t<uint8_t>(m_buf.ptr(m_beg), count));
					}
					else
					{
						m_flush(span_t<uint8_t>(m_buf.ptr(m_beg), toend));
						m_flush(span_t<uint8_t>(m_buf.ptr(m_beg + toend), count - toend));
					}
					m_beg = m_end;
				}
			};

			// Helper for generating Huffman codes up to 'max_bit_length' in length
			template <typename T>
			struct HuffCodeGen
			{
				// Notes:
				// - 'bit_length_counts' is the number of Huffman codes for each bit length.
				//    Knowing this is enough to regenerate the Huffman codes for each symbol in the alphabet.
				//    see https://www.w3.org/Graphics/PNG/RFC-1951#huffman
				//     or https://rosettacode.org/wiki/Huffman_coding

				// A register for generating the next huffman code of a given length
				std::array<T, 1 + sizeof(T) * 8> m_huffman_code;
				int m_max_bit_length;

				explicit HuffCodeGen(span_t<int const> bit_length_counts)
					:m_huffman_code()
					,m_max_bit_length(int(bit_length_counts.size()))
				{
					assert(bit_length_counts.size() < m_huffman_code.size());

					// 'total' is the number of different values that can be represented by the code lengths used so far
					// 'bit_length_counts[0]' is just a count of unused Huffman code lengths
					size_t total = 0;
					for (int i = 1; i != m_max_bit_length; ++i)
					{
						// Generate the first Huffman code for each bit length
						total = (total + bit_length_counts[i]) << 1;
						m_huffman_code[i + 1] = static_cast<T>(total);
					}
					if (total > (1ULL << m_max_bit_length))
						throw std::runtime_error("Invalid Huffman code bit length counts.");
				}

				// Generate Huffman codes sequentially for each 'bit_length'
				T operator()(int bit_length)
				{
					assert(bit_length <= m_max_bit_length && "Huffman code generator was not initialised for codes with this length");
					auto code = m_huffman_code[bit_length]++;
					assert((code & (~T() << bit_length)) == 0 && "Huffman code exceeds expected bit length");
					return ReverseBits(code, bit_length);
				}
			};

			// Ring buffers used to identify repeating sequences of bytes in the input stream
			struct LZDictionary
			{
				// Hash table constants
				static int const HashTableBits = 15; // Alternative for low memory environments: 12
				static int const LZHashShift = Div3(HashTableBits);
				static size_t const HashTableSize = 1 << HashTableBits;

				// A ring buffer of source bytes
				using ByteBuffer = RingBuffer<uint8_t, LZDictionarySize, MaxMatchLength>;
				ByteBuffer m_bytes;

				// Singularly linked lists of locations in 'm_bytes' that have the same hash value
				using NextBuffer = RingBuffer<uint16_t, LZDictionarySize>;
				NextBuffer m_next;

				// Mapping from the hash of a 3-byte sequence to its starting index position in 'm_bytes'
				using HashBuffer = RingBuffer<uint16_t, HashTableSize>;
				HashBuffer m_hash;

				// The number of bytes added to the dictionary (not wrapped to LZDictionarySize)
				ptrdiff_t m_size;

				LZDictionary()
					:m_bytes()
					,m_next()
					,m_hash()
					,m_size()
				{}

				// Return the range of bytes currently in the 'm_bytes' ring buffer
				Range Available() const
				{
					return Range(std::max<ptrdiff_t>(0, m_size - LZDictionarySize), std::min<ptrdiff_t>(m_size, LZDictionarySize));
				}

				// Push a source byte into the dictionary
				void Push(uint8_t b)
				{
					// Add the next byte to the dictionary
					m_bytes[m_size] = b;
					++m_size;

					// Calcalate the hash of the last 3 bytes
					// Wrap around is handled by the ring buffer.
					auto i = m_size - 3;
					auto hash =
						(m_bytes[i + 0] << (LZHashShift * 2)) ^
						(m_bytes[i + 1] << (LZHashShift * 1)) ^
						(m_bytes[i + 2] << (LZHashShift * 0));

					// Insert 'hash' at the head of the singularly linked list of dictionary positions with the same hash value.
					// Note that by inserting at the head, hashes are in order from most recently seen to most distant.
					//  e.g.
					//     hash = HashFn(bytes[i+0], bytes[i+1], bytes[i+2])
					//     m_hash[hash] = i -> the newest index position with hash value 'hash'
					//     m_next[i] -> index of the next most recent position that has the same hash value, say, "j".
					//         m_next[j] -> index of the next most recent position that has the same hash value, say, "k".
					//             m_next[k] -> 0 = end of list.
					//
					// Note that 'm_hash' is only needed for constructing the linked lists of index positions.
					// Matching only requires the 'm_next' buffer and a starting index position.
					m_next[i] = m_hash[hash];
					m_hash[hash] = static_cast<uint16_t>(i & ByteBuffer::Mask);
				}

				// Search the dictionary for another position that matches 'pos' that is longer than 'best_match'
				// 'pos' is the source location in the dictionary to match against.
				// 'probe_count' is the maximum number of searches to perform
				Range Match(ptrdiff_t pos, int probe_count) const
				{
					// Hashes are based on 3-bytes sequences, so at least 3 bytes must have been added before matches can be found.
					assert(pos + 3 <= m_size);

					// The hash value of the 3-byte sequence at 'm_bytes[pos]' is the value in 'm_hash[pos]'.
					// We don't actually need the hash, we just need to search the linked list of index locations whose head is at 'm_next[pos]'.
					// The dictionary only contains a maximum of 'LZDictionarySize' bytes, so if 'i' is further back than this, a match cannot be tested.
					Range best_match(pos, 1);
					for (auto i = m_next[pos]; probe_count-- != 0 && i != (pos & ByteBuffer::Mask); i = m_next[i])
					{
						auto ref = m_bytes.ptr(pos);
						auto cmp = m_bytes.ptr(i);

						// Find the length of the match
						int len = 0, max = std::min(MaxMatchLength, int(m_size - pos));
						for (; len != max && *cmp == *ref; ++len, ++cmp, ++ref) {}

						// If the match is longer than the current best match record it
						if (len > best_match.len)
						{
							// If a decent match is found, reduce the number of remaining probes to speed up searching
							if (len >= (MaxMatchLength + best_match.len) / 2)
								probe_count >>= 1;

							// Save the best match
							auto p = pos - ((pos - i) & ByteBuffer::Mask); // Absolute position
							best_match = Range(p, len);

							// Can't do better than this so stop searching
							if (len == MaxMatchLength)
								break;
						}
					}
					return best_match;
				}

				// Look for a range using run length encoding starting at 'pos'
				Range RLEMatch(ptrdiff_t pos) const
				{
					auto ref = m_bytes.ptr(pos);
					auto cmp = m_bytes.ptr(pos);

					int len = 0, max = std::min(MaxMatchLength, int(m_size - pos));
					for (; len != max && *cmp == *ref; ++len, ++cmp) {}

					return Range(pos, len);
				}

				// Data access
				uint8_t operator[](ptrdiff_t idx) const
				{
					return m_bytes[idx];
				}
			};

			// Records literal bytes, or (length,distance) pairs
			struct LZBuffer
			{
				// Notes:
				//  - Constructs an interlaced buffer of flags and literal bytes or (length,distance) pairs.
				//     e.g. [flags, bytes..., flags, bytes..., flags, etc...]
				//  - The LSB of a flags byte is the 'type' of data in the following byte
				//    0 - means a literal byte (length = 1)
				//    1 - means a (length, distance) pair (length = 3 bytes)

				static size_t const Size = 64 * 1024;
				static_assert(Size > LZDictionarySize);

				std::vector<uint8_t> m_buf;
				uint8_t* m_flags;   // Flags that record the type of data stored in the following bytes
				uint8_t* m_bytes;   // Where to insert the next literal byte or (length,distance) pair
				int m_num_flags;    // The number of flags used in the current byte pointed to by 'm_flags'
				size_t m_data_size; // The number of source bytes represented

				LZBuffer()
					:m_buf(Size)
					,m_flags(&m_buf[0])
					,m_bytes(&m_buf[1])
					,m_num_flags(0)
					,m_data_size(0)
				{}

				// The number of bytes currently in the buffer;
				size_t size() const
				{
					return (m_num_flags != 0 ? m_bytes : m_flags) - &m_buf[0];
				}

				// Reset the buffer
				void reset()
				{
					m_flags = &m_buf[0];
					m_bytes = &m_buf[1];
					m_num_flags = 0;
					m_data_size = 0;
				}

				// Add a literal byte to the buffer
				void add(uint8_t byte)
				{
					assert(m_bytes - &m_buf[0] + 1 <= Size && "LZBuffer overflow");
					push_flag<0>();
					*m_bytes++ = byte;
					m_data_size += 1;
				}

				// Add a *relative* match to the buffer
				void add(Range match)
				{
					static_assert(LZDictionarySize - 1 <= 0xFFFF);
					static_assert(MaxMatchLength - MinMatchLength <= 0xFF);
					assert(match.len >= MinMatchLength && match.len <= MaxMatchLength && "Match length is invalid");
					assert(match.pos >= 1 && match.pos <= LZDictionarySize && "Match distance is invalid");
					assert(m_bytes - &m_buf[0] + 3 <= Size && "LZBuffer overflow");

					push_flag<1>();
					*m_bytes++ = static_cast<uint8_t>(match.len - MinMatchLength);
					*m_bytes++ = static_cast<uint8_t>(((match.pos - 1) >> 0) & 0xFF);
					*m_bytes++ = static_cast<uint8_t>(((match.pos - 1) >> 8) & 0xFF);
					m_data_size = static_cast<size_t>(m_data_size + match.len);
				}

				// The number of input data bytes represented
				size_t data_size() const
				{
					return m_data_size;
				}

				// Begin/End to the range of contained data
				uint8_t const* begin() const
				{
					return &m_buf[0];
				}
				uint8_t const* end() const
				{
					return begin() + size();
				}

			private:

				// Append a flag
				template <int bit> void push_flag()
				{
					static_assert(bit <= 1);

					// If the flags byte is full, use the next byte in the buffer for flags
					if (m_num_flags == 8)
					{
						m_flags = m_bytes++;
						m_num_flags = 0;
					}
					*m_flags |= static_cast<uint8_t>(bit << m_num_flags);
					++m_num_flags;
				}
			};

			// Huffman lookup table
			struct HuffLookupTable
			{
				// Constants for 'm_lookup'
				static int const Bits = 10;           // Codes for bit lengths shorter than this are cached in 'm_lookup'
				static size_t const Size = 1 << Bits; // The maximum value to cache in 'm_lookup'
				static size_t const Mask = Size - 1;  // Mask for cached values

				int m_alphabet_count;                            // The number of alphabet values
				std::array<uint8_t, MaxTableSize> m_bit_lengths; // The Huffman code bit length for each alphabet value
				std::array<int16_t, MaxTableSize * 2> m_tree;    // Huffman encoding tree
				std::array<int16_t, Size> m_lookup;             // Map from Huffman code to alphabet value

				HuffLookupTable() = default;
				HuffLookupTable(int alphabet_count)
					:m_alphabet_count(alphabet_count)
					,m_bit_lengths()
					,m_tree()
					,m_lookup()
				{}

				// Populate the tree and lookup tables after 'm_bit_lengths' has been updated
				void Populate(int max_bit_length)
				{
					// The alphabet is all byte values in the range [0, m_alphabet_count)
					// With just the code sizes for each alphabet value, the Huffman tree can be
					// constructed, which implies the Huffman code used for each alphabet value.
					assert(max_bit_length <= MaxSupportedHuffCodeSize);

					// Find the counts of each code size
					std::array<int, 1+MaxSupportedHuffCodeSize> bit_length_counts = {};
					for (int i = 0; i != m_alphabet_count; ++i)
					{
						assert(m_bit_lengths[i] <= max_bit_length);
						bit_length_counts[m_bit_lengths[i]]++;
					}

					// Generate the lookup table and tree
					HuffCodeGen<uint16_t> gen({ bit_length_counts.data(), s_cast<size_t>(max_bit_length) + 1 });
					int16_t tree_cur, tree_next = -1;
					for (int sym_index = 0; sym_index != m_alphabet_count; ++sym_index)
					{
						// Get the bit length of the code
						auto bit_length = m_bit_lengths[sym_index];
						if (bit_length == 0)
							continue;

						// Get the next Huffman code for this bit length
						auto rev_code = gen(bit_length);

						// Populate the code cache
						if (bit_length <= HuffLookupTable::Bits)
						{
							// 'bit_length' is 4 bits, 'sym_index' is 9 bits.
							auto k = static_cast<int16_t>((bit_length << 9) | sym_index);
							for (; rev_code < HuffLookupTable::Size; rev_code += (1 << bit_length))
								m_lookup[rev_code] = k;

							continue;
						}

						// Grow the tree
						tree_cur = m_lookup[rev_code & HuffLookupTable::Mask];
						if (tree_cur == 0)
						{
							// Save the index to the next sub-tree
							m_lookup[rev_code & HuffLookupTable::Mask] = tree_next;
							tree_cur = tree_next;
							tree_next -= 2;
						}

						// Navigate the tree to find where to save 'sym_index'
						rev_code >>= HuffLookupTable::Bits - 1;
						for (int i = bit_length; i > HuffLookupTable::Bits + 1; --i)
						{
							rev_code >>= 1;
							tree_cur -= rev_code & 1;

							if (!m_tree[~tree_cur])
							{
								m_tree[~tree_cur] = tree_next;
								tree_cur = tree_next;
								tree_next -= 2;
							}
							else
							{
								tree_cur = m_tree[~tree_cur];
							}
						}

						tree_cur -= (rev_code >>= 1) & 1;
						m_tree[~tree_cur] = (int16_t)sym_index;
					}
				}
			};

			// Used to generate Huffman codes
			struct HuffCodeTable
			{
				int m_alphabet_count;                            // The number of alphabet values
				std::array<uint8_t, MaxTableSize> m_bit_lengths; // The Huffman code bit length for each alphabet value
				std::array<uint16_t, MaxTableSize> m_code;       // Map from alphabet value to Huffman code

				HuffCodeTable() = default;
				HuffCodeTable(int alphabet_count)
					:m_alphabet_count(alphabet_count)
					,m_bit_lengths()
					,m_code()
				{}

				// Populate 'm_bit_lengths' and 'm_code' using the given alphabet value frequencies
				void Populate(EBlock block_type, span_t<uint16_t> alphabet_value_counts, int max_bit_length)
				{
					// Determine the bit lengths for each alphabet value
					assert(max_bit_length <= MaxSupportedHuffCodeSize);
					std::array<int, 1+MaxSupportedHuffCodeSize> bit_length_counts = {};
					switch (block_type)
					{
					case EBlock::Static:
						{
							if (m_alphabet_count == LitTableSize)
							{
								memset(&m_bit_lengths[  0], 8, 144); bit_length_counts[8] += 144;
								memset(&m_bit_lengths[144], 9, 112); bit_length_counts[9] += 112;
								memset(&m_bit_lengths[256], 7,  24); bit_length_counts[7] +=  24;
								memset(&m_bit_lengths[280], 8,   8); bit_length_counts[8] +=   8;
							}
							else if (m_alphabet_count == DstTableSize)
							{
								memset(&m_bit_lengths[0], 5, 32); bit_length_counts[5] += 32;
							}
							else
							{
								throw std::runtime_error("Unknown static Huffman type");
							}
							break;
						}
					case EBlock::Dynamic:
						{
							// Determine the bit lengths using the Package-Merge algorithm: https://en.wikipedia.org/wiki/Package-merge_algorithm
							// This generates the Huffman codes while ensuring no code has a bit length greater than 'max_bit_length'
							// Paper: http://www.cs.ust.hk/mjg_lib/bibs/DPSu/DPSu.Files/p310-larmore.pdf
							// Better description: https://people.cs.nctu.edu.tw/~cjtsai/courses/imc/classnotes/imc14_03_Huffman_Codes.pdf
							using SymbolFreq = struct { uint16_t m_index; uint16_t m_count; };
							static bool const LittleEndian = true; // todo: use std::endian::native

							// Sort the alphabet values by their count then index, lowest frequency first
							assert(m_alphabet_count <= 0xFFFF);
							std::array<SymbolFreq, MaxTableSize> syms; int len = 0;
							for (uint16_t i = 0, iend = uint16_t(m_alphabet_count); i != iend; ++i)
							{
								if (alphabet_value_counts[i] == 0) continue;
								syms[len++] = SymbolFreq{ i, alphabet_value_counts[i] };
							}
							LittleEndian
								? std::sort(syms.data(), syms.data() + len, [](auto& l, auto& r) { return *reinterpret_cast<uint32_t const*>(&l) < *reinterpret_cast<uint32_t const*>(&r); })
								: std::sort(syms.data(), syms.data() + len, [](auto& l, auto& r) { return l.m_count != r.m_count ? l.m_count < r.m_count : l.m_index < r.m_index; });

							// Originally written by (November 1996):
							//  - Alistair Moffat, alistair@cs.mu.oz.au
							//  - Jyrki Katajainen, jyrki@diku.dk
							if      (len == 0) {}
							else if (len == 1) { syms[0].m_count = 1; }
							else
							{
								int root = 0;
								int next = 1;
								int leaf = 2;
								syms[0].m_count = static_cast<uint16_t>(syms[0].m_count + syms[1].m_count);
								for (; next < len - 1; ++next)
								{
									if (leaf >= len || syms[root].m_count < syms[leaf].m_count)
									{
										syms[next].m_count = syms[root].m_count;
										syms[root].m_count = static_cast<uint16_t>(next);
										root++;
									}
									else
									{
										syms[next].m_count = syms[leaf].m_count;
										leaf++;
									}

									if (leaf >= len || (root < next && syms[root].m_count < syms[leaf].m_count))
									{
										syms[next].m_count = static_cast<uint16_t>(syms[next].m_count + syms[root].m_count);
										syms[root].m_count = static_cast<uint16_t>(next);
										root++;
									}
									else
									{
										syms[next].m_count = static_cast<uint16_t>(syms[next].m_count + syms[leaf].m_count);
										leaf++;
									}
								}
								syms[len - 2].m_count = 0;
								for (next = len - 3; next >= 0; next--)
									syms[next].m_count = syms[syms[next].m_count].m_count + 1;

								int avbl = 1;
								int used = 0;
								int dpth = 0;
								root = len - 2;
								next = len - 1;
								while (avbl > 0)
								{
									while (root >= 0 && (int)syms[root].m_count == dpth)
									{
										used++;
										root--;
									}
									while (avbl > used)
									{
										syms[next--].m_count = static_cast<uint16_t>(dpth);
										avbl--;
									}

									avbl = 2 * used;
									dpth++;
									used = 0;
								}
							}

							// 'm_count' now contains the number of Huffman code bits for each symbol
							for (int i = 0; i != len; ++i)
								bit_length_counts[syms[i].m_count]++;

							// Limits canonical Huffman code table to codes with length <= 'max_bit_length'
							if (len > 1)
							{
								size_t i = {}, iend = {};

								// Count the number of codes with length > max_bit_length
								for (i = s_cast<size_t>(max_bit_length) + 1, iend = bit_length_counts.size(); i != iend; ++i)
									bit_length_counts[max_bit_length] += bit_length_counts[i];

								size_t total = 0;
								for (i = s_cast<size_t>(max_bit_length); i > 0; i--)
									total += static_cast<size_t>(bit_length_counts[i]) << (max_bit_length - i);

								for (; total != (1ULL << max_bit_length); --total)
								{
									bit_length_counts[max_bit_length]--;
									for (i = s_cast<size_t>(max_bit_length) - 1; i > 0; i--)
									{
										if (bit_length_counts[i] == 0) continue;
										bit_length_counts[i]--;
										bit_length_counts[i + 1] += 2;
										break;
									}
								}
							}

							// Record the bit lengths for each the code sizes
							for (int i = 0; i != max_bit_length; ++i)
								for (int j = bit_length_counts[s_cast<size_t>(i)+1]; j != 0; --j)
									m_bit_lengths[syms[--len].m_index] = static_cast<uint8_t>(i + 1);

							break;
						}
					case EBlock::Literal:
						{
							throw std::runtime_error("Block type does not have a Huffman table");
						}
					case EBlock::Reserved:
					default:
						{
							throw std::runtime_error("Invalid block type");
						}
					}

					// Generate the Huffman codes
					HuffCodeGen<uint16_t> gen({ bit_length_counts.data(), static_cast<size_t>(max_bit_length) });
					for (int i = 0; i != m_alphabet_count; ++i)
					{
						if (m_bit_lengths[i] == 0) continue;
						m_code[i] = gen(m_bit_lengths[i]);
					}
				}
			};

			// Read one byte from 'src'
			template <typename Src>
			uint8_t GetByte(SrcIter<Src>& src)
			{
				if (m_num_bits == 0)
				{
					auto b = *src; ++src;
					return b;
				}

				if (m_num_bits < 8)
				{
					// Append bits on the left
					m_bit_buf |= static_cast<bit_buf_t>(*src) << m_num_bits; ++src;
					m_num_bits += 8;
				}
				auto b = static_cast<uint8_t>(m_bit_buf & 0xFF);
				m_bit_buf >>= 8;
				m_num_bits -= 8;
				return b;
			}

			// Write one byte to 'out'
			template <typename Out>
			void PutByte(Out& out, uint8_t b)
			{
				if (m_num_bits == 0)
				{
					*out = b; ++out;
					return;
				}

				m_bit_buf |= static_cast<bit_buf_t>(b << m_num_bits);
				m_num_bits += 8;

				// Write out a whole byte
				*out = static_cast<uint8_t>(m_bit_buf & 0xFF); ++out;
				m_bit_buf >>= 8;
				m_num_bits -= 8;
			}

			// Read 'n' bits from the source stream into 'm_bit_buf'
			template <typename TInt, typename Src>
			TInt GetBits(SrcIter<Src>& src, int n)
			{
				assert(n <= sizeof(TInt) * 8 && "Return type not large enough for 'n' bits");
				for (; m_num_bits < n;)
				{
					// Append bits on the left
					m_bit_buf |= static_cast<bit_buf_t>(*src) << m_num_bits; ++src;
					m_num_bits += 8;
				}

				// Read and pop the lower 'n' bits
				auto b = static_cast<TInt>(m_bit_buf) & ((static_cast<TInt>(1) << n) - 1);
				m_bit_buf >>= n;
				m_num_bits -= n;
				return s_cast<TInt>(b);
			}

			// Write 'n' bits to the output stream, via 'm_bit_buf'
			template <typename Out>
			void PutBits(Out& out, bit_buf_t bits, int n)
			{
				assert((bits & (~bit_buf_t() << n)) == 0 && "'bits' has more than 'n' bits");
				assert(m_num_bits + n <= sizeof(bit_buf_t) * 8 && "Bit buffer overflow");

				// Add the bits on the left
				m_bit_buf |= bits << m_num_bits;
				m_num_bits += n;

				// Write out whole bytes
				for (; m_num_bits >= 8;)
				{
					*out = static_cast<uint8_t>(m_bit_buf & 0xFF); ++out;
					m_bit_buf >>= 8;
					m_num_bits -= 8;
				}
			}

			// Decodes and returns the next Huffman coded symbol.
			template <typename Src>
			int HuffDecode(SrcIter<Src>& src, HuffLookupTable const& table)
			{
				// Notes:
				//  - This function reads 2 bytes from 'src'.

				// It's more complex than you would initially expect because the zlib API expects the decompressor to never read
				// beyond the final byte of the deflate stream. (In other words, when this macro wants to read another byte from the input, it REALLY needs another byte in order to fully
				// decode the next Huffman code.) Handling this properly is particularly important on raw deflate (non-zlib) streams, which aren't followed by a byte aligned adler-32.
				// The slow path is only executed at the very end of the input buffer.

				// Ensure 'm_bit_buf' contains at least 15 bits
				for (; m_num_bits < 16; m_num_bits += 8)
				{
					m_bit_buf |= static_cast<bit_buf_t>(*src) << m_num_bits;
					++src;
				}

				// if symbol >= 0:
				//   bits [15..9] = code length
				//   bits [8] = word encoded flag
				//   bits [0..7] = literal byte (if bit 8 is 0), length (it bit 8 is 1)
				// else:
				//   Navigate the Huffman tree to locate the symbol

				// Read the Huff symbol
				auto symbol = table.m_lookup[m_bit_buf & HuffLookupTable::Mask];
				auto code_len = symbol >= 0 ? symbol >> 9 : HuffLookupTable::Bits;
				if (symbol >= 0)
					symbol &= 0x1FF;
				else
					for (; symbol < 0;)
						symbol = table.m_tree[~symbol + ((m_bit_buf >> code_len++) & 1)];

				// Consume 'code_len' bits
				m_bit_buf >>= code_len;
				m_num_bits -= code_len;
				return symbol;
			}

			// Uses the given tables to decompress data to the end of the block
			template <typename Src, typename Out>
			void ReadBlock(SrcIter<Src>& src, HuffLookupTable const& lit_table, HuffLookupTable const& dst_table, Out& out)
			{
				for (;;)
				{
					// Read an decode the next symbol
					auto symbol = HuffDecode(src, lit_table);

					// If the symbol is the end-of-block marker, done
					if (symbol == BlockTerminator)
					{
						break;
					}

					// If the symbol is not a length value, output the literal byte
					else if (!has_flag(symbol, 0x0100))
					{
						*out = static_cast<uint8_t>(symbol & 0xFF);
						++out;
					}

					// Otherwise the symbol is a length value, implying it's followed by a distance value
					else
					{
						// Read the length of the sequence
						auto idx = symbol - 257;
						static std::array<int, 31> const s_length_base = { 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258, 0, 0 };
						static std::array<int, 31> const s_length_extra = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0, 0, 0 };
						auto count = ptrdiff_t(s_length_base[idx] + GetBits<uint32_t>(src, s_length_extra[idx]));

						// Read the relative offset back to where to read from
						auto ofs = HuffDecode(src, dst_table);
						static std::array<int, 32> const s_dist_base = { 1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577, 0, 0 };
						static std::array<int, 32> const s_dist_extra = { 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13 };
						auto dist = ptrdiff_t(s_dist_base[ofs] + GetBits<uint32_t>(src, s_dist_extra[ofs]));

						// Repeat an earlier sequence from [prev, prev + count)
						for (; count-- != 0; ++out)
							*out = out[-dist];
					}
				}
			}

			// Write a block to the output
			template <typename Out>
			void WriteBlock(Out& out, LZBuffer const& lz_buffer, LZDictionary const& dict, ptrdiff_t pos, SymCount const& lit_counts, SymCount const& dst_counts, ECompressFlags flags, bool last)
			{
				// Write the "last block" flag
				PutBits(out, last, 1);

				// Decide what block type to output
				auto block_type =
					has_flag(flags, ECompressFlags::ForceAllStaticBlocks) ? EBlock::Static :
					lz_buffer.data_size() < StaticBlockSizeThreshold ? EBlock::Literal :
					lz_buffer.data_size() < DynamicBlockSizeThreshold ? EBlock::Static :
					EBlock::Dynamic;

				// Output a block
				switch (block_type)
				{
				case EBlock::Literal:
					{
						// Output the block header (2 bits)
						PutBits(out, int(EBlock::Literal), 2);

						// Align to the next byte
						if (m_num_bits != 0)
							PutBits(out, 0, 8 - m_num_bits);

						// Output the data length and its 2s complement
						assert(lz_buffer.data_size() <= 0xFFFF);
						auto len = static_cast<uint16_t>(lz_buffer.data_size());
						PutBits(out, len, 16);
						PutBits(out, ~len, 16);

						// Output the literal data
						auto range = Range(pos - len, len);
						assert(dict.Available().contains(range) && "Literal data not in dictionary");
						for (auto i = range.begin(); i != range.end(); ++i)
							PutByte(out, dict[i]);

						break;
					}
				case EBlock::Static:
					{
						// Initialise the literal/lengths table
						HuffCodeTable lit_table(LitTableSize);
						lit_table.Populate(EBlock::Static, lit_counts, 15);

						// Initialise the distance table
						HuffCodeTable dst_table(DstTableSize);
						dst_table.Populate(EBlock::Static, dst_counts, 15);

						// Output the block header (2 bits)
						PutBits(out, int(EBlock::Static), 2);

						// Output the compressed data
						WriteCompressedData(out, lz_buffer, lit_table, dst_table);

						// Write the block terminator
						PutBits(out, lit_table.m_code[BlockTerminator], lit_table.m_bit_lengths[BlockTerminator]);

						// Pad to the next byte boundary
						PutBits(out, 0, 8 - m_num_bits);
						break;
					}
				case EBlock::Dynamic:
					{
						HuffCodeTable lit_table(LitTableSize);
						HuffCodeTable dst_table(DstTableSize);
						lit_table.Populate(EBlock::Dynamic, lit_counts, 15);
						dst_table.Populate(EBlock::Dynamic, dst_counts, 15);

						// Count the length of 'bit_lengths', excluding trailing zeros
						auto TrimCount = [](auto& bit_lengths, int min, int max, uint8_t const* swizzle = nullptr)
						{
							int num = max;
							if (swizzle == nullptr)
								for (; num != min && bit_lengths[num - 1] == 0; --num) {}
							else
								for (; num != min && bit_lengths[swizzle[num - 1]] == 0; --num) {}
							return num;
						};
						auto num_lit_codes = TrimCount(lit_table.m_bit_lengths, 257, 286);
						auto num_dist_codes = TrimCount(dst_table.m_bit_lengths, 1, 30);

						// Collect the literals/lengths and distances into a single buffer
						std::array<uint8_t, LitTableSize + DstTableSize> bit_lengths = {};
						memcpy(&bit_lengths[0], &lit_table.m_bit_lengths[0], num_lit_codes);
						memcpy(&bit_lengths[num_lit_codes], &dst_table.m_bit_lengths[0], num_dist_codes);

						// Huffman encode the combined tables of bit lengths
						// Alphabet:
						//  [0, 15] = bit lengths 0 to 15,
						//  [16] = Copy the previous value 3 to 6 times, the next 2 bits indicate the length (+3). i.e. 0b00 = 3,... 0x11 = 6
						//  [17] = Repeat a value of 0 for 3 - 10 times, the next 3 bits indicate the length (+3). i.e. 0b000 = 3, .. 0b111 = 10
						//  [18] = Repeat a value of 0 for 11 - 138 times, the next 7 bits indicate the length (+11). i.e. 0b0000000 = 11, .. 0b1111111 = 138
						SymCount dyn_count(DynTableSize);
						std::array<uint8_t, LitTableSize + DstTableSize> lz_buf = {}; int lz_buf_count = 0;
						for (int i = 0, iend = num_lit_codes + num_dist_codes; i != iend; )
						{
							auto bit_length = bit_lengths[i];
							
							// Count the number of contiguous equal values
							int count = 1, max = bit_length == 0 ? 138 : 7;
							for (++i; i != iend && bit_lengths[i] == bit_length && count != max; ++i, ++count) {}

							// Zero is a special case
							if (bit_length == 0)
							{
								if (count == 1)
								{
									++dyn_count[bit_length];
									lz_buf[lz_buf_count++] = bit_length;
								}
								else if (count == 2)
								{
									++dyn_count[bit_length];
									++dyn_count[bit_length];
									lz_buf[lz_buf_count++] = bit_length;
									lz_buf[lz_buf_count++] = bit_length;
								}
								else if (count <= 10)
								{
									++dyn_count[17];
									lz_buf[lz_buf_count++] = 17;
									lz_buf[lz_buf_count++] = (uint8_t)(count - 3);
								}
								else
								{
									++dyn_count[18];
									lz_buf[lz_buf_count++] = 18;
									lz_buf[lz_buf_count++] = (uint8_t)(count - 11);
								}
							}
							// RLE 'bit_length'
							else
							{
								// Add the value to repeat
								++dyn_count[bit_length];
								lz_buf[lz_buf_count++] = bit_length;
								--count;

								// Write literals if <= 2
								if (count == 1)
								{
									++dyn_count[bit_length];
									lz_buf[lz_buf_count++] = bit_length;
								}
								else if (count == 2)
								{
									++dyn_count[bit_length];
									++dyn_count[bit_length];
									lz_buf[lz_buf_count++] = bit_length;
									lz_buf[lz_buf_count++] = bit_length;
								}
								else if (count >= 3)
								{
									// Add the number of repeats
									++dyn_count[16];
									lz_buf[lz_buf_count++] = 16;
									lz_buf[lz_buf_count++] = (uint8_t)(count - 3);
								}
							}
						}

						// Huffman encode 'lz_buf'
						HuffCodeTable dyn_table(DynTableSize);
						dyn_table.Populate(EBlock::Dynamic, dyn_count, 7);
						auto num_bit_lengths = TrimCount(dyn_table.m_bit_lengths, 4, 18, s_swizzle.data());

						// Write a dynamic block header
						PutBits(out, int(EBlock::Dynamic), 2);

						// Write the sizes of the dynamic Huffman tables
						PutBits(out, num_lit_codes - 257, 5);
						PutBits(out, num_dist_codes - 1, 5);
						PutBits(out, num_bit_lengths - 4, 4);

						// Write the Huffman encoded code sizes
						for (int i = 0; i != num_bit_lengths; ++i)
							PutBits(out, dyn_table.m_bit_lengths[s_swizzle[i]], 3);

						// Write the compressed table data
						for (int i = 0; i != lz_buf_count; )
						{
							auto sym = lz_buf[i++];
							PutBits(out, dyn_table.m_code[sym], dyn_table.m_bit_lengths[sym]);
							if (sym < 16) continue;
							PutBits(out, lz_buf[i++], "\02\03\07"[sym - 16]);
						}

						// Output the compressed data
						WriteCompressedData(out, lz_buffer, lit_table, dst_table);

						// Write the block terminator
						PutBits(out, lit_table.m_code[BlockTerminator], lit_table.m_bit_lengths[BlockTerminator]);

						// Pad to the next byte boundary
						PutBits(out, 0, 8 - m_num_bits);
						break;
					}
				case EBlock::Reserved:
				default:
					{
						throw std::runtime_error("Invalid output block type");
					}
				}
			}

			// Write the compressed data in 'lz_buffer' to the output, using the Huffman code tables for symbols and distances
			template <typename Out>
			void WriteCompressedData(Out& out, LZBuffer const& lz_buffer, HuffCodeTable const& lit_table, HuffCodeTable const& dst_table)
			{
				int flags = 1;
				for (auto ptr = lz_buffer.begin(); ptr != lz_buffer.end(); flags >>= 1)
				{
					// Every 8 loops, the next byte is the 'flags' byte
					if (flags == 1)
						flags = *ptr++ | 0x100;

					// If the LSB is 0, then the next byte is a literal
					if ((flags & 1) == 0)
					{
						// Write out the literal
						auto lit = *ptr++;
						assert(lit_table.m_bit_lengths[lit] != 0 && "No Huffman code assigned to this value");
						PutBits(out, lit_table.m_code[lit], lit_table.m_bit_lengths[lit]);
					}

					// Otherwise, this is a (length,distance) pair
					else
					{
						auto len = ptr[0];
						auto dst = ptr[1] | (ptr[2] << 8);
						ptr += 3;

						static std::array<uint32_t, 17> const s_bitmasks = { 0x0000, 0x0001, 0x0003, 0x0007, 0x000F, 0x001F, 0x003F, 0x007F, 0x00FF, 0x01FF, 0x03FF, 0x07FF, 0x0FFF, 0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF };

						// Write out the length value
						assert(lit_table.m_bit_lengths[s_len_sym[len]] != 0 && "No Huffman code assigned to this length value");
						PutBits(out, lit_table.m_code[s_len_sym[len]], lit_table.m_bit_lengths[s_len_sym[len]]);
						PutBits(out, len & s_bitmasks[s_len_extra[len]], s_len_extra[len]);

						// Write out the distance value
						auto sym = dst <= 0x1FF
							? s_small_dist_sym[dst >> 0]
							: s_large_dist_sym[dst >> 8];
						auto extra = dst <= 0x1FF
							? s_small_dist_extra[dst >> 0]
							: s_large_dist_extra[dst >> 8];
						assert(dst_table.m_bit_lengths[sym] != 0 && "No Huffman code assigned to this distance value");
						PutBits(out, dst_table.m_code[sym], dst_table.m_bit_lengths[sym]);
						PutBits(out, dst & s_bitmasks[extra], extra);
					}
				}
			}

			// Dynamic table swizzle
			inline static std::array<uint8_t, 19> const s_swizzle =  { 16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15 };
		};

	private:

		inline static std::array<uint16_t, 256> const s_len_sym =
		{
			257, 258, 259, 260, 261, 262, 263, 264, 265, 265, 266, 266, 267, 267, 268, 268, 269, 269, 269, 269, 270, 270, 270, 270, 271, 271, 271, 271, 272, 272, 272, 272,
			273, 273, 273, 273, 273, 273, 273, 273, 274, 274, 274, 274, 274, 274, 274, 274, 275, 275, 275, 275, 275, 275, 275, 275, 276, 276, 276, 276, 276, 276, 276, 276,
			277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278,
			279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280,
			281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281,
			282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282,
			283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283,
			284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 285,
		};
		inline static std::array<uint8_t, 256> const s_len_extra =
		{
			0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
			4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
			5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
			5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 0,
		};
		inline static std::array<uint8_t, 512> const s_small_dist_sym =
		{
			0, 1, 2, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11,
			11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13,
			13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
			14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
			14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
			15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
			16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
			16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
			16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
			17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
			17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
			17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
		};
		inline static std::array<uint8_t, 512> const s_small_dist_extra =
		{
			0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5,
			5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
			6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
			6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
			7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
			7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
			7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
			7, 7, 7, 7, 7, 7, 7, 7,
		};
		inline static std::array<uint8_t, 128> const s_large_dist_sym =
		{
			0, 0, 18, 19, 20, 20, 21, 21, 22, 22, 22, 22, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26,
			26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28,
			28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29,
		};
		inline static std::array<uint8_t, 128> const s_large_dist_extra =
		{
			0, 0, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
			12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
			13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
		};
	};
	using ZipArchive = ZipArchiveA<std::allocator<void>>;
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/common/memstream.h"

namespace pr::storage
{
	PRUnitTest(ZipArchiveTests)
	{
		auto path = (std::filesystem::path(__FILE__).parent_path() / ".." / ".." / ".." / "projects" / "tests" / "unittests" / "res").lexically_normal();
		if (!std::filesystem::exists(path))
			throw std::runtime_error("Unit test resources directory not found");

		auto FileToBytes = [](std::filesystem::path const& filepath)
		{
			// Open the file and read it into memory
			std::basic_ifstream<uint8_t> ifile(filepath, std::ios::binary);
			std::basic_stringstream<uint8_t> file_bytes; file_bytes << ifile.rdbuf();
			return std::move(file_bytes.str());
		};
		auto MatchToFile = [=](auto& bytes, std::filesystem::path const& filepath)
		{
			auto file_bytes = FileToBytes(filepath);
			auto matches = bytes == file_bytes;
			return matches;
		};
		auto WriteBytes = [](std::filesystem::path const& filepath, std::basic_string<uint8_t> const& bytes)
		{
			std::ofstream ofile(filepath, std::ios::binary);
			ofile.write(reinterpret_cast<char const*>(bytes.data()), bytes.size());
		};
		auto ReplaceAll = [](std::basic_string<uint8_t>& bytes, std::string_view from, std::string_view to)
		{
			auto from_bytes = std::basic_string<uint8_t>(from.begin(), from.end());
			auto to_bytes = std::basic_string<uint8_t>(to.begin(), to.end());
			if (from_bytes.size() != to_bytes.size())
				throw std::runtime_error("Replacement strings must be the same size");

			auto pos = bytes.begin();
			for (;;)
			{
				pos = std::search(pos, bytes.end(), from_bytes.begin(), from_bytes.end());
				if (pos == bytes.end())
					break;

				std::copy(to_bytes.begin(), to_bytes.end(), pos);
				pos += from_bytes.size();
			}
		};
		auto AsBytes = [](std::string_view text)
		{
			return std::basic_string<uint8_t>(text.begin(), text.end());
		};
		auto temp_root = std::filesystem::temp_directory_path() / "zip_archive_tests";
		std::filesystem::remove_all(temp_root);
		std::filesystem::create_directories(temp_root);

		{// Round trip time stamps
			//auto now = std::filesystem::file_time_type::clock::now();
			//auto dos = zip::ZipArchive::FSTimeToDosTime(now);
			//auto NOW = zip::ZipArchive::DosTimeToFSTime(dos);
			//PR_EXPECT(NOW - now < std::chrono::seconds(3)); // DOS time has 2s resolution
		}

		// Write a test zip file
		{
			// Create a zip
			zip::ZipArchive z;
			z.AddFile(path / "file1.txt");
			z.AddFile(path / "file2.txt");
			z.AddFile(path / "file3.txt");
			z.AddFile(path / "binary-00-0F.bin");
			z.Save(path / "zip_out.zip");
			z.Close();

			// Read back the zip
			zip::ZipArchive z2(path / "zip_out.zip");
			PR_EXPECT(z2.Count() == 4);
			{
				std::basic_string<uint8_t> bytes;
				pr::mem_ostream mem(bytes);
				z2.Extract("file1.txt", mem);
				PR_EXPECT(MatchToFile(bytes, path / "file1.txt"));
			}
			{
				std::basic_string<uint8_t> bytes;
				pr::mem_ostream mem(bytes);
				z2.Extract("file2.txt", mem);
				PR_EXPECT(MatchToFile(bytes, path / "file2.txt"));
			}
			{
				std::basic_string<uint8_t> bytes;
				pr::mem_ostream mem(bytes);
				z2.Extract("file3.txt", mem);
				PR_EXPECT(MatchToFile(bytes, path / "file3.txt"));
			}
			{
				std::basic_string<uint8_t> bytes;
				pr::mem_ostream mem(bytes);
				z2.Extract("binary-00-0F.bin", mem);
				PR_EXPECT(MatchToFile(bytes, path / "binary-00-0F.bin"));
			}

		}
		// Cleanup
		if (std::filesystem::exists(path / "zip_out.zip"))
			std::filesystem::remove(path / "zip_out.zip");

		// Read a zip from memory
		{
			auto file_data = FileToBytes(path / "binary-00-0F.zip");
			zip::ZipArchive z(file_data);
			PR_EXPECT(z.Count() == 1);
			PR_EXPECT(z.Name(0) == "binary-00-0F.bin");
			PR_EXPECT(z.IndexOf("binary-00-0F.bin") == 0);

			std::basic_string<uint8_t> bytes;
			pr::mem_ostream mem(bytes);
			z.Extract("binary-00-0F.bin", mem);
			PR_EXPECT(MatchToFile(bytes, path / "binary-00-0F.bin"));
		}

		// Read a zip from file
		{
			zip::ZipArchive z(path / "binary-00-0F.zip", zip::ZipArchive::EMode::ReadOnly, zip::ZipArchive::EZipFlags::FastNameLookup);
			PR_EXPECT(z.Count() == 1);
			PR_EXPECT(z.Name(0) == "binary-00-0F.bin");
			PR_EXPECT(z.IndexOf("binary-00-0F.bin") == 0);
			
			std::basic_string<uint8_t> bytes;
			pr::mem_ostream mem(bytes);
			z.Extract("binary-00-0F.bin", mem);
			PR_EXPECT(MatchToFile(bytes, path / "binary-00-0F.bin"));
		}

		// Extract a nested archive layout to make sure legitimate relative paths still work.
		{
			auto test_root = temp_root / "nested";
			auto archive_path = test_root / "nested.zip";
			auto extract_root = test_root / "extract";
			std::filesystem::remove_all(test_root);
			std::filesystem::create_directories(test_root);

			zip::ZipArchive z;
			z.AddBytes(zip::ZipArchive::span_t<uint8_t>{}, "nested/dir/");
			z.AddString("nested file", "nested/dir/file.txt");
			z.Save(archive_path);
			z.Close();

			zip::ZipArchive z2(archive_path);
			z2.ExtractAll(extract_root);

			PR_EXPECT(std::filesystem::exists(extract_root / "nested" / "dir"));
			PR_EXPECT(std::filesystem::is_directory(extract_root / "nested" / "dir"));
			PR_EXPECT(std::filesystem::exists(extract_root / "nested" / "dir" / "file.txt"));
			PR_EXPECT(FileToBytes(extract_root / "nested" / "dir" / "file.txt") == AsBytes("nested file"));
		}

		// Reject central-directory names that try to escape the extraction root.
		{
			auto test_root = temp_root / "traversal";
			auto archive_path = test_root / "traversal.zip";
			auto extract_root = test_root / "extract";
			std::filesystem::remove_all(test_root);
			std::filesystem::create_directories(test_root);

			zip::ZipArchive z;
			z.AddBytes(zip::ZipArchive::span_t<uint8_t>{}, "safe_dir__/");
			z.AddString("payload", "safe_file__");
			z.Save(archive_path);
			z.Close();

			auto bytes = FileToBytes(archive_path);
			ReplaceAll(bytes, "safe_dir__/", "../evildir/");
			ReplaceAll(bytes, "safe_file__", "../evil.txt");
			WriteBytes(archive_path, bytes);

			PR_EXPECT(!std::filesystem::exists(test_root / "evildir"));
			PR_EXPECT(!std::filesystem::exists(test_root / "evil.txt"));
			PR_THROWS(zip::ZipArchive z3(archive_path), std::runtime_error);
			PR_EXPECT(!std::filesystem::exists(test_root / "evildir"));
			PR_EXPECT(!std::filesystem::exists(test_root / "evil.txt"));
			PR_EXPECT(!std::filesystem::exists(extract_root / "evildir"));
			PR_EXPECT(!std::filesystem::exists(extract_root / "evil.txt"));
		}
		std::filesystem::remove_all(temp_root);
	}
}
#endif

// @license: http://www.opensource.org/licenses/mit-license.php
/*
   miniz.c v1.15 - public domain deflate/inflate, zlib-subset, ZIP reading/writing/appending, PNG writing
   See "unlicense" statement at the end of this file.
   Rich Geldreich <richgel99@gmail.com>, last updated Oct. 13, 2013
   Implements RFC 1950: http://www.ietf.org/rfc/rfc1950.txt and RFC 1951: http://www.ietf.org/rfc/rfc1951.txt

   Most API's defined in miniz.c are optional. For example, to disable the archive related functions just define
   MINIZ_NO_ARCHIVE_APIS, or to get rid of all stdio usage define MINIZ_NO_STDIO (see the list below for more macros).

   * Change History
	 10/13/13 v1.15 r4 - Interim bugfix release while I work on the next major release with Zip64 support (almost there!):
	   - Critical fix for the MZ_ZIP_FLAG_DO_NOT_SORT_CENTRAL_DIRECTORY bug (thanks kahmyong.moon@hp.com) which could cause locate files to not find files. This bug
		would only have occured in earlier versions if you explicitly used this flag, OR if you used mz_zip_extract_archive_file_to_heap() or mz_zip_add_mem_to_archive_file_in_place()
		(which used this flag). If you can't switch to v1.15 but want to fix this bug, just remove the uses of this flag from both helper funcs (and of course don't use the flag).
	   - Bugfix in mz_zip_reader_extract_to_mem_no_alloc() from kymoon when pUser_read_buf is not nullptr and compressed size is > uncompressed size
	   - Fixing mz_zip_reader_extract_*() funcs so they don't try to extract compressed data from directory entries, to account for weird zipfiles which contain zero-size compressed data on dir entries.
		 Hopefully this fix won't cause any issues on weird zip archives, because it assumes the low 16-bits of zip external attributes are DOS attributes (which I believe they always are in practice).
	   - Fixing ItemIsDirectory() so it doesn't check the internal attributes, just the filename and external attributes
	   - mz_zip_reader_init_file() - missing MZ_FCLOSE() call if the seek failed
	   - Added cmake support for Linux builds which builds all the examples, tested with clang v3.3 and gcc v4.6.
	   - Clang fix for tdefl_write_image_to_png_file_in_memory() from toffaletti
	   - Merged MZ_FORCEINLINE fix from hdeanclark
	   - Fix <time.h> include before config #ifdef, thanks emil.brink
	   - Added tdefl_write_image_to_png_file_in_memory_ex(): supports Y flipping (super useful for OpenGL apps), and explicit control over the compression level (so you can
		set it to 1 for real-time compression).
	   - Merged in some compiler fixes from paulharris's github repro.
	   - Retested this build under Windows (VS 2010, including static analysis), tcc  0.9.26, gcc v4.6 and clang v3.3.
	   - Added example6.c, which dumps an image of the mandelbrot set to a PNG file.
	   - Modified example2 to help test the MZ_ZIP_FLAG_DO_NOT_SORT_CENTRAL_DIRECTORY flag more.
	   - In r3: Bugfix to mz_zip_writer_add_file() found during merge: Fix possible src file fclose() leak if alignment bytes+local header file write faiiled
	   - In r4: Minor bugfix to mz_zip_writer_add_from_zip_reader(): Was pushing the wrong central dir header offset, appears harmless in this release, but it became a problem in the zip64 branch
	 5/20/12 v1.14 - MinGW32/64 GCC 4.6.1 compiler fixes: added MZ_FORCEINLINE, #include <time.h> (thanks fermtect).
	 5/19/12 v1.13 - From jason@cornsyrup.org and kelwert@mtu.edu - Fix mz_crc32() so it doesn't compute the wrong CRC-32's when mz_ulong is 64-bit.
	   - Temporarily/locally slammed in "typedef unsigned long mz_ulong" and re-ran a randomized regression test on ~500k files.
	   - Eliminated a bunch of warnings when compiling with GCC 32-bit/64.
	   - Ran all examples, miniz.c, and tinfl.c through MSVC 2008's /analyze (static analysis) option and fixed all warnings (except for the silly
		"Use of the comma-operator in a tested expression.." analysis warning, which I purposely use to work around a MSVC compiler warning).
	   - Created 32-bit and 64-bit Codeblocks projects/workspace. Built and tested Linux executables. The codeblocks workspace is compatible with Linux+Win32/x64.
	   - Added miniz_tester solution/project, which is a useful little app derived from LZHAM's tester app that I use as part of the regression test.
	   - Ran miniz.c and tinfl.c through another series of regression testing on ~500,000 files and archives.
	   - Modified example5.c so it purposely disables a bunch of high-level functionality (MINIZ_NO_STDIO, etc.). (Thanks to corysama for the MINIZ_NO_STDIO bug report.)
	   - Fix ftell() usage in examples so they exit with an error on files which are too large (a limitation of the examples, not miniz itself).
	 4/12/12 v1.12 - More comments, added low-level example5.c, fixed a couple minor level_and_flags issues in the archive API's.
	  level_and_flags can now be set to MZ_DEFAULT_COMPRESSION. Thanks to Bruce Dawson <bruced@valvesoftware.com> for the feedback/bug report.
	 5/28/11 v1.11 - Added statement from unlicense.org
	 5/27/11 v1.10 - Substantial compressor optimizations:
	  - Level 1 is now ~4x faster than before. The L1 compressor's throughput now varies between 70-110MB/sec. on a
	  - Core i7 (actual throughput varies depending on the type of data, and x64 vs. x86).
	  - Improved baseline L2-L9 compression perf. Also, greatly improved compression perf. issues on some file types.
	  - Refactored the compression code for better readability and maintainability.
	  - Added level 10 compression level (L10 has slightly better ratio than level 9, but could have a potentially large
	   drop in throughput on some files).
	 5/15/11 v1.09 - Initial stable release.

   * Low-level Deflate/Inflate implementation notes:

	 Compression: Use the "tdefl" API's. The compressor supports raw, static, and dynamic blocks, lazy or
	 greedy parsing, match length filtering, RLE-only, and Huffman-only streams. It performs and compresses
	 approximately as well as zlib.

	 Decompression: Use the "tinfl" API's. The entire decompressor is implemented as a single function
	 coroutine: see Decompress(). It supports decompression into a 32KB (or larger power of 2) wrapping buffer, or into a memory
	 block large enough to hold the entire file.

	 The low-level tdefl/tinfl API's do not make any use of dynamic memory allocation.

   * zlib-style API notes:

	 miniz.c implements a fairly large subset of zlib. There's enough functionality present for it to be a drop-in
	 zlib replacement in many apps:
		The z_stream struct, optional memory allocation callbacks
		deflateInit/deflateInit2/deflate/deflateReset/deflateEnd/deflateBound
		inflateInit/inflateInit2/inflate/inflateEnd
		compress, compress2, compressBound, uncompress
		CRC-32, Adler-32 - Using modern, minimal code size, CPU cache friendly routines.
		Supports raw deflate streams or standard zlib streams with adler-32 checking.

	 Limitations:
	  The callback API's are not implemented yet. No support for gzip headers or zlib static dictionaries.
	  I've tried to closely emulate zlib's various flavors of stream flushing and return status codes, but
	  there are no guarantees that miniz.c pulls this off perfectly.

   * PNG writing: See the tdefl_write_image_to_png_file_in_memory() function, originally written by
	 Alex Evans. Supports 1-4 bytes/pixel images.

   * ZIP archive API notes:

	 The ZIP archive API's where designed with simplicity and efficiency in mind, with just enough abstraction to
	 get the job done with minimal fuss. There are simple API's to retrieve file information, read files from
	 existing archives, create new archives, append new files to existing archives, or clone archive data from
	 one archive to another. It supports archives located in memory or the heap, on disk (using stdio.h),
	 or you can specify custom file read/write callbacks.

	 - Archive reading: Just call this function to read a single file from a disk archive:

	  void *mz_zip_extract_archive_file_to_heap(const char *pZip_filename, const char *pArchive_name,
		size_t *pSize, mz_uint zip_flags);

	 For more complex cases, use the "mz_zip_reader" functions. Upon opening an archive, the entire central
	 directory is located and read as-is into memory, and subsequent file access only occurs when reading individual files.

	 - Archives file scanning: The simple way is to use this function to scan a loaded archive for a specific file:

	 int IndexOf(mz_zip_archive *pZip, const char *pName, const char *pComment, mz_uint flags);

	 The locate operation can optionally check file comments too, which (as one example) can be used to identify
	 multiple versions of the same file in an archive. This function uses a simple linear search through the central
	 directory, so it's not very fast.

	 Alternately, you can iterate through all the files in an archive (using mz_zip_reader_get_num_files()) and
	 retrieve detailed info on each file by calling CDirEntry().

	 - Archive creation: Use the "mz_zip_writer" functions. The ZIP writer immediately writes compressed file data
	 to disk and builds an exact image of the central directory in memory. The central directory image is written
	 all at once at the end of the archive file when the archive is finalized.

	 The archive writer can optionally align each file's local header and file data to any power of 2 alignment,
	 which can be useful when the archive will be read from optical media. Also, the writer supports placing
	 arbitrary data blobs at the very beginning of ZIP archives. Archives written using either feature are still
	 readable by any ZIP tool.

	 - Archive appending: The simple way to add a single file to an archive is to call this function:

	  mz_bool mz_zip_add_mem_to_archive_file_in_place(const char *pZip_filename, const char *pArchive_name,
		const void *pBuf, size_t buf_size, const void *pComment, uint16_t comment_size, mz_uint level_and_flags);

	 The archive will be created if it doesn't already exist, otherwise it'll be appended to.
	 Note the appending is done in-place and is not an atomic operation, so if something goes wrong
	 during the operation it's possible the archive could be left without a central directory (although the local
	 file headers and file data will be fine, so the archive will be recoverable).

	 For more complex archive modification scenarios:
	 1. The safest way is to use a mz_zip_reader to read the existing archive, cloning only those bits you want to
	 preserve into a new archive using using the mz_zip_writer_add_from_zip_reader() function (which compiles the
	 compressed file data as-is). When you're done, delete the old archive and rename the newly written archive, and
	 you're done. This is safe but requires a bunch of temporary disk space or heap memory.

	 2. Or, you can convert an mz_zip_reader in-place to an mz_zip_writer using mz_zip_writer_init_from_reader(),
	 append new files as needed, then finalize the archive which will write an updated central directory to the
	 original archive. (This is basically what mz_zip_add_mem_to_archive_file_in_place() does.) There's a
	 possibility that the archive's central directory could be lost with this method if anything goes wrong, though.

	 - ZIP archive support limitations:
	 No zip64 or spanning support. Extraction functions can only handle unencrypted, stored or deflated files.
	 Requires streams capable of seeking.

   * This is a header file library, like stb_image.c. To get only a header file, either cut and paste the
	 below header, or create miniz.h, #define MINIZ_HEADER_FILE_ONLY, and then include miniz.c from it.

   * Important: For best perf. be sure to customize the below macros for your target platform:
	 #define MINIZ_USE_UNALIGNED_LOADS_AND_STORES 1
	 #define MINIZ_LITTLE_ENDIAN 1
	 #define MINIZ_HAS_64BIT_REGISTERS 1

   * On platforms using glibc, Be sure to "#define _LARGEFILE64_SOURCE 1" before including miniz.c to ensure miniz
	 uses the 64-bit variants: fopen64(), stat64(), etc. Otherwise you won't be able to process large files
	 (i.e. 32-bit stat() fails for me on files > 0x7FFFFFFF bytes).

 This is free and unencumbered software released into the public domain.

  Anyone is free to copy, modify, publish, use, compile, sell, or
  distribute this software, either in source code form or as a compiled
  binary, for any purpose, commercial or non-commercial, and by any
  means.

  In jurisdictions that recognize copyright laws, the author or authors
  of this software dedicate any and all copyright interest in the
  software to the public domain. We make this dedication for the benefit
  of the public at large and to the detriment of our heirs and
  successors. We intend this dedication to be an overt act of
  relinquishment in perpetuity of all present and future rights to this
  software under copyright law.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
  OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.

  For more information, please refer to <http://unlicense.org/>
*/
