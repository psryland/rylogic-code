//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "text_unicode.h"
#include "pr/view3d-ui/engine.h"

namespace pr::view3d::ui
{
	namespace
	{
		// One inclusive code-point range in a sorted, non-overlapping property table.
		struct CpRange
		{
			char32_t lo;
			char32_t hi;
		};

		// True if 'cp' falls inside one of the sorted ranges in 'table'.
		template <std::size_t N>
		bool InTable(char32_t cp, CpRange const (&table)[N])
		{
			auto lo = std::size_t(0);
			auto hi = N;
			while (lo != hi)
			{
				auto mid = lo + (hi - lo) / 2;
				if (cp < table[mid].lo)
					hi = mid;
				else if (cp > table[mid].hi)
					lo = mid + 1;
				else
					return true;
			}
			return false;
		}

		// Grapheme_Extend plus Emoji_Modifier, the variation selectors and the tag characters. The
		// table covers the combining marks that appear in ordinary editing (Latin diacritics,
		// Hebrew/Arabic/Indic/Thai/Lao/Tibetan/Myanmar marks, Hangul tone marks, halfwidth
		// voicing marks) and the emoji-critical ranges; scripts outside those blocks fall through
		// to 'Other' and are then treated as ordinary single-cluster characters.
		constexpr CpRange ExtendRanges[] = {
			{ 0x00300, 0x0036F }, { 0x00483, 0x00489 }, { 0x00591, 0x005BD }, { 0x005BF, 0x005BF },
			{ 0x005C1, 0x005C2 }, { 0x005C4, 0x005C5 }, { 0x005C7, 0x005C7 }, { 0x00610, 0x0061A },
			{ 0x0064B, 0x0065F }, { 0x00670, 0x00670 }, { 0x006D6, 0x006DC }, { 0x006DF, 0x006E4 },
			{ 0x006E7, 0x006E8 }, { 0x006EA, 0x006ED }, { 0x00711, 0x00711 }, { 0x00730, 0x0074A },
			{ 0x007A6, 0x007B0 }, { 0x007EB, 0x007F3 }, { 0x007FD, 0x007FD }, { 0x00816, 0x00819 },
			{ 0x0081B, 0x00823 }, { 0x00825, 0x00827 }, { 0x00829, 0x0082D }, { 0x00859, 0x0085B },
			{ 0x008D3, 0x008E1 }, { 0x008E3, 0x00902 }, { 0x0093A, 0x0093A }, { 0x0093C, 0x0093C },
			{ 0x00941, 0x00948 }, { 0x0094D, 0x0094D }, { 0x00951, 0x00957 }, { 0x00962, 0x00963 },
			{ 0x00981, 0x00981 }, { 0x009BC, 0x009BC }, { 0x009C1, 0x009C4 }, { 0x009CD, 0x009CD },
			{ 0x009E2, 0x009E3 }, { 0x009FE, 0x009FE }, { 0x00A01, 0x00A02 }, { 0x00A3C, 0x00A3C },
			{ 0x00A41, 0x00A42 }, { 0x00A47, 0x00A48 }, { 0x00A4B, 0x00A4D }, { 0x00A51, 0x00A51 },
			{ 0x00A70, 0x00A71 }, { 0x00A75, 0x00A75 }, { 0x00A81, 0x00A82 }, { 0x00ABC, 0x00ABC },
			{ 0x00AC1, 0x00AC5 }, { 0x00AC7, 0x00AC8 }, { 0x00ACD, 0x00ACD }, { 0x00AE2, 0x00AE3 },
			{ 0x00AFA, 0x00AFF }, { 0x00B01, 0x00B01 }, { 0x00B3C, 0x00B3C }, { 0x00B3F, 0x00B3F },
			{ 0x00B41, 0x00B44 }, { 0x00B4D, 0x00B56 }, { 0x00B62, 0x00B63 }, { 0x00B82, 0x00B82 },
			{ 0x00BC0, 0x00BC0 }, { 0x00BCD, 0x00BCD }, { 0x00C00, 0x00C00 }, { 0x00C04, 0x00C04 },
			{ 0x00C3E, 0x00C40 }, { 0x00C46, 0x00C56 }, { 0x00C62, 0x00C63 }, { 0x00C81, 0x00C81 },
			{ 0x00CBC, 0x00CBC }, { 0x00CBF, 0x00CBF }, { 0x00CC6, 0x00CC6 }, { 0x00CCC, 0x00CCD },
			{ 0x00CE2, 0x00CE3 }, { 0x00D00, 0x00D01 }, { 0x00D3B, 0x00D3C }, { 0x00D41, 0x00D44 },
			{ 0x00D4D, 0x00D4D }, { 0x00D62, 0x00D63 }, { 0x00D81, 0x00D81 }, { 0x00DCA, 0x00DCA },
			{ 0x00DD2, 0x00DD6 }, { 0x00E31, 0x00E31 }, { 0x00E34, 0x00E3A }, { 0x00E47, 0x00E4E },
			{ 0x00EB1, 0x00EB1 }, { 0x00EB4, 0x00EBC }, { 0x00EC8, 0x00ECD }, { 0x00F18, 0x00F19 },
			{ 0x00F35, 0x00F35 }, { 0x00F37, 0x00F37 }, { 0x00F39, 0x00F39 }, { 0x00F71, 0x00F7E },
			{ 0x00F80, 0x00F84 }, { 0x00F86, 0x00F87 }, { 0x00F8D, 0x00FBC }, { 0x00FC6, 0x00FC6 },
			{ 0x0102D, 0x01030 }, { 0x01032, 0x01037 }, { 0x01039, 0x0103A }, { 0x0103D, 0x0103E },
			{ 0x01058, 0x01059 }, { 0x0105E, 0x01060 }, { 0x01071, 0x01074 }, { 0x01082, 0x01082 },
			{ 0x01085, 0x01086 }, { 0x0108D, 0x0108D }, { 0x0109D, 0x0109D }, { 0x0135D, 0x0135F },
			{ 0x01712, 0x01714 }, { 0x01732, 0x01734 }, { 0x01752, 0x01753 }, { 0x01772, 0x01773 },
			{ 0x017B4, 0x017B5 }, { 0x017B7, 0x017BD }, { 0x017C6, 0x017C6 }, { 0x017C9, 0x017D3 },
			{ 0x017DD, 0x017DD }, { 0x0180B, 0x0180D }, { 0x0180F, 0x0180F }, { 0x01885, 0x01886 },
			{ 0x018A9, 0x018A9 }, { 0x01920, 0x01922 }, { 0x01927, 0x01928 }, { 0x01932, 0x01932 },
			{ 0x01939, 0x0193B }, { 0x01A17, 0x01A18 }, { 0x01A1B, 0x01A1B }, { 0x01A56, 0x01A56 },
			{ 0x01A58, 0x01A5E }, { 0x01A60, 0x01A60 }, { 0x01A62, 0x01A62 }, { 0x01A65, 0x01A6C },
			{ 0x01A73, 0x01A7C }, { 0x01A7F, 0x01A7F }, { 0x01AB0, 0x01ACE }, { 0x01B00, 0x01B03 },
			{ 0x01B34, 0x01B34 }, { 0x01B36, 0x01B3A }, { 0x01B3C, 0x01B3C }, { 0x01B42, 0x01B42 },
			{ 0x01B6B, 0x01B73 }, { 0x01B80, 0x01B81 }, { 0x01BA2, 0x01BA5 }, { 0x01BA8, 0x01BA9 },
			{ 0x01BAB, 0x01BAD }, { 0x01BE6, 0x01BE6 }, { 0x01BE8, 0x01BE9 }, { 0x01BED, 0x01BED },
			{ 0x01BEF, 0x01BF1 }, { 0x01C2C, 0x01C33 }, { 0x01C36, 0x01C37 }, { 0x01CD0, 0x01CD2 },
			{ 0x01CD4, 0x01CE0 }, { 0x01CE2, 0x01CE8 }, { 0x01CED, 0x01CED }, { 0x01CF4, 0x01CF4 },
			{ 0x01CF8, 0x01CF9 }, { 0x01DC0, 0x01DFF }, { 0x0200C, 0x0200C }, { 0x020D0, 0x020F0 },
			{ 0x02CEF, 0x02CF1 }, { 0x02D7F, 0x02D7F }, { 0x02DE0, 0x02DFF }, { 0x0302A, 0x0302F },
			{ 0x03099, 0x0309A }, { 0x0A66F, 0x0A672 }, { 0x0A674, 0x0A67D }, { 0x0A69E, 0x0A69F },
			{ 0x0A6F0, 0x0A6F1 }, { 0x0A802, 0x0A802 }, { 0x0A806, 0x0A806 }, { 0x0A80B, 0x0A80B },
			{ 0x0A825, 0x0A826 }, { 0x0A82C, 0x0A82C }, { 0x0A8C4, 0x0A8C5 }, { 0x0A8E0, 0x0A8F1 },
			{ 0x0A8FF, 0x0A8FF }, { 0x0A926, 0x0A92D }, { 0x0A947, 0x0A951 }, { 0x0A980, 0x0A982 },
			{ 0x0A9B3, 0x0A9B3 }, { 0x0A9B6, 0x0A9B9 }, { 0x0A9BC, 0x0A9BD }, { 0x0A9E5, 0x0A9E5 },
			{ 0x0AA29, 0x0AA2E }, { 0x0AA31, 0x0AA32 }, { 0x0AA35, 0x0AA36 }, { 0x0AA43, 0x0AA43 },
			{ 0x0AA4C, 0x0AA4C }, { 0x0AA7C, 0x0AA7C }, { 0x0AAB0, 0x0AAB0 }, { 0x0AAB2, 0x0AAB4 },
			{ 0x0AAB7, 0x0AAB8 }, { 0x0AABE, 0x0AABF }, { 0x0AAC1, 0x0AAC1 }, { 0x0AAEC, 0x0AAED },
			{ 0x0AAF6, 0x0AAF6 }, { 0x0ABE5, 0x0ABE5 }, { 0x0ABE8, 0x0ABE8 }, { 0x0ABED, 0x0ABED },
			{ 0x0FB1E, 0x0FB1E }, { 0x0FE00, 0x0FE0F }, { 0x0FE20, 0x0FE2F }, { 0x0FF9E, 0x0FF9F },
			{ 0x101FD, 0x101FD }, { 0x102E0, 0x102E0 }, { 0x10376, 0x1037A }, { 0x10A01, 0x10A0F },
			{ 0x10A38, 0x10A3F }, { 0x10AE5, 0x10AE6 }, { 0x11001, 0x11001 }, { 0x11038, 0x11046 },
			{ 0x1107F, 0x11081 }, { 0x110B3, 0x110B6 }, { 0x110B9, 0x110BA }, { 0x11100, 0x11102 },
			{ 0x11127, 0x1112B }, { 0x1112D, 0x11134 }, { 0x11173, 0x11173 }, { 0x11180, 0x11181 },
			{ 0x111B6, 0x111BE }, { 0x111C9, 0x111CC }, { 0x1122F, 0x11231 }, { 0x11234, 0x11234 },
			{ 0x11236, 0x11237 }, { 0x112DF, 0x112DF }, { 0x112E3, 0x112EA }, { 0x11300, 0x11301 },
			{ 0x1133B, 0x1133C }, { 0x11340, 0x11340 }, { 0x11366, 0x11374 }, { 0x11438, 0x1143F },
			{ 0x11442, 0x11444 }, { 0x11446, 0x11446 }, { 0x114B3, 0x114B8 }, { 0x114BA, 0x114BA },
			{ 0x114BF, 0x114C0 }, { 0x114C2, 0x114C3 }, { 0x115B2, 0x115B5 }, { 0x115BC, 0x115BD },
			{ 0x115BF, 0x115C0 }, { 0x11633, 0x1163A }, { 0x1163D, 0x1163D }, { 0x1163F, 0x11640 },
			{ 0x116AB, 0x116AB }, { 0x116AD, 0x116AD }, { 0x116B0, 0x116B5 }, { 0x116B7, 0x116B7 },
			{ 0x1171D, 0x1171F }, { 0x11722, 0x11725 }, { 0x11727, 0x1172B }, { 0x1193B, 0x1193C },
			{ 0x1193E, 0x1193E }, { 0x11943, 0x11943 }, { 0x11A01, 0x11A0A }, { 0x11A33, 0x11A38 },
			{ 0x11A3B, 0x11A3E }, { 0x11A47, 0x11A47 }, { 0x11A51, 0x11A56 }, { 0x11A59, 0x11A5B },
			{ 0x11A8A, 0x11A96 }, { 0x11A98, 0x11A99 }, { 0x11C30, 0x11C3D }, { 0x11C92, 0x11CA7 },
			{ 0x11CAA, 0x11CB0 }, { 0x11CB2, 0x11CB3 }, { 0x11CB5, 0x11CB6 }, { 0x11D31, 0x11D45 },
			{ 0x11D47, 0x11D47 }, { 0x11D90, 0x11D91 }, { 0x11D95, 0x11D95 }, { 0x11D97, 0x11D97 },
			{ 0x11EF3, 0x11EF4 }, { 0x16AF0, 0x16AF4 }, { 0x16B30, 0x16B36 }, { 0x16F4F, 0x16F4F },
			{ 0x16F8F, 0x16F92 }, { 0x1BC9D, 0x1BC9E }, { 0x1D165, 0x1D165 }, { 0x1D167, 0x1D169 },
			{ 0x1D16E, 0x1D172 }, { 0x1D17B, 0x1D182 }, { 0x1D185, 0x1D18B }, { 0x1D1AA, 0x1D1AD },
			{ 0x1D242, 0x1D244 }, { 0x1DA00, 0x1DA36 }, { 0x1DA3B, 0x1DA6C }, { 0x1DA75, 0x1DA75 },
			{ 0x1DA84, 0x1DA84 }, { 0x1DA9B, 0x1DAAF }, { 0x1E000, 0x1E02A }, { 0x1E130, 0x1E136 },
			{ 0x1E2EC, 0x1E2EF }, { 0x1E8D0, 0x1E8D6 }, { 0x1E944, 0x1E94A }, { 0x1F3FB, 0x1F3FF },
			{ 0xE0020, 0xE007F }, { 0xE0100, 0xE01EF },
		};

		// Grapheme_Cluster_Break=Control: the format/line/paragraph separators that always break on
		// both sides. CR/LF/ZWJ are handled separately because they carry their own rules.
		constexpr CpRange ControlRanges[] = {
			{ 0x00000, 0x00009 }, { 0x0000B, 0x0000C }, { 0x0000E, 0x0001F }, { 0x0007F, 0x0009F },
			{ 0x000AD, 0x000AD }, { 0x0061C, 0x0061C }, { 0x0180E, 0x0180E }, { 0x0200B, 0x0200B },
			{ 0x0200E, 0x0200F }, { 0x02028, 0x0202E }, { 0x02060, 0x0206F }, { 0x0FEFF, 0x0FEFF },
			{ 0x0FFF0, 0x0FFFB }, { 0x13430, 0x1343F }, { 0x1BCA0, 0x1BCA3 }, { 0x1D173, 0x1D17A },
			{ 0xE0001, 0xE0001 },
		};

		// Grapheme_Cluster_Break=SpacingMark: visible combining marks that attach to the preceding
		// cluster. Covers the Indic and South-East Asian blocks used in ordinary editing.
		constexpr CpRange SpacingMarkRanges[] = {
			{ 0x00903, 0x00903 }, { 0x0093B, 0x0093B }, { 0x0093E, 0x00940 }, { 0x00949, 0x0094C },
			{ 0x0094E, 0x0094F }, { 0x00982, 0x00983 }, { 0x009BF, 0x009C0 }, { 0x009C7, 0x009C8 },
			{ 0x009CB, 0x009CC }, { 0x00A03, 0x00A03 }, { 0x00A3E, 0x00A40 }, { 0x00A83, 0x00A83 },
			{ 0x00ABE, 0x00AC0 }, { 0x00AC9, 0x00AC9 }, { 0x00ACB, 0x00ACC }, { 0x00B02, 0x00B03 },
			{ 0x00B40, 0x00B40 }, { 0x00B47, 0x00B48 }, { 0x00B4B, 0x00B4C }, { 0x00BBF, 0x00BBF },
			{ 0x00BC1, 0x00BC2 }, { 0x00BC6, 0x00BC8 }, { 0x00BCA, 0x00BCC }, { 0x00C01, 0x00C03 },
			{ 0x00C41, 0x00C44 }, { 0x00C82, 0x00C83 }, { 0x00CBE, 0x00CBE }, { 0x00CC0, 0x00CC4 },
			{ 0x00CC7, 0x00CC8 }, { 0x00CCA, 0x00CCB }, { 0x00D02, 0x00D03 }, { 0x00D3F, 0x00D40 },
			{ 0x00D46, 0x00D48 }, { 0x00D4A, 0x00D4C }, { 0x00D82, 0x00D83 }, { 0x00DD0, 0x00DD1 },
			{ 0x00DD8, 0x00DDE }, { 0x00DF2, 0x00DF3 }, { 0x00E33, 0x00E33 }, { 0x00EB3, 0x00EB3 },
			{ 0x00F3E, 0x00F3F }, { 0x00F7F, 0x00F7F }, { 0x01031, 0x01031 }, { 0x0103B, 0x0103C },
			{ 0x01056, 0x01057 }, { 0x01084, 0x01084 }, { 0x017B6, 0x017B6 }, { 0x017BE, 0x017C5 },
			{ 0x017C7, 0x017C8 }, { 0x01923, 0x01926 }, { 0x01929, 0x0192B }, { 0x01930, 0x01931 },
			{ 0x01933, 0x01938 }, { 0x01A19, 0x01A1A }, { 0x01A55, 0x01A55 }, { 0x01A57, 0x01A57 },
			{ 0x01A6D, 0x01A72 }, { 0x01B04, 0x01B04 }, { 0x01B35, 0x01B35 }, { 0x01B3B, 0x01B3B },
			{ 0x01B3D, 0x01B41 }, { 0x01B43, 0x01B44 }, { 0x01B82, 0x01B82 }, { 0x01BA1, 0x01BA1 },
			{ 0x01BA6, 0x01BA7 }, { 0x01BAA, 0x01BAA }, { 0x01BE7, 0x01BE7 }, { 0x01BEA, 0x01BEC },
			{ 0x01BEE, 0x01BEE }, { 0x01BF2, 0x01BF3 }, { 0x01C24, 0x01C2B }, { 0x01C34, 0x01C35 },
			{ 0x01CE1, 0x01CE1 }, { 0x01CF7, 0x01CF7 }, { 0x0A823, 0x0A824 }, { 0x0A827, 0x0A827 },
			{ 0x0A880, 0x0A881 }, { 0x0A8B4, 0x0A8C3 }, { 0x0A952, 0x0A953 }, { 0x0A983, 0x0A983 },
			{ 0x0A9B4, 0x0A9B5 }, { 0x0A9BA, 0x0A9BB }, { 0x0A9BE, 0x0A9C0 }, { 0x0AA2F, 0x0AA30 },
			{ 0x0AA33, 0x0AA34 }, { 0x0AA4D, 0x0AA4D }, { 0x0AAEB, 0x0AAEB }, { 0x0AAEE, 0x0AAEF },
			{ 0x0AAF5, 0x0AAF5 }, { 0x0ABE3, 0x0ABE4 }, { 0x0ABE6, 0x0ABE7 }, { 0x0ABE9, 0x0ABEA },
			{ 0x0ABEC, 0x0ABEC },
		};

		// Grapheme_Cluster_Break=Prepend: format characters that attach to the *following* cluster.
		constexpr CpRange PrependRanges[] = {
			{ 0x00600, 0x00605 }, { 0x006DD, 0x006DD }, { 0x0070F, 0x0070F }, { 0x00890, 0x00891 },
			{ 0x008E2, 0x008E2 }, { 0x00D4E, 0x00D4E }, { 0x110BD, 0x110BD }, { 0x110CD, 0x110CD },
			{ 0x111C2, 0x111C3 }, { 0x1193F, 0x1193F }, { 0x11941, 0x11941 }, { 0x11A3A, 0x11A3A },
			{ 0x11A84, 0x11A89 }, { 0x11D46, 0x11D46 },
		};

		// Extended_Pictographic, the emoji-like code points rule GB11 joins across a ZWJ. The
		// emoji modifiers 1F3FB..1F3FF are deliberately absent here: they are Extend.
		constexpr CpRange ExtPictRanges[] = {
			{ 0x000A9, 0x000A9 }, { 0x000AE, 0x000AE }, { 0x0203C, 0x0203C }, { 0x02049, 0x02049 },
			{ 0x02122, 0x02122 }, { 0x02139, 0x02139 }, { 0x02194, 0x021AA }, { 0x0231A, 0x0231B },
			{ 0x02328, 0x02328 }, { 0x02388, 0x02388 }, { 0x023CF, 0x023FA }, { 0x024C2, 0x024C2 },
			{ 0x025AA, 0x025FE }, { 0x02600, 0x027BF }, { 0x02934, 0x02935 }, { 0x02B00, 0x02BFF },
			{ 0x03030, 0x03030 }, { 0x0303D, 0x0303D }, { 0x03297, 0x03297 }, { 0x03299, 0x03299 },
			{ 0x1F000, 0x1F0FF }, { 0x1F10D, 0x1F10F }, { 0x1F12F, 0x1F12F }, { 0x1F16C, 0x1F171 },
			{ 0x1F17E, 0x1F17F }, { 0x1F18E, 0x1F18E }, { 0x1F191, 0x1F19A }, { 0x1F1AD, 0x1F1E5 },
			{ 0x1F201, 0x1F20F }, { 0x1F21A, 0x1F21A }, { 0x1F22F, 0x1F22F }, { 0x1F232, 0x1F23A },
			{ 0x1F23C, 0x1F23F }, { 0x1F249, 0x1F3FA }, { 0x1F400, 0x1F53D }, { 0x1F546, 0x1F64F },
			{ 0x1F680, 0x1F6FF }, { 0x1F774, 0x1F77F }, { 0x1F7D5, 0x1F7FF }, { 0x1F80C, 0x1F80F },
			{ 0x1F848, 0x1F84F }, { 0x1F85A, 0x1F85F }, { 0x1F888, 0x1F88F }, { 0x1F8AE, 0x1F8FF },
			{ 0x1F90C, 0x1F93A }, { 0x1F93C, 0x1F945 }, { 0x1F947, 0x1FAFF }, { 0x1FC00, 0x1FFFD },
		};

		// Indic_Conjunct_Break=Linker: the viramas of the six scripts UAX #29 rule GB9c applies to.
		constexpr CpRange IncbLinkerRanges[] = {
			{ 0x0094D, 0x0094D }, { 0x009CD, 0x009CD }, { 0x00ACD, 0x00ACD }, { 0x00B4D, 0x00B4D },
			{ 0x00C4D, 0x00C4D }, { 0x00D4D, 0x00D4D },
		};

		// Indic_Conjunct_Break=Consonant: the Devanagari, Bengali, Gujarati, Oriya, Telugu and
		// Malayalam letters that a virama may join into a conjunct.
		constexpr CpRange IncbConsonantRanges[] = {
			{ 0x00915, 0x00939 }, { 0x00958, 0x0095F }, { 0x00978, 0x0097F }, { 0x00995, 0x009A8 },
			{ 0x009AA, 0x009B0 }, { 0x009B2, 0x009B2 }, { 0x009B6, 0x009B9 }, { 0x009DC, 0x009DD },
			{ 0x009DF, 0x009DF }, { 0x009F0, 0x009F1 }, { 0x00A95, 0x00AA8 }, { 0x00AAA, 0x00AB0 },
			{ 0x00AB2, 0x00AB3 }, { 0x00AB5, 0x00AB9 }, { 0x00AF9, 0x00AF9 }, { 0x00B15, 0x00B28 },
			{ 0x00B2A, 0x00B30 }, { 0x00B32, 0x00B33 }, { 0x00B35, 0x00B39 }, { 0x00B5C, 0x00B5D },
			{ 0x00B5F, 0x00B5F }, { 0x00C15, 0x00C28 }, { 0x00C2A, 0x00C39 }, { 0x00C58, 0x00C5A },
			{ 0x00D15, 0x00D3A },
		};

		// White_Space, the property that decides ECharClass::Whitespace.
		constexpr CpRange WhiteSpaceRanges[] = {
			{ 0x00009, 0x0000D }, { 0x00020, 0x00020 }, { 0x00085, 0x00085 }, { 0x000A0, 0x000A0 },
			{ 0x01680, 0x01680 }, { 0x02000, 0x0200A }, { 0x02028, 0x02029 }, { 0x0202F, 0x0202F },
			{ 0x0205F, 0x0205F }, { 0x03000, 0x03000 },
		};

		// The Unicode punctuation (P*) and symbol (S*) categories, restricted to the blocks this UI
		// can shape and the scripts it can plausibly edit. It covers ASCII and Latin-1 punctuation,
		// the Greek/Armenian/Hebrew/Arabic/Syriac/N'Ko/Indic/Thai/Lao/Tibetan/Myanmar/Ethiopic
		// terminators, General Punctuation, currency, arrows, mathematical and technical operators,
		// box/geometric/miscellaneous symbols, dingbats, supplemental punctuation, CJK punctuation
		// and the half/fullwidth forms. Code points outside it classify as Word, which is the safe
		// direction: word movement then treats them as run content rather than as an extra stop.
		// Extended_Pictographic emoji are intentionally excluded; see ECharClass.
		constexpr CpRange PunctuationRanges[] = {
			{ 0x00021, 0x0002F }, { 0x0003A, 0x00040 }, { 0x0005B, 0x0005E }, { 0x00060, 0x00060 },
			{ 0x0007B, 0x0007E }, { 0x000A1, 0x000A9 }, { 0x000AB, 0x000B1 }, { 0x000B4, 0x000B4 },
			{ 0x000B6, 0x000B8 }, { 0x000BB, 0x000BB }, { 0x000BF, 0x000BF }, { 0x000D7, 0x000D7 },
			{ 0x000F7, 0x000F7 }, { 0x00374, 0x00375 }, { 0x0037E, 0x0037E }, { 0x00384, 0x00385 },
			{ 0x00387, 0x00387 }, { 0x0055A, 0x0055F }, { 0x00589, 0x0058A }, { 0x0058D, 0x0058F },
			{ 0x005BE, 0x005BE }, { 0x005C0, 0x005C0 }, { 0x005C3, 0x005C3 }, { 0x005C6, 0x005C6 },
			{ 0x005F3, 0x005F4 }, { 0x0060C, 0x0060D }, { 0x0061B, 0x0061B }, { 0x0061E, 0x0061F },
			{ 0x0066A, 0x0066D }, { 0x006D4, 0x006D4 }, { 0x006DE, 0x006DE }, { 0x006E9, 0x006E9 },
			{ 0x006FD, 0x006FE }, { 0x00700, 0x0070D }, { 0x007F6, 0x007F9 }, { 0x00830, 0x0083E },
			{ 0x0085E, 0x0085E }, { 0x00964, 0x00965 }, { 0x00970, 0x00970 }, { 0x009F2, 0x009F3 },
			{ 0x009FA, 0x009FB }, { 0x009FD, 0x009FD }, { 0x00A76, 0x00A76 }, { 0x00AF0, 0x00AF1 },
			{ 0x00B70, 0x00B70 }, { 0x00B77, 0x00B77 }, { 0x00BF3, 0x00BFA }, { 0x00C77, 0x00C77 },
			{ 0x00C7F, 0x00C7F }, { 0x00C84, 0x00C84 }, { 0x00D4F, 0x00D4F }, { 0x00D79, 0x00D79 },
			{ 0x00DF4, 0x00DF4 }, { 0x00E3F, 0x00E3F }, { 0x00E4F, 0x00E4F }, { 0x00E5A, 0x00E5B },
			{ 0x00F01, 0x00F17 }, { 0x00F1A, 0x00F1F }, { 0x00F34, 0x00F3D }, { 0x00F85, 0x00F85 },
			{ 0x0104A, 0x0104F }, { 0x010FB, 0x010FB }, { 0x01360, 0x01368 }, { 0x01400, 0x01400 },
			{ 0x0166E, 0x0166E }, { 0x0169B, 0x0169C }, { 0x016EB, 0x016ED }, { 0x017D4, 0x017D6 },
			{ 0x017D8, 0x017DB }, { 0x01800, 0x0180A }, { 0x01944, 0x01945 }, { 0x01A1E, 0x01A1F },
			{ 0x01AA0, 0x01AA6 }, { 0x01AA8, 0x01AAD }, { 0x01B5A, 0x01B6A }, { 0x01B74, 0x01B7C },
			{ 0x01BFC, 0x01BFF }, { 0x01C3B, 0x01C3F }, { 0x01C7E, 0x01C7F }, { 0x01CC0, 0x01CC7 },
			{ 0x01CD3, 0x01CD3 }, { 0x02010, 0x02027 }, { 0x02030, 0x0205E }, { 0x0207A, 0x0207E },
			{ 0x0208A, 0x0208E }, { 0x020A0, 0x020C0 }, { 0x02100, 0x02101 }, { 0x02103, 0x02106 },
			{ 0x02108, 0x02109 }, { 0x02116, 0x02118 }, { 0x0211E, 0x02123 }, { 0x02140, 0x02144 },
			{ 0x0214A, 0x0214D }, { 0x0214F, 0x0214F }, { 0x02190, 0x02426 }, { 0x02440, 0x0244A },
			{ 0x02500, 0x02775 }, { 0x02794, 0x02BFF }, { 0x02E00, 0x02E7F }, { 0x03001, 0x03004 },
			{ 0x03008, 0x03020 }, { 0x03030, 0x03030 }, { 0x0303D, 0x0303F }, { 0x030A0, 0x030A0 },
			{ 0x030FB, 0x030FB }, { 0x0FD3E, 0x0FD3F }, { 0x0FE10, 0x0FE19 }, { 0x0FE30, 0x0FE6B },
			{ 0x0FF01, 0x0FF0F }, { 0x0FF1A, 0x0FF20 }, { 0x0FF3B, 0x0FF3E }, { 0x0FF40, 0x0FF40 },
			{ 0x0FF5B, 0x0FF65 }, { 0x0FFE0, 0x0FFEE },
		};
	}

	EIndicConjunctBreak IndicConjunctBreakOf(char32_t cp)
	{
		if (InTable(cp, IncbLinkerRanges))
			return EIndicConjunctBreak::Linker;
		if (InTable(cp, IncbConsonantRanges))
			return EIndicConjunctBreak::Consonant;

		// Every other Extend or ZWJ code point is InCB=Extend, except ZWNJ which deliberately
		// breaks a conjunct and so must not keep the linker sequence alive.
		if (cp == 0x200C)
			return EIndicConjunctBreak::None;

		auto const brk = GraphemeBreakOf(cp);
		if (brk == EGraphemeBreak::Extend || brk == EGraphemeBreak::ZWJ)
			return EIndicConjunctBreak::Extend;

		return EIndicConjunctBreak::None;
	}

	EGraphemeBreak GraphemeBreakOf(char32_t cp)
	{
		if (cp == 0x000D)
			return EGraphemeBreak::CR;
		if (cp == 0x000A)
			return EGraphemeBreak::LF;
		if (cp == 0x200D)
			return EGraphemeBreak::ZWJ;
		if (cp >= 0x1F1E6 && cp <= 0x1F1FF)
			return EGraphemeBreak::RegionalIndicator;

		// Hangul syllables decompose arithmetically: a syllable whose trailing-jamo index is zero
		// is LV, otherwise LVT (UAX #29 rules GB6-GB8 rely on this distinction).
		if (cp >= 0xAC00 && cp <= 0xD7A3)
			return ((cp - 0xAC00) % 28) == 0 ? EGraphemeBreak::HangulLV : EGraphemeBreak::HangulLVT;
		if ((cp >= 0x1100 && cp <= 0x115F) || (cp >= 0xA960 && cp <= 0xA97C))
			return EGraphemeBreak::HangulL;
		if ((cp >= 0x1160 && cp <= 0x11A7) || (cp >= 0xD7B0 && cp <= 0xD7C6))
			return EGraphemeBreak::HangulV;
		if ((cp >= 0x11A8 && cp <= 0x11FF) || (cp >= 0xD7CB && cp <= 0xD7FB))
			return EGraphemeBreak::HangulT;

		if (InTable(cp, ExtendRanges))
			return EGraphemeBreak::Extend;
		if (InTable(cp, PrependRanges))
			return EGraphemeBreak::Prepend;
		if (InTable(cp, SpacingMarkRanges))
			return EGraphemeBreak::SpacingMark;
		if (InTable(cp, ControlRanges))
			return EGraphemeBreak::Control;
		if (InTable(cp, ExtPictRanges))
			return EGraphemeBreak::ExtendedPictographic;

		return EGraphemeBreak::Other;
	}

	ECharClass CharClassOf(char32_t cp)
	{
		if (InTable(cp, WhiteSpaceRanges))
			return ECharClass::Whitespace;

		// Pictographic code points are content, not delimiters, so they are classified before the
		// punctuation/symbol table that would otherwise capture the symbol blocks they live in.
		if (InTable(cp, ExtPictRanges))
			return ECharClass::Word;

		if (InTable(cp, PunctuationRanges))
			return ECharClass::Punctuation;

		return ECharClass::Word;
	}

	bool Utf8Decode(std::string_view text, std::uint32_t offset, char32_t& out_cp, std::uint32_t& out_length)
	{
		if (offset >= text.size())
			return false;

		auto const* bytes = reinterpret_cast<std::uint8_t const*>(text.data());
		auto const available = static_cast<std::uint32_t>(text.size() - offset);
		auto const lead = bytes[offset];

		// Sequence length and the minimum scalar value the sequence is allowed to encode, so an
		// over-long encoding (a security hazard, not merely a cosmetic one) is rejected below.
		auto length = std::uint32_t{};
		auto cp = char32_t{};
		auto minimum = char32_t{};
		if (lead < 0x80)
		{
			length = 1;
			cp = lead;
			minimum = 0;
		}
		else if ((lead & 0xE0) == 0xC0)
		{
			length = 2;
			cp = lead & 0x1F;
			minimum = 0x80;
		}
		else if ((lead & 0xF0) == 0xE0)
		{
			length = 3;
			cp = lead & 0x0F;
			minimum = 0x800;
		}
		else if ((lead & 0xF8) == 0xF0)
		{
			length = 4;
			cp = lead & 0x07;
			minimum = 0x10000;
		}
		else
		{
			return false;
		}

		if (length > available)
			return false;

		for (auto i = std::uint32_t(1); i != length; ++i)
		{
			auto const trail = bytes[offset + i];
			if ((trail & 0xC0) != 0x80)
				return false;

			cp = (cp << 6) | (trail & 0x3F);
		}

		if (cp < minimum || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF))
			return false;

		out_cp = cp;
		out_length = length;
		return true;
	}

	bool Utf8Validate(std::string_view text)
	{
		for (auto offset = std::uint32_t(0); offset < text.size();)
		{
			char32_t cp = 0;
			std::uint32_t length = 0;
			if (!Utf8Decode(text, offset, cp, length))
				return false;

			offset += length;
		}
		return true;
	}

	void Utf8Append(std::string& out, char32_t cp)
	{
		if (cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF))
			throw EngineException(EStatus::InvalidArgument, std::format("code point U+{:04X} is not a Unicode scalar value", static_cast<std::uint32_t>(cp)));

		if (cp < 0x80)
		{
			out.push_back(static_cast<char>(cp));
		}
		else if (cp < 0x800)
		{
			out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
			out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
		}
		else if (cp < 0x10000)
		{
			out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
			out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
			out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
		}
		else
		{
			out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
			out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
			out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
			out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
		}
	}

	bool Utf16ToUtf8(std::wstring_view text, std::string& out)
	{
		out.clear();
		out.reserve(text.size());
		for (auto i = std::size_t(0); i != text.size(); ++i)
		{
			auto unit = static_cast<char32_t>(static_cast<std::uint16_t>(text[i]));
			if (unit >= 0xD800 && unit <= 0xDBFF)
			{
				// A high surrogate must be followed by a low surrogate; anything else is a
				// malformed payload the caller has to reject rather than silently repair.
				if (i + 1 == text.size())
				{
					out.clear();
					return false;
				}

				auto const low = static_cast<char32_t>(static_cast<std::uint16_t>(text[i + 1]));
				if (low < 0xDC00 || low > 0xDFFF)
				{
					out.clear();
					return false;
				}

				unit = 0x10000 + ((unit - 0xD800) << 10) + (low - 0xDC00);
				++i;
			}
			else if (unit >= 0xDC00 && unit <= 0xDFFF)
			{
				out.clear();
				return false;
			}

			Utf8Append(out, unit);
		}
		return true;
	}

	bool Utf8ToUtf16(std::string_view text, std::wstring& out)
	{
		out.clear();
		out.reserve(text.size());
		for (auto offset = std::uint32_t(0); offset < text.size();)
		{
			char32_t cp = 0;
			std::uint32_t length = 0;
			if (!Utf8Decode(text, offset, cp, length))
			{
				out.clear();
				return false;
			}

			if (cp < 0x10000)
			{
				out.push_back(static_cast<wchar_t>(cp));
			}
			else
			{
				auto const adjusted = cp - 0x10000;
				out.push_back(static_cast<wchar_t>(0xD800 + (adjusted >> 10)));
				out.push_back(static_cast<wchar_t>(0xDC00 + (adjusted & 0x3FF)));
			}
			offset += length;
		}
		return true;
	}

	std::vector<std::uint32_t> GraphemeBoundaries(std::string_view text)
	{
		std::vector<std::uint32_t> boundaries;
		boundaries.reserve(text.size() + 1);
		boundaries.push_back(0);

		auto previous = EGraphemeBreak::Other;
		auto have_previous = false;
		auto regional_run = 0;      // consecutive regional indicators immediately before the cursor (GB12/GB13)
		auto pictographic_run = false; // inside an Extended_Pictographic Extend* sequence (GB11)
		auto conjunct_consonant = false; // an InCB=Consonant has been seen with only InCB Extend/Linker since (GB9c)
		auto conjunct_linker = false;    // ...and at least one of those was an InCB=Linker

		for (auto offset = std::uint32_t(0); offset < text.size();)
		{
			char32_t cp = 0;
			std::uint32_t length = 0;
			if (!Utf8Decode(text, offset, cp, length))
			{
				// Segmentation must stay total even for a malformed buffer, so an undecodable byte
				// forms its own cluster and resets every stateful rule.
				if (offset != 0)
					boundaries.push_back(offset);

				offset += 1;
				have_previous = false;
				regional_run = 0;
				pictographic_run = false;
				conjunct_consonant = false;
				conjunct_linker = false;
				continue;
			}

			auto const current = GraphemeBreakOf(cp);
			auto const conjunct = IndicConjunctBreakOf(cp);
			auto brk = true;
			if (!have_previous)
			{
				brk = false; // GB1: the leading boundary is already recorded
			}
			else if (previous == EGraphemeBreak::CR && current == EGraphemeBreak::LF)
			{
				brk = false; // GB3
			}
			else if (previous == EGraphemeBreak::Control || previous == EGraphemeBreak::CR || previous == EGraphemeBreak::LF)
			{
				brk = true; // GB4
			}
			else if (current == EGraphemeBreak::Control || current == EGraphemeBreak::CR || current == EGraphemeBreak::LF)
			{
				brk = true; // GB5
			}
			else if (previous == EGraphemeBreak::HangulL && (current == EGraphemeBreak::HangulL || current == EGraphemeBreak::HangulV || current == EGraphemeBreak::HangulLV || current == EGraphemeBreak::HangulLVT))
			{
				brk = false; // GB6
			}
			else if ((previous == EGraphemeBreak::HangulLV || previous == EGraphemeBreak::HangulV) && (current == EGraphemeBreak::HangulV || current == EGraphemeBreak::HangulT))
			{
				brk = false; // GB7
			}
			else if ((previous == EGraphemeBreak::HangulLVT || previous == EGraphemeBreak::HangulT) && current == EGraphemeBreak::HangulT)
			{
				brk = false; // GB8
			}
			else if (current == EGraphemeBreak::Extend || current == EGraphemeBreak::ZWJ)
			{
				brk = false; // GB9
			}
			else if (current == EGraphemeBreak::SpacingMark)
			{
				brk = false; // GB9a
			}
			else if (previous == EGraphemeBreak::Prepend)
			{
				brk = false; // GB9b
			}
			else if (conjunct == EIndicConjunctBreak::Consonant && conjunct_consonant && conjunct_linker)
			{
				brk = false; // GB9c: a consonant reached through a virama continues the conjunct cluster
			}
			else if (previous == EGraphemeBreak::ZWJ && current == EGraphemeBreak::ExtendedPictographic && pictographic_run)
			{
				brk = false; // GB11: emoji ZWJ sequences stay one cluster
			}
			else if (previous == EGraphemeBreak::RegionalIndicator && current == EGraphemeBreak::RegionalIndicator && (regional_run % 2) == 1)
			{
				brk = false; // GB12/GB13: flags pair up two regional indicators at a time
			}

			if (brk)
				boundaries.push_back(offset);

			// GB11 tracks an Extended_Pictographic followed by any number of Extend characters and
			// then a ZWJ; any other character ends the sequence.
			if (current == EGraphemeBreak::ExtendedPictographic)
				pictographic_run = true;
			else if (current != EGraphemeBreak::Extend && current != EGraphemeBreak::ZWJ)
				pictographic_run = false;

			regional_run = current == EGraphemeBreak::RegionalIndicator ? regional_run + 1 : 0;

			// GB9c tracks a consonant followed only by InCB Extend/Linker characters; a consonant
			// restarts the sequence, an Extend/Linker continues it (recording whether a linker has
			// been seen), and anything else ends it.
			switch (conjunct)
			{
				case EIndicConjunctBreak::Consonant:
				{
					conjunct_consonant = true;
					conjunct_linker = false;
					break;
				}
				case EIndicConjunctBreak::Linker:
				{
					conjunct_linker = conjunct_consonant;
					break;
				}
				case EIndicConjunctBreak::Extend:
				{
					break;
				}
				case EIndicConjunctBreak::None:
				{
					conjunct_consonant = false;
					conjunct_linker = false;
					break;
				}
				default:
				{
					throw EngineException(EStatus::InternalError, "unknown Indic_Conjunct_Break class");
				}
			}

			previous = current;
			have_previous = true;
			offset += length;
		}

		// For empty text the leading and trailing boundary are the same position, so recording it
		// twice would report one cluster where there are none.
		if (boundaries.back() != text.size())
			boundaries.push_back(static_cast<std::uint32_t>(text.size()));

		return boundaries;
	}

	bool IsGraphemeBoundary(std::string_view text, std::uint32_t offset)
	{
		if (offset == 0 || offset >= text.size())
			return true;

		auto const boundaries = GraphemeBoundaries(text);
		return std::binary_search(boundaries.begin(), boundaries.end(), offset);
	}

	std::uint32_t NextGraphemeBoundary(std::string_view text, std::uint32_t offset)
	{
		auto const size = static_cast<std::uint32_t>(text.size());
		if (offset >= size)
			return size;

		auto const boundaries = GraphemeBoundaries(text);
		auto it = std::upper_bound(boundaries.begin(), boundaries.end(), offset);
		return it != boundaries.end() ? *it : size;
	}

	std::uint32_t PrevGraphemeBoundary(std::string_view text, std::uint32_t offset)
	{
		if (offset == 0)
			return 0;

		auto const size = static_cast<std::uint32_t>(text.size());
		auto const clamped = std::min(offset, size);
		auto const boundaries = GraphemeBoundaries(text);
		auto it = std::lower_bound(boundaries.begin(), boundaries.end(), clamped);
		return it != boundaries.begin() ? *(it - 1) : 0;
	}

	std::uint32_t ClampToGraphemeBoundary(std::string_view text, std::uint32_t offset)
	{
		auto const size = static_cast<std::uint32_t>(text.size());
		if (offset >= size)
			return size;
		if (offset == 0)
			return 0;

		auto const boundaries = GraphemeBoundaries(text);
		auto it = std::upper_bound(boundaries.begin(), boundaries.end(), offset);
		return *(it - 1);
	}

	std::uint32_t NearestGraphemeBoundary(std::string_view text, std::uint32_t offset)
	{
		auto const size = static_cast<std::uint32_t>(text.size());
		if (offset >= size)
			return size;
		if (offset == 0)
			return 0;

		auto const boundaries = GraphemeBoundaries(text);
		auto it = std::upper_bound(boundaries.begin(), boundaries.end(), offset);
		auto const before = *(it - 1);
		if (it == boundaries.end())
			return before;

		// An exact tie resolves to the earlier boundary so a click on the geometric centre of a
		// cluster behaves the same on every machine.
		auto const after = *it;
		return (offset - before) <= (after - offset) ? before : after;
	}

	std::uint32_t GraphemeCount(std::string_view text)
	{
		// GraphemeBoundaries always contains both the leading and trailing boundary, so the cluster
		// count is one less than the boundary count.
		return static_cast<std::uint32_t>(GraphemeBoundaries(text).size() - 1);
	}

	std::uint32_t GraphemeOffsetAt(std::string_view text, std::uint32_t index)
	{
		auto const boundaries = GraphemeBoundaries(text);
		return index < boundaries.size() ? boundaries[index] : static_cast<std::uint32_t>(text.size());
	}

	namespace
	{
		// The character class of the cluster starting at 'offset'; a malformed or empty position
		// reports Whitespace so word scanning terminates rather than looping.
		ECharClass ClassAt(std::string_view text, std::uint32_t offset)
		{
			char32_t cp = 0;
			std::uint32_t length = 0;
			if (!Utf8Decode(text, offset, cp, length))
				return ECharClass::Whitespace;

			return CharClassOf(cp);
		}
	}

	std::uint32_t NextWordBoundary(std::string_view text, std::uint32_t offset)
	{
		auto const size = static_cast<std::uint32_t>(text.size());
		auto pos = ClampToGraphemeBoundary(text, offset);
		if (pos >= size)
			return size;

		// Consume the run the caret currently sits in, then any whitespace after it, so the caret
		// lands on the first character of the next word.
		auto const start_class = ClassAt(text, pos);
		if (start_class != ECharClass::Whitespace)
		{
			while (pos < size && ClassAt(text, pos) == start_class)
				pos = NextGraphemeBoundary(text, pos);
		}
		while (pos < size && ClassAt(text, pos) == ECharClass::Whitespace)
			pos = NextGraphemeBoundary(text, pos);

		return pos;
	}

	std::uint32_t PrevWordBoundary(std::string_view text, std::uint32_t offset)
	{
		auto pos = ClampToGraphemeBoundary(text, offset);
		if (pos == 0)
			return 0;

		// Skip whitespace immediately before the caret, then the whole run before that, so the
		// caret lands on the first character of the word it just moved over.
		while (pos != 0)
		{
			auto const previous = PrevGraphemeBoundary(text, pos);
			if (ClassAt(text, previous) != ECharClass::Whitespace)
				break;

			pos = previous;
		}
		if (pos == 0)
			return 0;

		auto const run_class = ClassAt(text, PrevGraphemeBoundary(text, pos));
		while (pos != 0)
		{
			auto const previous = PrevGraphemeBoundary(text, pos);
			if (ClassAt(text, previous) != run_class)
				break;

			pos = previous;
		}
		return pos;
	}
}
