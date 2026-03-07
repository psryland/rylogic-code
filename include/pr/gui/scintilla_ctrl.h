//***********************************************
// Scintilla control
//  Copyright (c) Rylogic Ltd 2009
//***********************************************
// A 'wingui' control for scintilla
#pragma once
#include "scintilla/include/scintilla.h"
#include "scintilla/include/scilexer.h"
#include "pr/gui/wingui.h"
#include "pr/str/string_core.h"
#include "pr/win32/win32.h"

namespace pr::gui
{
	struct ScintillaCtrl :Control
	{
		// Notes:
		// - Remember to call win32::LoadDll<struct Scintilla>(L"scintilla.dll");
		//   before creating an instance of this control
		// - There is a C# port of this control called Rylogic.Gui.ScintillaCtrl. Try to
		//   keep these in sync

		enum { DefW = 50, DefH = 50 };
		static DWORD const DefaultStyle   = (DefaultControlStyle | WS_GROUP | SS_LEFT) & ~WS_TABSTOP;
		static DWORD const DefaultStyleEx = DefaultControlStyleEx | WS_EX_STATICEDGE; // NOT WS_BORDER|
		static wchar_t const* WndClassName() { return L"Scintilla"; }
			
		template <typename Derived = void>
		struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
		{
			using this_type = typename Params::this_type;
			Params()
			{
				this->wndclass(WndClassName())
					.name("scint")
					.wh(DefW, DefH)
					.style('=',DefaultStyle)
					.style_ex('=',DefaultStyleEx);
			}
			this_type& load_dll(wchar_t const* dllname = L"scintilla.dll", wchar_t const* dir = L".\\lib\\$(platform)")
			{
				win32::LoadDll<struct Scintilla>(dllname, dir);
				return this->me();
			}
		};

		#pragma region Helpers
		struct TxtRng :Sci_TextRange
		{
			TxtRng(char * text, long first, long last) :Sci_TextRange()
			{
				chrg.cpMin = first;
				chrg.cpMax = last;
				lpstrText = text;
			}
		};
		#pragma endregion

	private:

		using uint32_t = unsigned int;
		SciFnDirect m_snd;
		mutable sptr_t m_ptr;
		bool m_auto_indent;

	public:

		// Construct
		ScintillaCtrl() :ScintillaCtrl(Params<>()) {}
		ScintillaCtrl(Params<> const& p)
			:Control(p)
			,m_snd()
			,m_ptr()
			,m_auto_indent(false)
		{}

		// Helper function for calling the direct function and returned the result as 'TRet'
		template <typename TRet, typename WP, typename LP> TRet Cmd(uint32_t msg, WP wparam, LP lparam) const
		{
			auto res = m_snd(m_ptr, msg, (uptr_t)wparam, (sptr_t)lparam);
			return TRet(res);
		}
		template <typename TRet, typename WP> TRet Cmd(uint32_t msg, WP wparam) const
		{
			auto res = m_snd(m_ptr, msg, (uptr_t)wparam, sptr_t());
			return TRet(res);
		}
		template <typename TRet> TRet Cmd(uint32_t msg) const
		{
			auto res = m_snd(m_ptr, msg, uptr_t(), sptr_t());
			return TRet(res);
		}

		void Attach(HWND hwnd) override
		{
			Control::Attach(hwnd);

			// Get the direct access function for the control when the hwnd is available
			m_snd = SciFnDirect(::SendMessageW(m_hwnd, SCI_GETDIRECTFUNCTION, 0, 0));
			m_ptr = sptr_t     (::SendMessageW(m_hwnd, SCI_GETDIRECTPOINTER , 0, 0));
		}
		void Detach() override
		{
			m_snd = nullptr;
			m_ptr = 0;
			Control::Detach();
		}

		// Initialise styles with reasonable defaults
		void InitDefaultStyle()
		{
			CodePage(SC_CP_UTF8);
			ClearDocumentStyle();
			IndentationGuides(true);
			TabWidth(4);
			Indent(4);
			CaretPeriod(400);

			// source folding section
			// tell the lexer that we want folding information - the lexer supplies "folding levels"
			Property("fold"                       , "1");
			Property("fold.html"                  , "1");
			Property("fold.html.preprocessor"     , "1");
			Property("fold.comment"               , "1");
			Property("fold.at.else"               , "1");
			Property("fold.flags"                 , "1");
			Property("fold.preprocessor"          , "1");
			Property("styling.within.preprocessor", "1");
			Property("asp.default.language"       , "1");

			// Tell scintilla to draw folding lines UNDER the folded line
			FoldFlags(16);

			// Set margin 2 = folding margin to display folding symbols
			MarginMaskN(2, SC_MASK_FOLDERS);

			// allow notifications for folding actions
			ModEventMask(SC_MOD_INSERTTEXT|SC_MOD_DELETETEXT);
			//ModEventMask(SC_MOD_CHANGEFOLD|SC_MOD_INSERTTEXT|SC_MOD_DELETETEXT);

			// make the folding margin sensitive to folding events = if you click into the margin you get a notification event
			MarginSensitiveN(2, true);
		
			// define a set of markers to display folding symbols
			MarkerDefine(SC_MARKNUM_FOLDEROPEN    , SC_MARK_MINUS);
			MarkerDefine(SC_MARKNUM_FOLDER        , SC_MARK_PLUS);
			MarkerDefine(SC_MARKNUM_FOLDERSUB     , SC_MARK_EMPTY);
			MarkerDefine(SC_MARKNUM_FOLDERTAIL    , SC_MARK_EMPTY);
			MarkerDefine(SC_MARKNUM_FOLDEREND     , SC_MARK_EMPTY);
			MarkerDefine(SC_MARKNUM_FOLDEROPENMID , SC_MARK_EMPTY);
			MarkerDefine(SC_MARKNUM_FOLDERMIDTAIL , SC_MARK_EMPTY);

			// set the foreground color for some styles
			StyleSetFore(0 , RGB(0   ,0  ,0  ));
			StyleSetFore(2 , RGB(0   ,64 ,0  ));
			StyleSetFore(5 , RGB(0   ,0  ,255));
			StyleSetFore(6 , RGB(200 ,20 ,0  ));
			StyleSetFore(9 , RGB(0   ,0  ,255));
			StyleSetFore(10, RGB(255 ,0  ,64 ));
			StyleSetFore(11, RGB(0   ,0  ,0  ));

			// set the background color of brace highlights
			StyleSetBack(STYLE_BRACELIGHT, RGB(0,255,0));
		
			// set end of line mode to CRLF
			ConvertEOLs(2);
			EOLMode(2);
			//   SndMsg<void>(SCI_SETVIEWEOL, TRUE, 0);

			// set marker symbol for marker type 0 - bookmark
			MarkerDefine(0, SC_MARK_CIRCLE);
		
			//// display all margins
			//DisplayLinenumbers(TRUE);
			//SetDisplayFolding(TRUE);
			//SetDisplaySelection(TRUE);
		}
		void InitLdrStyle(bool dark = false)
		{
			ClearDocumentStyle();
			IndentationGuides(true);
			AutoIndent(true);
			TabWidth(4);
			Indent(4);
			CaretFore(dark ? 0xffffff : 0x000000);
			CaretPeriod(400);
			ConvertEOLs(SC_EOL_LF);
			EOLMode(SC_EOL_LF);
			Property("fold", "1");
			MultipleSelection(true);
			AdditionalSelectionTyping(true);
			VirtualSpace(SCVS_RECTANGULARSELECTION);

			struct StyleDesc { int id; int fore; int back; char const* font; };
			StyleDesc dark_style[] =
				{
					{STYLE_DEFAULT          , 0xc8c8c8 , 0x1e1e1e , "courier new"},
					{STYLE_LINENUMBER       , 0xc8c8c8 , 0x1e1e1e , "courier new"},
					{STYLE_INDENTGUIDE      , 0x484439 , 0x1e1e1e , "courier new"},
					{STYLE_BRACELIGHT       , 0x98642b , 0x5e1e1e , "courier new"},
					{SCE_LDR_DEFAULT        , 0xc8c8c8 , 0x1e1e1e , "courier new"},
					{SCE_LDR_COMMENT_BLK    , 0x4aa656 , 0x1e1e1e , "courier new"},
					{SCE_LDR_COMMENT_LINE   , 0x4aa656 , 0x1e1e1e , "courier new"},
					{SCE_LDR_STRING_LITERAL , 0x859dd6 , 0x1e1e1e , "courier new"},
					{SCE_LDR_CHAR_LITERAL   , 0x859dd6 , 0x1e1e1e , "courier new"},
					{SCE_LDR_NUMBER         , 0xf7f7f8 , 0x1e1e1e , "courier new"},
					{SCE_LDR_KEYWORD        , 0xd69c56 , 0x1e1e1e , "courier new"},
					{SCE_LDR_PREPROC        , 0xc563bd , 0x1e1e1e , "courier new"},
					{SCE_LDR_OBJECT         , 0x81c93d , 0x1e1e1e , "courier new"},
					{SCE_LDR_NAME           , 0xffffff , 0x1e1e1e , "courier new"},
					{SCE_LDR_COLOUR         , 0x7c97c3 , 0x1e1e1e , "courier new"},
				};
			StyleDesc light_style[] =
				{
					{STYLE_DEFAULT          , 0x120700 , 0xffffff , "courier new"},
					{STYLE_LINENUMBER       , 0x120700 , 0xffffff , "courier new"},
					{STYLE_INDENTGUIDE      , 0xc0c0c0 , 0xffffff , "courier new"},
					{STYLE_BRACELIGHT       , 0x2b6498 , 0xffffff , "courier new"},
					{SCE_LDR_DEFAULT        , 0x120700 , 0xffffff , "courier new"},
					{SCE_LDR_COMMENT_BLK    , 0x008100 , 0xffffff , "courier new"},
					{SCE_LDR_COMMENT_LINE   , 0x008100 , 0xffffff , "courier new"},
					{SCE_LDR_STRING_LITERAL , 0x154dc7 , 0xffffff , "courier new"},
					{SCE_LDR_CHAR_LITERAL   , 0x154dc7 , 0xffffff , "courier new"},
					{SCE_LDR_NUMBER         , 0x1e1e1e , 0xffffff , "courier new"},
					{SCE_LDR_KEYWORD        , 0xff0000 , 0xffffff , "courier new"},
					{SCE_LDR_PREPROC        , 0x8a0097 , 0xffffff , "courier new"},
					{SCE_LDR_OBJECT         , 0x81962a , 0xffffff , "courier new"},
					{SCE_LDR_NAME           , 0x000000 , 0xffffff , "courier new"},
					{SCE_LDR_COLOUR         , 0x83573c , 0xffffff , "courier new"},
				};
			static_assert(_countof(dark_style) == _countof(light_style), "");

			auto style = dark ? dark_style : light_style;
			for (int i = 0; i != _countof(dark_style); ++i)
			{
				auto& s = style[i];
				StyleSetFont(s.id, s.font);
				StyleSetFore(s.id, s.fore);
				StyleSetBack(s.id, s.back);
			}

			MarginTypeN(0, SC_MARGIN_NUMBER);
			MarginTypeN(1, SC_MARGIN_SYMBOL);
			
			MarginMaskN(1, SC_MASK_FOLDERS);

			MarginWidthN(0, TextWidth(STYLE_LINENUMBER, "_9999"));
			MarginWidthN(1, 0);

			// set marker symbol for marker type 0 - bookmark
			MarkerDefine(0, SC_MARK_CIRCLE);

			//// display all margins
			//DisplayLinenumbers(TRUE);
			//SetDisplayFolding(TRUE);
			//SetDisplaySelection(TRUE);

			// Initialise UTF-8 with the ldr lexer
			CodePage(SC_CP_UTF8);
			Lexer(SCLEX_LDR);
			LexerLanguage("ldr");
		}

		// Read contents from in 'istream'
		// If the stream is not UTF8, it's the caller's responsibility to
		// imbue the stream and skip over any byte order mask
		void Load(std::istream& in, bool readonly = false)
		{
			ClearAll();
			SetUndoCollection(false);

			for (char buf[8192]; in.read(buf, sizeof(buf)).gcount() != 0;)
					AddText(buf, int(in.gcount()));

			// Reset the cursor
			SetSel(0,0);
			SetUndoCollection(true);
			SetSavePoint();
			ReadOnly(readonly);
		}

		// Write the contents to an 'ostream'
		// The output content will be in utf8, it's the callers responsibility to
		// imbue the stream and add a byte order mask if needed.
		// Also, if this is a save, remember to call 'SetSavePoint()' to register the save point with scintilla
		void Save(std::ostream& out)
		{
			char buf[8192];
			Sci_TextRange range = {{},buf};
			for (std::size_t i = 0, iend = LengthInBytes(), writ; i != iend; i += writ)
			{
				writ = std::min(iend - i, sizeof(buf));
				range.chrg.cpMin = long(i);
				range.chrg.cpMax = long(i + writ);
				GetTextRange(range);

				if (!out.write(buf, writ).good())
					throw std::exception("Failed to write to output stream");
			}
		}

		// Get/Set the text in the control
		// Scintilla uses UTF-8 and so always deals with arrays of chars
		// If you're expecting Unicode, use pr::Widen
		std::string Text() const
		{
			std::string str;
			str.resize(LengthInBytes() + 1);
			GetText(&str[0], int(str.size()));
			return str;
		}
		void Text(char const* str)
		{
			SetText(str);
			SetSavePoint();
		}

		// Returns the length of the document in bytes (Note: *not* characters)
		size_t LengthInBytes() const
		{
			return Cmd<size_t>(SCI_GETLENGTH, 0, 0L); // Same as SCI_GETTEXTLENGTH
		}

		// Message map function
		// Return true to halt message processing, false to allow other controls to process the message
		bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
		{
			//WndProcDebug(hwnd, message, wparam, lparam, "Scint");
			switch (message)
			{
			case WM_NOTIFY:
				{
					// Handle notifications from the control
					auto hdr = reinterpret_cast<NMHDR*>(lparam);
					if (hdr->hwndFrom == m_hwnd)
					{
						auto nf = reinterpret_cast<SCNotification*>(lparam);
						return HandleSCNotification(hdr->code, nf);
					}
					break;
				}
			}
			return Control::ProcessWindowMessage(hwnd, message, wparam, lparam, result);
		}

		// Respond to notifications from the control
		virtual bool HandleSCNotification(int notification_code, SCNotification const* nf)
		{
			switch (notification_code)
			{
			case SCN_CHARADDED:
				{
					#pragma region Auto Indent
					if (AutoIndent())
					{
						auto lem = Cmd<int>(SCI_GETEOLMODE);
						auto lend =
							(lem == SC_EOL_CR && nf->ch == '\r') ||
							(lem == SC_EOL_LF && nf->ch == '\n') ||
							(lem == SC_EOL_CRLF && nf->ch == '\n');
						if (lend)
						{
							auto line = Cmd<int>(SCI_LINEFROMPOSITION, Cmd<int>(SCI_GETCURRENTPOS));
							auto indent = line > 0 ? Cmd<int>(SCI_GETLINEINDENTATION, line - 1) : 0;
							Cmd<void>(SCI_SETLINEINDENTATION, line, indent);
							Cmd<void>(SCI_GOTOPOS, Cmd<int>(SCI_GETLINEENDPOSITION, line));
						}
					}
					#pragma endregion
					break;
				}
			}
			return false;
		}

		#pragma region Text
		void ClearAll()                                            { return Cmd <void >(SCI_CLEARALL, 0, 0L); }
		void ClearDocumentStyle()                                  { return Cmd <void >(SCI_CLEARDOCUMENTSTYLE, 0, 0L); }
		int GetText(char * text, int length) const                 { return Cmd <int >(SCI_GETTEXT, length, text); }
		void SetText(const char * text)                            { return Cmd <void >(SCI_SETTEXT, 0, text); }
		char GetCharAt(uint32_t pos) const                         { return Cmd <char >(SCI_GETCHARAT, pos, 0L) & 0xFF; }
		int GetLine(int line, char * text) const                   { return Cmd <int >(SCI_GETLINE, line, text); }
		int GetLineCount() const                                   { return Cmd <int >(SCI_GETLINECOUNT, 0, 0L); }
		int GetTextRange(Sci_TextRange & tr) const                 { return Cmd <int >(SCI_GETTEXTRANGE, 0, &tr); }
		void AppendText(const char * text, int length)             { return Cmd <void >(SCI_APPENDTEXT, length, text); }
		void InsertText(uint32_t pos, const char * text)           { return Cmd <void >(SCI_INSERTTEXT, pos, text); }
		void ReplaceSel(const char * text)                         { return Cmd <void >(SCI_REPLACESEL, 0, text); }
		void AddText(const char * text, int length)                { return Cmd <void >(SCI_ADDTEXT, length, text); }
		void AddStyledText(const char * text, int length)          { return Cmd <void >(SCI_ADDSTYLEDTEXT, length, text); }
		int GetStyleAt(uint32_t pos) const                         { return Cmd <int >(SCI_GETSTYLEAT, pos, 0L); }
		int GetStyledText(Sci_TextRange & tr) const                { return Cmd <int >(SCI_GETSTYLEDTEXT, 0, &tr); }
		int GetStyledText(char * text, long first, long last) const{ TxtRng tr(text, first, last); return Cmd <int >(SCI_GETSTYLEDTEXT, 0, &tr); }
		int TargetAsUTF8(char * text)                              { return Cmd <int >(SCI_TARGETASUTF8, 0, text); }
		int EncodedFromUTF8(const char * utf8, char * encoded)     { return Cmd <int >(SCI_ENCODEDFROMUTF8, ( WPARAM)utf8, (LPARAM )encoded); }
		void SetLengthForEncode(int bytes)                         { return Cmd <void >(SCI_SETLENGTHFORENCODE, bytes, 0L); }
		#pragma endregion

		#pragma region Selection/Navigation
		void SelectAll()                                            { return Cmd <void >(SCI_SELECTALL, 0, 0L); }
		int SelectionMode() const                                   { return Cmd <int >(SCI_GETSELECTIONMODE, 0, 0L); }
		void SelectionMode(int mode)                                { return Cmd <void >(SCI_SETSELECTIONMODE, mode, 0L); }
		uint32_t CurrentPos() const                                 { return Cmd <uint32_t >(SCI_GETCURRENTPOS, 0, 0L); }
		void CurrentPos(uint32_t position)                          { return Cmd <void >(SCI_SETCURRENTPOS, position, 0L); }
		uint32_t SelectionStart() const                             { return Cmd <uint32_t >(SCI_GETSELECTIONSTART, 0, 0L); }
		void SelectionStart(uint32_t pos)                           { return Cmd <void >(SCI_SETSELECTIONSTART, pos, 0L); }
		uint32_t SelectionEnd() const                               { return Cmd <uint32_t >(SCI_GETSELECTIONEND, 0, 0L); }
		void SelectionEnd(uint32_t pos)                             { return Cmd <void >(SCI_SETSELECTIONEND, pos, 0L); }
		void SetSel(int start, int end)                             { return Cmd <void >(SCI_SETSEL, start, end); }
		int GetSelText(char * text) const                           { return Cmd <int >(SCI_GETSELTEXT, 0, text); }
		int GetCurLine(char * text, int length) const               { return Cmd <int >(SCI_GETCURLINE, length, text); }
		uint32_t GetLineSelStartPosition(int line) const            { return Cmd <uint32_t >(SCI_GETLINESELSTARTPOSITION, line, 0L); }
		uint32_t GetLineSelEndPosition(int line) const              { return Cmd <uint32_t >(SCI_GETLINESELENDPOSITION, line, 0L); }
		int GetFirstVisibleLine() const                             { return Cmd <int >(SCI_GETFIRSTVISIBLELINE, 0, 0L); }
		int LinesOnScreen() const                                   { return Cmd <int >(SCI_LINESONSCREEN, 0, 0L); }
		bool GetModify() const                                      { return Cmd <int >(SCI_GETMODIFY, 0, 0L) != 0; }
		void GotoPos(uint32_t pos)                                  { return Cmd <void >(SCI_GOTOPOS, pos, 0L); }
		void GotoLine(int line)                                     { return Cmd <void >(SCI_GOTOLINE, line, 0L); }
		uint32_t Anchor() const                                     { return Cmd <uint32_t >(SCI_GETANCHOR, 0, 0L); }
		void Anchor(uint32_t anchor)                                { return Cmd <void >(SCI_SETANCHOR, anchor, 0L); }
		int LineFromPosition(uint32_t pos) const                    { return Cmd <int >(SCI_LINEFROMPOSITION, pos, 0L); }
		uint32_t PositionFromLine(int line) const                   { return Cmd <uint32_t >(SCI_POSITIONFROMLINE, line, 0L); }
		int GetLineEndPosition(int line) const                      { return Cmd <int >(SCI_GETLINEENDPOSITION, line, 0L); }
		int LineLength(int line) const                              { return Cmd <int >(SCI_LINELENGTH, line, 0L); }
		int GetColumn(uint32_t pos) const                           { return Cmd <int >(SCI_GETCOLUMN, pos, 0L); }
		int FindColumn(int line, int column) const                  { return Cmd <int >(SCI_FINDCOLUMN, line, column); }
		uint32_t PositionFromPoint(int x, int y) const              { return Cmd <uint32_t >(SCI_POSITIONFROMPOINT, x, y); }
		uint32_t PositionFromPointClose(int x, int y) const         { return Cmd <uint32_t >(SCI_POSITIONFROMPOINTCLOSE, x, y); }
		int PointXFromPosition(uint32_t pos) const                  { return Cmd <int >(SCI_POINTXFROMPOSITION, 0, pos); }
		int PointYFromPosition(uint32_t pos) const                  { return Cmd <int >(SCI_POINTYFROMPOSITION, 0, pos); }
		void HideSelection(bool normal)                             { return Cmd <void >(SCI_HIDESELECTION, normal, 0L); }
		bool SelectionIsRectangle() const                           { return Cmd <int >(SCI_SELECTIONISRECTANGLE, 0, 0L) != 0; }
		void MoveCaretInsideView()                                  { return Cmd <void >(SCI_MOVECARETINSIDEVIEW, 0, 0L); }
		int WordStartPosition(uint32_t pos, bool onlyWordCharacters){ return Cmd <int >(SCI_WORDSTARTPOSITION, pos, onlyWordCharacters); }
		int WordEndPosition(uint32_t pos, bool onlyWordCharacters)  { return Cmd <int >(SCI_WORDENDPOSITION, pos, onlyWordCharacters); }
		uint32_t PositionBefore(uint32_t pos) const                 { return Cmd <uint32_t >(SCI_POSITIONBEFORE, pos, 0L); }
		uint32_t PositionAfter(uint32_t pos) const                  { return Cmd <uint32_t >(SCI_POSITIONAFTER, pos, 0L); }
		int TextWidth(int style, const char * text)                 { return Cmd <int >(SCI_TEXTWIDTH, style, text); }
		int TextHeight(int line) const                              { return Cmd <int >(SCI_TEXTHEIGHT, line, 0L); }
		void ChooseCaretX() const                                   { return Cmd <void >(SCI_CHOOSECARETX, 0, 0L); }

		// Enable or disable multiple selection. When multiple selection is disabled, it is not
		// possible to select multiple ranges by holding down the Ctrl key while dragging with the mouse.
		bool MultipleSelection() const       { return Cmd<int >(SCI_GETMULTIPLESELECTION, 0, 0L) != 0; }
		void MultipleSelection(bool enabled) { return Cmd<void>(SCI_SETMULTIPLESELECTION, enabled, 0L); }
		
		// Whether typing, backspace, or delete works with multiple selections simultaneously.
		bool AdditionalSelectionTyping() const       { return Cmd<int >(SCI_GETADDITIONALSELECTIONTYPING, 0, 0L) != 0; }
		void AdditionalSelectionTyping(bool enabled) { return Cmd<void>(SCI_SETADDITIONALSELECTIONTYPING, enabled, 0L); }

		// When pasting into multiple selections, the pasted text can go into just the main selection with
		// SC_MULTIPASTE_ONCE=0 or into each selection with SC_MULTIPASTE_EACH=1. SC_MULTIPASTE_ONCE is the default.
		int  MutliPaste() const    { return Cmd<int >(SCI_GETMULTIPASTE, 0, 0L); }
		void MutliPaste(int flags) { return Cmd<void>(SCI_SETMULTIPASTE, flags, 0L);  }

		// Virtual space can be enabled or disabled for rectangular selections or in other circumstances or in both.
		// There are two bit flags SCVS_RECTANGULARSELECTION=1 and SCVS_USERACCESSIBLE=2 which can be set independently.
		// SCVS_NONE=0, the default, disables all use of virtual space.
		int  VirtualSpace() const    { return Cmd<int >(SCI_GETVIRTUALSPACEOPTIONS, 0, 0L); }
		void VirtualSpace(int flags) { return Cmd<void>(SCI_SETVIRTUALSPACEOPTIONS, flags, 0L); }

		// Insert/Overwrite
		bool GetOvertype() const        { return Cmd<int >(SCI_GETOVERTYPE, 0, 0L) != 0; }
		void SetOvertype(bool overtype) { return Cmd<void>(SCI_SETOVERTYPE, overtype, 0L); }
		#pragma endregion

		#pragma region Indenting
		// Get/Set auto indent mode on/off
		bool AutoIndent() const { return m_auto_indent; }
		void AutoIndent(bool enable) { m_auto_indent = enable; }
		#pragma endregion

		#pragma region Cut, Copy And Paste
		void Cut                   ()                              { return Cmd<void>(SCI_CUT, 0, 0L); }
		void Copy                  ()                              { return Cmd<void>(SCI_COPY, 0, 0L); }
		void Paste                 ()                              { return Cmd<void>(SCI_PASTE, 0, 0L); }
		bool CanPaste              () const                        { return Cmd<int >(SCI_CANPASTE, 0, 0L) != 0; }
		void Clear                 ()                              { return Cmd<void>(SCI_CLEAR, 0, 0L); }
		void CopyRange             (uint32_t first, uint32_t last) { return Cmd<void>(SCI_COPYRANGE, first, last); }
		void CopyText              (const char * text, int length) { return Cmd<void>(SCI_COPYTEXT, length, text); }
		void SetPasteConvertEndings(bool convert)                  { return Cmd<void>(SCI_SETPASTECONVERTENDINGS, convert, 0L); }
		bool GetPasteConvertEndings() const                        { return Cmd<int >(SCI_GETPASTECONVERTENDINGS, 0, 0L) != 0; }
		#pragma endregion

		#pragma region Undo/Redo
		void Undo             ()                 { return Cmd<void>(SCI_UNDO, 0, 0L); }
		void Redo             ()                 { return Cmd<void>(SCI_REDO, 0, 0L); }
		bool CanUndo          () const           { return Cmd<int >(SCI_CANUNDO, 0, 0L) != 0; }
		bool CanRedo          () const           { return Cmd<int >(SCI_CANREDO, 0, 0L) != 0; }
		void EmptyUndoBuffer  ()                 { return Cmd<void>(SCI_EMPTYUNDOBUFFER, 0, 0L); }
		void SetUndoCollection(bool collectUndo) { return Cmd<void>(SCI_SETUNDOCOLLECTION, collectUndo, 0L); }
		bool GetUndoCollection() const           { return Cmd<int >(SCI_GETUNDOCOLLECTION, 0, 0L) != 0; }
		void BeginUndoAction  ()                 { return Cmd<void>(SCI_BEGINUNDOACTION, 0, 0L); }
		void EndUndoAction    ()                 { return Cmd<void>(SCI_ENDUNDOACTION, 0, 0L); }
		#pragma endregion

		#pragma region Find/Search/Replace
		uint32_t Find               (int flags, Sci_TextToFind & ttf) const { return Cmd<uint32_t>(SCI_FINDTEXT, flags, &ttf); }
		void SearchAnchor       ()                                          { return Cmd<void>(SCI_SEARCHANCHOR, 0, 0L); }
		int  SearchNext         (int flags, const char * text) const        { return Cmd<int >(SCI_SEARCHNEXT, flags, text); }
		int  SearchPrev         (int flags, const char * text) const        { return Cmd<int >(SCI_SEARCHPREV, flags, text); }
		uint32_t GetTargetStart     () const                                { return Cmd<uint32_t>(SCI_GETTARGETSTART, 0, 0L); }
		void SetTargetStart     (uint32_t pos)                              { return Cmd<void>(SCI_SETTARGETSTART, pos, 0L); }
		uint32_t GetTargetEnd       () const                                { return Cmd<uint32_t>(SCI_GETTARGETEND, 0, 0L); }
		void SetTargetEnd       (uint32_t pos)                              { return Cmd<void>(SCI_SETTARGETEND, pos, 0L); }
		void TargetFromSelection()                                          { return Cmd<void>(SCI_TARGETFROMSELECTION, 0, 0L); }
		int  GetSearchFlags     () const                                    { return Cmd<int >(SCI_GETSEARCHFLAGS, 0, 0L); }
		void SetSearchFlags     (int flags)                                 { return Cmd<void>(SCI_SETSEARCHFLAGS, flags, 0L); }
		int  SearchInTarget     (const char * text, int length)             { return Cmd<int >(SCI_SEARCHINTARGET, length, text); }
		int  ReplaceTarget      (const char * text, int length)             { return Cmd<int >(SCI_REPLACETARGET, length, text); }
		int  ReplaceTargetRE    (const char * text, int length)             { return Cmd<int >(SCI_REPLACETARGETRE, length, text); }
		#pragma endregion

		#pragma region Scrolling
		void LineScroll(int columns, int lines)   { return Cmd<void>(SCI_LINESCROLL, columns, lines); }
		void ScrollToLine(int line)               { return LineScroll( 0, line - LineFromPosition(CurrentPos())); }
		void ScrollCaret()                        { return Cmd<void>(SCI_SCROLLCARET, 0, 0L); }
		bool GetHScrollBar() const                { return Cmd<int >(SCI_GETHSCROLLBAR, 0, 0L) != 0; }
		void SetHScrollBar(bool show)             { return Cmd<void>(SCI_SETHSCROLLBAR, show, 0L); }
		bool GetVScrollBar() const                { return Cmd<int >(SCI_GETVSCROLLBAR, 0, 0L) != 0; }
		void SetVScrollBar(bool show)             { return Cmd<void>(SCI_SETVSCROLLBAR, show, 0L); }
		int  GetXOffset() const                   { return Cmd<int >(SCI_GETXOFFSET, 0, 0L); }
		void SetXOffset(int offset)               { return Cmd<void>(SCI_SETXOFFSET, offset, 0L); }
		int  GetScrollWidth() const               { return Cmd<int >(SCI_GETSCROLLWIDTH, 0, 0L); }
		void SetScrollWidth(int pixelWidth)       { return Cmd<void>(SCI_SETSCROLLWIDTH, pixelWidth, 0L); }
		bool GetEndAtLastLine() const             { return Cmd<int >(SCI_GETENDATLASTLINE, 0, 0L) != 0; }
		void SetEndAtLastLine(bool endAtLastLine) { return Cmd<void>(SCI_SETENDATLASTLINE, endAtLastLine, 0L); }
		#pragma endregion

		#pragma region Whitespace
		int  GetViewWS() const                                 { return Cmd<int >(SCI_GETVIEWWS, 0, 0L); }
		void SetViewWS(int viewWS)                             { return Cmd<void>(SCI_SETVIEWWS, viewWS, 0L); }
		void SetWhitespaceFore(bool useSetting, COLORREF fore) { return Cmd<void>(SCI_SETWHITESPACEFORE, useSetting, fore); }
		void SetWhitespaceBack(bool useSetting, COLORREF back) { return Cmd<void>(SCI_SETWHITESPACEBACK, useSetting, back); }
		#pragma endregion

		#pragma region Cursor
		int  GetCursor() const         { return Cmd<int >(SCI_GETCURSOR, 0, 0L); }
		void SetCursor(int cursorType) { return Cmd<void>(SCI_SETCURSOR, cursorType, 0L); }
		#pragma endregion

		#pragma region Mouse Capture
		bool GetMouseDownCaptures() const        { return Cmd<int >(SCI_GETMOUSEDOWNCAPTURES, 0, 0L) != 0; }
		void SetMouseDownCaptures(bool captures) { return Cmd<void>(SCI_SETMOUSEDOWNCAPTURES, captures, 0L); }
		#pragma endregion

		#pragma region End of Line
		int  EOLMode    () const       { return Cmd<int >(SCI_GETEOLMODE, 0, 0L); }
		void EOLMode    (int eolMode)  { return Cmd<void>(SCI_SETEOLMODE, eolMode, 0L); }
		void ConvertEOLs(int eolMode)  { return Cmd<void>(SCI_CONVERTEOLS, eolMode, 0L); }
		bool ViewEOL    () const       { return Cmd<int >(SCI_GETVIEWEOL, 0, 0L) != 0; }
		void ViewEOL    (bool visible) { return Cmd<void>(SCI_SETVIEWEOL, visible, 0L); }
		#pragma endregion

		#pragma region Style
		void StyleClearAll       ()                                       { return Cmd<void>(SCI_STYLECLEARALL, 0, 0L); }
		void StyleSetFont        (int style, const char * fontName) const { return Cmd<void>(SCI_STYLESETFONT, style, fontName); }
		void StyleSetSize        (int style, int sizePoints)              { return Cmd<void>(SCI_STYLESETSIZE, style, sizePoints); }
		void StyleSetBold        (int style, bool bold)                   { return Cmd<void>(SCI_STYLESETBOLD, style, bold); }
		void StyleSetItalic      (int style, bool italic)                 { return Cmd<void>(SCI_STYLESETITALIC, style, italic); }
		void StyleSetUnderline   (int style, bool underline)              { return Cmd<void>(SCI_STYLESETUNDERLINE, style, underline); }
		void StyleSetFore        (int style, COLORREF fore)               { return Cmd<void>(SCI_STYLESETFORE, style, fore); }
		void StyleSetBack        (int style, COLORREF back)               { return Cmd<void>(SCI_STYLESETBACK, style, back); }
		void StyleSetEOLFilled   (int style, bool filled)                 { return Cmd<void>(SCI_STYLESETEOLFILLED, style, filled); }
		void StyleSetCharacterSet(int style, int characterSet)            { return Cmd<void>(SCI_STYLESETCHARACTERSET, style, characterSet); }
		void StyleSetCase        (int style, int caseForce)               { return Cmd<void>(SCI_STYLESETCASE, style, caseForce); }
		void StyleSetVisible     (int style, bool visible)                { return Cmd<void>(SCI_STYLESETVISIBLE, style, visible); }
		void StyleSetChangeable  (int style, bool changeable)             { return Cmd<void>(SCI_STYLESETCHANGEABLE, style, changeable); }
		void StyleSetHotSpot     (int style, bool hotspot)                { return Cmd<void>(SCI_STYLESETHOTSPOT, style, hotspot); }
		uint32_t GetEndStyled        () const                             { return Cmd<uint32_t>(SCI_GETENDSTYLED, 0, 0L); }
		void StartStyling        (uint32_t pos, int mask)                 { return Cmd<void>(SCI_STARTSTYLING, pos, mask); }
		void SetStyling          (int length, int style)                  { return Cmd<void>(SCI_SETSTYLING, length, style); }
		void SetStylingEx        (int length, const char * styles)        { return Cmd<void>(SCI_SETSTYLINGEX, length, styles); }
		int  GetLineState        (int line) const                         { return Cmd<int >(SCI_GETLINESTATE, line, 0L); }
		void SetLineState        (int line, int state)                    { return Cmd<void>(SCI_SETLINESTATE, line, state); }
		int  GetMaxLineState     () const                                 { return Cmd<int >(SCI_GETMAXLINESTATE, 0, 0L); }
		void StyleResetDefault   ()                                       { return Cmd<void>(SCI_STYLERESETDEFAULT, 0, 0); }
		#pragma endregion

		#pragma region Control Char Symbol
		int  GetControlCharSymbol() const     { return Cmd<int >(SCI_GETCONTROLCHARSYMBOL, 0, 0L); }
		void SetControlCharSymbol(int symbol) { return Cmd<void>(SCI_SETCONTROLCHARSYMBOL, symbol, 0L); }
		#pragma endregion

		#pragma region Caret Style
		void     SetXCaretPolicy  (int caretPolicy, int caretSlop)     { return Cmd<void    >(SCI_SETXCARETPOLICY, caretPolicy, caretSlop); }
		void     SetYCaretPolicy  (int caretPolicy, int caretSlop)     { return Cmd<void    >(SCI_SETYCARETPOLICY, caretPolicy, caretSlop); }
		void     SetVisiblePolicy (int visiblePolicy, int visibleSlop) { return Cmd<void    >(SCI_SETVISIBLEPOLICY, visiblePolicy, visibleSlop); }
		void     ToggleCaretSticky()                                   { return Cmd<void    >(SCI_TOGGLECARETSTICKY, 0, 0L); }
		COLORREF CaretFore        () const                             { return Cmd<COLORREF>(SCI_GETCARETFORE, 0, 0L); }
		void     CaretFore        (COLORREF fore)                      { return Cmd<void    >(SCI_SETCARETFORE, fore, 0L); }
		bool     CaretLineVisible () const                             { return Cmd<int     >(SCI_GETCARETLINEVISIBLE, 0, 0L) != 0; }
		void     CaretLineVisible (bool show)                          { return Cmd<void    >(SCI_SETCARETLINEVISIBLE, show, 0L); }
		COLORREF CaretLineBack    () const                             { return Cmd<COLORREF>(SCI_GETCARETLINEBACK, 0, 0L); }
		void     CaretLineBack    (COLORREF back)                      { return Cmd<void    >(SCI_SETCARETLINEBACK, back, 0L); }
		int      CaretPeriod      () const                             { return Cmd<int     >(SCI_GETCARETPERIOD, 0, 0L); }
		void     CaretPeriod      (int periodMilliseconds)             { return Cmd<void    >(SCI_SETCARETPERIOD, periodMilliseconds, 0L); }
		int      CaretWidth       () const                             { return Cmd<int     >(SCI_GETCARETWIDTH, 0, 0L); }
		void     CaretWidth       (int pixelWidth)                     { return Cmd<void    >(SCI_SETCARETWIDTH, pixelWidth, 0L); }
		bool     CaretSticky      () const                             { return Cmd<int     >(SCI_GETCARETSTICKY, 0, 0L) != 0; }
		void     CaretSticky      (bool useCaretStickyBehaviour)       { return Cmd<void    >(SCI_SETCARETSTICKY, useCaretStickyBehaviour, 0L); }
		#pragma endregion

		#pragma region Selection Style
		void SetSelFore(bool useSetting, COLORREF fore) { return Cmd<void>(SCI_SETSELFORE, useSetting, fore); }
		void SetSelBack(bool useSetting, COLORREF back) { return Cmd<void>(SCI_SETSELBACK, useSetting, back); }
		#pragma endregion

		#pragma region Hotspot Style
		void SetHotspotActiveFore     (bool useSetting, COLORREF fore) { return Cmd<void>(SCI_SETHOTSPOTACTIVEFORE, useSetting, fore); }
		void SetHotspotActiveBack     (bool useSetting, COLORREF back) { return Cmd<void>(SCI_SETHOTSPOTACTIVEBACK, useSetting, back); }
		void SetHotspotActiveUnderline(bool underline)                 { return Cmd<void>(SCI_SETHOTSPOTACTIVEUNDERLINE, underline, 0L); }
		void SetHotspotSingleLine     (bool singleLine)                { return Cmd<void>(SCI_SETHOTSPOTSINGLELINE, singleLine, 0L); }
		#pragma endregion

		#pragma region Margins
		int  MarginTypeN     (int margin) const               { return Cmd<int >(SCI_GETMARGINTYPEN, margin, 0L); }
		void MarginTypeN     (int margin, int marginType)     { return Cmd<void>(SCI_SETMARGINTYPEN, margin, marginType); }
		int  MarginWidthN    (int margin) const               { return Cmd<int >(SCI_GETMARGINWIDTHN, margin, 0L); }
		void MarginWidthN    (int margin, int pixelWidth)     { return Cmd<void>(SCI_SETMARGINWIDTHN, margin, pixelWidth); }
		int  MarginMaskN     (int margin) const               { return Cmd<int >(SCI_GETMARGINMASKN, margin, 0L); }
		void MarginMaskN     (int margin, int mask)           { return Cmd<void>(SCI_SETMARGINMASKN, margin, mask); }
		bool MarginSensitiveN(int margin) const               { return Cmd<int >(SCI_GETMARGINSENSITIVEN, margin, 0L) != 0; }
		void MarginSensitiveN(int margin, bool sensitive)     { return Cmd<void>(SCI_SETMARGINSENSITIVEN, margin, sensitive); }
		int  MarginLeft      () const                         { return Cmd<int >(SCI_GETMARGINLEFT, 0, 0L); }
		void MarginLeft      (int pixelWidth)                 { return Cmd<void>(SCI_SETMARGINLEFT, 0, pixelWidth); }
		int  MarginRight     () const                         { return Cmd<int >(SCI_GETMARGINRIGHT, 0, 0L); }
		void MarginRight     (int pixelWidth)                 { return Cmd<void>(SCI_SETMARGINRIGHT, 0, pixelWidth); }
		#pragma endregion

		#pragma region Brace Highlighting
		void BraceHighlight(uint32_t pos1, uint32_t pos2) { return Cmd<void>(SCI_BRACEHIGHLIGHT, pos1, pos2); }
		void BraceBadLight (uint32_t pos)             { return Cmd<void>(SCI_BRACEBADLIGHT, pos, 0L); }
		uint32_t BraceMatch    (uint32_t pos)             { return Cmd<uint32_t>(SCI_BRACEMATCH, pos, 0L); }
		#pragma endregion

		#pragma region Tabs
		int  TabWidth          () const                   { return Cmd <int >(SCI_GETTABWIDTH, 0, 0); }
		void TabWidth          (int tabWidth)             { return Cmd <void>(SCI_SETTABWIDTH, tabWidth, 0); }
		bool UseTabs           () const                   { return Cmd<int >(SCI_GETUSETABS, 0, 0L) != 0; }
		void UseTabs           (bool useTabs)             { return Cmd<void>(SCI_SETUSETABS, useTabs, 0L); }
		int  Indent            () const                   { return Cmd<int >(SCI_GETINDENT, 0, 0L); }
		void Indent            (int indentSize)           { return Cmd<void>(SCI_SETINDENT, indentSize, 0L); }
		bool TabIndents        () const                   { return Cmd<int >(SCI_GETTABINDENTS, 0, 0L) != 0; }
		void TabIndents        (bool tabIndents)          { return Cmd<void>(SCI_SETTABINDENTS, tabIndents, 0L); }
		bool BackSpaceUnIndents() const                   { return Cmd<int >(SCI_GETBACKSPACEUNINDENTS, 0, 0L) != 0; }
		void BackSpaceUnIndents(bool bsUnIndents)         { return Cmd<void>(SCI_SETBACKSPACEUNINDENTS, bsUnIndents, 0L); }
		int  LineIndentation   (int line) const           { return Cmd<int >(SCI_GETLINEINDENTATION, line, 0L); }
		void LineIndentation   (int line, int indentSize) { return Cmd<void>(SCI_SETLINEINDENTATION, line, indentSize); }
		uint32_t LineIndentPosition(int line) const       { return Cmd<uint32_t>(SCI_GETLINEINDENTPOSITION, line, 0L); }
		bool IndentationGuides () const                   { return Cmd<int >(SCI_GETINDENTATIONGUIDES, 0, 0L) != 0; }
		void IndentationGuides (bool show)                { return Cmd<void>(SCI_SETINDENTATIONGUIDES, show, 0L); }
		int  HighlightGuide    () const                   { return Cmd<int >(SCI_GETHIGHLIGHTGUIDE, 0, 0L); }
		void HighlightGuide    (int column)               { return Cmd<void>(SCI_SETHIGHLIGHTGUIDE, column, 0L); }
		#pragma endregion

		#pragma region Markers
		void MarkerDefine        (int markerNumber, int markerSymbol)    { return Cmd<void>(SCI_MARKERDEFINE, markerNumber, markerSymbol); }
		void MarkerDefinePixmap  (int markerNumber, const char * pixmap) { return Cmd<void>(SCI_MARKERDEFINEPIXMAP, markerNumber, pixmap); }
		void MarkerSetFore       (int markerNumber, COLORREF fore)       { return Cmd<void>(SCI_MARKERSETFORE, markerNumber, fore); }
		void MarkerSetBack       (int markerNumber, COLORREF back)       { return Cmd<void>(SCI_MARKERSETBACK, markerNumber, back); }
		int  MarkerAdd           (int line, int markerNumber)            { return Cmd<int >(SCI_MARKERADD, line, markerNumber); }
		int  MarkerAddSet        (int line, int markerNumber)            { return Cmd<int >(SCI_MARKERADDSET, line, markerNumber); }
		void MarkerDelete        (int line, int markerNumber)            { return Cmd<void>(SCI_MARKERDELETE, line, markerNumber); }
		void MarkerDeleteAll     (int markerNumber)                      { return Cmd<void>(SCI_MARKERDELETEALL, markerNumber, 0L); }
		int  MarkerGet           (int line) const                        { return Cmd<int >(SCI_MARKERGET, line, 0L); }
		int  MarkerNext          (int lineStart, int markerMask) const   { return Cmd<int >(SCI_MARKERNEXT, lineStart, markerMask); }
		int  MarkerPrevious      (int lineStart, int markerMask) const   { return Cmd<int >(SCI_MARKERPREVIOUS, lineStart, markerMask ); }
		int  MarkerLineFromHandle(int handle) const                      { return Cmd<int >(SCI_MARKERLINEFROMHANDLE, handle, 0L); }
		void MarkerDeleteHandle  (int handle)                            { return Cmd<void>(SCI_MARKERDELETEHANDLE, handle, 0L); }
		#pragma endregion

		#pragma region Indicators
		int      IndicGetStyle(int indic) const          { return Cmd<int     >(SCI_INDICGETSTYLE, indic, 0L); }
		void     IndicSetStyle(int indic, int style)     { return Cmd<void    >(SCI_INDICSETSTYLE, indic, style); }
		COLORREF IndicGetFore (int indic) const          { return Cmd<COLORREF>(SCI_INDICGETFORE, indic, 0L); }
		void     IndicSetFore (int indic, COLORREF fore) { return Cmd<void    >(SCI_INDICSETFORE, indic, fore); }
		#pragma endregion
	
		#pragma region Autocomplete
		void AutoCShow(int lenEntered, const char * itemList) { return Cmd <void >(SCI_AUTOCSHOW, lenEntered, itemList); }
		void AutoCCancel()                                    { return Cmd <void >(SCI_AUTOCCANCEL, 0, 0L); }
		bool AutoCActive() const                              { return Cmd <int >(SCI_AUTOCACTIVE, 0, 0L) != 0; }
		uint32_t AutoCPosStart() const                        { return Cmd <uint32_t >(SCI_AUTOCPOSSTART, 0, 0L); }
		void AutoCComplete()                                  { return Cmd <void >(SCI_AUTOCCOMPLETE, 0, 0L); }
		void AutoCStops(const char * characterSet)            { return Cmd <void >(SCI_AUTOCSTOPS, 0, characterSet); }
		int  AutoCGetSeparator() const                        { return Cmd <int >(SCI_AUTOCGETSEPARATOR, 0, 0L); }
		void AutoCSetSeparator(int separatorCharacter)        { return Cmd <void >(SCI_AUTOCSETSEPARATOR, separatorCharacter, 0L); }
		void AutoCSelect(const char * text)                   { return Cmd <void >(SCI_AUTOCSELECT, 0, text); }
		int  AutoCGetCurrent() const                          { return Cmd <int >(SCI_AUTOCGETCURRENT, 0, 0L); }
		bool AutoCGetCancelAtStart() const                    { return Cmd <int >(SCI_AUTOCGETCANCELATSTART, 0, 0L) != 0; }
		void AutoCSetCancelAtStart(bool cancel)               { return Cmd <void >(SCI_AUTOCSETCANCELATSTART, cancel, 0L); }
		void AutoCSetFillUps(const char * characterSet)       { return Cmd <void >(SCI_AUTOCSETFILLUPS, 0, characterSet); }
		bool AutoCGetChooseSingle() const                     { return Cmd <int >(SCI_AUTOCGETCHOOSESINGLE, 0, 0L) != 0; }
		void AutoCSetChooseSingle(bool chooseSingle)          { return Cmd <void >(SCI_AUTOCSETCHOOSESINGLE, chooseSingle, 0L); }
		bool AutoCGetIgnoreCase() const                       { return Cmd <int >(SCI_AUTOCGETIGNORECASE, 0, 0L) != 0; }
		void AutoCSetIgnoreCase(bool ignoreCase)              { return Cmd <void >(SCI_AUTOCSETIGNORECASE, ignoreCase, 0L); }
		bool AutoCGetAutoHide() const                         { return Cmd <int >(SCI_AUTOCGETAUTOHIDE, 0, 0L) != 0; }
		void AutoCSetAutoHide(bool autoHide)                  { return Cmd <void >(SCI_AUTOCSETAUTOHIDE, autoHide, 0L); }
		bool AutoCGetDropRestOfWord() const                   { return Cmd <int >(SCI_AUTOCGETDROPRESTOFWORD, 0, 0L) != 0; }
		void AutoCSetDropRestOfWord(bool dropRestOfWord)      { return Cmd <void >(SCI_AUTOCSETDROPRESTOFWORD, dropRestOfWord, 0L); }
		void RegisterImage(int type, const char * xpmData)    { return Cmd <void >(SCI_REGISTERIMAGE, type, xpmData); }
		void ClearRegisteredImages()                          { return Cmd <void >(SCI_CLEARREGISTEREDIMAGES, 0, 0L); }
		int  AutoCGetTypeSeparator() const                    { return Cmd <int >(SCI_AUTOCGETTYPESEPARATOR, 0, 0L); }
		void AutoCSetTypeSeparator(int separatorCharacter)    { return Cmd <void >(SCI_AUTOCSETTYPESEPARATOR, separatorCharacter,  0L); }
		int  AutoCGetMaxWidth() const                         { return Cmd <int >(SCI_AUTOCGETMAXWIDTH, 0, 0L); }
		void AutoCSetMaxWidth(int characterCount)             { return Cmd <void >(SCI_AUTOCSETMAXWIDTH, characterCount, 0L); }
		int  AutoCGetMaxHeight() const                        { return Cmd <int >(SCI_AUTOCGETMAXHEIGHT, 0, 0L); }
		void AutoCSetMaxHeight(int rowCount)                  { return Cmd <void >(SCI_AUTOCSETMAXHEIGHT, rowCount, 0L); }
		#pragma endregion

		#pragma region User Lists
		void UserListShow(int listType, const char * itemList) { Cmd<void>(SCI_USERLISTSHOW, listType, itemList); }
		#pragma endregion

		#pragma region Call tips
		void CallTipShow(uint32_t pos, const char * definition) { return Cmd <void>(SCI_CALLTIPSHOW, pos, definition); }
		void CallTipCancel()                                    { return Cmd <void>(SCI_CALLTIPCANCEL, 0, 0L); }
		bool CallTipActive()                                    { return Cmd <int >(SCI_CALLTIPACTIVE, 0, 0L) != 0; }
		uint32_t CallTipPosStart() const                        { return Cmd <uint32_t>(SCI_CALLTIPPOSSTART, 0, 0L); }
		void CallTipSetHlt(int start, int end)                  { return Cmd <void>(SCI_CALLTIPSETHLT, start, end); }
		void CallTipSetBack(COLORREF back)                      { return Cmd <void>(SCI_CALLTIPSETBACK, back, 0L); }
		void CallTipSetFore(COLORREF fore)                      { return Cmd <void>(SCI_CALLTIPSETFORE, fore, 0L); }
		void CallTipSetForeHlt(COLORREF fore)                   { return Cmd <void>(SCI_CALLTIPSETFOREHLT, fore, 0L); }
		#pragma endregion

		#pragma region Keyboard Commands
		void LineDown()                { return Cmd<void>(SCI_LINEDOWN, 0, 0L); }
		void LineDownExtend()          { return Cmd<void>(SCI_LINEDOWNEXTEND, 0, 0L); }
		void LineUp()                  { return Cmd<void>(SCI_LINEUP, 0, 0L); }
		void LineUpExtend()            { return Cmd<void>(SCI_LINEUPEXTEND, 0, 0L); }
		void LineDownRectExtend()      { return Cmd<void>(SCI_LINEDOWNRECTEXTEND, 0, 0L); }
		void LineUpRectExtend()        { return Cmd<void>(SCI_LINEUPRECTEXTEND, 0, 0L); }
		void LineScrollDown()          { return Cmd<void>(SCI_LINESCROLLDOWN, 0, 0L); }
		void LineScrollUp()            { return Cmd<void>(SCI_LINESCROLLUP, 0, 0L); }
		void ParaDown()                { return Cmd<void>(SCI_PARADOWN, 0, 0L); }
		void ParaDownExtend()          { return Cmd<void>(SCI_PARADOWNEXTEND, 0, 0L); }
		void ParaUp()                  { return Cmd<void>(SCI_PARAUP, 0, 0L); }
		void ParaUpExtend()            { return Cmd<void>(SCI_PARAUPEXTEND, 0, 0L); }
		void CharLeft()                { return Cmd<void>(SCI_CHARLEFT, 0, 0L); }
		void CharLeftExtend()          { return Cmd<void>(SCI_CHARLEFTEXTEND, 0, 0L); }
		void CharRight()               { return Cmd<void>(SCI_CHARRIGHT, 0, 0L); }
		void CharRightExtend()         { return Cmd<void>(SCI_CHARRIGHTEXTEND, 0, 0L); }
		void CharLeftRectExtend()      { return Cmd<void>(SCI_CHARLEFTRECTEXTEND, 0, 0L); }
		void CharRightRectExtend()     { return Cmd<void>(SCI_CHARRIGHTRECTEXTEND, 0, 0L); }
		void WordLeft()                { return Cmd<void>(SCI_WORDLEFT, 0, 0L); }
		void WordLeftExtend()          { return Cmd<void>(SCI_WORDLEFTEXTEND, 0, 0L); }
		void WordRight()               { return Cmd<void>(SCI_WORDRIGHT, 0, 0L); }
		void WordRightExtend()         { return Cmd<void>(SCI_WORDRIGHTEXTEND, 0, 0L); }
		void WordLeftEnd()             { return Cmd<void>(SCI_WORDLEFTEND, 0, 0L); }
		void WordLeftEndExtend()       { return Cmd<void>(SCI_WORDLEFTENDEXTEND, 0, 0L); }
		void WordRightEnd()            { return Cmd<void>(SCI_WORDRIGHTEND, 0, 0L); }
		void WordRightEndExtend()      { return Cmd<void>(SCI_WORDRIGHTENDEXTEND, 0, 0L); }
		void WordPartLeft()            { return Cmd<void>(SCI_WORDPARTLEFT, 0, 0L); }
		void WordPartLeftExtend()      { return Cmd<void>(SCI_WORDPARTLEFTEXTEND, 0, 0L); }
		void WordPartRight()           { return Cmd<void>(SCI_WORDPARTRIGHT, 0, 0L); }
		void WordPartRightExtend()     { return Cmd<void>(SCI_WORDPARTRIGHTEXTEND, 0, 0L); }
		void Home()                    { return Cmd<void>(SCI_HOME, 0, 0L); }
		void HomeExtend()              { return Cmd<void>(SCI_HOMEEXTEND, 0, 0L); }
		void HomeRectExtend()          { return Cmd<void>(SCI_HOMERECTEXTEND, 0, 0L); }
		void HomeDisplay()             { return Cmd<void>(SCI_HOMEDISPLAY, 0, 0L); }
		void HomeDisplayExtend()       { return Cmd<void>(SCI_HOMEDISPLAYEXTEND, 0, 0L); }
		void HomeWrap()                { return Cmd<void>(SCI_HOMEWRAP, 0, 0L); }
		void HomeWrapExtend()          { return Cmd<void>(SCI_HOMEWRAPEXTEND, 0, 0L); }
		void VCHome()                  { return Cmd<void>(SCI_VCHOME, 0, 0L); }
		void VCHomeExtend()            { return Cmd<void>(SCI_VCHOMEEXTEND, 0, 0L); }
		void VCHomeRectExtend()        { return Cmd<void>(SCI_VCHOMERECTEXTEND, 0, 0L); }
		void VCHomeWrap()              { return Cmd<void>(SCI_VCHOMEWRAP, 0, 0L); }
		void VCHomeWrapExtend()        { return Cmd<void>(SCI_VCHOMEWRAPEXTEND, 0, 0L); }
		void LineEnd()                 { return Cmd<void>(SCI_LINEEND, 0, 0L); }
		void LineEndExtend()           { return Cmd<void>(SCI_LINEENDEXTEND, 0, 0L); }
		void LineEndRectExtend()       { return Cmd<void>(SCI_LINEENDRECTEXTEND, 0, 0L); }
		void LineEndDisplay()          { return Cmd<void>(SCI_LINEENDDISPLAY, 0, 0L); }
		void LineEndDisplayExtend()    { return Cmd<void>(SCI_LINEENDDISPLAYEXTEND, 0, 0L); }
		void LineEndWrap()             { return Cmd<void>(SCI_LINEENDWRAP, 0, 0L); }
		void LineEndWrapExtend()       { return Cmd<void>(SCI_LINEENDWRAPEXTEND, 0, 0L); }
		void DocumentStart()           { return Cmd<void>(SCI_DOCUMENTSTART, 0, 0L); }
		void DocumentStartExtend()     { return Cmd<void>(SCI_DOCUMENTSTARTEXTEND, 0, 0L); }
		void DocumentEnd()             { return Cmd<void>(SCI_DOCUMENTEND, 0, 0L); }
		void DocumentEndExtend()       { return Cmd<void>(SCI_DOCUMENTENDEXTEND, 0, 0L); }
		void PageUp()                  { return Cmd<void>(SCI_PAGEUP, 0, 0L); }
		void PageUpExtend()            { return Cmd<void>(SCI_PAGEUPEXTEND, 0, 0L); }
		void PageUpRectExtend()        { return Cmd<void>(SCI_PAGEUPRECTEXTEND, 0, 0L); }
		void PageDown()                { return Cmd<void>(SCI_PAGEDOWN, 0, 0L); }
		void PageDownExtend()          { return Cmd<void>(SCI_PAGEDOWNEXTEND, 0, 0L); }
		void PageDownRectExtend()      { return Cmd<void>(SCI_PAGEDOWNRECTEXTEND, 0, 0L); }
		void StutteredPageUp()         { return Cmd<void>(SCI_STUTTEREDPAGEUP, 0, 0L); }
		void StutteredPageUpExtend()   { return Cmd<void>(SCI_STUTTEREDPAGEUPEXTEND, 0, 0L); }
		void StutteredPageDown()       { return Cmd<void>(SCI_STUTTEREDPAGEDOWN, 0, 0L); }
		void StutteredPageDownExtend() { return Cmd<void>(SCI_STUTTEREDPAGEDOWNEXTEND, 0, 0L); }
		void DeleteBack()              { return Cmd<void>(SCI_DELETEBACK, 0, 0L); }
		void DeleteBackNotLine()       { return Cmd<void>(SCI_DELETEBACKNOTLINE, 0, 0L); }
		void DelWordLeft()             { return Cmd<void>(SCI_DELWORDLEFT, 0, 0L); }
		void DelWordRight()            { return Cmd<void>(SCI_DELWORDRIGHT, 0, 0L); }
		void DelLineLeft()             { return Cmd<void>(SCI_DELLINELEFT, 0, 0L); }
		void DelLineRight()            { return Cmd<void>(SCI_DELLINERIGHT, 0, 0L); }
		void LineDelete()              { return Cmd<void>(SCI_LINEDELETE, 0, 0L); }
		void LineCut()                 { return Cmd<void>(SCI_LINECUT, 0, 0L); }
		void LineCopy()                { return Cmd<void>(SCI_LINECOPY, 0, 0L); }
		void LineTranspose()           { return Cmd<void>(SCI_LINETRANSPOSE, 0, 0L); }
		void LineDuplicate()           { return Cmd<void>(SCI_LINEDUPLICATE, 0, 0L); }
		void LowerCase()               { return Cmd<void>(SCI_LOWERCASE, 0, 0L); }
		void UpperCase()               { return Cmd<void>(SCI_UPPERCASE, 0, 0L); }
		void Cancel()                  { return Cmd<void>(SCI_CANCEL, 0, 0L); }
		void EditToggleOvertype()      { return Cmd<void>(SCI_EDITTOGGLEOVERTYPE, 0, 0L); }
		void NewLine()                 { return Cmd<void>(SCI_NEWLINE, 0, 0L); }
		void FormFeed()                { return Cmd<void>(SCI_FORMFEED, 0, 0L); }
		void Tab()                     { return Cmd<void>(SCI_TAB, 0, 0L); }
		void BackTab()                 { return Cmd<void>(SCI_BACKTAB, 0, 0L); }
		void SelectionDuplicate()      { return Cmd<void>(SCI_SELECTIONDUPLICATE, 0, 0L); }
		#pragma endregion

		#pragma region Key Bindings
		void AssignCmdKey(DWORD key, int command)         { return Cmd<void>(SCI_ASSIGNCMDKEY, key, command); }
		void AssignCmdKey(WORD vk, WORD mod, int command) { return Cmd<void>(SCI_ASSIGNCMDKEY, vk + (mod << 16), command); }
		void ClearCmdKey(DWORD key)                       { return Cmd<void>(SCI_CLEARCMDKEY, key, 0); }
		void ClearAllCmdKeys()                            { return Cmd<void>(SCI_CLEARALLCMDKEYS, 0, 0); }
		void Null()                                       { return Cmd<void>(SCI_NULL, 0, 0L); }
		#pragma endregion

		#pragma region Context Menu
		void UsePopUp(bool allowPopUp) { return Cmd<void>(SCI_USEPOPUP, allowPopUp, 0L); }
		#pragma endregion

		#pragma region Macro Recording
		void StartRecord() { return Cmd<void>(SCI_STARTRECORD, 0, 0L); }
		void StopRecord()  { return Cmd<void>(SCI_STOPRECORD, 0, 0L); }
		#pragma endregion

		#pragma region Printing
		uint32_t FormatRange          (bool draw, Sci_RangeToFormat& fr) { return Cmd<uint32_t>(SCI_FORMATRANGE, draw, &fr); }
		int  GetPrintMagnification() const                        { return Cmd<int >(SCI_GETPRINTMAGNIFICATION, 0, 0L); }
		void SetPrintMagnification(int magnification)             { return Cmd<void>(SCI_SETPRINTMAGNIFICATION, magnification, 0L); }
		int  GetPrintColourMode   () const                        { return Cmd<int >(SCI_GETPRINTCOLOURMODE, 0, 0L); }
		void SetPrintColourMode   (int mode)                      { return Cmd<void>(SCI_SETPRINTCOLOURMODE, mode, 0L); }
		int  GetPrintWrapMode     () const                        { return Cmd<int >(SCI_GETPRINTWRAPMODE, 0, 0L); }
		void SetPrintWrapMode     (int mode)                      { return Cmd<void>(SCI_SETPRINTWRAPMODE, mode, 0L); }
		#pragma endregion

		#pragma region Multiple Views
		void* GetDocPointer  () const         { return Cmd<void*>(SCI_GETDOCPOINTER, 0, 0L); }
		void  SetDocPointer  (void * pointer) { return Cmd<void >(SCI_SETDOCPOINTER, 0, pointer); }
		void* CreateDocument ()               { return Cmd<void*>(SCI_CREATEDOCUMENT, 0, 0L); }
		void  AddRefDocument (void * doc)     { return Cmd<void >(SCI_ADDREFDOCUMENT, 0, doc); }
		void  ReleaseDocument(void * doc)     { return Cmd<void >(SCI_RELEASEDOCUMENT, 0, doc); }
		#pragma endregion

		#pragma region Folding
		int  VisibleFromDocLine        (int line) const                 { return Cmd<int >(SCI_VISIBLEFROMDOCLINE, line, 0L); }
		int  DocLineFromVisible        (int lineDisplay) const          { return Cmd<int >(SCI_DOCLINEFROMVISIBLE, lineDisplay, 0L); }
		void ShowLines                 (int lineStart, int lineEnd)     { return Cmd<void>(SCI_SHOWLINES, lineStart, lineEnd); }
		void HideLines                 (int lineStart, int lineEnd)     { return Cmd<void>(SCI_HIDELINES, lineStart, lineEnd); }
		bool GetLineVisible            (int line) const                 { return Cmd<int >(SCI_GETLINEVISIBLE, line, 0L) != 0; }
		int  FoldLevel                 (int line) const                 { return Cmd<int >(SCI_GETFOLDLEVEL, line, 0L); }
		void FoldLevel                 (int line, int level)            { return Cmd<void>(SCI_SETFOLDLEVEL, line, level); }
		void FoldFlags                 (int flags)                      { return Cmd<void>(SCI_SETFOLDFLAGS, flags, 0L); }
		int  GetLastChild              (int line, int level) const      { return Cmd<int >(SCI_GETLASTCHILD, line, level); }
		int  GetFoldParent             (int line) const                 { return Cmd<int >(SCI_GETFOLDPARENT, line, 0L); }
		bool FoldExpanded              (int line) const                 { return Cmd<int >(SCI_GETFOLDEXPANDED, line, 0L) != 0; }
		void FoldExpanded              (int line, bool expanded)        { return Cmd<void>(SCI_SETFOLDEXPANDED, line, expanded); }
		void ToggleFold                (int line)                       { return Cmd<void>(SCI_TOGGLEFOLD, line, 0L); }
		void EnsureVisible             (int line)                       { return Cmd<void>(SCI_ENSUREVISIBLE, line, 0L); }
		void EnsureVisibleEnforcePolicy(int line)                       { return Cmd<void>(SCI_ENSUREVISIBLEENFORCEPOLICY, line, 0L); }
		void SetFoldMarginColour       (bool useSetting, COLORREF back) { return Cmd<void>(SCI_SETFOLDMARGINCOLOUR, useSetting, back); }
		void SetFoldMarginHiColour     (bool useSetting, COLORREF fore) { return Cmd<void>(SCI_SETFOLDMARGINHICOLOUR, useSetting, fore); }
		#pragma endregion

		#pragma region Line Wrapping
		int  WrapMode               () const                      { return Cmd<int >(SCI_GETWRAPMODE, 0, 0L); }
		void WrapMode               (int mode)                    { return Cmd<void>(SCI_SETWRAPMODE, mode, 0L); }
		int  WrapVisualFlags        () const                      { return Cmd<int >(SCI_GETWRAPVISUALFLAGS, 0, 0L); }
		void WrapVisualFlags        (int wrapVisualFlags)         { return Cmd<void>(SCI_SETWRAPVISUALFLAGS, wrapVisualFlags, 0L); }
		int  WrapVisualFlagsLocation() const                      { return Cmd<int >(SCI_GETWRAPVISUALFLAGSLOCATION, 0, 0L); }
		void WrapVisualFlagsLocation(int wrapVisualFlagsLocation) { return Cmd<void>(SCI_SETWRAPVISUALFLAGSLOCATION, wrapVisualFlagsLocation, 0L); }
		int  WrapStartIndent        () const                      { return Cmd<int >(SCI_GETWRAPSTARTINDENT, 0, 0L); }
		void WrapStartIndent        (int indent)                  { return Cmd<void>(SCI_SETWRAPSTARTINDENT, indent, 0L); }
		int  LayoutCache            () const                      { return Cmd<int >(SCI_GETLAYOUTCACHE, 0, 0L); }
		void LayoutCache            (int mode)                    { return Cmd<void>(SCI_SETLAYOUTCACHE, mode, 0L); }
		void LinesSplit             (int pixelWidth)              { return Cmd<void>(SCI_LINESSPLIT, pixelWidth, 0L); }
		void LinesJoin              ()                            { return Cmd<void>(SCI_LINESJOIN, 0, 0L); }
		int  WrapCount              (int line) const              { return Cmd<int >(SCI_WRAPCOUNT, line, 0L); }
		#pragma endregion

		#pragma region Zooming
		void ZoomIn ()      { return Cmd<void>(SCI_ZOOMIN, 0, 0L); }
		void ZoomOut()      { return Cmd<void>(SCI_ZOOMOUT, 0, 0L); }
		int  Zoom() const   { return Cmd<int >(SCI_GETZOOM, 0, 0L); }
		void Zoom(int zoom) { return Cmd<void>(SCI_SETZOOM, zoom, 0L); }
		#pragma endregion

		#pragma region Long Lines
		int      EdgeMode  () const              { return Cmd<int     >(SCI_GETEDGEMODE, 0, 0L); }
		void     EdgeMode  (int mode)            { return Cmd<void    >(SCI_SETEDGEMODE, mode, 0L); }
		int      EdgeColumn() const              { return Cmd<int     >(SCI_GETEDGECOLUMN, 0, 0L); }
		void     EdgeColumn(int column)          { return Cmd<void    >(SCI_SETEDGECOLUMN, column, 0L); }
		COLORREF EdgeColour() const              { return Cmd<COLORREF>(SCI_GETEDGECOLOUR, 0, 0L); }
		void     EdgeColour(COLORREF edgeColour) { return Cmd<void    >(SCI_SETEDGECOLOUR, edgeColour, 0L); }
		#pragma endregion

		#pragma region Lexer
		int  Lexer              () const                                { return Cmd<int >(SCI_GETLEXER, 0, 0L); }
		void Lexer              (int lexer)                             { return Cmd<void>(SCI_SETLEXER, lexer, 0L); }
		void LexerLanguage      (const char * language)                 { return Cmd<void>(SCI_SETLEXERLANGUAGE, 0, language); }
		//void LoadLexerLibrary   (const char * path)                     { return Cmd<void>(SCI_LOADLEXERLIBRARY, 0, path); }
		void Colourise          (uint32_t start, uint32_t end)          { return Cmd<void>(SCI_COLOURISE, start, end); }
		int  Property           (const char * key, char * buf) const    { return Cmd<int >(SCI_GETPROPERTY, key, buf ); }
		void Property           (const char * key, const char * value)  { return Cmd<void>(SCI_SETPROPERTY, key, value); }
		int  GetPropertyExpanded(const char * key, char * buf) const    { return Cmd<int >(SCI_GETPROPERTYEXPANDED, key, buf); }
		int  GetPropertyInt     (const char * key) const                { return Cmd<int >(SCI_GETPROPERTYINT, key, 0L); }
		void SetKeyWords        (int keywordSet, const char * keyWords) { return Cmd<void>(SCI_SETKEYWORDS, keywordSet, keyWords); }
		#pragma endregion

		#pragma region Notifications
		int  ModEventMask  () const                 { return Cmd<int >(SCI_GETMODEVENTMASK, 0, 0L); }
		void ModEventMask  (int mask)               { return Cmd<void>(SCI_SETMODEVENTMASK, mask, 0L); }
		int  MouseDwellTime() const                 { return Cmd<int >(SCI_GETMOUSEDWELLTIME, 0, 0L); }
		void MouseDwellTime(int periodMilliseconds) { return Cmd<void>(SCI_SETMOUSEDWELLTIME, periodMilliseconds, 0L); }
		#pragma endregion

		#pragma region Misc
		void Allocate          (int bytes)               { return Cmd<void>(SCI_ALLOCATE, bytes, 0L); }
		void SetSavePoint      ()                        { return Cmd<void>(SCI_SETTEXT, 0, 0L); }
		bool BufferedDraw      () const                  { return Cmd<int >(SCI_GETBUFFEREDDRAW, 0, 0) != 0; }
		void BufferedDraw      (bool buffered)           { return Cmd<void>(SCI_SETBUFFEREDDRAW, buffered, 0L); }
		int  CodePage          () const                  { return Cmd<int >(SCI_GETCODEPAGE, 0, 0L); }
		void CodePage          (int codePage)            { return Cmd<void>(SCI_SETCODEPAGE, codePage, 0L); }
		void SetWordChars      (const char * characters) { return Cmd<void>(SCI_SETWORDCHARS, 0, characters); }
		void SetWhitespaceChars(const char * characters) { return Cmd<void>(SCI_SETWHITESPACECHARS, 0, characters); }
		void SetCharsDefault   ()                        { return Cmd<void>(SCI_SETCHARSDEFAULT, 0, 0L); }
		void GrabFocus         ()                        { return Cmd<void>(SCI_GRABFOCUS, 0, 0L); }
		bool Focus             () const                  { return Cmd<int >(SCI_GETFOCUS, 0, 0L) != 0; }
		void Focus             (bool focus)              { return Cmd<void>(SCI_SETFOCUS, focus, 0L); }
		bool ReadOnly() const         { return Cmd<int >(SCI_GETREADONLY, 0, 0L) != 0; }
		void ReadOnly(bool readOnly)  { return Cmd<void>(SCI_SETREADONLY, readOnly, 0L); }
		#pragma endregion

		#pragma region Status/Errors
		int  Status  () const         { return Cmd<int >(SCI_GETSTATUS, 0, 0L); }
		void Status  (int statusCode) { return Cmd<void>(SCI_SETSTATUS, statusCode, 0L); }
		#pragma endregion

	private:

		LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override
		{
			switch (message)
			{
				case WM_CREATE:
				{
					m_snd = SciFnDirect(::SendMessageW(m_hwnd, SCI_GETDIRECTFUNCTION, 0, 0));
					m_ptr = sptr_t(::SendMessageW(m_hwnd, SCI_GETDIRECTPOINTER, 0, 0));
					break;
				}
			}
			return Control::WndProc(message, wparam, lparam);
		}
	};
}

#if 0
#if defined(__ATLSTR_H__)
		int GetCurLine(CStringA& text)
		{
			PR_ISWND;
			int length=GetCurLine(0,0);
			int result=GetCurLine(text.GetBuffer(length),length);
			text.ReleaseBuffer(length-1);
			return result;
		}
	#endif

	#if defined(__ATLSTR_H__)
		int GetLine(int line,CStringA& c)
		{
			PR_ISWND;
			int length=GetLine(line,0);
			int result=GetLine(line,c.GetBuffer(length));
			c.ReleaseBuffer(length);
			return result;
		}
		int GetSimpleLine(int line,CStringA& c)
		{
			PR_ISWND;
			Sci_TextRange range;
			range.chrg.cpMin=PositionFromLine(line);
			range.chrg.cpMax=GetLineEndPosition(line);
			int length=range.chrg.cpMax-range.chrg.cpMin;
			range.lpstrText=c.GetBuffer(length);
			int result=GetTextRange(range);
			c.ReleaseBuffer(length);
			return result;
		}
	#endif

	#if defined(__ATLSTR_H__)
		int GetSelText(CStringA& text)
		{
			PR_ISWND;
			int length=GetSelText(0)-1;
			int result=GetSelText(text.GetBuffer(length));
			text.ReleaseBuffer(length);
			return result;
		}
	#endif



	#if defined(GTK)
		int TargetAsUTF8(char* s)
		{
			PR_ISWND;
			return ::SndMsg<>(m_hWnd,SCI_TARGETASUTF8,0,(int)s);
		}
		void SetLengthForEncode(int bytes)
		{
			PR_ISWND;
			::SndMsg<>(m_hWnd,SCI_SETLENGTHFORENCODE,bytes);
		}
		int EncodedFromUTF8(const char* utf8,char* encoded)
		{
			PR_ISWND;
			return ::SndMsg<>(m_hWnd,SCI_ENCODEDFROMUTF8,(int)utf8,(int)encoded);
		}
	#endif

	#if defined(__ATLSTR_H__)
		int GetProperty(const char* key,CStringA& buf)
		{
			PR_ISWND;
			int length=GetProperty(key,0);
			int result=GetProperty(key,buf.GetBuffer(length));
			buf.ReleaseBuffer(length);
			return result;
		}
	#endif

	#if defined(__ATLSTR_H__)
		int GetPropertyExpanded(const char* key,CStringA& buf)
		{
			PR_ISWND;
			int length=GetPropertyExpanded(key,0);
			int result=GetPropertyExpanded(key,buf.GetBuffer(length));
			buf.ReleaseBuffer(length);
			return result;
		}
	#endif
#endif