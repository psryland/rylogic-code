//***************************************************************************************************
// Ldr Script Editor Dialog
//  Copyright (c) Rylogic Ltd 2014
//***************************************************************************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/gui/wingui.h"
#include "pr/gui/scintilla_ctrl.h"

namespace pr::rdr12::ldraw
{
	// Script editor interface
	struct ScriptEditorUI :gui::Form
	{
		gui::ScintillaCtrl m_edit;
		gui::Button m_btn_render;
		gui::Button m_btn_close;

		enum
		{
			IDC_TEXT = 1000, IDC_BTN_RENDER, IDC_BTN_CLOSE,
			ID_LOAD, ID_SAVE, ID_CLOSE,
			ID_UNDO, ID_REDO, ID_CUT, ID_COPY, ID_PASTE,
		};

		// This code expects the scintilla.dll to be loaded already
		ScriptEditorUI(HWND parent)
			:Form(Params<>()
				.name("ldr-script-editor").title(L"Script Editor").wh(430, 380).start_pos(EStartPosition::CentreParent)
				.menu({{L"&File", gui::Menu(gui::Menu::EKind::Popup, {{L"&Load", ID_LOAD}, {L"&Save", ID_SAVE}, {gui::MenuItem::Separator}, {L"&Close", IDCANCEL}})}})
				.icon_bg((HICON)::SendMessageW(parent, WM_GETICON, ICON_BIG, 0))
				.icon_sm((HICON)::SendMessageW(parent, WM_GETICON, ICON_SMALL, 0))
				.parent(parent).hide_on_close(true).pin_window(true).visible(false)
				.wndclass(RegisterWndClass<ScriptEditorUI>()))
			,m_edit      (gui::ScintillaCtrl::Params<>().parent(this_).name("edit"      ).wh(Fill,Fill).margin(3,3,3,32).anchor(EAnchor::All))
			,m_btn_close (gui::Button       ::Params<>().parent(this_).name("btn-close" ).xy(-1, -1).text(L"&Close"      ).anchor(EAnchor::BottomRight))
			,m_btn_render(gui::Button       ::Params<>().parent(this_).name("btn-render").xy(+1, -1).text(L"&Render (F5)").anchor(EAnchor::BottomLeft))
		{
			CreateHandle();

			// Initialise the edit control
			// Note: don't grab input focus until the editor is actually visible
			m_edit.InitLdrStyle();
			m_edit.SetSel(-1, 0);
			m_edit.Key += [this](Control&, gui::KeyEventArgs const& args)
			{
				if (!args.m_down) return;
				if (args.m_vk_key == VK_F5)
					m_btn_render.OnClick();
			};

			// Hook up button handlers
			m_btn_render.Click += [this](gui::Button&, gui::EmptyArgs const&)
			{
				auto text = m_edit.Text();
				Render(*this, pr::Widen(text));
			};
			m_btn_close.Click += [this](gui::Button&, gui::EmptyArgs const&)
			{
				Close(EDialogResult::Close);
			};
		}

		// An event raised when the render button is clicked
		gui::EventHandler<ScriptEditorUI&, std::wstring const&> Render;

		// Get/Set the utf-8 text in the dialog
		virtual std::string Text() const
		{
			return m_edit.Text();
		}
		virtual void Text(char const* utf8)
		{
			m_edit.Text(utf8);
		}

	protected:

		// Message handler
		bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
		{
			using namespace pr::gui;

			switch (message)
			{
			case WM_COMMAND:
				#pragma region
				{
					auto id = LoWord(wparam);
					switch (id)
					{
					case IDCANCEL:
						{
							Form::Close(EDialogResult::Cancel);
							return true;
						}
					case ID_LOAD:
						{
							COMDLG_FILTERSPEC filters[] = {{L"Ldr Script (*.ldr)",L"*.ldr"}};//,{L"All Files",L"*.*"}};
							auto filename = OpenFileUI(nullptr, FileUIOptions(L"ldr", &filters[0], _countof(filters)));
							if (!filename.empty())
							{
								std::ifstream infile(filename[0]);
								if (!infile) MessageBoxW(m_hwnd, L"Failed to open file", L"Load Failed", MB_OK|MB_ICONERROR);
								else m_edit.Load(infile);
							}
							return true;
						}
					case ID_SAVE:
						{
							COMDLG_FILTERSPEC filters[] = {{L"Ldr Script (*.ldr)",L"*.ldr"}};//,{L"All Files",L"*.*"}};
							auto filename = SaveFileUI(m_hwnd, FileUIOptions(L"ldr", &filters[0], _countof(filters)));
							if (!filename.empty())
							{
								std::ofstream outfile(filename);
								if (!outfile) MessageBoxW(m_hwnd, L"Failed to open file for writing", L"Save Failed", MB_OK|MB_ICONERROR);
								else m_edit.Save(outfile);
							}
							return true;
						}
					}
					break;
				}
				#pragma endregion
			}
			return Form::ProcessWindowMessage(hwnd, message, wparam, lparam, result);
		}
	};
}

// Add a manifest dependency on common controls version 6
#if defined _M_IX86
#   pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_IA64
#   pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='ia64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#   pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#   pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif