//*********************************************
// Lua Console MFC
//	(C)opyright Rylogic Limited 2007
//*********************************************

#ifndef PR_MFC_LUA_CONSOLE_H
#define PR_MFC_LUA_CONSOLE_H

#include <afxwin.h>         // MFC core and standard components
#include <afxcmn.h>			// MFC support for Windows Common Controls
#include "pr/lua/Lua.h"
#include "pr/GUI/CodeEdit.h"
#include "pr/GUI/SplitterCtrl.h"
#include "pr/GUI/LuaConsoleResource.h"

namespace pr
{
	class LuaConsole;
	struct CLuaInputEdit : CCodeEdit
	{
		LuaConsole* m_parent;
		CLuaInputEdit(LuaConsole* parent);

		DECLARE_MESSAGE_MAP()
		afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	};

	// LuaConsole dialog
	class LuaConsole : public CDialog
	{
		DECLARE_DYNAMIC(LuaConsole)
	public:
		enum { IDD = IDD_DIALOG_LUA_CONSOLE };
		LuaConsole(lua::Lua& lua, CWnd* pParent = NULL);
		virtual ~LuaConsole();

		virtual void Create(CWnd* parent);
		virtual BOOL OnInitDialog();
		virtual void OnOK() {}

		void DoString(CString const& str, CString& syntax_error_msg);

		// Lua registered functions
		int LuaPrint(lua_State* lua_state);

	protected:
		friend struct CLuaInputEdit;
		virtual void DoDataExchange(CDataExchange* pDX);
		DECLARE_MESSAGE_MAP()
		afx_msg void OnGetMinMaxInfo(MINMAXINFO* lpMMI);
		afx_msg void OnSize(UINT nType, int cx, int cy);

		lua::Lua*		m_lua;
		pr::SplitterCtrl	m_splitter;
		CRichEditCtrl		m_output;
		CLuaInputEdit		m_input;
	};
}//namespace pr

#endif//PR_MFC_LUA_CONSOLE_H