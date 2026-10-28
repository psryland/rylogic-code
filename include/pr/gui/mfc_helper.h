//******************************************************************
//
//	Helper objects for MFC
//
//******************************************************************
#ifndef PR_MFC_HELPER_H
#define PR_MFC_HELPER_H

namespace pr
{
	// Return a sub menu by address
	// Use: HMENU menu = GetMenuByName(GetMenu(), "&File/&Recent Files");
	// Returns 0 if the sub menu isn't found
	inline HMENU GetMenuByName(HMENU root, TCHAR const* address)
	{
		// Find '/' in 'address'
		TCHAR const* end = address;
		for (; *end && *end != L'/'; ++end) {}
		if (end == address) return 0;

		// Look for the first part of the address in the items of 'root'
		for (UINT i = 0, i_end = ::GetMenuItemCount(root); i != i_end; ++i)
		{
			// Get the menu item name and length
			// Check the item name matches the first part of the address
			TCHAR item_name[256];
			int item_name_len = GetMenuString(root, i, item_name, 256, MF_BYPOSITION);
			if (item_name_len != end - address || _tcsncmp(address, item_name, item_name_len) != 0)
				continue;

			// If this is the last part of the address, then return the submenu
			HMENU sub_menu = GetSubMenu(root, i);
			if (*end == 0 || sub_menu == 0)
				return sub_menu;
			else
				return GetMenuByName(sub_menu, end + 1);
		}
		return 0;
	}

	//*****
	// Return a HMENU by name.
	// Usage:
	//	GetCMenuByName("&File,&Recent Files");
	namespace impl
	{
		template <typename T>
		HMENU GetMenuByName(HWND window_handle, const char* name_string)
		{
			// Get the top level menu
			HMENU menu = GetMenu(window_handle);
			if( !menu ) return 0;

			const char* start = name_string;
			const char* end	  = strstr(start, ",");

			uint32_t sm = 0, num_submenus = GetMenuItemCount(menu);
			while( sm < num_submenus )
			{
				// Get the name of the sub menu	
				char item_name[256];
				GetMenuString(menu, sm, item_name, 256, MF_BYPOSITION);

				// Match the menu item name to start-end
				const char* where = strstr(start, item_name);
				if( where && item_name[0] != '\0' )
				{
					// If this is the last part of the menu address return the HMENU
					if( end == 0 )
					{
						PR_ASSERT(PR_DBG, strcmp(start, item_name) == 0);
						return GetSubMenu(menu, sm);
					}
					// otherwise go to the next part of the address
					else if( where < end )
					{
						start = end + 1;
						end   = strstr(start, ",");

						// Get the sub menu
						menu = GetSubMenu(menu, sm);	PR_ASSERT(PR_DBG, menu);

						sm = 0;
						num_submenus = GetMenuItemCount(menu);
						continue;
					}
				}
				++sm;
			}

			return 0;
		}
	}//namespace impl

	inline HMENU GetMenuByName(HWND window_handle, const char* name_string)	{ return impl::GetMenuByName<void>(window_handle, name_string); }


	//*****
	// A class that prevents leaked GDI objects
	// Usage:
	//	void CMyDlg::OnPaint()
	//	{
	//		CPaintDC dc(this);
	//		CFont f; f.CreateFont(...);
	//		// Create everything out here
	//		{ // save context
	//			CSaveDC sdc(dc);
	//			dc.SelectObject(&f);
	//			dc.TextOut(...); // whatever...
	//		} // save context
	//	}// destructors called here...
	//
	class CSaveDC
	{
	public:
		CSaveDC(CDC& dc)   { m_sdc = &dc; m_saved = dc.SaveDC();  }
		CSaveDC(CDC* dc)   { m_sdc = dc;  m_saved = dc->SaveDC(); }
		virtual ~CSaveDC() { m_sdc->RestoreDC(m_saved);           }
	protected:
		CDC*	m_sdc;
		int		m_saved;
	};

	//*****
	// A memory DC class
	// Usage:
	//
	class CMemDC : public CDC
	{
	public:
		CMemDC(CWnd* parent) : m_parent(parent)
		{
			PR_ASSERT(PR_DBG, parent);
			m_parent->GetClientRect(&m_client_rect);
			m_parent->GetWindowRect(&m_window_rect);

			// Get the target so we can create the mem dc
			m_target_dc  = m_parent->BeginPaint(&m_paint_struct);
			m_mem_dc     .CreateCompatibleDC(m_target_dc);
			m_bitmap     .CreateCompatibleBitmap(m_target_dc, m_client_rect.Width(), m_client_rect.Height());
			m_old_bitmap = m_mem_dc.SelectObject(&m_bitmap);
			m_parent     ->EndPaint(&m_paint_struct);

			Attach(m_mem_dc);		//  Attach self to the memory buffer
		}
		~CMemDC()
		{
			m_mem_dc.SelectObject(m_old_bitmap);
			Detach();				//  Detach self from the buffer
		}
		void	Paint()
		{
			m_target_dc  = m_parent->BeginPaint(&m_paint_struct);
			CRect rect(m_paint_struct.rcPaint);
			m_target_dc->BitBlt(rect.left, rect.top, rect.Width(), rect.Height(), &m_mem_dc, rect.left, rect.top, SRCCOPY);
			m_parent->EndPaint(&m_paint_struct);
		}

		CRect	m_client_rect;
		CRect	m_window_rect;

	//    CRect UpdateRect() const	{ return m_paint_struct.rcPaint; }
		operator HDC() const		{ return m_mem_dc.m_hDC; }       //  DC handle for API functions

	private:	// Don't allow these
		CMemDC() {}
		CMemDC(const CMemDC&) = delete;
		CMemDC& operator = (const CMemDC&) = delete;

	protected: // Shouldn't be called directly
		BOOL	Attach(HDC hDC)	{ return CDC::Attach(hDC); }
		HDC		Detach()		{ return CDC::Detach();    }

	private:
		CWnd*		m_parent;			//  Pointer to the parent window
		CDC*		m_target_dc;		//  Pointer to target DC
		PAINTSTRUCT m_paint_struct;		//  Paint struct for BeginPaint()/EndPaint() pair
		CDC			m_mem_dc;
		CBitmap		m_bitmap;
		CBitmap*	m_old_bitmap;
	};
}//namespace pr

#endif//PR_MFC_HELPER_H
