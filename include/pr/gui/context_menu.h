//*****************************************************************************
// Context Menu
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
// Usage:
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "pr/gui/gdiplus.h"
#include "pr/gui/wingui.h"
#include "pr/common/fmt.h"

namespace pr
{
	namespace gui
	{
		// Forward
		struct ContextMenu;
		struct ContextMenuStyle;
		struct IContextMenuItem;
		using StylePtr  = std::shared_ptr<ContextMenuStyle>;
		using BitmapPtr = std::shared_ptr<gdi::Bitmap>;
		using FontPtr   = std::shared_ptr<gdi::Font>;

		enum EMenuItemState
		{
			Normal   = 0           ,
			Selected = ODS_SELECTED,
			Grayed   = ODS_GRAYED  ,
			Disabled = ODS_DISABLED,
			Checked  = ODS_CHECKED ,
			Focus    = ODS_FOCUS   ,
		};

		#pragma region HelperFunctions
		namespace impl
		{
			inline gdi::Color ColAdj(gdi::Color col, int dr, int dg, int db)
			{
				int r = col.GetR() + dr; if (r>255) r=255; else if (r<0) r=0;
				int g = col.GetG() + dg; if (g>255) g=255; else if (g<0) g=0;
				int b = col.GetB() + db; if (b>255) b=255; else if (b<0) b=0;
				return gdi::Color(col.GetA(), BYTE(r), BYTE(g), BYTE(b));
			}
			inline gdi::Rect Inflate(gdi::Rect rect, int dx, int dy)
			{
				rect.X -= dx;
				rect.Y -= dy;
				rect.Width += dx*2;
				rect.Height += dy*2;
				return rect;
			}
		}
		#pragma endregion

		#pragma region Style
		// Style object used to give menu items individual styles
		struct ContextMenuStyle
		{
			struct ColGrp
			{
				gdi::Color m_text; // The text colour for an item
				gdi::Color m_bkgd; // The background colour for an item
				gdi::Color m_brdr; // Border colour for an item
				ColGrp(gdi::Color text, gdi::Color bkgd, gdi::Color brdr)
					:m_text(text)
					,m_bkgd(bkgd)
					,m_brdr(brdr)
				{}
				void set(gdi::Color text, gdi::Color bkgd, gdi::Color brdr)
				{
					m_text = text;
					m_bkgd = bkgd;
					m_brdr = brdr;
				}
			};

			NonClientMetrics m_metrics;     // system metrics
			FontPtr          m_font_text;   // The font to display the text in
			FontPtr          m_font_marks;  // The font containing glyphs
			ColGrp           m_col_norm;    // Colours for a normal state menu item
			ColGrp           m_col_select;  // Colours for a selected menu item
			ColGrp           m_col_disable; // Colours for a disabled menu item
			int              m_margin_left; // The space to allow for bitmaps, check marks, etc
			int              m_text_margin; // The margin surrounding text in the item
			int              m_bmp_margin;  // The margin surrounding the bitmap in the item

			ContextMenuStyle()
				:m_metrics     ()
				,m_font_text   (std::make_shared<gdi::Font>(::GetDC(0), &m_metrics.lfMenuFont))
				,m_font_marks  (std::make_shared<gdi::Font>(L"Marlett", m_font_text->GetSize()))
				,m_col_norm    (To<gdi::Color>(::GetSysColor(COLOR_MENUTEXT)) , To<gdi::Color>(::GetSysColor(COLOR_MENU)) , gdi::Color())
				,m_col_select  (To<gdi::Color>(::GetSysColor(COLOR_MENUTEXT)) , gdi::Color(0xFF,0xD1,0xE2,0xF2)           , gdi::Color(0xFF,0x78,0xAE,0xE5))
				,m_col_disable (To<gdi::Color>(::GetSysColor(COLOR_GRAYTEXT)) , To<gdi::Color>(::GetSysColor(COLOR_MENU)) , gdi::Color())
				,m_margin_left (20)
				,m_text_margin (2)
				,m_bmp_margin  (1)
			{}

			// XP style menu highlighting
			static StylePtr WinXP()
			{
				auto sty = std::make_shared<ContextMenuStyle>();
				sty->m_col_norm   .set(To<gdi::Color>(::GetSysColor(COLOR_MENUTEXT))      , To<gdi::Color>(::GetSysColor(COLOR_MENU))        , gdi::Color());
				sty->m_col_select .set(To<gdi::Color>(::GetSysColor(COLOR_HIGHLIGHTTEXT)) , To<gdi::Color>(::GetSysColor(COLOR_MENUHILIGHT)) , gdi::Color());
				sty->m_col_disable.set(To<gdi::Color>(::GetSysColor(COLOR_GRAYTEXT))      , To<gdi::Color>(::GetSysColor(COLOR_MENU))        , gdi::Color());
				return sty;
			}

			// Return the colour pair for the given item state
			ColGrp Col(EMenuItemState item_state) const
			{
				bool selected = (item_state & EMenuItemState::Selected) != 0;
				bool disabled = (item_state & EMenuItemState::Disabled) != 0;
				if (disabled) return m_col_disable;
				if (selected) return m_col_select;
				return m_col_norm;
			}
		};
		#pragma endregion

		// Menu item base class
		struct ContextMenuItem
		{
			int              m_id;     // Menu item id
			ContextMenuItem* m_menu;   // The containing menu if not null
			EMenuItemState   m_state;  // Menu item state
			StylePtr         m_style;  // Item specific style if not null
			BitmapPtr        m_bitmap; // A bitmap to draw next to the item (null means no bitmap)
			Size             m_size;   // The size if this item within the containing menu

			// 'id' is the menu item id
			// 'parent' is the containing menu item that properties are inherited from
			ContextMenuItem(int id = -1, ContextMenuItem* menu = nullptr, EMenuItemState state = EMenuItemState::Normal, StylePtr style = nullptr, BitmapPtr bm = nullptr)
				:m_id(id)
				,m_menu(menu)
				,m_state(state)
				,m_style(style)
				,m_bitmap(bm)
				,m_size()
			{}
			virtual ~ContextMenuItem()
			{}

			// Called when the containing menu's HWND is created to allow menu items to create hosted controls
			virtual void CreateHostedControls()
			{}

			//// The window acting as the parent for hosted controls
			//virtual HWND CtrlHost() const
			//{
			//	assert(m_menu != nullptr && "Item is not contained within a ContextMenu"); // ContextMenu overrides this method
			//	return m_menu->CtrlHost();
			//}

			// Return the style to use for a context menu item
			virtual ContextMenuStyle const& Style() const
			{
				if (m_style) return *m_style;
				assert(m_menu != nullptr && "Item is not contained within a ContextMenu"); // ContextMenu overrides this method
				return m_menu->Style();
			}

			// True if the menu item can be selected
			virtual bool Selectable() const
			{
				return true;
			}

			// True if the given key matches the hot-key prefix for this item
			virtual bool HotkeyPrefix(wchar_t) const
			{
				return false;
			}

			// Get/Set whether this item has the given item state
			bool ItemState(EMenuItemState state) const
			{
				return (m_state & state) != 0;
			}
			void ItemState(EMenuItemState state, bool on)
			{
				if (on) m_state = EMenuItemState(m_state |  state);
				else    m_state = EMenuItemState(m_state & ~state);

				// Ensure states are consistent
				if ((m_state & EMenuItemState::Disabled) != 0)
				{
					m_state = EMenuItemState(m_state & ~EMenuItemState::Selected);
				}
			}

			virtual Size MeasureItem(gdi::Graphics& gfx) = 0;
			virtual void DrawItem   (gdi::Graphics& gfx, Rect const& rect) = 0;
		};

		// Context menu
		// Note: this class requires GdiPlus, remember to instantiate an instance
		// of 'pr::GdiPlus' somewhere so that the GDI+ dll is loaded
		struct ContextMenu :Form ,ContextMenuItem
		{
			using BForm = Form;
			using BMenu = ContextMenuItem;

			#pragma region Draw Helpers

			// Helper methods for drawing context menu items
			struct Draw
			{
				// Notes:
				// For GUI rendering, turn off antialiasing and smoothing
				// FillRectangle is [inclusive,exclusive)
				// DrawRectangle is [inclusive,inclusive]

				static Rect MeasureText(gdi::Graphics const& gfx, std::wstring const& text, gdi::Font const& font)
				{
					gdi::StringFormat fmt; fmt.SetHotkeyPrefix(Gdiplus::HotkeyPrefixShow);
					gdi::RectF text_sz; gfx.MeasureString(text.c_str(), int(text.size()), &font, gdi::PointF(), &fmt, &text_sz);
					text_sz.Height += 1.0f; text_sz.Width += 1.0f; // Round up
					return To<RECT>(text_sz);
				}
				static Rect MeasureBitmap(BitmapPtr const& bm, int width)
				{
					// Measure the size of a bitmap rescaled to fit width-wise within 'width'
					if (!bm) return Rect();
					Rect rect(0, 0, bm->GetWidth(), bm->GetHeight());
					rect.height(rect.height() * width / rect.width());
					rect.width(width);
					return rect;
				}
				static void Bkgd(gdi::Graphics& gfx, Rect const& rect, EMenuItemState item_state, ContextMenuStyle const& style)
				{
					auto col = style.Col(item_state);
					gdi::SolidBrush bsh_bkgd(col.m_bkgd);
					int x = rect.left + style.m_margin_left;

					gfx.FillRectangle(&bsh_bkgd, To<gdi::Rect>(rect));
					if ((item_state & EMenuItemState::Selected) == 0)
					{
						gdi::Pen pen_3dhi(impl::ColAdj(col.m_bkgd, 10, 10, 10));
						gdi::Pen pen_3dlo(impl::ColAdj(col.m_bkgd,-10,-10,-10));
						gfx.DrawLine(&pen_3dlo, x+0, rect.top, x+0, rect.bottom - 1);
						gfx.DrawLine(&pen_3dhi, x+1, rect.top, x+1, rect.bottom - 1);
					}
					else
					{
						if (col.m_brdr.GetValue() != gdi::Color().GetValue())
						{
							gdi::Pen pen_brdr(col.m_brdr);
							auto border = To<gdi::Rect>(rect.Adjust(0,0,-1,-1));
							gfx.DrawRectangle(&pen_brdr, border);
						}
					}
				}
				static void Border(gdi::Graphics& gfx, Rect const& rect, EMenuItemState item_state, ContextMenuStyle const& style)
				{
					auto col = style.Col(item_state);
					gdi::Pen pen(col.m_text);
					auto border = To<gdi::Rect>(rect.Adjust(0,0,-1,-1));
					gfx.DrawRectangle(&pen, border);
				}
				static void Bitmap(gdi::Graphics& gfx, Rect const& rect, BitmapPtr const& bm, ContextMenuStyle const& style)
				{
					if (bm == 0) return;
					auto bm_sz = MeasureBitmap(bm, style.m_margin_left - 2);
					gdi::Rect r(int(rect.left + 1), int(rect.top + 1), int(bm_sz.width()), int(bm_sz.height()));
					gfx.DrawImage(bm.get(), r);
				}
				static void Check(gdi::Graphics& gfx, Rect const& rect, int check_state, EMenuItemState item_state, ContextMenuStyle const& style)
				{
					if (check_state == 0) return;
					auto col = style.Col(item_state);
					auto tick = (check_state == 1) ? L"a" : L"h";
					int const tick_len = 1;
					
					gdi::SolidBrush bsh_text(col.m_text);
					gdi::RectF sz; gfx.MeasureString(tick, tick_len, style.m_font_marks.get(), gdi::PointF(), &sz);
					gdi::PointF pt(float(rect.left + (style.m_margin_left - sz.Width)*0.5f), float(rect.top + (rect.height() - sz.Height)*0.5f));
					gfx.DrawString(tick, tick_len, style.m_font_marks.get(), pt, &bsh_text);
				}
				//static void Label(gdi::Graphics& gfx, gdi::Rect const& rect, UINT item_state, int check_state, std::wstring const& text, BitmapPtr bitmap, ContextMenuStyle const& style)
				//{
				//	// Draw background and left margin items
				//	Bkgd  (gfx, rect, item_state, style);
				//	Bitmap(gfx, rect, bitmap, style);
				//	Check (gfx, rect, check_state, item_state, style);

				//	// Draw the label text
				//	auto col = style.Col(item_state);
				//	gdi::SolidBrush bsh_text(col.m_text);
				//	gdi::StringFormat fmt; fmt.SetHotkeyPrefix(Gdiplus::HotkeyPrefixShow);
				//	gdi::PointF pt(float(rect.X + style.m_margin_left + style.m_text_margin), float(rect.Y + (rect.Height - m_rect_text.Height)*0.5f));
				//	gfx.DrawString(text.c_str(), int(text.size()), style.m_font_text.get(), pt, &fmt, &bsh_text);
				//}
			};

			#pragma endregion

			#pragma region Menu Items

			// Separator menu item
			struct Separator :ContextMenuItem
			{
				Separator(ContextMenu& menu)
					:ContextMenuItem(-1, &menu)
				{
					menu.m_items.push_back(this);
				}
				bool Selectable() const override
				{
					return false;
				}
				Size MeasureItem(gdi::Graphics&) override
				{
					// Height = 6, 2 bkgd rows, dark row, light row, 2 bkgd rows
					return Size(20, 6);
				}
				void DrawItem(gdi::Graphics& gfx, Rect const& rect) override
				{
					auto& style = Style();
					Draw::Bkgd(gfx, rect, EMenuItemState::Normal, style);

					// Draw the separator line
					gdi::Pen pen_3dhi(impl::ColAdj(style.m_col_norm.m_bkgd, +10, +10, +10));
					gdi::Pen pen_3dlo(impl::ColAdj(style.m_col_norm.m_bkgd, -10, -10, -10));
					auto x0 = static_cast<INT>(rect.left + style.m_margin_left + 1);
					auto x1 = static_cast<INT>(rect.right - MenuMargin);
					auto y  = static_cast<INT>(rect.top + 2);
					if (x0 < x1)
					{
						gfx.DrawLine(&pen_3dlo, x0, y+0, x1, y+0);
						gfx.DrawLine(&pen_3dhi, x0, y+1, x1, y+1);
					}
				}
			};

			// Label menu item
			struct Label :ContextMenuItem
			{
				std::wstring m_text;      // The label text
				Rect         m_rect_text; // The dimensions of the text

				Label(ContextMenu& menu, wchar_t const* text = L"<menu item>", int id = 0, EMenuItemState state = EMenuItemState::Normal, StylePtr style = nullptr, BitmapPtr bm = nullptr)
					:ContextMenuItem(id, &menu, state, style, bm)
					,m_text(text)
					,m_rect_text()
				{
					menu.m_items.push_back(this);
				}
				bool HotkeyPrefix(wchar_t hk) const override
				{
					auto* p = m_text.c_str();
					for (; *p && (*p != L'&' || towupper(*(p+1)) != hk); ++p) {}
					return *p != 0;
				}
				Size MeasureItem(gdi::Graphics& gfx) override
				{
					auto& style = Style();

					// Measure the text size
					auto tx_sz = Draw::MeasureText(gfx, m_text, *style.m_font_text);
					m_rect_text = tx_sz;

					// Measure the bitmap size
					auto bm_sz = Draw::MeasureBitmap(m_bitmap, style.m_margin_left - 2*style.m_bmp_margin);

					// Return the item dimensions
					Size sz;
					sz.cx = tx_sz.width() + style.m_margin_left + 2*style.m_text_margin;
					sz.cy = std::max({tx_sz.height(), bm_sz.height(), long(GetSystemMetrics(SM_CYMENU))});
					return sz;
				}
				void DrawItem(gdi::Graphics& gfx, Rect const& rect) override
				{
					auto& style = Style();
					auto col = style.Col(m_state);
					auto check_state = ItemState(EMenuItemState::Checked) ? 1 : 0;

					// Draw background and left margin items
					Draw::Bkgd  (gfx, rect, m_state, style);
					Draw::Bitmap(gfx, rect, m_bitmap, style);
					Draw::Check (gfx, rect, check_state, m_state, style);

					// Draw the label text
					gdi::SolidBrush bsh_text(col.m_text);
					gdi::StringFormat fmt; fmt.SetHotkeyPrefix(Gdiplus::HotkeyPrefixShow);
					gdi::PointF pt(float(rect.left) + style.m_margin_left + style.m_text_margin, float(rect.top) + 0.5f*(rect.height() - m_rect_text.height()));
					gfx.DrawString(m_text.c_str(), int(m_text.size()), style.m_font_text.get(), pt, &fmt, &bsh_text);
				}
			};

			// Textbox menu item. Consists of a label followed by an edit box
			struct TextBox :Label
			{
				using TBox = gui::TextBox;
				enum { InnerMargin = 2, OuterMargin = 2 };

				TBox         m_edit;       // The hosted control
				Rect         m_rect_value; // The dimensions of the value text
				std::wstring m_value;      // The value to display in the edit box
				FontPtr      m_value_font; // The font to use for the value, if null the label font is used
				int          m_min_width;  // The minimum width of the edit box

				TextBox(ContextMenu& menu, wchar_t const* text = L"<menu item>", wchar_t const* value = L"", int id = 0, EMenuItemState state = EMenuItemState::Normal, StylePtr style = nullptr, BitmapPtr bm = nullptr)
					:Label(menu, text, id, state, style, bm)
					,m_edit(TBox::Params<>().name("cmenu-edit").parent(&menu).anchor(EAnchor::None)) // positioned manually
					,m_rect_value()
					,m_value(value)
					,m_value_font()
					,m_min_width(60)
				{
					m_edit.Key += [&](Control const&, KeyEventArgs const& a)
						{
							if (a.m_vk_key == VK_RETURN)
								menu.Close(EDialogResult::Ok);
						};
				}
				void CreateHostedControls() override
				{
					HWND parent = *static_cast<ContextMenu*>(m_menu);
					m_edit.Create(TBox::Params<>().text(m_value.c_str()).parent(parent).wh(50, 18).as<Control::Params<>>());
				}
				Size MeasureItem(gdi::Graphics& gfx) override
				{
					auto& style = Style();

					// Measure the label portion of the item
					auto lbl_sz = Label::MeasureItem(gfx);

					// Measure the edit box portion
					auto edit_sz = m_edit.ClientRect(true); // same as WindowRect();

					// Return the item dimensions
					Size sz;
					sz.cx = style.m_margin_left + m_rect_text.width() + edit_sz.width() + 3*style.m_text_margin;
					sz.cy = std::max({lbl_sz.cy, edit_sz.height()+4, long(GetSystemMetrics(SM_CYMENU))});
					return sz;
				}
				void DrawItem(gdi::Graphics& gfx, Rect const& rect) override
				{
					auto& style = Style();
					auto col = style.Col(m_state);

					// Draw the label
					Label::DrawItem(gfx, rect);

					// Position the edit box
					auto r = m_edit.ClientRect(true);
					auto pt = Point(
						style.m_margin_left + m_rect_text.width() + 2*style.m_text_margin,
						rect.top + (rect.height() - r.height()) / 2);
					
					m_edit.ParentRect(Rect(pt, r.size()));
					m_edit.Enabled(!ItemState(EMenuItemState::Disabled));
				}
			};

			#pragma endregion

			ContextMenu(StylePtr style = nullptr)
				:ContextMenu(nullptr, nullptr, EMenuItemState::Normal, style, nullptr)
			{}
			ContextMenu(ContextMenu& menu, TCHAR const* text = _T("<submenu>"), EMenuItemState state = EMenuItemState::Normal, StylePtr style = nullptr, BitmapPtr bm = nullptr)
				:ContextMenu(&menu, text, state, style, bm)
			{}

			// Child menu items
			std::vector<ContextMenuItem*> const& Items() const
			{
				return m_items;
			}

			// Show the context menu
			void Show(WndRef parent, int x, int y)
			{
				// Show the context menu as a modal dialog because the menu closes when this function returns
				if (m_items.empty()) return;
				cp().m_init_param = (void*)MakeLParam(x, y);
				Form::ShowDialog(parent);
			}

			// Result of a high test on the menu
			struct HitTestResult
			{
				// Note, menu's have a margin
				// The top left of the first item is at (MenuMargin,MenuMargin)
				ContextMenuItem* m_item;   // The item under the hit point
				int              m_index;  // Index of the hit item
				Point            m_point;  // The point at which the hit test was performed (in menu client space)
				Rect             m_bounds; // The menu-relative bounds of the hit item (in menu client space)

				HitTestResult()
					:m_item()
					,m_index()
					,m_point()
					,m_bounds()
				{}
				HitTestResult(ContextMenuItem* item, int index, Point pt, Rect bounds)
					:m_item(item)
					,m_index(index)
					,m_point(pt)
					,m_bounds(bounds)
				{}
			};

			// Hit test the menu. 'pt' should be in client space
			HitTestResult HitTest(Point const& pt)
			{
				// Find the menu item that the mouse is over
				auto index = -1;
				auto bounds = Rect(MenuMargin, MenuMargin, MenuMargin + m_size.cx, MenuMargin);
				for (auto item : m_items)
				{
					// Find the bounds for the item ([tl,br)]
					++index;
					bounds.top = bounds.bottom;
					bounds.bottom += item->m_size.cy;
					if (bounds.Contains(pt))
						return HitTestResult(item, index, pt, bounds);
				}
				return HitTestResult();
			}

			// Get/Set the menu item that was selected by the mouse or with keys
			HitTestResult Selected() const
			{
				return m_selected;
			}
			void Selected(HitTestResult hit, bool final_selection)
			{
				// Deselect the previously 'm_selected' item
				if (m_selected.m_item != nullptr)
				{
					m_selected.m_item->ItemState(EMenuItemState::Selected, false);
					Invalidate(false, &m_selected.m_bounds);
				}

				m_selected = hit;

				// Update 'm_selected' with the new hit item
				if (m_selected.m_item != nullptr)
				{
					if (!m_selected.m_item->ItemState(EMenuItemState::Disabled))
					{
						m_selected.m_item->ItemState(EMenuItemState::Selected, true);
						Invalidate(false, &m_selected.m_bounds);
					}
				}

				// Close the menu on the final selection
				if (final_selection)
				{
					if (m_selected.m_item != nullptr && m_selected.m_item->Selectable())
					{
						OnItemSelected();
						Close(EDialogResult::Ok);
					}
					else
					{
						Close(EDialogResult::Cancel);
					}
				}
			}

			// Select a menu item by index
			void Selected(int index, bool final_selection)
			{
				if (index < 0 || index >= int(m_items.size()))
					throw std::runtime_error(pr::FmtS("menu item index (%d) out of Range [0,%d)", index, int(m_items.size())));

				// Calculate the item bounds
				Rect bounds(MenuMargin, MenuMargin, MenuMargin + m_size.cx, MenuMargin);
				for (int i = 0; i <= index; ++i)
				{
					bounds.top = bounds.bottom;
					bounds.bottom += m_items[i]->m_size.cy;
				}

				// Selected the item
				Selected(HitTestResult(m_items[index], index, bounds.centre(), bounds), final_selection);
			}

			// Raised when a menu item is selected
			EventHandler<ContextMenu&, EmptyArgs const&> ItemSelected;

			// Handlers
			virtual void OnItemSelected()
			{
				ItemSelected(*this, EmptyArgs());
			}

		private:

			enum
			{
				// Minimum width of items in the menu
				MinimumWidth = 100,

				// The border around the items
				MenuMargin = 2,
			};

			// The name for the sub-menu
			std::wstring m_submenu_name;

			// Child menu items
			std::vector<ContextMenuItem*> m_items;

			// The last item that the mouse was over or key events have selected
			// Note: while the menu is open, this is the highlighted item, after the
			// menu is closed, then it becomes the selected item
			HitTestResult m_selected;

			using HookMap = std::unordered_map<DWORD, ContextMenu*>;
			static HookMap& ThreadHookMap()
			{
				static HookMap s_thread_hook_map;
				return s_thread_hook_map;
			}
			HHOOK m_mouse_hook;

			// Subclass the dialog window class for the context menu
			static WndClassEx const& RegWndClass()
			{
				static WndClassEx s_wc = [=]
				{
					static wchar_t const* class_name = L"gui::cmenu";
					WndClassEx wc(class_name);
					if (wc.m_atom != 0)
						return wc;

					// Subclass the dialog window class
					::GetClassInfoExW(wc.hInstance, (LPCWSTR)WC_DIALOG, &wc);
					wc.style |= CS_DROPSHADOW;
					wc.lpszClassName = class_name;
					return std::move(wc.Register());
				}();

				return s_wc;
			}
			static DlgTemplate const& Templ()
			{
				static DlgTemplate cmenu_templ(Params<>().dlg().wndclass(RegWndClass()).xy(0,0).wh(50,50).name("ctx-menu").style('=',WS_POPUP|WS_BORDER).style_ex('=',0));
				return cmenu_templ;
			}

			ContextMenu(ContextMenu* menu, TCHAR const* text, EMenuItemState state, StylePtr style, BitmapPtr bm)
				:Form(Params<>().templ(Templ()))
				,ContextMenuItem(-1, menu, state, style, bm)
				,m_submenu_name(Widen(text))
				,m_items()
				,m_mouse_hook()
				,m_selected()
			{
				// If no style is given and no parent to inherit the style from, create a default style
				if (!m_style && m_parent.ctrl() == nullptr)
					m_style = std::make_shared<ContextMenuStyle>();
			}

			// Message map function
			bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
			{
				switch (message)
				{
					case WM_INITDIALOG:
					{
						// Allow child menu items to create hosted controls now that we have an hwnd
						for (auto& item : m_items)
							item->CreateHostedControls();

						// Measure the size of the menu
						gdi::Graphics gfx(m_hwnd);
						Point pt(GetXLParam(lparam), GetYLParam(lparam));
						m_size = MeasureItem(gfx);

						// Client area is the contained item size plus margins
						auto client = Rect(Point(), m_size + Size(2 * MenuMargin, 2 * MenuMargin));
						auto bounds = AdjRect(client).Shifted(pt.x, pt.y);
						ParentRect(bounds, true);

						// Turn off dialog behaviour so that we get WM_MOUSEMOVE events
						cp().m_dlg_behaviour = false;

						// Mouse hook static callback
						auto MouseHook = [](int code, WPARAM wparam, LPARAM lparam)
						{
							auto This = ThreadHookMap()[GetCurrentThreadId()];
							auto& mhs = *reinterpret_cast<MOUSEHOOKSTRUCT*>(lparam);
							if (code >= 0)
							{
								// Messages ids are contiguous in the range [WM_LBUTTONUP,WM_MBUTTONDBLCLK]
								// this subset does not include WM_MOUSEMOVE, WM_MOUSEWHEEL and a few others
								if ((wparam >= WM_LBUTTONDOWN && wparam <= WM_MBUTTONDBLCLK) ||
									(wparam >= WM_NCLBUTTONDOWN && wparam <= WM_NCXBUTTONDBLCLK))
								{
									// Close when clicking on a window that isn't a child of 'This'
									if (!::IsChild(This->m_hwnd, mhs.hwnd))
										This->Close(EDialogResult::Cancel);
								}
							}
							return CallNextHookEx(This->m_mouse_hook, code, wparam, lparam);
						};

						// Hook the mouse to watch for mouse events outside of the context menu
						auto thread_id = GetCurrentThreadId();
						ThreadHookMap()[thread_id] = this;
						m_mouse_hook = SetWindowsHookExW(WH_MOUSE, static_cast<HOOKPROC>(MouseHook), 0, thread_id);
						Check(m_mouse_hook != nullptr, "Failed to install mouse hook procedure");

						return ForwardToChildren(hwnd, message, wparam, lparam, result, AllChildren);
					}
					case WM_DESTROY:
					{
						if (m_hwnd == hwnd)
						{
							// Remove the mouse hook
							auto thread_id = GetCurrentThreadId();
							UnhookWindowsHookEx(m_mouse_hook);
							ThreadHookMap().erase(thread_id);

							// Turn dialog behaviour back on so destruction occurs properly
							cp().m_dlg_behaviour = true;
						}
						break;
					}
					case WM_NCACTIVATE:
					{
						if (m_hwnd == hwnd)
						{
							if (wparam == 0)
								Close(EDialogResult::Cancel);
						}
						break;
					}
					case WM_PAINT:
					{
						PaintStruct ps(m_hwnd);
						gdi::Graphics gfx(ps.hdc);
						DrawItem(gfx, Rect());
						return false;
					}
					case WM_MOUSEMOVE:
					{
						// Only retest for hits when outside the last hit rect
						auto pt = Point(GetXLParam(lparam), GetYLParam(lparam));
						if (Selected().m_item != nullptr && Selected().m_bounds.Contains(pt))
							break;

						// Mouse is no longer over the last hit item, retest
						auto hit = HitTest(pt);
						if (hit.m_item != nullptr || Selected().m_item != nullptr)
							Selected(hit, false);

						break;
					}
					case WM_LBUTTONUP:
					{
						auto pt = Point(GetXLParam(lparam), GetYLParam(lparam));
						auto hit = HitTest(pt);
						Selected(hit, true);
						return true;
					}
					case WM_KEYUP:
					{
						// Handle key selection of menu items
						auto item_count = int(m_items.size());
						if (item_count == 0) break;
						auto ch = TCHAR(wparam);
						int index = Selected().m_item != nullptr ? Selected().m_index : item_count;

						auto prev = [=](int idx) { return int(idx - 1 + item_count) % item_count; };
						auto next = [=](int idx) { return int(idx + 1 + item_count) % item_count; };

						if (ch == VK_RETURN) // Select the current menu item
						{
							if (index != item_count && m_items[index]->Selectable())
							{
								Selected(index, true);
								return true;
							}
						}
						else if (ch == VK_HOME) // First selectable item
						{
							for (index = 0; index != item_count && !m_items[index]->Selectable(); ++index) {}
							if (index != item_count) Selected(index, false);
							return true;
						}
						else if (ch == VK_ESCAPE) // Cancel
						{
							Close(EDialogResult::Cancel);
							return true;
						}
						else if (ch == VK_END) // Last selectable item
						{
							for (index = item_count; index-- != 0 && !m_items[index]->Selectable(); ) {}
							if (index != -1) Selected(index, false);
							return true;
						}
						else if (ch == VK_DOWN) // Next selectable item
						{
							index = index != item_count ? next(index) : 0;
							int i; for (i = 0; i != item_count && !m_items[index]->Selectable(); ++i, index = next(index)) {}
							if (i != item_count) Selected(index, false);
							return true;
						}
						else if (ch == VK_UP) // Prev selectable item
						{
							index = index != item_count ? prev(index) : item_count - 1;
							int i; for (i = 0; i != item_count && !m_items[index]->Selectable(); ++i, index = prev(index)) {}
							if (i != item_count) Selected(index, false);
							return true;
						}
						else if (IsCharAlphaNumeric(ch)) // Next/Prev item with a matching Hotkey Prefix
						{
							wchar_t hk = Widen(&ch, 1)[0];
							if (!KeyState(VK_SHIFT))
							{
								index = index != item_count ? next(index) : 0;
								int i; for (i = 0; i != item_count && !m_items[index]->HotkeyPrefix(hk); ++i, index = next(index)) {}
								if (i != item_count) Selected(index, false);
							}
							else
							{
								index = index != item_count ? prev(index) : item_count - 1;
								int i; for (i = 0; i != item_count && !m_items[index]->HotkeyPrefix(hk); ++i, index = prev(index)) {}
								if (i != item_count) Selected(index, false);
							}
							return true;
						}
						break;
					}
				}

				// Messages that get here will be forwarded to child controls as well
				return Control::ProcessWindowMessage(hwnd, message, wparam, lparam, result);
			}

			// Measure the size of the context menu
			Size MeasureItem(gdi::Graphics& gfx) override
			{
				// Measure the required size of the context menu
				Size sz;
				for (auto item : m_items)
				{
					item->m_size = item->MeasureItem(gfx);
					sz.cx = std::max(sz.cx, item->m_size.cx);
					sz.cy += item->m_size.cy;
				}

				// Enforce a minimum menu width
				sz.cx = std::max(sz.cx, long(MinimumWidth));
				return sz;
			}
			void DrawItem(gdi::Graphics& gfx, Rect const&) override
			{
				auto& style = BMenu::Style();

				// Background
				gdi::SolidBrush bsh(style.m_col_norm.m_bkgd);
				gfx.FillRectangle(&bsh, 0, 0, m_size.cx + MenuMargin*2 + 1, m_size.cy + MenuMargin*2 + 1);

				// Menu items
				Rect bounds(MenuMargin, MenuMargin, MenuMargin + m_size.cx, MenuMargin); // [inclusive,exclusive)
				for (auto item : m_items)
				{
					// Find the bounds for the item ([tl,br)]
					bounds.top = bounds.bottom;
					bounds.bottom += item->m_size.cy;

					// Tell the item to draw
					//gfx.SetClip(To<gdi::Rect>(bounds.Inflate(0,0,1,1)));
					item->DrawItem(gfx, bounds);
				}
			}
		};

	// TODO: do this as a modal dialog
		#if 0
		namespace old
		{
		// Menu item base class
		struct ContextMenuItem
		{
			int              m_id;          // Menu item id
			ContextMenuItem* m_parent;      // The containing menu if not null
			int              m_check_state; // 0-Unchecked, 1-checked, 2-unknown
			StylePtr         m_style;       // Item specific style if not null
			BitmapPtr        m_bitmap;      // A bitmap to draw next to the item (null means no bitmap)

			// 'parent' is the containing menu item that properties are inherited from
			// 'id' is the menu item id
			ContextMenuItem(int id = -1, ContextMenuItem* parent = nullptr, int check_state = 0, StylePtr style = nullptr, BitmapPtr bm = nullptr)
				:m_id(id)
				,m_parent(parent)
				,m_check_state(check_state)
				,m_style(style)
				,m_bitmap(bm)
			{}
			virtual ~ContextMenuItem()
			{}

			// The window acting as the parent for hosted controls
			virtual HWND CtrlHost() const
			{
				assert(m_parent != nullptr && "Item is not parented to a ContextMenu item"); // ContextMenu overrides this method
				return m_parent->CtrlHost();
			}

			// Return the HMENU associated with m_parent
			virtual HMENU Menu() const
			{
				assert(m_parent != nullptr && "Item is not parented to a ContextMenu item"); // ContextMenu overrides this method
				return m_parent->Menu();
			}

			// Return the style to use for a context menu item
			virtual ContextMenuStyle const& Style() const
			{
				if (m_style) return *m_style;
				assert(m_parent != nullptr && "Item is not parented to a ContextMenu item"); // ContextMenu overrides this method
				return m_parent->Style();
			}

			virtual void MeasureItem(ContextMenu& menu, gdi::Graphics& gfx, MEASUREITEMSTRUCT* mi) = 0;
			virtual void DrawItem   (ContextMenu& menu, gdi::Graphics& gfx, DRAWITEMSTRUCT* di) = 0;
			virtual void Selected   (ContextMenu& /*menu*/, Rect const& /*rect*/, MENUITEMINFO const& /*mii*/) {}
		};

		// Context menu
		// Note: this class requires GdiPlus, remember to instantiate an instance
		// of 'pr::GdiPlus' somewhere so that the gdi+ dll is loaded
		struct ContextMenu :ContextMenuItem
		{
			#pragma region Draw Helpers

			// Helper methods for drawing context menu items
			struct Draw
			{
				static gdi::Rect MeasureText(gdi::Graphics const& gfx, std::wstring const& text, ContextMenuStyle const& style)
				{
					gdi::StringFormat fmt; fmt.SetHotkeyPrefix(Gdiplus::HotkeyPrefixShow);
					gdi::RectF text_sz; gfx.MeasureString(text.c_str(), int(text.size()), style.m_font_text.get(), gdi::PointF(), &fmt, &text_sz);
					return To<gdi::Rect>(text_sz);
				}
				static gdi::Rect MeasureBitmap(BitmapPtr const& bm, int width)
				{
					// Measure the size of a bitmap rescaled to fit widthwise within 'width'
					if (!bm) return gdi::Rect();
					gdi::Rect rect(0, 0, bm->GetWidth(), bm->GetHeight());
					rect.Height = rect.Height * width / rect.Width;
					rect.Width = width;
					return rect;
				}
				static void Bkgd(gdi::Graphics& gfx, gdi::Rect const& rect, UINT item_state, ContextMenuStyle const& style)
				{
					auto col = style.Col(item_state);
					gdi::Pen   pen_3dhi(impl::ColAdj(col.m_bkgd, 10, 10, 10));
					gdi::Pen   pen_3dlo(impl::ColAdj(col.m_bkgd,-10,-10,-10));
					gdi::SolidBrush bsh_bkgd(col.m_bkgd);

					gfx.FillRectangle(&bsh_bkgd, rect);
					int x = rect.X + style.m_margin_left;
					gfx.DrawLine(&pen_3dlo, x+0, rect.GetTop(), x+0, rect.GetBottom() - 1);
					gfx.DrawLine(&pen_3dhi, x+1, rect.GetTop(), x+1, rect.GetBottom() - 1);
				}
				static void Bitmap(gdi::Graphics& gfx, gdi::Rect const& rect, BitmapPtr const& bm, ContextMenuStyle const& style)
				{
					if (bm == 0) return;
					auto bm_sz = MeasureBitmap(bm, style.m_margin_left - 2);
					gdi::Rect r(rect.X + 1, rect.Y + 1, bm_sz.Width, bm_sz.Height);
					gfx.DrawImage(bm.get(), r);
				}
				static void Check(gdi::Graphics& gfx, gdi::Rect const& rect, int check_state, UINT item_state, ContextMenuStyle const& style)
				{
					if (check_state == 0) return;
					auto col = style.Col(item_state);
					gdi::SolidBrush bsh_text(col.m_text);
					auto tick = (check_state == 1) ? L"a" : L"h"; int const tick_len = 1;
					gdi::RectF sz; gfx.MeasureString(tick, tick_len, style.m_font_marks.get(), gdi::PointF(), &sz);
					gdi::PointF pt(float(rect.X + (style.m_margin_left - sz.Width)*0.5f), float(rect.Y + (rect.Height - sz.Height)*0.5f));
					gfx.DrawString(tick, tick_len, style.m_font_marks.get(), pt, &bsh_text);
				}
				//static void Label(gdi::Graphics& gfx, gdi::Rect const& rect, UINT item_state, int check_state, std::wstring const& text, BitmapPtr bitmap, ContextMenuStyle const& style)
				//{
				//	// Draw background and left margin items
				//	Bkgd  (gfx, rect, item_state, style);
				//	Bitmap(gfx, rect, bitmap, style);
				//	Check (gfx, rect, check_state, item_state, style);

				//	// Draw the label text
				//	auto col = style.Col(item_state);
				//	gdi::SolidBrush bsh_text(col.m_text);
				//	gdi::StringFormat fmt; fmt.SetHotkeyPrefix(Gdiplus::HotkeyPrefixShow);
				//	gdi::PointF pt(float(rect.X + style.m_margin_left + style.m_text_margin), float(rect.Y + (rect.Height - m_rect_text.Height)*0.5f));
				//	gfx.DrawString(text.c_str(), int(text.size()), style.m_font_text.get(), pt, &fmt, &bsh_text);
				//}
			};

			#pragma endregion

			#pragma region Menu Items

			// Separator menu item
			struct Separator :ContextMenuItem
			{
				Separator(ContextMenu& menu)
					:ContextMenuItem(-1, &menu)
				{
					auto cmi = static_cast<ContextMenuItem*>(this);
					Check(::AppendMenuW(Menu(), MF_SEPARATOR|MF_OWNERDRAW, m_id, LPCWSTR(cmi)), "Failed to append menu separater");
				}
				void MeasureItem(ContextMenu&, gdi::Graphics&, MEASUREITEMSTRUCT* mi) override
				{
					mi->itemWidth = 1;
					mi->itemHeight = 3;
				}
				void DrawItem(ContextMenu&, gdi::Graphics& gfx, DRAWITEMSTRUCT* di) override
				{
					auto& style = Style();
					auto rect = To<gdi::Rect>(di->rcItem);
					Draw::Bkgd(gfx, rect, di->itemState, style);

					// Draw the separator line
					gdi::Pen pen_3dhi(impl::ColAdj(style.m_col_norm.m_bkgd, +10, +10, +10));
					gdi::Pen pen_3dlo(impl::ColAdj(style.m_col_norm.m_bkgd, -10, -10, -10));
					auto x0 = rect.X + style.m_margin_left + 1;
					auto x1 = rect.X + rect.Width;
					auto y  = rect.Y + rect.Height / 2;
					if (x0 < x1)
					{
						gfx.DrawLine(&pen_3dlo, x0, y+0, x1, y+0);
						gfx.DrawLine(&pen_3dhi, x0, y+1, x1, y+1);
					}
				}
			};

			// Label menu item
			struct Label :ContextMenuItem
			{
				std::wstring m_text;      // The label text
				gdi::Rect      m_rect_text; // The dimensions of the text

				Label(ContextMenu& menu, TCHAR const* text = _T("<menu item>"), int id = 0, int check_state = 0, StylePtr style = nullptr, BitmapPtr bm = nullptr)
					:ContextMenuItem(id, &menu, check_state, style, bm)
					,m_text(Widen(text))
					,m_rect_text()
				{
					auto cmi = static_cast<ContextMenuItem*>(this);
					Check(::AppendMenuW(Menu(), MF_OWNERDRAW, m_id, LPCWSTR(cmi)), "Failed to append menu item");
				}
				void MeasureItem(ContextMenu&, gdi::Graphics& gfx, MEASUREITEMSTRUCT* mi) override
				{
					auto& style = Style();

					// Measure the text size
					auto tx_sz = Draw::MeasureText(gfx, m_text, style);

					// Measure the bitmap size
					auto bm_sz = Draw::MeasureBitmap(m_bitmap, style.m_margin_left - 2*style.m_bmp_margin);

					// Set item dimensions
					auto sz = impl::Inflate(tx_sz, style.m_text_margin, style.m_text_margin);
					mi->itemWidth  = style.m_margin_left + sz.Width;
					mi->itemHeight = std::max(sz.Height, std::max(bm_sz.Height, GetSystemMetrics(SM_CYMENU)));
					m_rect_text = tx_sz;
				}
				void DrawItem(ContextMenu&, gdi::Graphics& gfx, DRAWITEMSTRUCT* di) override
				{
					auto& style = Style();
					auto rect = To<gdi::Rect>(di->rcItem);

					// Draw background and left margin items
					Draw::Bkgd  (gfx, rect, di->itemState, style);
					Draw::Bitmap(gfx, rect, m_bitmap, style);
					Draw::Check (gfx, rect, m_check_state, di->itemState, style);

					// Draw the label text
					auto col = style.Col(di->itemState);
					gdi::SolidBrush bsh_text(col.m_text);
					gdi::StringFormat fmt; fmt.SetHotkeyPrefix(Gdiplus::HotkeyPrefixShow);
					gdi::PointF pt(float(rect.X + style.m_margin_left + style.m_text_margin), float(rect.Y + (rect.Height - m_rect_text.Height)*0.5f));
					gfx.DrawString(m_text.c_str(), int(m_text.size()), style.m_font_text.get(), pt, &fmt, &bsh_text);
				}

			private:
				// This constructor is used by ContextMenu to create submenus
				friend struct ContextMenu;
				Label(ContextMenu* parent = nullptr, TCHAR const* text = nullptr, StylePtr style = nullptr, BitmapPtr bm = nullptr)
					:ContextMenuItem(-1, parent, 0, style, bm)
					,m_text(Widen(text))
					,m_rect_text()
				{}
			};

			// Textbox menu item
			// Consists of a label followed by an edit box
			struct TextBox :Label
			{
				using TBox = gui::TextBox;
				enum { InnerMargin = 2, OuterMargin = 2 };

				TBox         m_edit;       // The hosted control
				gdi::Rect      m_rect_value; // The dimensions of the value text
				std::wstring m_value;      // The value to display in the edit box
				FontPtr      m_value_font; // The font to use for the value, if null the label font is used
				int          m_min_width;  // The minimum width of the edit box

				TextBox(ContextMenu& menu, TCHAR const* text = _T("<menu item>"), TCHAR const* value = _T(""), int id = 0, int check_state = 0, StylePtr style = nullptr, BitmapPtr bm = nullptr)
					:Label(menu, text, id, check_state, style, bm)
					,m_edit(value, 0, 0, TBox::DefW, TBox::DefH, id, menu.m_win, nullptr, EAnchor::None)
					,m_rect_value()
					,m_value(Widen(value))
					,m_value_font()
					,m_min_width(60)
				{}
				void MeasureItem(ContextMenu& menu, gdi::Graphics& gfx, MEASUREITEMSTRUCT* mi) override
				{
					auto& style = Style();

					// Measure the label portion of the item
					Label::MeasureItem(menu, gfx, mi);

					// Measure the edit box portion
					gdi::RectF text_sz; gfx.MeasureString(m_value.c_str(), int(m_value.size()), style.m_font_text.get(), gdi::PointF(), 0, &text_sz);
					m_rect_value = To<gdi::Rect>(text_sz);
					m_rect_value.Width = std::max(m_rect_value.Width, m_min_width);

					// Expand the item dimensions
					auto r = impl::Inflate(m_rect_value, InnerMargin+OuterMargin, InnerMargin+OuterMargin);
					mi->itemWidth += r.Width;
					mi->itemHeight = std::max(int(mi->itemHeight), r.Height);
				}
				void DrawItem(ContextMenu& menu, gdi::Graphics& gfx, DRAWITEMSTRUCT* di) override
				{
					auto& style = Style();
					auto col = style.Col(di->itemState);
					auto rect = To<gdi::Rect>(di->rcItem);

					// Draw the label
					Label::DrawItem(menu, gfx, di);

					// Draw an edit box
					auto disabled = (di->itemState & ODS_DISABLED) != 0;
					auto r = CtrlRect(rect, style.m_margin_left);
					gdi::SolidBrush bsh_bkgd(disabled ? gdi::Color((GdiARGB)gdi::Color::LightGray) : gdi::Color((GdiARGB)gdi::Color::WhiteSmoke));
					gdi::Pen        pen_brdr(gdi::Color((GdiARGB)gdi::Color::Black), 0);
					gfx.FillRectangle(&bsh_bkgd, r);
					gfx.DrawRectangle(&pen_brdr, r);

					// Write the edit box text
					gdi::SolidBrush bsh_edittxt(style.m_col_norm.m_text);
					gdi::PointF pt(float(r.X + InnerMargin), float(r.Y + InnerMargin));
					gfx.DrawString(m_value.c_str(), int(m_value.size()), style.m_font_text.get(), pt, 0, &bsh_edittxt);
				}
				void Selected(ContextMenu&, Rect const& rect, MENUITEMINFO const&) override
				{
					auto& style = Style();

					// Set the font to use in the edit control
					//gdi::Graphics gfx(menu); LOGFONTW logfont;
					//auto& font = m_value_font ? m_value_font : style.m_font_text;
					//if (font && font->GetLogFontW(&gfx, &logfont) == Gdiplus::Ok)
					//	m_font = ::CreateFontIndirectW(&logfont);

					// Display the control
					// 'rect' should be the position of the control in screen space
					auto r = To<RECT>(CtrlRect(To<gdi::Rect>(rect), style.m_margin_left));
					m_edit.MoveWindow(r);
					//TextBox::Show( DoModal(menu->m_hWnd, (LPARAM)&r);
				}

				// Returns the bounds of the hosted control
				gdi::Rect CtrlRect(gdi::Rect const& rect, int margin) const
				{
					auto& style = Style();
					return gdi::Rect(
						rect.X + margin + style.m_text_margin + m_rect_text.Width + style.m_text_margin + OuterMargin,
						rect.Y + OuterMargin,
						m_rect_value.Width + 2*InnerMargin,
						m_rect_value.Height + 2*InnerMargin);
				}

				//BEGIN_MSG_MAP(Edit)
				//	MSG_WM_INITDIALOG(OnInitDialog)
				//	MSG_WM_COMMAND(OnCommand)
				//END_MSG_MAP()
				//BOOL OnInitDialog(CWindow, LPARAM lparam)
				//{
				//	// 'rect' should be the position of the control in screen space
				//	CRect const& rect = *(CRect const*)lparam;

				//	CRect winrect = rect;
				//	AdjustWindowRect(&winrect, (DWORD)GetWindowLongPtr(GWL_STYLE), FALSE);
				//	SetWindowPos(0, &winrect, SWP_NOZORDER);

				//	CRect r; GetClientRect(&r);
				//	m_edit.Create(m_hWnd, &r, 0, WS_CHILD|WS_VISIBLE|WS_BORDER|ES_AUTOHSCROLL, 0);
				//	if (!m_font.IsNull()) m_edit.SetFont(m_font);
				//	::SetWindowTextW(m_edit, m_value.c_str());

				//	return TRUE;
				//}
				//void OnCommand(UINT, int nID, CWindow)
				//{
				//	switch (nID)
				//	{
				//	default: break;
				//	case IDCANCEL: EndDialog(0); break;
				//	case IDOK: // Copy the text from the control
				//		m_value.resize(::GetWindowTextLengthW(m_edit) + 1);
				//		if (!m_value.empty()) ::GetWindowTextW(m_edit, &m_value[0], int(m_value.size()));
				//		while (!m_value.empty() && *(--m_value.end()) == 0) m_value.resize(m_value.size() - 1);
				//		EndDialog(0);
				//		break;
				//	}
				//}
			};

			#pragma endregion

		private:
			// This window is used to handle menu messages such as measure/draw item messages
			// It is a zero-sized non-modal window created when the menu is created.
			// It also serves as the parent for hosted controls
			struct MenuParentWindow :Form<MenuParentWindow>
			{
				// A selected menu item
				struct Selection
				{
					HMENU m_menu;
					UINT  m_flags;
					UINT  m_id;
					Rect  m_rect;
					Selection()
						:m_menu()
						,m_flags(0)
						,m_id(0)
						,m_rect()
					{}
				};

				ContextMenu* m_owner;
				Selection m_sel;      // The menu item that was selected

				MenuParentWindow(ContextMenu* owner)
					:Form<MenuParentWindow>(_T(""), nullptr, 0, 0, 0, 0, 0, 0, IDC_UNUSED, "ctx-menu")
					//:Form<MenuParentWindow>(DlgTemplate(nullptr, 0, 0, 0, 0, DS_CENTER|DS_SHELLFONT|WS_VISIBLE|WS_GROUP|WS_POPUP, 0))
					,m_owner(owner)
					,m_sel()
				{}

				// Message map function
				bool ProcessWindowMessage(HWND parent_hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
				{
					DebugMessage(parent_hwnd, message, wparam, lparam);
					switch (message)
					{
					case WM_MENUSELECT:
						{
							auto item  = (UINT)LOWORD(wparam);
							auto flags = (UINT)HIWORD(wparam);
							auto menu  = (HMENU)lparam;

							OutputDebugStringA(pr::FmtS("OnMenuSelect: id: %d, flags: %x, handle: %x\n", item, flags, menu));
							if (flags != 0x0000FFFF) // Menu closing notification
							{
								m_sel.m_menu  = menu;
								m_sel.m_flags = flags;
								m_sel.m_id    = item;
								GetMenuItemRect(m_hwnd, menu, item, &m_sel.m_rect);
							}
							break;
						}
					case WM_LBUTTONDOWN:
						{
							OutputDebugStringA(pr::FmtS("OnLButtonDown:\n"));
							break;
						}
					case WM_KEYDOWN:
						{
							OutputDebugStringA(pr::FmtS("OnKeyDown:\n"));
							break;
						}
					case WM_CAPTURECHANGED:
						{
							// Capture changed occurs just as the menu is closing
							// but, crucially, while the window is still visible
							// :-/ doesn't work for sub menus...
							//m_result = ApplySelection();
							break;
						}
					case WM_MENUCHAR:
						{
							auto ch    = (TCHAR)LOWORD(wparam);
							auto flags = (UINT)HIWORD(wparam);
							auto menu  = (HMENU)lparam;
							OutputDebugStringA("OnMenuChar\n");
							(void)ch,flags,menu;
							break;
						}
					case WM_MEASUREITEM:
						{
							auto mi = reinterpret_cast<MEASUREITEMSTRUCT*>(lparam);

							// If you get a crash in here, check that a "static_cast<ContextMenuItem*>(this)" pointer was used in AppendMenuW()
							if (mi->CtlType == ODT_MENU)
							{
								gdi::Graphics gfx(m_hwnd);
								auto item = static_cast<ContextMenuItem*>((void*)(mi->itemData));
								item->MeasureItem(*m_owner, gfx, mi);
							}
							break;
						}
					case WM_DRAWITEM:
						{
							auto di = reinterpret_cast<DRAWITEMSTRUCT*>(lparam);

							// If you get a crash in here, check that a "static_cast<ContextMenuItem*>(this)" pointer was used in AppendMenuW()
							if (di->CtlType == ODT_MENU)
							{
								gdi::Graphics gfx(di->hDC);
								auto item = static_cast<ContextMenuItem*>((void*)(di->itemData));
								item->DrawItem(*m_owner, gfx, di);
							}
							break;
						}
					}
					return Control::ProcessWindowMessage(parent_hwnd, message, wparam, lparam, result);
				}
			};

			MenuParentWindow m_win; // A zero-sized window that handles the menu messages
			gui::Menu m_root;   // The handle to the popup menu
			Label m_label;          // The text label for the submenu

			HWND CtrlHost() const override
			{
				return m_win;
			}
			HMENU Menu() const override
			{
				return m_root;
			}
			void MeasureItem(ContextMenu& menu, gdi::Graphics& gfx, MEASUREITEMSTRUCT* mi) override
			{
				m_label.MeasureItem(menu, gfx, mi);
			}
			void DrawItem(ContextMenu& menu, gdi::Graphics& gfx, DRAWITEMSTRUCT* di) override
			{
				m_label.DrawItem(menu, gfx, di);
			}

		public:

			// The context menu item is created from a zero-size dialog template.
			// This allows the creation of the window to be deferred until the menu is displayed (and parented to an arbitrary window) 
			ContextMenu(ContextMenu* parent = nullptr, TCHAR const* text = nullptr, StylePtr style = nullptr, BitmapPtr bm = nullptr)
				:ContextMenuItem(-1, parent, 0, style, bm)
				,m_win(this)
				,m_root(::CreatePopupMenu())
				,m_label(parent, text, style, bm)
			{
				// If no style is given and no parent to inherit the style from, create a default style
				if (!m_style && !m_parent)
					m_style = std::make_shared<ContextMenuStyle>();

				// If this is a sub menu, append it to the parent
				if (m_parent)
				{
					auto cmi = static_cast<ContextMenuItem*>(this);
					::AppendMenuW(m_parent->Menu(), MF_OWNERDRAW|MF_POPUP, UINT_PTR(m_root.m_menu), LPCWSTR(cmi));
				}
			}

			// Show the context menu. Blocks until the menu is closed
			// Returns the id of the item selected
			int Show(HWND parent, int x, int y)
			{
				//m_win.Show(SW_HIDE, parent);
				TrackPopupMenu(m_root, GetSystemMetrics(SM_MENUDROPALIGNMENT)|TPM_LEFTBUTTON, x, y, 0, m_win, nullptr);
				return m_win.m_sel.m_id;
			}

			// Return the HMENU associated with this menu
			operator HMENU() const
			{
				return m_root;
			}
		};

		#pragma region Implementation


		#pragma endregion

/*
#if 0
			class Combo :public ControlHost<Combo> ,public Label
			{
			protected:
				enum { InnerMargin = 2, OuterMargin = 2, ArwBtnWidth = 20 };
				CComboBox     m_combo;         // The hosted combo control
				gdi::Rect m_rect_value;    // The dimensions of the largest value
				CFont         m_font;          // The font used in the combo control

			public:
				typedef std::vector<std::wstring> ValueCont;
				ValueCont m_values;            // The items to display in the combo box
				int       m_index;             // Index of the selected item
				FontPtr   m_value_font;        // The font to use for text in the control
				int       m_min_width;         // Minimum width of the text area in the combo box

				Combo(wchar_t const* text = L"Menu item", ValueCont const* values = 0, UINT_PTR id = 0, int check_state = 0, StylePtr style = StylePtr(), BitmapPtr bm = BitmapPtr())
				:Label(text, id, check_state, style, bm)
				,m_combo()
				,m_rect_value()
				,m_font()
				,m_values(values ? *values : ValueCont())
				,m_index()
				,m_value_font()
				,m_min_width(60)
				{}
				gdi::Rect CtrlRect(gdi::Rect const& rect, int margin) const
				{
					return gdi::Rect(
						rect.X + margin + style.m_text_margin + m_rect_text.Width + style.m_text_margin + OuterMargin,
						rect.Y + OuterMargin,
						m_rect_value.Width + 2*InnerMargin + ArwBtnWidth,
						m_rect_value.Height + 2*InnerMargin);
				}
				CWindow ContextMenuWnd() const
				{
					return ::FindWindowW(L"#32768", 0);
				}
				void MeasureItem(ContextMenu* menu, gdi::Graphics& gfx, MEASUREITEMSTRUCT* mi)
				{
					StylePtr const& style = m_style ? m_style : menu->m_def_style;

					// Measure the label portion of the item
					Label::MeasureItem(menu, gfx, mi);

					// Measure the combo box portion
					gdi::RectF text_sz(0,0,0,0);
					for (auto value : m_values)
					{
						gdi::RectF sz; gfx.MeasureString(value.c_str(), int(value.size()), style->m_font_text.get(), gdi::PointF(), 0, &sz);
						gdi::RectF::Union(text_sz, text_sz, sz);
					}
					m_rect_value = ToRect(text_sz);
					m_rect_value.Width = std::max(m_rect_value.Width, m_min_width);

					// Expand the item dimensions
					auto r = m_rect_value;
					r = Inflate(r, InnerMargin, InnerMargin);
					r.Width += ArwBtnWidth;
					r = Inflate(r, OuterMargin, OuterMargin);
					mi->itemWidth += r.Width;
					mi->itemHeight = std::max(int(mi->itemHeight), r.Height);
				}
				void DrawItem(ContextMenu* menu, gdi::Graphics& gfx, DRAWITEMSTRUCT* di)
				{
					StylePtr const& style = m_style ? m_style : menu->m_def_style;
					bool disabled = (di->itemState & ODS_DISABLED) != 0;
					auto rect = ToRect(di->rcItem);
					auto rect_ctrl = CtrlRect(rect, style->m_margin_left);
					auto rect_btn  = rect_ctrl; rect_btn.X = rect_btn.GetRight() - ArwBtnWidth; rect_btn.Width = ArwBtnWidth;

					gdi::SolidBrush bsh_edittxt(style->m_col_text);
					gdi::SolidBrush bsh_bkgd(disabled ? gdi::Color((GdiARGB)gdi::Color::LightGray) : gdi::Color((GdiARGB)gdi::Color::WhiteSmoke));
					gdi::SolidBrush bsh_bkgdarw(gdi::Color((GdiARGB)gdi::Color::LightGray));
					gdi::Pen        pen_brdr(gdi::Color((GdiARGB)gdi::Color::Black), 0);

					// Draw the label
					Label::DrawItem(menu, gfx, di);

					// Draw the combo box
					wchar_t const* arw = L"6";
					gfx.FillRectangle(&bsh_bkgd, rect_ctrl);
					gfx.DrawRectangle(&pen_brdr, rect_ctrl);
					gfx.FillRectangle(&bsh_bkgdarw, rect_btn);
					gfx.DrawRectangle(&pen_brdr, rect_btn);
					gdi::RectF rect_arw; gfx.MeasureString(arw, 1, style->m_font_marks.get(), gdi::PointF(), &rect_arw);
					gdi::PointF pt(rect_btn.X + (rect_btn.Width - rect_arw.Width) * 0.5f, rect_btn.Y + (rect_btn.Height - rect_arw.Height) * 0.5f);
					gfx.DrawString(arw, 1, style->m_font_marks.get(), pt, &bsh_edittxt);

					// Draw the selected value text
					if (m_index < int(m_values.size()))
					{
						gdi::PointF pt(float(rect_ctrl.X + InnerMargin), float(rect_ctrl.Y + InnerMargin));
						gfx.DrawString(m_values[m_index].c_str(), int(m_values[m_index].size()), style->m_font_text.get(), pt, 0, &bsh_edittxt);
					}
				}
				void Selected(ContextMenu* menu, RECT const& rect, MENUITEMINFO const&)
				{
					StylePtr const& style = m_style ? m_style : menu->m_def_style;

					// Set the font to use in the control
					gdi::Graphics gfx(menu->m_hWnd); LOGFONTW logfont;
					FontPtr font = m_value_font ? m_value_font : style->m_font_text;
					if (font && font->GetLogFontW(&gfx, &logfont) == GdiOk) m_font = ::CreateFontIndirectW(&logfont);

					// Display the control
					CRect r = ToRECT(CtrlRect(ToRect(rect), style->m_margin_left));
					DoModal(menu->m_hWnd, (LPARAM)&r);
				}

				BEGIN_MSG_MAP(Combo)
					MSG_WM_INITDIALOG(OnInitDialog)
					MSG_WM_COMMAND(OnCommand)
				END_MSG_MAP()
				BOOL OnInitDialog(CWindow, LPARAM lparam)
				{
					// 'rect' should be the position of the control in screen space
					CRect const& rect = *(CRect const*)lparam;

					CRect winrect = rect;
					AdjustWindowRect(&winrect, (DWORD)GetWindowLongPtr(GWL_STYLE), FALSE);
					SetWindowPos(0, &winrect, SWP_NOZORDER);

					CRect r; GetClientRect(&r);
					m_combo = ::CreateWindowExW(0, WC_COMBOBOXW, 0, WS_CHILD|WS_VISIBLE|WS_BORDER|CBS_DROPDOWNLIST|CBS_AUTOHSCROLL|CBS_HASSTRINGS, r.left, r.top, r.Width(), r.Height(), m_hWnd, 0, _AtlBaseModule.GetModuleInstance(), 0);
					if (!m_font.IsNull()) m_combo.SetFont(m_font);
					for (ValueCont::const_iterator i = m_values.begin(), iend = m_values.end(); i != iend; ++i)
						m_combo.AddString((LPCTSTR)i->c_str());

					m_combo.SetCurSel(m_index);
					return TRUE;
				}
				void OnCommand(UINT, int nID, CWindow)
				{
					switch (nID)
					{
					default: break;
					case IDCANCEL: EndDialog(0); break;
					case IDOK:
						m_index = m_combo.GetCurSel();
						EndDialog(0);
						break;
					}
				}
			};
#endif
*/
/*

		protected:

			pr::GdiPlus   m_gdip_ctx; // Ensure the dlls have been loaded
			ItemCont      m_items;    // The items in this menu
			gui::Menu m_root;     // The handle to the root menu
			Selection     m_sel;      // The menu item that was selected

			int ApplySelection()
			{
				if (!m_sel.m_menu || (m_sel.m_flags & (MF_POPUP|MF_DISABLED|MF_GRAYED)) != 0)
					return 0;

				MenuItemInfo mii; mii.fMask = MIIM_DATA;
				GetMenuItemInfo(m_sel.m_menu, m_sel.m_id, TRUE, &mii);
				IContextMenuItem* item = static_cast<IContextMenuItem*>((void*)(mii.dwItemData));
				item->Selected(*this, m_sel.m_rect, mii);
				return item->GetId();
			}

			// IContextMenuItem interface implementation
			//int GetId() const
			//{
			//	return m_label.GetId();
			//}
			void AddToMenu(ContextMenu& menu, int) override
			{
				// Link up the submenu items
				int index = 0;
				for (auto& item : m_items)
					item->AddToMenu(*this, index++);

				auto cmi = static_cast<IContextMenuItem*>(this);
				::AppendMenuW(menu.m_root, MF_POPUP|MF_OWNERDRAW, UINT_PTR(m_root.m_menu), LPCWSTR(cmi));
			}

		public:

			StylePtr m_def_style; // A default style for when 'm_style' pointers are null
			//Label    m_label;     // The text for this menu (used when added as a submenu)

			ContextMenu(HWND parent, TCHAR const* text = _T("Menu"), int check_state = 0, StylePtr style = StylePtr())
				:Form<ContextMenu>(_T("menu"), parent, 0, 0, 0, 0, DS_CENTER|DS_SHELLFONT|WS_VISIBLE|WS_GROUP|WS_POPUP, 0, -1, "ctx-menu")
				,IContextMenuItem(0)
				,m_gdip_ctx()
				,m_items()
				,m_root(::CreatePopupMenu())
				,m_def_style(new ContextMenuStyle())
				//,m_label(text, id, check_state, style)
				,m_sel()
			{}

			// Add a context menu item to this menu
			void AddItem(ItemPtr item)
			{
				// Note: item ID's for the menu items are indices.
				item->AddToMenu(*this, int(m_items.size()));
				m_items.push_back(item);
			}

			// Add a context menu item to this menu, 'item' *MUST* be allocated using 'new'
			template <typename Item> Item& AddItem(ItemPtr item)
			{
				AddItem(item);
				return static_cast<Item&>(*item);
			}

			// Add an item by raw pointer (held as a shared pointer)
			template <typename Item> Item& AddItem(Item* item)
			{
				return AddItem<Item>(std::shared_ptr<Item>(item));
			}

		};
*/
		}
		#endif
	}
}