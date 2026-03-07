//***************************************************************************************************
// Ldr Object Manager
//  Copyright (c) Rylogic Ltd 2009
//***************************************************************************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/gui/wingui.h"

namespace pr::rdr12::ldraw
{
	// Forwards
	class ObjectManagerUI;

	// User interface for managing LdrObjects
	// LdrObject is completely unaware that this class exists.
	// Note: this object does not add references to LdrObjects
	class ObjectManagerUI :public gui::Form
	{
		gui::StatusBar m_status;
		gui::Button    m_btn_expand;
		gui::Button    m_btn_collapse;
		gui::Button    m_btn_filter;
		gui::TextBox   m_tb_filter;
		gui::Splitter  m_split;
		gui::TreeView  m_tree;
		gui::ListView  m_list;
		bool               m_expanding;         // True during a recursive expansion of a node in the tree view
		bool               m_selection_changed; // Dirty flag for the selection bbox/object
		bool               m_suspend_layout;    // True while a block of changes are occurring.

		enum class EColumn
		{
			Name,
			LdrType,
			Colour,
			Visible,
			Wireframe,
			Volume,
			CtxtId,
			NumberOf,
		};

		// UI data for an ldr object
		struct UIData
		{
			HTREEITEM m_tree_item;
			int       m_list_item;

			UIData()
				:m_tree_item(gui::TreeView::NoItem())
				,m_list_item(gui::ListView::NoItem())
			{}

			// Return the 'uidata' for an object
			static UIData* get(LdrObject& obj) { return &obj.m_user_data.get<UIData>(); }
			static UIData* get(LdrObject* obj) { return obj ? get(*obj) : nullptr; }
		};
		enum { ID_BTN_EXPAND = 100, ID_BTN_COLLAPSE, ID_BTN_FILTER, ID_TB_FILTER };
		static Params<> Params(HWND parent)
		{
			return Form::Params<>().wndclass(RegisterWndClass<ObjectManagerUI>())
				.name("ldr-object-manager").title(L"Scene Object Manager").wh(430, 380)
				.icon_bg((HICON)::SendMessageW(parent, WM_GETICON, ICON_BIG, 0))
				.icon_sm((HICON)::SendMessageW(parent, WM_GETICON, ICON_SMALL, 0))
				.start_pos(gui::EStartPosition::CentreParent)
				.parent(parent).hide_on_close(true).pin_window(true);
		};

	public:

		ObjectManagerUI(HWND parent)
			:Form(Params(parent))
			,m_status      (gui::StatusBar::Params<>().parent(this_         ).name("status"      ).xy(0,-1).wh(Fill,gui::StatusBar::DefH).dock(EDock::Bottom))
			,m_btn_expand  (gui::Button   ::Params<>().parent(this_         ).name("btn-expand"  ).id(ID_BTN_EXPAND).xy(0,0).wh(20,20).text(L"+").margin(2).anchor(EAnchor::TopLeft))
			,m_btn_collapse(gui::Button   ::Params<>().parent(this_         ).name("btn-collapse").id(ID_BTN_COLLAPSE).xy(Left|RightOf|ID_BTN_EXPAND, 0).wh(20,20).text(L"-").margin(2).anchor(EAnchor::TopLeft))
			,m_btn_filter  (gui::Button   ::Params<>().parent(this_         ).name("btn-filter"  ).id(ID_BTN_FILTER).xy(-1,0).wh(60,20).text(L"Filter").margin(2).anchor(EAnchor::TopRight))
			,m_tb_filter   (gui::TextBox  ::Params<>().parent(this_         ).name("tb-filter"   ).id(ID_TB_FILTER).xy(0,0).wh(Fill, 18).margin(50,3,64,3).anchor(EAnchor::LeftTopRight))
			,m_split       (gui::Splitter ::Params<>().parent(this_         ).name("split"       ).xy(0,Top|BottomOf|ID_TB_FILTER).wh(Fill,Fill).margin(3).anchor(EAnchor::All).vertical())
			,m_tree        (gui::TreeView ::Params<>().parent(&m_split.Pane0).name("tree"        ).margin(0).border().dock(EDock::Fill))
			,m_list        (gui::ListView ::Params<>().parent(&m_split.Pane1).name("list"        ).margin(0).border().dock(EDock::Fill).mode(gui::ListView::EViewType::Report))
			,m_expanding(false)
			,m_selection_changed(true)
			,m_suspend_layout(false)
		{
			CreateHandle();
			m_list.InsertColumn((int)EColumn::Name     , gui::ListView::ColumnInfo(L"Name"       ).width(100));
			m_list.InsertColumn((int)EColumn::LdrType  , gui::ListView::ColumnInfo(L"Object Type").width(100));
			m_list.InsertColumn((int)EColumn::Colour   , gui::ListView::ColumnInfo(L"Colour"     ).width(100));
			m_list.InsertColumn((int)EColumn::Visible  , gui::ListView::ColumnInfo(L"Visible"    ).width(100));
			m_list.InsertColumn((int)EColumn::Wireframe, gui::ListView::ColumnInfo(L"Wireframe"  ).width(100));
			m_list.InsertColumn((int)EColumn::Volume   , gui::ListView::ColumnInfo(L"Volume"     ).width(100));
			m_list.InsertColumn((int)EColumn::CtxtId   , gui::ListView::ColumnInfo(L"CtxtId"     ).width(100));
		}
		ObjectManagerUI(ObjectManagerUI const&) = delete;
		ObjectManagerUI& operator=(ObjectManagerUI const&) = delete;

		// Get/Set settings for the object manager window
		std::string Settings() const
		{
			return std::string();
		}
		void Settings(std::string const& settings) 
		{
			(void)settings;
		}

		// Repopulate the dialog with the collection 'cont'
		template <typename ObjectCont> void Populate(ObjectCont const& cont)
		{
			struct L {
			static LdrObject* pointer(LdrObjectPtr p) { return p.m_ptr; }
			static LdrObject* pointer(LdrObject*   p) { return p; }
			};
			BeginPopulate();
			for (auto obj : cont) Add(L::pointer(obj));
			EndPopulate();
		}

		// Begin repopulating the dialog
		void BeginPopulate()
		{
			m_tree.Clear();
			m_list.Clear();
		}

		// Finished populating the dialog
		void EndPopulate()
		{
			for (int i = 0, iend = int(m_list.ColumnCount()); i != iend; ++i)
				m_list.ColumnWidth(i, LVSCW_AUTOSIZE);
		}
			
		// Add a root level object recursively to the dialog
		void Add(LdrObject* obj)
		{
			Add(obj, nullptr);
		}

		// Return the number of selected objects
		size_t SelectedCount() const
		{
			return m_list.SelectedCount();
		}

		// Enumerate the selected items
		// 'iter' is an 'in/out' parameter, initialise it to -1 for the first call
		// Returns 'nullptr' after the last selected item
		LdrObject const* EnumSelected(int& iter) const
		{
			iter = m_list.NextItem(LVNI_SELECTED, iter);
			if (iter == -1) return nullptr;
			return &GetLdrObject(iter);
		}

		// Tri-state
		enum class ETriState { Off, On, Toggle };

		// Event fired from the UI when the selected object changes
		EventHandler<ObjectManagerUI&, EmptyArgs const&> SelectionChanged;

		// Called when one or more objects have changed state
		// The LdrObject is the object that has changed. If null, then more than one object has changed
		EventHandler<ObjectManagerUI&, LdrObject*> Refresh;

		// Raised whenever settings have changed
		EventHandler<ObjectManagerUI&, EmptyArgs const&> SettingsChanged;

	private:

		using TreeItem = gui::TreeView::HITEM;
		using ListItem = gui::ListView::HITEM;

		// Return the LdrObject associated with a tree item or list item
		LdrObject const& GetLdrObject(TreeItem item) const { return *m_tree.UserData<LdrObject>(item); }
		LdrObject&       GetLdrObject(TreeItem item)       { return *m_tree.UserData<LdrObject>(item); }
		LdrObject const& GetLdrObject(ListItem item) const { return *m_list.UserData<LdrObject>(item); }
		LdrObject&       GetLdrObject(ListItem item)       { return *m_list.UserData<LdrObject>(item); }

		// Recursively add 'obj' and its children to the tree and list control
		void Add(LdrObject* obj, LdrObject* prev, bool last_call = true)
		{
			assert(obj != nullptr && "Attempting to add a null object to the UI");
			assert((!obj->m_parent || UIData::get(*obj->m_parent)->m_tree_item != gui::TreeView::NoItem()) && "Parent is not in the tree");

			// Ensure the object has UI data
			obj->m_user_data.get<UIData>(this) = UIData();

			// Get UI data for the related objects
			prev = prev ? prev : PrevSibbling(obj);
			auto obj_uidata    = UIData::get(obj);
			auto prev_uidata   = UIData::get(prev);
			auto parent_uidata = UIData::get(obj->m_parent);

			auto obj_name = Widen(obj->m_name);

			// Add the item to the tree
			obj_uidata->m_tree_item = m_tree.InsertItem(
				gui::TreeView::ItemInfo(obj_name.c_str()),
				obj->m_parent ? parent_uidata->m_tree_item : TVI_ROOT,
				prev          ? prev_uidata->m_tree_item   : TVI_LAST);

			// Save a back reference pointer to this object in the tree
			if (obj_uidata->m_tree_item != gui::TreeView::NoItem())
				m_tree.UserData(obj_uidata->m_tree_item, obj);
			else {} // todo: Report errors, without spamming the user...

			// If 'obj' is a top level object, then add it to the list
			if (obj->m_parent == nullptr)
			{
				obj_uidata->m_list_item = m_list.InsertItem(gui::ListView::ItemInfo(obj_name.c_str(), (int)m_list.ItemCount()));
				if (obj_uidata->m_list_item == gui::ListView::NoItem()) {}
				// todo: Report errors, without spamming the user...
			}
			// Otherwise, if 'prev' is visible in the list then display 'obj' in the list as well
			else if (prev && prev_uidata->m_list_item != gui::ListView::NoItem())
			{
				obj_uidata->m_list_item = m_list.InsertItem(gui::ListView::ItemInfo(obj_name.c_str()).index(prev_uidata->m_list_item + 1));
				if (obj_uidata->m_list_item == gui::ListView::NoItem()) {}
				// todo: Report errors, without spamming the user...
			}
			// Otherwise, leave out of the list
			else
			{
				obj_uidata->m_list_item = gui::ListView::NoItem();
			}
			if (obj_uidata->m_list_item != gui::ListView::NoItem())
			{
				// Save a pointer to this object in the list
				m_list.UserData(obj_uidata->m_list_item, obj);

				// Set the other columns in the list
				UpdateListItem(*obj, false);
			}

			// Add the children
			LdrObject* p = nullptr;
			for (auto child : obj->m_child)
			{
				Add(child.m_ptr, p, false);
				p = child.m_ptr;
			}

			// On leaving the last recursive call, fix up the references
			if (last_call)
				FixListCtrlReferences(obj_uidata->m_list_item);
		}

		// Return the sibling immediately before 'obj' in 'obj->m_parent' (or nullptr)
		static LdrObject* PrevSibbling(LdrObject* obj)
		{
			assert(obj != nullptr);

			// No parent, then 'obj' isn't a child
			if (!obj->m_parent)
				return nullptr;

			// Search the children for the object prior to 'obj'
			auto& children = obj->m_parent->m_child;
			for (auto i = std::begin(children), iend = std::end(children); i != iend; ++i)
			{
				if (i->m_ptr != obj) continue;
				if (i == std::begin(children)) break;
				return (*(--i)).m_ptr;
			}
			return nullptr;
		}

		// Update the displayed properties in the list
		void UpdateListItem(LdrObject& object, bool recursive)
		{
			auto obj_uidata = UIData::get(object);
			if (obj_uidata->m_list_item == gui::ListView::NoItem()) return;

			auto info = gui::ListView::ItemInfo(obj_uidata->m_list_item);
			m_list.Item(info.subitem((int)EColumn::Name     ).text(Widen(object.m_name).c_str()));
			m_list.Item(info.subitem((int)EColumn::LdrType  ).text(Enum<ELdrObject>::ToStringW(object.m_type)));
			m_list.Item(info.subitem((int)EColumn::Colour   ).text(pr::FmtS(L"%8.8X", object.m_colour.argb)));
			m_list.Item(info.subitem((int)EColumn::Visible  ).text(object.Visible() ? L"Visible" : L"Hidden"));
			m_list.Item(info.subitem((int)EColumn::Wireframe).text(object.Wireframe() ? L"Wireframe" : L"Solid"));
			m_list.Item(info.subitem((int)EColumn::Volume   ).text(pr::FmtS(L"%3.3f", Volume(object.BBoxMS(EBBoxFlags::None)))));
			m_list.Item(info.subitem((int)EColumn::CtxtId   ).text(pr::FmtS(L"%d", object.m_context_id)));

			if (!recursive) return;
			for (auto& child : object.m_child)
				UpdateListItem(*child.m_ptr, recursive);
		}

		// For each object in the list from 'start_index' to the end, set the list index
		// in the object UIData. The list control uses contiguous memory so we have to do
		// this whenever objects are inserted/deleted from the list
		void FixListCtrlReferences(int start_index)
		{
			// 'start_index'==-1 means all list items
			if (start_index < 0) start_index = 0;
			for (int i = start_index, iend = (int)m_list.ItemCount(); i < iend; ++i)
				UIData::get(GetLdrObject(i))->m_list_item = i;
		}

		// Remove selection from the tree and list controls
		void SelectNone()
		{
			for (auto i = m_list.NextItem(LVNI_SELECTED); i != -1; i = m_list.NextItem(LVNI_SELECTED ,i))
				m_list.Item(gui::ListView::ItemInfo(i).state(0, LVIS_SELECTED));
		}

		// Select an ldr object
		void SelectLdrObject(LdrObject& object, bool make_visible)
		{
			auto obj_uidata = UIData::get(object);

			// Select in the tree
			m_tree.ItemState(obj_uidata->m_tree_item, TVIS_SELECTED, TVIS_SELECTED);
			if (make_visible) m_tree.EnsureVisible(obj_uidata->m_tree_item);

			// Select in the list and make visible
			if (obj_uidata->m_list_item != gui::ListView::NoItem())
			{
				m_list.ItemState(obj_uidata->m_list_item, LVIS_SELECTED, LVIS_SELECTED);
				if (make_visible) m_list.EnsureVisible(obj_uidata->m_list_item, FALSE);
			}

			// Flag the selection data as invalid
			m_selection_changed = true;
			SelectionChanged(*this);
		}

		// Invert the selection from the tree and list controls
		void InvSelection()
		{
			for (int i = m_list.NextItem(LVNI_ALL); i != -1; i = m_list.NextItem(LVNI_ALL, i))
				m_list.ItemState(i, ~m_list.ItemState(i, LVIS_SELECTED), LVIS_SELECTED);
		}

		// Set the visibility of the currently selected objects
		void SetVisibilty(ETriState state, bool include_children)
		{
			for (int i = m_list.NextItem(LVNI_SELECTED); i != -1; i = m_list.NextItem(LVNI_SELECTED, i))
			{
				auto& object = GetLdrObject(i);
				object.Visible(state == ETriState::Off ? false : state == ETriState::On ? true : !object.Visible(), include_children ? "" : nullptr);
				UpdateListItem(object, include_children);
			}
			Refresh(*this, nullptr);
		}

		// Set wireframe for the currently selected objects
		void SetWireframe(ETriState state, bool include_children)
		{
			for (int i = m_list.NextItem(LVNI_SELECTED); i != -1; i = m_list.NextItem(LVNI_SELECTED, i))
			{
				auto& object = GetLdrObject(i);
				object.Wireframe(state == ETriState::Off ? false : state == ETriState::On ? true : !object.Wireframe(), include_children ? "" : nullptr);
				UpdateListItem(object, include_children);
			}
			Refresh(*this, nullptr);
		}

		// Handle a key press in either the list or tree view controls
		void OnKey(gui::KeyEventArgs& args) override
		{
			Form::OnKey(args);
			if (args.m_handled) return;
			if (!args.m_down) return;
			switch (tolower(args.m_vk_key))
			{
			case VK_ESCAPE:
				{
					Close();
					args.m_handled = true;
					return;
				}
			case 'a':
				{
					if (KeyDown(VK_CONTROL))
					{
						SelectNone();
						InvSelection();
						args.m_handled = true;
						return;
					}
					//else OnBnClickedButtonToggleAlpha();
					break;
				}
			case 'w':
				{
					SetWireframe(ETriState::Toggle, !pr::KeyDown(VK_SHIFT));
					args.m_handled = true;
					return;
				}
			case ' ':
				{
					SetVisibilty(ETriState::Toggle, !pr::KeyDown(VK_SHIFT));
					args.m_handled = true;
					return;
				}
			case VK_DELETE:
				{
					//pr::events::Send(LdrObjMgr_DeleteObjectRequest());
					break;
				}
			case VK_F6:
				{
					::SetFocus(m_tb_filter);
					m_tb_filter.SelectAll();
					args.m_handled = true;
					return;
				}
			}
			return;
		}

		// Add/Remove items from the list view based on the filter
		// If the filter is empty the list is re-populated
		void ApplyFilter()
		{
			// If the filter edit box is not empty then remove all that aren't selected
			if (m_tb_filter.TextLength() != 0)
			{
				for (int i = (int)m_list.ItemCount() - 1; i != -1; --i)
				{
					// Delete all non-selected items
					if ((m_list.ItemState(i, LVIS_SELECTED) & LVIS_SELECTED) == 0)
					{
						UIData::get(GetLdrObject(i))->m_list_item = gui::ListView::NoItem();
						m_list.DeleteItem(i);
					}
				}
				FixListCtrlReferences(0);
			}
			// Else, remove all items from the list and re-add them based on what's displayed in the tree
			else
			{
				// Remove all items from the list
				for (int i = m_list.NextItem(LVNI_ALL); i != -1; i = m_list.NextItem(LVNI_ALL, i))
					UIData::get(GetLdrObject(i))->m_list_item = gui::ListView::NoItem();
				m_list.Clear();

				// Re-add items based on what's displayed in the tree
				int list_position = 0;
				for (TreeItem i = m_tree.NextItem(gui::TreeView::Root); i != 0; i = m_tree.NextItem(gui::TreeView::NextVisible, i), ++list_position)
				{
					auto& object = GetLdrObject(i);
					auto name = Widen(object.m_name);

					// Add a list item for this tree item
					UIData::get(object)->m_list_item = list_position;
					m_list.InsertItem(gui::ListView::ItemInfo(name.c_str()).index(list_position).user(&object));
					UpdateListItem(object, false);
				}
			}
		}

		// Recursively remove 'obj' and its children from the tree and list controls.
		// Note that objects are not deleted from the ObjectManager
		void Remove(LdrObject* obj, bool last_call = true)
		{
			auto obj_uidata = UIData::get(obj);
			if (obj_uidata == nullptr) return; // Object wasn't added so has no UIData
			int list_position = obj_uidata->m_list_item;

			// Recursively delete children in reverse order to prevent corrupting list control indices
			for (std::size_t c = obj->m_child.size(); c-- != 0;)
				Remove(obj->m_child[c].m_ptr, false);

			// If the object is in the list, remove it. We'll fix up the list
			// references after all children of 'obj' have been removed.
			if (obj_uidata->m_list_item != gui::ListView::NoItem())
			{
				m_list.DeleteItem(obj_uidata->m_list_item);
				obj_uidata->m_list_item = gui::ListView::NoItem();
			}

			// Remove it from the tree.
			m_tree.DeleteItem(obj_uidata->m_tree_item);
			obj_uidata->m_tree_item = gui::TreeView::NoItem();

			// Remove the UIData from the object
			obj->m_user_data.erase<UIData>();

			if (last_call)
				FixListCtrlReferences(list_position);
		}

		// Collapse 'object' and its children in 'tree'.
		// Remove 'object's children from the 'list'
		void Collapse(LdrObject* object)
		{
			CollapseRecursive(object);

			// Fix the indices of the remaining list members
			FixListCtrlReferences(UIData::get(object)->m_list_item);
		}

		// Recursively collapse objects in the tree.
		// Depth-first so that we can remove items from the list control at the same time
		void CollapseRecursive(LdrObject* object)
		{
			std::size_t child_count = object->m_child.size();
			for (std::size_t c = child_count; c != 0; --c)
			{
				LdrObject* child = object->m_child[c - 1].m_ptr;
				CollapseRecursive(child);

				// Remove this child from the list control
				auto child_uidata = UIData::get(child);
				if (child_uidata->m_list_item != gui::ListView::NoItem())
				{
					m_list.DeleteItem(child_uidata->m_list_item);
					child_uidata->m_list_item = gui::ListView::NoItem();
				}
			}

			// Collapse this tree item
			m_tree.ExpandItem(UIData::get(object)->m_tree_item, gui::TreeView::Collapse);
		}

		// Expand 'object' in the tree and add its children to 'list'
		void Expand(LdrObject* object, bool recursive, bool& expanding)
		{
			// Calling tree.Expand causes notification messages to be sent,
			// Believe me, I've try to find a better solution, this is the best I could do
			// after several days :-/. But hey, it works.
			if (!expanding)
			{
				expanding = true;
				int list_position = UIData::get(object)->m_list_item + 1;
				ExpandRecursive(object, recursive, list_position);
				expanding = false;
			}

			// Fix the indices of the remaining list members
			FixListCtrlReferences(UIData::get(object)->m_list_item + 1);
		}

		// Expand this object. If 'all_children' is true, expand all of its children.
		// Add all children to the list control if the parent is in the list control.
		void ExpandRecursive(LdrObject* object, bool all_children, int& list_position)
		{
			std::size_t child_count = object->m_child.size();
			for (std::size_t c = 0; c != child_count; ++c)
			{
				auto child = object->m_child[c].m_ptr;

				// Add this child to the list control
				if (UIData::get(object)->m_list_item != gui::ListView::NoItem() &&
					UIData::get(child )->m_list_item == gui::ListView::NoItem())
				{
					auto name = Widen(child->m_name);
					UIData::get(child)->m_list_item = list_position;
					m_list.InsertItem(gui::ListView::ItemInfo(name.c_str()).index(list_position).user(child));
					UpdateListItem(*child, false);
					++list_position;
				}

				if (all_children)
					ExpandRecursive(child, all_children, list_position);
			}

			// Expand this tree item
			m_tree.ExpandItem(UIData::get(object)->m_tree_item, gui::TreeView::Expand);
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