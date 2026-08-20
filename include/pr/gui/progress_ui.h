//***************************************************************************************************
// Progress Dialog
//  Copyright (c) Rylogic Ltd 2014
//***************************************************************************************************
// Self contained progress dialog and background thread
// No resource files, etc, needed. See unit tests for usage.

#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cassert>
#include <algorithm>
#include "pr/gui/wingui.h"

namespace pr::gui
{
	class ProgressUI : public Form
	{
		using Lock = std::unique_lock<std::timed_mutex>;
		struct State
		{
			enum class EMask
			{
				None  = 0,
				Title = 1 << 0,
				Desc  = 1 << 1,
				PC    = 1 << 2,
				allow_bitops,
			};
			HWND         m_hwnd;   // The hwnd of this progress window
			std::wstring m_title;  // The title bar text
			std::wstring m_desc;   // The description text
			float        m_pc;     // Percent complete
			EMask        m_mask;

			State(HWND hwnd, wchar_t const* title = nullptr, wchar_t const* desc = nullptr, float pc = -1.0f)
				:m_hwnd(hwnd)
				,m_title(title ? title : L"")
				,m_desc(desc ? desc : L"")
				,m_pc(pc)
				,m_mask(
					(title ? EMask::Title : EMask::None) |
					(desc  ? EMask::Desc  : EMask::None) |
					(pc!=-1? EMask::PC    : EMask::None))
			{}
			State& operator = (State const& rhs)
			{
				if (this != &rhs)
				{
					if (AllSet(rhs.m_mask, EMask::Title)) m_title = rhs.m_title;
					if (AllSet(rhs.m_mask, EMask::Desc )) m_desc  = rhs.m_desc;
					if (AllSet(rhs.m_mask, EMask::PC   )) m_pc    = rhs.m_pc;
					m_mask = m_mask | rhs.m_mask;
				}
				return *this;
			}
		};
		enum { IDC_TEXT_DESC=1000, IDC_PROGRESS_BAR, DefW=480, DefH=180};
		enum { WM_PROGRESS_UPDATE = WM_USER_BASE+1, WM_WORKER_COMPLETE, };

		Label                       m_lbl_desc;  // The progress description
		ProgressBar                 m_bar;       // The progress bar
		Button                      m_btn;       // The cancel button
		State                       m_state;     // Progress dialog UI state
		bool                        m_done;      // Task complete flag, set by the worker
		bool                        m_cancel;    // Cancel signalled flag, set by this thread
		std::exception_ptr          m_exception; // An exception thrown by the task
		bool                        m_shown;     // True when the dialog is visible
		std::timed_mutex            m_mutex;     // Sync
		std::condition_variable_any m_cv;        // Sync for flags
		std::thread                 m_worker;    // The worker thread

	public:

		enum ECancelFlags
		{
			NonBlocking        = 0,
			BlockTillCancelled = 1 << 0,
			OptionalCancel     = 1 << 1,
			allow_bitops
		};

		struct Params :Form::Params<Params>
		{
			using this_type = typename Params::this_type;

			wchar_t const* m_desc;

			Params()
				:m_desc()
			{
				wndclass(RegisterWndClass<ProgressUI>())
					.name("progress-ui")
					.wh(360, 200)
					.start_pos(EStartPosition::CentreParent);
			}
			this_type& desc(wchar_t const* d)
			{
				m_desc = d;
				return me();
			}
		};

		ProgressUI()
			:ProgressUI(Params())
		{}
		explicit ProgressUI(Params const& p)
			:Form(p)
			,m_lbl_desc(Label      ::Params<>().parent(this_).name("desc"  ).id(IDC_TEXT_DESC).text(p.m_desc))
			,m_bar     (ProgressBar::Params<>().parent(this_).name("bar"   ).id(IDC_PROGRESS_BAR))
			,m_btn     (Button     ::Params<>().parent(this_).name("cancel").id(IDCANCEL).text(L"Cancel").def_btn())
			,m_state(nullptr, p.m_text, p.m_desc, 0.0f)
			,m_done(false)
			,m_cancel(false)
			,m_exception()
			,m_shown()
			,m_mutex()
			,m_cv()
			,m_worker()
		{
			m_dialog_result = EDialogResult::Ok;
			m_btn.Click += [&](Button&,EmptyArgs const&)
			{
				if (Cancel(OptionalCancel|BlockTillCancelled))
					Close();
			};
		}
		~ProgressUI()
		{
			Close();
		}

		// Construct the dialog starting the worker thread immediately.
		// 'func' should have 'ProgressUI*' as the first parameter
		template <typename Func, typename... Args>
		ProgressUI(wchar_t const* title, wchar_t const* desc, Func&& func, Args&&... args)
			:ProgressUI(Params().title(title).desc(desc))
		{
			StartWorker(title, desc, std::forward<Func>(func), std::forward<Args>(args)...);
		}

		// Execute a work function in a different thread while displaying a non-modal dialog.
		// 'func' should have 'ProgressUI*' as the first parameter
		template <typename Func, typename... Args>
		void Show(wchar_t const* title, wchar_t const* desc, Func&& func, Args&&... args)
		{
			CreateHandle();

			Text(title);
			m_lbl_desc.Text(desc);
			StartWorker(title, desc, std::forward<Func>(func), std::forward<Args>(args)...);
			Form::ShowInternal(SW_SHOW);
		}

		// Execute a work function in a different thread while displaying the modal dialog
		// Returns IDOK if the dialog completed, IDCANCEL if the operation was cancelled
		EDialogResult ShowDialog(WndRefC parent, int delay_ms)
		{
			// Wait for up to 'delay_ms' in case no dialog is needed
			auto done = false;
			{
				Lock lock(m_mutex);
				done = m_cv.wait_for(lock, std::chrono::milliseconds(delay_ms), [&]{ return m_done; });
			}

			// If not done yet, show the dialog
			if (!done)
				Form::ShowDialogInternal(parent);

			// Ensure the thread has ended
			BlockTillWorkerDone();

			// Return the result
			if (m_dialog_result == EDialogResult::Abort)
				std::rethrow_exception(m_exception);

			return m_dialog_result;
		}
		EDialogResult ShowDialog(WndRefC parent) override
		{
			return ShowDialog(parent, 0);
		}

		// Called by the worker thread to update the UI or by callers to set the progress state
		bool Progress(float pc = -1.0f, wchar_t const* desc = nullptr, wchar_t const* title = nullptr)
		{
			// Try to lock to update the state, skip if can't lock
			Lock lock(m_mutex, std::chrono::milliseconds::zero());
			if (!lock.owns_lock())
				return true;

			// Update the state
			m_state = State(m_state.m_hwnd, title, desc, pc);

			// If the owner window is visible, send progress update
			if (m_state.m_hwnd)
				::PostMessageW(m_state.m_hwnd, WM_PROGRESS_UPDATE, 0, 0);

			return !m_cancel;
		}

		// Cancel the background thread, with optional cancel-the-cancel event
		bool Cancel(ECancelFlags flags = NonBlocking)
		{
			// Query to cancel the 'cancel'
			if (flags & ECancelFlags::OptionalCancel)
			{
				CancelEventArgs args;
				OnCancelling(args);
				if (args.m_cancel)
					return false;
			}

			// Cancelling was not cancelled, so cancel...
			{
				Lock lock(m_mutex);
				m_cancel = true;
				m_cv.notify_all();
			}

			// Wait till the thread exits
			if (flags & ECancelFlags::BlockTillCancelled)
				BlockTillWorkerDone();

			return true;
		}

		// Close the form, cancelling the worker thread if necessary
		bool Close()
		{
			// Don't close the window until the task has exited
			Cancel(BlockTillCancelled);
			return Form::Close(m_dialog_result);
		}

		// An event raised when the cancel button is hit. Handlers can opt to not cancel.
		EventHandler<ProgressUI&, CancelEventArgs&> Cancelling;

	protected:

		// Message map function
		bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
		{
			switch (message)
			{
			case WM_PROGRESS_UPDATE:
				#pragma region
				{
					// Lock so we can use m_state.
					// Holding the lock doesn't block the background thread it just
					// doesn't update m_state while it's locked
					Lock lock(m_mutex);

					// The window is initially created "hidden" (actually zero sized)
					// On the first call to update, make the window the correct size and style
					auto rect = ScreenRect();
					if (rect.width() * rect.height() == 0)
					{
						::MoveWindow(m_hwnd, rect.left, rect.top, DefW, DefH, TRUE);
						CenterWindow(GetParent(m_hwnd));
						Visible(true);
					}

					// Update the title
					if (AllSet(m_state.m_mask, State::EMask::Title))
						::SetWindowTextW(m_hwnd, m_state.m_title.c_str());

					// Update the description text
					if (AllSet(m_state.m_mask, State::EMask::Desc))
						::SetWindowTextW(m_lbl_desc, m_state.m_desc.c_str());

					// Update the percent complete
					if (AllSet(m_state.m_mask, State::EMask::PC))
					{
						// Use marquee style if pc is out of range
						auto bar_style = ::GetWindowLongPtrW(m_bar, GWL_STYLE);
						if (m_state.m_pc <= 0 || m_state.m_pc > 1.0f)
						{
							if ((bar_style & PBS_MARQUEE) == 0)
							{
								::SetWindowLongPtrW(m_bar, GWL_STYLE, bar_style|PBS_MARQUEE);
								m_bar.Marquee(true, 30);
							}
						}
						else
						{
							if ((bar_style & PBS_MARQUEE) != 0)
							{
								::SetWindowLongPtrW(m_bar, GWL_STYLE, bar_style & ~PBS_MARQUEE);
								m_bar.Marquee(false);
							}
							m_bar.Range(0, 100);
							m_bar.Pos(static_cast<int>(m_state.m_pc * 100));
						}
					}
					Invalidate();
					break;
				}
				#pragma endregion
			case WM_WORKER_COMPLETE:
				#pragma region
				{
					Close();
					return true;
				}
				#pragma endregion
			}
			return Form::ProcessWindowMessage(hwnd, message, wparam, lparam, result);
		}

		// Handlers
		virtual void OnCreate(CreateStruct const& cs) override
		{
			Form::OnCreate(cs);

			{
				Lock lock(m_mutex);
				m_state.m_hwnd = m_hwnd;
				Text(m_state.m_title);
				m_lbl_desc.Text(m_state.m_desc);
			}

			// Layout the dialog
			OnLayout(ClientRect(true));
		}
		virtual void OnDestroy() override
		{
			// In case of abnormal shutdown, don't close the window until the task has exited
			Cancel(BlockTillCancelled);
		}
		virtual void OnLayout(Rect const& client)
		{
			auto Clamp = [](Rect& rect)
			{
				if (rect.right  < rect.left) rect.right  = rect.left;
				if (rect.bottom < rect.top ) rect.bottom = rect.top;
				return rect;
			};

			Rect r;
			const int btn_w = 80, btn_h = 24, prog_h = 18, sp = 2;

			// Position the description
			r = Rect(client.left, client.top, client.right, client.bottom - std::max(btn_h, prog_h) - sp);
			m_lbl_desc.ParentRect(Clamp(r));

			// Position the progress bar
			r = Rect(client.left, client.bottom - abs(btn_h-prog_h)/2 - prog_h, client.right - btn_w - sp, client.bottom - abs(btn_h-prog_h)/2);
			m_bar.ParentRect(Clamp(r));

			// Position the cancel button
			r = Rect(client.right - btn_w, client.bottom - btn_h, client.right, client.bottom);
			m_btn.ParentRect(Clamp(r));
		}
		virtual void OnWindowPosChange(WindowPosEventArgs const& args) override
		{
			// Layout the dialog whenever it resizes
			if (!args.m_before && args.IsResize() && !args.Iconic())
				OnLayout(ClientRect(true));
		}
		virtual void OnCancelling(CancelEventArgs& args)
		{
			Cancelling(*this, args);
		}

		// Start the worker thread running
		template <typename Func, typename... Args>
		void StartWorker(wchar_t const* title, wchar_t const* desc, Func&& func, Args&&... args)
		{
			// Stop first ... if needed
			Cancel(BlockTillCancelled);

			// Reset flags
			m_done = false;
			m_cancel = false;
			m_state = State(m_hwnd, title, desc);

			// Start the worker
			m_worker = std::thread([&]
			{
				try
				{
					// Run the task
					// Pass the dialog to the task function so that it can update progress.
					// Passed as a pointer so that users have the option of passing nullptr
					std::bind(std::forward<Func>(func), this, std::forward<Args>(args)...)();

					// Notify task complete
					Lock lock(m_mutex);
					m_done = true;
					m_dialog_result = m_cancel ? EDialogResult::Cancel : EDialogResult::Ok;
					m_cv.notify_all();
				}
				catch (...)
				{
					Lock lock(m_mutex);
					m_done = true;
					m_dialog_result = EDialogResult::Abort;
					m_exception = std::current_exception();
					m_cv.notify_all();
				}
				Progress(1.0f);
				::PostMessageW(m_hwnd, WM_WORKER_COMPLETE, 0, 0);
			});
		}

		// Blocks until the worker thread exits
		void BlockTillWorkerDone()
		{
			if (m_worker.joinable())
				m_worker.join();
		}
	};
}

// Add a manifest dependency on common controls version 6
#if defined _M_IX86
  #pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_IA64
  #pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='ia64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
  #pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
  #pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/common/fmt.h"

namespace pr
{
	namespace unittests
	{
		PRUnitTest(pr_gui_progress_dlg, Quick)
		{
			using namespace pr::gui;
			int arg = 42;

			ProgressUI dlg(L"Progressing...", L"This is a progress dialog",
				[](ProgressUI* dlg, int a)
				{
					for (int i = 0; i != a; ++i)
					{
						if (!dlg->Progress((i + 1.0f) / (float)a, pr::FmtS(L"Processing index %d", i), nullptr))
							return;
						Sleep(50);
					}
				}, arg);

			auto r = dlg.ShowDialog(nullptr);
			PR_EXPECT(r == pr::gui::EDialogResult::Ok || r == pr::gui::EDialogResult::Cancel);
		}
	}
}
#endif

