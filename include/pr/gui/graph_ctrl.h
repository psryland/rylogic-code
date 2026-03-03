//***********************************************
// Graph Control
//  Copyright (c) Rylogic Ltd 2009
//***********************************************

#pragma once

#include <vector>
#include <algorithm>
#include <mutex>
#include <thread>
#include <cassert>

#include "pr/common/min_max_fix.h"
#include "pr/common/event_handler.h"
#include "pr/common/range.h"
#include "pr/maths/maths.h"
#include "pr/gui/gdiplus.h"
#include "pr/gui/context_menu.h"
#include "pr/gui/wingui.h"

namespace pr::gui
{
	// A default/example data source for the graph control
	struct GraphDatum
	{
		typedef double real;

		// Example Get/Set properties
		struct { real m_value;
		operator real () const        { return m_value; }
		real& operator = (real value) { return m_value = value; }
		} x, y;

		// Place holder for impersonating a readonly member
		struct {
		operator real () const { return 0; }
		} ylo, yhi;

		GraphDatum(real x_ = 0, real y_ = 0) { x = x_; y = y_; }
	};

	// A control for rendering a graph
	template <typename Elem = GraphDatum, typename real = double>
	struct GraphCtrl :Control
	{
		using MyType     = GraphCtrl<Elem, real>;
		using Graphics   = Gdiplus::Graphics;
		using Color      = Gdiplus::Color;
		using PointF     = Gdiplus::PointF;
		using RectF      = Gdiplus::RectF;
		using ARGB       = Gdiplus::ARGB;
		using SolidBrush = Gdiplus::SolidBrush;
		using Pen        = Gdiplus::Pen;
		using Font       = Gdiplus::Font;
		using Bitmap     = Gdiplus::Bitmap;
		using Matrix     = Gdiplus::Matrix;
		struct Point
		{
			real x, y;
			Point(real x_ = 0, real y_ = 0) :x(x_) ,y(y_) {}
			Point(pr::gui::Point const& pt) :x(real(pt.x)) ,y(real(pt.y)) {}
			Point(PointF const& pt) :x(real(pt.X)) ,y(real(pt.Y)) {}
		};

		struct RdrOptions
		{
			// Transform for positioning the graph title, offset from top centre
			Matrix TitleTransform;
				
			// Graph element Colours
			Color BkColour;        // Control background colour
			Color PlotBkColour;    // Plot background colour
			Color TitleColour;     // Title string colour
			Color AxisColour;      // Axis colour
			Color GridColour;      // Grid line colour
			Color SelectionColour; // Area selection colour

			// Graph margins
			int LeftMargin;
			int TopMargin;
			int RightMargin;
			int BottomMargin;

			// Fonts
			Font TitleFont;
			Font NoteFont;

			enum class EBorder { None, Single };
			EBorder Border;
			PointF PixelsPerTick;

			RdrOptions()
				:TitleTransform (1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f)
				,BkColour       (0xFF000000 | ::GetSysColor(COLOR_BTNFACE))
				,PlotBkColour   (ARGB(Color::WhiteSmoke))
				,TitleColour    (ARGB(Color::Black))
				,AxisColour     (ARGB(Color::Black))
				,GridColour     (ARGB(Color::MakeARGB(255,230,230,230)))
				,SelectionColour(ARGB(Color::MakeARGB(255,128,128,128)))
				,LeftMargin     (3)
				,TopMargin      (3)
				,RightMargin    (3)
				,BottomMargin   (3)
				,TitleFont      (L"tahoma", 18, Gdiplus::FontStyleBold)
				,NoteFont       (L"tahoma",  8, Gdiplus::FontStyleRegular)
				,Border         (EBorder::None)
				,PixelsPerTick  (30, 24)
			{}
		};
		struct Series
		{
			struct RdrOptions
			{
				enum class EPlotType  { Point, Line, Bar };

				bool      Visible;
				bool      DrawData;
				bool      DrawErrorBars;
				EPlotType PlotType;
				Color     PointColour;
				float     PointSize;
				Color     LineColour;
				float     LineWidth;
				Color     BarColour;
				float     BarWidth;
				Color     ErrorBarColour;
				bool      DrawMovingAvr;
				int       MAWindowSize;
				Color     MALineColour;
				float     MALineWidth;

				RdrOptions()
					:Visible       (true)
					,DrawData      (true)
					,DrawErrorBars (false)
					,PlotType      (EPlotType::Line)
					,PointColour   (0xff, 0x80, 0, 0xff)
					,PointSize     (5.0f)
					,LineColour    (0xff, 0, 0, 0xff)
					,LineWidth     (1.0f)
					,BarColour     (0xff, 0x80, 0, 0xff)
					,BarWidth      (0.8f)
					,ErrorBarColour(0x80, 0xff, 0, 0xff)
					,DrawMovingAvr (false)
					,MAWindowSize  (10)
					,MALineColour  (0xff, 0, 0, 0xFF)
					,MALineWidth   (3.0f)
				{}
				Color color() const
				{
					return
						PlotType == EPlotType::Point ? PointColour :
						PlotType == EPlotType::Line  ? LineColour :
						BarColour;
				}
			};
			using DataCont = std::vector<Elem>;
			using DataCIter = typename DataCont::const_iterator;

			std::wstring m_name;
			RdrOptions   m_opts;
			DataCont     m_values;

			Series(std::wstring const& name = L"")
				:m_name(name)
				,m_opts()
				,m_values()
			{}

			size_t size() const
			{
				return m_values.size();
			}
			DataCIter begin() const
			{
				return begin(m_values);
			}
			DataCIter end() const
			{
				return end(m_values);
			}
			Elem const& first() const
			{
				assert(size() != 0);
				return m_values.front();
			}
			Elem const& last() const
			{
				assert(size() != 0);
				return m_values.back();
			}
			Elem const& operator[](size_t i) const
			{
				return m_values[i];
			}
			Elem& operator[](size_t i)
			{
				return m_values[i];
			}

			// Return the range of indices that need to be considered when plotting from 'xmin' to 'xmax'
			// Note: in general, this range should include one point to the left of xmin and one to the right
			// of xmax so that line graphs plot a line up to the border of the plot area
			void IndexRange(real xmin, real xmax, size_t& imin, size_t& imax) const
			{
				Elem lwr(xmin, 0), upr(xmax, 0);
				auto cmpx = [](Elem const& l, Elem const& r){ return l.x < r.x; };
				auto i0 = std::lower_bound(std::begin(m_values), std::end(m_values), lwr, cmpx);
				auto i1 = std::upper_bound(i0                  , std::end(m_values), upr, cmpx);
				imin = i0 - std::begin(m_values); imin -= imin != 0;
				imax = i1 - std::begin(m_values); imax += imax != m_values.size();
			}

			// Apply an operation over the range of values by index [i0,i1)
			template <typename Op> void Values(size_t i0, size_t i1, Op op) const
			{
				for (auto i = i0; i != i1; ++i)
					op(m_values[i]);
			}
			template <typename Op> void Values(Op op) const
			{
				Values(0, m_values.size(), op);
			}

			// Apply an operation over the range of values by x-axis range [xmin,xmax]
			template <typename Op> void Values(real xmin, real xmax, Op op) const
			{
				size_t i0,i1;
				IndexRange(xmin, xmax, i0, i1);
				Values(i0, i1, op);
			}

			// Plot colour generator
			static Color Colour(int i)
			{
				using namespace Gdiplus;
				static const Color s_colours[] =
				{
					Color::Black     ,
					Color::Blue      , Color::Red         , Color::Green      ,
					Color::DarkBlue  , Color::DarkRed     , Color::DarkGreen  ,
					Color::LightBlue , Color::LightSalmon , Color::LightGreen ,
					Color::Yellow    , Color::Orange      , Color::Magenta    ,
					Color::Purple    , Color::Turquoise   ,
				};
				return s_colours[i % _countof(s_colours)];
			}
		};
		struct Axis
		{
			struct Range
			{
				real m_min;
				real m_max;

				Range() :m_min(0) ,m_max(1) {}
				Range(real mn, real mx) :m_min(mn) ,m_max(mx) {}
				real span() const   { return m_max - m_min; }
				real span(real s)   { auto c = centre(); m_min = c - s/2; m_max = c + s/2; return s; }
				real centre() const { return (m_min + m_max) / 2; }
				real centre(real c) { auto d = c - centre(); m_min += d; m_max += d; return c; }
			};
			struct RdrOptions
			{
				Matrix LabelTransform; // Offset transform from default label position
				Font   LabelFont;      // Font of axis label
				Font   TickFont;       // Font of tick labels
				Color  LabelColour;    // Colour of axis label
				Color  TickColour;     // Colour of tick labels
				int    TickLength;     // Length of axis ticks

				RdrOptions()
					:LabelTransform(1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f)
					,LabelFont     (L"tahoma", 14, Gdiplus::FontStyleRegular)
					,TickFont      (L"tahoma", 10, Gdiplus::FontStyleRegular)
					,LabelColour   (ARGB(Color::Black))
					,TickColour    (ARGB(Color::Black))
					,TickLength    (5)
				{}
			};

			using TickTextFunc = std::wstring (*)(real tick);
			static std::wstring ToText(real tick)
			{
				tick = std::round(tick * 1000) / 1000;
				wchar_t buf[20]; _snwprintf_s(buf, _countof(buf), L"%.4g", tick);
				return buf;
			}

			std::wstring m_label;
			RdrOptions   m_opts;
			Range        m_range;
			TickTextFunc m_tick_text;
			bool         m_allow_scroll;
			bool         m_allow_zoom;
			bool         m_lock_range;

			Axis()
				:m_label(L"Axis")
				,m_opts()
				,m_range()
				,m_tick_text(ToText)
				,m_allow_scroll(true)
				,m_allow_zoom(true)
				,m_lock_range(false)
			{}
			real min() const    { return m_range.m_min; }
			real min(real x)    { return m_range.m_min = x; }
			real max() const    { return m_range.m_max; }
			real max(real x)    { return m_range.m_max = x; }
			real span() const   { return m_range.span(); }
			real span(real x)   { return m_range.span(x); }
			real centre() const { return m_range.centre(); }
			real centre(real x) { return m_range.centre(x); }
			void shift(real delta)
			{
				if (!m_allow_scroll) return;
				m_range.m_min += delta;
				m_range.m_max += delta;
			}
		};

		using SeriesCont       = std::vector<Series*>;
		using AxisRange        = typename Axis::Range;
		using SeriesRdrOptions = typename Series::RdrOptions;

		static wchar_t const* WndClassName() { return L"pr::gui::GraphCtrl"; }

		template <typename Derived = void>
		struct Params :Control::Params<not_void_t<Derived, Params<Derived>>>
		{
			Params()
			{
				this->wndclass(RegisterWndClass<GraphCtrl>());
			}
		};

	protected:
		struct Snapshot
		{
			std::shared_ptr<Bitmap> m_bm;     // A copy of the plot area at a moment in time
			AxisRange               m_xrange; // The x axis range when the snapshot was taken
			AxisRange               m_yrange; // The y axis range when the snapshot was taken

			Snapshot() :m_bm() ,m_xrange() ,m_yrange() {}
			Size size() const { return Size(m_bm?m_bm->GetWidth():0, m_bm?m_bm->GetHeight():0); }
			Rect rect() const { return Rect(size()); }
		};
		struct Tooltip //:CWindowImpl<Tooltip>
		{
		//	HWND m_parent;
		//	std::wstring  m_text;
		//	Gdiplus::Font m_font;
		//	
		//	Tooltip() :m_text() ,m_font(Gdiplus::FontFamily::GenericSansSerif(), 10.0f) {}
		//	DECLARE_WND_CLASS_EX(_T("PRWTLGRAPHTT"),0,COLOR_INFOBK)
		//	BEGIN_MSG_MAP(Tooltip)
		//		MSG_WM_CREATE(OnCreate)
		//		MSG_WM_PAINT(OnPaint)
		//	END_MSG_MAP()
		//	int OnCreate(LPCREATESTRUCT create)
		//	{
		//		SetWindowPos(0, 0, 0, 0, 0, SWP_NOZORDER|SWP_NOMOVE);
		//		m_parent = create->hwndParent;
		//		return S_OK;
		//	}
		//	void OnPaint(CDCHandle)
		//	{
		//		using namespace Gdiplus;
		//		CPaintDC paint_dc(m_hWnd);
		//		Graphics gfx(paint_dc);
		//		Color bkgd; bkgd.SetFromCOLORREF(::GetSysColor(COLOR_INFOBK));
		//		SolidBrush bsh_text((ARGB)Color::Black);
		//		gfx.Clear(bkgd);
		//		gfx.DrawString(m_text.c_str(), int(m_text.size()), &m_font, PointF(), &bsh_text);
		//	}
			void SetTipText(int x, int y, wchar_t const* text)
			{
				(void)x,y,text;
		//		using namespace Gdiplus;
		//		m_text = text;
		//		Graphics gfx(m_hWnd);
		//		RectF sz; gfx.MeasureString(m_text.c_str(), int(m_text.size()), &m_font, PointF(), &sz);
		//		CPoint pt(x,y); ::ClientToScreen(m_parent, &pt);
		//		SetWindowPos(0, pt.x, pt.y, int(sz.Width), int(sz.Height), SWP_NOZORDER);
		//		Invalidate();
			}
			bool IsWindowVisible() const
			{
				return false;
			}
			void ShowWindow(int show)
			{
				(void)show;
			}
		};

		pr::GdiPlus      m_gdiplus;     // Initialises the GdiPlus library
		std::thread      m_rdr_thread;  // A thread used to render
		std::atomic_bool m_rdr_cancel;  // Cancel rendering flag
		std::mutex       m_mutex_snap;  // A mutex to protect access to the snapshot bitmap
		Snapshot         m_snap;        // A snapshot of the plot area, used as a temporary copy during drag/zoom operations
		Snapshot         m_tmp;         // A temporary bitmap used for background thread rendering
		Rect             m_plot_area;   // The area containing the graph data
		AxisRange        m_base_xrange; // The default x range of the data
		AxisRange        m_base_yrange; // The default y range of the data
		float            m_zoom;        // The amount of zoom
		pr::Range<float> m_zoom_limits; // The limits for zoom
		HCURSOR          m_cur_arrow;   // The normal cursor
		HCURSOR          m_cur_cross;   // The cross cursor
		HCURSOR          m_cur_grab;    // The grab cursor
		Tooltip          m_tt;          // Tracking tooltip for displaying coordinates
		Point            m_pt_grab;     // The grab start location
		Rect             m_selection;   // The selection area
		bool             m_dragging;    // True when the graph is being dragged around
		bool             m_selecting;   // True while performing an area selection
		bool             m_impl_dirty;  // True when the graph must be redrawn

	public:

		std::wstring     m_title;       // The graph title
		RdrOptions       m_opts;        // Global graph rendering options
		Axis             m_xaxis;       // The x axis
		Axis             m_yaxis;       // The y axis
		SeriesCont       m_series;      // The container of graph data
			
		// A mutex that is held by the control during rendering.
		// This should be used to synchronise source data changes with rendering.
		std::mutex MutexRendering;

		GraphCtrl() :GraphCtrl(Params<>()) {}
		GraphCtrl(Params<> const& p)
			:Control(p)
			,m_gdiplus()
			,m_rdr_thread()
			,m_rdr_cancel()
			,m_mutex_snap()
			,m_snap()
			,m_tmp()
			,m_plot_area()
			,m_base_xrange()
			,m_base_yrange()
			,m_zoom(1.0f)
			,m_zoom_limits({pr::maths::tinyf, pr::maths::float_max})
			,m_cur_arrow()
			,m_cur_cross()
			,m_cur_grab()
			,m_tt()
			,m_pt_grab()
			,m_selection()
			,m_dragging(false)
			,m_selecting(false)
			,m_impl_dirty(true)
			,m_title(L"Graph")
			,m_opts()
			,m_xaxis()
			,m_yaxis()
			,m_series()
			,MutexRendering()
			,MouseNavigation(true)
		{
			m_cur_arrow = ::LoadCursor(nullptr, IDC_ARROW);
			m_cur_cross = ::LoadCursor(nullptr, IDC_CROSS);
			m_cur_grab  = ::LoadCursor(nullptr, IDC_HAND);
			//m_tt.Create(m_hWnd, 0, 0, WS_BORDER|WS_POPUP, WS_EX_TOPMOST);
		}
		~GraphCtrl()
		{
			m_rdr_cancel = true;
			if (m_rdr_thread.joinable())
				m_rdr_thread.join();
		}

		#pragma region Data Access
	public:
		// Returns the 'Y' for a given 'X' value in a series in the graph (lerped)
		real GetValueAt(int series_index, real x) const
		{
			if (series_index < 0 || series_index >= int(m_series.size()))
				throw std::exception("series index out of range");

			// Get the series
			auto& series = *m_series[series_index];
			if (series.size() == 0)
				return 0;

			// Find the index range that contains 'x'
			size_t i0 = 0, i1 = 0;
			series.IndexRange(x, x, i0, i1);

			// Search for the nearest on the left and right of 'x'
			// (note: not assuming the series is sorted here)
			auto lhs = &series.first();
			auto rhs = &series.last();
			for (auto i = i0; i != i1; ++i)
			{
				auto tmp = &series[i];
				if (lhs->x < tmp->x && tmp->x < x) lhs = tmp;
				if (rhs->x > tmp->x && tmp->x > x) rhs = tmp;
			}
			if (lhs->x > x && rhs->x < x) return 0;
			if (lhs->x > x) return rhs->y;
			if (rhs->x < x) return lhs->y;
			auto t = (x - lhs->x) / (rhs->x - lhs->x);
			return (1 - t) * lhs->y + (t) * rhs->y;
		}

		// Returns the nearest graph data point (gv) to 'pt' given a selection tolerance.
		// 'pt' should be in graph space (use PointToGraph).
		// Returns false if no point is within the selection tolerance
		bool GetValueAt(int series_index, PointF const& pt, Elem& gv, int px_tol = 5)
		{
			if (series_index < 0 || series_index >= int(m_series.size()))
				throw std::exception("Series index out of range");

			auto& series = *m_series[series_index];
			auto tol = px_tol * m_plot_area.Width / m_xaxis.span();
			auto dist_sq = tol * tol;

			series.Values(pt.X - tol, pt.X + tol, [&](Elem const& e)
			{
				auto d = pr::LenSq(e.x - pt.X, e.y - pt.Y);
				if (d < dist_sq)
				{
					dist_sq = d;
					gv = e;
				}
			});
			return dist_sq < tol * tol;
		}

		#pragma endregion

		#pragma region Navigation
	public:

		// Get/Set the default ranges for the x/y axes
		AxisRange BaseRangeX() const               { return m_base_xrange; }
		void      BaseRangeX(AxisRange const& rng) { m_base_xrange = rng; }
		AxisRange BaseRangeY() const               { return m_base_yrange; }
		void      BaseRangeY(AxisRange const& rng) { m_base_yrange = rng; }

		// Find the appropriate range for all data in the graph
		// Call ResetToDefaultRange() to zoom the graph to this range
		void FindDefaultRange()
		{
			auto xrng = AxisRange(pr::maths::float_max, -pr::maths::float_max);
			auto yrng = AxisRange(pr::maths::float_max, -pr::maths::float_max);
			for (auto& s : m_series)
			{
				if (m_rdr_cancel)
					break;

				auto& series = *s;
				if (!series.m_opts.Visible)
					continue;

				series.Values([&](Elem const& e)
				{
					if (e.x < xrng.m_min) xrng.m_min = e.x;
					if (e.x > xrng.m_max) xrng.m_max = e.x;
					if (e.y < yrng.m_min) yrng.m_min = e.y;
					if (e.y > yrng.m_max) yrng.m_max = e.y;
				});
			}
			if (xrng.span() > 0.0f) xrng.span(xrng.span() * 1.05f); else xrng.span(1.0f);
			if (yrng.span() > 0.0f) yrng.span(yrng.span() * 1.05f); else yrng.span(1.0f);
			m_base_xrange = xrng;
			m_base_yrange = yrng;
		}

		// Reset the axis ranges to the default
		// Call FindDefaultRange() first to set the default range
		void ResetToDefaultRange()
		{
			if (!m_xaxis.m_lock_range) { m_xaxis.m_range = m_base_xrange; }
			if (!m_yaxis.m_lock_range) { m_yaxis.m_range = m_base_yrange; }
			Dirty(true);
		}

		// Returns a point in graph space from a point in client space
		// Use to convert mouse (client-space) locations to graph coordinates
		Point PointToGraph(Point const& point) const
		{
			return Point(
				m_xaxis.min() + (point.x - m_plot_area.left  ) * m_xaxis.span() / m_plot_area.width(),
				m_yaxis.min() - (point.y - m_plot_area.bottom) * m_yaxis.span() / m_plot_area.height());
		}

		// Returns a point in client space from a point in graph space.
		// Inverse of PointToGraph
		Point GraphToPoint(Point const& gs_point) const
		{
			return Point(
				m_plot_area.left   + (gs_point.x - m_xaxis.min()) * m_plot_area.width()  / m_xaxis.span(),
				m_plot_area.bottom - (gs_point.y - m_yaxis.min()) * m_plot_area.height() / m_yaxis.span());
		}

		// Shifts the X and Y range of the graph so that graph space position
		// 'gs_point' is at client space position 'cs_point'
		void PositionGraph(Point const& cs_point, Point const& gs_point)
		{
			auto dst = PointToGraph(cs_point);
			m_xaxis.shift(gs_point.x - dst.x);
			m_yaxis.shift(gs_point.y - dst.y);
			Dirty(true);
		}

		// Get/Set the centre of the graph.
		Point Centre() const
		{
			return Point(
				m_xaxis.min() + m_xaxis.span()*0.5f,
				m_yaxis.min() + m_yaxis.span()*0.5f);
		}
		void Centre(Point ctr)
		{
			m_xaxis.min(ctr.x - m_xaxis.span()*0.5f);
			m_yaxis.min(ctr.y - m_yaxis.span()*0.5f);
			Dirty(true);
		}

		// Zoom in/out on the graph. Remember to call refresh.
		// Zoom is a floating point value where 1f = no zoom, 2f = 2x magnification
		float Zoom() const
		{
			return static_cast<float>(
				m_xaxis.m_allow_zoom ? m_xaxis.span() / m_base_xrange.span() :
				m_yaxis.m_allow_zoom ? m_yaxis.span() / m_base_yrange.span() :
				1.0f);
		}
		void Zoom(float zm)
		{
			auto aspect = (m_yaxis.span() * m_base_xrange.span()) / (m_base_yrange.span() * m_xaxis.span());
			aspect = pr::Clamp(pr::IsFinite(aspect) ? aspect : 1.0, 0.001, 1000.0);

			zm = pr::Clamp(zm, m_zoom_limits.m_beg, m_zoom_limits.m_end);
			if (m_xaxis.m_allow_zoom) m_xaxis.span(m_base_xrange.span() * zm         );
			if (m_yaxis.m_allow_zoom) m_yaxis.span(m_base_yrange.span() * zm * aspect);
			Dirty(true);
		}

		// Get/Set the Zoom limits
		float ZoomMin() const  { return m_zoom_limits.m_min; }
		float ZoomMin(float x) { return m_zoom_limits.m_min = x; assert(x > 0.0f); }
		float ZoomMax() const  { return m_zoom_limits.m_max; }
		float ZoomMax(float x) { return m_zoom_limits.m_max = x; assert(x > 0.0f); }

		// Enable/Disable mouse navigation
		bool MouseNavigation;

	protected:

		// Mouse Navigation
		void OnMouseButton(MouseEventArgs& args) override
		{
			Control::OnMouseButton(args);
			if (args.m_handled) return;

			if (args.m_down)
			{
				m_selection = Rect(args.m_point, Size());
				if (MouseNavigation && int(args.m_button & EMouseKey::Left) != 0)
				{
					m_dragging = true;
					m_pt_grab = PointToGraph(args.m_point);
					::SetCursor(m_cur_grab);
					::SetCapture(m_hwnd);
				}
				if (int(args.m_button & EMouseKey::Right) != 0)
				{
					m_selecting = true;
					::SetCapture(m_hwnd);
				}
			}
			else
			{
				if (m_dragging)
				{
					m_dragging = false;
				}
				if (m_selecting)
				{
					m_selecting = false;

					// If the selection has area, rescale the graph
					if (abs(m_selection.width()) != 0 && abs(m_selection.height()) != 0)
					{
						auto sel = m_selection.NormalizeRect();

						// Rescale the graph
						auto lower = PointToGraph(Point(float(sel.left), float(sel.bottom)));
						auto upper = PointToGraph(Point(float(sel.right), float(sel.top)));
						m_xaxis.min(lower.x);
						m_xaxis.max(upper.x);
						m_yaxis.min(lower.y);
						m_yaxis.max(upper.y);
						Dirty(true);

						// Clear the selection
						m_selection.right  = m_selection.left;
						m_selection.bottom = m_selection.top;
					}
					else
					{
						auto pt = args.m_point;
						::ClientToScreen(m_hwnd, &pt);
						ShowContextMenu(pt);
					}
				}
				::ReleaseCapture();
			}
			args.m_handled = true;
		}
		void OnMouseMove(MouseEventArgs& args) override
		{
			Control::OnMouseMove(args);
			if (args.m_handled) return;

			if (int(args.m_button & EMouseKey::Left) != 0 && m_dragging)
			{
				auto grab_loc = GraphToPoint(m_pt_grab);
				auto dx = args.m_point.x - grab_loc.x;
				auto dy = args.m_point.y - grab_loc.y;
				if (dx*dx + dy*dy >= 25.0f) // must drag at least 5 pixels
					PositionGraph(Point(args.m_point), m_pt_grab);
			}
			if (int(args.m_button & EMouseKey::Right) != 0 && m_selecting)
			{
				int const MinAreaSelectDistance = 3;
				m_selection.right  = args.m_point.x;
				m_selection.bottom = args.m_point.y;
				if (abs(m_selection.width())  < MinAreaSelectDistance) m_selection.right  = m_selection.left;
				if (abs(m_selection.height()) < MinAreaSelectDistance) m_selection.bottom = m_selection.top;
				Invalidate();
			}
			if (m_tt.IsWindowVisible())
			{
				auto pt = PointToGraph(Point(float(args.m_point.x), float(args.m_point.y)));
				m_tt.SetTipText(args.m_point.x, args.m_point.y - 40, pr::FmtS(L"%f %f", pt.x, pt.y));
			}
			args.m_handled = true;
		}
		void OnMouseWheel(MouseWheelArgs& args) override
		{
			Control::OnMouseWheel(args);
			if (args.m_handled) return;

			args.m_handled = true;
			auto point = args.m_point;
			::ScreenToClient(m_hwnd, &point);
			if (!m_plot_area.Contains(point))
				return;

			auto pt = PointToGraph(point);
			int delta = pr::Clamp<short>(args.m_delta, -999, 999);
			Zoom(Zoom() * (1.0f - delta * 0.001f));
			PositionGraph(Point(point), pt);
			Dirty(true);
		}

		#pragma endregion

		#pragma region Rendering

		// Mark the control as needing a repaint
		bool Dirty() const
		{
			return m_impl_dirty;
		}
		void Dirty(bool dirty)
		{
			if (!m_impl_dirty) Invalidate();
			m_impl_dirty |= dirty;
		}

		// Render the graph into a device context (synchronously).
		// To use this to create bitmaps use:
		//   CSize size(100,100);
		//   CDC dc;     dc.CreateCompatibleDC();
		//   CBitmap bm; bm.CreateCompatibleBitmap(dc, size.cx, size.cy);
		//   pr::DCSelect<HBITMAP>(dc, bm, false);
		//   CRect area(POINT(), size);
		//   RenderGraph(dc, area);
		void RenderGraph(HDC hdc, Rect const& graph_area, Rect& plot_area) const
		{
			Graphics gfx(hdc);
			plot_area = PlotArea(gfx, graph_area);
			RenderGraphFrame(gfx, graph_area, plot_area);
			RenderData(gfx, plot_area);
		}
		void RenderGraph(HDC hdc, Rect const& graph_area) const
		{
			Rect plot_area;
			RenderGraph(hdc, graph_area, plot_area);
		}

		// Get the transform from client space to graph space in 'c2g' and the x/y scaling factor in 'scale'.
		// 'c2g' is an out parameter because Matrix is non-copyable.
		// Note: The returned transform has no scale because lines and points would also be scaled turning them into elipses or caligraphy etc
		void ClientToGraphSpace(Rect const& plot_area, Matrix& c2g, Point& scale) const
		{
			auto plot = plot_area.Shifted(1,1).Adjust(0, 0, -1, -1);
			scale = Point(plot.width()/m_xaxis.span(), plot.height()/m_yaxis.span());
			if (!pr::IsFinite(scale.x)) scale.x = scale.x >= 0 ? pr::maths::float_max : -pr::maths::float_max;
			if (!pr::IsFinite(scale.y)) scale.y = scale.y >= 0 ? pr::maths::float_max : -pr::maths::float_max;
			c2g.SetElements(1.0f, 0.0f, 0.0f, -1.0f
				,float(plot.left   - m_xaxis.min() * scale.x)
				,float(plot.bottom + m_yaxis.min() * scale.y));
		}
		void ClientToGraphSpace(Matrix& c2g, Point& scale) const
		{
			return ClientToGraphSpace(m_plot_area, c2g, scale);
		}
		void ClientToGraphSpace(Matrix& c2g) const
		{
			Point scale;
			return ClientToGraphSpace(c2g, scale);
		}

	protected:

		// Render the graph in to 'dc'
		void DoPaint(HDC dc, Rect const& area)
		{
			MemDC memdc(dc, area, nullptr);
			Graphics gfx(memdc);
			assert(gfx.GetLastStatus() == Gdiplus::Ok && "GDI+ not initialised");

			try
			{
				m_plot_area = PlotArea(gfx, area);

				// If the graph is dirty, begin an asynchronous render of the plot into 'm_tmp'
				if (Dirty())
				{
					m_rdr_cancel = true;
					if (m_rdr_thread.joinable())
						m_rdr_thread.join();
					m_rdr_cancel = false;

					// Make sure the temporary bitmap and the snapshot bitmap are the correct size
					auto plot_size = m_plot_area.size();
					if (m_tmp.size() != plot_size)
						m_tmp.m_bm = std::make_shared<Bitmap>(plot_size.cx, plot_size.cy);

					m_tmp.m_xrange = m_xaxis.m_range;
					m_tmp.m_yrange = m_yaxis.m_range;

					// Plot rendering (done in a background thread).
					// This thread renders the plot into the bitmap in 'm_tmp' using readonly access to the series data.
					m_rdr_thread = std::thread([&]
					{
						// Hold the rendering CS. Clients should hold this if they want to
						// modify the data while the graph is potentially rendering
						{
							std::lock_guard<std::mutex> lock(MutexRendering);

							// Render the plot into 'm_tmp'
							auto& g = *Graphics::FromImage(m_tmp.m_bm.get());
							RenderData(g, m_tmp.rect());
						}

						// If the render was cancelled, ignore the result
						if (m_rdr_cancel)
							return;

						// Otherwise get the main thread to do something with the plot bitmap
						{
							std::lock_guard<std::mutex> lock(m_mutex_snap);
							std::swap(m_snap, m_tmp);
						}

						// Cause a refresh
						Invalidate();
					});
					m_impl_dirty = false;
				}

				// In the mean time, compose the graph in 'm_bm' by rendering the frame
				// synchronously and blt'ing the last snapshot into the plot area
				RenderGraphFrame(gfx, area, m_plot_area);

				gfx.SetClip(pr::To<Gdiplus::Rect>(m_plot_area.Shifted(1,1).Adjust(0, 0, -1,-1)));
				gfx.SetSmoothingMode(Gdiplus::SmoothingModeHighQuality);
				{
					std::lock_guard<std::mutex> lock(m_mutex_snap);
					if (m_snap.m_bm)
					{
						auto tl = GraphToPoint(Point(m_snap.m_xrange.m_min, m_snap.m_yrange.m_max));
						auto br = GraphToPoint(Point(m_snap.m_xrange.m_max, m_snap.m_yrange.m_min));
						Rect dst_rect(int(tl.x), int(tl.y), int(br.x), int(br.y));
						Rect src_rect(m_snap.rect());
						gfx.DrawImage(m_snap.m_bm.get(), pr::To<Gdiplus::Rect>(dst_rect), int(src_rect.left), int(src_rect.top), int(src_rect.width()), int(src_rect.height()), Gdiplus::UnitPixel);
					}
				}

				// Allow clients to draw on the graph
				AddOverlayOnPaint(*this, gfx);

				// Draw the selection rubber band
				if (m_selection.width() != 0 && m_selection.height() != 0)
				{
					auto sel = m_selection.NormalizeRect();
					Pen pen(m_opts.SelectionColour);
					pen.SetDashStyle(Gdiplus::DashStyleDot);
					gfx.DrawRectangle(&pen, pr::To<Gdiplus::Rect>(sel));
				}

				gfx.ResetClip();
			}
			catch (std::exception const&)
			{
				SolidBrush bsh(static_cast<ARGB>(Color::Red));
				wchar_t msg[] = L"Rendering error in GDI+";
				gfx.DrawString(msg, _countof(msg), &m_opts.TitleFont, PointF(), &bsh);
			}
		}

		// Returns an area for the plot part of the graph given a bitmap with size 'size'. (i.e. excluding titles, axis labels, etc)
		Rect PlotArea(Graphics const& gfx, Rect const& area) const
		{
			RectF rect(0.0f, 0.0f, float(area.width()), float(area.height()));
			RectF r;

			// Add margins
			rect.X      += m_opts.LeftMargin;
			rect.Y      += m_opts.TopMargin;
			rect.Width  -= m_opts.LeftMargin + m_opts.RightMargin;
			rect.Height -= m_opts.TopMargin  + m_opts.BottomMargin;

			// Add space for tick marks
			rect.X      += m_yaxis.m_opts.TickLength;
			rect.Width  -= m_yaxis.m_opts.TickLength;
			rect.Height -= m_xaxis.m_opts.TickLength;

			// Add space for the title and axis labels
			if (!m_title.empty())
			{
				gfx.MeasureString(m_title.c_str(), int(m_title.size()), &m_opts.TitleFont, PointF(), &r);
				rect.Y      += r.Height;
				rect.Height -= r.Height;
			}
			if (!m_xaxis.m_label.empty())
			{
				gfx.MeasureString(m_xaxis.m_label.c_str() ,int(m_xaxis.m_label.size()) ,&m_xaxis.m_opts.LabelFont ,PointF(), &r);
				rect.Height -= r.Height;
			}
			if (!m_yaxis.m_label.empty())
			{
				gfx.MeasureString(m_yaxis.m_label.c_str() ,int(m_yaxis.m_label.size()) ,&m_yaxis.m_opts.LabelFont ,PointF(), &r);
				rect.X     += r.Height; // will be rotated by 90°
				rect.Width -= r.Height;
			}

			// Add space for tick labels
			wchar_t const lbl[] = L"9.999";
			int const lbl_len = sizeof(lbl)/sizeof(lbl[0]);

			gfx.MeasureString(lbl, lbl_len, &m_xaxis.m_opts.TickFont ,PointF(), &r);
			rect.Height -= r.Height;

			gfx.MeasureString(lbl, lbl_len, &m_yaxis.m_opts.TickFont ,PointF(), &r);
			rect.X     += r.Width;
			rect.Width -= r.Width;

			return Rect(int(rect.X), int(rect.Y), int(rect.X + rect.Width), int(rect.Y + rect.Height));
		}

		// Return the min, max, and step size for the x/y axes
		void PlotGrid(Rect const& plot_area, PointF& min, PointF& max, PointF& step)
		{
			// Choose step sizes
			auto max_ticks_x = plot_area.width()  / m_opts.PixelsPerTick.X;
			auto max_ticks_y = plot_area.height() / m_opts.PixelsPerTick.Y;
			auto xspan = m_xaxis.span();
			auto yspan = m_yaxis.span();
			auto step_x = float(pow(10.0, (int)log10(xspan))); step.X = step_x;
			auto step_y = float(pow(10.0, (int)log10(yspan))); step.Y = step_y;
			for (auto s : {0.05f, 0.1f, 0.2f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 5.0f, 10.0f, 20.0f, 50.0f})
			{
				if (s * xspan / step_x <= max_ticks_x) step.X = step_x / s;
				if (s * yspan / step_y <= max_ticks_y) step.Y = step_y / s;
			}

			min.X  = float((m_xaxis.min() - fmod(m_xaxis.min(), step.X)) - m_xaxis.min()); if (min.X < 0.0) min.X += step.X;
			min.Y  = float((m_yaxis.min() - fmod(m_yaxis.min(), step.Y)) - m_yaxis.min()); if (min.Y < 0.0) min.Y += step.Y;
			max.X  = float(m_xaxis.span() * 1.0001);
			max.Y  = float(m_yaxis.span() * 1.0001);

			// protect against increments smaller than can be represented by a float
			if (min.X + step.X == min.X) step.X = (max.X - min.X) * 0.01f;
			if (min.Y + step.Y == min.Y) step.Y = (max.Y - min.Y) * 0.01f;

			// protect against too many ticks along the axis
			if (max.X - min.X > step.X*100) step.X = (max.X - min.X) * 0.01f;
			if (max.Y - min.Y > step.Y*100) step.Y = (max.Y - min.Y) * 0.01f;
		}

		// Render the basic graph, axes, title, labels, etc
		void RenderGraphFrame(Graphics& gfx, Rect const& area, Rect const& plot_area)
		{
			// This is not enforced in the axis.Min/Max accessors because it's useful
			// to be able to change the min/max independently of each other, set them
			// to float max etc. It's only invalid to render a graph with a negative range
			assert(m_xaxis.span() > 0 && "Negative x range");
			assert(m_yaxis.span() > 0 && "Negative y range");

			// Clear to the background colour
			gfx.Clear(m_opts.BkColour);

			// Draw the graph title and labels
			Gdiplus::RectF r;
			if (!m_title.empty())
			{
				SolidBrush bsh(m_opts.TitleColour);
				gfx.MeasureString(m_title.c_str(), int(m_title.size()), &m_opts.TitleFont, PointF(), &r);
				auto x = float(area.width() - r.Width) * 0.5f;
				auto y = float(area.top + m_opts.TopMargin);
				gfx.TranslateTransform(x, y);
				gfx.MultiplyTransform(&m_opts.TitleTransform);
				gfx.DrawString(m_title.c_str(), int(m_title.size()), &m_opts.TitleFont, PointF(), &bsh);
				gfx.ResetTransform();
			}
			if (!m_xaxis.m_label.empty())
			{
				SolidBrush bsh(m_xaxis.m_opts.LabelColour);
				gfx.MeasureString(m_xaxis.m_label.c_str(), int(m_xaxis.m_label.size()), &m_xaxis.m_opts.LabelFont, PointF(), &r);
				auto x = float(area.width() - r.Width) * 0.5f;
				auto y = float(area.bottom - m_opts.BottomMargin - r.Height);
				gfx.TranslateTransform(x, y);
				gfx.MultiplyTransform(&m_xaxis.m_opts.LabelTransform);
				gfx.DrawString(m_xaxis.m_label.c_str(), int(m_xaxis.m_label.size()), &m_xaxis.m_opts.LabelFont, PointF(), &bsh);
				gfx.ResetTransform();
			}
			if (!m_yaxis.m_label.empty())
			{
				SolidBrush bsh(m_yaxis.m_opts.LabelColour);
				gfx.MeasureString(m_yaxis.m_label.c_str(), int(m_yaxis.m_label.size()), &m_yaxis.m_opts.LabelFont, PointF(), &r);
				auto x = float(area.left + m_opts.LeftMargin);
				auto y = float(area.height() + r.Width) * 0.5f;
				gfx.TranslateTransform(x, y);
				gfx.RotateTransform(-90.0f);
				gfx.MultiplyTransform(&m_yaxis.m_opts.LabelTransform);
				gfx.DrawString(m_yaxis.m_label.c_str(), int(m_yaxis.m_label.size()), &m_yaxis.m_opts.LabelFont, PointF(), &bsh);
				gfx.ResetTransform();
			}

			{// Draw the graph frame and background
				SolidBrush bsh_bkgd(m_opts.BkColour);
				SolidBrush bsh_axis(m_opts.AxisColour);
				Pen        pen_axis(m_opts.AxisColour, 0.0f);

				// Background
				RenderPlotBkgd(gfx, plot_area);

				PointF min,max,step;
				PlotGrid(plot_area, min, max, step);

				// Tick marks and labels
				SolidBrush bsh_xtick(m_xaxis.m_opts.TickColour);
				SolidBrush bsh_ytick(m_yaxis.m_opts.TickColour);
				auto lblx = float(plot_area.left - m_yaxis.m_opts.TickLength - 1);
				auto lbly = float(plot_area.top + plot_area.height() + m_xaxis.m_opts.TickLength + 1);
				for (auto x = min.X; x < max.X; x += step.X)
				{
					int X = int(plot_area.left + x * plot_area.width() / m_xaxis.span());
					auto s = m_xaxis.m_tick_text(x + m_xaxis.min());
					gfx.MeasureString(s.c_str(), int(s.size()), &m_xaxis.m_opts.TickFont, PointF(), &r);
					gfx.DrawString   (s.c_str(), int(s.size()), &m_xaxis.m_opts.TickFont, PointF(X - r.Width*0.5f, lbly), &bsh_xtick);
					gfx.DrawLine(&pen_axis, X, plot_area.top + plot_area.height(), X, plot_area.top + plot_area.height() + m_xaxis.m_opts.TickLength);
				}
				for (auto y = min.Y; y < max.Y; y += step.Y)
				{
					int Y = int(plot_area.top + plot_area.height() - y * plot_area.height() / m_yaxis.span());
					auto s = m_yaxis.m_tick_text(y + m_yaxis.min());
					gfx.MeasureString(s.c_str(), int(s.size()), &m_yaxis.m_opts.TickFont, PointF(), &r);
					gfx.DrawString   (s.c_str(), int(s.size()), &m_yaxis.m_opts.TickFont, PointF(lblx - r.Width, Y - r.Height*0.5f), &bsh_ytick);
					gfx.DrawLine(&pen_axis, plot_area.left - m_yaxis.m_opts.TickLength, Y, plot_area.left, Y);
				}

				// Graph border
				gfx.DrawRectangle(&pen_axis, pr::To<Gdiplus::Rect>(plot_area));
			}

			// Control border
			switch (m_opts.Border)
			{
			default: throw std::exception("");
			case RdrOptions::EBorder::None: break;
			case RdrOptions::EBorder::Single:
				{
					Pen pen_border(static_cast<ARGB>(Color::Black), 0.0f);
					gfx.DrawRectangle(&pen_border, pr::To<Gdiplus::Rect>(area.Adjust(0, 0, -1, -1)));
					break;
				}
			}
		}

		// Render the plot background including grid lines
		void RenderPlotBkgd(Graphics& gfx, Rect const& plot_area)
		{
			SolidBrush bsh_plot(m_opts.PlotBkColour);
			Pen        pen_grid(m_opts.GridColour, 0.0f);

			PointF min,max,step;
			PlotGrid(plot_area, min, max, step);

			// Background
			gfx.FillRectangle(&bsh_plot, pr::To<Gdiplus::Rect>(plot_area));

			// Grid lines
			for (auto x = min.X; x < max.X; x += step.X)
			{
				auto X = int(plot_area.left + x * plot_area.width() / m_xaxis.span());
				gfx.DrawLine(&pen_grid, X, plot_area.top, X, plot_area.bottom);
			}
			for (auto y = min.Y; y < max.Y; y += step.Y)
			{
				auto Y = int(plot_area.bottom - y * plot_area.height() / m_yaxis.span());
				gfx.DrawLine(&pen_grid, plot_area.left, Y, plot_area.right, Y);
			}
		}

		// Render the series data into the graph (within 'area')
		void RenderData(Graphics& gfx, Rect const& plot_area)
		{
			auto plot = plot_area.Shifted(1,1).Adjust(0, 0, -1, -1);
			gfx.SetClip(pr::To<Gdiplus::Rect>(plot));

			RenderPlotBkgd(gfx, plot_area);

			// Set the transform so that we can draw directly in graph space.
			// Note: We can't use a scale transform here because the lines and
			// points will also be scaled, into elipses or caligraphy etc
			Matrix c2g; Point scale;
			ClientToGraphSpace(plot_area, c2g, scale);
			gfx.SetTransform(&c2g);

			// Plot each series
			for (auto s : m_series)
			{
				if (m_rdr_cancel)
					break;

				auto& series = *s;
				auto& opts = series.m_opts;
				if (!opts.Visible)
					continue;

				SolidBrush bsh_pt  (opts.PointColour);
				SolidBrush bsh_bar (opts.BarColour);
				SolidBrush bsh_err (opts.ErrorBarColour);
				Pen        pen_line(opts.LineColour, 0.0f);
				Pen        pen_bar (opts.BarColour, 0.0f);

				// Find the range of indices to consider
				size_t i0, i1;
				series.IndexRange(m_xaxis.min(), m_xaxis.max(), i0, i1);

				// Loop over data points
				for (auto i = i0; i != i1;)
				{
					if (m_rdr_cancel)
						break;

					// Get the data point
					ScreenPoint pt(series, plot_area, scale, i, i1);

					// Render the data point
					switch (opts.PlotType)
					{
					// Draw the data point
					case Series::RdrOptions::EPlotType::Point:
						PlotPoint(gfx, pt, opts, bsh_pt, bsh_err);
						break;

					// Draw the data point and connect with a line
					case Series::RdrOptions::EPlotType::Line:
						PlotLine(gfx, pt, opts, bsh_pt, pen_line, bsh_err);
						break;

					// Draw the data as columns in a bar graph
					case Series::RdrOptions::EPlotType::Bar:
						PlotBar(gfx, pt, opts, bsh_bar, pen_bar, bsh_err);
						break;
					}
				}

				// Add a moving average line
				if (opts.DrawMovingAvr)
				{
					i0 = std::max<size_t>(0            , i0 - opts.MAWindowSize);
					i1 = std::min<size_t>(series.size(), i1 + opts.MAWindowSize);
					PlotMovingAverage(gfx, opts, scale, series, i0, i1);
				}
			}

			gfx.ResetTransform();

			// Allow clients to draw on the graph
			AddOverlayOnRender(*this, gfx);

			gfx.ResetClip();
		}

		// A helper for rendering that finds the bounds of all points at the same screen space X position
		struct ScreenPoint
		{
			Series const& m_series;       // The series that this point came from
			Rect const&   m_plot_area;    // The screen space area that the point must be within
			Point const&  m_scale;        // Graph space to screen space scaling factors
			size_t        m_imin, m_imax; // The index range of the data points included
			real          m_xmin, m_xmax; // The range of data point X values
			real          m_ymin, m_ymax; // The range of data point Y values
			real          m_ylo , m_yhi ; // The bounds on the error bars of the Y values
			mutable int   m_lhs , m_rhs ; // Error bars/Bar graph width in screen space
				
			// Scan 'i' forward to the next data point that has a screen space X value not equal to that of the i'th data point
			ScreenPoint(Series const& series, Rect const& plot_area, Point const& scale, size_t& i, size_t iend)
				:m_series(series)
				,m_plot_area(plot_area)
				,m_scale(scale)
				,m_imin(i)
				,m_imax(i)
				,m_xmin()
				,m_xmax()
				,m_ymin()
				,m_ymax()
				,m_ylo()
				,m_yhi()
				,m_lhs()
				,m_rhs()
			{
				// Get the data point
				auto& gv0 = series[i];
				auto sx = (int)(gv0.x * scale.x);

				// Init members
				m_imin = m_imax = i;
				m_xmin = m_xmax = gv0.x;
				m_ymin = m_ymax = gv0.y;
				m_ylo  = gv0.y + gv0.ylo;
				m_yhi  = gv0.y + gv0.yhi;

				// While the data point still represents the same X coordinate on-screen
				// scan forward until x != sx, finding the bounds on points that fall at this X
				for (++i; i != iend; ++i)
				{
					auto& gv1 = series[i];
					auto x = (int)(gv1.x * scale.x);
					if (x != sx) break;

					m_imax = i;
					m_xmax = gv1.x;
					m_ymin = std::min<real>(m_ymin , gv1.y);
					m_ymax = std::max<real>(m_ymax , gv1.y);
					m_ylo  = std::min<real>(m_ylo  , gv1.y + gv1.ylo);
					m_yhi  = std::max<real>(m_yhi  , gv1.y + gv1.yhi);
				}
			}
			ScreenPoint(ScreenPoint const&) = delete;
			ScreenPoint& operator =(ScreenPoint const&) = delete;

			// True if this is a single point, false if it represents multiple points
			bool IsSingle() const { return m_imin == m_imax; }

			// Calculate the on-screen bar width for error bars or bar graphs
			void CalcBarWidth(float width_scale = 1.0f) const
			{
				// Calc the left and right side of the bar
				if (m_imin != 0)
				{
					auto prev_x = m_series[m_imin - 1].x;
					m_lhs = int(std::max(0.0, 0.5*(m_xmin - prev_x) * width_scale * m_scale.x));
				}
				if (m_imax+1 != int(m_series.size()))
				{
					auto next_x = m_series[m_imax + 1].x;
					m_rhs = int(std::max(1.0, 0.5*(next_x - m_xmax) * width_scale * m_scale.x));
				}
				if (m_lhs == 0) m_lhs = m_rhs; // i_min == 0 case
				if (m_rhs == 0) m_rhs = m_lhs; // i_max == Count-1 case
			}
		};

		// Draw error bars. lhs/rhs are the screen space size of the bar
		void PlotErrorBars(Graphics& gfx, ScreenPoint const& pt, SolidBrush const& bsh_err)
		{
			auto x   = (int)(pt.m_xmin * pt.m_scale.x);
			auto ylo = (int)(pt.m_ylo * pt.m_scale.y);
			auto yhi = (int)(pt.m_yhi * pt.m_scale.y);
			if (yhi - ylo > 0)
				gfx.FillRectangle(&bsh_err, Gdiplus::Rect(x - pt.m_lhs, ylo, pt.m_lhs + pt.m_rhs, yhi - ylo));
		}

		// Plot a point on the graph
		void PlotPoint(Graphics& gfx, ScreenPoint const& pt, SeriesRdrOptions const& opts, SolidBrush const& bsh_pt, SolidBrush const& bsh_err)
		{
			// Draw error bars is on
			if (opts.DrawErrorBars)
			{
				pt.CalcBarWidth();
				PlotErrorBars(gfx, pt, bsh_err);
			}

			// Plot the data point
			if (opts.DrawData)
			{
				auto x = (int)(pt.m_xmin * pt.m_scale.x);
				auto y = (int)(pt.m_ymin * pt.m_scale.y);
				auto h = (int)((pt.m_ymax - pt.m_ymin) * pt.m_scale.y);
				gfx.FillEllipse(&bsh_pt, RectF(x - opts.PointSize*0.5f, y - opts.PointSize*0.5f, opts.PointSize, h + opts.PointSize));
			}
		}

		// Plot a line segment on the graph
		void PlotLine(Graphics& gfx, ScreenPoint const& pt, SeriesRdrOptions const& opts, SolidBrush const& bsh_pt, Pen const& pen_line, SolidBrush const& bsh_err)
		{
			// Draw error bars is on
			if (opts.DrawErrorBars)
			{
				pt.CalcBarWidth();
				PlotErrorBars(gfx, pt, bsh_err);
			}

			// Plot the point and line
			if (opts.DrawData)
			{
				// Draw the line from the previous point
				if (pt.m_imin != 0) // if this is not the first point
				{
					auto px = (int)(pt.m_series[pt.m_imin - 1].x * pt.m_scale.x);
					auto py = (int)(pt.m_series[pt.m_imin - 1].y * pt.m_scale.y);
					auto x  = (int)(pt.m_series[pt.m_imin    ].x * pt.m_scale.x);
					auto y  = (int)(pt.m_series[pt.m_imin    ].y * pt.m_scale.y);
					gfx.DrawLine(&pen_line, px, py, x, y);
				}

				// Draw a vertical line if the screen point represents multiple points
				if (!pt.IsSingle())
				{
					auto x   = (int)(pt.m_xmin * pt.m_scale.x);
					auto ylo = (int)(pt.m_ymin * pt.m_scale.y);
					auto yhi = (int)(pt.m_ymax * pt.m_scale.y);
					gfx.DrawLine(&pen_line, x, ylo, x, yhi);
				}

				// Plot the point (if it's size is non-zero)
				if (opts.PointSize > 0)
				{
					auto x = (int) (pt.m_xmin * pt.m_scale.x);
					auto y = (int) (pt.m_ymin * pt.m_scale.y);
					auto h = (int)((pt.m_ymax - pt.m_ymin) * pt.m_scale.y);
					gfx.FillEllipse(&bsh_pt, RectF(x - opts.PointSize*0.5f, y - opts.PointSize*0.5f, opts.PointSize, h + opts.PointSize));
				}
			}
		}

		// Plot a single bar on the graph
		void PlotBar(Graphics& gfx, ScreenPoint const& pt, SeriesRdrOptions opts, SolidBrush const& bsh_bar, Pen const& pen_bar, SolidBrush const& bsh_err)
		{
			// Calc the left and right side of the bar
			pt.CalcBarWidth(opts.BarWidth);

			// Draw error bars is on
			if (opts.DrawErrorBars)
				PlotErrorBars(gfx, pt, bsh_err);

			// Plot the bar
			if (opts.DrawData)
			{
				auto x   = static_cast<int>( pt.m_xmin                        * pt.m_scale.x);
				auto ylo = static_cast<int>((pt.m_ymin > 0 ? 0.0 : pt.m_ymin) * pt.m_scale.y);
				auto yhi = static_cast<int>((pt.m_ymax < 0 ? 0.0 : pt.m_ymax) * pt.m_scale.y);
				
				if (yhi - ylo > 0)
					gfx.FillRectangle(&bsh_bar, Gdiplus::Rect(x - pt.m_lhs, ylo, std::max(1, pt.m_lhs + pt.m_rhs), yhi - ylo));
				else 
					gfx.DrawLine(&pen_bar, x - pt.m_lhs, 0, pt.m_lhs + pt.m_rhs, 0);
			}
		}

		// Plot a moving average curve over the data
		void PlotMovingAverage(Graphics& gfx, SeriesRdrOptions const& opts, Point const& scale, Series const& series, size_t i0, size_t i1)
		{
			pr::maths::ExpMovingAvr<> max(opts.MAWindowSize);
			pr::maths::ExpMovingAvr<> may(opts.MAWindowSize);
			Pen ma_pen(opts.MALineColour, opts.MALineWidth);

			bool first = true;
			int px = 0, py = 0;
			for (auto i = i0; i != i1; ++i)
			{
				auto& gv = series[i];
				max.Add(gv.x);
				may.Add(gv.y);

				auto x = static_cast<int>(max.Mean() * scale.x);
				auto y = static_cast<int>(may.Mean() * scale.y);
				if (first)
				{
					first = false;
					px = x;
					py = y;
				}
				else if (x != px)
				{
					gfx.DrawLine(&ma_pen, px, py, x, y);
					px = x;
					py = y;
				}
			}
		}

		#pragma endregion

		#pragma region Context Menu
	public:
		// Show a right click context menu
		void ShowContextMenu(Point point)
		{
			(void)point;
			// Todo, waiting for context menu to be convert to wingui
/*
			enum class ECmd
			{
				ShowValues = 1,
				ResetZoom,
				Visible,
				VisibleData,
				VisibleErrorBars,
				PlotType,
				PointSize,
				PointColour,
				LineWidth,
				LineColour,
				BarWidth,
				BarColour,
			};
			int const idx_all = 0xFFFF;

			// Construct the menu
			ContextMenu menu;
			menu.AddItem(std::make_shared<ContextMenu::Label>(L"&Show Values", MAKEWPARAM(ECmd::ShowValues, idx_all), m_tt.IsWindowVisible()));
			menu.AddItem(std::make_shared<ContextMenu::Label>(L"&Reset Zoom", MAKEWPARAM(ECmd::ResetZoom, idx_all)));
			if (!m_series.empty())
			{
				std::vector<std::wstring> plot_types;
				plot_types.push_back(L"Point");
				plot_types.push_back(L"Line");
				plot_types.push_back(L"Bar");

				int vis = 0, invis = 0;
				for (auto& series : m_series)
					(series->m_opts.Visible ? vis : invis) = 1;

				// All series options
				auto& series_all = menu.AddItem<ContextMenu>(std::make_shared<ContextMenu>(L"Series: All"));
				series_all.AddItem(std::make_shared<ContextMenu::Label>(L"&Visible", MAKEWPARAM(ECmd::Visible, idx_all), vis + invis));

				// Specific series options
				int idx_series = -1;
				for (auto s : m_series)
				{
					auto& series = *s;
					++idx_series;

					// Create a sub menu for this series
					StylePtr style(new ContextMenuStyle());
					style->m_col_text = series.m_opts.color();
					auto& series_m = menu.AddItem<ContextMenu>(std::make_shared<ContextMenu>(series.m_name.c_str(), 0, series.m_opts.Visible, style));

					// Visibility
					series_m.AddItem(std::make_shared<ContextMenu::Label>(L"&Visible"     ,MAKEWPARAM(ECmd::Visible          ,idx_series), series.m_opts.Visible));
					series_m.AddItem(std::make_shared<ContextMenu::Label>(L"Series &Data" ,MAKEWPARAM(ECmd::VisibleData      ,idx_series), series.m_opts.DrawData));
					series_m.AddItem(std::make_shared<ContextMenu::Label>(L"&Error Bars"  ,MAKEWPARAM(ECmd::VisibleErrorBars ,idx_series), series.m_opts.DrawErrorBars));

					// Plot Type
					series_m.AddItem(std::make_shared<ContextMenu::Combo>(L"&Plot Type"   ,&plot_types, MAKEWPARAM(ECmd::PlotType ,idx_series)));

					{// Appearance menu
						auto& appearance = series_m.AddItem<ContextMenu>(std::make_shared<ContextMenu>(L"&Appearance"));
						if (series.m_opts.PlotType == Series::RdrOptions::EPlotType::Point ||
							series.m_opts.PlotType == Series::RdrOptions::EPlotType::Line)
						{
							appearance.AddItem(std::make_shared<ContextMenu::Edit>(L"Point Size:", L"9", MAKEWPARAM(ECmd::PointSize, idx_series)));
							appearance.AddItem(std::make_shared<ContextMenu::Edit>(L"Point Colour:", L"9", MAKEWPARAM(ECmd::PointColour, idx_series)));
						}
						if (series.m_opts.PlotType == Series::RdrOptions::EPlotType::Line)
						{
							appearance.AddItem(std::make_shared<ContextMenu::Edit>(L"Line Width:", L"9", MAKEWPARAM(ECmd::LineWidth, idx_series)));
							appearance.AddItem(std::make_shared<ContextMenu::Edit>(L"Line Colour:", L"9", MAKEWPARAM(ECmd::LineColour, idx_series)));
						}
						if (series.m_opts.PlotType == Series::RdrOptions::EPlotType::Bar)
						{
							appearance.AddItem(std::make_shared<ContextMenu::Edit>(L"Bar Width:", L"9", MAKEWPARAM(ECmd::BarWidth, idx_series)));
							appearance.AddItem(std::make_shared<ContextMenu::Edit>(L"Bar Colour:", L"9", MAKEWPARAM(ECmd::BarColour, idx_series)));
						}
					}
				}
			}
				
			int res = menu.Show(m_hwnd, int(point.x), int(point.y));
			ECmd cmd = (ECmd)LOWORD(res);
			int  idx = (int)HIWORD(res);
			switch (cmd)
			{
			default: break;
			case ECmd::ShowValues:
				m_tt.ShowWindow(m_tt.IsWindowVisible() ? SW_HIDE : SW_SHOW);
				break;
			case ECmd::ResetZoom:
				ResetToDefaultRange();
				Dirty(true);
				break;
			case ECmd::Visible:
				if (idx == idx_all)
				{
					for (SeriesCont::iterator i = m_series.begin(), iend = m_series.end(); i != iend; ++i)
					{
					}
				}
				break;
			}
				
			//ContextMenu::Label show_values(L"&Show Values", ECmd_ShowValues, 0, m_tt.IsWindowVisible());
			//menu.AddItem(show_values);
			//
			//ContextMenu::Label reset_zoom(L"&Reset Zoom", ECmd_ResetZoom);
			//menu.AddItem(reset_zoom);
			//
			//Series::Menu series_all(L"Series: &All");
			//ContextMenu::Label all_visible(L"&Visible", ECmd_All_Visible);
			//series_all.AddItem(all_visible);
			//if (!m_series.empty()) menu.AddItem(series_all);
			//int invis = 0, vis = 0;
			//for (SeriesCont::const_iterator i = m_series.begin(), iend = m_series.end(); i != iend; ++i) (*i)->m_opts.m_visible ? vis : invis) = 1;
			//all_visible.m_check_state = vis + invis;
			//
			//
			//struct Menu :pr::gui::ContextMenu
			//{
			//	pr::gui::ContextMenuStyle m_style;
			//	pr::gui::ContextMenu::Label m_vis;
			//	
			//	Menu()
			//	:pr::gui::ContextMenu(L"", &m_style)
			//	,m_vis(L"&Visible", ECmd_Series)
			//	{}
			//};
			//
			//
			//for (SeriesCont::iterator i = m_series.begin(), iend = m_series.end(); i != iend; ++i)
			//{
			//	Series& series = **i;
			//	series.m_menu_style
			//	(*i)->m_opts.m_visible ? vis : invis) = 1;
			//
			//	
			//	series.AppendMenu(menu);

			//		series.m_menu.m_label.m_text      = series.m_name;
			//		series.m_menu.m_style.m_col_text  = series.m_opts.color();
			//		series.m_menu.m_vis.m_check_state = series.m_opts.m_visible;
			//		series.m_menu.AddItem(series.m_menu.m_vis);
			//		menu.AddItem(series.m_menu);
			//	}
			//}
			//
*/
		}

		#pragma endregion

	public:

		// Called whenever the control is repainted to allow user graphics to be overlayed over the cached bitmap
		// 'gfx' is the graphics object to use when adding graphics
		// Rendering is in screen space, use GraphToPoint(),PointToGraph(),ClientToGraphSpace()
		pr::EventHandler<GraphCtrl&, Graphics&> AddOverlayOnPaint;

		// Called when the cached graph bitmap is created to allow user graphics to be baked into the cached bitmap.
		// 'gfx' is the graphics object to use when adding graphics
		// Rendering is in screen space, use GraphToPoint(),PointToGraph(),ClientToGraphSpace()
		// Note: this method is called in the worker thread context.
		pr::EventHandler<GraphCtrl&, Graphics&> AddOverlayOnRender;

	protected:

		void OnWindowPosChange(WindowPosEventArgs const& args) override
		{
			Dirty(true);
			Control::OnWindowPosChange(args);
		}
		void OnPaint(PaintEventArgs& args) override
		{
			PaintStruct ps(m_hwnd);
			DoPaint(args.m_dc, ClientRect(false));
			Control::OnPaint(args);
		}
	};
}
