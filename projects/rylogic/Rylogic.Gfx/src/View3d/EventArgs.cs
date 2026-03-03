//#define PR_VIEW3D_CREATE_STACKTRACE
using System;
using System.ComponentModel;
using System.Drawing;
using Rylogic.Common;

namespace Rylogic.Gfx
{
	public sealed partial class View3d
	{
		public class ErrorEventArgs :EventArgs
		{
			public ErrorEventArgs(string message, string filepath, int line, long pos)
			{
				Message = message;
				Filepath = filepath;
				FileLine = line;
				FileOffset = pos;
			}

			/// <summary>A description of the error</summary>
			public string Message { get; }

			/// <summary>The file that contains the error (if known)</summary>
			public string Filepath { get; }

			/// <summary>The line index in 'Filepath'</summary>
			public int FileLine { get; }

			/// <summary>The location in 'Filepath' of the error</summary>
			public long FileOffset { get; }
		}

		public class ParsingProgressEventArgs :CancelEventArgs
		{
			public ParsingProgressEventArgs(Guid context_id, string filepath, long file_offset, long file_size, bool complete)
			{
				ContextId = context_id;
				Filepath = filepath;
				FileOffset = file_offset;
				FileSize = file_size;
				Complete = complete;
			}

			/// <summary>An anonymous pointer unique to each 'AddFile' call</summary>
			public Guid ContextId { get; }

			/// <summary>The file currently being parsed</summary>
			public string Filepath { get; }

			/// <summary>How far through the current file parsing is up to</summary>
			public long FileOffset { get; }

			/// <summary>The total size of the file being parsed</summary>
			public long FileSize { get; }

			/// <summary>Last progress update notification</summary>
			public bool Complete { get; }
		}

		public class SourcesChangedEventArgs :EventArgs
		{
			public SourcesChangedEventArgs(ESourcesChangedReason reason, IntPtr ids, int count, bool before)
			{
				Reason = reason;
				ContextIds = Marshal_.PtrToArray<Guid>(ids, count);
				Before = before;
			}

			/// <summary>The cause of the source changes</summary>
			public ESourcesChangedReason Reason { get; }

			/// <summary>The sources that changed</summary>
			public Guid[] ContextIds { get; }

			/// <summary>True if files are about to change</summary>
			public bool Before { get; }
			public bool After => !Before;
		}

		public class SceneChangedEventArgs :EventArgs
		{
			public SceneChangedEventArgs(SceneChanged args)
			{
				ChangeType = args.ChangeType;
				ContextIds = args.ContextIds;
				Object = args.Object;
			}

			/// <summary>How the scene was changed</summary>
			public ESceneChanged ChangeType { get; }

			/// <summary>The context ids of the objects that were changed in the scene</summary>
			public Guid[] ContextIds { get; }

			/// <summary>The LdrObject involved in the change (single object changes only)</summary>
			public Object? Object { get; }
		}

		public class AnimationEventArgs :EventArgs
		{
			public AnimationEventArgs(EAnimCommand command, double clock)
			{
				Command = command;
				Clock = clock;
			}

			/// <summary>The state change for the update</summary>
			public EAnimCommand Command { get; }

			/// <summary>The current animation clock value</summary>
			public double Clock { get; }
		}

		public class SettingChangeEventArgs :EventArgs
		{
			public SettingChangeEventArgs(ESettings setting)
			{
				Setting = setting;
			}

			/// <summary>The setting that changed</summary>
			public ESettings Setting { get; }
		}

		public class MouseNavigateEventArgs :EventArgs
		{
			public MouseNavigateEventArgs(Point point, EMouseBtns btns, ENavOp nav_op, bool nav_beg_or_end)
			{
				ZNavigation = false;
				Point = point;
				Btns = btns;
				NavOp = nav_op;
				NavBegOrEnd = nav_beg_or_end;
				Handled = false;
			}
			public MouseNavigateEventArgs(Point point, EMouseBtns btns, float delta, bool along_ray)
			{
				ZNavigation = true;
				Point = point;
				Btns = btns;
				Delta = delta;
				AlongRay = along_ray;
				Handled = false;
			}

			/// <summary>The mouse pointer in client rect space</summary>
			public Point Point { get; private set; }

			/// <summary>The current state of the mouse buttons</summary>
			public EMouseBtns Btns { get; private set; }

			/// <summary>The mouse wheel scroll delta</summary>
			public float Delta { get; private set; }

			/// <summary>The navigation operation to perform</summary>
			public ENavOp NavOp { get; private set; }

			/// <summary>True if this is the beginning or end of the navigation, false if during</summary>
			public bool NavBegOrEnd { get; private set; }

			/// <summary>True if this is a Z axis navigation</summary>
			public bool ZNavigation { get; private set; }

			/// <summary>True if Z axis navigation moves the camera along a ray through the mouse pointer</summary>
			public bool AlongRay { get; private set; }

			/// <summary>A flag used to prevent this mouse navigation being forwarded to the window</summary>
			public bool Handled { get; set; }
		}

		public class HitTestAsyncResultEventArgs :EventArgs
		{
			public HitTestAsyncResultEventArgs(HitTestResult[]? results)
			{
				Results = results ?? Array.Empty<HitTestResult>();
			}

			/// <summary>The hit test results (nearest first)</summary>
			public HitTestResult[] Results { get; }
		}
	}
}
