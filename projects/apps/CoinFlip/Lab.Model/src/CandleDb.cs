using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Runtime.ExceptionServices;
using System.Threading;
using Binance.API;
using Rylogic.Attrib;
using Rylogic.DB;

namespace Lab.Model
{
	/// <summary>
	/// A local SQLite cache of candle data, keyed by (symbol, timeframe, open-time).
	/// Lets the laboratory scrub through history offline once data has been downloaded.</summary>
	public sealed class CandleDb : IDisposable
	{
		// Notes:
		//  - Uses Rylogic.DB's thin native sqlite3 wrapper (sqlite3.dll must be loaded via Sqlite.LoadDll).
		//  - sqlite3 handles (the connection and every prepared statement) must be created *and*
		//    released on the same thread - this is enforced by the SafeHandle wrapper, independent
		//    of the FullMutex setting. To satisfy that while still allowing background downloads to
		//    write batches, the connection lives on a single dedicated worker thread and all access
		//    is marshalled onto it. Callers may be on any thread.

		private readonly BlockingCollection<Action> m_queue = new();
		private readonly Thread m_thread;
		private Sqlite.Connection m_db = null!;

		public CandleDb(string filepath)
		{
			Directory.CreateDirectory(Path.GetDirectoryName(filepath) ?? ".");
			var cs = $"Data Source={filepath};Version=3;Mode=ReadWriteCreate;ThreadSafe=On;Journal Mode=WAL;Synchronous=Normal";

			// Open the connection and pump DB work on a dedicated thread so all sqlite
			// handles are created and released on the same thread.
			using var ready = new ManualResetEventSlim(false);
			ExceptionDispatchInfo? open_err = null;
			m_thread = new Thread(() =>
			{
				try
				{
					m_db = new Sqlite.Connection(cs, "LabCandles");
					EnsureSchema();
				}
				catch (Exception ex)
				{
					open_err = ExceptionDispatchInfo.Capture(ex);
					ready.Set();
					return;
				}
				ready.Set();

				foreach (var work in m_queue.GetConsumingEnumerable())
					work();

				m_db.Dispose();
			})
			{
				IsBackground = true,
				Name = "LabCandleDb",
			};
			m_thread.Start();
			ready.Wait();
			open_err?.Throw();
		}
		public void Dispose()
		{
			m_queue.CompleteAdding();
			if (m_thread.IsAlive)
				m_thread.Join();
			m_queue.Dispose();
		}

		/// <summary>The earliest and latest cached open-times (Unix ms) for a symbol/timeframe, or (null,null) if empty</summary>
		public (long? Min, long? Max) Extent(string symbol, EMarketPeriod tf)
		{
			return Run(() => ExtentImpl(symbol, tf));
		}

		/// <summary>Read cached candles in the inclusive time range [begMs, endMs], ordered by time</summary>
		public List<Candle> Read(string symbol, EMarketPeriod tf, long begMs, long endMs)
		{
			return Run(() => ReadImpl(symbol, tf, begMs, endMs));
		}

		/// <summary>Insert or replace a batch of candles (transactional)</summary>
		public void Upsert(string symbol, EMarketPeriod tf, IReadOnlyList<Candle> candles)
		{
			Run(() => { UpsertImpl(symbol, tf, candles); return 0; });
		}

		/// <summary>Create the candle table if it doesn't exist (runs on the DB thread)</summary>
		private void EnsureSchema()
		{
			m_db.Cmd(
				"CREATE TABLE IF NOT EXISTS Candles (" +
				"  Symbol TEXT NOT NULL," +
				"  TimeFrame TEXT NOT NULL," +
				"  [Timestamp] INTEGER NOT NULL," +
				"  [Open] REAL NOT NULL," +
				"  [High] REAL NOT NULL," +
				"  [Low] REAL NOT NULL," +
				"  [Close] REAL NOT NULL," +
				"  [Volume] REAL NOT NULL," +
				"  PRIMARY KEY (Symbol, TimeFrame, [Timestamp])" +
				")").Execute();
		}
		private (long? Min, long? Max) ExtentImpl(string symbol, EMarketPeriod tf)
		{
			using var cmd = m_db
				.Cmd("SELECT MIN([Timestamp]), MAX([Timestamp]), COUNT(*) FROM Candles WHERE Symbol=@sym AND TimeFrame=@tf")
				.AddParam("@sym", symbol)
				.AddParam("@tf", tf.Assoc<string>());
			using var r = cmd.ExecuteQuery();
			if (!r.Read() || r.Get<long>(2) == 0)
				return (null, null);
			return (r.Get<long>(0), r.Get<long>(1));
		}
		private List<Candle> ReadImpl(string symbol, EMarketPeriod tf, long begMs, long endMs)
		{
			using var cmd = m_db
				.Cmd("SELECT [Timestamp],[Open],[High],[Low],[Close],[Volume] FROM Candles " +
					"WHERE Symbol=@sym AND TimeFrame=@tf AND [Timestamp]>=@beg AND [Timestamp]<=@end ORDER BY [Timestamp]")
				.AddParam("@sym", symbol)
				.AddParam("@tf", tf.Assoc<string>())
				.AddParam("@beg", begMs)
				.AddParam("@end", endMs);
			using var r = cmd.ExecuteQuery();
			var list = new List<Candle>();
			while (r.Read())
			{
				list.Add(new Candle(
					r.Get<long>(0),
					r.Get<double>(1),
					r.Get<double>(2),
					r.Get<double>(3),
					r.Get<double>(4),
					r.Get<double>(5)));
			}
			return list;
		}
		private void UpsertImpl(string symbol, EMarketPeriod tf, IReadOnlyList<Candle> candles)
		{
			if (candles.Count == 0)
				return;

			var tfs = tf.Assoc<string>();
			using var tx = m_db.BeginTransaction();
			const string sql =
				"INSERT OR REPLACE INTO Candles (Symbol,TimeFrame,[Timestamp],[Open],[High],[Low],[Close],[Volume]) " +
				"VALUES (@sym,@tf,@ts,@o,@h,@l,@c,@v)";
			foreach (var c in candles)
			{
				using var cmd = m_db.Cmd(sql, tx)
					.AddParam("@sym", symbol)
					.AddParam("@tf", tfs)
					.AddParam("@ts", c.Timestamp)
					.AddParam("@o", c.Open)
					.AddParam("@h", c.High)
					.AddParam("@l", c.Low)
					.AddParam("@c", c.Close)
					.AddParam("@v", c.Volume);
				cmd.Execute();
			}
			tx.Commit();
		}

		/// <summary>Marshal 'fn' onto the DB thread, block for the result, and rethrow any error</summary>
		private T Run<T>(Func<T> fn)
		{
			var result = default(T)!;
			ExceptionDispatchInfo? err = null;
			using var done = new ManualResetEventSlim(false);
			m_queue.Add(() =>
			{
				try { result = fn(); }
				catch (Exception ex) { err = ExceptionDispatchInfo.Capture(ex); }
				finally { done.Set(); }
			});
			done.Wait();
			err?.Throw();
			return result;
		}
	}
}
