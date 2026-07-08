using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Binance.API;

namespace Lab.Model
{
	/// <summary>
	/// Top-level data access for the laboratory: composes the Binance source and the local
	/// SQLite cache. History is downloaded on demand, cached, and served from the cache so the
	/// UI can scrub through it offline.
	///
	/// Construct on the UI thread (BinanceSource needs a SynchronizationContext), but the async
	/// download/cache methods may be awaited from any thread.</summary>
	public sealed class MarketData : IDisposable
	{
		private readonly CandleDb m_db;
		private readonly BinanceSource m_src;

		public MarketData(string dbFilepath, CancellationToken shutdown)
		{
			m_db = new CandleDb(dbFilepath);
			m_src = new BinanceSource(shutdown);
		}
		public void Dispose()
		{
			m_src.Dispose();
			m_db.Dispose();
		}

		/// <summary>Initialise the exchange connection (symbol map + throttle limits)</summary>
		public Task InitAsync() => m_src.InitAsync();

		/// <summary>Read cached candles only (no network access)</summary>
		public List<Candle> ReadCached(string symbol, EMarketPeriod tf, DateTimeOffset beg, DateTimeOffset end)
		{
			return m_db.Read(symbol, tf, beg.ToUnixTimeMilliseconds(), end.ToUnixTimeMilliseconds());
		}

		/// <summary>
		/// Ensure the range [beg, end] is cached (downloading any missing spans), then return it.
		/// Only the gaps below the cached minimum and above the cached maximum are downloaded, so
		/// repeat requests over an already-cached range hit the network minimally.</summary>
		public async Task<List<Candle>> GetHistoryAsync(string symbol, EMarketPeriod tf, DateTimeOffset beg, DateTimeOffset end, IProgress<string>? progress, CancellationToken cancel)
		{
			symbol = symbol.ToUpperInvariant();
			var begMs = beg.ToUnixTimeMilliseconds();
			var endMs = end.ToUnixTimeMilliseconds();

			var (min, max) = m_db.Extent(symbol, tf);
			void Save(IReadOnlyList<Candle> batch) => m_db.Upsert(symbol, tf, batch);

			if (min == null || max == null)
			{
				// Nothing cached - download the whole range.
				progress?.Report($"Downloading {symbol} history...");
				await m_src.DownloadAsync(symbol, tf, beg, end, Save, progress, cancel);
			}
			else
			{
				// Backfill the span below the cached minimum.
				if (begMs < min.Value)
				{
					progress?.Report($"Backfilling earlier {symbol} history...");
					var gapEnd = DateTimeOffset.FromUnixTimeMilliseconds(min.Value);
					await m_src.DownloadAsync(symbol, tf, beg, gapEnd, Save, progress, cancel);
				}

				// Extend the span above the cached maximum.
				if (endMs > max.Value)
				{
					progress?.Report($"Fetching recent {symbol} history...");
					var gapBeg = DateTimeOffset.FromUnixTimeMilliseconds(max.Value + 1);
					await m_src.DownloadAsync(symbol, tf, gapBeg, end, Save, progress, cancel);
				}
			}

			progress?.Report("Reading from cache...");
			return m_db.Read(symbol, tf, begMs, endMs);
		}
	}
}
