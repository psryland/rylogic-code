using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Binance.API;
using Binance.API.DomainObjects;
using Rylogic.Attrib;
using Rylogic.Utility;

namespace Lab.Model
{
	/// <summary>
	/// Fetches historical candle data from Binance. Only public market-data endpoints
	/// are used, so no API key or secret is required.</summary>
	public sealed class BinanceSource : IDisposable
	{
		// Notes:
		//  - Binance 'klines' returns candles with open-time >= startTime, capped per request.
		//    We page forward by requesting from (last-open-time + 1ms) until 'end' is reached.
		//  - Must be constructed on a thread that has a SynchronizationContext (e.g. the UI thread),
		//    because ExchangeApi captures SynchronizationContext.Current.

		private readonly Logger m_log;
		private BinanceApi m_api;

		public BinanceSource(CancellationToken shutdown)
		{
			m_log = new Logger("Lab", new LogToDebug());
			m_api = new BinanceApi(string.Empty, string.Empty, shutdown, m_log);
		}
		public void Dispose()
		{
			Util.Dispose(ref m_api!);
			m_log.Dispose();
		}

		/// <summary>Populate exchange rules (the symbol map and request-throttle limits). Call once before downloading.</summary>
		public Task InitAsync() => m_api.InitAsync();

		/// <summary>
		/// Download candles in the range [beg, end), invoking 'onBatch' for each page as it arrives.
		/// Returns the total number of candles downloaded.</summary>
		public async Task<int> DownloadAsync(string symbol, EMarketPeriod tf, DateTimeOffset beg, DateTimeOffset end, Action<IReadOnlyList<Candle>> onBatch, IProgress<string>? progress, CancellationToken cancel)
		{
			var pair = CurrencyPair.Parse(symbol.ToUpperInvariant());
			var tfs = tf.Assoc<string>();
			var endMs = end.ToUnixTimeMilliseconds();
			var t = beg;
			var total = 0;

			// The 5000 iteration cap is a safety valve against a runaway loop (~2.5M candles).
			for (var iter = 0; t < end && iter != 5000; ++iter)
			{
				cancel.ThrowIfCancellationRequested();

				var data = await m_api.GetChartData(pair, tf, new UnixMSec(t.ToUnixTimeMilliseconds()), new UnixMSec(endMs), cancel);
				if (data.Count == 0)
					break;

				var candles = data
					.Select(x => new Candle(x.Time.ToUnixTimeMilliseconds(), x.Open, x.High, x.Low, x.Close, x.Volume))
					.ToList();
				onBatch(candles);
				total += candles.Count;

				var last = data[data.Count - 1].Time;
				progress?.Report($"{symbol} {tfs}: {total:N0} candles (through {last:yyyy-MM-dd})");

				// Advance just past the last candle. Because candles are spaced by 'tf', the next
				// batch begins at the following candle, guaranteeing forward progress.
				var next = last + TimeSpan.FromMilliseconds(1);
				if (next <= t)
					break;
				t = next;
			}
			return total;
		}
	}
}
