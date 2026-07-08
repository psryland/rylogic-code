using System;

namespace Lab.Model
{
	/// <summary>
	/// A single OHLCV candle. The raw sample unit for all downstream signal analysis.
	/// 'Timestamp' is the candle open time in Unix milliseconds (UTC). Property names
	/// match the columns in the SQLite cache.</summary>
	public sealed class Candle
	{
		public Candle()
		{ }
		public Candle(long timestamp, double open, double high, double low, double close, double volume)
		{
			Timestamp = timestamp;
			Open = open;
			High = high;
			Low = low;
			Close = close;
			Volume = volume;
		}

		/// <summary>Candle open time, Unix milliseconds (UTC)</summary>
		public long Timestamp { get; set; }

		/// <summary>Open price</summary>
		public double Open { get; set; }

		/// <summary>Highest price during the candle</summary>
		public double High { get; set; }

		/// <summary>Lowest price during the candle</summary>
		public double Low { get; set; }

		/// <summary>Close price</summary>
		public double Close { get; set; }

		/// <summary>Volume traded during the candle (base asset)</summary>
		public double Volume { get; set; }

		/// <summary>Candle open time as a DateTimeOffset (UTC)</summary>
		public DateTimeOffset TimeUtc => DateTimeOffset.FromUnixTimeMilliseconds(Timestamp);

		/// <summary>Mean of OHLC - a simple de-noised price estimate</summary>
		public double Median => 0.25 * (Open + High + Low + Close);

		/// <summary>Typical price (HLC/3)</summary>
		public double Typical => (High + Low + Close) / 3.0;
	}
}
