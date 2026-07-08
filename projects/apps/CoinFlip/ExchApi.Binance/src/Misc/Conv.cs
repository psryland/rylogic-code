using System;
using Binance.API.DomainObjects;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace Binance.API
{
	public static class Extn
	{
		/// <summary>True if binance considers this order type an algorithm order type</summary>
		public static bool IsAlgo(this EOrderType ot)
		{
			return ot == EOrderType.STOP_LOSS || ot == EOrderType.STOP_LOSS_LIMIT || ot == EOrderType.TAKE_PROFIT || ot == EOrderType.TAKE_PROFIT_LIMIT;
		}
	}

	internal static class Conv
	{
		public static TimeSpan ToTimeSpan(this EMarketPeriod period, double count = 1)
		{
			switch (period)
			{
			default: throw new Exception($"Unknown market period: {period}");
			case EMarketPeriod.None:      return TimeSpan.Zero;
			case EMarketPeriod.Minutes1:  return TimeSpan.FromMinutes(1 * count);
			case EMarketPeriod.Minutes3:  return TimeSpan.FromMinutes(3 * count);
			case EMarketPeriod.Minutes5:  return TimeSpan.FromMinutes(5 * count);
			case EMarketPeriod.Minutes15: return TimeSpan.FromMinutes(15 * count);
			case EMarketPeriod.Minutes30: return TimeSpan.FromMinutes(30 * count);
			case EMarketPeriod.Hours1:    return TimeSpan.FromHours(1 * count);
			case EMarketPeriod.Hours2:    return TimeSpan.FromHours(2 * count);
			case EMarketPeriod.Hours4:    return TimeSpan.FromHours(4 * count);
			case EMarketPeriod.Hours6:    return TimeSpan.FromHours(6 * count);
			case EMarketPeriod.Hours8:    return TimeSpan.FromHours(8 * count);
			case EMarketPeriod.Hours12:   return TimeSpan.FromHours(12 * count);
			case EMarketPeriod.Day1:      return TimeSpan.FromDays(1 * count);
			case EMarketPeriod.Day3:      return TimeSpan.FromDays(3 * count);
			case EMarketPeriod.Week1:     return TimeSpan.FromDays(7 * count);
			case EMarketPeriod.Month1:    return TimeSpan.FromDays(30.44 * count);
			}
		}
	}
	public class ToCurrencyPair :JsonConverter<CurrencyPair>
	{
		public override bool CanWrite => true;

		/// <inheritdoc/>
		public override CurrencyPair ReadJson(JsonReader reader, Type objectType, CurrencyPair existingValue, bool hasExistingValue, JsonSerializer serializer)
		{
			if (reader.Value is not string str) throw new Exception("Expected a string value");
			return CurrencyPair.Parse(str);
		}

		/// <inheritdoc/>
		public override void WriteJson(JsonWriter writer, CurrencyPair value, JsonSerializer serializer)
		{
			writer.WriteValue(value.Id);
		}
	}
	public class SymbolFilterConverter :JsonConverter
	{
		public override bool CanWrite => false;
		public override bool CanConvert(Type objectType) => false;

		/// <inheritdoc/>
		public override object ReadJson(JsonReader reader, Type objectType, object? existingValue, JsonSerializer serializer)
		{
			var jObject = JObject.Load(reader);
			if (jObject.ToObject<ServerRulesData.Filter>() is not ServerRulesData.Filter filter)
				throw new Exception("Expected a Filter");

			ServerRulesData.Filter item;
			switch (filter.FilterType)
			{
				case EFilterType.PRICE_FILTER:
				{
					item = new ServerRulesData.FilterPrice();
					break;
				}
				case EFilterType.PERCENT_PRICE:
				{
					item = new ServerRulesData.FilterPercentPrice();
					break;
				}
				case EFilterType.PERCENT_PRICE_BY_SIDE:
				{
					item = new ServerRulesData.FilterPercentPrice();
					break;
				}
				case EFilterType.NOTIONAL:
				{
					item = new ServerRulesData.FilterMinNotional();
					break;
				}
				case EFilterType.LOT_SIZE:
				{
					item = new ServerRulesData.FilterLotSize();
					break;
				}
				case EFilterType.MARKET_LOT_SIZE:
				{
					item = new ServerRulesData.FilterLotSize();
					break;
				}
				case EFilterType.MIN_NOTIONAL:
				{
					item = new ServerRulesData.FilterMinNotional();
					break;
				}
				case EFilterType.MAX_NUM_ORDERS:
				{
					item = new ServerRulesData.FilterLimit();
					break;
				}
				case EFilterType.MAX_NUM_ALGO_ORDERS:
				{
					item = new ServerRulesData.FilterLimit();
					break;
				}
				case EFilterType.EXCHANGE_MAX_NUM_ORDERS:
				{
					item = new ServerRulesData.FilterLimit();
					break;
				}
				case EFilterType.EXCHANGE_MAX_NUM_ALGO_ORDERS:
				{
					item = new ServerRulesData.FilterLimit();
					break;
				}
				case EFilterType.MAX_NUM_ICEBERG_ORDERS:
				{
					item = new ServerRulesData.FilterLimit();
					break;
				}
				case EFilterType.ICEBERG_PARTS:
				{
					item = new ServerRulesData.FilterLimit();
					break;
				}
				case EFilterType.MAX_POSITION:
				{
					item = new ServerRulesData.FilterMaxPosition();
					break;
				}
				case EFilterType.TRAILING_DELTA:
				{
					item = new ServerRulesData.FIlterTrailingDelta();
					break;
				}
				default:
				{
					// Tolerate filter types this (currently unmaintained) adapter doesn't model yet
					// (e.g. exchange additions), so market-data initialisation isn't broken by them.
					item = filter;
					break;
				}
			}

			serializer.Populate(jObject.CreateReader(), item);
			return item;
		}
		public override void WriteJson(JsonWriter writer, object? value, JsonSerializer serializer)
		{
			throw new NotImplementedException();
		}
	}
}