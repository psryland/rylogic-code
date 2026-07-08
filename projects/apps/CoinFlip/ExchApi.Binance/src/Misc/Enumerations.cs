using Rylogic.Attrib;

namespace Binance.API
{
	/// <summary>Error codes to distinguish replies</summary>
	public enum EErrorCode
	{
		Success,
		Failure,
		InvalidParameter,
		TooMuchDataRequested,
		InvalidTimestamp = -1021,
		InvalidSymbol = -1121,
	}

	/// <summary>Endpoint security types</summary>
	public enum ESecurityType
	{
		PUBLIC,
		TRADE,
		USER_DATA,
		USER_STREAM,
		MARKET_DATA,
	}

	/// <summary>Status for a currency pair</summary>
	public enum ESymbolStatus
	{
		PRE_TRADING,
		TRADING,
		POST_TRADING,
		END_OF_DAY,
		HALT,
		AUCTION_MATCH,
		BREAK,
	}

	/// <summary></summary>
	public enum ESymbolType
	{
		SPOT,
	}

	/// <summary>The current state of an order</summary>
	public enum EOrderStatus
	{
		NEW,
		PARTIALLY_FILLED,
		FILLED,
		CANCELED,
		PENDING_CANCEL, // (currently unused)
		REJECTED,
		EXPIRED,
	}

	/// <summary>Actions associated with an order (i.e. what happen, or what to do to it)</summary>
	public enum EExecutionType
	{
		NEW,
		CANCELED,
		REPLACED,
		REJECTED,
		TRADE,
		EXPIRED,
	}

	/// <summary>Reasons for not allowing an order</summary>
	public enum EOrderRejectReason
	{
		NONE,
		UNKNOWN_INSTRUMENT,
		MARKET_CLOSED,
		PRICE_QTY_EXCEED_HARD_LIMITS,
		UNKNOWN_ORDER,
		DUPLICATE_ORDER,
		UNKNOWN_ACCOUNT,
		INSUFFICIENT_BALANCE,
		ACCOUNT_INACTIVE,
		ACCOUNT_CANNOT_SETTLE,
	}

	/// <summary>Order type</summary>
	public enum EOrderType
	{
		/// <summary>Trade at a fixed price</summary>
		LIMIT,

		/// <summary>Trade at the current market price, consuming offers until the trade is filled</summary>
		MARKET,

		/// <summary>When the spot price reaches the target price, place a market order. Stop loss orders are not visible in the order book</summary>
		STOP_LOSS,

		/// <summary>When the spot price reaches the target price, place a limit order. Stop loss limit orders are not visible in the order book</summary>
		STOP_LOSS_LIMIT,

		/// <summary>When the spot price reaches the target price, place a market order. Take profit orders are not visible in the order book</summary>
		TAKE_PROFIT,

		/// <summary>When the spot price reaches the target price, place a limit order. Take profit limit orders are not visible in the order book</summary>
		TAKE_PROFIT_LIMIT,

		/// <summary>Limit maker orders are limit orders that will be rejected if they would immediately match and trade as a taker. The distinction effects the fees charged</summary>
		LIMIT_MAKER,
	}

	/// <summary>Trade type</summary>
	public enum EOrderSide
	{
		BUY,
		SELL,
	}

	/// <summary>Order lifetime</summary>
	public enum ETimeInForce
	{
		GTC, // Good till cancelled
		IOC, // 
		FOK, //
	}

	/// <summary></summary>
	public enum EFilterType
	{
		PRICE_FILTER,
		PERCENT_PRICE,
		LOT_SIZE,
		MIN_NOTIONAL,
		ICEBERG_PARTS,
		MARKET_LOT_SIZE,
		MAX_NUM_ORDERS,
		MAX_NUM_ALGO_ORDERS,
		MAX_NUM_ICEBERG_ORDERS,
		EXCHANGE_MAX_NUM_ORDERS,
		EXCHANGE_MAX_NUM_ALGO_ORDERS,
		MAX_POSITION,
		TRAILING_DELTA,
		PERCENT_PRICE_BY_SIDE,
		NOTIONAL,
		MAX_NUM_ORDER_LISTS,
		MAX_NUM_ORDER_AMENDS,
		EXCHANGE_MAX_NUM_ICEBERG_ORDERS,
	}

	public enum ERateLimitType
	{
		RAW_REQUEST,
		RAW_REQUESTS,
		REQUEST_WEIGHT,
		ORDERS,
	}

	/// <summary>Represents a time frame of a market.</summary>
	public enum EMarketPeriod
	{
		None = 0,
		[Assoc("1m")] Minutes1,
		[Assoc("3m")] Minutes3,
		[Assoc("5m")] Minutes5,
		[Assoc("15m")] Minutes15,
		[Assoc("30m")] Minutes30,
		[Assoc("1h")] Hours1,
		[Assoc("2h")] Hours2,
		[Assoc("4h")] Hours4,
		[Assoc("6h")] Hours6,
		[Assoc("8h")] Hours8,
		[Assoc("12h")] Hours12,
		[Assoc("1d")] Day1,
		[Assoc("3d")] Day3,
		[Assoc("1w")] Week1,
		[Assoc("1M")] Month1,
	}

	/// <summary>Status of a deposit</summary>
	public enum EDepositStatus
	{
		Pending = 0,
		Success = 1,
		CreditedButCannotWithdraw = 6,
	}

	/// <summary>Status of a withdrawal</summary>
	public enum EWithdrawalStatus
	{
		EmailSent        = 0,
		Cancelled        = 1,
		AwaitingApproval = 2,
		Rejected         = 3,
		Processing       = 4,
		Failure          = 5,
		Completed        = 6,
	}
}
