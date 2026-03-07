//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/openxr/openxr.h"
#include "view3d-12/src/openxr/conversion.h"
#include "openxr/openxr.h"
#include "openxr/openxr_platform.h"

namespace pr::rdr12::openxr
{
	using XrInstancePtr = std::shared_ptr<XrInstance_T>;
	using XrSessionPtr = std::shared_ptr<XrSession_T>;
	using XrSwapchainPtr = std::shared_ptr<XrSwapchain_T>;

	struct CreateInfo : XrInstanceCreateInfo
	{
		std::string m_app_name;
		std::string m_engine_name;
		std::vector<char const*> m_layers;
		std::vector<char const*> m_extensions;

		CreateInfo(std::string_view app_name, std::string_view engine_name, int version)
			: XrInstanceCreateInfo{ .type = XR_TYPE_INSTANCE_CREATE_INFO }
			, m_app_name(app_name)
			, m_engine_name(engine_name)
		{
			applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

			std::strncpy(applicationInfo.applicationName, m_app_name.data(), std::min(_countof(applicationInfo.applicationName), m_app_name.size()));
			applicationInfo.applicationVersion = version;

			std::strncpy(applicationInfo.engineName, m_engine_name.data(), std::min(_countof(applicationInfo.engineName), m_engine_name.size()));
			applicationInfo.engineVersion = version;
		}
		
		CreateInfo& ApiVersion(XrVersion version)
		{
			applicationInfo.apiVersion = version;
			return *this;
		}
		CreateInfo& Flags(XrInstanceCreateFlags f)
		{
			createFlags = f;
			return *this;
		}
		CreateInfo& Layer(char const* layer_name)
		{
			m_layers.push_back(layer_name);
			return build();
		}
		CreateInfo& Extension(char const* ext_name)
		{
			m_extensions.push_back(ext_name);
			return build();
		}
		CreateInfo& build()
		{
			enabledApiLayerCount = static_cast<uint32_t>(m_layers.size());
			enabledApiLayerNames = m_layers.data();

			enabledExtensionCount = static_cast<uint32_t>(m_extensions.size());
			enabledExtensionNames = m_extensions.data();

			return *this;
		}
	};
	struct SystemGetInfo : XrSystemGetInfo
	{
		SystemGetInfo()
			: XrSystemGetInfo{ .type = XR_TYPE_SYSTEM_GET_INFO }
		{
		}
		SystemGetInfo& FormFactor(XrFormFactor ff)
		{
			formFactor = ff;
			return *this;
		}
	};
	struct SessionCreateInfo : XrSessionCreateInfo
	{
		SessionCreateInfo()
			: XrSessionCreateInfo{ .type = XR_TYPE_SESSION_CREATE_INFO }
		{
		}
		SessionCreateInfo& Flags(XrSessionCreateFlags f)
		{
			createFlags = f;
			return *this;
		}
		SessionCreateInfo& SystemId(XrSystemId id)
		{
			systemId = id;
			return *this;
		}
		SessionCreateInfo& Next(auto const& n)
		{
			next = &n;
			return *this;
		}
	};
	struct GraphicsBindingDx12 : XrGraphicsBindingD3D12KHR
	{
		GraphicsBindingDx12()
			: XrGraphicsBindingD3D12KHR{ .type = XR_TYPE_GRAPHICS_BINDING_D3D12_KHR }
		{
		}
		GraphicsBindingDx12& Device(ID3D12Device* d)
		{
			device = d;
			return *this;
		}
		GraphicsBindingDx12& CmdQueue(ID3D12CommandQueue* q)
		{
			queue = q;
			return *this;
		}
	};

	// Convert 'XrResult to a string
	inline char const* ToString(XrResult r)
	{
		#pragma region XrResult strings
		switch (r)
		{
			case XrResult::XR_SUCCESS: return "XR_SUCCESS";
			case XrResult::XR_TIMEOUT_EXPIRED: return "XR_TIMEOUT_EXPIRED";
			case XrResult::XR_SESSION_LOSS_PENDING: return "XR_SESSION_LOSS_PENDING";
			case XrResult::XR_EVENT_UNAVAILABLE: return "XR_EVENT_UNAVAILABLE";
			case XrResult::XR_SPACE_BOUNDS_UNAVAILABLE: return "XR_SPACE_BOUNDS_UNAVAILABLE";
			case XrResult::XR_SESSION_NOT_FOCUSED: return "XR_SESSION_NOT_FOCUSED";
			case XrResult::XR_FRAME_DISCARDED: return "XR_FRAME_DISCARDED";
			case XrResult::XR_ERROR_VALIDATION_FAILURE: return "XR_ERROR_VALIDATION_FAILURE";
			case XrResult::XR_ERROR_RUNTIME_FAILURE: return "XR_ERROR_RUNTIME_FAILURE";
			case XrResult::XR_ERROR_OUT_OF_MEMORY: return "XR_ERROR_OUT_OF_MEMORY";
			case XrResult::XR_ERROR_API_VERSION_UNSUPPORTED: return "XR_ERROR_API_VERSION_UNSUPPORTED";
			case XrResult::XR_ERROR_INITIALIZATION_FAILED: return "XR_ERROR_INITIALIZATION_FAILED";
			case XrResult::XR_ERROR_FUNCTION_UNSUPPORTED: return "XR_ERROR_FUNCTION_UNSUPPORTED";
			case XrResult::XR_ERROR_FEATURE_UNSUPPORTED: return "XR_ERROR_FEATURE_UNSUPPORTED";
			case XrResult::XR_ERROR_EXTENSION_NOT_PRESENT: return "XR_ERROR_EXTENSION_NOT_PRESENT";
			case XrResult::XR_ERROR_LIMIT_REACHED: return "XR_ERROR_LIMIT_REACHED";
			case XrResult::XR_ERROR_SIZE_INSUFFICIENT: return "XR_ERROR_SIZE_INSUFFICIENT";
			case XrResult::XR_ERROR_HANDLE_INVALID: return "XR_ERROR_HANDLE_INVALID";
			case XrResult::XR_ERROR_INSTANCE_LOST: return "XR_ERROR_INSTANCE_LOST";
			case XrResult::XR_ERROR_SESSION_RUNNING: return "XR_ERROR_SESSION_RUNNING";
			case XrResult::XR_ERROR_SESSION_NOT_RUNNING: return "XR_ERROR_SESSION_NOT_RUNNING";
			case XrResult::XR_ERROR_SESSION_LOST: return "XR_ERROR_SESSION_LOST";
			case XrResult::XR_ERROR_SYSTEM_INVALID: return "XR_ERROR_SYSTEM_INVALID";
			case XrResult::XR_ERROR_PATH_INVALID: return "XR_ERROR_PATH_INVALID";
			case XrResult::XR_ERROR_PATH_COUNT_EXCEEDED: return "XR_ERROR_PATH_COUNT_EXCEEDED";
			case XrResult::XR_ERROR_PATH_FORMAT_INVALID: return "XR_ERROR_PATH_FORMAT_INVALID";
			case XrResult::XR_ERROR_PATH_UNSUPPORTED: return "XR_ERROR_PATH_UNSUPPORTED";
			case XrResult::XR_ERROR_LAYER_INVALID: return "XR_ERROR_LAYER_INVALID";
			case XrResult::XR_ERROR_LAYER_LIMIT_EXCEEDED: return "XR_ERROR_LAYER_LIMIT_EXCEEDED";
			case XrResult::XR_ERROR_SWAPCHAIN_RECT_INVALID: return "XR_ERROR_SWAPCHAIN_RECT_INVALID";
			case XrResult::XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED: return "XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED";
			case XrResult::XR_ERROR_ACTION_TYPE_MISMATCH: return "XR_ERROR_ACTION_TYPE_MISMATCH";
			case XrResult::XR_ERROR_SESSION_NOT_READY: return "XR_ERROR_SESSION_NOT_READY";
			case XrResult::XR_ERROR_SESSION_NOT_STOPPING: return "XR_ERROR_SESSION_NOT_STOPPING";
			case XrResult::XR_ERROR_TIME_INVALID: return "XR_ERROR_TIME_INVALID";
			case XrResult::XR_ERROR_REFERENCE_SPACE_UNSUPPORTED: return "XR_ERROR_REFERENCE_SPACE_UNSUPPORTED";
			case XrResult::XR_ERROR_FILE_ACCESS_ERROR: return "XR_ERROR_FILE_ACCESS_ERROR";
			case XrResult::XR_ERROR_FILE_CONTENTS_INVALID: return "XR_ERROR_FILE_CONTENTS_INVALID";
			case XrResult::XR_ERROR_FORM_FACTOR_UNSUPPORTED: return "XR_ERROR_FORM_FACTOR_UNSUPPORTED";
			case XrResult::XR_ERROR_FORM_FACTOR_UNAVAILABLE: return "XR_ERROR_FORM_FACTOR_UNAVAILABLE";
			case XrResult::XR_ERROR_API_LAYER_NOT_PRESENT: return "XR_ERROR_API_LAYER_NOT_PRESENT";
			case XrResult::XR_ERROR_CALL_ORDER_INVALID: return "XR_ERROR_CALL_ORDER_INVALID";
			case XrResult::XR_ERROR_GRAPHICS_DEVICE_INVALID: return "XR_ERROR_GRAPHICS_DEVICE_INVALID";
			case XrResult::XR_ERROR_POSE_INVALID: return "XR_ERROR_POSE_INVALID";
			case XrResult::XR_ERROR_INDEX_OUT_OF_RANGE: return "XR_ERROR_INDEX_OUT_OF_RANGE";
			case XrResult::XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED: return "XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED";
			case XrResult::XR_ERROR_ENVIRONMENT_BLEND_MODE_UNSUPPORTED: return "XR_ERROR_ENVIRONMENT_BLEND_MODE_UNSUPPORTED";
			case XrResult::XR_ERROR_NAME_DUPLICATED: return "XR_ERROR_NAME_DUPLICATED";
			case XrResult::XR_ERROR_NAME_INVALID: return "XR_ERROR_NAME_INVALID";
			case XrResult::XR_ERROR_ACTIONSET_NOT_ATTACHED: return "XR_ERROR_ACTIONSET_NOT_ATTACHED";
			case XrResult::XR_ERROR_ACTIONSETS_ALREADY_ATTACHED: return "XR_ERROR_ACTIONSETS_ALREADY_ATTACHED";
			case XrResult::XR_ERROR_LOCALIZED_NAME_DUPLICATED: return "XR_ERROR_LOCALIZED_NAME_DUPLICATED";
			case XrResult::XR_ERROR_LOCALIZED_NAME_INVALID: return "XR_ERROR_LOCALIZED_NAME_INVALID";
			case XrResult::XR_ERROR_GRAPHICS_REQUIREMENTS_CALL_MISSING: return "XR_ERROR_GRAPHICS_REQUIREMENTS_CALL_MISSING";
			case XrResult::XR_ERROR_RUNTIME_UNAVAILABLE: return "XR_ERROR_RUNTIME_UNAVAILABLE";
			case XrResult::XR_ERROR_EXTENSION_DEPENDENCY_NOT_ENABLED: return "XR_ERROR_EXTENSION_DEPENDENCY_NOT_ENABLED";
			case XrResult::XR_ERROR_PERMISSION_INSUFFICIENT: return "XR_ERROR_PERMISSION_INSUFFICIENT";
			case XrResult::XR_ERROR_ANDROID_THREAD_SETTINGS_ID_INVALID_KHR: return "XR_ERROR_ANDROID_THREAD_SETTINGS_ID_INVALID_KHR";
			case XrResult::XR_ERROR_ANDROID_THREAD_SETTINGS_FAILURE_KHR: return "XR_ERROR_ANDROID_THREAD_SETTINGS_FAILURE_KHR";
			case XrResult::XR_ERROR_CREATE_SPATIAL_ANCHOR_FAILED_MSFT: return "XR_ERROR_CREATE_SPATIAL_ANCHOR_FAILED_MSFT";
			case XrResult::XR_ERROR_SECONDARY_VIEW_CONFIGURATION_TYPE_NOT_ENABLED_MSFT: return "XR_ERROR_SECONDARY_VIEW_CONFIGURATION_TYPE_NOT_ENABLED_MSFT";
			case XrResult::XR_ERROR_CONTROLLER_MODEL_KEY_INVALID_MSFT: return "XR_ERROR_CONTROLLER_MODEL_KEY_INVALID_MSFT";
			case XrResult::XR_ERROR_REPROJECTION_MODE_UNSUPPORTED_MSFT: return "XR_ERROR_REPROJECTION_MODE_UNSUPPORTED_MSFT";
			case XrResult::XR_ERROR_COMPUTE_NEW_SCENE_NOT_COMPLETED_MSFT: return "XR_ERROR_COMPUTE_NEW_SCENE_NOT_COMPLETED_MSFT";
			case XrResult::XR_ERROR_SCENE_COMPONENT_ID_INVALID_MSFT: return "XR_ERROR_SCENE_COMPONENT_ID_INVALID_MSFT";
			case XrResult::XR_ERROR_SCENE_COMPONENT_TYPE_MISMATCH_MSFT: return "XR_ERROR_SCENE_COMPONENT_TYPE_MISMATCH_MSFT";
			case XrResult::XR_ERROR_SCENE_MESH_BUFFER_ID_INVALID_MSFT: return "XR_ERROR_SCENE_MESH_BUFFER_ID_INVALID_MSFT";
			case XrResult::XR_ERROR_SCENE_COMPUTE_FEATURE_INCOMPATIBLE_MSFT: return "XR_ERROR_SCENE_COMPUTE_FEATURE_INCOMPATIBLE_MSFT";
			case XrResult::XR_ERROR_SCENE_COMPUTE_CONSISTENCY_MISMATCH_MSFT: return "XR_ERROR_SCENE_COMPUTE_CONSISTENCY_MISMATCH_MSFT";
			case XrResult::XR_ERROR_DISPLAY_REFRESH_RATE_UNSUPPORTED_FB: return "XR_ERROR_DISPLAY_REFRESH_RATE_UNSUPPORTED_FB";
			case XrResult::XR_ERROR_COLOR_SPACE_UNSUPPORTED_FB: return "XR_ERROR_COLOR_SPACE_UNSUPPORTED_FB";
			case XrResult::XR_ERROR_SPACE_COMPONENT_NOT_SUPPORTED_FB: return "XR_ERROR_SPACE_COMPONENT_NOT_SUPPORTED_FB";
			case XrResult::XR_ERROR_SPACE_COMPONENT_NOT_ENABLED_FB: return "XR_ERROR_SPACE_COMPONENT_NOT_ENABLED_FB";
			case XrResult::XR_ERROR_SPACE_COMPONENT_STATUS_PENDING_FB: return "XR_ERROR_SPACE_COMPONENT_STATUS_PENDING_FB";
			case XrResult::XR_ERROR_SPACE_COMPONENT_STATUS_ALREADY_SET_FB: return "XR_ERROR_SPACE_COMPONENT_STATUS_ALREADY_SET_FB";
			case XrResult::XR_ERROR_UNEXPECTED_STATE_PASSTHROUGH_FB: return "XR_ERROR_UNEXPECTED_STATE_PASSTHROUGH_FB";
			case XrResult::XR_ERROR_FEATURE_ALREADY_CREATED_PASSTHROUGH_FB: return "XR_ERROR_FEATURE_ALREADY_CREATED_PASSTHROUGH_FB";
			case XrResult::XR_ERROR_FEATURE_REQUIRED_PASSTHROUGH_FB: return "XR_ERROR_FEATURE_REQUIRED_PASSTHROUGH_FB";
			case XrResult::XR_ERROR_NOT_PERMITTED_PASSTHROUGH_FB: return "XR_ERROR_NOT_PERMITTED_PASSTHROUGH_FB";
			case XrResult::XR_ERROR_INSUFFICIENT_RESOURCES_PASSTHROUGH_FB: return "XR_ERROR_INSUFFICIENT_RESOURCES_PASSTHROUGH_FB";
			case XrResult::XR_ERROR_UNKNOWN_PASSTHROUGH_FB: return "XR_ERROR_UNKNOWN_PASSTHROUGH_FB";
			case XrResult::XR_ERROR_RENDER_MODEL_KEY_INVALID_FB: return "XR_ERROR_RENDER_MODEL_KEY_INVALID_FB";
			case XrResult::XR_RENDER_MODEL_UNAVAILABLE_FB: return "XR_RENDER_MODEL_UNAVAILABLE_FB";
			case XrResult::XR_ERROR_MARKER_NOT_TRACKED_VARJO: return "XR_ERROR_MARKER_NOT_TRACKED_VARJO";
			case XrResult::XR_ERROR_MARKER_ID_INVALID_VARJO: return "XR_ERROR_MARKER_ID_INVALID_VARJO";
			case XrResult::XR_ERROR_MARKER_DETECTOR_PERMISSION_DENIED_ML: return "XR_ERROR_MARKER_DETECTOR_PERMISSION_DENIED_ML";
			case XrResult::XR_ERROR_MARKER_DETECTOR_LOCATE_FAILED_ML: return "XR_ERROR_MARKER_DETECTOR_LOCATE_FAILED_ML";
			case XrResult::XR_ERROR_MARKER_DETECTOR_INVALID_DATA_QUERY_ML: return "XR_ERROR_MARKER_DETECTOR_INVALID_DATA_QUERY_ML";
			case XrResult::XR_ERROR_MARKER_DETECTOR_INVALID_CREATE_INFO_ML: return "XR_ERROR_MARKER_DETECTOR_INVALID_CREATE_INFO_ML";
			case XrResult::XR_ERROR_MARKER_INVALID_ML: return "XR_ERROR_MARKER_INVALID_ML";
			case XrResult::XR_ERROR_LOCALIZATION_MAP_INCOMPATIBLE_ML: return "XR_ERROR_LOCALIZATION_MAP_INCOMPATIBLE_ML";
			case XrResult::XR_ERROR_LOCALIZATION_MAP_UNAVAILABLE_ML: return "XR_ERROR_LOCALIZATION_MAP_UNAVAILABLE_ML";
			case XrResult::XR_ERROR_LOCALIZATION_MAP_FAIL_ML: return "XR_ERROR_LOCALIZATION_MAP_FAIL_ML";
			case XrResult::XR_ERROR_LOCALIZATION_MAP_IMPORT_EXPORT_PERMISSION_DENIED_ML: return "XR_ERROR_LOCALIZATION_MAP_IMPORT_EXPORT_PERMISSION_DENIED_ML";
			case XrResult::XR_ERROR_LOCALIZATION_MAP_PERMISSION_DENIED_ML: return "XR_ERROR_LOCALIZATION_MAP_PERMISSION_DENIED_ML";
			case XrResult::XR_ERROR_LOCALIZATION_MAP_ALREADY_EXISTS_ML: return "XR_ERROR_LOCALIZATION_MAP_ALREADY_EXISTS_ML";
			case XrResult::XR_ERROR_LOCALIZATION_MAP_CANNOT_EXPORT_CLOUD_MAP_ML: return "XR_ERROR_LOCALIZATION_MAP_CANNOT_EXPORT_CLOUD_MAP_ML";
			case XrResult::XR_ERROR_SPATIAL_ANCHORS_PERMISSION_DENIED_ML: return "XR_ERROR_SPATIAL_ANCHORS_PERMISSION_DENIED_ML";
			case XrResult::XR_ERROR_SPATIAL_ANCHORS_NOT_LOCALIZED_ML: return "XR_ERROR_SPATIAL_ANCHORS_NOT_LOCALIZED_ML";
			case XrResult::XR_ERROR_SPATIAL_ANCHORS_OUT_OF_MAP_BOUNDS_ML: return "XR_ERROR_SPATIAL_ANCHORS_OUT_OF_MAP_BOUNDS_ML";
			case XrResult::XR_ERROR_SPATIAL_ANCHORS_SPACE_NOT_LOCATABLE_ML: return "XR_ERROR_SPATIAL_ANCHORS_SPACE_NOT_LOCATABLE_ML";
			case XrResult::XR_ERROR_SPATIAL_ANCHORS_ANCHOR_NOT_FOUND_ML: return "XR_ERROR_SPATIAL_ANCHORS_ANCHOR_NOT_FOUND_ML";
			case XrResult::XR_ERROR_SPATIAL_ANCHOR_NAME_NOT_FOUND_MSFT: return "XR_ERROR_SPATIAL_ANCHOR_NAME_NOT_FOUND_MSFT";
			case XrResult::XR_ERROR_SPATIAL_ANCHOR_NAME_INVALID_MSFT: return "XR_ERROR_SPATIAL_ANCHOR_NAME_INVALID_MSFT";
			case XrResult::XR_SCENE_MARKER_DATA_NOT_STRING_MSFT: return "XR_SCENE_MARKER_DATA_NOT_STRING_MSFT";
			case XrResult::XR_ERROR_SPACE_MAPPING_INSUFFICIENT_FB: return "XR_ERROR_SPACE_MAPPING_INSUFFICIENT_FB";
			case XrResult::XR_ERROR_SPACE_LOCALIZATION_FAILED_FB: return "XR_ERROR_SPACE_LOCALIZATION_FAILED_FB";
			case XrResult::XR_ERROR_SPACE_NETWORK_TIMEOUT_FB: return "XR_ERROR_SPACE_NETWORK_TIMEOUT_FB";
			case XrResult::XR_ERROR_SPACE_NETWORK_REQUEST_FAILED_FB: return "XR_ERROR_SPACE_NETWORK_REQUEST_FAILED_FB";
			case XrResult::XR_ERROR_SPACE_CLOUD_STORAGE_DISABLED_FB: return "XR_ERROR_SPACE_CLOUD_STORAGE_DISABLED_FB";
			case XrResult::XR_ERROR_SPACE_INSUFFICIENT_RESOURCES_META: return "XR_ERROR_SPACE_INSUFFICIENT_RESOURCES_META";
			case XrResult::XR_ERROR_SPACE_STORAGE_AT_CAPACITY_META: return "XR_ERROR_SPACE_STORAGE_AT_CAPACITY_META";
			case XrResult::XR_ERROR_SPACE_INSUFFICIENT_VIEW_META: return "XR_ERROR_SPACE_INSUFFICIENT_VIEW_META";
			case XrResult::XR_ERROR_SPACE_PERMISSION_INSUFFICIENT_META: return "XR_ERROR_SPACE_PERMISSION_INSUFFICIENT_META";
			case XrResult::XR_ERROR_SPACE_RATE_LIMITED_META: return "XR_ERROR_SPACE_RATE_LIMITED_META";
			case XrResult::XR_ERROR_SPACE_TOO_DARK_META: return "XR_ERROR_SPACE_TOO_DARK_META";
			case XrResult::XR_ERROR_SPACE_TOO_BRIGHT_META: return "XR_ERROR_SPACE_TOO_BRIGHT_META";
			case XrResult::XR_ERROR_PASSTHROUGH_COLOR_LUT_BUFFER_SIZE_MISMATCH_META: return "XR_ERROR_PASSTHROUGH_COLOR_LUT_BUFFER_SIZE_MISMATCH_META";
			case XrResult::XR_ENVIRONMENT_DEPTH_NOT_AVAILABLE_META: return "XR_ENVIRONMENT_DEPTH_NOT_AVAILABLE_META";
			case XrResult::XR_ERROR_RENDER_MODEL_ID_INVALID_EXT: return "XR_ERROR_RENDER_MODEL_ID_INVALID_EXT";
			case XrResult::XR_ERROR_RENDER_MODEL_ASSET_UNAVAILABLE_EXT: return "XR_ERROR_RENDER_MODEL_ASSET_UNAVAILABLE_EXT";
			case XrResult::XR_ERROR_RENDER_MODEL_GLTF_EXTENSION_REQUIRED_EXT: return "XR_ERROR_RENDER_MODEL_GLTF_EXTENSION_REQUIRED_EXT";
			case XrResult::XR_ERROR_NOT_INTERACTION_RENDER_MODEL_EXT: return "XR_ERROR_NOT_INTERACTION_RENDER_MODEL_EXT";
			case XrResult::XR_ERROR_HINT_ALREADY_SET_QCOM: return "XR_ERROR_HINT_ALREADY_SET_QCOM";
			case XrResult::XR_ERROR_NOT_AN_ANCHOR_HTC: return "XR_ERROR_NOT_AN_ANCHOR_HTC";
			case XrResult::XR_ERROR_SPATIAL_ENTITY_ID_INVALID_BD: return "XR_ERROR_SPATIAL_ENTITY_ID_INVALID_BD";
			case XrResult::XR_ERROR_SPATIAL_SENSING_SERVICE_UNAVAILABLE_BD: return "XR_ERROR_SPATIAL_SENSING_SERVICE_UNAVAILABLE_BD";
			case XrResult::XR_ERROR_ANCHOR_NOT_SUPPORTED_FOR_ENTITY_BD: return "XR_ERROR_ANCHOR_NOT_SUPPORTED_FOR_ENTITY_BD";
			case XrResult::XR_ERROR_SPATIAL_ANCHOR_NOT_FOUND_BD: return "XR_ERROR_SPATIAL_ANCHOR_NOT_FOUND_BD";
			case XrResult::XR_ERROR_SPATIAL_ANCHOR_SHARING_NETWORK_TIMEOUT_BD: return "XR_ERROR_SPATIAL_ANCHOR_SHARING_NETWORK_TIMEOUT_BD";
			case XrResult::XR_ERROR_SPATIAL_ANCHOR_SHARING_AUTHENTICATION_FAILURE_BD: return "XR_ERROR_SPATIAL_ANCHOR_SHARING_AUTHENTICATION_FAILURE_BD";
			case XrResult::XR_ERROR_SPATIAL_ANCHOR_SHARING_NETWORK_FAILURE_BD: return "XR_ERROR_SPATIAL_ANCHOR_SHARING_NETWORK_FAILURE_BD";
			case XrResult::XR_ERROR_SPATIAL_ANCHOR_SHARING_LOCALIZATION_FAIL_BD: return "XR_ERROR_SPATIAL_ANCHOR_SHARING_LOCALIZATION_FAIL_BD";
			case XrResult::XR_ERROR_SPATIAL_ANCHOR_SHARING_MAP_INSUFFICIENT_BD: return "XR_ERROR_SPATIAL_ANCHOR_SHARING_MAP_INSUFFICIENT_BD";
			case XrResult::XR_ERROR_SCENE_CAPTURE_FAILURE_BD: return "XR_ERROR_SCENE_CAPTURE_FAILURE_BD";
			case XrResult::XR_ERROR_SPACE_NOT_LOCATABLE_EXT: return "XR_ERROR_SPACE_NOT_LOCATABLE_EXT";
			case XrResult::XR_ERROR_PLANE_DETECTION_PERMISSION_DENIED_EXT: return "XR_ERROR_PLANE_DETECTION_PERMISSION_DENIED_EXT";
			case XrResult::XR_ERROR_MISMATCHING_TRACKABLE_TYPE_ANDROID: return "XR_ERROR_MISMATCHING_TRACKABLE_TYPE_ANDROID";
			case XrResult::XR_ERROR_TRACKABLE_TYPE_NOT_SUPPORTED_ANDROID: return "XR_ERROR_TRACKABLE_TYPE_NOT_SUPPORTED_ANDROID";
			case XrResult::XR_ERROR_ANCHOR_ID_NOT_FOUND_ANDROID: return "XR_ERROR_ANCHOR_ID_NOT_FOUND_ANDROID";
			case XrResult::XR_ERROR_ANCHOR_ALREADY_PERSISTED_ANDROID: return "XR_ERROR_ANCHOR_ALREADY_PERSISTED_ANDROID";
			case XrResult::XR_ERROR_ANCHOR_NOT_TRACKING_ANDROID: return "XR_ERROR_ANCHOR_NOT_TRACKING_ANDROID";
			case XrResult::XR_ERROR_PERSISTED_DATA_NOT_READY_ANDROID: return "XR_ERROR_PERSISTED_DATA_NOT_READY_ANDROID";
			case XrResult::XR_ERROR_SERVICE_NOT_READY_ANDROID: return "XR_ERROR_SERVICE_NOT_READY_ANDROID";
			case XrResult::XR_ERROR_FUTURE_PENDING_EXT: return "XR_ERROR_FUTURE_PENDING_EXT";
			case XrResult::XR_ERROR_FUTURE_INVALID_EXT: return "XR_ERROR_FUTURE_INVALID_EXT";
			case XrResult::XR_ERROR_SYSTEM_NOTIFICATION_PERMISSION_DENIED_ML: return "XR_ERROR_SYSTEM_NOTIFICATION_PERMISSION_DENIED_ML";
			case XrResult::XR_ERROR_SYSTEM_NOTIFICATION_INCOMPATIBLE_SKU_ML: return "XR_ERROR_SYSTEM_NOTIFICATION_INCOMPATIBLE_SKU_ML";
			case XrResult::XR_ERROR_WORLD_MESH_DETECTOR_PERMISSION_DENIED_ML: return "XR_ERROR_WORLD_MESH_DETECTOR_PERMISSION_DENIED_ML";
			case XrResult::XR_ERROR_WORLD_MESH_DETECTOR_SPACE_NOT_LOCATABLE_ML: return "XR_ERROR_WORLD_MESH_DETECTOR_SPACE_NOT_LOCATABLE_ML";
			case XrResult::XR_ERROR_FACIAL_EXPRESSION_PERMISSION_DENIED_ML: return "XR_ERROR_FACIAL_EXPRESSION_PERMISSION_DENIED_ML";
			case XrResult::XR_ERROR_COLOCATION_DISCOVERY_NETWORK_FAILED_META: return "XR_ERROR_COLOCATION_DISCOVERY_NETWORK_FAILED_META";
			case XrResult::XR_ERROR_COLOCATION_DISCOVERY_NO_DISCOVERY_METHOD_META: return "XR_ERROR_COLOCATION_DISCOVERY_NO_DISCOVERY_METHOD_META";
			case XrResult::XR_COLOCATION_DISCOVERY_ALREADY_ADVERTISING_META: return "XR_COLOCATION_DISCOVERY_ALREADY_ADVERTISING_META";
			case XrResult::XR_COLOCATION_DISCOVERY_ALREADY_DISCOVERING_META: return "XR_COLOCATION_DISCOVERY_ALREADY_DISCOVERING_META";
			case XrResult::XR_ERROR_SPACE_GROUP_NOT_FOUND_META: return "XR_ERROR_SPACE_GROUP_NOT_FOUND_META";
			case XrResult::XR_ERROR_ANCHOR_NOT_OWNED_BY_CALLER_ANDROID: return "XR_ERROR_ANCHOR_NOT_OWNED_BY_CALLER_ANDROID";
			case XrResult::XR_ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT: return "XR_ERROR_SPATIAL_CAPABILITY_UNSUPPORTED_EXT";
			case XrResult::XR_ERROR_SPATIAL_ENTITY_ID_INVALID_EXT: return "XR_ERROR_SPATIAL_ENTITY_ID_INVALID_EXT";
			case XrResult::XR_ERROR_SPATIAL_BUFFER_ID_INVALID_EXT: return "XR_ERROR_SPATIAL_BUFFER_ID_INVALID_EXT";
			case XrResult::XR_ERROR_SPATIAL_COMPONENT_UNSUPPORTED_FOR_CAPABILITY_EXT: return "XR_ERROR_SPATIAL_COMPONENT_UNSUPPORTED_FOR_CAPABILITY_EXT";
			case XrResult::XR_ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT: return "XR_ERROR_SPATIAL_CAPABILITY_CONFIGURATION_INVALID_EXT";
			case XrResult::XR_ERROR_SPATIAL_COMPONENT_NOT_ENABLED_EXT: return "XR_ERROR_SPATIAL_COMPONENT_NOT_ENABLED_EXT";
			case XrResult::XR_ERROR_SPATIAL_PERSISTENCE_SCOPE_UNSUPPORTED_EXT: return "XR_ERROR_SPATIAL_PERSISTENCE_SCOPE_UNSUPPORTED_EXT";
			case XrResult::XR_ERROR_SPATIAL_PERSISTENCE_SCOPE_INCOMPATIBLE_EXT: return "XR_ERROR_SPATIAL_PERSISTENCE_SCOPE_INCOMPATIBLE_EXT";
			case XrResult::XR_RESULT_MAX_ENUM: return "XR_RESULT_MAX_ENUM";
			default: return "Unknown XR Result code";
		}
		#pragma endregion
	}

	// OpenXR API functions. Spec: https://registry.khronos.org/OpenXR/specs/1.1/man/html/openxr.html
	struct Api
	{
		// Global (no instance required) functions
		PFN_xrGetInstanceProcAddr m_xrGetInstanceProcAddr = nullptr;
		PFN_xrEnumerateApiLayerProperties m_xrEnumerateApiLayerProperties = nullptr;
		PFN_xrEnumerateInstanceExtensionProperties m_xrEnumerateInstanceExtensionProperties = nullptr;

		// Instance functions
		PFN_xrCreateInstance m_xrCreateInstance = nullptr;
		PFN_xrDestroyInstance m_xrDestroyInstance = nullptr;
		//typedef XrResult (XRAPI_PTR *PFN_xrGetInstanceProperties)(XrInstance instance, XrInstanceProperties* instanceProperties);
		//typedef XrResult (XRAPI_PTR *PFN_xrPollEvent)(XrInstance instance, XrEventDataBuffer* eventData);
		PFN_xrResultToString m_xrResultToString = nullptr;
		//typedef XrResult (XRAPI_PTR *PFN_xrStructureTypeToString)(XrInstance instance, XrStructureType value, char buffer[XR_MAX_STRUCTURE_NAME_SIZE]);
		PFN_xrGetSystem m_xrGetSystem = nullptr;
		//typedef XrResult (XRAPI_PTR *PFN_xrGetSystemProperties)(XrInstance instance, XrSystemId systemId, XrSystemProperties* properties);
		//typedef XrResult (XRAPI_PTR *PFN_xrEnumerateEnvironmentBlendModes)(XrInstance instance, XrSystemId systemId, XrViewConfigurationType viewConfigurationType, uint32_t environmentBlendModeCapacityInput, uint32_t* environmentBlendModeCountOutput, XrEnvironmentBlendMode* environmentBlendModes);
		PFN_xrCreateSession m_xrCreateSession = nullptr;
		PFN_xrDestroySession m_xrDestroySession = nullptr;
		//typedef XrResult (XRAPI_PTR *PFN_xrEnumerateReferenceSpaces)(XrSession session, uint32_t spaceCapacityInput, uint32_t* spaceCountOutput, XrReferenceSpaceType* spaces);
		//typedef XrResult (XRAPI_PTR *PFN_xrCreateReferenceSpace)(XrSession session, const XrReferenceSpaceCreateInfo* createInfo, XrSpace* space);
		//typedef XrResult (XRAPI_PTR *PFN_xrGetReferenceSpaceBoundsRect)(XrSession session, XrReferenceSpaceType referenceSpaceType, XrExtent2Df* bounds);
		//typedef XrResult (XRAPI_PTR *PFN_xrCreateActionSpace)(XrSession session, const XrActionSpaceCreateInfo* createInfo, XrSpace* space);
		//typedef XrResult (XRAPI_PTR *PFN_xrLocateSpace)(XrSpace space, XrSpace baseSpace, XrTime time, XrSpaceLocation* location);
		//typedef XrResult (XRAPI_PTR *PFN_xrDestroySpace)(XrSpace space);
		PFN_xrEnumerateViewConfigurations m_xrEnumerateViewConfigurations = nullptr;
		//typedef XrResult (XRAPI_PTR *PFN_xrGetViewConfigurationProperties)(XrInstance instance, XrSystemId systemId, XrViewConfigurationType viewConfigurationType, XrViewConfigurationProperties* configurationProperties);
		PFN_xrEnumerateViewConfigurationViews m_xrEnumerateViewConfigurationViews = nullptr;
		//typedef XrResult (XRAPI_PTR *PFN_xrEnumerateSwapchainFormats)(XrSession session, uint32_t formatCapacityInput, uint32_t* formatCountOutput, int64_t* formats);
		PFN_xrCreateSwapchain m_xrCreateSwapchain = nullptr;
		PFN_xrDestroySwapchain m_xrDestroySwapchain = nullptr;
		PFN_xrEnumerateSwapchainImages m_xrEnumerateSwapchainImages = nullptr;
		//typedef XrResult (XRAPI_PTR *PFN_xrAcquireSwapchainImage)(XrSwapchain swapchain, const XrSwapchainImageAcquireInfo* acquireInfo, uint32_t* index);
		//typedef XrResult (XRAPI_PTR *PFN_xrWaitSwapchainImage)(XrSwapchain swapchain, const XrSwapchainImageWaitInfo* waitInfo);
		//typedef XrResult (XRAPI_PTR *PFN_xrReleaseSwapchainImage)(XrSwapchain swapchain, const XrSwapchainImageReleaseInfo* releaseInfo);
		//typedef XrResult (XRAPI_PTR *PFN_xrBeginSession)(XrSession session, const XrSessionBeginInfo* beginInfo);
		//typedef XrResult (XRAPI_PTR *PFN_xrEndSession)(XrSession session);
		//typedef XrResult (XRAPI_PTR *PFN_xrRequestExitSession)(XrSession session);
		PFN_xrWaitFrame m_xrWaitFrame = nullptr;
		PFN_xrBeginFrame m_xrBeginFrame = nullptr;
		//typedef XrResult (XRAPI_PTR *PFN_xrEndFrame)(XrSession session, const XrFrameEndInfo* frameEndInfo);
		//typedef XrResult (XRAPI_PTR *PFN_xrLocateViews)(XrSession session, const XrViewLocateInfo* viewLocateInfo, XrViewState* viewState, uint32_t viewCapacityInput, uint32_t* viewCountOutput, XrView* views);
		//typedef XrResult (XRAPI_PTR *PFN_xrStringToPath)(XrInstance instance, const char* pathString, XrPath* path);
		//typedef XrResult (XRAPI_PTR *PFN_xrPathToString)(XrInstance instance, XrPath path, uint32_t bufferCapacityInput, uint32_t* bufferCountOutput, char* buffer);
		//typedef XrResult (XRAPI_PTR *PFN_xrCreateActionSet)(XrInstance instance, const XrActionSetCreateInfo* createInfo, XrActionSet* actionSet);
		//typedef XrResult (XRAPI_PTR *PFN_xrDestroyActionSet)(XrActionSet actionSet);
		//typedef XrResult (XRAPI_PTR *PFN_xrCreateAction)(XrActionSet actionSet, const XrActionCreateInfo* createInfo, XrAction* action);
		//typedef XrResult (XRAPI_PTR *PFN_xrDestroyAction)(XrAction action);
		//typedef XrResult (XRAPI_PTR *PFN_xrSuggestInteractionProfileBindings)(XrInstance instance, const XrInteractionProfileSuggestedBinding* suggestedBindings);
		//typedef XrResult (XRAPI_PTR *PFN_xrAttachSessionActionSets)(XrSession session, const XrSessionActionSetsAttachInfo* attachInfo);
		//typedef XrResult (XRAPI_PTR *PFN_xrGetCurrentInteractionProfile)(XrSession session, XrPath topLevelUserPath, XrInteractionProfileState* interactionProfile);
		//typedef XrResult (XRAPI_PTR *PFN_xrGetActionStateBoolean)(XrSession session, const XrActionStateGetInfo* getInfo, XrActionStateBoolean* state);
		//typedef XrResult (XRAPI_PTR *PFN_xrGetActionStateFloat)(XrSession session, const XrActionStateGetInfo* getInfo, XrActionStateFloat* state);
		//typedef XrResult (XRAPI_PTR *PFN_xrGetActionStateVector2f)(XrSession session, const XrActionStateGetInfo* getInfo, XrActionStateVector2f* state);
		//typedef XrResult (XRAPI_PTR *PFN_xrGetActionStatePose)(XrSession session, const XrActionStateGetInfo* getInfo, XrActionStatePose* state);
		//typedef XrResult (XRAPI_PTR *PFN_xrSyncActions)(XrSession session, const XrActionsSyncInfo* syncInfo);
		//typedef XrResult (XRAPI_PTR *PFN_xrEnumerateBoundSourcesForAction)(XrSession session, const XrBoundSourcesForActionEnumerateInfo* enumerateInfo, uint32_t sourceCapacityInput, uint32_t* sourceCountOutput, XrPath* sources);
		//typedef XrResult (XRAPI_PTR *PFN_xrGetInputSourceLocalizedName)(XrSession session, const XrInputSourceLocalizedNameGetInfo* getInfo, uint32_t bufferCapacityInput, uint32_t* bufferCountOutput, char* buffer);
		//typedef XrResult (XRAPI_PTR *PFN_xrApplyHapticFeedback)(XrSession session, const XrHapticActionInfo* hapticActionInfo, const XrHapticBaseHeader* hapticFeedback);
		//typedef XrResult (XRAPI_PTR *PFN_xrStopHapticFeedback)(XrSession session, const XrHapticActionInfo* hapticActionInfo);

		// Platform specific
		PFN_xrGetD3D12GraphicsRequirementsKHR m_xrGetD3D12GraphicsRequirementsKHR = nullptr;

		// XR Instance
		XrInstancePtr m_instance;

		Api(HMODULE dll, Config const& config)
			: m_xrGetInstanceProcAddr((PFN_xrGetInstanceProcAddr)GetProcAddress(dll, "xrGetInstanceProcAddr"))
		{
			if (m_xrGetInstanceProcAddr == nullptr)
				throw std::runtime_error("xrGetInstanceProcAddr function not found in openxr_loader.dll");

			// Create global functions (that don't need an instance pointer)
			Check(m_xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrCreateInstance", (PFN_xrVoidFunction*)&m_xrCreateInstance));
			Check(m_xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrEnumerateApiLayerProperties", (PFN_xrVoidFunction*)&m_xrEnumerateApiLayerProperties));
			Check(m_xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrEnumerateInstanceExtensionProperties", (PFN_xrVoidFunction*)&m_xrEnumerateInstanceExtensionProperties));

			CreateInfo info = CreateInfo(config.m_app_name, "Rylogic View3d-12", config.m_app_version)
				.Extension(XR_KHR_D3D12_ENABLE_EXTENSION_NAME)
				.ApiVersion(config.m_xr_version);

			// Create the instance
			XrInstance inst;
			Check(m_xrCreateInstance(&info.build(), &inst));
			Check(m_xrGetInstanceProcAddr(inst, "xrDestroyInstance", (PFN_xrVoidFunction*)&m_xrDestroyInstance));
			m_instance = XrInstancePtr(inst, [release = m_xrDestroyInstance](XrInstance p) { if (p) release(p); });

			// Create instance functions
			Check(m_xrGetInstanceProcAddr(inst, "xrResultToString", (PFN_xrVoidFunction*)&m_xrResultToString));
			Check(m_xrGetInstanceProcAddr(inst, "xrGetSystem", (PFN_xrVoidFunction*)&m_xrGetSystem));
			Check(m_xrGetInstanceProcAddr(inst, "xrCreateSession", (PFN_xrVoidFunction*)&m_xrCreateSession));
			Check(m_xrGetInstanceProcAddr(inst, "xrDestroySession", (PFN_xrVoidFunction*)&m_xrDestroySession));
			Check(m_xrGetInstanceProcAddr(inst, "xrEnumerateViewConfigurations", (PFN_xrVoidFunction*)&m_xrEnumerateViewConfigurations));
			Check(m_xrGetInstanceProcAddr(inst, "xrEnumerateViewConfigurationViews", (PFN_xrVoidFunction*)&m_xrEnumerateViewConfigurationViews));
			Check(m_xrGetInstanceProcAddr(inst, "xrCreateSwapchain", (PFN_xrVoidFunction*)&m_xrCreateSwapchain));
			Check(m_xrGetInstanceProcAddr(inst, "xrDestroySwapchain", (PFN_xrVoidFunction*)&m_xrDestroySwapchain));
			Check(m_xrGetInstanceProcAddr(inst, "xrEnumerateSwapchainImages", (PFN_xrVoidFunction*)&m_xrEnumerateSwapchainImages));
			Check(m_xrGetInstanceProcAddr(inst, "xrWaitFrame", (PFN_xrVoidFunction*)&m_xrWaitFrame));
			Check(m_xrGetInstanceProcAddr(inst, "xrBeginFrame", (PFN_xrVoidFunction*)&m_xrBeginFrame));

			// Platfrom specific functions
			Check(m_xrGetInstanceProcAddr(inst, "xrGetD3D12GraphicsRequirementsKHR", (PFN_xrVoidFunction*)&m_xrGetD3D12GraphicsRequirementsKHR));
		}

		// Access the instance
		operator XrInstance() const
		{
			return m_instance.get();
		}

		// Convert a result into a string
		std::string ToString(XrResult r) const
		{
			if (m_xrResultToString == nullptr)
				return openxr::ToString(r);

			char buffer[XR_MAX_RESULT_STRING_SIZE];
			m_xrResultToString(m_instance.get(), r, buffer);
			return { &buffer[0] };
		}

		// Check an XrResult and throw if it indicates failure
		void Check(XrResult r) const
		{
			if (XR_SUCCEEDED(r)) return;
			throw std::runtime_error(std::format("OpenXR call failed. Error: {} - {}", (int)r, ToString(r)));
		}
	};

	// Open XR implementation
	struct OpenXRImpl : OpenXR
	{
		HMODULE m_dll;
		Api m_api;
		Config m_config;
		XrSystemId m_system_id;
		DeviceRequirementsData m_device_requirements;
		XrSessionPtr m_session;
		XrSwapchainPtr m_swapchain;

		OpenXRImpl(Config const& config)
			: m_dll(win32::LoadDll<OpenXR>("openxr_loader.dll", L".\\lib\\$(platform)"))
			, m_api(m_dll, config)
			, m_config(config)
			, m_system_id()
			, m_device_requirements()
			, m_session()
			, m_swapchain()
		{
			// Get the system ID for the HMD form factor.
			// If this fails, the system probably doesn't have a VR headset connected.
			Check(m_api.m_xrGetSystem(m_api, &SystemGetInfo{}.FormFactor(XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY), &m_system_id));

			// Confirm that the system supports Dx12 binding
			XrGraphicsRequirementsD3D12KHR req = { XR_TYPE_GRAPHICS_REQUIREMENTS_D3D12_KHR };
			Check(m_api.m_xrGetD3D12GraphicsRequirementsKHR(m_api, m_system_id, &req));
			m_device_requirements = DeviceRequirementsData {
				.m_feature_level = req.minFeatureLevel,
				.m_adapter_luid = req.adapterLuid,
			};

			// Enumerate the API layers
			std::vector<XrApiLayerProperties> props;
			{
				uint32_t count = 0;
				Check(m_api.m_xrEnumerateApiLayerProperties(count, &count, props.data()));
				props.resize(count, {XR_TYPE_API_LAYER_PROPERTIES});
				Check(m_api.m_xrEnumerateApiLayerProperties(count, &count, props.data()));
			}

			// Check if the Dx12 extension is available
			std::vector<XrExtensionProperties> exts;
			{
				uint32_t count = 0;
				Check(m_api.m_xrEnumerateInstanceExtensionProperties(nullptr, count, &count, nullptr));
				exts.resize(count, {XR_TYPE_EXTENSION_PROPERTIES});
				Check(m_api.m_xrEnumerateInstanceExtensionProperties(nullptr, count, &count, exts.data()));

				// Scan for the 'XR_KHR_D3D12_enable' extension
				if (!std::ranges::any_of(exts, [](auto const& ext) { return std::strcmp(ext.extensionName, XR_KHR_D3D12_ENABLE_EXTENSION_NAME) == 0; }))
					throw std::runtime_error("The OpenXR runtime does not support the XR_KHR_D3D12_enable extension.");
			}

			auto view_type = To<XrViewConfigurationType>(config.m_view_type);

			// Confirm the view configuration type is available
			std::vector<XrViewConfigurationType> view_types;
			{
				uint32_t count = 0;
				Check(m_api.m_xrEnumerateViewConfigurations(m_api, m_system_id, count, &count, nullptr));
				view_types.resize(count, XrViewConfigurationType{});
				Check(m_api.m_xrEnumerateViewConfigurations(m_api, m_system_id, count, &count, view_types.data()));
			}

			// Enumerate the view configurations supported by the system
			std::vector<XrViewConfigurationView> views;
			{
				uint32_t count = 0;
				Check(m_api.m_xrEnumerateViewConfigurationViews(m_api, m_system_id, view_type, count, &count, nullptr));
				views.resize(count, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
				Check(m_api.m_xrEnumerateViewConfigurationViews(m_api, m_system_id, view_type, count, &count, views.data()));
			}

		}

		// Return the XR device requirements
		DeviceRequirementsData DeviceRequirements() const override
		{
			return m_device_requirements;
		}

		// Get the list of supported view specs
		std::vector<ViewSpec> GetViewSpecs() const override
		{
			std::vector<ViewSpec> result;

			// Enumerate the view configurations supported by the system
			std::vector<XrViewConfigurationType> view_types;
			{
				uint32_t count = 0;
				Check(m_api.m_xrEnumerateViewConfigurations(m_api, m_system_id, count, &count, nullptr));
				view_types.resize(count, XrViewConfigurationType{});
				Check(m_api.m_xrEnumerateViewConfigurations(m_api, m_system_id, count, &count, view_types.data()));
			}

			// For each view configuration type
			for (auto vt : view_types)
			{
				// Enumerate the views for this configuration
				std::vector<XrViewConfigurationView> views;
				{
					uint32_t count = 0;
					Check(m_api.m_xrEnumerateViewConfigurationViews(m_api, m_system_id, vt, count, &count, nullptr));
					views.resize(count, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
					Check(m_api.m_xrEnumerateViewConfigurationViews(m_api, m_system_id, vt, count, &count, views.data()));
				}
				for (auto const& v : views)
				{
					result.push_back(ViewSpec{
						.m_view_type = static_cast<EViewType>(vt),
						.m_image_size_rec = iv2{static_cast<int>(v.recommendedImageRectWidth), static_cast<int>(v.recommendedImageRectHeight)},
						.m_image_size_max = iv2{static_cast<int>(v.maxImageRectWidth), static_cast<int>(v.maxImageRectHeight)},
						.m_samples_rec = static_cast<int>(v.recommendedSwapchainSampleCount),
						.m_samples_max = static_cast<int>(v.maxSwapchainSampleCount),
					});
				}
			}

			return result;
		}

		// Attempt to initialize the OpenXR runtime
		void CreateSession(ViewSpec const& view) override
		{
			// Create the session. This is where the drive does most of it's initialization, load drivers, etc.
			XrSession session;
			GraphicsBindingDx12 dx12 = GraphicsBindingDx12{}.Device(m_config.m_device).CmdQueue(m_config.m_queue);
			SessionCreateInfo info = SessionCreateInfo{}.SystemId(m_system_id).Next(dx12);
			Check(m_api.m_xrCreateSession(m_api, &info, &session));
			m_session = XrSessionPtr(session, [release = m_api.m_xrDestroySession](XrSession p) { if (p) release(p); });

			// Create Swapchains (the VR Render Targets). Can create one swapchain per eye, or a stereo array one.
			XrSwapchainCreateInfo sci{
				.type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
				.createFlags = {},
				.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT,
				.format = DXGI_FORMAT_R8G8B8A8_UNORM, // or one the runtime supports
				.sampleCount = static_cast<uint32_t>(view.m_samples_rec),
				.width = static_cast<uint32_t>(view.m_image_size_rec.x),
				.height = static_cast<uint32_t>(view.m_image_size_rec.y),
				.faceCount = 1,
				.arraySize = 1,
				.mipCount = 1,
			};
			XrSwapchain swapchain;
			Check(m_api.m_xrCreateSwapchain(session, &sci, &swapchain));
			m_swapchain = XrSwapchainPtr(swapchain, [release = m_api.m_xrDestroySwapchain](XrSwapchain p) { if (p) release(p); });

			// Enumerate swapchain images. These are actual ID3D12Resource* to render into.
			std::vector<XrSwapchainImageD3D12KHR> images;
			{
				uint32_t count = 0;
				Check(m_api.m_xrEnumerateSwapchainImages(swapchain, 0, &count, nullptr));
				images.resize(count, { XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR });
				Check(m_api.m_xrEnumerateSwapchainImages(swapchain, count, &count, reinterpret_cast<XrSwapchainImageBaseHeader*>(images.data())));
			}

			#if 0
			{
				7. Begin the Main Loop
					Every frame :

				(a)xrWaitFrame()

					Runtime tells you when to draw for best timing.

					xrWaitFrame(session, nullptr, &frameState);

				(b)xrBeginFrame()
					xrBeginFrame(session, nullptr);

				(c)Locate Views
					This gives poses + projection matrices per eye.

					XrViewLocateInfo locate{ XR_TYPE_VIEW_LOCATE_INFO };
				locate.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
				locate.displayTime = frameState.predictedDisplayTime;
				locate.space = yourAppSpace; // usually LOCAL or STAGE

				XrViewState viewState{ XR_TYPE_VIEW_STATE };

				std::vector<XrView> views(viewCount, { XR_TYPE_VIEW });
				xrLocateViews(session, &locate, &viewState, viewCount, &viewCount, views.data());

				Views contain :
				pose → camera transform

					fov → projection

					(d) Acquire + Render into Swapchain Images

					For each eye :

				uint32_t index;
				xrAcquireSwapchainImage(swapchain, nullptr, &index);
				xrWaitSwapchainImage(swapchain, nullptr);
				RenderEye(index, views[i]);  // YOU render here
				xrReleaseSwapchainImage(swapchain, nullptr);


				You render with the view's pose + projection.

					(e)Submit Layers

					Create a projection layer :

				XrCompositionLayerProjection layer{ XR_TYPE_COMPOSITION_LAYER_PROJECTION };
				layer.space = yourAppSpace;

				std::vector<XrCompositionLayerProjectionView> projViews(viewCount);
				for (int i = 0; i < viewCount; i++) {
					projViews[i].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
					projViews[i].pose = views[i].pose;
					projViews[i].fov = views[i].fov;
					projViews[i].subImage.swapchain = swapchains[i];
					projViews[i].subImage.imageArrayIndex = 0;
					projViews[i].subImage.imageRect = { 0,0,width,height };
				}

				layer.viewCount = viewCount;
				layer.views = projViews.data();


			Submit:

				XrFrameEndInfo end{ XR_TYPE_FRAME_END_INFO };
				end.displayTime = frameState.predictedDisplayTime;
				end.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
				end.layerCount = 1;
				end.layers = (const XrCompositionLayerBaseHeader* const*)&layer;

				xrEndFrame(session, &end);

				8. Handle Resize, Reprojection, Tracking Loss, Runtime Events

					Poll events :

				XrEventDataBuffer event{ XR_TYPE_EVENT_DATA_BUFFER };
				while (xrPollEvent(instance, &event) == XR_SUCCESS) {
					// handle session state changes
				}

				🎯 That’s the entire OpenXR startup& frame loop path

					Once you implement this, your DX12 renderer is fully VR - ready.

					If you want, mi can write you a single self - contained.cpp file that does all the OpenXR init + DX12 binding the way a proper game engine would — dynamic loading, clean layering, zero leaks.

					Just say the word, mi boss.
			}
			#endif
		}

		// Check an XrResult and throw if it indicates failure
		void Check(XrResult r) const
		{
			if (XR_SUCCEEDED(r)) return;
			throw std::runtime_error(std::format("OpenXR call failed. Error: {}", m_api.ToString(r)));
		}
	};

	// Create an OpenXR instance
	std::unique_ptr<OpenXR> CreateInstance(Config const& config)
	{
		return std::unique_ptr<OpenXR>{ new OpenXRImpl(config) };
	}
}