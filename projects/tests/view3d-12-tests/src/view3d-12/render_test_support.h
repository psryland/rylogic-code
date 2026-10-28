#pragma once
#include "forward.h"

// Shared fixture and pixel helpers for the view3d-12 renderer tests.
// Register each area's tests in its own source file. CPU-only tests are 'Quick'; tests that render and read back on the GPU are 'Extended | Serial'.
// GPU tests are 'Serial' because View3D binds its dll context to the thread that first initialises it, so fixtures on other test threads cannot share it.
// Tests with an MSAA-dependent path run both single-sampled and 4x multisampled.
namespace render_tests
{
	namespace api = pr::view3d;
	namespace ui = pr::view3d::ui;
	using Microsoft::WRL::ComPtr;
	constexpr int ImageSize = 128;

	// Fail the fixture without relying on debug-only assertions.
	void Require(bool condition, char const* message);

	// Reject a failed D3D operation at its ownership boundary.
	void Check(HRESULT result);

	// The main scene light paired with the scene-wide ambient colour, so tests can snapshot and restore both together.
	struct SceneLighting : api::Light
	{
		api::Colour m_ambient;
	};

	// Read light 0 and the scene ambient light from 'window'.
	SceneLighting SceneLightingGet(api::Window window);

	// Write light 0 and the scene ambient light to 'window'.
	void SceneLightingSet(api::Window window, SceneLighting const& lighting);

	// Collect DLL failures without throwing through native callback frames.
	void __stdcall ReportError(void* context, char const* message, char const*, int, int64_t);

	// Convert framebuffer bytes back to the linear colour space used by alpha blending.
	float Linear(unsigned char value);

	// Count differing RGB bytes so GPU channel tests do not depend on one hand-picked lattice value.
	size_t ImageDifference(std::vector<unsigned char> const& lhs, std::vector<unsigned char> const& rhs);

	// Assert compositing against linear source-over expectations, allowing RGBA8 quantization.
	void Expect(std::vector<unsigned char> const& image, float red, float green, float blue, int x = 64, int y = 64);

	// Own a bounded invisible window, its renderer context, and all fixture geometry.
	struct Fixture
	{
		std::vector<std::string> m_errors;
		HWND m_hwnd = nullptr;
		api::DllHandle m_context = nullptr;
		api::Window m_window = nullptr;
		std::vector<api::Object> m_objects;
		std::vector<api::Shader> m_shaders;
		ComPtr<ID3D12Device> m_device;
		ComPtr<ID3D12InfoQueue> m_info;

		// Set up a deterministic orthographic camera and the existing alpha/MSAA configuration.
		explicit Fixture(int samples);

		// Release fixture-owned resources only, without touching other processes or applications.
		~Fixture();

		// Surface unexpected DLL errors immediately rather than accepting a stale frame.
		void CheckErrors();

		// Add an unlit two-sided quad at a chosen forward depth.
		api::Object Quad(float depth, unsigned colour, float half_width = 45, api::Shader shader = nullptr, float right_depth = 0, unsigned vertex_colour = 0xFFFFFFFF, bool textured = false, bool normals = false);

		// Add one independently owned rectangular mesh for cross-object coordinate continuity tests.
		api::Object QuadRect(float x0, float x1, float depth);

		// Replace the active scene without changing ownership of any created object.
		void Clear();

		// Attach a fixed retained-overlay host marker for pixel-invariance checks.
		void AttachOverlay();

		// Reuse an owned overlay wrapper while supplying independent fixture vertex or pixel bytecode.
		api::Shader CustomShader(bool vertex_only);

		// Read the completed final image on a separate queue after synchronizing the renderer.
		std::vector<unsigned char> Image();

		// Reject GPU debug-layer errors after all bounded render cases have completed.
		void CheckDebugLayer();
	};

	// Immutable recipe used by all three public procedural vertex-stage contracts.
	struct alignas(16) ProceduralVertexConstants
	{
		api::Vec4 m_positions[3];
		api::Vec4 m_colour;
		api::Vec4 m_normal = {0, 0, 1, 0};
		std::array<unsigned char, api::ProceduralBinding::ConstantsSize - 5 * sizeof(api::Vec4)> m_padding;
	};
	static_assert(sizeof(ProceduralVertexConstants) == api::ProceduralBinding::ConstantsSize);
	static_assert(sizeof(api::ProceduralBinding) == 96);
	static_assert(offsetof(api::ProceduralBinding, m_forward_pixel_model) == 40);
	static_assert(sizeof(api::ShaderOptions) == 136);
	static_assert(offsetof(api::ShaderOptions, m_stage) == 8);
	static_assert(offsetof(api::ShaderOptions, m_bytecode) == 16);
	static_assert(offsetof(api::ShaderOptions, m_procedural) == 40);
}
