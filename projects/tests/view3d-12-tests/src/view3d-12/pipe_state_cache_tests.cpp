#include "forward.h"

namespace render_tests
{
	namespace rdr = pr::rdr12;
	namespace gpu = pr::compute;

	// Fail a pipeline cache assertion without relying on an interactive Debug assertion dialog.
	void RequirePipeState(bool condition, char const* message)
	{
		if (!condition)
			throw std::runtime_error(message);
	}

	// A shader whose vertex stage is a runtime copy owned by the shader, like a runtime-compiled shader.
	struct OwnedVertexShader : rdr::Shader
	{
		explicit OwnedVertexShader(rdr::Renderer& rdr)
			: Shader(rdr)
		{
			// Copy the stock bytes so that the stage address is owned by this shader rather than by static data
			auto const& vs = rdr::shader_code::shadow_map_vs;
			auto const* bytes = static_cast<BYTE const*>(vs.pShaderBytecode);
			m_code.VS = OwnByteCode({ bytes, bytes + vs.BytecodeLength });
		}
	};

	// Check that pipelines are shared by final state and released with the shader memory that identifies them.
	void PipeStateCacheTests()
	{
		// All native resources belong to this executable's renderer, never the independently linked DLL.
		auto settings = rdr::RdrSettings(GetModuleHandleW(nullptr)).DefaultAdapter();
		auto renderer = rdr::Renderer(settings);
		auto& cache = renderer.PipeStates();
		auto smap = rdr::shaders::ShadowMap(renderer);

		// A valid depth-only description built from the stock shadow map shader
		auto make_desc = [&](rdr::ShaderCode const& code)
		{
			return rdr::PipeStateDesc(D3D12_GRAPHICS_PIPELINE_STATE_DESC{
				.pRootSignature = smap.m_signature.get(),
				.VS = code.VS,
				.PS = smap.m_code.PS,
				.StreamOutput = gpu::StreamOutputDesc{},
				.BlendState = gpu::BlendStateDesc{},
				.SampleMask = UINT_MAX,
				.RasterizerState = gpu::RasterStateDesc{},
				.DepthStencilState = gpu::DepthStateDesc{},
				.InputLayout = rdr::Vert::LayoutDesc(),
				.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED,
				.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE,
				.NumRenderTargets = 0U,
				.DSVFormat = DXGI_FORMAT_D32_FLOAT,
				.SampleDesc = gpu::MultiSamp{},
			});
		};
		auto const initial = cache.Count();
		auto const base = make_desc(smap.m_code);
		auto const base_pso = cache.Get(base);
		RequirePipeState(cache.Count() == initial + 1, "The first request did not create one pipeline");

		// The same final state reached by different orders of changes, including a change that is later undone, must share one pipeline
		{
			auto a = base;
			a.Apply(rdr::PSO<rdr::EPipeState::CullMode>(D3D12_CULL_MODE_NONE));
			a.Apply(rdr::PSO<rdr::EPipeState::FillMode>(D3D12_FILL_MODE_WIREFRAME));
			auto b = base;
			b.Apply(rdr::PSO<rdr::EPipeState::FillMode>(D3D12_FILL_MODE_WIREFRAME));
			b.Apply(rdr::PSO<rdr::EPipeState::CullMode>(D3D12_CULL_MODE_FRONT));
			b.Apply(rdr::PSO<rdr::EPipeState::CullMode>(D3D12_CULL_MODE_NONE));
			RequirePipeState(a == b && a.Hash() == b.Hash(), "Equal final states compared unequal");
			auto const pso = cache.Get(a);
			RequirePipeState(pso != base_pso, "A different state reused another pipeline");
			RequirePipeState(cache.Get(b) == pso && cache.Count() == initial + 2, "Equal final states created separate pipelines");

			// A state that returns to the base must find the base pipeline
			auto c = a;
			c.Apply(rdr::PSO<rdr::EPipeState::CullMode>(base.Get<rdr::EPipeState::CullMode>()));
			c.Apply(rdr::PSO<rdr::EPipeState::FillMode>(base.Get<rdr::EPipeState::FillMode>()));
			RequirePipeState(cache.Get(c) == base_pso && cache.Count() == initial + 2, "A restored state did not reuse its pipeline");
		}

		// Deleting a shader must forget the pipelines that use its owned bytecode, and only those
		{
			auto owned = std::make_unique<OwnedVertexShader>(renderer);
			auto const desc = make_desc(owned->m_code);
			RequirePipeState(desc.Get<rdr::EPipeState::VS>().pShaderBytecode != smap.m_code.VS.pShaderBytecode, "The test shader does not own its bytecode");
			cache.Get(desc);
			RequirePipeState(cache.Count() == initial + 3, "The owned bytecode did not create its own pipeline");
			owned.reset();
			RequirePipeState(cache.Count() == initial + 2, "Deleting the shader did not forget its pipeline");
			RequirePipeState(cache.Get(base) == base_pso, "Deleting the shader forgot an unrelated pipeline");
		}

		// Prewarmed states must be created by the workers, so drawing them later creates nothing
		{
			// Two new states, one requested twice, and one already cached
			auto a = base;
			a.Apply(rdr::PSO<rdr::EPipeState::CullMode>(D3D12_CULL_MODE_FRONT));
			auto b = base;
			b.Apply(rdr::PSO<rdr::EPipeState::DepthClipEnable>(FALSE));
			auto const before = cache.GetStats();
			rdr::PipeStateDesc const descs[] = { a, b, a, base };
			RequirePipeState(cache.Prewarm(descs) == 2, "Prewarm did not request only the new, distinct states");

			// Wait for the workers to finish
			auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
			for (; cache.GetStats().m_pending != 0; std::this_thread::sleep_for(std::chrono::milliseconds(1)))
				RequirePipeState(std::chrono::steady_clock::now() < deadline, "Prewarm requests did not complete");

			auto const after = cache.GetStats();
			RequirePipeState(after.m_prewarmed == before.m_prewarmed + 2 && after.m_failed == before.m_failed, "The workers did not create the requested states");
			RequirePipeState(cache.Prewarm(descs) == 0, "Prewarm requested states that are already cached");
			cache.Get(a);
			cache.Get(b);
			RequirePipeState(cache.GetStats().m_created == after.m_created, "Drawing a prewarmed state created a new pipeline");
		}

		// Forgetting bytecode must drop its pending requests, so workers never read released bytecode
		{
			// Many variants keep the workers busy while the shader is deleted
			auto owned = std::make_unique<OwnedVertexShader>(renderer);
			std::vector<rdr::PipeStateDesc> descs;
			for (int i = 0; i != 48; ++i)
			{
				auto desc = make_desc(owned->m_code);
				desc.Apply(rdr::PSO<rdr::EPipeState::CullMode>(static_cast<D3D12_CULL_MODE>(D3D12_CULL_MODE_NONE + i % 3)));
				desc.Apply(rdr::PSO<rdr::EPipeState::FillMode>(static_cast<D3D12_FILL_MODE>(D3D12_FILL_MODE_WIREFRAME + (i / 3) % 2)));
				desc.Apply(rdr::PSO<rdr::EPipeState::DepthFunc>(static_cast<D3D12_COMPARISON_FUNC>(D3D12_COMPARISON_FUNC_NEVER + i / 6)));
				descs.push_back(desc);
			}
			auto const count = cache.Count();
			cache.Prewarm(descs);
			owned.reset();
			RequirePipeState(cache.GetStats().m_pending == 0, "Forgetting the bytecode left pending requests");
			RequirePipeState(cache.Count() == count, "Forgetting the bytecode left prewarmed pipelines that use it");
		}

		// Forgetting a root signature releases every pipeline that uses it
		cache.Forget(smap.m_signature.get(), {});
		RequirePipeState(cache.Count() == initial, "Forgetting the signature left pipelines that use it");
		renderer.Poll();
		std::cout << "PASS pipeline sharing by final state, prewarming, and release with owned bytecode and signatures" << std::endl;
	}
}

namespace pr::unittests::view3d12
{
	PRUnitTest(View3d12_PipeStateCache, Extended | Serial)
	{
		render_tests::PipeStateCacheTests();
	}
}
