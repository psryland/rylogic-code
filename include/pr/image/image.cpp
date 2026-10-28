//********************************************************
//
//	ImageManipulator / Image
//
//********************************************************

#include "pr/common/assert.h"
#include "pr/maths/maths.h"
#include <d3d9.h>
#include <d3dx9.h>
#include <d3dx9tex.h>
#include "pr/Image/Image.h"

using namespace pr;
using namespace pr::image;

// ImageInfo methods ********************************************************

//*****
// Constructor
ImageInfo::ImageInfo(const char* filename)
	:ImageInfo()
{
	// Start from the default description, then fill in the image details from the file
	m_filename = filename;
	Verify(D3DXGetImageInfoFromFile(m_filename.c_str(), static_cast<D3DXIMAGE_INFO*>(this)));
}

// image::Context methods ********************************************************

//*****
// Use this if you don't have a d3d device. A d3d interface and device will be created. 
// If you're running from a console app use GetConsoleWindow(). You'll need
// to include windows and define _WIN32_WINNT >= 0x0500 in your project settings
Context Context::make(HWND hwnd)
{
	Context ctx;
	ctx.m_d3d = D3DPtr<IDirect3D9>(Direct3DCreate9(D3D_SDK_VERSION));
	if( !ctx.m_d3d ) { throw Exception(EResult_CreateD3DInterfaceFailed); }
	
	D3DPRESENT_PARAMETERS pp; memset(&pp, 0, sizeof(pp));
	pp.Windowed			= TRUE;
	pp.SwapEffect		= D3DSWAPEFFECT_DISCARD;
	pp.BackBufferFormat = D3DFMT_UNKNOWN;
	if( Failed(ctx.m_d3d->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_REF, hwnd, D3DCREATE_SOFTWARE_VERTEXPROCESSING, &pp, &ctx.m_d3d_device.m_ptr)) )
	{
		throw Exception(EResult_CreateD3DDeviceFailed);
	}
	return ctx;
}

// Lock methods *****************************************************************
		
Lock::~Lock()
{
	if( m_image ) m_image->UnlockRect(m_mip_level);
}

// image functions *****************************************************************

//*****
// Create a blank image
Image2D pr::image::Create2DImage(Context& context, const ImageInfo& image_info)
{
	D3DPtr<IDirect3DTexture9> image;
	if( Failed(D3DXCreateTexture(
		context.m_d3d_device.m_ptr,
		image_info.Width,
		image_info.Height,
		image_info.MipLevels,
		image_info.m_usage,
		image_info.Format,
		image_info.m_pool,
		&image.m_ptr)) )
	{
		throw Exception(EResult_CreateTextureFailed);
	}
	return Image2D(image_info, image);
}

//*****
// Create an image from file
Image2D pr::image::Load2DImage(Context& context, const ImageInfo& image_info)
{
	// Load the image
	ImageInfo info;
	D3DPtr<IDirect3DTexture9> image;
	if( Failed(D3DXCreateTextureFromFileEx(
		context.m_d3d_device.m_ptr,
		image_info.m_filename.c_str(),
		image_info.Width,
		image_info.Height,
		image_info.MipLevels,
		image_info.m_usage,
		image_info.Format,
		image_info.m_pool,
		image_info.m_filter,
		image_info.m_mip_filter,
		image_info.m_color_key,
		&info,
		info.m_palette,
		&image.m_ptr)) )
	{
		throw Exception(EResult_CreateTextureFailed);
	}
	return Image2D(info, image);
}

//*****
// Save an image to file
bool pr::image::Save2DImage(const Image2D& image)
{
	PR_ASSERT(PR_DBG_IMAGE, image.m_info.ImageFileFormat != D3DXIFF_TGA, "Can't save as tga format sorry");

	HRESULT result = D3DXSaveTextureToFile(
		image.m_info.m_filename.c_str(),
		image.m_info.ImageFileFormat,
		image.m_image.m_ptr,
		image.m_info.m_palette);

	if( Failed(result) )
	{
		PR_INFO(PR_DBG_IMAGE, "Failed to save image to file");
		return false;
	}
	return true;
}

