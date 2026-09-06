//*****************************************
//*****************************************
#include "test.h"
#include "pr/common/fmt.h"
#include "pr/image/image.h"
#include "pr/maths/perlinnoise.h"
#include <windows.h>

namespace TestImage
{
	using namespace pr;
	using namespace pr::image;

	void Run()
	{
		Context context = Context::make(GetConsoleWindow());

		uint SIZE = 1000;

		ImageInfo img_info;
		img_info.Width	= SIZE;
		img_info.Height	= SIZE;
		img_info.Format	= D3DFMT_L16;//D3DFMT_A8R8G8B8;//
		img_info.ImageFileFormat = D3DXIFF_PNG;//D3DXIFF_BMP;
		img_info.m_filename = "D:/DeleteMe/DeleteMe.png";
		
		Image2D img = Create2DImage(context, img_info);

		{Lock lock;
			Image2D::iterator iter = img.Lock(lock);
			if( iter )
			{
				for( uint i = 0; i < SIZE; ++i )
				{
					for( uint j = 0; j < SIZE; ++j )
					{
						iter(i,j) = (j*64)%65536;
					}
				}
			}
		}
		Save2DImage(img);
		

		//PerlinNoiseGenerator Perlin(42);
		//float freq = 10.0f;
		//float amp = 0.5f;
		//float offset = 0.5f;

		//Lock lock;
		//Image2D::iterator iter = img.Lock(lock);
		//if( iter )
		//{
		//	for( uint i = 0; i < SIZE; ++i )
		//	{
		//		for( uint j = 0; j < SIZE; ++j )
		//		{
		//			//*iter = D3DColour(j / 100.0f, 0.0f, 0.0f, 1.0f);
		//			//++iter;
		//			//iter(i,j) = pr::Colour32::make(j / 100.0f, 0.0f, 0.0f, 1.0f);
		//			float x = 2.0f * i / (float)SIZE; //(i - SIZE * 0.5f) / (SIZE * 0.5f);
		//			float y = 2.0f * j / (float)SIZE; //(j - SIZE * 0.5f) / (SIZE * 0.5f);
		//			float noise = Perlin.Noise(x * freq, y * freq, 0.0f) * amp + offset;
		//			iter(i,j) = Colour32::make(noise, noise, noise, 1.0f);
		//		}
		//	}
		//}
		//Save2DImage(img);
		



		//for( uint i = 1; i <= 10; ++i )
		//{
		//	std::string filename = Fmt("C:\\Transfer\\WaterNormalMapTest\\NA21C_%0.3d.dds", i);
		//	std::string out_filename = Fmt("C:\\Transfer\\WaterNormalMapTest\\NA21C_%0.3d.bmp", i);
		//				
		//	ImageInfo img_info;
		//	img_info.ImageFileFormat = D3DXIFF_DDS;
		//	img_info.m_filename = filename;
		//	
		//	Image2D img;
		//	img_man.Load(img_info, img);

		//	img.m_info.ImageFileFormat = D3DXIFF_BMP;
		//	img.m_info.m_filename = out_filename;
		//	img_man.Save(img);
		//}

		//ImageInfo img_info;
		//img_info.Width	= 100;
		//img_info.Height	= 100;
		//img_info.Format	= D3DFMT_A8R8G8B8;
		//img_info.ImageFileFormat = D3DXIFF_BMP;
		//img_info.m_filename = "C:/DeleteMe.bmp";
		//
		//Image2D img;
		//img_man.Create(img_info, img);

		//Image2D::Iter iter = img.Lock();
		//if( iter )
		//{
		//	for( uint i = 0; i < 100; ++i )
		//	{
		//		for( uint j = 0; j < 100; ++j )
		//		{
		//			//*iter = D3DColour(j / 100.0f, 0.0f, 0.0f, 1.0f);
		//			//++iter;
		//			iter(i,j) = D3DColour(j / 100.0f, 0.0f, 0.0f, 1.0f);
		//		}
		//	}
		//	img.UnLock();
		//}
		//img_man.Save(img);
	}
}//namespace TestImage
