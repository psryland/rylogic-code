//***********************************************************************
// UnitTest
//  Copyright (c) Rylogic Ltd 2008
//***********************************************************************
/* Get yer boilerplate here...
#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::unittests::your::namespace
{
	PRUnitTest(TestName)
	{
		using namespace pr;
		PR_CHECK(1+1, 2);
	}
}
#endif
*/

// Notes:
//  - For faster build times, comment out the "all-headers" include and just include the header you care about
//  - Avoid including extra stuff in the #if PR_UNITTESTS blocks. For standard headers add them to 'unittests.h'
//    For pr headers, consider putting the unit test in a different file to avoid additional dependencies

#include "pr/common/unittests.h"
#include "src/unittests.h" // all tests
//#include "pr/common/flags_enum.h" // This is often needed for single files
//#include "pr/common/allocator.h"
//#include "pr/common/bit_data.h"
//#include "pr/common/cancel_token.h"
//#include "pr/common/compress.h"
//#include "pr/common/coroutine.h"
//#include "pr/common/event_handler.h"
//#include "pr/common/expr_eval.h"
//#include "pr/common/guid.h"
//#include "pr/common/hash.h"
//#include "pr/common/log.h"
//#include "pr/common/memstream.h"
//#include "pr/common/tweakables.h"
//#include "pr/container/bit_array.h"
//#include "pr/container/byte_data.h"
//#include "pr/container/deque.h"
//#include "pr/container/dimension_index.h"
//#include "pr/container/kdtree.h"
//#include "pr/container/ring_buffer.h"
//#include "pr/container/suffix_array.h"
//#include "pr/container/tri_table.h"
//#include "pr/container/vector.h"
//#include "pr/macros/enum.h"
//#include "pr/maths/interpolate.h"
//#include "pr/math/math.h"
//#include "pr/maths/matrix6x8.h"
//#include "pr/maths/matrix4x4.h"
//#include "pr/maths/matrix3x4.h"
//#include "pr/maths/matrix2x2.h"
//#include "pr/maths/quaternion.h"
//#include "pr/maths/vector8.h"
//#include "pr/maths/vector4.h"
//#include "pr/maths/vector3.h"
//#include "pr/maths/vector2.h"
//#include "pr/maths/vector2i.h"
//#include "pr/maths/vector3i.h"
//#include "pr/maths/vector4i.h"
//#include "pr/maths/maths_core.h"
//#include "pr/maths/spline.h"
//#include "pr/maths/matrix.h"
//#include "pr/maths/large_int.h"
//#include "pr/maths/spatial.h"
//#include "pr/maths/frustum.h"
//#include "pr/maths/half.h"
//#include "pr/maths/conversion.h"
//#include "pr/gfx/colour.h"
//#include "pr/gfx/onebit.h"
//#include "pr/geometry/intersect.h"
//#include "pr/geometry/point.h"
//#include "pr/geometry/p3d.h"
//#include "pr/hardware/find_bt_device.h"
//#include "pr/filesys/filesys.h"
//#include "pr/storage/json.h"
//#include "pr/str/convert_utf.h"
//#include "pr/str/string_core.h"
//#include "pr/str/string_util.h"
//#include "pr/str/extract.h"
//#include "pr/str/string_filter.h"
//#include "pr/str/string.h"
//#include "pr/str/string2.h"
//#include "pr/str/to_string.h"
//#include "pr/str/utf8.h"
//#include "pr/script/buf.h"
//#include "pr/script/location.h"
//#include "pr/script/script_core.h"
//#include "pr/script/filter.h"
//#include "pr/script/token.h"
//#include "pr/script/tokeniser.h"
//#include "pr/script/macros.h"
//#include "pr/script/includes.h"
//#include "pr/script/embedded_lua.h"
//#include "pr/script/src_stack.h"
//#include "pr/script/preprocessor.h"
//#include "pr/script/reader.h"
//#include "pr/script/byte_reader.h"
//#include "pr/script/unit_tests.h"
//#include "pr/threads/thread_pool.h"
//#include "pr/network/pipe.h"
//#include "pr/network/pipe2.h"
//#include "pr/physics2/rigid_body/rigid_body.h"
//#include "pr/view3d-12/ldraw/ldraw_serialiser_binary.h"
//#include "pr/view3d-12/ldraw/ldraw_serialiser_text.h"

// Export a function for executing the tests
extern "C"
{
	__declspec(dllexport) int __stdcall RunAllTests(int wordy)
	{
		return pr::unittests::RunAllTests(wordy != 0);
	}
}
int main(int argc, char* argv[])
{
	auto wordy = argc > 1 && strcmp(argv[1], "-verbose") == 0;
	return pr::unittests::RunAllTests(wordy);
}
