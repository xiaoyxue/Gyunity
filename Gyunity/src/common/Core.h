#pragma once

#include "Platform.h"
#include "Def.h"
#include "fmt/bundled/ostream.h"
#include "fmt/fmt.h"
#include "Logger.h"

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <type_traits>
#include <cstdint>
#include <algorithm>
#include <functional>
#include <memory>
#include <csignal>
#include <vector>
#include <map>
#include <list>

#define ISE_SSE

GYT_NAMESPACE_BEGIN

/***************** Type ******************/
using uchar = unsigned char;

using int8 = int8_t;
using uint8 = uint8_t;

using int16 = int16_t;
using uint16 = uint16_t;

using int32 = int32_t;
using uint32 = uint32_t;

using int64 = int64_t;
using uint64 = uint64_t;

using float32 = float;
using float64 = double;

#ifdef USING_DOUBLE
using real = double;
using real_bit = uint64;
#else
using real = float;
using real_bit = uint32;
#endif

#if defined(GYT_PLATFORM_WINDOWS)
#define GYT_FORCE_INLINE __forceinline
#else
#define GYT_FORCE_INLINE inline __attribute__((always_inline))
#endif

#if defined(GYT_COMPILER_MSVC)
#define GYT_ALIGNED(x) __declspec(align(x))
#endif


/****************** Utils *********************/
/************ Type ************/
namespace type 
{
	class Object;

	template <typename T>
	using remove_cvref =
		typename std::remove_cv<typename std::remove_reference<T>::type>;

	template <typename T>
	using remove_cvref_t = typename remove_cvref<T>::type;

	template <typename T>
	using is_object = typename std::is_base_of<Object, remove_cvref_t<T>>;

	template <typename T>
	using is_object_t = typename is_object<T>::type;
}  // namespace Type


/************ String ************/
inline std::string TrimString(const std::string& s)
{
	int begin = 0, end = (int)s.size();
	while (begin < end && s[begin] == ' ') 
	{
		begin++;
	}
	while (begin < end && s[end - 1] == ' ') 
	{
		end--;
	}
	return std::string(s.begin() + begin, s.begin() + end);
}

inline std::vector<std::string> SplitString(const std::string& s, const std::string& seperators) 
{
	std::vector<std::string> ret;
	bool isSeperator[256] = { false };
	for (auto& ch : seperators) 
	{
		isSeperator[(unsigned int)ch] = true;
	}
	int begin = 0;
	for (int i = 0; i <= (int)s.size(); i++) 
	{
		if (isSeperator[(uint8)s[i]] || i == (int)s.size()) 
		{
			ret.push_back(std::string(s.begin() + begin, s.begin() + i));
			begin = i + 1;
		}
	}
	return ret;
}

inline bool EndsWith(std::string const& str, std::string const& ending) 
{
	if (ending.size() > str.size())
		return false;
	else
		return std::equal(ending.begin(), ending.end(), str.end() - ending.size());
}

inline bool StartsWith(std::string const& str, std::string const& ending) 
{
	if (ending.size() > str.size())
		return false;
	else
		return std::equal(ending.begin(), ending.end(), str.begin());
}


/****************** Logging *********************/
#define GYT_UNREACHABLE __assume(0);

#define GYT_Print(...) fmt::print(__VA_ARGS__)

#define SPD_LOG(X, ...) Gyunity::GetDefaultLogger()->X(fmt::format(__VA_ARGS__))

#define SPD_INFO_LOG(X, ...) Gyunity::GetInfoLogger()->X(fmt::format(__VA_ARGS__))

#define SPD_LOCATED_LOG(X, ...)                                                 \
  Gyunity::GetDefaultLogger()->X(                                               \
      fmt::format("[{}: {} @{}] ", __FILE__, __FUNCTION__, __LINE__) +          \
      fmt::format(__VA_ARGS__))

#define GYT_LOG_PRINT(...) SPD_INFO_LOG(Info, __VA_ARGS__)
#define GYT_SET_LOG_LEVEL(Level) Gyunity::GetDefaultLogger()->SetLevel(Level)
#define GYT_INFO(...) SPD_LOG(Info, __VA_ARGS__)
#define GYT_WARN(...) SPD_LOCATED_LOG(Warn, __VA_ARGS__)
#define GYT_DEBUG(...) SPD_LOCATED_LOG(Debug, __VA_ARGS__)
#define GYT_TRACE(...) SPD_LOCATED_LOG(Trace, __VA_ARGS__)
#define GYT_ERROR(...)                          \
do                                              \
{                                               \
    SPD_LOCATED_LOG(Error, __VA_ARGS__);        \
    exit(2);                                    \
    GYT_UNREACHABLE                             \
} while (0)


#define GYT_STATIC_ASSERT(x) static_assert((x), #x)
#define GYT_STATIC_ASSERT_INFO(x, info) static_assert((x), info)

#define GYT_ASSERT(x) GYT_ASSERT_INFO((x), "Assertion failure: " #x)

#define GYT_ASSERT_INFO(x, ...)             \
do                                          \
{                                           \
    if (!(x))                               \
    {                                       \
        GYT_ERROR(__VA_ARGS__);             \
    }                                       \
} while(0)

/****************** Constant *********************/

constexpr real PI = (real)3.14159265358979;
constexpr real INV_PI = (real)0.31830988618379067154;
constexpr real INV_2PI = (real)0.15915494309189533577;
constexpr real INV_4PI = (real)0.07957747154594766788;
constexpr real PI_Over2 = (real)1.57079632679489661923;
constexpr real PI_Over4 = (real)0.78539816339744830961;
constexpr real Eps = (real)1e-6;
constexpr real Inf = std::numeric_limits<real>::infinity();
constexpr real RayEps = (real)1e-4;
constexpr real ShadowRayEps = (real)1e-4;
constexpr real PhtotonEdgeEps = (real)0.0009;
constexpr real NumericalEps = 1e-6;
constexpr real MachineEps = std::numeric_limits<real>::epsilon() * (real)0.5;
constexpr real MaxReal = std::numeric_limits<real>::max();
constexpr real Infinity = std::numeric_limits<real>::infinity();

GYT_NAMESPACE_END