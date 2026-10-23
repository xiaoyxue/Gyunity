#pragma once
#include "common/Core.h"
#include "math/Linalg.h"
#include <map>
#include <sstream>

GYT_NAMESPACE_BEGIN

class Config
{
private:
	std::map<std::string, std::string> mData;
public:
	Config() = default;

	static bool IsIntegral(const std::string& str)
	{
		if (str.empty() || ((!isdigit(str[0])) && (str[0] != '-') && (str[0] != '+'))) 
			return false;
		char* p;
		strtol(str.c_str(), &p, 10);
		return (*p == 0);
	}

	template<typename T>
	static std::string GetPtrString(T* ptr)
	{
		std::stringstream ss;
		ss << typeid(T).name() << "\t" << reinterpret_cast<uint64>(ptr);
		return ss.str();
	}

	std::string GetString(const std::string& key) const
	{
		if (mData.find(key) == mData.end())
			GYT_ERROR("Key named '{}' not found.", key);

		return mData.find(key)->second;
	}

	template<typename T>
	T* GetPtr(const std::string& key) const 
	{
		std::string value = GetString(key);
		std::stringstream ss(value);
		std::string t;
		uint64 ptrUint64;
		std::getline(ss, t, '\t');
		GYT_ASSERT_INFO(t == typeid(T).name(),
			"Pointer type mismatch, expect: \"" + std::string(typeid(T).name()) + "\"" + " but actual: \"" + t + "\"");
		ss >> ptrUint64;
		return reinterpret_cast<T*>(ptrUint64);
	}

	template<typename T>
	typename std::enable_if_t<(!(type::is_Vector<T>::value) && !(std::is_reference<T>::value) && !(std::is_pointer<T>::value)), T>
	Get(const std::string& key) const;

	template<typename T>
	std::enable_if_t<std::is_pointer<T>::value, T>
	Get(const std::string& key) const 
	{
		return GetPtr<std::remove_pointer_t<T>>(key);
	}

	template<typename T, typename std::enable_if_t<(type::is_Vector<T>::value), T>* = nullptr>
	T Get(const std::string& key) const
	{
		constexpr int N = T::dim;
		using ElementType = typename T::type;

		std::string str = this->GetString(key);
		std::string pattern;
		if (str[0] == '(')
		{
			pattern = "(";
		}
		else if (str[0] == '[')
		{
			pattern = "[";
		}
		for (int i = 0; i < N; ++i)
		{
			std::string placeHolder;
			if (std::is_same<ElementType, float32>::value)
			{
				placeHolder = "%f";
			}
			else if (std::is_same<ElementType, float64>::value)
			{
				placeHolder = "%lf";
			}
			else if (std::is_same<ElementType, int32>::value)
			{
				placeHolder = "%d";
			}
			else if (std::is_same<ElementType, uint32>::value)
			{
				placeHolder = "%u";
			}
			else if (std::is_same<ElementType, int64>::value)
			{
				placeHolder = "%lld";
			}
			else if (std::is_same<ElementType, uint64>::value)
			{
				placeHolder = "%llu";
			}
			else
			{
				GYT_ASSERT(false);
			}
			pattern += placeHolder;
			if (i != N - 1)
			{
				pattern += ",";
			}
		}
		if (str[0] == '(')
		{
			pattern += ")";
		}
		else if (str[0] == '[')
		{
			pattern += "]";
		}
		Vector<N, ElementType> value;
		if (N == 1)
		{
			sscanf(str.c_str(), pattern.c_str(), &value[0]);
		}
		else if (N == 2)
		{
			sscanf(str.c_str(), pattern.c_str(), &value[0], &value[1]);
		}
		else if (N == 3)
		{
			sscanf(str.c_str(), pattern.c_str(), &value[0], &value[1], &value[2]);
		}
		else if (N == 4)
		{
			sscanf(str.c_str(), pattern.c_str(), &value[0], &value[1], &value[2], &value[3]);
		}
		return value;
	}

	template<typename T, typename std::enable_if_t<((!type::is_Vector<T>::value) && (!std::is_pointer<T>::value)), int> = 0>
	Config& Set(const std::string& name, T value)
	{
		std::stringstream ss;
		ss << value;
		mData[name] = ss.str();
		return *this;
	}

	Config& Set(const std::string& name, const char* value)
	{
		std::stringstream ss;
		ss << value;
		mData[name] = ss.str();
		return *this;
	}

	template<typename T, typename std::enable_if_t<type::is_Vector<T>::value, int> = 0>
	Config& Set(const std::string& name, const T& value)
	{
		std::stringstream ss;
		int N = value.dim;
		ss << "(";
		for (int i = 0; i < N; ++i)
		{
			if (i != N - 1)
				ss << value[i] << ",";
			else
				ss << value[i];
		}
		ss << ")";
		mData[name] = ss.str();
		return *this;
	}

	template<typename T>
	Config& Set(const std::string& name, T* const ptr)
	{
		mData[name] = GetPtrString(ptr);
		return *this;
	}

	template<typename T, typename std::enable_if_t<std::is_pointer<T>::value, T> = nullptr>
	Config& Set(const std::string& name, T ptr)
	{
		mData[name] = GetPtrString(ptr);
		return *this;
	}

private:
	void CheckValueIntegral(const std::string& str) const
	{
		if (!IsIntegral(str))
		{
			GYT_ERROR("The '{}' is not an integar value.", str);
		}
	}

};

template<>
inline std::string Config::Get<std::string>(const std::string& key) const
{
	return GetString(key);
}

template<>
inline float32 Config::Get<float32>(const std::string& key) const
{
	return (float32)std::atof(GetString(key).c_str());
}

template<>
inline float64 Config::Get<float64>(const std::string& key) const
{
	return (float64)std::atof(GetString(key).c_str());
}

template<>
inline int32 Config::Get<int32>(const std::string& key) const
{
	CheckValueIntegral(GetString(key));
	return std::stoi(GetString(key));
}

template<>
inline uint32 Config::Get<uint32>(const std::string& key) const
{
	CheckValueIntegral(GetString(key));
	return uint32(std::stoll(GetString(key)));
}

template<>
inline int64 Config::Get<int64>(const std::string& key) const
{
	CheckValueIntegral(GetString(key));
	return std::stoll(GetString(key));
}

template<>
inline uint64 Config::Get<uint64>(const std::string& key) const
{
	CheckValueIntegral(GetString(key));
	return std::stoull(GetString(key));
}

template<>
inline bool Config::Get<bool>(const std::string& key) const
{
	std::string str = GetString(key);
	static std::map<std::string, bool> dict
	{
		{"true", true}, {"True", true}, {"t", true}, {"T", true}, {"1", true},
		{"false", false}, {"False", false}, {"f", false}, {"F", false}, {"0", false}
	};
	if (dict.find(str) == dict.end())
	{
		GYT_ERROR("Token is not recognized.");
		ASSERT(false);
	}
	return dict.find(str)->second;

}

GYT_NAMESPACE_END