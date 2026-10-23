#include "common/Config.h"
#include "gtest/gtest.h"
#include "math/Linalg.h"


GYT_NAMESPACE_BEGIN

TEST(TestConfig, TestConfig) {
	Config config;

	std::string stringVal = "string";
	int32 int32Val = 32;
	int64 int64Val = 64;
	float32 float32Val = 32.0f;
	float64 float64Val = 64.0;
	Vec3 vec3Val = Vec3(1, 2, 3);

	std::string* ptrString = &stringVal;
	int32* ptrInt32 = &int32Val;
	int64* ptrInt64 = &int64Val;
	float32* ptrFloat32 = &float32Val;
	float64* ptrFloat64 = &float64Val;
	Vec3* ptrVec3 = &vec3Val;

	config.Set<std::string>("stringVal", stringVal);
	config.Set<int32>("int32Val", int32Val);
	config.Set<int64>("int64Val", int64Val);
	config.Set<float32>("float32Val", float32Val);
	config.Set<float64>("float64Val", float64Val);
	config.Set<Vec3>("vec3Val", vec3Val);

	config.Set<std::string>("stringPtr1", ptrString);
	config.Set<std::string*>("stringPtr2", ptrString);
	config.Set<int32>("int32Ptr1", ptrInt32);
	config.Set<int32*>("int32Ptr2", ptrInt32);
	config.Set<int64>("int64Ptr1", ptrInt64);
	config.Set<int64*>("int64Ptr2", ptrInt64);
	config.Set<float32>("float32Ptr1", ptrFloat32);
	config.Set<float32*>("float32Ptr2", ptrFloat32);
	config.Set<float64>("float64Ptr1", ptrFloat64);
	config.Set<float64*>("float64Ptr2", ptrFloat64);
	config.Set<Vec3>("vec3Ptr1", ptrVec3);
	config.Set<Vec3*>("vec3Ptr2", ptrVec3);

	//Test value
	EXPECT_EQ(config.Get<std::string>("stringVal"), "string");
	EXPECT_EQ(config.Get<int32>("int32Val"), 32);
	EXPECT_EQ(config.Get<int64>("int64Val"), 64);
	EXPECT_EQ(config.Get<float32>("float32Val"), 32.f);
	EXPECT_EQ(config.Get<float64>("float64Val"), 64.0);
	EXPECT_EQ(config.Get<Vec3>("vec3Val") == Vec3(1, 2, 3), true);

	//Test ptr1
	EXPECT_EQ(*(config.Get<std::string*>("stringPtr1")), "string");
	EXPECT_EQ(*(config.Get<int32*>("int32Ptr1")), 32);
	EXPECT_EQ(*(config.Get<int64*>("int64Ptr1")), 64);
	EXPECT_EQ(*(config.Get<float32*>("float32Ptr1")), 32.f);
	EXPECT_EQ(*(config.Get<float64*>("float64Ptr1")), 64.0);
	EXPECT_EQ(*(config.Get<Vec3*>("vec3Ptr1")) == Vec3(1, 2, 3), true);

	//Test ptr2
	EXPECT_EQ(*(config.Get<std::string*>("stringPtr2")), "string");
	EXPECT_EQ(*(config.Get<int32*>("int32Ptr2")), 32);
	EXPECT_EQ(*(config.Get<int64*>("int64Ptr2")), 64);
	EXPECT_EQ(*(config.Get<float32*>("float32Ptr2")), 32.f);
	EXPECT_EQ(*(config.Get<float64*>("float64Ptr2")), 64.0);
	EXPECT_EQ(*(config.Get<Vec3*>("vec3Ptr2")) == Vec3(1, 2, 3), true);
};

GYT_NAMESPACE_END