#pragma once
#include "def.h"
#include "scene.h"
#include "sphere.h"
#include "diffuse.h"
#include "mirror.h"
#include "glass.h"
#include "arealight.h"
#include "roughness.h"
#include "constant_texture.h"
#include "triangle.h"

NAMESPACE_BEGIN

class CornellBoxSphere {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .75f, .75f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));


		//Left
		std::shared_ptr<Shape> leftWallShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 1e5f, Vec3(1e5f + 1, 40.8f, 81.6f), true));
		std::shared_ptr<Material> leftWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(redConstant));
		std::shared_ptr<Primitive> leftWall = std::shared_ptr<Primitive>(new GeometryPrimitive(leftWallShape, leftWallMaterial));
		scene->AddPrimitive(leftWall);

		//Right
		std::shared_ptr<Shape> rightWallShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 1e5f, Vec3(-1e5f + 99, 40.8f, 81.6f), true));
		std::shared_ptr<Material> rightWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		std::shared_ptr<Primitive> rightWall = std::shared_ptr<Primitive>(new GeometryPrimitive(rightWallShape, rightWallMaterial));
		scene->AddPrimitive(rightWall);

		//Back
		std::shared_ptr<Shape> backWallShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 1e5f, Vec3(50, 40.8f, 1e5), true));
		std::shared_ptr<Material> backWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> backWall = std::shared_ptr<Primitive>(new GeometryPrimitive(backWallShape, backWallMaterial));
		scene->AddPrimitive(backWall);

		//Botom
		std::shared_ptr<Shape> botomShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 1e5f, Vec3(50, 1e5, 81.6f), true));
		std::shared_ptr<Material> botomMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botom = std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape, botomMaterial));
		scene->AddPrimitive(botom);

		//Top
		std::shared_ptr<Shape> topShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr,1e5f, Vec3(50, -1e5f + 81.6f, 81.6f), true));
		std::shared_ptr<Material> topMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> top = std::shared_ptr<Primitive>(new GeometryPrimitive(topShape, topMaterial));
		scene->AddPrimitive(top);

		//Diffuse Ball1
		std::shared_ptr<Shape> diffuseBallShape1 = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 9.5f, Vec3(23, 0.0f, 98)));
		std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(fullWhiteConstant));
		std::shared_ptr<Primitive> diffuseBall1 = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape1, diffuseBallMaterial));
		scene->AddPrimitive(diffuseBall1);

		//Glass Ball1
		std::shared_ptr<Shape> glassBallShape1 = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 16.5f, Vec3(73, 26.5f, 78)));
		std::shared_ptr<Material> glassBallMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Primitive> glassBall1 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape1, glassBallMaterial));
		scene->AddPrimitive(glassBall1);

		//Diffuse Ball2 new Sphere(7.0, Vec3(27, 16.5, 47), Vec3(), Vec3(.25, .25, .75)
		//std::shared_ptr<Shape> diffuseBallShape2 = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 7.0, Vec3(27, 16.5, 47)));
		//std::shared_ptr<Material> diffuseBallMaterial2 = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		//std::shared_ptr<Primitive> diffuseBall2 = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape2, diffuseBallMaterial2));
		//scene->AddPrimitive(diffuseBall2);


		//Glass Ball2 new Sphere(16.5f, Vec3(27, 16.5f, 47)
		std::shared_ptr<Shape> glassBallShape2 = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 16.5f, Vec3(27, 16.5f, 47)));
		std::shared_ptr<Primitive> glassBall2 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape2, glassBallMaterial));
		scene->AddPrimitive(glassBall2);

		//Glass Ball3 new Sphere(9.5f, Vec3(53, 9.5f, 88)
		std::shared_ptr<Shape> glassBallShape3 = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 9.5f, Vec3(53, 9.5f, 88)));
		std::shared_ptr<Primitive> glassBall3 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape3, glassBallMaterial));
		scene->AddPrimitive(glassBall3);

		//Light
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 8.f, Vec3(50, 81.6f - 16.5f, 81.6f)));//Lite
		std::shared_ptr<Light> light0 = std::shared_ptr<Light>(new AreaLight(lightShape, Vec3(0.3f, 0.3f, 0.3f) * 100));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> lightPrimitive = std::shared_ptr<Primitive>(new GeometryPrimitive(lightShape, lightMaterial, light0));
		scene->AddLight(lightPrimitive);

	}
};


class CornellBoxTriangle {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .75f, .75f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));


		//Left
		std::shared_ptr<Shape> leftWallShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 1e5f, Vec3(1e5f + 1, 40.8f, 81.6f), true));
		std::shared_ptr<Material> leftWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(redConstant));
		std::shared_ptr<Primitive> leftWall = std::shared_ptr<Primitive>(new GeometryPrimitive(leftWallShape, leftWallMaterial));
		scene->AddPrimitive(leftWall);

		//Right
		std::shared_ptr<Shape> rightWallShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 1e5f, Vec3(-1e5f + 99, 40.8f, 81.6f), true));
		std::shared_ptr<Material> rightWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		std::shared_ptr<Primitive> rightWall = std::shared_ptr<Primitive>(new GeometryPrimitive(rightWallShape, rightWallMaterial));
		scene->AddPrimitive(rightWall);

		//Back
		std::shared_ptr<Shape> backWallShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 1e5f, Vec3(50, 40.8f, 1e5), true));
		std::shared_ptr<Material> backWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> backWall = std::shared_ptr<Primitive>(new GeometryPrimitive(backWallShape, backWallMaterial));
		scene->AddPrimitive(backWall);

		//Botom
		std::shared_ptr<Shape> botomShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 1e5f, Vec3(50, 1e5, 81.6f), true));
		std::shared_ptr<Material> botomMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botom = std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape, botomMaterial));
		scene->AddPrimitive(botom);

		//Top
		std::shared_ptr<Shape> topShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 1e5f, Vec3(50, -1e5f + 81.6f, 81.6f), true));
		std::shared_ptr<Material> topMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> top = std::shared_ptr<Primitive>(new GeometryPrimitive(topShape, topMaterial));
		scene->AddPrimitive(top);

		//Triangle face
		Transform transformTriangle = Transform::Translate(Vec3(27, 16.5, 47)) * Transform::RotateY(30);
		Vec3 p0 = transformTriangle(Vec3(10, 20, 0));
		Vec3 p1 = transformTriangle(Vec3(-10, 0, 0));
		Vec3 p2 = transformTriangle(Vec3(10, 0, 0));
		Vec3 faceNormal = Cross(p1 - p0, p2 - p0).Norm();
		std::shared_ptr<Shape> triangleShape =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p0, p1, p2, faceNormal));
		std::shared_ptr<Material> triangleMaterial = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		std::shared_ptr<Primitive> triangle = std::shared_ptr<Primitive>(new GeometryPrimitive(triangleShape, triangleMaterial));
		scene->AddPrimitive(triangle);


		//Diffuse Ball1
		std::shared_ptr<Shape> diffuseBallShape1 = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 9.5f, Vec3(23, 0.0f, 98)));
		std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(fullWhiteConstant));
		std::shared_ptr<Primitive> diffuseBall1 = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape1, diffuseBallMaterial));
		scene->AddPrimitive(diffuseBall1);

		////Glass Ball1
		//std::shared_ptr<Shape> glassBallShape1 = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 16.5f, Vec3(73, 26.5f, 78)));
		//std::shared_ptr<Material> glassBallMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		//std::shared_ptr<Primitive> glassBall1 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape1, glassBallMaterial));
		//scene->AddPrimitive(glassBall1);

		//Diffuse Ball2 new Sphere(7.0, Vec3(27, 16.5, 47), Vec3(), Vec3(.25, .25, .75)
		//std::shared_ptr<Shape> diffuseBallShape2 = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 7.0, Vec3(27, 16.5, 47)));
		//std::shared_ptr<Material> diffuseBallMaterial2 = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		//std::shared_ptr<Primitive> diffuseBall2 = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape2, diffuseBallMaterial2));
		//scene->AddPrimitive(diffuseBall2);


		//Glass Ball2 new Sphere(16.5f, Vec3(27, 16.5f, 47)
		//std::shared_ptr<Shape> glassBallShape2 = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 16.5f, Vec3(27, 16.5f, 47)));
		//std::shared_ptr<Primitive> glassBall2 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape2, glassBallMaterial));
		//scene->AddPrimitive(glassBall2);

		////Glass Ball3 new Sphere(9.5f, Vec3(53, 9.5f, 88)
		//std::shared_ptr<Shape> glassBallShape3 = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 9.5f, Vec3(53, 9.5f, 88)));
		//std::shared_ptr<Primitive> glassBall3 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape3, glassBallMaterial));
		//scene->AddPrimitive(glassBall3);

		//Light
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 8.f, Vec3(50, 81.6f - 16.5f, 81.6f)));//Lite
		std::shared_ptr<Light> light0 = std::shared_ptr<Light>(new AreaLight(lightShape, Vec3(0.3f, 0.3f, 0.3f) * 100));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> lightPrimitive = std::shared_ptr<Primitive>(new GeometryPrimitive(lightShape, lightMaterial, light0));
		scene->AddLight(lightPrimitive);

		//Light
		//Transform lightTransform = Transform::Translate(Vec3(50, 81.6f - 2.0f, 81.6f));
		//Vec3 lightP0 = lightTransform(Vec3(8.5f, 0, 8.5f));
		//Vec3 lightP1 = lightTransform(Vec3(8.5f, 0, -8.5f));
		//Vec3 lightP2 = lightTransform(Vec3(-8.5f, 0, 8.5f));
		//Vec3 lightP3 = lightTransform(Vec3(-8.5f, 0, -8.5f));
		//Vec3 lightNormal = Vec3(0, -1, 0);
		//std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		//std::shared_ptr<Shape> lightTriangleShape0 = 
		//	std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, lightP0, lightP1, lightP2, lightNormal));
		//std::shared_ptr<Shape> lightTriangleShape1 =
		//	std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, lightP1, lightP3, lightP2, lightNormal));
		//std::shared_ptr<Light> light0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 200));
		//std::shared_ptr<Light> light1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 200));
		//std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		//std::shared_ptr<Primitive> lightPrimitive0 = 
		//	std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, light0));
		//std::shared_ptr<Primitive> lightPrimitive1 = 
		//	std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, light1));
		//scene->AddLight(lightPrimitive0);
		//scene->AddLight(lightPrimitive1);
	}
};





class CornellBoxTriangle2 {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .75f, .75f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));

		//Points
		Vec3 p0(-1, -1, 1);
		Vec3 p1(-1, -1, -1);
		Vec3 p2(-1, 1, -1);
		Vec3 p3(-1, 1, 1);
		Vec3 p4(1, -1, 1);
		Vec3 p5(1, -1, -1);
		Vec3 p6(1, 1, -1);
		Vec3 p7(1, 1, 1);

		//Left
		Vec3 normalLeft = Vec3(1, 0, 0);
		std::shared_ptr<Shape> leftShape1 = 
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p0, p1, p2, normalLeft));
		std::shared_ptr<Shape> leftShape2 =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p2, p3, p0, normalLeft));
		std::shared_ptr<Material> leftWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(redConstant));
		std::shared_ptr<Primitive> leftTriangle1 = 
			std::shared_ptr<Primitive>(new GeometryPrimitive(leftShape1, leftWallMaterial));
		std::shared_ptr<Primitive> leftTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(leftShape2, leftWallMaterial));
		scene->AddPrimitive(leftTriangle1);
		scene->AddPrimitive(leftTriangle2);


		//Right
		Vec3 normalRight = Vec3(-1, 0, 0);
		std::shared_ptr<Shape> rightShape1 =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p4, p5, p6, normalRight));
		std::shared_ptr<Shape> rightShape2 =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p4, p6, p7, normalRight));
		std::shared_ptr<Material> rightWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		std::shared_ptr<Primitive> rightTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(rightShape1, rightWallMaterial));
		std::shared_ptr<Primitive> rightTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(rightShape2, rightWallMaterial));
		scene->AddPrimitive(rightTriangle1);
		scene->AddPrimitive(rightTriangle2);

		//Back
		Vec3 normalBack = Vec3(0, 0, 1);
		std::shared_ptr<Shape> backShape1 =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p1, p5, p6, normalBack));
		std::shared_ptr<Shape> backShape2 =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p1, p6, p2, normalBack));
		std::shared_ptr<Material> backWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> backTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(backShape1, backWallMaterial));
		std::shared_ptr<Primitive> backTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(backShape2, backWallMaterial));
		scene->AddPrimitive(backTriangle1);
		scene->AddPrimitive(backTriangle2);

		//Botom
		Vec3 normalBotom = Vec3(0, 1, 0);
		std::shared_ptr<Shape> botomShape1 =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p0, p4, p5, normalBotom));
		std::shared_ptr<Shape> botomShape2 =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p0, p5, p1, normalBotom));
		std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botomTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		std::shared_ptr<Primitive> botomTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		scene->AddPrimitive(botomTriangle1);
		scene->AddPrimitive(botomTriangle2);

		////Top
		Vec3 normalTop = Vec3(0, -1, 0);
		std::shared_ptr<Shape> topShape1 =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p3, p7, p6, normalTop));
		std::shared_ptr<Shape> topShape2 =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, p3, p6, p2, normalTop));
		std::shared_ptr<Material> topWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> topTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape1, topWallMaterial));
		std::shared_ptr<Primitive> topTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape2, topWallMaterial));
		scene->AddPrimitive(topTriangle1);
		scene->AddPrimitive(topTriangle2);


		//Glass Ball
		std::shared_ptr<Shape> glassBallShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 0.35f, Vec3(0.5f, -0.6f, 0.3f)));
		std::shared_ptr<Material> glassBallMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Primitive> glassBall = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape, glassBallMaterial));
		scene->AddPrimitive(glassBall);

		//Diffuse Ball
		//std::shared_ptr<Shape> diffuseBallShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 0.35f, Vec3(-0.5f, -0.6f, -0.3f)));
		//std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		//std::shared_ptr<Primitive> diffuseBall = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape, diffuseBallMaterial));
		//scene->AddPrimitive(diffuseBall);

		//Mirror Ball
		//std::shared_ptr<Shape> mirrorBallShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 0.35f, Vec3(-0.5f, -0.6f, -0.3f)));
		//std::shared_ptr<Material> mirrorBallMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		//std::shared_ptr<Primitive> mirrorBall = std::shared_ptr<Primitive>(new GeometryPrimitive(mirrorBallShape, mirrorBallMaterial));
		//scene->AddPrimitive(mirrorBall);

		//Roughness Ball
		std::shared_ptr<Texture<real>> eta = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(1.5));
		std::shared_ptr<Texture<real>> k = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(1.5));
		std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.06));

		std::shared_ptr<Shape> roughBallShape = std::shared_ptr<Shape>(new Sphere(nullptr, nullptr, 0.35f, Vec3(-0.5f, -0.6f, -0.3f)));
		std::shared_ptr<Material> roughBallMaterial = 
			std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		std::shared_ptr<Primitive> roughBall = std::shared_ptr<Primitive>(new GeometryPrimitive(roughBallShape, roughBallMaterial));
		scene->AddPrimitive(roughBall);

		//Light
		Vec3 lightP0 = Vec3(0.25f, 0.965, 0.25f);
		Vec3 lightP1 = Vec3(0.25f, 0.965, -0.25f);
		Vec3 lightP2 = Vec3(-0.25f, 0.965, -0.25f);
		Vec3 lightP3 = Vec3(-0.25f, 0.965, 0.25f);
		Vec3 lightNormal = Vec3(0, -1, 0);
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightTriangleShape0 = 
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, lightP0, lightP1, lightP2, lightNormal));
		std::shared_ptr<Shape> lightTriangleShape1 =
			std::shared_ptr<Shape>(new Triangle(nullptr, nullptr, lightP0, lightP2, lightP3, lightNormal));
		std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 100));
		std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 100));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> triangleLightPrimitive0 = 
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, triangleLight0));
		std::shared_ptr<Primitive> triangleLightPrimitive1 = 
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, triangleLight1));
		scene->AddLight(triangleLightPrimitive0);
		scene->AddLight(triangleLightPrimitive1);
	}
};

NAMESPACE_END