#pragma once
#include "common/Core.h"
#include "visual\Scene.h"
#include "shape\sphere.h"
#include "material\Diffuse.h"
#include "material\Mirror.h"
#include "material\Glass.h"
#include "light\AreaLight.h"
#include "material\Roughness.h"
#include "texture\ConstantTexture.h"
#include "texture\ImageTexture.h"
#include "shape\Triangle.h"
#include "light\EnvLight.h"
#include "material\Plastic.h"
#include "material\RoughDielectric.h"
#include "shape\HeartSurface.h"
#include "light\PointLight.h"
#include "volume\Homogeneous.h"

using namespace Gyunity;

class CornellBoxSphere {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .75f, .75f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));


		//Left
		std::shared_ptr<Shape> leftWallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 1e5f, Vec3(1e5f + 1, 40.8f, 81.6f), true));
		std::shared_ptr<Material> leftWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(redConstant));
		std::shared_ptr<Primitive> leftWall = std::shared_ptr<Primitive>(new GeometryPrimitive(leftWallShape, leftWallMaterial));
		scene->AddPrimitive(leftWall);

		//Right
		std::shared_ptr<Shape> rightWallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 1e5f, Vec3(-1e5f + 99, 40.8f, 81.6f), true));
		std::shared_ptr<Material> rightWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		std::shared_ptr<Primitive> rightWall = std::shared_ptr<Primitive>(new GeometryPrimitive(rightWallShape, rightWallMaterial));
		scene->AddPrimitive(rightWall);

		//Back
		std::shared_ptr<Shape> backWallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 1e5f, Vec3(50, 40.8f, 1e5), true));
		std::shared_ptr<Material> backWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> backWall = std::shared_ptr<Primitive>(new GeometryPrimitive(backWallShape, backWallMaterial));
		scene->AddPrimitive(backWall);

		//Botom
		std::shared_ptr<Shape> botomShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 1e5f, Vec3(50, 1e5, 81.6f), true));
		std::shared_ptr<Material> botomMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botom = std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape, botomMaterial));
		scene->AddPrimitive(botom);

		//Top
		std::shared_ptr<Shape> topShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(),1e5f, Vec3(50, -1e5f + 81.6f, 81.6f), true));
		std::shared_ptr<Material> topMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> top = std::shared_ptr<Primitive>(new GeometryPrimitive(topShape, topMaterial));
		scene->AddPrimitive(top);

		//Diffuse Ball1
		std::shared_ptr<Shape> diffuseBallShape1 = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 9.5f, Vec3(23, 0.0f, 98)));
		std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(fullWhiteConstant));
		std::shared_ptr<Primitive> diffuseBall1 = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape1, diffuseBallMaterial));
		scene->AddPrimitive(diffuseBall1);

		//Glass Ball1
		std::shared_ptr<Shape> glassBallShape1 = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 16.5f, Vec3(73, 26.5f, 78)));
		std::shared_ptr<Material> glassBallMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Primitive> glassBall1 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape1, glassBallMaterial));
		scene->AddPrimitive(glassBall1);

		//Diffuse Ball2 new Sphere(7.0, Vec3(27, 16.5, 47), Vec3(), Vec3(.25, .25, .75)
		//std::shared_ptr<Shape> diffuseBallShape2 = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 7.0, Vec3(27, 16.5, 47)));
		//std::shared_ptr<Material> diffuseBallMaterial2 = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		//std::shared_ptr<Primitive> diffuseBall2 = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape2, diffuseBallMaterial2));
		//scene->AddPrimitive(diffuseBall2);


		//Glass Ball2 new Sphere(16.5f, Vec3(27, 16.5f, 47)
		std::shared_ptr<Shape> glassBallShape2 = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 16.5f, Vec3(27, 16.5f, 47)));
		std::shared_ptr<Primitive> glassBall2 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape2, glassBallMaterial));
		scene->AddPrimitive(glassBall2);

		//Glass Ball3 new Sphere(9.5f, Vec3(53, 9.5f, 88)
		std::shared_ptr<Shape> glassBallShape3 = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 9.5f, Vec3(53, 9.5f, 88)));
		std::shared_ptr<Primitive> glassBall3 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape3, glassBallMaterial));
		scene->AddPrimitive(glassBall3);

		//Light
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 8.f, Vec3(50, 81.6f - 16.5f, 81.6f)));//Lite
		std::shared_ptr<Light> light0 = std::shared_ptr<Light>(new AreaLight(lightShape, Vec3(0.3f, 0.3f, 0.3f) * 100));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> lightPrimitive = std::shared_ptr<Primitive>(new GeometryPrimitive(lightShape, lightMaterial, light0));
		scene->AddLight(light0);
		scene->AddPrimitive(lightPrimitive);

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
		std::shared_ptr<Shape> leftWallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 1e5f, Vec3(1e5f + 1, 40.8f, 81.6f), true));
		std::shared_ptr<Material> leftWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(redConstant));
		std::shared_ptr<Primitive> leftWall = std::shared_ptr<Primitive>(new GeometryPrimitive(leftWallShape, leftWallMaterial));
		scene->AddPrimitive(leftWall);

		//Right
		std::shared_ptr<Shape> rightWallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 1e5f, Vec3(-1e5f + 99, 40.8f, 81.6f), true));
		std::shared_ptr<Material> rightWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		std::shared_ptr<Primitive> rightWall = std::shared_ptr<Primitive>(new GeometryPrimitive(rightWallShape, rightWallMaterial));
		scene->AddPrimitive(rightWall);

		//Back
		std::shared_ptr<Shape> backWallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 1e5f, Vec3(50, 40.8f, 1e5), true));
		std::shared_ptr<Material> backWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> backWall = std::shared_ptr<Primitive>(new GeometryPrimitive(backWallShape, backWallMaterial));
		scene->AddPrimitive(backWall);

		//Botom
		std::shared_ptr<Shape> botomShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 1e5f, Vec3(50, 1e5, 81.6f), true));
		std::shared_ptr<Material> botomMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botom = std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape, botomMaterial));
		scene->AddPrimitive(botom);

		//Top
		std::shared_ptr<Shape> topShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 1e5f, Vec3(50, -1e5f + 81.6f, 81.6f), true));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p1, p2, faceNormal));
		std::shared_ptr<Material> triangleMaterial = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		std::shared_ptr<Primitive> triangle = std::shared_ptr<Primitive>(new GeometryPrimitive(triangleShape, triangleMaterial));
		scene->AddPrimitive(triangle);


		//Diffuse Ball1
		std::shared_ptr<Shape> diffuseBallShape1 = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 9.5f, Vec3(23, 0.0f, 98)));
		std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(fullWhiteConstant));
		std::shared_ptr<Primitive> diffuseBall1 = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape1, diffuseBallMaterial));
		scene->AddPrimitive(diffuseBall1);

		////Glass Ball1
		//std::shared_ptr<Shape> glassBallShape1 = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 16.5f, Vec3(73, 26.5f, 78)));
		//std::shared_ptr<Material> glassBallMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		//std::shared_ptr<Primitive> glassBall1 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape1, glassBallMaterial));
		//scene->AddPrimitive(glassBall1);

		//Diffuse Ball2 new Sphere(7.0, Vec3(27, 16.5, 47), Vec3(), Vec3(.25, .25, .75)
		//std::shared_ptr<Shape> diffuseBallShape2 = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 7.0, Vec3(27, 16.5, 47)));
		//std::shared_ptr<Material> diffuseBallMaterial2 = std::shared_ptr<Material>(new DiffuseMaterial(blueConstant));
		//std::shared_ptr<Primitive> diffuseBall2 = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape2, diffuseBallMaterial2));
		//scene->AddPrimitive(diffuseBall2);


		//Glass Ball2 new Sphere(16.5f, Vec3(27, 16.5f, 47)
		//std::shared_ptr<Shape> glassBallShape2 = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 16.5f, Vec3(27, 16.5f, 47)));
		//std::shared_ptr<Primitive> glassBall2 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape2, glassBallMaterial));
		//scene->AddPrimitive(glassBall2);

		////Glass Ball3 new Sphere(9.5f, Vec3(53, 9.5f, 88)
		//std::shared_ptr<Shape> glassBallShape3 = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 9.5f, Vec3(53, 9.5f, 88)));
		//std::shared_ptr<Primitive> glassBall3 = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape3, glassBallMaterial));
		//scene->AddPrimitive(glassBall3);

		//Light
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 8.f, Vec3(50, 81.6f - 16.5f, 81.6f)));//Lite
		std::shared_ptr<Light> light0 = std::shared_ptr<Light>(new AreaLight(lightShape, Vec3(0.3f, 0.3f, 0.3f) * 100));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> lightPrimitive = std::shared_ptr<Primitive>(new GeometryPrimitive(lightShape, lightMaterial, light0));
		scene->AddLight(light0);
		scene->AddPrimitive(lightPrimitive);

		//Light
		//Transform lightTransform = Transform::Translate(Vec3(50, 81.6f - 2.0f, 81.6f));
		//Vec3 lightP0 = lightTransform(Vec3(8.5f, 0, 8.5f));
		//Vec3 lightP1 = lightTransform(Vec3(8.5f, 0, -8.5f));
		//Vec3 lightP2 = lightTransform(Vec3(-8.5f, 0, 8.5f));
		//Vec3 lightP3 = lightTransform(Vec3(-8.5f, 0, -8.5f));
		//Vec3 lightNormal = Vec3(0, -1, 0);
		//std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		//std::shared_ptr<Shape> lightTriangleShape0 = 
		//	std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP1, lightP2, lightNormal));
		//std::shared_ptr<Shape> lightTriangleShape1 =
		//	std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP1, lightP3, lightP2, lightNormal));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p1, p2, normalLeft));
		std::shared_ptr<Shape> leftShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p2, p3, p0, normalLeft));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p5, p6, normalRight));
		std::shared_ptr<Shape> rightShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p6, p7, normalRight));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p5, p6, normalBack));
		std::shared_ptr<Shape> backShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p6, p2, normalBack));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom));
		std::shared_ptr<Shape> botomShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom));
		std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botomTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		std::shared_ptr<Primitive> botomTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		scene->AddPrimitive(botomTriangle1);
		scene->AddPrimitive(botomTriangle2);

		//std::shared_ptr<Texture<Vec3>> botomTexture =
		//	std::shared_ptr<Texture<Vec3>>(new ImageTexture<Vec3>("..\\resources\\texture_images\\checkboard3.bmp"));
		//static Vec2 uvs1[3] = { Vec2(0, 0), Vec2(1, 0), Vec2(1, 1) };
		//static Vec2 uvs2[3] = { Vec2(0, 0), Vec2(1, 1), Vec2(0, 1) };
		//Vec3 normalBotom = Vec3(0, 1, 0);
		//std::shared_ptr<Shape> botomShape1 =
		//	std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom, &uvs1[0]));
		//std::shared_ptr<Shape> botomShape2 =
		//	std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom, &uvs2[0]));
		//std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(botomTexture));
		//std::shared_ptr<Primitive> botomTriangle1 =
		//	std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		//std::shared_ptr<Primitive> botomTriangle2 =
		//	std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		//scene->AddPrimitive(botomTriangle1);
		//scene->AddPrimitive(botomTriangle2);

		////Top
		Vec3 normalTop = Vec3(0, -1, 0);
		std::shared_ptr<Shape> topShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p7, p6, normalTop));
		std::shared_ptr<Shape> topShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p6, p2, normalTop));
		std::shared_ptr<Material> topWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> topTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape1, topWallMaterial));
		std::shared_ptr<Primitive> topTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape2, topWallMaterial));
		scene->AddPrimitive(topTriangle1);
		scene->AddPrimitive(topTriangle2);


		//Glass Ball
		std::shared_ptr<Shape> glassBallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 0.35f, Vec3(0.5f, -0.6f, 0.3f)));
		std::shared_ptr<Material> glassBallMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Primitive> glassBall = std::shared_ptr<Primitive>(new GeometryPrimitive(glassBallShape, glassBallMaterial));
		scene->AddPrimitive(glassBall);

		//Diffuse Ball
		//std::shared_ptr<Shape> diffuseBallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 0.35f, Vec3(-0.5f, -0.6f, -0.3f)));
		//std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		//std::shared_ptr<Primitive> diffuseBall = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape, diffuseBallMaterial));
		//scene->AddPrimitive(diffuseBall);

		//Mirror Ball
		//std::shared_ptr<Shape> mirrorBallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 0.35f, Vec3(-0.5f, -0.6f, -0.3f)));
		//std::shared_ptr<Material> mirrorBallMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		//std::shared_ptr<Primitive> mirrorBall = std::shared_ptr<Primitive>(new GeometryPrimitive(mirrorBallShape, mirrorBallMaterial));
		//scene->AddPrimitive(mirrorBall);

		//Roughness Ball
		//std::shared_ptr<Texture<real>> eta = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(1.5));
		//std::shared_ptr<Texture<real>> k = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(1.5));
		//std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		//std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.06));
		//std::shared_ptr<Shape> roughBallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 0.35f, Vec3(-0.5f, -0.6f, -0.3f)));
		//std::shared_ptr<Material> roughBallMaterial = 
		//	std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		//std::shared_ptr<Primitive> roughBall = std::shared_ptr<Primitive>(new GeometryPrimitive(roughBallShape, roughBallMaterial));
		//scene->AddPrimitive(roughBall);

		//Diffuse Ball
		//static Transform o2w = 
		//	Transform::Translate(Vec3(-0.5f, -0.6f, -0.3f)) * Transform::RotateY(220) * Transform::RotateX(90) * Transform::Scale(-1, 1, 1);
		static Transform o2w =
			Transform::Translate(Vec3(-0.5f, -0.6f, -0.3f)) * Transform::RotateY(220) * Transform::Scale(1, 1, 1);
		static Transform w2o = Inverse(o2w);
		std::shared_ptr<Texture<Vec3>> imageTexture1 =
			std::shared_ptr<Texture<Vec3>>(new ImageTexture<Vec3>("..\\resources\\texture_images\\sphere_mapping_8k.jpg"));
		std::shared_ptr<Shape> diffuseBallShape = std::shared_ptr<Shape>(new Sphere(o2w, w2o, 0.35f, Vec3(0, 0, 0)));
		std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(imageTexture1));
		std::shared_ptr<Primitive> diffuseBall = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape, diffuseBallMaterial));
		scene->AddPrimitive(diffuseBall);

		//Light
		Vec3 lightP0 = Vec3(0.25f, 0.965, 0.25f);
		Vec3 lightP1 = Vec3(0.25f, 0.965, -0.25f);
		Vec3 lightP2 = Vec3(-0.25f, 0.965, -0.25f);
		Vec3 lightP3 = Vec3(-0.25f, 0.965, 0.25f);
		Vec3 lightNormal = Vec3(0, -1, 0);
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightTriangleShape0 = 
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP1, lightP2, lightNormal));
		std::shared_ptr<Shape> lightTriangleShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP2, lightP3, lightNormal));
		std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 100));
		std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 100));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> triangleLightPrimitive0 = 
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, triangleLight0));
		std::shared_ptr<Primitive> triangleLightPrimitive1 = 
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, triangleLight1));
		scene->AddLight(triangleLight0);
		scene->AddLight(triangleLight1);
		scene->AddPrimitive(triangleLightPrimitive0);
		scene->AddPrimitive(triangleLightPrimitive1);
		

	}
};



class CornellBoxMesh {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .75f, .75f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));


		std::shared_ptr<Texture<Vec3>> objWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.5f, .5f, .5f)));

		std::shared_ptr<Material> glassMeshMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Material> diffuseMeshMaterial = std::shared_ptr<Material>(new DiffuseMaterial(objWhiteConstant));
		std::shared_ptr<Material> mirrorMeshMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		std::shared_ptr<Texture<Vec3>> eta = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<Vec3>> k = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.06));
		std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Material> roughMeshMaterial =
			std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		std::shared_ptr<Material> waterMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant, 1.f, 1.33));

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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p1, p2, normalLeft));
		std::shared_ptr<Shape> leftShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p2, p3, p0, normalLeft));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p5, p6, normalRight));
		std::shared_ptr<Shape> rightShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p6, p7, normalRight));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p5, p6, normalBack));
		std::shared_ptr<Shape> backShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p6, p2, normalBack));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom));
		std::shared_ptr<Shape> botomShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom));
		std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botomTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		std::shared_ptr<Primitive> botomTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		scene->AddPrimitive(botomTriangle1);
		scene->AddPrimitive(botomTriangle2);

		//Top
		Vec3 normalTop = Vec3(0, -1, 0);
		std::shared_ptr<Shape> topShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p7, p6, normalTop));
		std::shared_ptr<Shape> topShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p6, p2, normalTop));
		std::shared_ptr<Material> topWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> topTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape1, topWallMaterial));
		std::shared_ptr<Primitive> topTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape2, topWallMaterial));
		scene->AddPrimitive(topTriangle1);
		scene->AddPrimitive(topTriangle2);


		////Diffuse Ball
		//std::shared_ptr<Shape> diffuseBallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 0.35f, Vec3(-0.5f, -0.6f, -0.3f)));
		//std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		//std::shared_ptr<Primitive> diffuseBall = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape, diffuseBallMaterial));
		//scene->AddPrimitive(diffuseBall);

		//Mesh
		//std::shared_ptr<Material> glassMeshMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		//std::shared_ptr<Material> diffuseMeshMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		//std::shared_ptr<Material> mirrorMeshMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		//std::shared_ptr<Texture<Vec3>> eta = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		//std::shared_ptr<Texture<Vec3>> k = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		//std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		//std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.06));
		//std::shared_ptr<Material> roughMeshMaterial = 
		//	std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		
		std::shared_ptr<Mesh> mesh = std::shared_ptr<Mesh>(new Mesh());
		//mesh->LoadFromFile("..\\resources\\meshs\\bunny.obj");
		//mesh->SetMaterial(glassMeshMaterial);
		//Transform transform = Transform::Translate(Vec3(0, -1, 0)) * Transform::Scale(6.0, 6.0, 6.0);
		//mesh->LoadFromFile("..\\resources\\meshs\\bunny2.obj");
		//mesh->SetMaterial(glassMeshMaterial);
		//Transform transform = Transform::Translate(Vec3(0, -0.65, 0)) * Transform::Scale(0.65, 0.65, 0.65);

		mesh->LoadFromFile("..\\resources\\meshs\\bunny3.obj");
		mesh->SetMaterial(glassMeshMaterial);
		Transform transform = Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(0.28, 0.28, 0.28);
		scene->AddMesh(*mesh, transform);

		//std::shared_ptr<Shape> ball = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 0.35f, Vec3(0.f, 0.f, 0.f)));
		//std::shared_ptr<Primitive> ballPrimitive = std::shared_ptr<Primitive>(new GeometryPrimitive(ball, roughMeshMaterial));
		//scene->AddPrimitive(ballPrimitive);

		//std::shared_ptr<Mesh> waterMesh = std::shared_ptr<Mesh>(new Mesh());
		//waterMesh->LoadFromFile("..\\resources\\meshs\\water.obj");
		//Transform transformWater =
		//	Transform::Translate(Vec3(0, -0.5, 0)) *

		//	Transform::Scale(0.02, 0.02, 1) *
		//	Transform::RotateY(-180) *
		//	Transform::RotateX(-90) *

		//	Transform::Translate(Vec3(-50, -50, 0)) *
		//	Transform::Scale(1, 2, 1);
		//waterMesh->SetMaterial(waterMaterial);
		//scene->AddMesh(*waterMesh, transformWater);


		//std::shared_ptr<Mesh> objMesh = std::shared_ptr<Mesh>(new Mesh());
		//objMesh->LoadFromFile("..\\resources\\meshs\\Alucy.obj");

		//Transform transformWater =
		//	Transform::Translate(Vec3(0, -1, 0)) *
		//	Transform::RotateY(60) * 
		//	Transform::Scale(0.0019, 0.0019, 0.0019);
		//objMesh->SetMaterial(glassMeshMaterial);
		//scene->AddMesh(*objMesh, transformWater);


		//std::shared_ptr<Mesh> meshVenus = std::shared_ptr<Mesh>(new Mesh());
		//meshVenus->LoadFromFile("..\\resources\\meshs\\venusm.obj");
		//meshVenus->SetMaterial(diffuseMeshMaterial);
		//Transform transformVenus = Transform::Translate(Vec3(-0.4, -0.73, 0)) * Transform::Scale(0.0003, 0.0003, 0.0003);
		//scene->AddMesh(*meshVenus, transformVenus);

		//Light
		Vec3 lightP0 = Vec3(0.25f, 0.965, 0.25f);
		Vec3 lightP1 = Vec3(0.25f, 0.965, -0.25f);
		Vec3 lightP2 = Vec3(-0.25f, 0.965, -0.25f);
		Vec3 lightP3 = Vec3(-0.25f, 0.965, 0.25f);
		//Vec3 lightP0 = Vec3(0.0015f, 0.965, 0.0015f);
		//Vec3 lightP1 = Vec3(0.0015f, 0.965, -0.0015f);
		//Vec3 lightP2 = Vec3(-0.0015f, 0.965, -0.0015f);
		//Vec3 lightP3 = Vec3(-0.0015f, 0.965, 0.0015f);
		Vec3 lightNormal = Vec3(0, -1, 0);
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightTriangleShape0 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP1, lightP2, lightNormal));
		std::shared_ptr<Shape> lightTriangleShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP2, lightP3, lightNormal));
		//std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 2000000));
		//std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 2000000));
		std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 85));
		std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 85));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> triangleLightPrimitive0 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, triangleLight0));
		std::shared_ptr<Primitive> triangleLightPrimitive1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, triangleLight1));
		scene->AddLight(triangleLight0);
		scene->AddLight(triangleLight1);
		scene->AddPrimitive(triangleLightPrimitive0);
		scene->AddPrimitive(triangleLightPrimitive1);

		////Light
		//std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		//std::shared_ptr<Shape> lightShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 0.13f, Vec3(0.f, 0.65f, 0.f)));//Lite
		//std::shared_ptr<Light> light0 = std::shared_ptr<Light>(new AreaLight(lightShape, Vec3(0.3f, 0.3f, 0.3f) * 100));
		//std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		//std::shared_ptr<Primitive> lightPrimitive = std::shared_ptr<Primitive>(new GeometryPrimitive(lightShape, lightMaterial, light0));
		//scene->AddLight(lightPrimitive);


		//PointLight
		//Vec3 intensity(2.5, 2.5, 2.5);
		//Vec3 pLight(0, 0.965, 0);
		//std::shared_ptr<Light> light = std::shared_ptr<Light>(new PointLight(pLight, intensity));
		//scene->AddLight(light);
	}
};





class CornellBoxMeshEmpty {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .75f, .75f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));


		std::shared_ptr<Texture<Vec3>> objWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.5f, .5f, .5f)));

		std::shared_ptr<Material> glassMeshMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Material> diffuseMeshMaterial = std::shared_ptr<Material>(new DiffuseMaterial(objWhiteConstant));
		std::shared_ptr<Material> mirrorMeshMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		std::shared_ptr<Texture<Vec3>> eta = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<Vec3>> k = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.06));
		std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Material> roughMeshMaterial =
			std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		std::shared_ptr<Material> waterMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant, 1.f, 1.33));

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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p1, p2, normalLeft));
		std::shared_ptr<Shape> leftShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p2, p3, p0, normalLeft));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p5, p6, normalRight));
		std::shared_ptr<Shape> rightShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p6, p7, normalRight));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p5, p6, normalBack));
		std::shared_ptr<Shape> backShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p6, p2, normalBack));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom));
		std::shared_ptr<Shape> botomShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom));
		std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botomTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		std::shared_ptr<Primitive> botomTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		scene->AddPrimitive(botomTriangle1);
		scene->AddPrimitive(botomTriangle2);

		//Top
		Vec3 normalTop = Vec3(0, -1, 0);
		std::shared_ptr<Shape> topShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p7, p6, normalTop));
		std::shared_ptr<Shape> topShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p6, p2, normalTop));
		std::shared_ptr<Material> topWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> topTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape1, topWallMaterial));
		std::shared_ptr<Primitive> topTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape2, topWallMaterial));
		scene->AddPrimitive(topTriangle1);
		scene->AddPrimitive(topTriangle2);

		std::shared_ptr<Mesh> mesh = std::shared_ptr<Mesh>(new Mesh());

		//mesh->LoadFromFile("..\\resources\\meshs\\bunny3.obj");
		//mesh->SetMaterial(glassMeshMaterial);
		//Transform transform = Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(0.28, 0.28, 0.28);
		//scene->AddMesh(*mesh, transform);


		//Light
		Vec3 lightP0 = Vec3(0.25f, 0.965, 0.25f);
		Vec3 lightP1 = Vec3(0.25f, 0.965, -0.25f);
		Vec3 lightP2 = Vec3(-0.25f, 0.965, -0.25f);
		Vec3 lightP3 = Vec3(-0.25f, 0.965, 0.25f);

		Vec3 lightNormal = Vec3(0, -1, 0);
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightTriangleShape0 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP1, lightP2, lightNormal));
		std::shared_ptr<Shape> lightTriangleShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP2, lightP3, lightNormal));

		std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 85));
		std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 85));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> triangleLightPrimitive0 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, triangleLight0));
		std::shared_ptr<Primitive> triangleLightPrimitive1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, triangleLight1));
		scene->AddLight(triangleLight0);
		scene->AddLight(triangleLight1);
		scene->AddPrimitive(triangleLightPrimitive0);
		scene->AddPrimitive(triangleLightPrimitive1);


	}
};



class CornellBoxMeshWithSlope{
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .75f, .75f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));


		std::shared_ptr<Texture<Vec3>> objWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.5f, .5f, .5f)));

		std::shared_ptr<Material> glassMeshMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Material> diffuseMeshMaterial = std::shared_ptr<Material>(new DiffuseMaterial(objWhiteConstant));
		std::shared_ptr<Material> mirrorMeshMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		std::shared_ptr<Texture<Vec3>> eta = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<Vec3>> k = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.06));
		std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Material> roughMeshMaterial =
			std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		std::shared_ptr<Material> waterMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant, 1.f, 1.33));

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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p1, p2, normalLeft));
		std::shared_ptr<Shape> leftShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p2, p3, p0, normalLeft));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p5, p6, normalRight));
		std::shared_ptr<Shape> rightShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p6, p7, normalRight));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p5, p6, normalBack));
		std::shared_ptr<Shape> backShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p6, p2, normalBack));
		std::shared_ptr<Material> backWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> backTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(backShape1, backWallMaterial));
		std::shared_ptr<Primitive> backTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(backShape2, backWallMaterial));
		scene->AddPrimitive(backTriangle1);
		scene->AddPrimitive(backTriangle2);

		//Bottom
		Vec3 normalBotom = Vec3(0, 1, 0);
		std::shared_ptr<Shape> botomShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom));
		std::shared_ptr<Shape> botomShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom));
		std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botomTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		std::shared_ptr<Primitive> botomTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		scene->AddPrimitive(botomTriangle1);
		scene->AddPrimitive(botomTriangle2);

		//Top
		Vec3 normalTop = Vec3(0, -1, 0);
		std::shared_ptr<Shape> topShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p7, p6, normalTop));
		std::shared_ptr<Shape> topShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p6, p2, normalTop));
		std::shared_ptr<Material> topWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> topTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape1, topWallMaterial));
		std::shared_ptr<Primitive> topTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape2, topWallMaterial));
		scene->AddPrimitive(topTriangle1);
		scene->AddPrimitive(topTriangle2);

		std::shared_ptr<Mesh> mesh = std::shared_ptr<Mesh>(new Mesh());


		//slope
		Vec3 s0(-1, 0, -1);
		Vec3 s1(1, 0, -1);
		//Vec3 s0(-1, 1, -1);
		//Vec3 s1(1, 1, -1);
		Vec3 s2(1, -1, 1);
		Vec3 s3(-1, -1, 1);

		Vec3 slopeNormal = Cross(s0 - s1, s2 - s1).Norm();
		std::shared_ptr<Shape> slopeShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), s0, s1, s2, slopeNormal));
		std::shared_ptr<Shape> slopeShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), s0, s2, s3, slopeNormal));
		std::shared_ptr<Material> slopeMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> slope1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(slopeShape1, slopeMaterial));
		std::shared_ptr<Primitive> slope2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(slopeShape2, slopeMaterial));
		scene->AddPrimitive(slope1);
		scene->AddPrimitive(slope2);


		//Light
		Vec3 lightP0 = Vec3(0.25f, 0.965, 0.25f);
		Vec3 lightP1 = Vec3(0.25f, 0.965, -0.25f);
		Vec3 lightP2 = Vec3(-0.25f, 0.965, -0.25f);
		Vec3 lightP3 = Vec3(-0.25f, 0.965, 0.25f);

		Vec3 lightNormal = Vec3(0, -1, 0);
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightTriangleShape0 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP1, lightP2, lightNormal));
		std::shared_ptr<Shape> lightTriangleShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP2, lightP3, lightNormal));

		std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 85));
		std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 85));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> triangleLightPrimitive0 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, triangleLight0));
		std::shared_ptr<Primitive> triangleLightPrimitive1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, triangleLight1));
		scene->AddLight(triangleLight0);
		scene->AddLight(triangleLight1);
		scene->AddPrimitive(triangleLightPrimitive0);
		scene->AddPrimitive(triangleLightPrimitive1);


	}
};




class CornellBoxDiffuseBunny {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .75f, .75f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));


		std::shared_ptr<Texture<Vec3>> objWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.5f, .5f, .5f)));

		std::shared_ptr<Material> glassMeshMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Material> diffuseMeshMaterial = std::shared_ptr<Material>(new DiffuseMaterial(objWhiteConstant));
		std::shared_ptr<Material> mirrorMeshMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		std::shared_ptr<Texture<Vec3>> eta = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<Vec3>> k = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.06));
		std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Material> roughMeshMaterial =
			std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		std::shared_ptr<Material> waterMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant, 1.f, 1.33));

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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p1, p2, normalLeft));
		std::shared_ptr<Shape> leftShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p2, p3, p0, normalLeft));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p5, p6, normalRight));
		std::shared_ptr<Shape> rightShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p6, p7, normalRight));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p5, p6, normalBack));
		std::shared_ptr<Shape> backShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p6, p2, normalBack));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom));
		std::shared_ptr<Shape> botomShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom));
		std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botomTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		std::shared_ptr<Primitive> botomTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		scene->AddPrimitive(botomTriangle1);
		scene->AddPrimitive(botomTriangle2);

		//Top
		Vec3 normalTop = Vec3(0, -1, 0);
		std::shared_ptr<Shape> topShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p7, p6, normalTop));
		std::shared_ptr<Shape> topShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p6, p2, normalTop));
		std::shared_ptr<Material> topWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> topTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape1, topWallMaterial));
		std::shared_ptr<Primitive> topTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape2, topWallMaterial));
		scene->AddPrimitive(topTriangle1);
		scene->AddPrimitive(topTriangle2);

		std::shared_ptr<Mesh> mesh = std::shared_ptr<Mesh>(new Mesh());

		mesh->LoadFromFile("..\\resources\\meshs\\bunny3.obj");
		mesh->SetMaterial(diffuseMeshMaterial);
		Transform transform = Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(0.28, 0.28, 0.28);
		scene->AddMesh(*mesh, transform);


		//Light
		Vec3 lightP0 = Vec3(0.25f, 0.965, 0.25f);
		Vec3 lightP1 = Vec3(0.25f, 0.965, -0.25f);
		Vec3 lightP2 = Vec3(-0.25f, 0.965, -0.25f);
		Vec3 lightP3 = Vec3(-0.25f, 0.965, 0.25f);

		Vec3 lightNormal = Vec3(0, -1, 0);
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightTriangleShape0 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP1, lightP2, lightNormal));
		std::shared_ptr<Shape> lightTriangleShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP2, lightP3, lightNormal));

		std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 85));
		std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 85));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> triangleLightPrimitive0 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, triangleLight0));
		std::shared_ptr<Primitive> triangleLightPrimitive1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, triangleLight1));
		scene->AddLight(triangleLight0);
		scene->AddLight(triangleLight1);
		scene->AddPrimitive(triangleLightPrimitive0);
		scene->AddPrimitive(triangleLightPrimitive1);


	}
};


class CornellBoxWater {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .75f, .75f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));


		std::shared_ptr<Texture<Vec3>> objWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.5f, .5f, .5f)));

		std::shared_ptr<Material> glassMeshMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Material> diffuseMeshMaterial = std::shared_ptr<Material>(new DiffuseMaterial(objWhiteConstant));
		std::shared_ptr<Material> mirrorMeshMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		std::shared_ptr<Texture<Vec3>> eta = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<Vec3>> k = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.06));
		std::shared_ptr<Material> roughMeshMaterial =
			std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		std::shared_ptr<Material> waterMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant, 1.f, 1.33));

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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p1, p2, normalLeft));
		std::shared_ptr<Shape> leftShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p2, p3, p0, normalLeft));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p5, p6, normalRight));
		std::shared_ptr<Shape> rightShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p6, p7, normalRight));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p5, p6, normalBack));
		std::shared_ptr<Shape> backShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p6, p2, normalBack));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom));
		std::shared_ptr<Shape> botomShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom));
		std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botomTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		std::shared_ptr<Primitive> botomTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		scene->AddPrimitive(botomTriangle1);
		scene->AddPrimitive(botomTriangle2);

		//Top
		Vec3 normalTop = Vec3(0, -1, 0);
		std::shared_ptr<Shape> topShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p7, p6, normalTop));
		std::shared_ptr<Shape> topShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p6, p2, normalTop));
		std::shared_ptr<Material> topWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> topTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape1, topWallMaterial));
		std::shared_ptr<Primitive> topTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape2, topWallMaterial));
		scene->AddPrimitive(topTriangle1);
		scene->AddPrimitive(topTriangle2);


		////Diffuse Ball
		//std::shared_ptr<Shape> diffuseBallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 0.35f, Vec3(-0.5f, -0.6f, -0.3f)));
		//std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		//std::shared_ptr<Primitive> diffuseBall = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape, diffuseBallMaterial));
		//scene->AddPrimitive(diffuseBall);


		std::shared_ptr<Mesh> mesh = std::shared_ptr<Mesh>(new Mesh());
		//mesh->LoadFromFile("..\\resources\\meshs\\bunny.obj");
		//mesh->SetMaterial(glassMeshMaterial);
		//Transform transform = Transform::Translate(Vec3(0, -1, 0)) * Transform::Scale(6.0, 6.0, 6.0);
		//mesh->LoadFromFile("..\\resources\\meshs\\bunny2.obj");
		//mesh->SetMaterial(glassMeshMaterial);
		//Transform transform = Transform::Translate(Vec3(0, -0.65, 0)) * Transform::Scale(0.65, 0.65, 0.65);

		//mesh->LoadFromFile("..\\resources\\meshs\\bunny3.obj");
		//mesh->SetMaterial(diffuseMeshMaterial);
		//Transform transform = Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(0.28, 0.28, 0.28);
		//scene->AddMesh(*mesh, transform);

		//std::shared_ptr<Shape> ball = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 0.35f, Vec3(0.f, 0.f, 0.f)));
		//std::shared_ptr<Primitive> ballPrimitive = std::shared_ptr<Primitive>(new GeometryPrimitive(ball, roughMeshMaterial));
		//scene->AddPrimitive(ballPrimitive);

		std::shared_ptr<Mesh> waterMesh = std::shared_ptr<Mesh>(new Mesh());
		waterMesh->LoadFromFile("..\\resources\\meshs\\water.obj");
		Transform transformWater =
			Transform::Translate(Vec3(0, -0.35, 0)) *

			Transform::Scale(0.02, 0.02, 0.02) *
			//Transform::RotateY(-180) *
			

			Transform::Translate(Vec3(-50, 0, 50)) *
			Transform::RotateX(-90)*
			Transform::Scale(1, 2, 1);
		waterMesh->SetMaterial(waterMaterial);
		scene->AddMesh(*waterMesh, transformWater);


		std::shared_ptr<Mesh> objMesh = std::shared_ptr<Mesh>(new Mesh());
		objMesh->LoadFromFile("..\\resources\\meshs\\Alucy.obj");

		Transform transformObj =
			Transform::Translate(Vec3(-0.3, 0, -0.2)) *
			Transform::Translate(Vec3(0, -1, 0)) *
			Transform::RotateY(70) *
			Transform::Scale(0.0017, 0.0017, 0.0017);
		objMesh->SetMaterial(diffuseMeshMaterial);
		scene->AddMesh(*objMesh, transformObj);


		//Light
		//Vec3 lightP0 = Vec3(0.25f, 0.965, 0.25f);
		//Vec3 lightP1 = Vec3(0.25f, 0.965, -0.25f);
		//Vec3 lightP2 = Vec3(-0.25f, 0.965, -0.25f);
		//Vec3 lightP3 = Vec3(-0.25f, 0.965, 0.25f);
		Vec3 lightP0 = Vec3(0.15f, 0.965, 0.15f);
		Vec3 lightP1 = Vec3(0.15f, 0.965, -0.15f);
		Vec3 lightP2 = Vec3(-0.15f, 0.965, -0.15f);
		Vec3 lightP3 = Vec3(-0.15f, 0.965, 0.15f);
		Vec3 lightNormal = Vec3(0, -1, 0);
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightTriangleShape0 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP1, lightP2, lightNormal));
		std::shared_ptr<Shape> lightTriangleShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP2, lightP3, lightNormal));
		//std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 2000000));
		//std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 2000000));
		std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 350));
		std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 350));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> triangleLightPrimitive0 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, triangleLight0));
		std::shared_ptr<Primitive> triangleLightPrimitive1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, triangleLight1));
		scene->AddLight(triangleLight0);
		scene->AddLight(triangleLight1);
		scene->AddPrimitive(triangleLightPrimitive0);
		scene->AddPrimitive(triangleLightPrimitive1);

	}
};





class CornellBoxHeartSurface {
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p1, p2, normalLeft));
		std::shared_ptr<Shape> leftShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p2, p3, p0, normalLeft));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p5, p6, normalRight));
		std::shared_ptr<Shape> rightShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p6, p7, normalRight));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p5, p6, normalBack));
		std::shared_ptr<Shape> backShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p6, p2, normalBack));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom));
		std::shared_ptr<Shape> botomShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom));
		std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botomTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		std::shared_ptr<Primitive> botomTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		scene->AddPrimitive(botomTriangle1);
		scene->AddPrimitive(botomTriangle2);


		//Top
		Vec3 normalTop = Vec3(0, -1, 0);
		std::shared_ptr<Shape> topShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p7, p6, normalTop));
		std::shared_ptr<Shape> topShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p6, p2, normalTop));
		std::shared_ptr<Material> topWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> topTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape1, topWallMaterial));
		std::shared_ptr<Primitive> topTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape2, topWallMaterial));
		scene->AddPrimitive(topTriangle1);
		scene->AddPrimitive(topTriangle2);


		//Diffuse Ball
		//static Transform o2w = 
		//	Transform::Translate(Vec3(-0.5f, -0.6f, -0.3f)) * Transform::RotateY(220) * Transform::RotateX(90) * Transform::Scale(-1, 1, 1);
		//static Transform o2w =
		//	Transform::Translate(Vec3(-0.5f, -0.6f, -0.3f)) * Transform::RotateY(220) * Transform::Scale(1, 1, 1);
		//static Transform w2o = Inverse(o2w);
		//std::shared_ptr<Texture<Vec3>> imageTexture1 =
		//	std::shared_ptr<Texture<Vec3>>(new ImageTexture<Vec3>("..\\resources\\texture_images\\sphere_mapping_8k.jpg"));
		//std::shared_ptr<Shape> diffuseBallShape = std::shared_ptr<Shape>(new Sphere(&o2w, &w2o, 0.35f, Vec3(0, 0, 0)));
		//std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(imageTexture1));
		//std::shared_ptr<Primitive> diffuseBall = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape, diffuseBallMaterial));
		//scene->AddPrimitive(diffuseBall);

		//Heart Surface
		std::shared_ptr<Material> glassHeartSurfaceMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Material> diffuseHeartSurfaceMaterial = std::shared_ptr<Material>(new DiffuseMaterial(redConstant));
		//static Transform o2w = Transform::Translate(Vec3(-0.3, -0.3, -0.3)) * Transform::Scale(0.35, 0.35, 0.35);
		static Transform o2w = Transform::Translate(Vec3(-0.3, -0.3, -0.3)) * Transform::Scale(0.35, 0.35, 0.35);
		static Transform w2o = Inverse(o2w);
		std::shared_ptr<Shape> heartSurfaceShape = std::shared_ptr<Shape>(new HeartSurface(o2w, w2o));
		//std::shared_ptr<Shape> heartSurfaceShape = std::shared_ptr<Shape>(new Sphere(&o2w, &w2o, 1, Vec3()));
		std::shared_ptr<Primitive> heartSurface = std::shared_ptr<Primitive>(new Primitive(heartSurfaceShape, glassHeartSurfaceMaterial));
		scene->AddPrimitive(heartSurface);

		//Mesh
		//std::shared_ptr<Mesh> meshBunny = std::shared_ptr<Mesh>(new Mesh());
		//meshBunny->LoadFromFile("..\\resources\\meshs\\bunny3.obj");
		//std::shared_ptr<Material> meshMaterial = std::shared_ptr<Material>(new DiffuseMaterial(redConstant));
		//meshBunny->SetMaterial(meshMaterial);
		//Transform transformBunny = Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(-0.28, 0.28, 0.28);
		//scene->AddMesh(*meshBunny, transformBunny);

		//Light
		Vec3 lightP0 = Vec3(0.25f, 0.965, 0.25f);
		Vec3 lightP1 = Vec3(0.25f, 0.965, -0.25f);
		Vec3 lightP2 = Vec3(-0.25f, 0.965, -0.25f);
		Vec3 lightP3 = Vec3(-0.25f, 0.965, 0.25f);
		Vec3 lightNormal = Vec3(0, -1, 0);
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightTriangleShape0 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP1, lightP2, lightNormal));
		std::shared_ptr<Shape> lightTriangleShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP2, lightP3, lightNormal));
		std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 100));
		std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 100));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> triangleLightPrimitive0 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, triangleLight0));
		std::shared_ptr<Primitive> triangleLightPrimitive1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, triangleLight1));
		scene->AddLight(triangleLight0);
		scene->AddLight(triangleLight1);
		scene->AddPrimitive(triangleLightPrimitive0);
		scene->AddPrimitive(triangleLightPrimitive1);
	}
};




class HeartSurfaceEnvironmentMapScene {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.40f, .40f, .40f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));
		std::shared_ptr<Texture<Vec3>> purpleConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0.9999f, 0.75f, 0.9999f)));

		std::shared_ptr<Texture<Vec3>> objWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.5f, .5f, .5f)));

		std::shared_ptr<Material> glassMeshMaterial = std::shared_ptr<Material>(new GlassMaterial(purpleConstant, purpleConstant));
		std::shared_ptr<Material> diffuseMeshMaterial = std::shared_ptr<Material>(new DiffuseMaterial(objWhiteConstant));
		std::shared_ptr<Material> mirrorMeshMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		std::shared_ptr<Texture<Vec3>> eta = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<Vec3>> k = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Material> roughMeshMaterial =
			std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		std::shared_ptr<Material> waterMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant, 1.f, 1.33));

		std::shared_ptr<Texture<Vec3>> kd = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0.75, 0.55, 0.75)));
		std::shared_ptr<Texture<Vec3>> ks = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0.05, 0.05, 0.05)));
		std::shared_ptr<Texture<real>> roughness = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.1));
		std::shared_ptr<Material> plasticMaterial = std::shared_ptr<Material>(new PlasticMaterial(kd, ks, roughness));

		std::shared_ptr<Texture<Vec3>> kr = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.0, 1.0, 1.0)));
		std::shared_ptr<Texture<Vec3>> kt = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.0, 1.0, 1.0)));
		std::shared_ptr<Texture<real>> xRoughness = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Texture<real>> yRoughness = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Texture<real>> index = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(1.5));
		std::shared_ptr<Material> roughDielectricMaterial = std::shared_ptr<Material>(new RoughDielectricMaterial(kr, kt, xRoughness, yRoughness, index));

		//Points
		Vec3 p0(-1, -1, 1);
		Vec3 p1(-1, -1, -1);
		Vec3 p2(-1, 1, -1);
		Vec3 p3(-1, 1, 1);
		Vec3 p4(1, -1, 1);
		Vec3 p5(1, -1, -1);
		Vec3 p6(1, 1, -1);
		Vec3 p7(1, 1, 1);


		//Botom
		//Vec3 normalBotom = Vec3(0, 1, 0);
		//std::shared_ptr<Shape> botomShape1 =
		//	std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom));
		//std::shared_ptr<Shape> botomShape2 =
		//	std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom));
		//std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		//std::shared_ptr<Primitive> botomTriangle1 =
		//	std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		//std::shared_ptr<Primitive> botomTriangle2 =
		//	std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		//scene->AddPrimitive(botomTriangle1);
		//scene->AddPrimitive(botomTriangle2);

		//Heart Surface
		std::shared_ptr<Material> glassHeartSurfaceMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Material> diffuseHeartSurfaceMaterial = std::shared_ptr<Material>(new DiffuseMaterial(redConstant));
		//static Transform o2w = Transform::Translate(Vec3(-0.3, -0.3, -0.3)) * Transform::Scale(0.35, 0.35, 0.35);
		static Transform o2w1 = Transform::Translate(Vec3(-0.6, -0.3, -0.3)) * Transform::Scale(0.35, 0.35, 0.35);
		static Transform w2o1 = Inverse(o2w1);
		std::shared_ptr<Shape> heartSurfaceShape1 = std::shared_ptr<Shape>(new HeartSurface(o2w1, w2o1));
		std::shared_ptr<Primitive> heartSurface1 = std::shared_ptr<Primitive>(new Primitive(heartSurfaceShape1, diffuseHeartSurfaceMaterial));
		scene->AddPrimitive(heartSurface1);


		//Heart Surface
		//static Transform o2w2 = Transform::Translate(Vec3(0.5, -0.3, -0.3)) * Transform::Scale(0.35, 0.35, 0.35);
		//static Transform w2o2 = Inverse(o2w2);
		//std::shared_ptr<Shape> heartSurfaceShape2 = std::shared_ptr<Shape>(new HeartSurface(&o2w2, &w2o2));
		//std::shared_ptr<Primitive> heartSurface2 = std::shared_ptr<Primitive>(new Primitive(heartSurfaceShape2, glassHeartSurfaceMaterial));
		//scene->AddPrimitive(heartSurface2);

		//Light
		std::shared_ptr<Light> light = std::shared_ptr<Light>(new EnvironmentLight("..\\resources\\media\\pictures\\uffizi-large.hdr", 0));
		//std::shared_ptr<Light> light = std::shared_ptr<Light>(new EnvironmentLight("..\\resources\\media\\pictures\\grace-new.hdr", 0));
		scene->AddLight(light);

	}
};


class EnvironmentMapScene {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.40f, .40f, .40f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));
		std::shared_ptr<Texture<Vec3>> purpleConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0.9999f, 0.75f, 0.9999f)));

		std::shared_ptr<Texture<Vec3>> objWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.5f, .5f, .5f)));

		std::shared_ptr<Material> glassMeshMaterial = std::shared_ptr<Material>(new GlassMaterial(purpleConstant, purpleConstant));
		std::shared_ptr<Material> diffuseMeshMaterial = std::shared_ptr<Material>(new DiffuseMaterial(objWhiteConstant));
		std::shared_ptr<Material> mirrorMeshMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		std::shared_ptr<Texture<Vec3>> eta = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<Vec3>> k = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Material> roughMeshMaterial =
			std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		std::shared_ptr<Material> waterMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant, 1.f, 1.33));

		std::shared_ptr<Texture<Vec3>> kd = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0.75, 0.55, 0.75)));
		std::shared_ptr<Texture<Vec3>> ks = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0.05, 0.05, 0.05)));
		std::shared_ptr<Texture<real>> roughness = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.1));
		std::shared_ptr<Material> plasticMaterial = std::shared_ptr<Material>(new PlasticMaterial(kd, ks, roughness));

		std::shared_ptr<Texture<Vec3>> kr = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.0, 1.0, 1.0)));
		std::shared_ptr<Texture<Vec3>> kt = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.0, 1.0, 1.0)));
		std::shared_ptr<Texture<real>> xRoughness = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Texture<real>> yRoughness = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Texture<real>> index = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(1.5));
		std::shared_ptr<Material> roughDielectricMaterial = std::shared_ptr<Material>(new RoughDielectricMaterial(kr, kt, xRoughness, yRoughness, index));

		//Points
		Vec3 p0(-1, -1, 1);
		Vec3 p1(-1, -1, -1);
		Vec3 p2(-1, 1, -1);
		Vec3 p3(-1, 1, 1);
		Vec3 p4(1, -1, 1);
		Vec3 p5(1, -1, -1);
		Vec3 p6(1, 1, -1);
		Vec3 p7(1, 1, 1);


		//Botom
		Vec3 normalBotom = Vec3(0, 1, 0);
		std::shared_ptr<Shape> botomShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom));
		std::shared_ptr<Shape> botomShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom));
		std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botomTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		std::shared_ptr<Primitive> botomTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		scene->AddPrimitive(botomTriangle1);
		scene->AddPrimitive(botomTriangle2);


		//Diffuse Ball
		//std::shared_ptr<Shape> diffuseBallShape = std::shared_ptr<Shape>(new Sphere(Transform(), Transform(), 0.35f, Vec3(0.f, -0.4f, 0.f)));
		//std::shared_ptr<Material> diffuseBallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		//std::shared_ptr<Primitive> diffuseBall = std::shared_ptr<Primitive>(new GeometryPrimitive(diffuseBallShape, roughDielectricMaterial));
		//scene->AddPrimitive(diffuseBall);

		//std::shared_ptr<Mesh> meshVenus = std::shared_ptr<Mesh>(new Mesh());
		//meshVenus->LoadFromFile("..\\resources\\meshs\\venusm.obj");
		//meshVenus->SetMaterial(roughDielectricMaterial);
		//Transform transformVenus = Transform::Translate(Vec3(-0.4, -0.69, 0)) * Transform::Scale(0.00035, 0.00035, 0.00035);
		//scene->AddMesh(*meshVenus, transformVenus);

		std::shared_ptr<Mesh> meshBunny = std::shared_ptr<Mesh>(new Mesh());
		meshBunny->LoadFromFile("..\\resources\\meshs\\bunny3.obj");
		meshBunny->SetMaterial(roughMeshMaterial);
		//Transform transformBunny = Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(-0.28, 0.28, 0.28);
		Transform transformBunny = Transform::Translate(Vec3(0.52, -0.92, 0.3)) * Transform::Scale(0.22, 0.22, 0.22);
		scene->AddMesh(*meshBunny, transformBunny);

		std::shared_ptr<Mesh> objMesh = std::shared_ptr<Mesh>(new Mesh());
		objMesh->LoadFromFile("..\\resources\\meshs\\Alucy.obj");
		//Transform transformObj =
		//	Transform::Translate(Vec3(0, -1, 0)) *
		//	Transform::RotateY(60) * 
		//	Transform::Scale(0.0019, 0.0019, 0.0019);
		Transform transformObj =
			Transform::Translate(Vec3(-0.4, 0, 0)) *
			Transform::Translate(Vec3(0, -1, 0)) *
			Transform::RotateY(15) *
			Transform::RotateY(60) *
			Transform::Scale(0.0019, 0.0019, 0.0019);
		objMesh->SetMaterial(roughDielectricMaterial);
		scene->AddMesh(*objMesh, transformObj);

		//Light
		std::shared_ptr<Light> light = std::shared_ptr<Light>(new EnvironmentLight("..\\resources\\media\\pictures\\uffizi-large.hdr", 0));
		//std::shared_ptr<Light> light = std::shared_ptr<Light>(new EnvironmentLight("..\\resources\\media\\pictures\\grace-new.hdr", 0));
		scene->AddLight(light);

		//////Light
		//Vec3 lightP0 = Vec3(0.25f, 0.965, 0.25f);
		//Vec3 lightP1 = Vec3(0.25f, 0.965, -0.25f);
		//Vec3 lightP2 = Vec3(-0.25f, 0.965, -0.25f);
		//Vec3 lightP3 = Vec3(-0.25f, 0.965, 0.25f);

		//////Vec3 lightP0 = Vec3(-0.965, 0.25f, 0.25f);
		//////Vec3 lightP1 = Vec3(-0.965, 0.25f, -0.25f);
		//////Vec3 lightP2 = Vec3(-0.965 , -0.25f, -0.25f);
		//////Vec3 lightP3 = Vec3(-0.965 , -0.25f, 0.25f);
		//Vec3 lightNormal = Vec3(0, -1, 0);
		//std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		//std::shared_ptr<Shape> lightTriangleShape0 =
		//	std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP1, lightP2, lightNormal));
		//std::shared_ptr<Shape> lightTriangleShape1 =
		//	std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP2, lightP3, lightNormal));
		//std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 100));
		//std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 100));
		//std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		//std::shared_ptr<Primitive> triangleLightPrimitive0 =
		//	std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, triangleLight0));
		//std::shared_ptr<Primitive> triangleLightPrimitive1 =
		//	std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, triangleLight1));
		//scene->AddLight(triangleLightPrimitive0);
		//scene->AddLight(triangleLightPrimitive1);
	}
};



class CornellBoxMedium {
public:
	static void SetScene(std::shared_ptr<Scene>& scene) {
		//texture
		std::shared_ptr<Texture<Vec3>> redConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .25f, .25f)));
		std::shared_ptr<Texture<Vec3>> blueConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.25f, .25f, .75f)));
		std::shared_ptr<Texture<Vec3>> whiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.75f, .75f, .75f)));
		std::shared_ptr<Texture<Vec3>> fullWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1, 1, 1)));


		std::shared_ptr<Texture<Vec3>> objWhiteConstant = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(.5f, .5f, .5f)));

		std::shared_ptr<Material> glassMeshMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant));
		std::shared_ptr<Material> diffuseMeshMaterial = std::shared_ptr<Material>(new DiffuseMaterial(objWhiteConstant));
		std::shared_ptr<Material> mirrorMeshMaterial = std::shared_ptr<Material>(new MirrorMaterial(fullWhiteConstant));
		std::shared_ptr<Texture<Vec3>> eta = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<Vec3>> k = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(1.5, 1.5, 1.5)));
		std::shared_ptr<Texture<real>> roughnessx = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.06));
		std::shared_ptr<Texture<real>> roughnessy = std::shared_ptr<Texture<real>>(new ConstantTexture<real>(0.02));
		std::shared_ptr<Material> roughMeshMaterial =
			std::shared_ptr<Material>(new RoughnessMaterial(fullWhiteConstant, roughnessx, roughnessy, eta, k));
		std::shared_ptr<Material> waterMaterial = std::shared_ptr<Material>(new GlassMaterial(fullWhiteConstant, fullWhiteConstant, 1.f, 1.33));

		//Homogenoug media
		Vec3 sigma_a(0, 0, 0), sigma_s(1, 1, 1);
		Medium* homogenoursMedium = new HomogeneousMedium(sigma_a, sigma_s, 0);
		MediumInterface mediaInterface(homogenoursMedium, nullptr);

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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p1, p2, normalLeft));
		std::shared_ptr<Shape> leftShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p2, p3, p0, normalLeft));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p5, p6, normalRight));
		std::shared_ptr<Shape> rightShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p4, p6, p7, normalRight));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p5, p6, normalBack));
		std::shared_ptr<Shape> backShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p1, p6, p2, normalBack));
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
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p5, normalBotom));
		std::shared_ptr<Shape> botomShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p5, p1, normalBotom));
		std::shared_ptr<Material> botomWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> botomTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape1, botomWallMaterial));
		std::shared_ptr<Primitive> botomTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(botomShape2, botomWallMaterial));
		scene->AddPrimitive(botomTriangle1);
		scene->AddPrimitive(botomTriangle2);

		//Top
		Vec3 normalTop = Vec3(0, -1, 0);
		std::shared_ptr<Shape> topShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p7, p6, normalTop));
		std::shared_ptr<Shape> topShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p3, p6, p2, normalTop));
		std::shared_ptr<Material> topWallMaterial = std::shared_ptr<Material>(new DiffuseMaterial(whiteConstant));
		std::shared_ptr<Primitive> topTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape1, topWallMaterial));
		std::shared_ptr<Primitive> topTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(topShape2, topWallMaterial));
		scene->AddPrimitive(topTriangle1);
		scene->AddPrimitive(topTriangle2);


		//Front
		Vec3 normalFront = Vec3(0, 0, 1);
		std::shared_ptr<Shape> frontShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p4, p7, normalFront));
		std::shared_ptr<Shape> frontShape2 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), p0, p7, p3, normalFront));
		std::shared_ptr<Primitive> frontTriangle1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(frontShape1, nullptr, nullptr, mediaInterface));
		std::shared_ptr<Primitive> frontTriangle2 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(frontShape2, nullptr, nullptr, mediaInterface));
		scene->AddPrimitive(frontTriangle1);
		scene->AddPrimitive(frontTriangle2);


		std::shared_ptr<Mesh> mesh = std::shared_ptr<Mesh>(new Mesh());
		mesh->LoadFromFile("..\\resources\\meshs\\bunny3.obj");
		mesh->SetMaterial(diffuseMeshMaterial);
		Transform transform = Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(0.28, 0.28, 0.28);
		scene->AddMesh(*mesh, transform);


		//Cube
		//std::shared_ptr<Mesh> cubeMesh = std::shared_ptr<Mesh>(new Mesh());
		//cubeMesh->LoadFromFile("..\\resources\\meshs\\cube.obj");
		//Transform cubeTransform =
		//	Transform::Scale(0.6, 0.6, 0.6) *
		//	Transform::Translate(Vec3(-0.5, -0.5, -0.5));
		////cubeMesh->SetMaterial(diffuseMeshMaterial);
		//cubeMesh->SetMedium(mediaInterface);
		//scene->AddMesh(*cubeMesh, cubeTransform);



		//Light
		Vec3 lightP0 = Vec3(0.25f, 0.965, 0.25f);
		Vec3 lightP1 = Vec3(0.25f, 0.965, -0.25f);
		Vec3 lightP2 = Vec3(-0.25f, 0.965, -0.25f);
		Vec3 lightP3 = Vec3(-0.25f, 0.965, 0.25f);
		//Vec3 lightP0 = Vec3(0.0015f, 0.965, 0.0015f);
		//Vec3 lightP1 = Vec3(0.0015f, 0.965, -0.0015f);
		//Vec3 lightP2 = Vec3(-0.0015f, 0.965, -0.0015f);
		//Vec3 lightP3 = Vec3(-0.0015f, 0.965, 0.0015f);
		Vec3 lightNormal = Vec3(0, -1, 0);
		std::shared_ptr<Texture<Vec3>> lightTexture = std::shared_ptr<Texture<Vec3>>(new ConstantTexture<Vec3>(Vec3(0, 0, 0)));
		std::shared_ptr<Shape> lightTriangleShape0 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP1, lightP2, lightNormal));
		std::shared_ptr<Shape> lightTriangleShape1 =
			std::shared_ptr<Shape>(new Triangle(Transform(), Transform(), lightP0, lightP2, lightP3, lightNormal));
		//std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 2000000));
		//std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 2000000));
		std::shared_ptr<Light> triangleLight0 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape0, Vec3(0.3f, 0.3f, 0.3f) * 85));
		std::shared_ptr<Light> triangleLight1 = std::shared_ptr<Light>(new AreaLight(lightTriangleShape1, Vec3(0.3f, 0.3f, 0.3f) * 85));
		std::shared_ptr<Material> lightMaterial = std::shared_ptr<Material>(new DiffuseMaterial(lightTexture));
		std::shared_ptr<Primitive> triangleLightPrimitive0 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape0, lightMaterial, triangleLight0));
		std::shared_ptr<Primitive> triangleLightPrimitive1 =
			std::shared_ptr<Primitive>(new GeometryPrimitive(lightTriangleShape1, lightMaterial, triangleLight1));

		scene->AddLight(triangleLight0);
		scene->AddLight(triangleLight1);
		scene->AddPrimitive(triangleLightPrimitive0);
		scene->AddPrimitive(triangleLightPrimitive1);



		//PointLight
		//Vec3 intensity(2.5, 2.5, 2.5);
		//Vec3 pLight(0, 0.965, 0);
		//std::shared_ptr<Light> light = std::shared_ptr<Light>(new PointLight(pLight, intensity));
		//scene->AddLight(light);
	}
};
