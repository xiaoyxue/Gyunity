from gyscene import Camera, Diffuse, Quad, RenderSettings, Scene, Sphere

scene = Scene(
    name="NativeDemo",
    camera=Camera.look_at((0., 1.3, 5.), (0., 0.5, 0.), fov_y=45.),
    render=RenderSettings(width=512, height=384, spp=64),
)
scene.add_material("white", Diffuse(reflectance=(0.7, 0.7, 0.7)))
scene.add_material("red", Diffuse(reflectance=(0.6, 0.1, 0.05)))
scene.add_material("black", Diffuse(reflectance=(0., 0., 0.)))
scene.add(Quad(id="floor", vertices=((-2., -0.2, 2.), (2., -0.2, 2.), (2., -0.2, -2.), (-2., -0.2, -2.)), material="white"))
scene.add(Sphere(id="sphere", center=(0.3, 0.4, 0.), radius=0.6, material="red"))
scene.add(Quad(id="light", vertices=((-0.4, 2., -0.4), (0.4, 2., -0.4), (0.4, 2., 0.4), (-0.4, 2., 0.4)),
               material="black", emission=(8., 8., 8.)))
