//
//Things defined until now:
//  RayCast function + Intersection
//  ColorAt function
//  Reflectivity
//  Not good: Sensor type (an array of 0's to do thee shading - RGB in future) - The image
//
//  types for world
//  Camera 
//  Sphere / Plane(not used yet)
//  Light
//  Ray casting bucle
//
//  Carefull with Fatt funtion, used as 8/dist because light is color, not intensity!!!
open System
open System.Drawing

open System.IO
//open MathNet.Numerics
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open ObjReader
open RayType
open RayCore
open RayColor





//Definitions of values
//
// Pixels
let PixNumW = 100
let PixNumH = 100
let PixWide = 2.0/float(PixNumW) //Write something
let PixHeigh = 2.0/float(PixNumH)
//
//Camera: observer and sensor
//let sensor = Sensor(PixWide, PixHeigh, PixNumW, PixNumH) 
 
//Scenario1
let camera={EyePoint=Point3D(-2.5,0.0,0.0);LookAt=Vector3D(2.5,0.0,0.0); Up=Vector3D(0.0,0.0,1.0)}

let light = {origin = Point3D(2.0,0.750,2.50);color=Color(1.0,1.0,1.0); intensity = 30.0}
let light2 = {origin = Point3D(0.00,-2.0,-1.50);color=Color(1.0,1.0,1.0); intensity = 100.0}
let light3 = {origin = Point3D(4.50,0.0,1.0);color=Color(1.0,1.0,1.0); intensity = 70.0}
let light4 = {origin = Point3D(-1.00,2.50,5.0);color=Color(1.0,1.0,1.0); intensity = 90.0}

//
// scene
// Materials
let refractive= {DiffuseLight = Color(0.001,0.001,0.0175);SpecularLight = Color(0.95,0.95,0.99);shinness= 60; R=0.01; T=0.99; n= 1.95} 
let reflective ={DiffuseLight = Color(0.25,0.490,0.25);SpecularLight = Color(0.9,0.9,0.9);shinness= 50; R=0.950; T=0.0; n= 1.45} 
let difus = {DiffuseLight = Color(0.25,0.90,0.5);SpecularLight = Color(0.5,0.5,0.9);shinness= 80; R=1.0; T=0.0; n= 1.45}
let whitte ={DiffuseLight = Color(0.9,0.90,0.9);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.050; T=0.0; n= 1.45}
// I define the spheres and meshes

//Spheres
let ball = {center=Point3D(0.70,0.20,0.0); radius=0.250; material=refractive }
let ball2 = {center=Point3D(10.0,-1.50,1.0); radius=1.965; material=reflective }
//let ball3 = {center = Point3D(3.8125, 0.38, 1.5); radius = 1. ; material= difus}
// meshes
let path = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\humanoid_tri.obj"
let path2 = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\plane.obj"
//pyramid.obj" 
//humanoid_tri.obj" 
//gourd.obj"/
//let path = "MeshSamples/humanoid_tri.obj"
let mesh1= ReadMeshWavefront(path,difus) |> fun x -> Scale x [0.25;0.25;0.25]|> fun x -> Translate x (Vector3D(3.50,1.0,-2.0))
let mesh2 = ReadMeshWavefront(path2,whitte)
//printfn "%+A" mesh1.Vertices
let all = {Meshes = [mesh1;mesh2];Sphere = [ball;ball2]}
let Scene = {Camera=camera ;World = all; Light=[light;light2;light3;light4]} 
//ball;ball2;ball3;ball4;ball5
// Scenario2


//Viewing Coordinate System w u v
let w = camera.LookAt.Normalize()
let u = camera.Up.CrossProduct(w).Normalize()
let v = w.CrossProduct(u)
// Bucle
let bmp = new Bitmap(PixNumW,PixNumH)

//Backward ray tracing, from cam to light
for i in 0..(PixNumW-1) do 
    for j in 0..(PixNumH-1) do
            // create a ray from eye to pixel
            let direct = camera.LookAt+u.ScaleBy(PixWide*float(PixNumW/2-i))+v.ScaleBy(PixHeigh*float(PixNumH/2-j)) // Real to where it points
            let Ray = {uvec=direct.Normalize(); length=infinity; from=camera.EyePoint; travelled=0.0 } //BAD
            //let intersects = castRay (Scene, Ray)
            let intersects = CastRay_nest (Scene, Ray)
            match intersects with
            | [] -> bmp.SetPixel(i,j,Color.Gray)
            | _ ->  let color = GlobalIllum(intersects |>List.minBy(fun x -> x.t), Scene  )//colorAt (intersects |>List.minBy(fun x -> x.t), Scene  )
                     //(fun x -> x) |> |> List.head S
                    //printfn "Color is: %+A" color
                    bmp.SetPixel(i,j,Color.FromArgb(int(255.0*color.r),  int(255.0*color.g) , int(255.0*color.b) ))

(*
let form = new Form(Text="Rendering test",TopMost=true)
form.Show()

let img = new PictureBox(Dock=DockStyle.Fill)
form.Controls.Add(img)

img.Image <- bmp *)
//bmp.Save(@"C:\Users\JoseM\Desktop\test_all.jpg")



bmp.Save(@"C:\Users\JoseM\Desktop\test_2r.jpg")