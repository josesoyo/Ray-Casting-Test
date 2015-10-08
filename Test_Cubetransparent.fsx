// Test in which there are spheres and meshes

#r @"packages\MathNet.Spatial.0.2.0-alpha\lib\net40\MathNet.Spatial.dll"
#r @"packages\MathNet.Numerics.Signed.3.7.0\lib\net40\MathNet.Numerics.dll"
#r @"packages\MathNet.Numerics.FSharp.Signed.3.7.0\lib\net40\MathNet.Numerics.FSharp.dll"

#load "RayType.fs"
#load "RayCore.fs"
#load "RayColor.fs"
#load "ObjReader.fs"


open System.IO
open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open ObjReader
open RayType
open RayCore
open RayColor



//Definitions of values
//
// Pixels
let PixNumW = 250
let PixNumH = 250
let PixWide = 2.0/float(PixNumW) //Write something
let PixHeigh = 2.0/float(PixNumH)
//
//Camera: observer and sensor
//let sensor = Sensor(PixWide, PixHeigh, PixNumW, PixNumH) 
 
//Scenario1
let camera={EyePoint=Point3D(-5.,0.0,0.0);LookAt=Vector3D(5.,0.0,0.0); Up=Vector3D(0.0,0.0,1.0)}

let light = {origin = Point3D(2.0,-0.750,2.50);color=Color(1.0,1.0,1.0); intensity = 0.50}
let light1 = {origin = Point3D(8.2,2.0,3.0);color=Color(1.0,1.0,1.0); intensity = 0.50}
let light2 = {origin = Point3D(0.00,-2.0,-1.50);color=Color(1.0,1.0,1.0); intensity = 1.0}
let light3 = {origin = Point3D(4.50,0.0,1.0);color=Color(1.0,1.0,1.0); intensity = 0.60}
let light4 = {origin = Point3D(-1.00,2.50,5.0);color=Color(1.0,1.0,1.0); intensity = 0.60}

//
// scene
// Materials
let refractive= {DiffuseLight = Color(0.001,0.001,0.0075);SpecularLight = Color(0.95,0.95,0.99);shinness= 60; R=0.01; T=0.99; n= 1.65;Fresnel=true} 
let reflective ={DiffuseLight = Color(0.25,0.490,0.25);SpecularLight = Color(0.9,0.9,0.9);shinness= 50; R=0.950; T=0.0; n= 1.45;Fresnel = false} 
let difus_human = {DiffuseLight = Color(0.5,0.850,0.1);SpecularLight = Color(0.5,0.5,0.9);shinness= 80; R=1.0; T=0.0; n= 1.3245;Fresnel = false}
let whitte ={DiffuseLight = Color(0.9,0.90,0.9);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=1.0; T=0.0; n= 1.45;Fresnel = false}
// I define the spheres and meshes

//Spheres
let ball = {center=Point3D(1.0,0.10,-0.50); radius=0.250; material=refractive } //0.7,0.2,0.0
let ball2 = {center=Point3D(10.0,-1.50,1.0); radius=1.965; material=reflective }
//let ball3 = {center = Point3D(3.8125, 0.38, 1.5); radius = 1. ; material= difus}
// meshes
let path = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\cube0.obj"
let path2 = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\plane.obj"
//pyramid.obj" 
//humanoid_tri.obj" 
//gourd.obj"/

//let mesh1= ReadMeshWavefront(path,reflective) |> fun x -> Scale x [1.25;1.25;1.25]|> fun x -> Translate x (Vector3D(1.50,0.50,0.150))
let mes31= ReadMeshWavefront(path,refractive) |> fun x -> Scale x [1.25;2.25;2.25]|> fun x -> Translate x (Vector3D(7.50,0.850,1.0))
let mesh2 = ReadMeshWavefront(path2,difus_human)
//printfn "%+A" mesh1.Vertices
let all = {Meshes = [mesh2;mes31];Sphere = [ball;ball2]}
let Scene = {Camera=camera ;World = all; Light=[light;light1;light2;light3;light4]} 
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


let form = new Form(Text="Rendering test",TopMost=true)
form.Show()

let img = new PictureBox(Dock=DockStyle.Fill)
form.Controls.Add(img)

img.Image <- bmp
printfn "end"
//bmp.Save(@"C:\Users\JoseM\Desktop\test_all_fresnel.jpg")
mes31.Vertices.Head