// Test in which there are spheres and meshes
// Done now at 4-September-2015


#r @"packages\MathNet.Spatial.0.2.0-alpha\lib\net40\MathNet.Spatial.dll"
#r @"packages\MathNet.Numerics.Signed.3.7.0\lib\net40\MathNet.Numerics.dll"
#r @"packages\MathNet.Numerics.FSharp.Signed.3.7.0\lib\net40\MathNet.Numerics.FSharp.dll"

#load "RayType.fs"
#load "BBox.fs"
#load "RayCore.fs"
#load "RandomMethods.fs"
#load "RayColor.fs"
#load "ObjReader.fs"


open System
open System.IO
open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml


//open BBox 

open ObjReader
open RayType
open RayCore
open RayColor

let PI = 3.141592653589


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
//let camera={EyePoint=Point3D(-10.50,10.50, 10.75);LookAt=Vector3D(2.20,-2.20,-2.20); Up=Vector3D(0.0,0.0,1.0)}  
let camera={EyePoint=Point3D(-2.0,0.0 ,1.0); LookAt=Vector3D(1.0,1.e-16,-1.0); Up=Vector3D(0.0,0.0,1.0)}  

let light = {origin =  Point3D(10.0,10.0,25.0);color=Color(1.0,1.0,1.0); intensity = 3000.0}
let light1 = {origin = Point3D(-10.0, -10.,25.0);color=Color(1.0,1.0,1.0); intensity = 1000.0}
let light2 = {origin = Point3D(-10.0,10.0,25.5);color=Color(1.0,1.0,1.0); intensity = 1000.0}
//let clight =  {Centre=Point3D(4.0,0.0,15.0);Normal=UnitVector3D(-0.750,0.0,-1.0);Radius=0.6;Area=PI; color=Color(1.0,1.0,0.90); intensityDensity=500.; RotationMatrix=Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),UnitVector3D(0.10,0.0,-1.0))}

let clight =  {Centre=Point3D(-0.0,0.0,5.0); Normal=UnitVector3D(-0.0,0.0,-1.0);
                Radius=0.50;Area=0.25*PI; 
               color=Color(1.0,1.0,0.950); 
                intensityDensity=200.;
                RotationMatrix=Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),UnitVector3D(0.0,0.0,-1.0))
                }
let clight1 =  {Centre=Point3D(4.0,0.0,0.0); Normal=UnitVector3D(-1.0,0.0,0.0);
                Radius=0.50;Area=0.25*PI; 
                color=Color(1.0,1.0,1.0); 
                intensityDensity=400.;
                RotationMatrix=Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),UnitVector3D(-1.0,0.0,0.0))
                }
let lights ={Point= [];Circle = [clight]}

//
// scene
// Materials
let refractive= {DiffuseLight = Color(0.005,0.005,0.0175);SpecularLight = Color(0.4,0.4,0.5);shinness= 100; R=0.01; T=0.99; n= 1.333;Fresnel=true} 
let refractive1= {DiffuseLight = Color(0.005,0.015,0.0175);SpecularLight = Color(0.4,0.4,0.5);shinness= 50; R=0.01; T=0.99; n= 1.35;Fresnel=true} 
let reflective ={DiffuseLight = Color(0.7,0.49,0.52);SpecularLight = Color(0.9,0.85,0.8);shinness= 50; R=0.750; T=0.0; n= 1.45;Fresnel = false} 
let reflective1 ={DiffuseLight = Color(0.005,0.123,0.12);SpecularLight = Color(0.013,0.12,0.2);shinness= 50; R=0.50; T=0.0; n= 1.75;Fresnel = false} 
let reflective2 ={DiffuseLight = Color(0.5,0.3,0.2);SpecularLight = Color(0.53,0.4,0.34);shinness= 50; R=0.350; T=0.0; n= 1.75;Fresnel = false} 
let whitte ={DiffuseLight = Color(0.9,0.90,0.7);SpecularLight = Color(0.51,0.51,0.591);shinness= 6; R=0.0950; T=0.0; n= 1.45;Fresnel = false}
// I define the spheres and meshes
// 139;90;43
//Spheres
let ball3 = {center = Point3D(0.0, 0.0, 0.0); radius = 0.50 ; material= refractive}
// meshes
let path = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\cube.obj"
let path2 = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\plane.obj"
//let path3 = @"C:\Users\JoseM\OneDrive\Phd\render\Wavefront obj FileFormat\teapot_resized.obj"

//pyramid.obj" 
//humanoid_tri.obj" 
//gourd.obj"/
(*
let mesh1= ReadMeshWavefront(path,refractive) |> fun x -> Translate x (Vector3D(2.50,1.0,0.0))|> Mesh_BBox 
let mesh2 = ReadMeshWavefront(path2,whitte) |> Mesh_BBox //|> fun x -> Scale x [0.25;0.25;0.25]|> fun x -> Translate x (Vector3D(3.50,1.0,-2.0))
let mesh3 = ReadMeshWavefront(path,reflective)|> fun x -> Translate x (Vector3D(-2.00,-2.0,1.0))|> Mesh_BBox 
//printfn "%+A" mesh1.Vertices
*)
let mesh0 = ReadMeshWavefront(path2,whitte) |> Mesh_BBox //|> fun x -> Scale x [0.25;0.25;0.25]|> fun x -> Translate x (Vector3D(3.50,1.0,-2.0))
let mesh1= ReadMeshWavefront(path,refractive) |> fun x -> Translate x (Vector3D(0.0,0.0,-0.0)) |> Mesh_BBox //
(*
let mesh1= ReadMeshWavefront(path,refractive1) |> fun x -> Translate x (Vector3D(3.5,0.0,-0.5))|> Mesh_BBox 
let mesh2= ReadMeshWavefront(path,refractive) |> fun x -> Translate x (Vector3D(1.0,0.0,2.5))|> Mesh_BBox 
let mesh3= ReadMeshWavefront(path,refractive) |> fun x -> Translate x (Vector3D(-1.5,0.0,-0.5))|> Mesh_BBox 
let mesh4= ReadMeshWavefront(path,refractive) |> fun x -> Translate x (Vector3D(3.5,-2.0,-0.5))|> Mesh_BBox 
let mesh5= ReadMeshWavefront(path,refractive1) |> fun x -> Translate x (Vector3D(1.0,-2.0,-0.5))|> Mesh_BBox 
let mesh6= ReadMeshWavefront(path,refractive) |> fun x -> Translate x (Vector3D(-1.50,-2.0,-0.5))|> Mesh_BBox 
let mesh7 = ReadMeshWavefront(path,reflective)|> fun x -> Translate x (Vector3D(-2.00,1.0,-1.0))|> Mesh_BBox 
let mesh8 = ReadMeshWavefront(path,reflective1)|> fun x -> Translate x (Vector3D(1.0,0.0,-0.5))|> Mesh_BBox 
let mesh9 = ReadMeshWavefront(path,reflective2)|> fun x -> Translate x (Vector3D(4.00,-0.0,1.50))|> Mesh_BBox 
*)
let all = {Meshes = [mesh0];Sphere = [ball3]}
//;mesh1;mesh2;mesh3;mesh4;mesh5;mesh6;mesh7;mesh8;mesh9
let Scene = {Camera=camera ;World = all; Light=lights;Nsamples = 10} 
//ball;ball2;ball3;ball4;ball5
// Scenario2


//Viewing Coordinate System w u v
let w = camera.LookAt.Normalize()
let u = camera.Up.CrossProduct(w).Normalize()
let v = w.CrossProduct(u)
// Bucle
let bmp = new Bitmap(PixNumW,PixNumH)

//Backward ray tracing, from cam to light
(*
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
    if i%25=0 then printfn "%i" i
*)

let p0 = [0..4..PixNumH-1]
let p1 = [1..4..PixNumH-1]
let p2 = [2..4..PixNumH-1]
let p3 = [3..4..PixNumH-1]
//Backward ray tracing, from cam to light
let casting (pixH:int list) =
    let mutable rimage = DenseMatrix.init PixNumW PixNumH (fun i j -> 0.0)
    let mutable gimage = DenseMatrix.init PixNumW PixNumH (fun i j -> 0.0)
    let mutable bimage = DenseMatrix.init PixNumW PixNumH (fun i j -> 0.0)
 
    for i in 0..(PixNumW-1) do 
        for j in pixH do
                // create a ray from eye to pixel
                let direct = camera.LookAt+u.ScaleBy(PixWide*float(PixNumW/2-i))+v.ScaleBy(PixHeigh*float(PixNumH/2-j)) // Real to where it points
                let Ray = {uvec= direct.Normalize(); length=infinity; from=camera.EyePoint; travelled=0.0 } //BAD UnitVector3D(0.0,0.0,-1.)  
                //let intersects = castRay (Scene, Ray)
                let intersects = CastRay_nest (Scene, Ray)
                match intersects with
                | [] -> rimage.[i,j] <- 0.0
                        gimage.[i,j] <- 0.0
                        bimage.[i,j] <- 0.0 //bmp.SetPixel(i,j,Color.Gray)
                | _ ->  let color = GlobalIllum(intersects |>List.minBy(fun x -> x.t), Scene  )//colorAt (intersects |>List.minBy(fun x -> x.t), Scene  )
                            //(fun x -> x) |> |> List.head S
                        //printfn "Color is: %+A" color
                        rimage.[i,j] <- color.b
                        gimage.[i,j] <- color.g 
                        bimage.[i,j] <- color.b
        if i%100=0 then printfn "%i" i
    [rimage;gimage;bimage]

let asyncasting pixW = async {return casting pixW}    // Prepare the parallel
#time
let mClr = [p0;p1;p2;p3] 
                  |> List.collect(fun x -> [asyncasting x])
                  |> Async.Parallel |> Async.RunSynchronously
                  |> Array.toList |> List.collect(fun x -> x)

let imager = mClr.[0] + mClr.[3] + mClr.[6] + mClr.[9]      // Red
let imageg = mClr.[1] + mClr.[4] + mClr.[7] + mClr.[10]     //Green
let imageb = mClr.[2] + mClr.[5] + mClr.[8] + mClr.[11]     //Blue
for i in 0..(PixNumW-1) do
    for j in 0..(PixNumW-1) do
        bmp.SetPixel(i,j,Color.FromArgb(int(255.0*imager.[i,j]),  int(255.0*imageg.[i,j]) , int(255.0*imageb.[i,j]) ))

#time    

bmp.Save(@"C:\Users\JoseM\Desktop\img_render\test_meshes_TransmisionMC01.jpg")

let form = new Form(Text="Rendering test",TopMost=true)
form.Show()

let img = new PictureBox(Dock=DockStyle.Fill)
form.Controls.Add(img)

img.Image <- bmp
printfn "end "