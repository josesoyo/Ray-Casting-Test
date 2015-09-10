// Test in which there are spheres and meshes

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



//Definitions of values
//
// Pixels
let PixNumW = 500
let PixNumH = 500
let PixWide = 2.0/float(PixNumW) //Write something
let PixHeigh = 2.0/float(PixNumH)
//
//Camera: observer and sensor
//let sensor = Sensor(PixWide, PixHeigh, PixNumW, PixNumH) 
 
//Scenario1
let camera={EyePoint=Point3D(10.50,-10.50, 10.75);LookAt=Vector3D(-2.20,2.20,-2.20); Up=Vector3D(0.0,0.0,1.0)}  

let light = {origin =  Point3D(10.0,10.0,25.0);color=Color(1.0,1.0,1.0); intensity = 3000.0}
let light1 = {origin = Point3D(-5.0, -10.,25.0);color=Color(1.0,1.0,1.0); intensity = 3000.0}
let light2 = {origin = Point3D(-10.0,10.0,25.5);color=Color(1.0,1.0,1.0); intensity = 3000.0}


//
// scene
// Materials
let refractive= {DiffuseLight = Color(0.005,0.005,0.0175);SpecularLight = Color(0.4,0.4,0.5);shinness= 100; R=0.01; T=0.99; n= 1.65;Fresnel=true} 
let reflective ={DiffuseLight = Color(0.6,0.40,0.2);SpecularLight = Color(0.9,0.85,0.8);shinness= 50; R=0.750; T=0.0; n= 1.45;Fresnel = false} 
let whitte ={DiffuseLight = Color(0.9,0.90,0.9);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.050; T=0.0; n= 1.45;Fresnel = false}
// I define the spheres and meshes
// 139;90;43
//Spheres
//let ball3 = {center = Point3D(3.8125, 0.38, 1.5); radius = 1. ; material= difus}
// meshes
let path = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\gourd.obj"
let path2 = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\plane.obj"
let path3 = @"C:\Users\JoseM\OneDrive\Phd\render\Wavefront obj FileFormat\teapot_resized.obj"

//pyramid.obj" 
//humanoid_tri.obj" 
//gourd.obj"/

let mesh1= ReadMeshWavefront(path,refractive)|> fun x -> Translate x (Vector3D(2.50,1.0,0.0)) |> Mesh_BBox //|> fun x -> Scale x [0.25;0.25;0.25]|> fun x -> Translate x (Vector3D(3.50,1.0,-2.0))
let mesh2 = ReadMeshWavefront(path2,whitte)|> fun x -> Translate x (Vector3D(-2.0,1.0,1.0)) |> Mesh_BBox
let mesh3 = ReadMeshWavefront(path3,whitte)|> fun x -> Translate x (Vector3D(-2.00,-2.0,0.0)) |> Mesh_BBox
//printfn "%+A" mesh1.Vertices
let all = {Meshes = [mesh1;mesh2;mesh3];Sphere = []}
let Scene = {Camera=camera ;World = all; Light=[light;light2]} 
//ball;ball2;ball3;ball4;ball5
// Scenario2


//Viewing Coordinate System w u v
let w = camera.LookAt.Normalize()
let u = camera.Up.CrossProduct(w).Normalize()
let v = w.CrossProduct(u)
// Bucle
let bmp = new Bitmap(PixNumW,PixNumH)

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
                let Ray = {uvec=direct.Normalize(); length=infinity; from=camera.EyePoint; travelled=0.0 } //BAD
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
        if i%20=0 then printfn "%i" i
    [rimage;gimage;bimage]

let asyncasting pixW = async {return casting pixW}    // Prepare the parallel
//#time // Only in scrip
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

//#time // Only in script    

bmp.Save(@"C:\Users\JoseM\Desktop\img_render\test_meshes_parallel.jpg")

let form = new Form(Text="Rendering test",TopMost=true)
form.Show()

let img = new PictureBox(Dock=DockStyle.Fill)
form.Controls.Add(img)

img.Image <- bmp
printfn "end"