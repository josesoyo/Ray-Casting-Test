
open System.IO
open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open ObjReader
open RayType
open RayTypeMethods // ????
open RayCore
open Octree           // Octree
open RayColorOctree   // Octree
open RayColorGrid     // 3DGrid
open PreprocesorGrid
open RayCoreOctree    // Octree
open RayCoreGrid      // 3DGrid

let PI = 3.141592653589


//Definitions of values
//
// Pixels
let PixNumW = 200
let PixNumH = 200
let PixWide = 2.0/float(PixNumW) //Write something
let PixHeigh = 2.0/float(PixNumH)
//
//Camera: observer and sensor
//let sensor = Sensor(PixWide, PixHeigh, PixNumW, PixNumH) 
 
//Scenario1
let camera={EyePoint=Point3D(-2.5,0.0,-0.250);LookAt=Vector3D(5.95,1.e-10,1.e-10); Up=Vector3D(0.0,0.0,1.0)}
//let camera={EyePoint=Point3D(-2.5,-2.5,0.0);LookAt=Vector3D(1.5,1.5,1.e-10); Up=Vector3D(0.0,0.0,1.0)}

let light = {origin = Point3D(2.0,0.750,3.0);color=Color(1.0,1.0,1.0); intensity = 50.0}
let light2 = {origin = Point3D(0.00,-2.0,-1.50);color=Color(1.0,1.0,1.0); intensity = 150.0}
let light3 = {origin = Point3D(4.50,0.0,1.0);color=Color(1.0,1.0,1.0); intensity = 190.0}
let light4 = {origin = Point3D(-1.00,2.50,5.0);color=Color(1.0,1.0,1.0); intensity = 99.0}
let light5= {origin = Point3D(11.7500,0.0,1.0);color=Color(1.0,1.0,1.0); intensity = 200.0}
(*
let clight = {Centre=Point3D(-0.0,-1.0,5.0); Normal=UnitVector3D(0.50,0.0,-1.0);
               Radius=2.0;Area=PI; 
               color=Color(1.0,1.0,0.950); 
               intensityDensity=100000.;
               RotationMatrix=Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),UnitVector3D(0.50,0.0,-1.0))
               }
*)
let clight = {Centre=Point3D(-1.10,-0.50,5.0); Normal=UnitVector3D(0.50,0.5,-1.0);
               Radius=2.0;Area=PI; 
               color=Color(1.0,1.0,0.950); 
               intensityDensity=100.;
               RotationMatrix=Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),UnitVector3D(0.50,0.0,-1.0))
               }
//
// scene
// Materials
let refractive= {DiffuseLight = Color(0.001,0.001,0.0175);SpecularLight = Color(0.795,0.795,0.799);shinness= 60; R=0.01; T=0.99; n= 1.95;Fresnel=true} 
let reflective ={DiffuseLight = Color(0.29,0.590,0.29);SpecularLight = Color(0.9,0.9,0.9);shinness= 50; R=0.250; T=0.0; n= 1.45;Fresnel = false} 
let difus_human = {DiffuseLight = Color(0.25,0.70,0.4);SpecularLight = Color(0.3,0.2,0.5);shinness= 80; R=0.80; T=0.0; n= 1.45;Fresnel = false}
let whitte ={DiffuseLight = Color(1.0,1.0,1.0);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.0450; T=0.0; n= 1.45;Fresnel = false}
// I define the spheres and meshes

//Spheres
let ball = {center=Point3D(1.0,0.20,0.0); radius=0.250; material=refractive } //0.7,0.2,0.0
let ball2 = {center=Point3D(10.0,-1.50,1.0); radius=1.965; material=reflective }
//let ball3 = {center = Point3D(3.8125, 0.38, 1.5); radius = 1. ; material= difus}
// meshes
//let path = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\humanoid_tri.obj"
let path = Path.Combine(__SOURCE_DIRECTORY__,"..\MeshSamples\humanoid_tri.obj")
//let path3 = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\gourd.obj"
let path3 =  Path.Combine(__SOURCE_DIRECTORY__,"..\MeshSamples\gourd.obj")
//let path2 = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\plane.obj"
let path2 = Path.Combine(__SOURCE_DIRECTORY__,"..\MeshSamples\plane.obj")
printfn "Source: %+A" __SOURCE_DIRECTORY__

//pyramid.obj" 
//humanoid_tri.obj" 
//gourd.obj"/
//difus_human ) |> fun x -> Translate x (Vector3D(0.0,0.0,-0.10)) |> Mesh_BBox//
let mesh1= ReadMeshWavefront(path,difus_human)|> fun x -> Scale x [0.25;0.25;0.25]|> fun x -> Translate x (Vector3D(3.50,1.0,-2.0))|> Mesh_BBox 

let nrmM15 = UnitVector3D(0.0,0.,-1.0)
let mesh15= ReadMeshWavefront(path,difus_human) |> fun x -> RotateMesh x nrmM15
            |> fun x -> Scale x [0.35;0.35;0.35]|> fun x -> Translate x (Vector3D(5.0,4.0,5.0))|> Mesh_BBox 
let mesh2 = ReadMeshWavefront(path2,whitte) |> Mesh_BBox 

let mesht = ReadMeshWavefront(path3,refractive)|> fun x -> Scale x [0.25;0.25;0.25] 
let mesh3 = [-1..5] |> List.collect(fun x ->( [-3..7]|>List.collect(fun y -> [(Translate mesht (Vector3D(0.6*float(x),0.7*float(y),-1.0))|> Mesh_BBox)]) ))
//mesht.Triangles.Length
//mesh3.Length
//648*77 

let all = {Meshes = [mesh2;mesh1]; //;mesh15 @mesh3
           Sphere = [ball;ball2 ];PartSphere= []; SurfaceLens = [] ;
           Cylinder =[]} // mesh2; -  ball;ball2

let lights ={Point= [light];Circle = [clight]} //light;light2;light3;light4;light5
//partition.
let pPartition = Partitionate (all, 1)
// Octree
//#time // In case of execute as a script
let partition =  OctreeSubdivision (all, WorldLimitsExtend (all),45,0,6)
//#time // In case of execute as a script
let Scene = {Camera=camera ;World = all; Light=lights; Nsamples=20} 
//ball;ball2;ball3;ball4;ball5
// Scenario2
//Viewing Coordinate System w u v
let w = camera.LookAt.Normalize()
let u = camera.Up.CrossProduct(w).Normalize()
let v = w.CrossProduct(u)
// Bucle
let bmp = new Bitmap(PixNumW,PixNumH)

//Backward ray tracing, from cam to light

//let p0 = [0..4..PixNumH-1]
//let p1 = [1..4..PixNumH-1]
//let p2 = [2..4..PixNumH-1]
//let p3 = [3..4..PixNumH-1]
let Nproc = 6
let pi =[0..(Nproc-1)] |> List.collect(fun x -> [[x..Nproc..PixNumH-1]])

//Backward ray tracing, from cam to light
let casting (scene:scene, partition, pPartition ,pixH:int list) =
    let mutable rimage = DenseMatrix.init PixNumW PixNumH (fun i j -> 0.0)
    let mutable gimage = DenseMatrix.init PixNumW PixNumH (fun i j -> 0.0)
    let mutable bimage = DenseMatrix.init PixNumW PixNumH (fun i j -> 0.0)
 
    for i in 0..(PixNumW-1) do 
        for j in pixH do
                // create a ray from eye to pixel
                let direct = camera.LookAt+u.ScaleBy(PixWide*float(PixNumW/2-i))+v.ScaleBy(PixHeigh*float(PixNumH/2-j)) // Real to where it points
                let Ray = {uvec= direct.Normalize(); length=infinity; from=camera.EyePoint; travelled=0.0 } //BAD UnitVector3D(0.0,0.0,-1.)  
                //let intersects = castRay (Scene, Ray)
                
                //let grid = Cast_3DGrid (Scene,Ray,pPartition)//CastRay_nest (Scene, Ray) 3DGrid
                let intersects = Cast_Octree (Scene,Ray,partition)// Octree
                //if i =20 && j = 40 then
                //    printfn "Will be a problem..."
                //if grid.Length <> intersects.Length then
                //    printfn "There's a problem at i%d j:%d" i j
                
                //match grid with
                match intersects with
                | [] -> rimage.[i,j] <- 0.0
                        gimage.[i,j] <- 0.0
                        bimage.[i,j] <- 0.0 //bmp.SetPixel(i,j,Color.Gray)
                //match intersects with
                //| _ ->  let color = GlobalIllum(grid |>List.minBy(fun x -> x.t), Scene ,pPartition )//colorAt (intersects |>List.minBy(fun x -> x.t), Scene  )
                | _ ->  let color = GlobalIllum_Octree(intersects |>List.minBy(fun x -> x.t), Scene ,partition )//colorAt (intersects |>List.minBy(fun x -> x.t), Scene  )
                        //let color = colorAtOctree(intersects |>List.minBy(fun x -> x.t), Scene ,partition )// Direct color
                            //(fun x -> x) |> |> List.head S
                        //printfn "Color is: %+A" color
                        rimage.[i,j] <- color.b
                        gimage.[i,j] <- color.g 
                        bimage.[i,j] <- color.b
                
        if i%100=0 then printfn "%i" i
    [rimage;gimage;bimage]

let asyncasting (scene:scene, partition, pPartition, pixW) = async {return casting (scene, partition, pPartition, pixW)}    // Prepare the parallel
//#time // In case of use as a script
let mClr = pi//[p0;p1;p2;p3] 
                  |> List.collect(fun x -> [asyncasting (Scene, partition, pPartition,x)])
                  |> Async.Parallel |> Async.RunSynchronously
                  |> Array.toList |> List.collect(fun x -> x)
//#time // In case of execute as a script
let imagergb = 
    let mutable er = DenseMatrix.init PixNumW PixNumH (fun i j -> 0.0)
    let mutable eg = DenseMatrix.init PixNumW PixNumH (fun i j -> 0.0)
    let mutable eb = DenseMatrix.init PixNumW PixNumH (fun i j -> 0.0)
    for i in [0..3..mClr.Length-1] do
        er <- (mClr.[i]+er)//mClr.[i-3])
        eg <- (mClr.[i+1]+eg)//mClr.[i-2])
        eb <- (mClr.[i+2]+eb)//mClr.[i-1])
    [er;eg;eb]

let imager = imagergb.[0]
let imageg = imagergb.[1]
let imageb =imagergb.[2]
(*
let imager = mClr.[0] + mClr.[3] + mClr.[6] + mClr.[9] + mClr.[12]  + mClr.[15] // Red
let imageg = mClr.[1] + mClr.[4] + mClr.[7] + mClr.[10] + mClr.[13]  + mClr.[16]    //Green
let imageb = mClr.[2]+ mClr.[5] + mClr.[8] + mClr.[11] + mClr.[14]   + mClr.[17]   //Blue
*)
//let (imager,imageg,imageb) = [0..mClr.Length-1]
for i in 0..(PixNumW-1) do
    for j in 0..(PixNumH-1) do
        //printfn "pixel i:%d j:%d" i j
        bmp.SetPixel(i,j,Color.FromArgb(int(255.0*imager.[i,j]),  int(255.0*imageg.[i,j]) , int(255.0*imageb.[i,j]) ))

//bmp.Save(@"C:\Users\JoseM\Desktop\test_AllScene_gridDirLight3.jpg")
bmp.Save(@"C:\Users\JoseM\Desktop\img_render\comparative\test_Scene2_Octree22.jpg")
let form = new Form(Text="Rendering test",TopMost=true)
form.Show()

let img = new PictureBox(Dock=DockStyle.Fill)
form.Controls.Add(img)

img.Image <- bmp
printfn "end"
