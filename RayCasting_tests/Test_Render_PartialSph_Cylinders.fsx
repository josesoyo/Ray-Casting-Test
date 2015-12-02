// Render test in which there are Partial spheres and Cylinders
// Partial spheres of 2 types: Like surface of a lens and cutted spheres

#r @"..\packages\MathNet.Spatial.0.2.0-alpha\lib\net40\MathNet.Spatial.dll"
#r @"..\packages\MathNet.Numerics.3.8.0\lib\net40\MathNet.Numerics.dll"
#r @"..\packages\MathNet.Numerics.FSharp.3.8.0\lib\net40\MathNet.Numerics.FSharp.dll"

#load "RayType.fs"
#load "BBox.fs"
#load "RayTypeMethods.fs"
#load "RayCore.fs"
#load "RandomMethods.fs"
#load "RayColor.fs"
#load "ObjReader.fs"


open System.IO
open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open ObjReader
open RayType
open RayTypeMethods
open RayCore
open RayColor


let PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062
//
//Methods to intersect a partial sphere
//

//Definitions of values
//
// Pixels
let PixNumW = 200
let PixNumH = 200
let PixWide = 2.0/float(PixNumW) //Write something
let PixHeigh = 2.0/float(PixNumH)
//
let camera={EyePoint=Point3D(-2.5,-0.0,0.0);LookAt=Vector3D(1.5,1.e-10,1.e-10); Up=Vector3D(0.0,0.0,1.0)}
let light = {origin = Point3D(1.0,0.750,2.50);color=Color(1.0,1.0,1.0); intensity = 40.0}
let light2 = {origin = Point3D(-2.0,-0.750,2.50);color=Color(1.0,1.0,1.0); intensity = 40.0}
let light3 = {origin = Point3D(0.50,-0.50,0.00);color=Color(1.0,1.0,1.0); intensity = 40.0}
let reflective ={DiffuseLight = Color(0.0,0.0,1.0);SpecularLight = Color(0.0,0.,1.0);shinness= 50; R=0.250; T=0.0; n= 1.45;Fresnel = false} 
let refl2 ={DiffuseLight = Color(0.59,0.20,0.39);SpecularLight = Color(0.7,0.8,0.9);shinness= 90; R=0.20; T=0.0; n= 1.5;Fresnel = false} 
let whitte ={DiffuseLight = Color(1.0,1.0,1.0);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.050; T=0.0; n= 1.45;Fresnel = false}

// Partial Sphere Type1
let partSph = GeneratePartialSphere(Point3D(0.5,-0.750,0.),1.,reflective,-0.25,0.5,PI,3.*PI/2.,UnitVector3D(0.,0.,1.))
//Partial Sphere of Type2
let surfaceLens2 = GenerateLensSurface(Point3D(1.5,-0.0,0.4),1.,refl2,1.75,UnitVector3D(0.0,0.,-1.0),true)
let surfaceLens = {Sphere=surfaceLens2.Sphere;CosMin = -1.*surfaceLens2.CosMin; Axis = surfaceLens2.Axis; Convex = true}
// Cylinder
let cyl = GenerateCylinder(0.50,0.,1.5,Point3D(2.,2.,0.),UnitVector3D(0.,-0.5,1.),reflective)
// Generate a lens
let (r1,r2) = (1.,1.5)
let axis = UnitVector3D(0.5,1.,0.)
let (th,dia) = (0.65, 1.5)
let stP = Point3D(1.,0.3,0.)
//let(ls1,lcyl,ls2) = CreateLensBiConvex(r1, r2, axis, th, dia, stP, reflective)
let(ls1,lcyl,ls2) = CreateLensBiConcave(r1, r2, axis, th, dia, stP, reflective)

let path2 = Path.Combine(__SOURCE_DIRECTORY__,"..\MeshSamples\plane.obj")
let mesh1 = ReadMeshWavefront(path2,whitte) |> Mesh_BBox 
//let all = {Meshes = [mesh1];Sphere = [];PartSphere= []; SurfaceLens = [surfaceLens] ;Cylinder =[cyl]} //partSph
let all = {Meshes = [mesh1];Sphere = [];PartSphere= []; SurfaceLens = [ls1;ls2] ;Cylinder =[lcyl]} //partSph
let lights ={Point= [light;light2;light3];Circle = []} 
let Scene = {Camera=camera ;World = all; Light=lights; Nsamples=25} 

//Viewing Coordinate System w u v
let w = camera.LookAt.Normalize()
let u = camera.Up.CrossProduct(w).Normalize()
let v = w.CrossProduct(u)
// Bucle
let bmp = new Bitmap(PixNumW,PixNumH)

let Nproc = 1
let proci =[0..(Nproc-1)] |> List.collect(fun x -> [[x..Nproc..PixNumH-1]])
//let p0 = [0..1..PixNumH-1]
//let p1 = [1..4..PixNumH-1]
//let p2 = [2..4..PixNumH-1]
//let p3 = [3..4..PixNumH-1]
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
                let intersects = CastRay_nest (Scene, Ray)

                match intersects with
                | [] -> rimage.[i,j] <- 0.0
                        gimage.[i,j] <- 0.0
                        bimage.[i,j] <- 0.0 //bmp.SetPixel(i,j,Color.Gray)
                | _ ->  //let color = GlobalIllum(intersects |>List.minBy(fun x -> x.t), Scene) // INPUT FOR NEST NO PARTITION
                        let color =colorAt (intersects |>List.minBy(fun x -> x.t), Scene  )
                            //(fun x -> x) |> |> List.head S
                        //printfn "Color is: %+A" color
                        rimage.[i,j] <- color.r
                        gimage.[i,j] <- color.g 
                        bimage.[i,j] <- color.b
        if i%100=0 then printfn "%i" i
    [rimage;gimage;bimage]

let asyncasting pixW = async {return casting pixW}    // Prepare the parallel

let mClr = proci//[p0]//;p1;p2;p3] 
                  |> List.collect(fun x -> [asyncasting x])
                  |> Async.Parallel |> Async.RunSynchronously
                  |> Array.toList |> List.collect(fun x -> x)


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
let imager = mClr.[0]// + mClr.[3] + mClr.[6] + mClr.[9]      // Red
let imageg = mClr.[1]// + mClr.[4] + mClr.[7] + mClr.[10]     //Green
let imageb = mClr.[2]// + mClr.[5] + mClr.[8] + mClr.[11]     //Blue
*)
for i in 0..(PixNumW-1) do
    for j in 0..(PixNumW-1) do
        bmp.SetPixel(i,j,Color.FromArgb(int(255.0*imager.[i,j]),  int(255.0*imageg.[i,j]) , int(255.0*imageb.[i,j]) ))


// Show the image
let form = new Form(Text="Rendering test",TopMost=true)
form.Show()
let img = new PictureBox(Dock=DockStyle.Fill)
form.Controls.Add(img)
img.Image <- bmp

printfn "ciao"
printfn "ciao"
