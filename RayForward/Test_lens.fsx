// Test of a single focusing lens

#r @"..\packages\MathNet.Spatial.0.2.0-alpha\lib\net40\MathNet.Spatial.dll"
#r @"..\packages\MathNet.Numerics.3.8.0\lib\net40\MathNet.Numerics.dll"
#r @"..\packages\MathNet.Numerics.FSharp.3.8.0\lib\net40\MathNet.Numerics.FSharp.dll"

#load "..\RayCasting_tests/RayType.fs"
#load "..\RayCasting_tests/BBox.fs"
#load "..\RayCasting_tests/RayTypeMethods.fs"
#load "..\RayCasting_tests/RandomMethods.fs"
#load "..\RayCasting_tests/RayCore.fs"
#load "..\RayCasting_tests/RayCoreGrid.fs"
#load "..\RayCasting_tests/ObjReader.fs"
#load "..\RayCasting_tests/PreprocesorGrid.fs"
#load "..\RayCasting_tests/RayCoreGrid.fs"
//#load "RayColorGrid.fs"
#load "Sensor.fs"
#load "GaussianSource.fs"
#load "ForwardCore.fs"
#load "ForwardCoreComplexMono.fs"

open System.IO
open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml
open MathNet.Numerics.Statistics

open ObjReader
open RayType
open RayTypeMethods
open RayCore
//open RayColorGrid
open RayCoreGrid
open PreprocesorGrid
open RandomMethods
open SensorModule
open GaussianSource
open ForwardCore
open ForwardCoreComplexMono
let PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062

let lights ={Point= [];Circle = []} //light;light2;light3;light4;light5
// Gaussian properties
let w0 = 5.e-3 //25 - m => 5mm
let diaMaxGen = 1.5*w0
let distz0 = 1000000.
let origing =Point3D(-0.0,0.0,-1.e-5)//  for lens   //Point3D(-0.0,0.0,-3.e-1) ball 

// Glass material
let Glass ={DiffuseLight = Color(0.0090,0.0090,0.00990);SpecularLight = Color(0.04,0.04,0.04);shinness= 60; R=0.0; T=1.0; n= 1.517;Fresnel = true}

// Generate a lens
(* 
// lens 1
let (r1,r2) = (8.88e-3,8.88e-3)
let axis = UnitVector3D(0.,0.,-1.)
let (th,dia) = (2.38e-3, 4.5e-3)
let stP = Point3D(0.,0.0,0.)
*)
// lens 2 - EFL= 10mm from EO
let (r1,r2) = (9.98e-3,9.98e-3)
let axis = UnitVector3D(0.,0.,-1.)
let (th,dia) = (2e-3, 5.e-3)
let stP = Point3D(0.,0.0,0.)

let(ls1,lcyl,ls2) = CreateLensBiConvex(r1, r2, axis, th, dia, stP, Glass)
//-0.00795+0.00995
let sph = {center = Point3D(0.0,0.0,0.0) ; radius = 0.25 ; material = Glass}

// Add it to the world
let all = {Meshes = [];
            Sphere = []; PartSphere=[]; SurfaceLens =[ls1;ls2];
            Cylinder =[lcyl]} //for sphere

let partition = Partitionate (all, 2)
let camera={EyePoint=Point3D(-20.,0.,0.);LookAt=Vector3D(992.,1.e-10,1.e-10); Up=Vector3D(1.,1.,1.)} // SHOULD NOT
let scene = {Camera=camera ;World = all; Light=lights; Nsamples=1} 
let xpix = 3
let ypix = 1000//820
let pixsize = 0.0000151//25 //751
let photosSature = 150


let normal= UnitVector3D(0.0,0.0,1.0)

let rotationMatrix= rotateSource (normal) // era otra varianble
let origin = Point3D(-pixsize,0.,9.31e-3+th)//21.31e-2) // for the LENS the focus are 8.16 & 9.31
let MaxNrays = 10000
let nthreads = 4
let imasinc MaxNrays nthreads =
    let image = 
        //let mutable startSensor = InitiateComplexMonoSensor (origin, normal, xpix, ypix, pixsize, photosSature)
        let mutable startSensor = InitiateComplexSensor (origin, normal, xpix, ypix, pixsize, photosSature)
        for i in 0..(MaxNrays/nthreads-1) do
            //let rayforward = genRayFoward(clight,scene.Nsamples)
            let rayGaussforward = genGaussRayMaxRadY(w0,distz0,origing,normal,rotationMatrix,1e-6,diaMaxGen)
            //(rayGaussforward.from-origing).Length
            startSensor <- forwardComplexMono(scene, partition, FromRayGaussianToRayForward(rayGaussforward), startSensor,1e-6)
            //startSensor <- forwardComplexMono(scene, partition, FromRayGaussianToRayForward(rayGaussforward), startSensor,1e-6)
        startSensor
    image
// Async
#time
let asyncima  NaxNrays nthreads = async {return imasinc MaxNrays nthreads } 

let threadsima = [1..nthreads] 
                  |> List.collect(fun x -> [asyncima  MaxNrays nthreads])
                  |> Async.Parallel |> Async.RunSynchronously
                  |> Array.toList //|> List.collect(fun x -> x)

#time
let image =  // 
    // Sum the values of each parallel process
     let mutable ima = InitiateComplexSensor (origin, normal, xpix, ypix, pixsize, photosSature)
     for i in [0..threadsima.Length-1] do
        ima <- sensorCSum (ima,threadsima.[i])
     ima
//image.Origin.Z-2.e-3
image.c2
#r @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\packages\FSharp.Charting.0.90.12\lib\net40\FSharp.Charting.dll"
open FSharp.Charting

// Transform into a 1dimensional
let monoarr =  // 1719
    [0..819]|> List.map(fun x -> (100.*pixsize*float(x),image.Sensor.[1,x].Magnitude*image.Sensor.[1,x].Magnitude)) //Magnitude 
let pharr = 
    [0..819]|> List.map(fun x -> (100.*pixsize*float(x),image.Sensor.[1,x].Phase)) //Magnitude 
let logarr = monoarr |> List.map(fun x -> ((fst x),log10(snd x))) 
            |> List.map(fun x -> if snd x > 0.0 then x ; else (fst x,0.))
let phlogarr = pharr |> List.map(fun x -> ((fst x),log10(snd x))) 
            |> List.map(fun x -> if snd x > 0.0 then x ; else (fst x,0.))

let gr = Chart.Line(logarr, Name= "Logintensity at teoretical focus")
let phgr = Chart.Line(pharr, Name= "Logintensity a focus point")
//phgr.ShowChart()
gr.ShowChart()
//gr.SaveChartAs(@"C:\Users\JoseM\Desktop\500k.jpeg",ChartTypes.ChartImageFormat.Jpeg)    //Logarithmic charting
