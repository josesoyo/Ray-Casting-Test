//  Script file with the algorithms to create  the forwardRayTracing
//  Pre-modifications, include sensor as an object of the world (square one)
// 

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
let w0 = 0.005 //25
let diaMaxGen = 2.*w0
let distz0 = 5.
let origing = Point3D(-0.0,0.0,0.0050)  //
let normalg = UnitVector3D(0.0,0.0,-1.0)
let rotationMatrix= rotateSource (normalg)
//  Sphere
//let Glass ={DiffuseLight = Color(0.90,0.90,0.990);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.50; T=0.50; n= 1.3;Fresnel = true}
//let sph = {center = Point3D(0.0,0.0,0.0) ; radius = 0.25 ; material = Glass}
//Cylinder
let Mirror ={DiffuseLight = Color(0.90,0.90,0.990);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=1.0; T=0.0; n= 1.3;Fresnel = false}
let cyl = GenerateCylinder(diaMaxGen,0.,0.090,Point3D(0.,0.0,0.),UnitVector3D(0.,0.0,-1.), Mirror)
printfn "Cylinder is: %+A \n" cyl
let all = {Meshes = [];Sphere = [];Cylinder =[cyl]} //for sphere
//let all = {Meshes = [mesh1];Sphere = []} //Mesh

let partition = Partitionate (all, 2)
let camera={EyePoint=Point3D(-20.,0.,0.);LookAt=Vector3D(992.,1.e-10,1.e-10); Up=Vector3D(1.,1.,1.)} // SHOULD NOT
let scene = {Camera=camera ;World = all; Light=lights; Nsamples=1} 

let xpix = 3
let ypix = 1820
let pixsize = 0.0000151//25 //751
let photosSature = 150

let normal= UnitVector3D(0.0,0.0,1.0)

let origin = Point3D(-pixsize,-0.0,-0.50) // for sensor
//
//let BlackSensor = InitiateSensor (origin:Point3D, normal, xpix, ypix, pixsize, photosSature)
let MaxNrays = 100000
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
let image =  // if other sensor type sensorCSum(s1,s2)
     let im1 = sensorCSum (threadsima.[0],threadsima.[1])
     let im2 = sensorCSum (threadsima.[2],threadsima.[3])
     sensorCSum(im1,im2)
image.Origin
image.c2
image.ypix
let path = @"C:\Users\JoseM\Desktop\ForwardTestPhase2.jpg"
//SensorCromToImage(image, path)
//SensorMonoCromToImage(image, path)
((2.633547655)%1e-6)/1e-6
3.2%2.
let Rsmat = SparseMatrix.init 3 3 (fun i j -> float(j+i))       // To initialize - GOOD??
Rsmat.Map(fun i -> i*i)//   .Map((fun i j -> j*j),Rsmat)

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
//Chart.Line(monoarr, Name= "intensity 5m").ShowChart()
//Chart.Line(logarr, Name= "Logintensity a 5m Cil 50k").ShowChart()    //Logarithmic charting
let gr = Chart.Line(logarr, Name= "Logintensity a 1m Cila rad5 pic")
let phgr = Chart.Line(phlogarr, Name= "Logintensity a 1m Cila rad5 pic").ShowChart()
gr.ShowChart()
gr.SaveChartAs(@"C:\Users\JoseM\Desktop\500k.jpeg",ChartTypes.ChartImageFormat.Jpeg)    //Logarithmic charting
