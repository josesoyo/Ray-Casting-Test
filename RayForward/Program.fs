﻿// Learn more about F# at http://fsharp.org
// See the 'F# Tutorial' project for more help.


open System.IO
open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml
open MathNet.Numerics.Statistics

open ObjReader
open RayType
open RayCore
open RayTypeMethods
//open RayColorGrid
open RayCoreGrid
open PreprocesorGrid
open RandomMethods
open GaussianSource
open ForwardCore
open SensorModule
open ForwardCoreComplexMono
let PI = 3.141592653589


///////
//
///////

let lights ={Point= [];Circle = []} 



//
//              Fossils of Backwards rayTracing
//              Things required by initial creation of a render engine not required in ForwardRayTracing
//
//  Materials
//let Glass ={DiffuseLight = Color(0.90,0.90,0.990);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.50; T=0.50; n= 1.3;Fresnel = true}
let Mirror ={DiffuseLight = Color(0.90,0.90,0.990);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=1.0; T=0.0; n= 1.0;Fresnel = false}
let SRM ={DiffuseLight = Color(0.90,0.90,0.990);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.5; T=0.50; n= 1.0;Fresnel = false}

// Objects
//let sph = {center = Point3D(0.0,0.0,0.0) ; radius = 0.25 ; material = Glass}
//let cyl = GenerateCylinder(0.25,0.,0.090,Point3D(0.,0.0,0.),UnitVector3D(0.,0.0,-1.), Mirror)

let camera={EyePoint=Point3D(-20.,0.,0.);LookAt=Vector3D(992.,1.e-10,1.e-10); Up=Vector3D(1.,1.,1.)} // SHOULD NOT

//
//       Gaussian properties
//

let w0 = 0.00125
let distz0 = 15.5
let origing = Point3D(-0.50,0.0,0.000)  //
let normalg = UnitVector3D(1.0,0.0,0.0)
let rotationMatrix= rotateSource (normalg)

//
//      Sensor definition
//
let xpix = 100
let ypix = 100
let pixsize = 0.0005   // 10 Micron def
let photosSature = 1          //Changed
let normal= UnitVector3D(0.0,1.0,0.0)   // Sensor normal 
let origin = Point3D(-0.021,-0.20,0.02510)   // sensor origin (One corner)

let MaxNrays = 1000        //Rays traced
let nthreads = 1            // Num of parallel threads


//To play with mesh
let path2 = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\Unitplane.obj"
let nEMX = UnitVector3D(-1.0,0.0,0.0)
let tEMX = Vector3D(0.250 ,0., 0.)
//  X end mirror
let EMX = ReadMeshWavefront(path2,Mirror) |> fun x -> Scale x [0.1;0.1;0.1]   // Scale
          |>fun x -> RotateMesh x nEMX                                        // Rotate
          |> fun x -> Translate x tEMX
          |>  Mesh_BBox                          
//  y end mirror
let nEMY = UnitVector3D(0.0,-1.0,0.0)//.Rotate(UnitVector3D(0.0,0.,1.0),0.153,MathNet.Spatial.Units.Degrees())
let tEMY = Vector3D(0. ,0.250, 0.)
let EMY = ReadMeshWavefront(path2,Mirror) |> fun x -> Scale x [0.1;0.1;0.1]   // Scale
          |> fun x -> RotateMesh x nEMY                                        // Rotate
          |> fun x -> Translate x tEMY                                         // Translate
          |>  Mesh_BBox                         
// BS
let nBS = UnitVector3D(-1.0,1.0,0.0)
let BS = ReadMeshWavefront(path2,SRM) //|> fun x -> Scale x [0.1;0.1;0.1]   // Scale
         |>fun x -> RotateMesh x nBS 
         |>  Mesh_BBox                          // Rotate

//
// Generate world
let all = {Meshes = [EMX;EMY;BS];Sphere = [];Cylinder =[]} //for sphere
let partition = Partitionate (all, 2)
let scene = {Camera=camera ;World = all; Light=lights; Nsamples=1} 

/////
//
//  Repetitive part
//
/////
let imasinc MaxNrays nthreads =
    let image = 
        let mutable startSensor = InitiateComplexSensor (origin, normal, xpix, ypix, pixsize, photosSature)
        for i in 0..(MaxNrays/nthreads-1) do
            //let rayforward = genRayFoward(clight,scene.Nsamples)
            let rayGaussforward = genGaussRayMaxRad(w0,distz0,origing,normal,rotationMatrix,1e-6,0.25)
            //(rayGaussforward.from-origing).Length
            startSensor <- forwardComplexMono(scene, partition, FromRayGaussianToRayForward(rayGaussforward), startSensor,1e-6)
        startSensor
    image
// Async
//#time
let asyncima  NaxNrays nthreads = async {return imasinc MaxNrays nthreads } 

let threadsima = [1..nthreads] 
                  |> List.collect(fun x -> [asyncima  MaxNrays nthreads])
                  |> Async.Parallel |> Async.RunSynchronously
                  |> Array.toList //|> List.collect(fun x -> x)

//#time
let image = threadsima.[0]
//     let im1 = sensorCSSum (threadsima.[0],threadsima.[1])
//     let im2 = sensorCSSum (threadsima.[2],threadsima.[3])
//     sensorCSSum(im1,im2)

// Save the image
let path = @"C:\Users\JoseM\Desktop\Michelson.jpg"
SensorCromToImage(image, path)
//SensorMonoCromToImage(image, path)
