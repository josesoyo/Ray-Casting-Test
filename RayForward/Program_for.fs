// Learn more about F# at http://fsharp.org
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
let PI = 3.141592653589


let clight = {Centre=Point3D(-0.0,0.0,5.0); Normal=UnitVector3D(1e-14,1e-14,-1.0);
               Radius=0.40;Area=PI*0.4*0.4; 
               color=Color(1.0,1.0,0.0); 
               intensityDensity=10000.;
               RotationMatrix=Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),UnitVector3D(0.0,-0.0,-1.0))
               }
let lights ={Point= [];Circle = [clight]} //light;light2;light3;light4;light5
// Gaussian properties
let w0 = 0.25
let distz0 = 0.5
let origing = Point3D(-0.0,0.0,0.0150)  //
let normalg = UnitVector3D(0.0,0.0,-1.0)
let rotationMatrix= rotateSource (normalg)
//  Sphere
//let Glass ={DiffuseLight = Color(0.90,0.90,0.990);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.50; T=0.50; n= 1.3;Fresnel = true}
//let sph = {center = Point3D(0.0,0.0,0.0) ; radius = 0.25 ; material = Glass}
//Cylinder
let Mirror ={DiffuseLight = Color(0.90,0.90,0.990);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=1.0; T=0.0; n= 1.3;Fresnel = false}
let cyl = GenerateCylinder(0.5,0.,0.0510,Point3D(0.,0.0,0.),UnitVector3D(0.,0.0,-1.), Mirror)
printfn "Cylinder is: %+A \n" cyl
let all = {Meshes = [];Sphere = [];Cylinder =[cyl]} //for sphere
//let all = {Meshes = [mesh1];Sphere = []} //Mesh

let partition = Partitionate (all, 2)
let camera={EyePoint=Point3D(-20.,0.,0.);LookAt=Vector3D(992.,1.e-10,1.e-10); Up=Vector3D(1.,1.,1.)} // SHOULD NOT
let scene = {Camera=camera ;World = all; Light=lights; Nsamples=1} 

let xpix = 150
let ypix = 150
let pixsize = 0.02667//751
let photosSature = 10

let normal= UnitVector3D(0.0,0.0,1.0)

let origin = Point3D(-2.0,-2.0,-2.5) // for sphere
//
//let BlackSensor = InitiateSensor (origin:Point3D, normal, xpix, ypix, pixsize, photosSature)
let MaxNrays = 50000
let nthreads = 4
let imasinc MaxNrays nthreads =
    let image = 
        let mutable startSensor = InitiateSensor (origin, normal, xpix, ypix, pixsize, photosSature)
        for i in 0..(MaxNrays/nthreads-1) do
            //let rayforward = genRayFoward(clight,scene.Nsamples)
            let rayGaussforward = genGaussRayMaxRad(w0,distz0,origing,normal,rotationMatrix,1e-6,0.5)
            //(rayGaussforward.from-origing).Length
            startSensor <- forward(scene, partition, FromRayGaussianToRayForward(rayGaussforward), startSensor)
        startSensor
    image
// Async
let asyncima  NaxNrays nthreads = async {return imasinc MaxNrays nthreads } 
let threadsima = [1..nthreads] 
                  |> List.collect(fun x -> [asyncima  MaxNrays nthreads])
                  |> Async.Parallel |> Async.RunSynchronously
                  |> Array.toList //|> List.collect(fun x -> x)

let image = 
     let im1 = sensorSum (threadsima.[0],threadsima.[1])
     let im2 = sensorSum (threadsima.[2],threadsima.[3])
     sensorSum(im1,im2)
//image.Origin
//image.c2


let path = @"C:\Users\JoseM\Desktop\ForwardTest.jpg"
SensorToImage(image, path)