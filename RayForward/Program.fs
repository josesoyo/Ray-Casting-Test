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
let BK7 ={DiffuseLight = Color(0.90,0.90,0.990);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.0; T=1.0; n= 1.5170;Fresnel = false} // No reflectifity or dispersion

// Objects
//let sph = {center = Point3D(0.0,0.0,0.0) ; radius = 0.25 ; material = Glass}
//let cyl = GenerateCylinder(0.25,0.,0.090,Point3D(0.,0.0,0.),UnitVector3D(0.,0.0,-1.), Mirror)

let camera={EyePoint=Point3D(-20.,0.,0.);LookAt=Vector3D(992.,1.e-10,1.e-10); Up=Vector3D(1.,1.,1.)} // SHOULD NOT

//
//       Gaussian properties
//

let w0 = 7.1253e-3 // orig: 7.1253e-3
let distz0 = 25.5
let origing = Point3D(-0.50,0.0,0.000)  //
let normalg = UnitVector3D(1.0,0.0,0.0)
let rotationMatrix= rotateSource (normalg)

//
//      Sensor definition
//
let xpix = 200//400
let ypix = 200//400
let pixsize = 0.000125   // orig: 0.00025 
let photosSature = 1          //Changed
let normal= UnitVector3D(0.0,1.0,0.0)   // Sensor normal 
let origin = Point3D(-0.0121,-0.20,0.012510)   // sensor origin (One corner)

let MaxNrays = 100000        //Rays traced
let nthreads = 1            // Num of parallel threads
(*
//
//  Place a lens before the interferometer to see if I can simulate circular rings
//
// lens 2 - E
let (r1,r2) = (700e-3,700e-3)
let axis = UnitVector3D(1.0,0.,0.0)
let (th,dia) = (6e-3, 50.e-3)
let stP = Point3D(-0.2,0.0,0.)

//let(ls1,lcyl,ls2) = CreateLensBiConvex(r1, r2, axis, th, dia, stP, BK7)
let(ls1,lcyl,ls2) = CreateLensBiConcave(r1, r2, axis, th, dia, stP, BK7)
*)
//To play with mesh
//let path2 = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\Unitplane.obj"
let path2 =  Path.Combine(__SOURCE_DIRECTORY__,"..\MeshSamples\Unitplane.obj")
//let path2 = Path.Combine(__SOURCE_DIRECTORY__,
let nEMX = UnitVector3D(-1.0,0.0,0.0)
let tEMX = Vector3D(0.07150 ,0., 0.)
//  X end mirror
let EMX = ReadMeshWavefront(path2,Mirror) |> fun x -> Scale x [0.1;0.1;0.1]   // Scale
          |>fun x -> RotateMesh x nEMX                                        // Rotate
          |> fun x -> Translate x tEMX
          |>  Mesh_BBox                          
//  y end mirror - FLAT
let nEMY = UnitVector3D(0.0,-1.0,0.0)//.Rotate(UnitVector3D(0.0,0.,1.0),0.0153,MathNet.Spatial.Units.Degrees())
let tEMY = Vector3D(0. ,0.07150+0.5e-6 , 0.)          // QuarterWave added to the mirror:+0.25e-6 
let EMY = ReadMeshWavefront(path2,Mirror) |> fun x -> Scale x [0.1;0.1;0.1]   // Scale
          |> fun x -> RotateMesh x nEMY                                        // Rotate
          |> fun x -> Translate x tEMY                                         // Translate
          |>  Mesh_BBox                         
// y end mirror - CURVED

let EMYc = GenerateLensSurface(tEMX.ToPoint3D()+Vector3D(-1.,0.,0.),0.04,Mirror,0.075,nEMY.Negate(),false)
// BS
let nBS = UnitVector3D(-1.0,1.0,0.0)
let BS = ReadMeshWavefront(path2,SRM) //|> fun x -> Scale x [0.1;0.1;0.1]   // Scale
         |>fun x -> RotateMesh x nBS 
         |>  Mesh_BBox                          // Rotate

//
// Generate world
let all = {Meshes = [EMX;BS]; 
           Sphere = []; PartSphere=[];SurfaceLens=[EMYc];
           Cylinder =[]} //for sphere
let partition = Partitionate (all, 2)
let scene = {Camera=camera ;World = all; Light=lights; Nsamples=1} 
/////
//
//  Repetitive part
//
/////
let imasinc MaxNrays nthreads =
    let image = 
        //let mutable startSensor = InitiateComplexMonoSensor (origin, normal, xpix, ypix, pixsize, photosSature)
        let mutable startSensor = InitiateComplexSensor (origin, normal, xpix, ypix, pixsize, photosSature)
        for i in 0..(MaxNrays/nthreads-1) do
            //let rayforward = genRayFoward(clight,scene.Nsamples)
            let rayGaussforward = genGaussRayMaxRad(w0,distz0,origing,normal,rotationMatrix,1e-6,0.25)
            //printfn "%+A" rayGaussforward
            //(rayGaussforward.from-origing).Length
            startSensor <- forwardComplexMono(scene, partition, FromRayGaussianToRayForward(rayGaussforward), startSensor,1e-6)
        startSensor
    image
// Async

let asyncima  NaxNrays nthreads = async {return imasinc MaxNrays nthreads } 

let threadsima = [1..nthreads] 
                  |> List.collect(fun x -> [asyncima  MaxNrays nthreads])
                  |> Async.Parallel |> Async.RunSynchronously
                  |> Array.toList //|> List.collect(fun x -> x)


(*
let image = 
     //let im1 = sensorCSSum (threadsima.[0],threadsima.[1])  // Re+im
     let im1 = sensorCSum (threadsima.[0],threadsima.[1])
     //let im2 = sensorCSSum (threadsima.[2],threadsima.[3])    // Re+im
     let im2 = sensorCSum (threadsima.[2],threadsima.[3])
     //sensorCSSum(im1,im2)
     sensorCSum(im1,im2)
*)
let image =  // 
    // Sum the values of each parallel process
     let mutable ima = InitiateComplexSensor (origin, normal, xpix, ypix, pixsize, photosSature)
     for i in [0..threadsima.Length-1] do
        ima <- sensorCSum (ima,threadsima.[i])
     ima
//image.ISensor.Map
image.c2
// Save the image
let path = @"C:\Users\JoseM\Desktop\Michelson05.jpg"
//SensorMonoCromToImage(image, path)        // Re+im
SensorCromToImage(image, path)


// Plot phase
let phaseval = SensorComplexPhase(image)
let bmp2 = new Bitmap(image.xpix,image.ypix)
let maxmono = matrixmax(phaseval)
let minmono = matrixmin(phaseval)
for i in 0..(image.xpix-1) do
    for j in 0..(image.ypix-1) do
        bmp2.SetPixel(i,j,Color.FromArgb(min 255 (abs(int((255.*(phaseval.[i,j]+minmono))/(2.*maxmono)))),  
                                        min 255 (abs(int((255.*(phaseval.[i,j]+minmono))/(2.*maxmono)))),
                                        min 255 (abs(int((255.*(phaseval.[i,j]+minmono))/(2.*maxmono)))) )
                                        )    
//let path = @"C:\Users\JoseM\Desktop\phase.jpg"
bmp2.Save(@"C:\Users\JoseM\Desktop\phase_align.jpg")
let form = new Form(Text="Rendering test",TopMost=true)
form.Show()

let img = new PictureBox(Dock=DockStyle.Fill)
form.Controls.Add(img)

img.Image <- bmp2
printfn "end phase plot"

