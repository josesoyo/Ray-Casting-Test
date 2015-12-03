//  Script file with the algorithms to create  the forwardRayTracing
//  Pre-modifications, include sensor as an object of the world (square one)
// 

#r @"packages\MathNet.Spatial.0.2.0-alpha\lib\net40\MathNet.Spatial.dll"
#r @"packages\MathNet.Numerics.3.8.0\lib\net40\MathNet.Numerics.dll"
#r @"packages\MathNet.Numerics.FSharp.3.8.0\lib\net40\MathNet.Numerics.FSharp.dll"

#load "RayCasting_tests/RayType.fs"
#load "RayCasting_tests/BBox.fs"
#load "RayCasting_tests/RayCore.fs"
#load "RayCasting_tests/RayCoreGrid.fs"
#load "RayCasting_tests/RandomMethods.fs"
//#load "RayColorGrid.fs"
#load "RayCasting_tests/ObjReader.fs"
#load "Sensor.fs"


#load "RayCasting_tests/PreprocesorGrid.fs"
#load "RayCasting_tests/RayCoreGrid.fs"

open System.IO
open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml
open MathNet.Numerics.Statistics

open ObjReader
open RayType
open RayCore
//open RayColorGrid
open RayCoreGrid
open PreprocesorGrid
open RandomMethods
open SensorModule
let PI = 3.141592653589

//Test about the foward ray tracing

//1st - Generate Ray from source
let genRayFoward(clight:Clight,samples:int) =
    //Sample a point from the source
    // Initial seed for the ray going out from the source

    let (RandR,RandPhi) = SampDisk clight.Radius
    let (x,y) = (RandR*cos(RandPhi),RandR*sin(RandPhi))
    let Rpoint = Point3D(x,y,0.0)
    let shape = Rpoint.TransformBy(m=clight.RotationMatrix) // I must rorate the normal vector (0.,0.,1.) -> (xn,yn,zn)
    let Translate = clight.Centre
    let point = Point3D(Translate.X+shape.X,Translate.Y+shape.Y,Translate.Z+shape.Z)
    // Now I have a point in the surface of the Light, after that, gen a direction
    // Generate a direction for the ray - Cosine weighthed
    let (ux,uy,uz)= SampUnitHemiCosToCart (1) //  1 = dummy
    let unitV = clight.Normal                      // in the future UnitVector3D(ux,uy,uz)
    let rayForward = {uvec=unitV; 
                      mlength=9000.;          // Works as max length that can travel a Ray
                      from=point; 
                      travelled=0.0; 
                      color=clight.color;   // The color o the light
                      intensity=clight.intensityDensity*clight.Area/float(samples)}
    rayForward
(*
////////////
//      Test this random number
///////////

let a = [0..10000] |> List.collect(fun x -> [genRayFoward(clight,10).uvec])
let b = a |> List.collect(fun x -> [x.X*x.X+x.Y*x.Y+x.Z*x.Z]) // Norm
let cosinus = a |> List.collect(fun x -> [((x.X*x.X+x.Y*x.Y), x.Z)])
Chart.Point(cosinus, Name= "Cosinus corrected").ShowChart()

let hight = a |> List.collect(fun x -> [x.Z])
let histo = Histogram(hight,5)
let myhisto =[0..(histo.BucketCount-1)] |> List.map(fun x -> float(histo.[x].Count)/float(histo.DataCount))
Chart.Point(myhisto).ShowChart() // I don't understand this histogram =_=

////////////
*)
//
//
//
//
let ReflectedRay (ray:RayForward, intersection) =
    //It returns the reflected ray
    //
    let normal = intersection.normal
    let lightDir = ray.uvec // Direction of the ray
    let Reflvect = -normal.ScaleBy(2.0*normal.DotProduct(lightDir))+lightDir //Inverted - Reflected ray
    //
    printfn "%+A" (Reflvect.Normalize())
    Reflvect.Normalize()
    //
let TransmittedRay (ray:RayForward, intersection) =
    //
    //
    let RayDir = ray.uvec
    let LightDir = RayDir.Negate() //Ray that incides on the surface * -1
    let SideRay (ci,index) =   
        // Changes the situation checking from air or to  
       if ci < 0.  then 
        (-ci, 1./index) 
       else
        (ci, index)
    let n = intersection.material.n // With AIR
    let ci = intersection.normal.DotProduct(LightDir) //Cosinus incident angle
 
    let (cos_inc,nu) = SideRay(ci, n)
    let inv_n = 1./nu // It is used the inverse = (n_from/n_to)
    let AngCritic n_transm =
        // Obtain Critical angle for TIR
        if n_transm > 1. then
            1.571
        else
            let tir = asin(n_transm) // Pi/2  
            tir
    let ang_critic = AngCritic nu
    let ang_inc = acos(cos_inc)
    //
    if ang_inc < ang_critic then // TIR
        let cos_trans = sqrt(1.-(inv_n*inv_n )*(1.-cos_inc*cos_inc)) // Cosinus transmited
        let vtrans = RayDir.ScaleBy(inv_n) - intersection.normal.ScaleBy(cos_trans - inv_n*cos_inc)
        vtrans.Normalize()
    else  ReflectedRay (ray, intersection) // Else reflected
    //
    //

let ReflectedOrTransmited(ray:RayForward, intersection) =
    //Use the bool to generate a reflected or transmitted ray
    let fresnelbool = intersection.material.Fresnel

    if fresnelbool then TransmittedRay(ray, intersection)
    else ReflectedRay(ray, intersection)
//
//
//
//
//








let Cast_Forward (scene:scene,ray,grid:Grid3D list,sensor:Sensor)= 
    // This function does the intersection as 3DGrid and at the same time intersects with the Sensor
    // If: finds that the closest intersection is the sensor, gives the sensor intersection
    let Cast_Sensor = 
        //Things to do in Cast Sensor: transform the Sensor into a Square sensor mesh
        let vmesh =[sensor.Origin;              //The best is to define them in the sensor type
                    sensor.c1; 
                    sensor.c2 ]
        let smesh = [1;2;3]
        let mesh = {Vertices = vmesh;
                    Triangles = [smesh]; 
                    material ={DiffuseLight=Color.Zero; SpecularLight=Color.Zero;shinness=0;T=0.;R=0.;n=0.;Fresnel = false};
                    normals = [sensor.Normal];
                    Bbox={Pmin=[];Pmax=[]}
                     }
        //type mesh = {Vertices:Point3D list ; Triangles: int list list;  material:material;normals: UnitVector3D list;Bbox:BBox}
        
        intersec_mesh (ray,  mesh,smesh,sensor.Normal, 0, 's')
        //RotationMatrix:Matrix<float>

    //let gridcasting = Cast_3DGrid (scene,ray,grid)
    //printfn "gridcast: %+A \n sens: %+A \n ray: %+A" gridcasting  Cast_Sensor ray
    
    // Select where is the first intersection: the Sensor or another object of the scene 
    match Cast_Sensor with
    | [] -> let aa = Cast_3DGrid (scene,ray,grid)
            //printfn "\n ray: %+A" aa
            // In not arrives to sensor, Cast other objects
            match aa with 
            | [] -> //printfn "HERE!"
                    //printfn "ray: %+A" ray
                    []
            | _ ->  //printfn "HUY!!!"
                    [aa|>List.minBy(fun x -> x.t)]
    | _ ->  let aa = Cast_3DGrid (scene,ray,grid)
            //printfn "The intersect Point is: %+A" Cast_Sensor.[0].point
            //printfn "\n ray: %+A" aa
            match aa with
            | [] -> Cast_Sensor
            | _ -> let Intmin =[aa |>List.minBy(fun x -> x.t)]
                   let tmin = Intmin.[0].t
                   
                   if tmin < Cast_Sensor.[0].t then Intmin
                   else Cast_Sensor
   

let rec forward(scene:scene, partition:Grid3D list, rayforward:RayForward, sensor:Sensor) =
    //  One point of including sensor is that I don't want to modify scene I already have on backward ray tracing
    //  Thus, I stablish that Nsamples = 0 on intersection means the intersection with sensor
    
    let rayfromType={uvec=rayforward.uvec; length=rayforward.mlength; from=rayforward.from;travelled=rayforward.travelled}              
    let intersection = Cast_Forward// Casting for that
                                   (scene,
                                   rayfromType,
                                    partition,
                                    sensor)
    match intersection with
    | [] -> sensor
    | _ -> let minInter = intersection|>List.minBy(fun x -> x.t)
           // 3 options:
           // 1-  Finds the sensor and deposites the energy
           // 2 - The ray finds an object and takes the color of the object
           // 3-  the ray travels more than allowed and nothing happens
           if minInter.Nsamples = 0 then                            // Nsamples = 0 means the output is the sensor
               //printfn "The intersect Point is: %+A \n and the ray is: %+A" minInter.point rayforward.from
               sensorUpdate(sensor,minInter.point,rayforward)
           elif minInter.ray.travelled < minInter.ray.length  then  // Or a counter of bounces....
               let addcolor = minInter.material.SpecularLight //GlobalIllum(intersects |>List.minBy(fun x -> x.t), Scene ,partition 
               //
               // Generate ray or Rays
               //   To start generate only one
               //
               //let lightDir = rayforward.uvec // Direction of the ray
               //let Reflvect = -minInter.normal.ScaleBy(2.0*minInter.normal.DotProduct(lightDir))+lightDir //Inverted - Reflected ray
               let newvect = ReflectedOrTransmited(rayforward, minInter)
               printfn "vect of intersection: %+A " newvect
               let ReflIntensity = rayforward.intensity*minInter.material.R
               //
               //
               // ----------Think about how to simulte the color here--------------- cannot be color*addcolor

               //
               let nrayforward = {uvec=newvect; mlength=rayforward.mlength; 
                                  from=minInter.point; travelled=minInter.t; 
                                  color=rayforward.color*addcolor; intensity=ReflIntensity}
               forward(scene, partition, nrayforward, sensor)                               // Recursivity
           else // out of range
               sensor
            

//////////////////////////////||
//                            ||
//Initiate the world          ||
//                            ||
//////////////////////////////||
//let clight = {Centre=Point3D(-0.00001,5.0,5.0); Normal=UnitVector3D(0.0000001,-1.000000,-1.0);
//               Radius=0.10;Area=PI*0.1*0.1; 
//               color=Color(1.0,1.0,0.0); 
//               intensityDensity=30.;
//               RotationMatrix=Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),UnitVector3D(0.0,-0.0,-1.0))
//               }
let clight = {Centre=Point3D(-0.0,0.0,5.0); Normal=UnitVector3D(1e-14,1e-14,-1.0);
               Radius=0.40;Area=PI*0.4*0.4; 
               color=Color(1.0,1.0,0.0); 
               intensityDensity=10000.;
               RotationMatrix=Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),UnitVector3D(0.0,-0.0,-1.0))
               }
let lights ={Point= [];Circle = [clight]} //light;light2;light3;light4;light5

//  Mesh
//let whitte ={DiffuseLight = Color(1.0,1.0,1.0);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=1.0; T=0.0; n= 1.45;Fresnel = false}
//let path2 = @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\MeshSamples\Trueplane.obj"
//let mesh1 = ReadMeshWavefront(path2,whitte) |> Mesh_BBox 

//  Sphere
let Glass ={DiffuseLight = Color(0.90,0.90,0.990);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.50; T=0.50; n= 1.3;Fresnel = true}
let sph = {center = Point3D(0.0,0.0,0.0) ; radius = 0.25 ; material = Glass}

let all = {Meshes = [];Sphere = [sph]} //for sphere
//let all = {Meshes = [mesh1];Sphere = []} //Mesh

let partition = Partitionate (all, 2)
let camera={EyePoint=Point3D(-20.,0.,0.);LookAt=Vector3D(992.,1.e-10,1.e-10); Up=Vector3D(1.,1.,1.)} // SHOULD NOT
let scene = {Camera=camera ;World = all; Light=lights; Nsamples=1} 

let xpix = 100
let ypix = 100
let pixsize = 0.01
let photosSature = 10

let normal= UnitVector3D(0.000000001,0.0,1.0)
let origin = Point3D(-0.50,-0.50,-0.25141666666666666) // for sphere
//let origin = Point3D(-0.50,-5.0,5.50)  //
//
//let BlackSensor = InitiateSensor (origin:Point3D, normal, xpix, ypix, pixsize, photosSature)
let MaxNrays = 5000
let image = 
    let mutable startSensor = InitiateSensor (origin, normal, xpix, ypix, pixsize, photosSature)
    for i in 0..(MaxNrays-1) do
        let rayforward = genRayFoward(clight,scene.Nsamples)
        startSensor <- forward(scene, partition, rayforward, startSensor)
    startSensor
//image.Origin
//image.c2
let path = @"C:\Users\JoseM\Desktop\ForwardTest.jpg"
SensorToImage(image, path)
(*
let rayforward = genRayFoward(clight,5)
let ryiwant = {uvec= UnitVector3D(0.001,0.0001,-1.0);length= infinity; from = Point3D(0.0001,0.0001,1.0);travelled= 0.0}
//let aa = Cast_3DGrid (scene, ryiwant,partition)
//let  aa = CastRay_nest(scene, ryiwant) 
let  aa = castRay_mesh(scene, ryiwant) 
scene.World.Meshes.[0].Bbox
partition.[4]

let Cast_3DGrid2 (scene,ray,grid:Grid3D list) =
    let listGrid = partitiongrid//scene.Grid3D


////
////
open BBox

let  Grid_IDList (i:int ,grid:Grid3D list, ray) =
    let bool = BBox_intersec( grid.[i].Bbox, ray)
    printfn "%+A \n ray: %+A" grid.[i].Bbox ray
    if bool then [i]
    else []

Grid_IDList(0, partition,ryiwant)
partition.[3]
///
///
let IntListGrid = [0..(listGrid.Length-1)]      // Gives the list of partitions that are intersected
                    |> List.collect(fun x -> Grid_IDList(0, listGrid,ray)) // index of grid with intersections
                    |> List.collect(fun x ->[listGrid.[x]])              //Partitions that intersect

    // Do the intersections with the spheres and meshes
    //printfn "%+A" IntListGrid
    IntListGrid 
*)