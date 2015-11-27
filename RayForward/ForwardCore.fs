module ForwardCore


open System.IO
open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml
open MathNet.Numerics.Random;

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
let PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062



// Generate a Ray Forward from a circular source
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
    let (ux,uy,uz)= SampUnitHemiCosToCart () //  1 = dummy
    let unitV = clight.Normal                      // in the future UnitVector3D(ux,uy,uz)
    let rayForward = {uvec=unitV; 
                      mlength=9000.;          // Works as max length that can travel a Ray
                      from=point; 
                      travelled=0.0; 
                      color=clight.color;   // The color o the light
                      intensity=clight.intensityDensity*clight.Area/float(samples)}
    rayForward


let rec genGaussRayMaxRad  (w0:float ,                         // Waist size
                            distz0:float,                      // Dist. from Waist
                            origin:Point3D,                    // Center of the gausian beam
                            normal:UnitVector3D,               // Direction of the beam
                            rotationMatrix:Matrix<float>,       // Rotate from ez to normal - saves time if precomputed
                            wl:float,
                            maxrad) =
    // Recursive funtion to generate rays from a gaussian source with maximum diameter
    let gray = generatePsRandGausianRay(w0,distz0,origin,normal,rotationMatrix,wl)
    let rad = (gray.from-origin).Length                                             // Dist: ray - centerSource
    // Ray generated should be whithin a certain distance from center
    if rad > maxrad then genGaussRayMaxRad(w0,distz0,origin,normal,rotationMatrix,wl,maxrad)
    else gray


let rec genGaussRayMaxRadY (w0:float ,                         // Waist size
                            distz0:float,                      // Dist. from Waist
                            origin:Point3D,                    // Center of the gausian beam
                            normal:UnitVector3D,               // Direction of the beam
                            rotationMatrix:Matrix<float>,       // Rotate from ez to normal - saves time if precomputed
                            wl:float,
                            maxrad) =
    // Recursive funtion to generate rays from a gaussian source with maximum diameter
    let gray = generatePsRandGausianRayY(w0,distz0,origin,normal,rotationMatrix,wl)
    let rad = (gray.from-origin).Length                                             // Dist: ray - centerSource
    // Ray generated should be whithin a certain distance from center
    if rad > maxrad then genGaussRayMaxRadY(w0,distz0,origin,normal,rotationMatrix,wl,maxrad)
    else gray



//      ///     ///     ///
//      Forward ray casting
//      ///     ///     ///

let ReflectedRay (ray:RayForward, intersection) =
    //It returns the reflected ray
    //
    let pnormal = intersection.normal
    let lightDir = ray.uvec // Direction of the ray
    let normal = 
        if lightDir.DotProduct(pnormal)< 0. then  pnormal
        else  pnormal.Negate()
    let Reflvect = -normal.ScaleBy(2.0*normal.DotProduct(lightDir))+lightDir //Inverted - Reflected ray
    //
    //printfn "%+A" (Reflvect.Normalize())
    let newvect = Reflvect.Normalize()
    {uvec=newvect; mlength=ray.mlength; 
    from=intersection.point; travelled=(intersection.t+ray.travelled+(1.e-6/2.)); 
    color=ray.color; intensity=1.}
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
        let newvect = vtrans.Normalize()
        {uvec=newvect; mlength=ray.mlength; 
         from=intersection.point; travelled=(intersection.t+ray.travelled); 
         color=ray.color; intensity=1.}
    else  ReflectedRay (ray, intersection) // Else reflected
    //
    //

let ReflectedOrTransmited(ray:RayForward, intersection) =
    //Use the bool to generate a reflected or transmitted ray
    let fresnelbool = intersection.material.Fresnel
    //printfn "transmited? %+A" fresnelbool
    if fresnelbool then TransmittedRay(ray, intersection)
    else ReflectedRay(ray, intersection)
let IsReflectedOrTransmited(ray:RayForward, intersection) =
    // reflected or transmitted ray following the probability
    let TransProb = intersection.material.T
    let PSRN = (Random.doubles 1).[0]
    //printfn "transmited? %+A" fresnelbool
    if PSRN < TransProb  then TransmittedRay(ray, intersection)
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
    | [] -> let aa = CastRay_nest (scene,ray) // Cast_3DGrid(scene,ray,grid) or: CastRay_nest
            //printfn "\n ray: %+A" aa
            // In not arrives to sensor, Cast other objects
            match aa with 
            | [] -> //printfn "HERE!"
                    //printfn "ray: %+A" ray
                    []
            | _ ->  //printfn "HUY!!!"
                    [aa|>List.minBy(fun x -> x.t)]
    | _ ->  let aa = CastRay_nest (scene,ray)
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
               let newvect = IsReflectedOrTransmited(rayforward, minInter)
               //printfn "vect of intersection: %+A " newvect
               let ReflIntensity = rayforward.intensity*minInter.material.R
               //
               //
               // ----------Think about how to simulte the color here--------------- cannot be color*addcolor

               //
               let nrayforward = newvect//{uvec=newvect; mlength=rayforward.mlength; 
                                 //from=minInter.point; travelled=minInter.t; 
                                 // color=rayforward.color*addcolor; intensity=ReflIntensity}
               forward(scene, partition, nrayforward, sensor)                               // Recursivity
           else // out of range
               sensor
