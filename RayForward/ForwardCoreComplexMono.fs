module ForwardCoreComplexMono

open System.IO
open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml


open RayType
open RayTypeMethods
open RayCore
//open RayColorGrid
open RayCoreGrid
open PreprocesorGrid
open RandomMethods
open GaussianSource
open ForwardCore
let PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062



let rec genGaussRayMaxRad (w0:float ,                         // Waist size
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



//      ///     ///     ///     ///     ///
//      Forward ray casting for complex propagation
//      ///     ///     ///     ///     ///




let Cast_ComplexMono (scene:scene,ray,grid:Grid3D list,sensor:ComplexSensor)= 
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
        
        intersec_mesh (ray,  mesh,smesh,sensor.Normal, 0, 's') |> List.filter(fun x -> x.t>0.)
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
   

let rec forwardComplexMono(scene:scene, partition:Grid3D list, rayforward:RayForward, sensor:ComplexSensor,wl:float) =
    //  One point of including sensor is that I don't want to modify scene I already have on backward ray tracing
    //  Thus, I stablish that Nsamples = 0 on intersection means the intersection with sensor
    
    let rayfromType={uvec=rayforward.uvec; length=rayforward.mlength; from=rayforward.from;travelled=rayforward.travelled}              
    let intersection = Cast_ComplexMono// Casting for that
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
           //printfn "\n discante: %+A" minInter.t
           if minInter.Nsamples = 0 then                            // Nsamples = 0 means the output is the sensor
               //printfn "The intersect Point is: %+A \n and the ray is: %+A" minInter.point rayforward.from
               let nrayforward = {uvec=rayforward.uvec; mlength=rayforward.mlength; 
                                  from=rayforward.from; travelled=(minInter.t+rayforward.travelled); 
                                  color=rayforward.color; intensity=rayforward.intensity}
               //ComplexMonosensorUpdate(sensor,minInter.point,nrayforward,wl) //
               //   Complex sensor
               ComplexsensorUpdate(sensor,minInter.point,nrayforward,wl)
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

               let nrayforward = newvect
                (*   if minInter.material.Fresnel = true then                                
                        {uvec=newvect; mlength=rayforward.mlength; 
                         from=minInter.point; travelled=(minInter.t+rayforward.travelled); 
                         color=rayforward.color*addcolor; intensity=ReflIntensity}
                   else 
                        {uvec=newvect; mlength=rayforward.mlength; 
                         from=minInter.point; travelled=(minInter.t+rayforward.travelled+(wl/2.)); 
                         color=rayforward.color*addcolor; intensity=ReflIntensity}
                *) 


               forwardComplexMono(scene, partition, nrayforward, sensor,wl)                               // Recursivity
           else // out of range
               sensor
