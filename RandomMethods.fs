module RandomMethods

open System
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayType
open BBox 

// Script to check the the use of PRNG 
//
// Up to now there're methods written to sample:
//  -Unit circle method:      SampUnitDisk
//  -unit Hemisphere Method:  SampUnitHemisphereToCart
//  -triangle (u,v):          SampTriangle

open MathNet.Numerics.Random;
let PI = 3.141592653589

// TRansfor it to a vecctor3D
let SampUnitHemisphereToCart (dummy:int) =
    // Generate a PRandom unit vector  in 3D in +Z
    // Not efficient
    // For Lambertian dispersion
    let (phi, ctheta)  =
        let samples = Random.doubles 2
        let sol = (2.0*PI*samples.[0],samples.[1])
        //printfn "%+A" samples
        sol
    let stheta = sqrt(1.-(pown ctheta 2))
    let x = (sin phi )*stheta 
    let y = (cos phi) *stheta 
    let z = ctheta
    //let modul = (x*x)+(y*y)+(z*z)
    (x,y,z)

let SampDisk (radius:float) =
    let samples = Random.doubles 2
    let r = radius* sqrt (samples.[0])
    let theta = 2.*PI*samples.[1]
    (r,theta)

let SampUnitHemiCosToCart (dummy:int) =
    // Hemisphere cosine weighted
    let (r,theta) = SampDisk (1.)
    let (x,y) = (r*cos theta , r*sin theta)
    let z = sqrt(max 0.0 (1.-x*x-y*y))
    //let modul = x*x+y*y+z*z
    (x,y,z)//,modul)


let SampUnitDisk (dummy:int) =
    // unit dist
    let samples = Random.doubles 2
    let ru = sqrt samples.[0]
    let theta = 2.*PI*samples.[1]
    (ru,theta,dummy)

let SampTriangle (dummy:int) =
    // u v to random sample a triangle
    let samples = Random.doubles 2
    let squ = sqrt samples.[0]
    (1.-squ, squ*samples.[1])    


