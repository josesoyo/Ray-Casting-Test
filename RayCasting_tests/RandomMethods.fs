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
//  -Gaussian Number X2       Samp2Gauss

open MathNet.Numerics.Random;
let PI = double(3.1415926535897932384626433832795028841971693993751058209749445923078164062)

// TRansfor it to a vecctor3D
let SampUnitHemisphereToCart () =
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

let SampUnitHemiCosToCart () =
    // Hemisphere cosine weighted
    let (r,theta) = SampDisk (1.)
    let (x,y) = (r*cos theta , r*sin theta)
    let z = sqrt(max 0.0 (1.-x*x-y*y))
    //let modul = x*x+y*y+z*z
    (x,y,z)//,modul)

let SampUnitDisk () =
    // unit dist
    let samples = Random.doubles 2
    let ru = sqrt samples.[0]
    let theta = 2.*PI*samples.[1]
    (ru,theta)

let SampTriangle () =
    // u v to random sample a triangle
    let samples = Random.doubles 2
    let squ = sqrt samples.[0]
    (1.-squ, squ*samples.[1])    



let Samp2DGauss ( sigma:float, mu:float) =
    // Generates a gaussian PRN with 
    //sigma variance and mu decenter 
    let samples = Random.doubles 2
    //log(x) = natural logarithm
    let R = sqrt(-2.*(log(samples.[0])))
    let phi = 2.*PI*samples.[1]
    let z0 = R * cos(phi)    
    let z1 = R * sin(phi)
    [z0*sigma+mu;z1*sigma+mu]


let samp2NGauss (sigma:float, mu:float, N: int) =
    // Gives a list of 2N Gaussian PRN
    [1..N]|>List.collect(fun x -> Samp2DGauss(mu,sigma))

(*
//Test of Gaussian numbers
let many2 = samp2NGauss( 0., 0.5, 100000)
let many = samp2NGauss( 0., 1., 100000)
#r @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\packages\FSharp.Charting.0.90.12\lib\net40\FSharp.Charting.dll"
open FSharp.Charting
open MathNet.Numerics.Statistics

let histo2 = Histogram(many2,50,-5.,5.)                                                //Generate Histogram

let counts2 = [0..(histo.BucketCount-1)]|>List.collect(fun x -> [histo2.[x].Count/histo2.DataCount])    //do the counter - Gaussian
Chart.Combine(
    [Chart.Line(counts, Name= "counts");
    Chart.Line(counts2, Name= "counts2")]
    ).ShowChart()
*)
//Endtest

