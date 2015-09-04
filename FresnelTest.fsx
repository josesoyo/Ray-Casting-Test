#r @"packages\MathNet.Spatial.0.2.0-alpha\lib\net40\MathNet.Spatial.dll"
#r @"packages\MathNet.Numerics.Signed.3.7.0\lib\net40\MathNet.Numerics.dll"
#r @"packages\MathNet.Numerics.FSharp.Signed.3.7.0\lib\net40\MathNet.Numerics.FSharp.dll"
#r @"C:\Users\JoseM\OneDrive\Phd\render\ray casting\RayCastingTest\Ray-Casting-Test\packages\FSharp.Charting.0.90.12\lib\net40\FSharp.Charting.dll"

#load "RayType.fs"
#load "RayCore.fs"
#load "RayColor.fs"

open System.IO
open System.Windows.Forms
open System.Drawing

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml
open FSharp.Charting



open RayColor
let PI = 3.141592653589
let n = 1.45
let normal = UnitVector3D(0.0,1.0,0.0)
let theta_integer = [0.0;0.1;0.2;0.3;0.4;0.5;0.6;0.7;0.8;0.9;0.95;1.0]


let angle = theta_integer 
             |> List.collect (fun x -> [PI*0.5*x]) //radians from pi/2 to few
let frenels = angle
             |> List.collect (fun x -> [UnitVector3D(cos(x), sin(x),0.0)])
             |> List.collect (fun x -> [(cosinus_in_tr(normal, x,n))])
//(ci,cr,nu)

let RFresnel (ci:float, ct:float, nu:float) =
  // ALways taking into account that it's done with material/air interfase 
  // nu = n1/n2 = nTo/nFrom
  // Hall illumination model[HALL83]
  let term1 = pown ((ci/nu-ct*nu)/(ci/nu+ct*nu)) 2
  let term2 = pown ((ci*nu-ct/nu)/(ci*nu+ct/nu)) 2
  if ct = 0. then  (acos(ci), 0.5*(term1+term2)) 
  else (acos(ci),0.5*(term1+term2))


let R = frenels |> List.collect (fun x -> [RFresnel x]) // T, R
RFresnel(1., 1., 1.45)
(0.0,1.0)
Chart.Combine(
    [Chart.Line(T, Name= "Transmitivity");
    Chart.Line(R, Name= "Reflectivity")]).ShowChart()
R