module GaussianSource

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayType
open RandomMethods
//This module is going to be used to modelize all the Gaussian Source type and functions
//
//  1 - Type GaussianRay
//  2 - Create the GaussianRay with the basic things required: 
//          waist size and position 




//Ray Types
//type RayFrom = {uvec:UnitVector3D; length: float; from:Point3D; travelled:float} //length is only for light intersection not infinity
//type RayForward = {uvec:UnitVector3D; mlength: float; from:Point3D; travelled:float; color:Color; intensity:float}
type RayGaussian = {uvec:UnitVector3D; mlength: float; from:Point3D; travelled:float; intensity:int }//  ;wl:1e-6}
// RayFrom + Intensity = intefer because is the number of photons

let generatePsRandGausianRay (w0:float , z0:float, origin:Point3D, normal:UnitVector3D) =
    // To genreate a pseudo random gausian ray it is required two parameters:
    // waist size and distance from source to waist and the direction of the source (Transformation matrix)
    let randomVect = Samp2DGauss ( w0,0.0)
