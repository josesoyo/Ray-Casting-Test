module RayCore

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayType


///////////////////////////////////////////////////
//
//  castRAy dapted from martindoms.com
//
/// Get the position of a ray at a given time

let pointAtTime (rayfrom:RayFrom, time:float) =
    rayfrom.from + rayfrom.uvec.ScaleBy(time) 
    //Ray3D.Direction.ScaleBy -> to multiply by a float

let intersection(ray:RayFrom,sphere:sphere)=
    //let s = ray.origin - scene.Sphere.center //raycast for sphere
    let s = ray.from - sphere.center //raycast for sphere
    let rayDir = ray.uvec //ray.Direction// Normalized direction of the ray3D
    let sv = rayDir.DotProduct(s)
    let ss = s.DotProduct(s)
    let discr = sv*sv - ss + sphere.radius*sphere.radius
    if discr < 0.0 then []
    else
        let normalAtTime t = UnitVector3D((pointAtTime( ray, t) - sphere.center).X,(pointAtTime( ray, t) - sphere.center).Y,(pointAtTime( ray, t) - sphere.center).Z)//sphere.radius //normalized with the sphere.radius
        let (t1,t2) = (-sv + sqrt(discr), -sv - sqrt(discr)) //times at which the ray is intercepted
        let (travel1,travel2) = ((t1+ray.travelled),(t2+ray.travelled)) //Update distance travelled by the ray
        let (ray1,ray2) = ({uvec=ray.uvec; length=ray.length; from=ray.from; travelled=travel1},{uvec=ray.uvec; length=ray.length; from=ray.from; travelled=travel2})
        let intersects = [ { normal = normalAtTime t1 ; point = pointAtTime (ray, t1); ray = ray1 ; sphere=sphere; t=t1};
                           { normal = normalAtTime t2 ; point = pointAtTime (ray, t2); ray = ray2 ; sphere=sphere;t=t2}]
        //printfn "t value: %f %f" intersects.Head.t intersects.[1].t
        intersects 

let castRay (scene:scene, ray:RayFrom) = 
    scene.Sphere 
    |> List.collect (fun x -> intersection(ray, x))  // Find interstion with all the spheres
    |> List.filter (fun x -> x.t > 0.01)             // Select the ones that are not negative-€[e,infinity)
