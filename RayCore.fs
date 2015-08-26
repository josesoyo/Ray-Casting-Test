module RayCore

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml


// define sensor
(*
type Sensor (psw:float, psh:float, pw:int, ph:int) =
    // psw and psh: pixel size (in natural=artificial units, should be mm)
    // pw and ph: num pixels width and height
    //
    //  REDUNDANT AND USELESS
    //
    //
    let mutable SeArray = DenseMatrix.zero<float> pw ph
    let Xpos = DenseMatrix.init pw ph (fun i j -> (psw*float(pw/2-i)))
    let Ypos = DenseMatrix.init pw ph (fun i j -> (psw*float(ph/2-j)))
    member this.value = SeArray
    member this.X = Xpos
    member this.Y = Ypos *)
//Color type from martindoms (CopyPaste)
type Color(r: float, g: float, b:float) =
    member this.r = r
    member this.g = g
    member this.b = b
    //
    // Now the sum of colors, multilication between them and wit a scalar are defined
    static member ( * ) (c1:Color, c2:Color) =
        Color (c1.r*c2.r, c1.g*c2.g, c1.b*c2.b)
    static member ( * ) (c:Color, s:float) =
        let r = min (c.r*s) 1.0
        let g = min (c.g*s) 1.0
        let b = min (c.b*s) 1.0
        Color (r,g,b)
    static member ( + ) (c1:Color, c2:Color) =
        let r = min (c1.r+c2.r) 1.0
        let g = min (c1.g+c2.g) 1.0
        let b = min (c1.b+c2.b) 1.0
        Color (r,g,b)
    static member Zero = Color(0.0,0.0,0.0)
//

// Types for the world
type wall = {surf:Plane;Color:float} // style from MartinDot // Endworld
type material = {DiffuseLight:Color; SpecularLight:Color; shinness:int;R:float; T:float; n:float} //Shinnes: exponent to phong model
type light = {origin:Point3D; color:Color; intensity:float}
type cam = {EyePoint:Point3D; LookAt:Vector3D; Up:Vector3D}//; film:Sensor}
type sphere = {center:Point3D; radius:float; material:material }
type scene = {Camera:cam; Sphere:sphere list; EndWorld:wall; Light:light list} //I create a list of Spheres
//type RayFrom = {ray:Ray3D; from:Point3D; travelled:float}
type RayFrom = {uvec:UnitVector3D; length: float; from:Point3D; travelled:float}


///////////////////////////////////////////////////
//
//  castRAy dapted from martindoms.com
//
/// Get the position of a ray at a given time

type Intersection = { normal:UnitVector3D; point:Point3D; ray:RayFrom; sphere:sphere;t:float}//; t:float }

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
