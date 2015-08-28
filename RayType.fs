module RayType


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


type wall = {surf:Plane;Color:float} // style from MartinDot // Endworld
type material = {DiffuseLight:Color; SpecularLight:Color; shinness:int;R:float; T:float; n:float} //Shinnes: exponent to phong model
type light = {origin:Point3D; color:Color; intensity:float}
type cam = {EyePoint:Point3D; LookAt:Vector3D; Up:Vector3D}//; film:Sensor}
type sphere = {center:Point3D; radius:float; material:material }
//type scene = {Camera:cam; Sphere:sphere list; EndWorld:wall; Light:light list} //I create a list of Spheres
type RayFrom = {uvec:UnitVector3D; length: float; from:Point3D; travelled:float}
type mesh = {Vertices:Point3D list ; Triangles: int list list;  material:material;normals: UnitVector3D list}

type world = {Meshes: mesh list;Sphere:sphere list}
type scene = {Camera:cam; World:world; Light:light list} //I create a list of Spheres



//type Intersection = { normal:UnitVector3D; point:Point3D; ray:RayFrom; sphere:sphere;t:float}//; t:float }
type Intersection = { normal:UnitVector3D; point:Point3D; ray:RayFrom; material:material;t:float}