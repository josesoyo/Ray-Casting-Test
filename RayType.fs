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
    member this.Y = Ypos 
*)
type Sensor = {//This should be the type
               Centre:Point3D;
               Normal:UnitVector3D; // Direction of the sensor = Dir of camera
               xpix :int; ypix: int // (1,0,0) y (0,1,0) -> Rotated
               pixsize:float // Suposed rectangular
               //Up not required since it's up +Y in local coordinates - CAREFULL
               RotationMatrix:Matrix<float> // From local coordinates to global coordinates
               
               //Real sensor properties
               PhotosSature: int // Number of photons that saturate the image
               Rcolor:Matrix<int> // Saves number of photons received
               Gcolor:Matrix<int>
               Bcolor:Matrix<int>
               }


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

//let a = Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),UnitVector3D(0.5,0.5,0.))
//type wall = {surf:Plane;Color:float} //  Endworld
type material = {DiffuseLight:Color; SpecularLight:Color; shinness:int;R:float; T:float; n:float; Fresnel:bool} //Shinnes: exponent to phong model
// Lights:
//  *PLight Spot light with a direction and cosd to cut at max angle and p to directionality if 0, then tipical
//  *Clight: Circle light
//          centre of the cercle, normal, radius, density of intesity: <W/m^2>  or <w>?
//  *Slight: Square light -> to be done in the future
//  *InfLight: Light from infinity= parallel
type Plight = {origin:Point3D; color:Color; intensity:float} // Direction:UnitVector3D; p:int;cosd:float}
//type Slight = {origin:Point3D; color:Color; intensity:float; Direction:UnitVector3D; p:int;cosd:float}
type Clight = {Centre:Point3D;Normal:UnitVector3D;Radius:float;Area:float; color:Color; intensityDensity:float; RotationMatrix:Matrix<float>}
type light = {Point: Plight list; Circle: Clight list}//;Square: Slight list }


type cam = {EyePoint:Point3D; LookAt:Vector3D; Up:Vector3D}//; film:Sensor}
type sphere = {center:Point3D; radius:float; material:material }
//type scene = {Camera:cam; Sphere:sphere list; EndWorld:wall; Light:light list} //I create a list of Spheres
type RayFrom = {uvec:UnitVector3D; length: float; from:Point3D; travelled:float} //length is only for light intersection not infinity
type RayForward = {uvec:UnitVector3D; mlength: float; from:Point3D; travelled:float; color:Color; intensity:float}
(* TEst de BBox
type mesh = {Vertices:Point3D list ; Triangles: int list list;  material:material;normals: UnitVector3D list}

*)
type BBox = {Pmin :float list; Pmax:float list}
type mesh = {Vertices:Point3D list ; Triangles: int list list;  material:material;normals: UnitVector3D list;Bbox:BBox}

type world = {Meshes: mesh list;Sphere:sphere list}


// In one moment will become subworld

type Grid3D= {Bbox:BBox                                                            //BBox of the 3DGrid
              MeshID:int list; MeshTriangles: int list list list;MBBox:BBox list;  //mesh list
              SphID:int list; SphBox:BBox list;                                    //spheres list
              }

type scene = {Camera:cam; World:world; Light:light; Nsamples:int} //I create a list of Spheres - Nsamples for monteCarlo Methods



//type Intersection = { normal:UnitVector3D; point:Point3D; ray:RayFrom; sphere:sphere;t:float}//; t:float }
type Intersection = { normal:UnitVector3D; point:Point3D; ray:RayFrom; material:material;t:float; Nsamples:int}
