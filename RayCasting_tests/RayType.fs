module RayType

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml


// Sensor defined on Sensor.fs

//Box type
type BBox = {Pmin :float list; Pmax:float list}


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

// Lights projected to be done:
//  *PLight Spot light with a direction and cosd to cut at max angle and p to directionality if 0, then tipical
//  *Clight: Circle light
//          centre of the cercle, normal, radius, density of intesity: <W/m^2>  or <w>?
//  *Slight: Square light -> to be done in the future
//  *InfLight: Light from infinity= parallel
//type Slight = {origin:Point3D; color:Color; intensity:float; Direction:UnitVector3D; p:int;cosd:float}

// Point light
type Plight = {origin:Point3D; color:Color; intensity:float} 
// Direction:UnitVector3D; p:int;cosd:float}  // => For the day I want it directional

//Circular light
type Clight = {Centre:Point3D;                      // Centre of the light-disc
               Normal:UnitVector3D;                 // Direction in which the light points
               Radius:float;Area:float;             // Radius and Area (are not independent)
               color:Color; intensityDensity:float; // Color of the light and its intensity
               RotationMatrix:Matrix<float>         // Rotation from the UnitVector3D(0.,0.,1.) -> Normal
               }
type light = {Point: Plight list; Circle: Clight list}//;Square: Slight list }
//
//

//Ray Types
type RayFrom = {uvec:UnitVector3D; length: float; from:Point3D; travelled:float} //length is only for light intersection not infinity
type RayForward = {uvec:UnitVector3D; mlength: float; from:Point3D; travelled:float; color:Color; intensity:float}
//
(* 
type RayGaussian = {uvec:UnitVector3D; mlength: float; from:Point3D; 
                    travelled:float;        // Works as Phase - to compute it
                    intensity:int ;         // Works as a number of photons
                    wl:float                // To compute the phase from dist
                    }
*)

// Objects that can be intersecterd: Mesh and spheres
type sphere = {center:Point3D; radius:float; material:material }
type partSphere = {Sphere:sphere; 
                   zmin:float;zmax:float;
                   phimin:float;phimax:float;
                   ObjToWorld:Matrix<float>;  // Transforms from (0.,0.,1.) -> Normal
                   WorldToObj:Matrix<float>  // Transforms from Normal -> (0.,0.,1.)                   
                  }
type SphSurfaceLens = { Sphere:sphere ;
                        CosMin: float;
                        Axis: UnitVector3D;
                        Convex:bool; // Convex = convergent (normal sphere) 
                      } // Name because will be the surface of a lens
                      
                     

type mesh = {Vertices:Point3D list ; Triangles: int list list;  material:material;normals: UnitVector3D list;Bbox:BBox}
type cylinder = {Radius: float;             // Radius of the cylinder
                 zmin:float; zmax:float;    // By definition in object space z aligned
                 LBbox:BBox; WBbox:BBox;    // Local and world Bounding box
                 Origin:Point3D             // Origin of the axis in real world (Translation)
                 Normal:UnitVector3D;       // Direction of the cylinder
                 material:material
                 ObjToWorld:Matrix<float>;  // Transforms from (0.,0.,1.) -> Normal
                 WorldToObj:Matrix<float>  // Transforms from Normal -> (0.,0.,1.)
                 }

(* Before BBbox
type mesh = {Vertices:Point3D list ; Triangles: int list list;  material:material;normals: UnitVector3D list}

*)

type cam = {EyePoint:Point3D; LookAt:Vector3D; Up:Vector3D}//; film:Sensor}
type world = {Meshes: mesh list;
              Sphere:sphere list; PartSphere:partSphere list ; SurfaceLens: SphSurfaceLens list;
              Cylinder: cylinder list}



// Type of the partition of the world - Uses: Cast_3DGrid (Scene,Ray,partition)
type Grid3D= {Bbox:BBox                                                            //BBox of the 3DGrid
              MeshID:int list; MeshTriangles: int list list list;MeshNormals:UnitVector3D list list ; MBBox:BBox list;  //mesh list
              SphID:int list; SphBox:BBox list;                                    //spheres list
              }
              // Define Octree type

type OctreeSystem=
    |Octree of Octree
    |Partition of Grid3D
and Octree = {Bbox:BBox; Octree: OctreeSystem list}


type scene = {Camera:cam; World:world; Light:light; Nsamples:int} //I create a list of Spheres - Nsamples for monteCarlo Methods
//type scene = {Camera:cam; Sphere:sphere list; EndWorld:wall; Light:light list} //I create a list of Spheres




type Intersection = { normal:UnitVector3D; point:Point3D; ray:RayFrom; material:material;t:float; 
                      Nsamples:int} //Nsamples = 0 means intersection with Sensor
//First intersection type - Save the relics
//type Intersection = { normal:UnitVector3D; point:Point3D; ray:RayFrom; sphere:sphere;t:float}//; t:float }