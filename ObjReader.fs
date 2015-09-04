//
// I consider in this module everything that is done to the mesh before it becomes into scene type
// ie:
//      *Read .obj
//      *Transforms: Scale, Translate, Rotate
//
module ObjReader


open System.IO
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml
open RayType
open BBox 
//
// Functions to read the Wavefront file
//


let VerfromFile (seq:string)=
    if seq.StartsWith("v") then
        //printfn "tenemos un vector!: %s" seq
        let a= seq.Substring(2)
        //printfn "tenemos los valores : %s" a
        let b = a.Split(' ')
        // Convert the array into a list and string to float
        //printfn "input: %+A" a
        //printfn "input: %+A" b
        let (x,y,z) = (float(b.[0]),float(b.[1]),float(b.[2]))//(float(b.[1]),float(b.[2]),float(b.[0]))
        let pos = Point3D(x, y, z)
        [pos]
    else
        []
        //printfn "%A" pos
        

let TrifromFile (seq:string) =
    if seq.StartsWith("f") then
        //printfn "tenemos un vector!: %s" seq
        let a= seq.Substring(2)
        //printfn "tenemos los valores : %s" a
        let b = a.Split(' ')
        // Convert the array into a list and string to float
        let (x,y,z) =(int(b.[0]),int(b.[1]),int(b.[2])) //(int(b.[1]),int(b.[2]),int(b.[0]))
        let pos = [x; y; z]
        [pos]
    else
        [[]]    //    printfn "tenemos un triangulo: %s" seq 

let MeshNormals(nodes:Point3D list, triangle:int list list) =
    let mutable normals =[UnitVector3D(0.0,0.0,1.0)]
    //printfn "triangles %d" triangle.Length
    for i in 0..triangle.Length-1 do
       
        let (n0,n1,n2) = ((triangle.[i].[0])-1, (triangle.[i].[1])-1, (triangle.[i].[2])-1)
        //printfn "%d %d %d" n0 n1 n2
        let (u0u1,u0u2) = (nodes.[n1]-nodes.[n0],nodes.[n2]-nodes.[n0]) //must be cyclic
        // normal of the triangle and associate Plane 

        normals <- normals@[(u0u1.CrossProduct(u0u2)).Normalize()]       
    normals.Tail


let ReadMeshWavefront(path:string,mat:material)=
    let lines = File.ReadLines(path)    
    let list_lines = Seq.toList(lines)
    let v_list = list_lines |>List.collect(fun x -> VerfromFile x) //|>List.filter(fun x -> x.Length > 0)
    let t_list = list_lines |>List.collect(fun x -> TrifromFile x) |>List.filter(fun x -> x.Length > 0)
    let n_list = MeshNormals( v_list, t_list)
    let bouncing =BBox_creation (v_list)
    //mesh = {Vertices:Point3D list ; Triangles: int list list; material:material}
    {Vertices=v_list;Triangles = t_list;  material=mat ;normals = n_list; Bbox = bouncing}

    
///////////////////
//
// Transformations
//
//////////////////


//Scale

let Scale mesh scaling =
  // scaling = 3d
  let nvert(vert:Point3D list, scaling:float list) =
    vert 
    |> List.collect(fun x -> [Point3D(x.X*scaling.[0],x.Y*scaling.[1],x.Z*scaling.[2])])   
  let nBBOX (Bbox:BBox, scaling:float list) =
    // {Pmin =[pxmin;pymin;pzmin];Pmax=[pxmax;pymax;pzmax]}
    let nmin = [Bbox.Pmin.[0]*scaling.[0];Bbox.Pmin.[1]*scaling.[1];Bbox.Pmin.[2]*scaling.[2]]
    let nmax = [Bbox.Pmax.[0]*scaling.[0];Bbox.Pmax.[1]*scaling.[1];Bbox.Pmax.[2]*scaling.[2]]
    {Pmin=nmin;Pmax=nmax}
  let nbox = nBBOX(mesh.Bbox, scaling)
  {Vertices = nvert(mesh.Vertices, scaling); Triangles = mesh.Triangles; material = mesh.material; normals= mesh.normals;Bbox=nbox}

// Translate
let Translate mesh tvec =
  // tvec = translation vector: vector3d
  // mesh
  let nBBOX (Bbox:BBox, tvec:Vector3D) =
    // {Pmin =[pxmin;pymin;pzmin];Pmax=[pxmax;pymax;pzmax]}
    let nmin = [Bbox.Pmin.[0]+tvec.X;Bbox.Pmin.[1]+tvec.Y;Bbox.Pmin.[2]+tvec.Z]
    let nmax = [Bbox.Pmax.[0]+tvec.X;Bbox.Pmax.[1]+tvec.Y;Bbox.Pmax.[2]+tvec.Z]
    {Pmin=nmin;Pmax=nmax}
  let nvert( vert:Point3D list, tvec:Vector3D) =
    vert |> List.collect(fun x -> [x + tvec]) 
  let nbox = nBBOX(mesh.Bbox, tvec)
  {Vertices = nvert(mesh.Vertices, tvec); Triangles = mesh.Triangles; material = mesh.material; normals= mesh.normals;Bbox=nbox}
