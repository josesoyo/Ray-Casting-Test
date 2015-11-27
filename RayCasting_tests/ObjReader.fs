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
    //
    //let bouncing =BBox_creation (v_list) // should be 0's and do it after transform...
    //
    //mesh = {Vertices:Point3D list ; Triangles: int list list; material:material}
    {Vertices=v_list;Triangles = t_list;  material=mat ;normals = n_list; Bbox = {Pmin=[0.;0.;0.]; Pmax=[0.;0.;0.]}}

    
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
  {Vertices = nvert(mesh.Vertices, scaling); Triangles = mesh.Triangles; material = mesh.material; normals= mesh.normals;Bbox=mesh.Bbox}

// Translate
let Translate mesh tvec =
  // tvec = translation vector: vector3d
  // mesh
  let nvert( vert:Point3D list, tvec:Vector3D) =
    vert |> List.collect(fun x -> [x + tvec])
  {Vertices = nvert(mesh.Vertices, tvec); Triangles = mesh.Triangles; material = mesh.material; normals= mesh.normals;Bbox=mesh.Bbox}

let Mesh_BBox (mesh:mesh) =
  //Compute the real BBbox
  // Must be computed once the object is finally placed
  let bouncing = BBox_creation (mesh.Vertices) // should be 0's and do it after transform...
  {Vertices = mesh.Vertices; Triangles = mesh.Triangles; material = mesh.material; normals= mesh.normals;Bbox=bouncing}

let RotateMesh mesh nrm =
  // Rotate the mesh
  // mesh
  let rotateSource (nrm:UnitVector3D) =
    //Generate a rotation matrix
    Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),nrm)
  let matRot = rotateSource(nrm) 
  let rvert( vert:Point3D list, nrm:UnitVector3D, matRot) =
    
    vert |> List.collect(fun x -> [x.TransformBy(m=matRot)])

  let rnormals ( normals:UnitVector3D list, nrm:UnitVector3D, matRot: Matrix<float>) =
    normals |> List.collect(fun x -> [x.TransformBy(m=matRot).Normalize()])

  {Vertices = rvert(mesh.Vertices, nrm,matRot); 
   Triangles = mesh.Triangles; 
   material = mesh.material; 
   normals= rnormals(mesh.normals, nrm,matRot);
   Bbox=mesh.Bbox}

