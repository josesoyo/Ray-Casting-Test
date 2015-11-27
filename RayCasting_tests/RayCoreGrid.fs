module RayCoreGrid

open System
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayType
open BBox 
open RayCore

let PinBBox(point: Point3D,box : BBox)=
    let xc =
        if point.X < box.Pmax.[0] && point.X > box.Pmin.[0] then true
        else false
    let yc =
        if point.Y < box.Pmax.[1] && point.Y > box.Pmin.[1] then true
        else false
    let zc =
        if point.Z < box.Pmax.[2] && point.Z > box.Pmin.[2] then true
        else false
    if xc && yc && zc then true
    else false

let IntersecinGrid (box:BBox, inter: Intersection) =
    let IsinGrid = PinBBox(inter.point,box)
    if IsinGrid then [inter]
    else []
let  Grid_IDList (i:int ,grid:Grid3D list, ray) =
    let bool = BBox_intersec( grid.[i].Bbox, ray)
    if bool then [i]
    else []
let  IDList (i:int ,box:BBox, ray) =
    let bool = BBox_intersec( box, ray)
    if bool then [i]
    else []

let InsidePartition (igrid:Grid3D , scene:scene, ray) =
    // do the BBox intersection with the spheres and meshes of the partition
    let IntListMesh = // List of positions of meshes that have an intersection
        [0..(igrid.MeshID.Length-1)] 
        |> List.collect(fun x -> IDList(x, igrid.MBBox.[x],ray)) // index of grid with intersections
    let IntListSph =
        [0..(igrid.SphID.Length-1)] 
        |> List.collect(fun x -> IDList(x, igrid.SphBox.[x],ray))

    //Interception routines
    let minterceptions (ray:RayFrom, mesh:mesh,triangles:int list list,normals:UnitVector3D list ,nsamples:int)  = // interceptions modified
        [0..(List.length(triangles)-1)]
        |> List.collect (fun x -> intersec_tri(ray, mesh, triangles.[x],normals.[x],nsamples))
        |> List.filter (fun x -> x.t > 0.01)  //
    let sinterceptions (sph: sphere, ray:RayFrom, nsamples:int) =  //CastRay Modified
        intersection(ray, sph,nsamples)                  // Find interstion with of  one sphere
        |> List.filter (fun x -> x.t > 0.01)             // Select the ones that are not negative-€[e,infinity)

    //Do the interceptions
    let TrianglesIntercepted = // Triangles intersected
        IntListMesh
        |> List.collect (fun x -> minterceptions (ray, scene.World.Meshes.[igrid.MeshID.[x]],igrid.MeshTriangles.[x],igrid.MeshNormals.[x] ,scene.Nsamples) )
        |> List.collect(fun x -> IntersecinGrid (igrid.Bbox, x) )

    let SpheresIntercepted = //Spheres intesected
        IntListSph
        |> List.collect(fun x -> sinterceptions((scene.World.Sphere.[igrid.SphID.[x]], ray, scene.Nsamples) ))

    List.append TrianglesIntercepted  SpheresIntercepted


    

let Cast_3DGrid (scene,ray,grid:Grid3D list) =
    let listGrid = grid//scene.Grid3D

    let IntListGrid = [0..(listGrid.Length-1)]      // Gives the list of partitions that are intersected
                      |> List.collect(fun x -> Grid_IDList(x, listGrid,ray)) // index of grid with intersections
                      |> List.collect(fun x ->[listGrid.[x]])              //Partitions that intersect

    // Do the intersections with the spheres and meshes
    //printfn "%+A" IntListGrid
    IntListGrid |> List.collect(fun x -> InsidePartition (x, scene, ray))

