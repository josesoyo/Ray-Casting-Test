module RayCoreGrid

open System
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayType
open BBox 
open RayCore


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
    let minterceptions (ray:RayFrom, mesh:mesh,triangles:int list list, nsamples:int)  = // interceptions modified
        [0..(List.length(triangles)-1)]
        |> List.collect (fun x -> intersec_tri(ray, mesh, triangles.[x],mesh.normals.[x],nsamples))
        |> List.filter (fun x -> x.t > 0.01)  //
    let sinterceptions (sph: sphere, ray:RayFrom, nsamples:int) =  //CastRay Modified
        intersection(ray, sph,nsamples)                  // Find interstion with of  one sphere
        |> List.filter (fun x -> x.t > 0.01)             // Select the ones that are not negative-€[e,infinity)

    //Do the interceptions
    let TrianglesIntercepted = // Triangles intersected
        IntListMesh
        |> List.collect (fun x -> minterceptions (ray, scene.World.Meshes.[igrid.MeshID.[x]],igrid.MeshTriangles.[x], scene.Nsamples) )
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

