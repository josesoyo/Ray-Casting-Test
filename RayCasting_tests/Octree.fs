module Octree

open System.IO
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml
open RayType
open BBox 
open RayCore
open PreprocesorGrid
open RayCoreGrid


//  Pseudocode:
//
// Check world limits with a slight extra on max (1e-6 should work)
// let rec Octree (world, box)
//  Partitionate
//  Generate an empty octreeList (OCtreeSystem)
//
//  Iterate partitions
//    Fill partition with the elements included in it
//    if elementsInPartition > limElem then      
//        OctreeList <- OctreeList@Octree(partition,BBoxPartion)
//    else
//        OctreeList <- OctreeList@[Partition]
//
//  I cannot subdivide the mesh because I'll create problems between the VerticeID and thre IDinTriagle...
//  The result may produce a slow subdivisiona algorithm...
//
let rec OctreeSubdivision (world:world, subdivision:BBox, maxEle:int, depth:int, maxdepth:int) =
    // Subdivision of the octree
    let stepx = (subdivision.Pmax.[0]-subdivision.Pmin.[0])/2.
    let stepy = (subdivision.Pmax.[1]-subdivision.Pmin.[1])/2.
    let stepz = (subdivision.Pmax.[2]-subdivision.Pmin.[2])/2.          

    let mutable boxes = [] // Each element on the list is a partition of the world
    for i in [1..2] do                              // From Pmin to Pmax
        let pxmin = stepx*float(i-1)+subdivision.Pmin.[0]
        let pxmax = stepx * float(i)+subdivision.Pmin.[0]

        for j in [1..2] do                          // From Pmin to Pmax
            let pymin = stepy*float(j-1)+subdivision.Pmin.[1]
            let pymax = stepy * float(j)+subdivision.Pmin.[1]

            for k in [1..2] do                      // From Pmin to Pmax
                let pzmin = stepz*float(k-1)+subdivision.Pmin.[2]
                let pzmax = stepz * float(k)+subdivision.Pmin.[2]
                let thisbox = {Pmin = [pxmin;pymin;pzmin] ;
                                Pmax = [pxmax;pymax;pzmax] }
                (*if depth = 5 then// i = 1 && j = 1 && k= 1 then
                    printfn "ijk: %+A%+A%+A %+A" i j k depth
                else
                    printfn "ijk: %+A%+A%+A" i j k
                *)    
                let (sphIDs,sphbox) = SphereToGrid (thisbox, world.Sphere)
                // Cylinders
                let (MeshesID,triangleslist,normalslist,meshesbox) = MeshToGrid (thisbox, world.Meshes)
                let partition = Partition{Bbox = thisbox;
                                          MeshID = MeshesID; MeshTriangles = triangleslist;MeshNormals=normalslist; MBBox = meshesbox; // Meshes
                                          SphID = sphIDs; SphBox = sphbox                                      // Spheres
                                          } 
                //let triannum = triangleslist |> List.map(fun x -> x.Length) |>List.sum
                let NumEle = 
                    let ele = sphIDs.Length +  (triangleslist |> List.map(fun x -> x.Length) |>List.sum )
                    if ele > maxEle then
                        if depth <= maxdepth then ele//|> (fun x -> x.Length))
                        else maxEle
                    else ele
                //printfn "NumEle: %i" NumEle
                //if  NumEle = 70 then
                //    printfn "NumEle: %i" NumEle
                //else printfn "NumEle: %i" NumEle
                if NumEle <= maxEle then 
                    match (MeshesID,sphIDs) with
                    | ([],[]) -> boxes <- boxes
                    |(_,_) ->  boxes <- boxes@[partition]
                else boxes <- boxes@[Octree{Bbox=thisbox;Octree=OctreeSubdivision (world, thisbox, maxEle,depth+1,maxdepth)}]
    boxes


let WorldLimitsExtend (world:world) =
    //read all the elements on the scene and computes the BBox of the world

    //I start with meshes
    let (meshMin,meshMax)  =
        let bm = world.Meshes|> List.collect (fun x -> [x.Bbox]) // each mesh has a Bbox
        
        match bm with  //In case there are no meshes
        |[] -> ([infinity;infinity;infinity ],[-infinity;-infinity;-infinity])
        |_ -> let mMin i = bm |> List.collect(fun x -> [x.Pmin.[i]]) |> List.min 
              let mMax i = bm |> List.collect(fun x -> [x.Pmax.[i]]) |> List.max 
              //([mMin 0; mMin 1 ; mMin 2],[(mMax 0); (mMax 1); (mMax 2)])
              ([(mMin 0)- 1e-8; (mMin 1 )- 1e-8; (mMin 2 )- 1e-8],[(mMax 0)+1e-8; (mMax 1)+1e-8; (mMax 2)+1e-8])
        
    //Spheres
    let sphminmax (sph:sphere) = //Limits of the sphere
        let Smax = [sph.center.X + sph.radius; sph.center.Y + sph.radius; sph.center.Z + sph.radius]
        let Smin = [sph.center.X - sph.radius; sph.center.Y - sph.radius; sph.center.Z - sph.radius]
        { Pmin=Smin ;Pmax=Smax}

    let (smin,smax) = //find max and min of spheres
        let bs = world.Sphere |> List.collect(fun x -> [sphminmax x])
        match bs with // In case there are no Spheres
        |[] -> ([infinity;infinity;infinity ],[-infinity;-infinity;-infinity])
        |_ -> let mMin i = bs |> List.collect(fun x -> [x.Pmin.[i]]) |> List.min              
              let mMax i = bs |> List.collect(fun x -> [x.Pmax.[i]]) |> List.max 
              ([(mMin 0)- 1e-10; (mMin 1 )- 1e-10; (mMin 2 )- 1e-10],[(mMax 0)+1e-10; (mMax 1)+1e-10; (mMax 2)+1e-10]) // Min and Max of shperes
    let bb = {Pmin = List.map2 (fun x y -> (min x y)) smin meshMin; 
              Pmax= List.map2 (fun x y -> (max x y)) smax meshMax}
    printfn " The Limits of the worlds are: %+A \n" bb 
    bb
//let oct = OctreeSubdivision (all, WorldLimitsExtend (all),100)  // all is in F# interactive
//oct.Length
//oct.[7]
// 3+132+23+49+81+19+ should 259 not 30x magari per che un triangolo a due partizioni
