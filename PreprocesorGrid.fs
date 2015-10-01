module PreprocesorGrid

open System.IO
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml
open RayType
open BBox 
open RayCore

//type Grid3D= {BBox:BBox                                                            //BBox of the 3DGrid
//              MeshID:int list; MeshTriangles: int list list list;MBBox:BBox list;  //mesh list
//              SphID:int list; SphBox:BBox list;                                    //spheres list
//              }

//Steps:
//  1- Create the world box
//  2- Create the Grid
//
//      2.1 - Partition the box (do min+(i-1)*max/steps, min + i*max/steps) i= [1..part]
//      2.2 - Find the meshes inside the box and it's triangles
//      2.3 - Find the spheres inside the mesh and its fraction
//      3.4 - Create the Grid3D

let WorldLimits (world:world) =
    //read all the elements on the scene and computes the BBox of the world

    //I start with meshes
    let (meshMin,meshMax)  =
        let bm = world.Meshes|> List.collect (fun x -> [x.Bbox]) // each mesh has a Bbox
        
        match bm with  //In case there are no meshes
        |[] -> ([infinity;infinity;infinity ],[-infinity;-infinity;-infinity])
        |_ -> let mMin i = bm |> List.collect(fun x -> [x.Pmin.[i]]) |> List.min 
              let mMax i = bm |> List.collect(fun x -> [x.Pmax.[i]]) |> List.max 
              ([mMin 0; mMin 1 ; mMin 2],[mMax 0; mMax 1; mMax 2])
        
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
              ([mMin 0; mMin 1 ; mMin 2],[mMax 0; mMax 1; mMax 2]) // Min and Max of shperes
    let bb = {Pmin = List.map2 (fun x y -> (min x y)) smin meshMin; 
              Pmax= List.map2 (fun x y -> (max x y)) smax meshMax}
    printfn " The Limits of the worlds are: %+A \n" bb 
    bb


let MeshToGrid (box:BBox, meshes:mesh list) =
    // return (MeshID list , MeshTriangles list,BBox list)
    //
    // the input bbox is the Bounding Box of one spatial partition
    //  Main function inside is: AllMeshesIter
    // SubMeshT -> triangles in the grid of a mesh
    // SubMeshBox -> Bounding box of the submesh
    // I need to apply the iteration on all the meshes and see which are inside an MeshID

    let vinside(vertex:Point3D, box:BBox) =
        // Check if one vertex is inside the BBox - Not enough to say if 1Triangle is inside
        //               CAREFULL!!!
        //               One side should be < and the other >=, but I create a problem on the edges...
        if vertex.X <= box.Pmax.[0] && vertex.Y <= box.Pmax.[1] && vertex.Z <= box.Pmax.[2] then 
            if vertex.X >= box.Pmin.[0] && vertex.Y >= box.Pmin.[1] && vertex.Z >= box.Pmin.[2] then true
            else false     
        else false
    let VertInside (box, mesh:mesh)= 
        mesh.Vertices|> List.collect(fun x -> [vinside(x,box)])
    
    let  IDList (i:int ,bol:bool) =
        // Small funtion to create lists of the trues
        if bol then [i]
        else []

    let Edges2RayFrom (box:BBox) =
        // Find the RayFrom object of each edge of the partition box
        // 1st - Find the points of the cube in order of vector calculation
        let modpointmin (pmin:float list,pmax:float list, i: int) =
            if i = 0 then [pmax.[0];pmin.[1];pmin.[2]]
            else if i = 1 then [pmin.[0];pmax.[1];pmin.[2]]
            else [pmin.[0];pmin.[1];pmax.[2]]
        let modpointmins2 (pmin2:float list,pmax:float list, i: int) =
            //2nd generation of points to compute the vectors
            if i = 0 then [[pmin2.[0];pmax.[1];pmin2.[2]];[pmin2.[0];pmin2.[1];pmax.[2]]]
            else if i = 1 then [[pmax.[0];pmin2.[1];pmin2.[2]];[pmin2.[0];pmin2.[1];pmax.[2]]]
            else [[pmax.[0];pmin2.[1];pmin2.[2]];[pmin2.[0];pmax.[1];pmin2.[2]]]
        let modpointmax (pmin:float list,pmax:float list, i: int) =
            //last  points to compute vectors (pmax-this)
            if i = 0 then [pmin.[0];pmax.[1];pmax.[2]]
            else if i = 1 then [pmax.[0];pmin.[1];pmax.[2]]
            else [pmax.[0];pmax.[1];pmin.[2]]
        //Points
        let pmins = [0..2] |> List.collect(fun x ->  [modpointmin(box.Pmin,box.Pmax,x)])
        let pintermediate = [0..2] |> List.collect (fun x ->  [modpointmins2(pmins.[x],box.Pmax,x)])
        let pmaxs = [0..2] |> List.collect(fun x->  [modpointmax(box.Pmin,box.Pmax,x)])
        //Vertexs
        let subs2RayFrom(toP:float list, fromP: float list) =
            // create the RayFrom object from two points
            // toPoint - fromPoint  = Vector3D
            let p1 = Point3D(toP.[0],toP.[1],toP.[2])
            let p2 = Point3D(fromP.[0],fromP.[1],fromP.[2])
            let ve= p1-p2
            {uvec=ve.Normalize();length=ve.Length;from= p2; travelled = 0.}
        //type RayFrom = {uvec:UnitVector3D; length: float; from:Point3D; travelled:float}
        let vfirst = pmins |> List.collect(fun x-> [subs2RayFrom(x,box.Pmin)])
        let vsecond =  
            [0..2]
            |>List.collect(fun x -> [subs2RayFrom(pintermediate.[x].[0],pmins.[x]);subs2RayFrom(pintermediate.[x].[1],pmins.[x])])

        let vlast = pmaxs |> List.collect(fun x -> [subs2RayFrom(box.Pmax, x)])
        List.append (List.append vfirst vsecond) vlast
        // RayFrom Types of the edges of the cube

    let tinside (boolv:bool list,mesh:mesh, edgeRays:RayFrom list)= 
        // Says if one mesh intersectes one partition
        // boolv = boolans of the vertices
        // edgeRays = Vectors of the edges of the partition
        //
        // First check the vertices as a trivial way (VertInside & vinside funtions)
        // Then check the intersections of the edges ob the partition box
        let realboolean (tri:int list, boolv:bool list,mesh:mesh, i:int,edgeRays:RayFrom list)= //boolean of intersection of triangle with box
            //printfn "Triangles lenght: %+A" tri.Length
            //printfn "%+A" tri
            //printfn "Boollength: %+A %+A" boolv.Length mesh.Triangles.Length
            if boolv.[tri.[0]-1] || boolv.[tri.[1]-1] || boolv.[tri.[2]-1] then true // || = or
            else 
                let intersecTRI = edgeRays 
                                    |> List.collect(fun x -> intersec_tri(x, mesh, tri,mesh.normals.[i],0))
                                    |> List.filter (fun x -> x.t > 0.01)
                match intersecTRI with
                | [] -> false //No intersection
                | _ -> true
        
        let triangles = mesh.Triangles
        let BoolT = [0..(triangles.Length-1)] 
                    |> List.collect(fun x -> [realboolean( mesh.Triangles.[x], boolv, mesh, x,edgeRays)]) //triangles insi
        BoolT

    let SubMeshT (mesh:mesh,boolv: bool list,edgeRays:RayFrom list) =    
        // List of triangles intersected

        let triangles = mesh.Triangles
        //printfn "Inside the SubMEshT: %+A" triangles
        let BoolT = tinside( boolv, mesh,edgeRays)

        [0..(BoolT.Length-1)] 
        |> List.collect(fun x -> IDList (x, BoolT.[x])) 
        |> List.collect(fun x -> [triangles.[x]])
      
      
    let SubMeshBox (mesh:mesh,boolv:bool list) = // NOT USED
        //Find the bounding box of a submesh inside a partition
        let vertexs = mesh.Vertices
        //Bounding box of the submesh 
        //Wrong I must use BoxofIntersection (meshBox:BBox, partitionBox:BBox) in BBox.fs --> NONONONO and NO 
        // In any case shall be done with the triangles obtained in SubMeshT
        let select = 
            [0..(boolv.Length-1)]
            |> List.collect(fun x -> IDList(x,boolv.[x]))
        let listx = select |> List.collect(fun x -> [vertexs.[x].X]) 
        let listy = select  |> List.collect(fun x -> [vertexs.[x].Y]) 
        let listz = select  |> List.collect(fun x -> [vertexs.[x].Z]) 
        //printfn "SubmeshBox:\n %+A \n %+A \n "select boolv
        {Pmin=[List.min(listx); List.min(listy); List.min( listz)];
         Pmax=[List.max (listx); List.max (listy); List.max (listz)]}
          

    let AllMeshesIter (gbox:BBox, meshes:mesh list) =
        // Check mesh by mesh which one is inside the Grid/Partition in order to insert it.
        //
        // 1 - Iter the meshes to check them
        // 
        let FirstMeshListID =  // I MUST DO IT WITH ALL
            // List of meshes that intersect the partition
            [0..(meshes.Length-1)]
            |> List.collect(fun x -> IDList(x,BoxBoxIntersection(meshes.[x].Bbox, gbox)))

        let boolVinMeshes = //I define this strange name to diferentiate with boolv
            // List which contais:
            // After checking if the boxes are intersected I must know if there're vertices inside the partition
            // [[booleans of vertices of Mesh1];[booleans of vertices of Mesh2];...] 
            FirstMeshListID  
            |> List.collect(fun x -> [VertInside (gbox, meshes.[x])])
        let rayEdges = Edges2RayFrom(gbox)
        
        //tinside (boolv:bool list,mesh:mesh, edgeRays:RayFrom list)
        
        let RealMeshID( boolVinMeshes: bool list list) = // Bool Mesh# Vertice#
            let trianglesInMeshesIntersected = [0..(boolVinMeshes.Length-1)] //// List list of (TRIANGLES: true or false)
                                               |>List.collect(fun x -> [tinside(boolVinMeshes.[x],meshes.[FirstMeshListID.[x]],rayEdges)]) 
                     //boolv 
            let TriangleFromBoolToVertice (trianglesBool1MEsh:bool list,mesh:mesh) = 
                // The booleans of the triangles intercepted are changed by the triangle
                // trianglesBool1MEsh = The list of triangles that is a bool. True = triangle intercepted by the partition
                [0..trianglesBool1MEsh.Length-1] |> List.collect(fun x -> IDList(x,trianglesBool1MEsh.[x]))
                |> List.collect(fun x -> [mesh.Triangles.[x]])
            let MeshIDFromTriangles (boolTriangles:bool list, i:int) =
                // It iterates in each one of the meshes to see if one triangle is intersected with the partition
                let IsThereTriangleIntersection = [0..(boolTriangles.Length-1)]|> List.collect(fun x -> IDList(x,boolTriangles.[x]) )
                match IsThereTriangleIntersection with 
                |[] -> []
                |_ -> [i]
            let MeshID = [0..(trianglesInMeshesIntersected.Length-1)]
                         |> List.collect(fun x -> MeshIDFromTriangles(trianglesInMeshesIntersected.[x],FirstMeshListID.[x]) ) 
                         //|> List.collect(fun x-> IDList(0,x)) |>List.iteri(fun i x -> IDList(i,x) )
            let MeshTriangles = [0..(trianglesInMeshesIntersected.Length-1)] ////CArefull with the mesh number
                                |> List.collect(fun x -> [TriangleFromBoolToVertice(trianglesInMeshesIntersected.[x],meshes.[FirstMeshListID.[x]])]) 
            (MeshID, MeshTriangles)
        //let bol2 = boolVinMeshes |> List.collect(fun x -> TrueIn x)
        
        //True meshes that have an intersection

        // let tinside (tri:int list, boolv:bool list,mesh:mesh, i:int,edgeRays:RayFrom list)= 
        let (MeshListID,MeshTriangles) = RealMeshID(boolVinMeshes)//[0..FirstMeshListID.Length-1] 
                         //|>List.collect(fun x ->[( meshes.[x].Triangles,boolv.[x], rayEdges)])
                         //|> List.collect(fun x -> IDList(FirstMeshListID.[x],bol2.[x]))
        (*
        let MeshTriangles =
            [0..(MeshListID.Length-1)] 
            |>List.collect(fun x ->[( meshes.[MeshListID.[x]],boolVinMeshes.[x], rayEdges)]) // I'm doing the intersection of triangles two times
            |> List.collect(fun x -> [SubMeshT x])
        *)
        printfn "MEsh %+A \n %+A" FirstMeshListID boolVinMeshes
        let MeshsubBoxes = 
            // Not the best, should be computed as the BBox of the triangles inside the partition
            // The current box may be much bigger than the required (CAn be seen on the paper)
            MeshListID//[0..(MeshListID.Length-1)]
            //|>List.collect(fun x ->[( meshes.[MeshListID.[x]],boolv.[x])])
            |>List.collect(fun x ->[( meshes.[x].Bbox,gbox)])
            |> List.collect(fun x -> [BoxofIntersection x]) 
            //|>List.collect(fun x -> [SubMeshBox x]) // BoxofIntersection (meshBox:BBox, partitionBox:BBox)
        
        (MeshListID, MeshTriangles, MeshsubBoxes)
        
    AllMeshesIter (box, meshes)

let SphereToGrid (box:BBox, sphs: sphere list) =
    //steps:
    //  1 - Find which spheres are inside the spatial partition -> bool list
    //  2 - add their label into a list -> list of the sphere numbers inside
    //  3 - Check which part is inside the box
    let  IDList (i:int ,bol:bool) =
       if bol then [i]
       else []
    let sphBool =   [0..(sphs.Length-1)]  |> List.collect(fun x -> [BoxSphereIntersection (sphs.[x], box)])
    let sphID = [0..(sphs.Length-1)] |> List.collect(fun x -> [(x,sphBool.[x])]) 
                 |> List.collect(fun x -> IDList x) 
    let sphBBoxs = sphID|> List.collect(fun x -> [SphBBoxInBox (sphs.[x], box)]) 
    // SphBBoxInBox works because I already know it's inside
    (sphID,sphBBoxs)
   
let Partitionate (world:world,part:int)=
    // The result must be a list of Grid3D
    let limits = WorldLimits (world) // Bounding box of all the elements that create the world
    let (partx,party,partz) = (part,part,part)
    let stepx = (limits.Pmax.[0]-limits.Pmin.[0])/float(partx)
    let stepy = (limits.Pmax.[1]-limits.Pmin.[1])/float(party)
    let stepz = (limits.Pmax.[2]-limits.Pmin.[2])/float(partz)
    
    let grids=
        let mutable boxes = [] // Each element on the list is a partition of the world
        for i in [1..partx] do                              // From Pmin to Pmax
            let pxmin = stepx*float(i-1)+limits.Pmin.[0]
            let pxmax = stepx * float(i)+limits.Pmin.[0]
            for j in [1..party] do                          // From Pmin to Pmax
                let pymin = stepy*float(j-1)+limits.Pmin.[1]
                let pymax = stepy * float(j)+limits.Pmin.[1]
                for k in [1..partz] do                      // From Pmin to Pmax
                    let pzmin = stepz*float(k-1)+limits.Pmin.[2]
                    let pzmax = stepz * float(k)+limits.Pmin.[2]
                    let thisbox = {Pmin = [pxmin;pymin;pzmin] ;
                                   Pmax = [pxmax;pymax;pzmax] }
                    let (sphIDs,sphbox) = SphereToGrid (thisbox, world.Sphere)
                    let (MeshesID,triangleslist,meshesbox) = MeshToGrid (thisbox, world.Meshes)
                    let partition = {Bbox = thisbox;
                                     MeshID = MeshesID; MeshTriangles = triangleslist; MBBox = meshesbox; // Meshes
                                     SphID = sphIDs; SphBox = sphbox                                      // Spheres
                                     } 
                    //printfn "the hell is here: \n %+A" partition
                    match partition.MeshID with
                    | [] -> match partition.SphID with
                            |[] -> boxes <- boxes   //If empty don't create it
                            | _ -> printfn "%+A" partition
                                   boxes <- List.append boxes [partition]
                    | _ -> printfn "%+A" partition
                           boxes <- List.append boxes [partition]                
                    
        boxes
    grids

//type Grid3D= {BBox:BBox                                                            //BBox of the 3DGrid
//              MeshID:int list; MeshTriangles: int list list list;MBBox:BBox list;  //mesh list
//              SphID:int list; SphBox:BBox list;                                    //spheres list
//              }
    //(do min+(i-1)*max/steps, min + i*max/steps) i= [1..part]