module BBox //Bounding box

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml


open RayType

// type mesh = {Vertices:Point3D list ; Triangles: int list list;  material:material;normals: UnitVector3D list; Bbox : BBox}
//type BBox = {Pmin :float list; Pmax:float list}
// also change the input to include the Bbox
let BBox_creation (nodes:Point3D list) =
    let pxmin = (nodes |> List.minBy (fun x -> x.X)).X
    let pymin = (nodes |> List.minBy (fun x -> x.Y)).Y
    let pzmin = (nodes |> List.minBy (fun x -> x.Z)).Z

    let pxmax = (nodes |> List.maxBy (fun x -> x.X)).X
    let pymax = (nodes |> List.maxBy (fun x -> x.Y)).Y
    let pzmax = (nodes |> List.maxBy (fun x -> x.Z)).Z
    {Pmin =[pxmin;pymin;pzmin];Pmax=[pxmax;pymax;pzmax]}


let BBox_intersec( box:BBox, ray:RayFrom) =
    
    // Not happy with convert all to list
    let raylist = [ray.uvec.X; ray.uvec.Y; ray.uvec.Z]
    let rayfrom =[ray.from.X; ray.from.Y; ray.from.Z]
    let Pminl = box.Pmin
    let Pmaxl = box.Pmax
    let maxBox (rayi, boxmi:float, boxma:float)=
        if rayi < 0. then //min is the max
                boxmi//pma.[i] <- box.Pmin.[i]
            else 
                boxma
    let minBox (rayi, boxmi:float, boxma:float)=
        if rayi > 0. then //min is the max
                boxmi//pma.[i] <- box.Pmin.[i]
            else 
                boxma
    let pmax = [0..2] |> List.collect(fun x -> [maxBox(raylist.[x], Pminl.[x], Pmaxl.[x])] )
    let pmin = [0..2] |> List.collect(fun x -> [minBox(raylist.[x], Pminl.[x], Pmaxl.[x])] )
    let mutable (vmin,vmax) =(0.0,infinity)
    let intersection_c i =
        //it gives the closest and the furthers intersection
        let tclose = (pmin.[i]-rayfrom.[i])/raylist.[i]// def t
        if tclose < 0.0 then 
            0.0
        else
            tclose
    let intersection_f i =
        //it gives the closest and the furthers intersection
        let tfar = (pmax.[i]-rayfrom.[i])/raylist.[i]// def t
        tfar

    let update_tc tmin tclose =
            if tclose > tmin then tclose
            else tmin

    let update_tf tmax tfar =
            if tmax > tfar then tfar
            else tmax

    let minminlist = [0..2]|> List.collect (fun x -> [intersection_c x])
    let minmin = update_tc vmin (List.max minminlist)
    //printfn "ciao"
    let maxmax = [0..2]|> List.collect (fun x -> [intersection_f x]) |> List.min  |> update_tf vmax    
    if (minmin > maxmax) then false
    else true
    //let  = [0..2] |> List.collect( fun x -> []) 
    //maxmins|> List.minBy (maxmins.)

let BoxBoxIntersection (box1:BBox, box2:BBox) =
    // Check if two boxes have not null intersection
    // Check that min or max are OUT the minmax of the other box

    (*  Algorithm for box box intersection:
        - Conditions there's no intersection

	    return NOT (
		    (Rect1.Bottom < Rect2.Top) OR  ----> Top is lower value
		    (Rect1.Top > Rect2.Bottom) OR
		    (Rect1.Left > Rect2.Right) OR  
		    (Rect1.Right < Rect2.Left) )
     ------------------------ 
     My case
            Rect1 = box1 Rect2 = box2
            Right = max         Left = min
    *)
    let xlim =
        // If true they don't intersect on X axis
        if box1.Pmax.[0] < box2.Pmin.[0] || box1.Pmin.[0] > box2.Pmax.[0] then true
        else false

    let ylim =
        // If true they don't intersect on y axis
        if box1.Pmax.[1] < box2.Pmin.[1] || box1.Pmin.[1] > box2.Pmax.[1] then true
        else false

    let zlim =
        // If true they don't intersect on z axis
        if box1.Pmax.[2] < box2.Pmin.[2] || box1.Pmin.[2] > box2.Pmax.[2] then true
        else false

    if xlim || ylim || zlim then false
    else true

let BoxofIntersection (meshBox:BBox, partitionBox:BBox) =
    //Checks which one is the intersected space between a mesh and a partition
    // box1 = mesh
    let sphBox = meshBox
    let box = partitionBox
    let Nmin = 
        let xmin =
            if sphBox.Pmin.[0] > box.Pmin.[0] then sphBox.Pmin.[0]
            else box.Pmin.[0]
        let ymin =
            if sphBox.Pmin.[1] > box.Pmin.[1] then sphBox.Pmin.[1]
            else box.Pmin.[1]
        let zmin =
            if sphBox.Pmin.[2] > box.Pmin.[2] then sphBox.Pmin.[2]
            else box.Pmin.[2]
        [xmin;ymin;zmin]
    let Nmax = 
        let xmax =
            if sphBox.Pmax.[0] < box.Pmax.[0] then sphBox.Pmax.[0]
            else box.Pmax.[0]
        let ymax =
            if sphBox.Pmax.[1] < box.Pmax.[1] then sphBox.Pmax.[1]
            else box.Pmax.[1]
        let zmax =
            if sphBox.Pmax.[2] < box.Pmax.[2] then sphBox.Pmax.[2]
            else box.Pmax.[2]
        [xmax;ymax;zmax]
    {Pmin= Nmin; Pmax =Nmax}

let BoxSphereIntersection (sph:sphere, box:BBox) =
    // Intersection between one sphere and a box
    // as BoxBoxIntersection, but center+/-r
    let center = sph.center
    let radius = sph.radius
    let sphBox = {Pmin=[center.X-radius;center.Y-radius;center.Z-radius] ; 
                  Pmax=[center.X+radius;center.Y+radius;center.Z+radius]}
    BoxBoxIntersection (sphBox, box)

let SphBBoxInBox (sph:sphere, box:BBox) =
    let center = sph.center
    let radius = sph.radius
    let sphBox = {Pmin=[center.X-radius;center.Y-radius;center.Z-radius] ; 
                  Pmax=[center.X+radius;center.Y+radius;center.Z+radius]}
    let Nmin = 
        let xmin =
            if sphBox.Pmin.[0] > box.Pmin.[0] then sphBox.Pmin.[0]
            else box.Pmin.[0]
        let ymin =
            if sphBox.Pmin.[1] > box.Pmin.[1] then sphBox.Pmin.[1]
            else box.Pmin.[1]
        let zmin =
            if sphBox.Pmin.[2] > box.Pmin.[2] then sphBox.Pmin.[2]
            else box.Pmin.[2]
        [xmin;ymin;zmin]
    let Nmax = 
        let xmax =
            if sphBox.Pmax.[0] < box.Pmax.[0] then sphBox.Pmax.[0]
            else box.Pmax.[0]
        let ymax =
            if sphBox.Pmax.[1] < box.Pmax.[1] then sphBox.Pmax.[1]
            else box.Pmax.[1]
        let zmax =
            if sphBox.Pmax.[2] < box.Pmax.[2] then sphBox.Pmax.[2]
            else box.Pmax.[2]
        [xmax;ymax;zmax]
    {Pmin= Nmin; Pmax =Nmax}