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