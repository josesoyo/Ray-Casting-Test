module RayCore

open System
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayType
open BBox 


///////////////////////////////////////////////////
//
//  castRAy dapted from martindoms.com
//
/// Get the position of a ray at a given time for a Sphere

let pointAtTime (rayfrom:RayFrom, time:float) =
    rayfrom.from + rayfrom.uvec.ScaleBy(time) 
    //Ray3D.Direction.ScaleBy -> to multiply by a float

let intersection(ray:RayFrom,sphere:sphere,nsamples:int)=
    //let s = ray.origin - scene.Sphere.center //raycast for sphere
    let s = ray.from - sphere.center //raycast for sphere
    let rayDir = ray.uvec //ray.Direction// Normalized direction of the ray3D
    let sv = rayDir.DotProduct(s)
    let ss = s.DotProduct(s)
    let discr = sv*sv - ss + sphere.radius*sphere.radius
    if discr < 0.0 then []
    else
        let normalAtTime t = UnitVector3D((pointAtTime( ray, t) - sphere.center).X,(pointAtTime( ray, t) - sphere.center).Y,(pointAtTime( ray, t) - sphere.center).Z)//sphere.radius //normalized with the sphere.radius
        let (t1,t2) = (-sv + sqrt(discr), -sv - sqrt(discr)) //times at which the ray is intercepted
        let (travel1,travel2) = ((t1+ray.travelled),(t2+ray.travelled)) //Update distance travelled by the ray
        let (ray1,ray2) = ({uvec=ray.uvec; length=ray.length; from=ray.from; travelled=travel1},{uvec=ray.uvec; length=ray.length; from=ray.from; travelled=travel2})
        let intersects = [ { normal = normalAtTime t1 ; point = pointAtTime (ray, t1); ray = ray1 ; material=sphere.material; t=t1;Nsamples=nsamples};
                           { normal = normalAtTime t2 ; point = pointAtTime (ray, t2); ray = ray2 ; material=sphere.material;t=t2;Nsamples=nsamples}]
        //printfn "t value: %f %f" intersects.Head.t intersects.[1].t
        intersects 

let castRay (scene:scene, ray:RayFrom) = 
    scene.World.Sphere 
    |> List.collect (fun x -> intersection(ray, x,scene.Nsamples))  // Find interstion with all the spheres
    |> List.filter (fun x -> x.t > 0.01)             // Select the ones that are not negative-€[e,infinity)

//////////////////////////
//
// Intersection for Meshes
// Not implemented yet, but intersection can handle triangles and squares as a primitives now
//
//////////////////////////

let intersec_mesh (ray:RayFrom,  mesh:mesh,triangle:int list,nrm:UnitVector3D, nsamples:int, shape:char)= // normal is only passing
    // Method 2 from PBRT book eq 3.5
    let nodes = mesh.Vertices // *** Is wasting memory *** I can: mesh.Vertices.[n0]

    let (n0,n1,n2) = (triangle.[0]-1, triangle.[1]-1, triangle.[2]-1)
    let (u0u1,u0u2) = (nodes.[n1]-nodes.[n0],nodes.[n2]-nodes.[n0]) 

    let raydir = ray.uvec
    // A couple of def:
    let s = ray.from - nodes.[n0]//ray.from
    let s1 = raydir.CrossProduct(u0u2) //dxe2 = ray.ray X u0u2
    let s2 = (s).CrossProduct(u0u1)// sxe1 = V3D(ray.from - u0) X u0u1
    // Test to check interception
    let s1Dote1 = s1.DotProduct(u0u1)
    let u = s1.DotProduct(s)/ s1Dote1
    let v = s2.DotProduct(raydir)/s1Dote1
    let logic =
        if shape = 't' then // intersection with a triangle condition
            if (u>0. && v>0. && (u+v)<1.) then true
            else false
        elif shape = 's' then // intersection with a square condition
            if (u>0. && v>0. && u<1. && v < 1.) then true
            else false
        else 
            printfn "ERROR in the definition of the shape"
            false
    if logic then
        let t1 = s2.DotProduct(u0u2) / (s1Dote1)
        if t1 >= ray.length then [] 
        // The collision cannot be further than the light when we do a shadow
        // The equal is because in the case s1Dote1 = 0 degenerate and there's no collision (t= infinity)
        // generates a problem in multiple transmision/reflection if ray.lenght is not inf intersecting not for shadow
        // Problem solved?
        else
            // t1 < dist (light- point) case shadow
            let newRay = {uvec=ray.uvec; from = ray.from;length = ray.length; travelled = (t1+ ray.travelled)}
            let VIntersect = u0u1.ScaleBy(u) + u0u2.ScaleBy(v) 
            let PIntersect = Point3D( VIntersect.X + nodes.[n0].X,VIntersect.Y + nodes.[n0].Y,VIntersect.Z + nodes.[n0].Z )
            [{ normal=nrm; point=PIntersect; ray=newRay;material=mesh.material; t=t1; Nsamples = nsamples}]
    else
        []
        //type Intersection_mesh = { normal:Vector3D; point:Point3D; ray:RayFrom; mesh:mesh;t:float}

// Prepared to accept triangles and squares as a primitive for intersection. 
//Squares should reduce the computation time
let intersec_tri (ray, mesh,triangle,nrm,nsamples)= intersec_mesh (ray, mesh,triangle,nrm,nsamples,'t')
let intersec_square (ray, mesh,triangle,nrm,nsamples)= intersec_mesh (ray, mesh,triangle,nrm,nsamples,'s')
//


let castRay_mesh (scene:scene, ray:RayFrom) = //here it's only for sphere
    let interceptions (ray:RayFrom, mesh:mesh,nsamples)  =
        [0..(List.length(mesh.Triangles)-1)]
        |> List.collect (fun x -> intersec_tri(ray, mesh, mesh.Triangles.[x],mesh.normals.[x],nsamples))
        |> List.filter (fun x -> x.t > 0.01)  //
    // Meshes_intersections = castRay_mesh(scene,ray)
    let CastBBox(mesh:mesh,ray:RayFrom) =
        let bolean = BBox_intersec( mesh.Bbox, ray) 
        if bolean then interceptions(ray,mesh,scene.Nsamples)
        else []
    (*
    let aCastBBox mesh ray = 
        let a = CastBBox(mesh,ray)
        async{return a}                                 // The async type creation
    //scene.World.Meshes |> List.collect(fun x -> CastBBox(x,ray))
    let b = scene.World.Meshes 
            |> List.collect(fun x -> [aCastBBox x ray]) 
            |>Async.Parallel|> Async.RunSynchronously   //Parallel functions
            |> Array.toList                             // The RunSynchronously generates an array
            |> List.collect(fun x -> x)                 // intersection list [] -> intersection list
    b
    *)
    //printfn "%i" b.Length
    //printfn "the b is:\n %+A" b
    //printfn "ciao"
    
    
    //scene.World.Meshes |> List.collect(fun x -> CastBBox(x,ray))
    scene.World.Meshes |> List.collect(fun x -> CastBBox(x,ray))

/////
//
//          Main function to choose the intersections 
//
/////

let CastRay_nest(scene:scene, ray:RayFrom) =
    let Spheres_intersections = castRay(scene,ray) //_sphere
    let Meshes_intersections = castRay_mesh(scene,ray)
    List.append Spheres_intersections Meshes_intersections //append is 2 lists