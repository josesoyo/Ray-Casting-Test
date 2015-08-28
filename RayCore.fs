module RayCore

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayType


///////////////////////////////////////////////////
//
//  castRAy dapted from martindoms.com
//
/// Get the position of a ray at a given time for a Sphere

let pointAtTime (rayfrom:RayFrom, time:float) =
    rayfrom.from + rayfrom.uvec.ScaleBy(time) 
    //Ray3D.Direction.ScaleBy -> to multiply by a float

let intersection(ray:RayFrom,sphere:sphere)=
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
        let intersects = [ { normal = normalAtTime t1 ; point = pointAtTime (ray, t1); ray = ray1 ; material=sphere.material; t=t1};
                           { normal = normalAtTime t2 ; point = pointAtTime (ray, t2); ray = ray2 ; material=sphere.material;t=t2}]
        //printfn "t value: %f %f" intersects.Head.t intersects.[1].t
        intersects 

let castRay (scene:scene, ray:RayFrom) = 
    scene.World.Sphere 
    |> List.collect (fun x -> intersection(ray, x))  // Find interstion with all the spheres
    |> List.filter (fun x -> x.t > 0.01)             // Select the ones that are not negative-€[e,infinity)

//////////////////////////
//
// Intersection for Meshes
//
//////////////////////////

let intersec_tri (ray:RayFrom,  mesh:mesh,triangle:int list,nrm:UnitVector3D)= // normal is only passing
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
    if (u>0. && v>0. && (u+v)<1.) then
        let t1 = s2.DotProduct(u0u2) / (s1Dote1)
        if t1 >= ray.length then [] 
        // The collision cannot be further than the light - shadow
        // The equal is because in the case s1Dote1 = 0 degenerate and there's no collision (t= infinity)
        else
            // t1 < dist (light- point)
            let newRay = {uvec=ray.uvec; from = ray.from;length = ray.length; travelled = (t1+ ray.travelled)}
            let VIntersect = u0u1.ScaleBy(u) + u0u2.ScaleBy(v) 
            let PIntersect = Point3D( VIntersect.X + nodes.[n0].X,VIntersect.Y + nodes.[n0].Y,VIntersect.Z + nodes.[n0].Z )
            [{ normal=nrm; point=PIntersect; ray=newRay;material=mesh.material; t=t1}]
    else
        []
        //type Intersection_mesh = { normal:Vector3D; point:Point3D; ray:RayFrom; mesh:mesh;t:float}


let castRay_mesh (scene:scene, ray:RayFrom) = //here it's only for sphere
    let interceptions (ray:RayFrom, mesh:mesh)  =
        [0..(List.length(mesh.Triangles)-1)]
        |> List.collect (fun x -> intersec_tri(ray, mesh, mesh.Triangles.[x],mesh.normals.[x]))
        |> List.filter (fun x -> x.t > 0.051)  //
    scene.World.Meshes |> List.collect(fun x -> interceptions(ray,x))

/////
//
//          Main function to choose the intersections 
//
/////

let CastRay_nest(scene:scene, ray:RayFrom) =
    let Spheres_intersections = castRay(scene,ray) //_sphere
    let Meshes_intersections = castRay_mesh(scene,ray)
    List.append Spheres_intersections Meshes_intersections //append is 2 lists