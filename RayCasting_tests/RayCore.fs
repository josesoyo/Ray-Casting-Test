module RayCore

open System
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayType
open BBox 
open RayTypeMethods
let PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062

///////////////////////////////////////////////////
//
//  castRAy dapted from martindoms.com
//
/// Get the position of a ray at a given time for a Sphere

let pointAtTime (rayfrom:RayFrom, time:float) =
    rayfrom.from + rayfrom.uvec.ScaleBy(time) 
    //Ray3D.Direction.ScaleBy -> to multiply by a float

let intersection_sphere(ray:RayFrom,sphere:sphere,nsamples:int)=
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
    |> List.collect (fun x -> intersection_sphere(ray, x,scene.Nsamples))  // Find interstion with all the spheres
    |> List.filter (fun x -> x.t > 1e-10)             // Select the ones that are not negative-€[e,infinity)
    |> List.filter(fun x -> x.t < x.ray.length)     // If is further no intersection with light, the ray.length means the distance to a light in RayColor.fs

//////////////////////////
//
//  Partial spheres
//
//////////////////////////

let phi  y x =
    // Gives the angle of spherical coordinates in (0,2*pi)
    let prephi = atan2 y x
    if prephi > 0. then prephi
    else prephi+ 2.*PI //Because it goes from -pi to +pi
// Cast the partial sphere
let intersect_PartSphere(ray:RayFrom,psphere:partSphere,nsamples:int) =
    let sphere = psphere.Sphere
    let intersec_sphere = intersection_sphere(ray,sphere,nsamples)
    match intersec_sphere with
    | [] -> []
    | _ -> let translate = sphere.center.ToVector3D().Negate()
           //printfn "Intersecta somewhere"
           let intersect_points = intersec_sphere 
                                  |> List.collect(fun x -> [x.point+translate])                     // Translate
                                  |> List.collect(fun x -> [x.TransformBy(m=psphere.WorldToObj)])   // Rotate
           let phiAndzlist = intersect_points |> List.map(fun x -> ((phi x.Y x.X), x.Z)) // get angles of intersection in uv
           //let zlist = intersect_points |> List.map(fun x -> x.Z)

           // Conditions for intersection with the partial sphere
           let cond (phi, z) =
            // Intersects or not?
            if z> psphere.zmin && z < psphere.zmax then 
                if phi> psphere.phimin && phi < psphere.phimax then true
                else false
            else false

           let part_int (intersec:Intersection,bol:bool)=  
            if bol then [intersec]
            else []
           let bool = phiAndzlist |> List.map(fun x -> cond x) // Bools for the intersection list
           //printfn "phi and z are: %+A" phiAndzlist
           //printfn "The points are: %+A" intersect_points
           //printfn "bols are: %+A" bool
           List.map2(fun x y -> part_int(x, y)) intersec_sphere bool
           |> List.collect(fun x -> x)                                  //Required to solve problem of Intersection list list


let castRay_PartSphere (scene:scene, ray:RayFrom) = 
    scene.World.PartSphere
    |> List.collect (fun x -> intersect_PartSphere(ray, x,scene.Nsamples))  // Find interstion with all the cylinders
    |> List.filter (fun x -> x.t > 1e-10)           // Select the ones that are not negative-€[e,infinity)
    |> List.filter(fun x -> x.t < x.ray.length)     // If is further no intersection with light, the ray.length means the distance to a light in RayColor.fs

////
//Cast the surface of a lens (sphere with a max dia)
////           
let intersect_SurfLens(ray:RayFrom,psphere:SphSurfaceLens,nsamples:int) =
    let sphere = psphere.Sphere
    let intersec_sphere = intersection_sphere(ray,sphere,nsamples)
    match intersec_sphere with
    | [] -> []
    | _ -> let intersect_costh = intersec_sphere 
                                  |> List.collect(fun x -> [psphere.Axis.DotProduct(x.normal)])
           
           //[x.normal.DotProduct(psphere.Axis)])       //  Get the normal x.normal.DotProduct(pshere.Axis)
           //let cosmin = psphere.CosMin
           // Conditions for intersection with the partial sphere - Surface lens
           let cond(costh:float) =
             // Intersects or not?
             if costh < 1. && costh > psphere.CosMin then true
             else false
           let normalConcave(intersection:Intersection,psphere:SphSurfaceLens) =
            // If we the lens is not convex changes the sign og the normal since it's defined for convec
            if psphere.Convex then intersection
            else
                {normal=intersection.normal.Negate();point=intersection.point;ray=intersection.ray;
                 material=intersection.material;t=intersection.t; Nsamples=intersection.Nsamples}

           let part_int (intersec:Intersection,bol:bool)=  
            // compares with the boof if it's intersection or not with the part
            if bol then [intersec]
            else []
           let bool = intersect_costh |> List.map(fun x -> cond x) // Bools for the intersection list
           // Result of intersections of the sph
           List.map2(fun x y -> part_int(x, y)) intersec_sphere bool
           |> List.collect(fun x -> x)                       //Required to solve problem of Intersection list list
           |> List.map(fun x -> normalConcave(x,psphere))    // Changes the normal  if the surface is concave

let castRay_SurfLens (scene:scene, ray:RayFrom) = 
    scene.World.SurfaceLens
    |> List.collect (fun x -> intersect_SurfLens(ray, x,scene.Nsamples))  // Find interstion with all the cylinders
    |> List.filter (fun x -> x.t > 1e-10)           // Select the ones that are not negative-€[e,infinity)
    |> List.filter(fun x -> x.t < x.ray.length)     // If is further no intersection with light, the ray.length means the distance to a light in RayColor.fs




//////////////////////////
//
// Intersection for Cylinders
//
//////////////////////////


let intersect_cyl(ray:RayFrom,cylinder:cylinder,nsamples:int) =
    //Intersection with cylinder, it must be done in local coordinates
    let radius = cylinder.Radius
    // Functions required

    // Transform ray to object space - Origin and then rotate to align with axis of z cylinder
    let newRayOriginObject = (ray.from-cylinder.Origin).ToPoint3D().TransformBy(m=cylinder.WorldToObj)
    let newRayDirObject = ray.uvec.TransformBy(m=cylinder.WorldToObj)                   // Rotate direction
    
    //Compute cylinder quadratic coeficients
    let A = newRayDirObject.X*newRayDirObject.X+newRayDirObject.Y*newRayDirObject.Y
    let B = 2.*(newRayDirObject.X*newRayOriginObject.X+newRayDirObject.Y*newRayOriginObject.Y)
    let C = newRayOriginObject.X*newRayOriginObject.X+newRayOriginObject.Y*newRayOriginObject.Y-radius*radius 

    //Solve equation  for t values
    let disc = B*B-4.*A*C
    if disc < 0. then []
    else
        let sdisc = sqrt(disc)
        let t1 = 0.5*(-B + sdisc)/A   // A is not normalized since A not Mod(RaydirObjec)
        let t2 = 0.5*(-B- sdisc)/A
        let z1 = (newRayOriginObject + newRayDirObject.ScaleBy(t1) )//.ToPoint3D()
        let z2 = (newRayOriginObject + newRayDirObject.ScaleBy(t2))//.ToPoint3D()
        // Create the intersections
        let inter1 = 
            if (z1.Z < cylinder.zmax) && (z1.Z >= cylinder.zmin) then 
                let normalt1 = (Vector3D(z1.X,z1.Y,0.).TransformBy(m=cylinder.ObjToWorld)).Normalize()
                let ray1 =  {uvec=ray.uvec; length=ray.length; from=ray.from; travelled=(t1+ray.travelled)}
                let z1real = 
                    let z1rot = z1.TransformBy(m=cylinder.ObjToWorld)
                    Point3D(z1rot.X+cylinder.Origin.X, z1rot.Y+cylinder.Origin.Y, z1rot.Z+cylinder.Origin.Z)
                [{ normal = normalt1 ; point = z1real; ray = ray1 ; material=cylinder.material; t=t1;Nsamples=nsamples}]
            else []
        let inter2 = 
            if (z2.Z < cylinder.zmax) && (z2.Z >= cylinder.zmin) then 
                let normalt2 = (Vector3D(z2.X,z2.Y,0.).TransformBy(m=cylinder.ObjToWorld)).Normalize()
                let ray2 =  {uvec=ray.uvec; length=ray.length; from=ray.from; travelled=(t2+ray.travelled)}
                let z2real = 
                    let z2rot = z2.TransformBy(m=cylinder.ObjToWorld)
                    Point3D(z2rot.X+cylinder.Origin.X, z2rot.Y+cylinder.Origin.Y, z2rot.Z+cylinder.Origin.Z)
                [{ normal = normalt2 ; point = z2real; ray = ray2 ; material=cylinder.material; t=t2;Nsamples=nsamples}]
            else []
        List.append inter1 inter2   



let castRay_cyl (scene:scene, ray:RayFrom) = 
    scene.World.Cylinder
    |> List.collect (fun x -> intersect_cyl(ray, x,scene.Nsamples))  // Find interstion with all the cylinders
    |> List.filter (fun x -> x.t > 1e-10)           // Select the ones that are not negative-€[e,infinity)
    |> List.filter(fun x -> x.t < x.ray.length)     // If is further no intersection with light, the ray.length means the distance to a light in RayColor.fs

//////////////////////////
//
// Intersection for Meshes
// Intersection of a Square primitive used on Sensor for fowrard ray tracing
//
//////////////////////////

let intersec_mesh (ray:RayFrom,  mesh:mesh,triangle:int list,nrm:UnitVector3D, nsamples:int, shape:char)= // normal is only passing
    // Method 2 from PBRT book eq 3.5
    let nodes = mesh.Vertices // *** Is wasting memory --> It copyes the direction, no waste memory *** I can: mesh.Vertices.[n0]

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
            if (u>0. && v>0. && (u+v)<=1.) then true
            else false
        elif shape = 's' then // intersection with a square condition - PArallelograms (Not trapezoids)
            if (u>0. && v>0. && u<=1. && v <= 1.) then true
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
let intersec_square (ray, mesh,triangle,nrm,nsamples)= intersec_mesh (ray, mesh,triangle,nrm,nsamples,'s') // Parallelograms
//


let castRay_mesh (scene:scene, ray:RayFrom) = //here it's only for sphere
    let interceptions (ray:RayFrom, mesh:mesh,nsamples)  =
        [0..(List.length(mesh.Triangles)-1)]
        |> List.collect (fun x -> intersec_tri(ray, mesh, mesh.Triangles.[x],mesh.normals.[x],nsamples))
        |> List.filter (fun x -> x.t > 1e-10)  //
    // Meshes_intersections = castRay_mesh(scene,ray)
    let CastBBox(mesh:mesh,ray:RayFrom) =
        let bolean = BBox_intersec( mesh.Bbox, ray) 
        if bolean then interceptions(ray,mesh,scene.Nsamples)
        else []
    (*
    let aCastBBox mesh ray =  // Parallel algorithm
        let a = CastBBox(mesh,ray)
        async{return a}                                 // The async type creation
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
    let Cylinder_intersections = castRay_cyl(scene,ray)
    let PartSpheres_intersections = castRay_PartSphere (scene, ray)
    let SurfaceLens = castRay_SurfLens (scene, ray)

    let first = List.append Spheres_intersections Meshes_intersections //append is 2 lists
    first@Cylinder_intersections@PartSpheres_intersections@SurfaceLens