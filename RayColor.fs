module RayColor

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayCore
open RayType

let colorAt (intersection:Intersection,scn:scene )=
    // Here is the function that defines the shader
    (*
    //As a starting I want to obtain only the dot product
    let FirstShading (sphere:sphere, ray:RayFrom)=
        let dotproduct= ray.ray.Direction.DotProduct(intersection.normal)
        let colore = sphere.color*sqrt(dotproduct*dotproduct) //Must be positive
        colore
    FirstShading (intersection.sphere, intersection.ray)*)
    // I want to implement the Phong model:
    //      I = Ia*Ka*Od+Fatt*Ipoint[Kd*Od(N·L)+Ks*Os(R·V)^n]
    //                  Ka+Kd+Ks<=1
    //                  Ka=0.1*(rgb)
    //                  Kd=sphere.color(rgb)
    //                  Ks=0.5*(rgb) -> first instance is white
    //Ambient light
    let Fatt intersection light=
        // attenuator for distance
        let LightDist = (light.origin - intersection.point).Length
        let distance  = intersection.ray.travelled + LightDist
        //printfn "the .t is: %f and the ray.travelled: %f" intersection.t intersection.ray.travelled
        List.min([1.0/(distance*distance); 1.0])
        //1.0
    let light = scn.Light
    let Ia = Color(0.05,0.05,0.05) //Should be defined with the scene
    let AmbLight = Ia*intersection.material.DiffuseLight //Ia*Ka*Od
    let IsShadow intersection light =
        let LightDir = light.origin - intersection.point
        let RayLight = {uvec=LightDir.Normalize();length=LightDir.Length; from=intersection.point; travelled=intersection.ray.travelled}
        let intersects = CastRay_nest (scn, RayLight)
        //printfn "there's an intersection at: %f" intersects.Head.ray.travelled
        match intersects with
            |[] -> true
            |_ ->  false
    // Diffuse
    let DiffLight intersection light    =
        let KdOd = intersection.material.DiffuseLight
        let LightDir = light.origin - intersection.point
        let NormLightDir = LightDir.Normalize() //dir from point to light
        let Id = light.color* List.max([intersection.normal.DotProduct(NormLightDir);0.0])
        let fatt= Fatt intersection light
        //printfn "Distance travelled is: %f" fatt
        if IsShadow intersection light then KdOd*Id*fatt*light.intensity //Id*Kd*Od
        else Color(0.0,0.0,0.0)
    //Specular
    let SpecLight intersection light =
        let Ks =  intersection.material.SpecularLight //reflectance specular - MODIFY IT in FUTURE
        let LightDir = light.origin - intersection.point
        let NormLightDir = LightDir.Normalize() //dir from point to light
        let Rvect = intersection.normal.ScaleBy(2.0*intersection.normal.DotProduct(NormLightDir))-NormLightDir
        let DotProds = List.max([Rvect.DotProduct(intersection.ray.uvec.ScaleBy(-1.0));0.0])
        let Is = light.color *pown DotProds intersection.material.shinness
        let fatt= Fatt intersection light
        if IsShadow intersection light then Ks*Is*fatt*light.intensity
        else Color(0.0,0.0,0.0)
    // Sum

    let DiffSimp light = DiffLight intersection light
    let SpecSimp light = SpecLight intersection light    


    AmbLight + List.sumBy(fun x -> DiffSimp x) light + List.sumBy(fun x -> SpecSimp x) light

//
//
   

let rec ReflectedRay (intersection:Intersection,scene:scene,dpt:int) =
    let RayDir = intersection.ray.uvec
    let NormLightDir = RayDir.Negate()//.ScaleBy(-1.0) //Inverse of the ray direction to reflect
    let ReflRayv = intersection.normal.ScaleBy(2.0*intersection.normal.DotProduct(NormLightDir))-NormLightDir //Reflected ray it's unit
    let ReflRay = {uvec=ReflRayv.Normalize(); length=ReflRayv.Length; from =intersection.point;travelled=intersection.ray.travelled}
    let intersects = CastRay_nest (scene, ReflRay)
    match intersects with
        |[] -> Color(0.0,0.0,0.0) // No reflection returns black             
        |_ -> let intersecMin = (intersects |>List.minBy(fun x -> x.t) ) //colorvar//Intersection
              let rcolor= colorAt(intersecMin, scene)
              let Depth = dpt + 1
              let LenghtMax = 400.0
              if intersection.ray.travelled < LenghtMax && Depth < 4 then 
                let (T, R) = (intersection.material.T, intersection.material.R)
                
                rcolor+ReflectedRay(intersecMin,scene,Depth)*R+TransmittedRay(intersecMin,scene,Depth)*T
              else
                rcolor

and TransmittedRay (intersection:Intersection,scene:scene,dpt:int) =
    let RayDir = intersection.ray.uvec
    let LightDir = RayDir.Negate() //Ray that incides on the surface * -1
    let SideRay (ci,index) =   
        // Changes the situation checking from air or to  
       if ci < 0.  then 
        (-ci, 1./index) 
       else
        (ci, index)
    let nu = intersection.material.n // With AIR
    let ci = intersection.normal.DotProduct(LightDir) //Cosinus incident angle
    (*
    if ci < 0. then
        // ci < 0 means that goes from inside to outside
        printfn "Hola" 
    else
        printfn "ciao"   *)    
    let (cos_inc,n) = SideRay(ci, nu)
    let inv_n = 1./n // It is used the inverse
    let AngCritic n_transm =
        // Obtain Critical angle for TIR
        if n_transm > 1. then
            1.571
        else
            let tir = asin(n_transm) // Pi/2  
            tir
    let ang_critic = AngCritic n
    let ang_inc = acos(cos_inc)

    if ang_inc < ang_critic then // TIR
        let cos_trans = sqrt(1.-(inv_n*inv_n )*(1.-cos_inc*cos_inc)) // Cosinus transmited
        let vtrans = RayDir.ScaleBy(inv_n) - intersection.normal.ScaleBy(cos_trans - inv_n*cos_inc)
        let TransRay = {uvec=vtrans.Normalize();length=vtrans.Length; from =intersection.point;travelled=intersection.ray.travelled}
        let intersects = CastRay_nest (scene, TransRay)
        match intersects with
            |[] -> Color(0.0,0.0,0.0)
            |_ -> let intersecMin = (intersects |>List.minBy(fun x -> x.t) )
                  let tcolor = colorAt(intersecMin, scene)
                  let Depth = dpt + 1 //Idem as reflected
                  let LenghtMax = 400.0
                  if intersection.ray.travelled < LenghtMax && Depth < 4 then
                      let (T, R) = (intersection.material.T, intersection.material.R)
                      if T=0. then tcolor + ReflectedRay(intersecMin,scene,Depth)*R // Don't do trans f not trans next collision
                      else tcolor + TransmittedRay(intersecMin,scene,Depth)*T + ReflectedRay(intersecMin,scene,Depth)*R
                  else
                      tcolor
    else
       Color(0.0,0.0,0.0) // Total Internal Reflection
 
let GlobalIllum (intersection:Intersection, scene:scene) =
    let DirColor = colorAt(intersection, scene) //First  direct color
    let RefColor = ReflectedRay (intersection,scene,0)*intersection.material.R // Generate ray reflected
    let TransColor = TransmittedRay (intersection,scene,0)*intersection.material.T //Generate Ray transmited
    DirColor+RefColor+ TransColor
