module RayColorGrid
//
//  Things of this module:
//  ColorAt -> Shadowing funtion
//              Includes: -Direct illumination from:
//                                  *Point lights
//                                  *Circular lights (sampling them and generating equivalent Plights)
//                        -Diffuse (N·L) and Specular (L·R)
//  Global Illumination methods:
//      The number of iteration/branches defined here ---> Should be modified in the future
//      Reflection/Transmision 
//      Transmitted and Reflected via Fresnel coeficient can be computed
//

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayCore
open RayType
open RandomMethods
open RayCoreGrid

(*let colorAt (intersection:Intersection,scn:scene )=
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
*)
let colorAt (intersection:Intersection,scn:scene,partition:Grid3D list)=
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
    let light = scn.Light.Point // Point Light
    let circleLight = scn.Light.Circle // Circle Light list
    let Ia = Color(0.05,0.05,0.05) //Should be defined with the scene
    let AmbLight = Ia*intersection.material.DiffuseLight //Ia*Ka*Od
//    let IsShadow intersection light =
//        let LightDir = light.origin - intersection.point
//        let RayLight = {uvec=LightDir.Normalize();length=LightDir.Length; from=intersection.point; travelled=intersection.ray.travelled}
//        let intersects = CastRay_nest (scn, RayLight)
//        //printfn "there's an intersection at: %f" intersects.Head.ray.travelled
//        match intersects with
//            |[] -> true
//            |_ ->  false


    // Diffuse
    let DiffLight (intersection, light:Plight ,fatt:float, normLightDir:UnitVector3D) =
        let KdOd = intersection.material.DiffuseLight
        //let LightDir = light.origin - intersection.point
        //let NormLightDir = LightDir.Normalize() //dir from point to light
        let Id = light.color* (max (intersection.normal.DotProduct(normLightDir)) 0.0)
        //let fatt= Fatt intersection light
        //printfn "Distance travelled is: %f" fatt
        KdOd*Id*fatt*light.intensity//if IsShadow intersection light then KdOd*Id*fatt*light.intensity //Id*Kd*Od
        //else Color(0.0,0.0,0.0)
    //Specular
    let SpecLight (intersection, light:Plight ,fatt:float, normLightDir:UnitVector3D)=
        let Ks =  intersection.material.SpecularLight //reflectance specular - MODIFY IT in FUTURE
        //let LightDir = light.origin - intersection.point
        //let NormLightDir = LightDir.Normalize() //dir from point to light
        let Rvect = intersection.normal.ScaleBy(2.0*intersection.normal.DotProduct(normLightDir))-normLightDir
        let DotProds = max (Rvect.DotProduct(intersection.ray.uvec.ScaleBy(-1.0))) 0.0
        let Is = light.color *pown DotProds intersection.material.shinness
        //let fatt= Fatt intersection light
        Ks*Is*fatt*light.intensity//if IsShadow intersection light then Ks*Is*fatt*light.intensity
        //else Color(0.0,0.0,0.0)
    // Sum
    let IsShadow partition intersection light =
        // If LightDir.Length = 1 the function will FAIL
        let LightDir = light.origin - intersection.point
        let NormLightDir = LightDir.Normalize()
        let RayLight = {uvec=NormLightDir; from=intersection.point; length=LightDir.Length; travelled=intersection.ray.travelled}
        let intersects = Cast_3DGrid(scn, RayLight,partition)//CastRay_nest//CastRay_nest (scn, RayLight)
        //printfn "there's an intersection at: %f" intersects.Head.ray.travelled
        match intersects with
            |[] -> let fatt= Fatt intersection light //let unit = paralel intersection
                   DiffLight (intersection, light, fatt, NormLightDir) + SpecLight (intersection, light, fatt, NormLightDir)
            |_ ->  Color(0.0,0.0,0.0)
    //let DiffSimp light = DiffLight intersection light
    //let SpecSimp light = SpecLight intersection light    
    let OneShadow light = IsShadow partition intersection light

    let ExtendedLight (intersection, clight:Clight) =
        // Only Circle light
        // when other shpahes do a match to select the sphape
        //
        // Computes the color with one random point of the surface -> CircleLight -> PointLight
        // and considers the fraction of the intensity
        let (RandR,RandPhi) = SampDisk clight.Radius
        let (x,y) = (RandR*cos(RandPhi),RandR*sin(RandPhi))
        let Rpoint = Point3D(x,y,0.0)
        let shape = Rpoint.TransformBy(m=clight.RotationMatrix) // I must rorate the normal vector (0.,0.,1.) -> (xn,yn,zn)
        let lightPoint = Point3D(shape.X+clight.Centre.X,shape.Y+clight.Centre.Y,shape.Z+clight.Centre.Z)
        let Ldotl = max (clight.Normal.DotProduct((intersection.point-lightPoint).Normalize())) 0.// directionality of light l·L
        let tempLight = {origin = lightPoint;color = clight.color; intensity =(clight.intensityDensity  *(clight.Area/float(intersection.Nsamples)))} // {origin:Point3D; color:Color; intensity:float} 
        match Ldotl with        // If 0.0 I save time computing the shadowing
        | 0. -> Color(0.,0.,0.)
        |_ -> (OneShadow tempLight)*Ldotl

    let ExtendedShadow cLight= [1..intersection.Nsamples] |> List.sumBy(fun x -> (ExtendedLight (intersection, cLight)))
    AmbLight + List.sumBy(fun x -> OneShadow x) light + List.sumBy(fun x -> ExtendedShadow x) circleLight//+ List.sumBy(fun x -> SpecSimp x) light

//
//
 // This will be activated on material with a material.Fresnel = True/False

let cosinus_in_tr (normal:UnitVector3D, dir:UnitVector3D, nu0) =
  let ci0 = normal.DotProduct(dir)
  let SideRay (ci,index) =   
    // Changes the situation checking from air or to  
    if ci < 0.  then 
      (-ci, 1./index) 
    else
      (ci, index)
  let (ci, nu) = SideRay(ci0,nu0)

  let AngCritic n_transm =
    // Obtain Critical angle for TIR
    if n_transm > 1. then
      1.571
    else
      let tir = asin(n_transm) // Pi/2  
      tir

  let ang_critic = AngCritic nu
  let ang_inc = acos(ci)
  if ang_inc < ang_critic then
    let inv_n =1./nu 
    let ct = sqrt(1.-(inv_n*inv_n )*(1.-ci*ci)) // Cosinus transmited
    (ci,ct, nu)
  else
    //3.14159265358979323846/2. //Pi/2
    (ci,0.0, nu)

let RFresnel (ci:float, ct:float, nu:float) =
  // ALways taking into account that it's done with material/air interfase 
  // nu = n1/n2 = nTo/nFrom
  // Hall illumination model[HALL83]
  let term1 = pown ((ci/nu-ct*nu)/(ci/nu+ct*nu)) 2
  let term2 = pown ((ci*nu-ct/nu)/(ci*nu+ct/nu)) 2
  if ct = 0. then  (0.0,0.5*(term1+term2)) 
  else (1.-0.5*(term1+term2),0.5*(term1+term2))

//
//  GLOBAL ILLUMINATION
//
let rec ReflectedRay (intersection:Intersection,scene:scene,dpt:int,partition:Grid3D list) =
    let RayDir = intersection.ray.uvec
    let NormLightDir = RayDir.Negate()//.ScaleBy(-1.0) //Inverse of the ray direction to reflect
    let ReflRayv = intersection.normal.ScaleBy(2.0*intersection.normal.DotProduct(NormLightDir))-NormLightDir //Reflected ray it's unit
    let ReflRay = {uvec=ReflRayv.Normalize(); length=infinity; from =intersection.point;travelled=intersection.ray.travelled}
    // Ray.length is created to allow shadows, it's not required more, so it creates a problem if it's not infinity intersecting objects
    let intersects = Cast_3DGrid(scene,  ReflRay,partition)//CastRay_nest//CastRay_nest (scene, ReflRay)
    match intersects with
        |[] -> Color(0.0,0.0,0.0) // No reflection returns black             
        |_ -> let intersecMin = (intersects |>List.minBy(fun x -> x.t) ) //colorvar//Intersection
              let rcolor= colorAt(intersecMin, scene,partition)
              let Depth = dpt + 1
              let LenghtMax = 400.0
              if intersection.ray.travelled < LenghtMax && Depth < 2 then
                let (T, R) =
                    // Whitte ilumination model
                    if intersection.material.Fresnel = false then 
                        (intersection.material.T, intersection.material.R)
                    else
                        // Hall illumination model[HALL83]
                        let (ci,cr,nu)= cosinus_in_tr (intersection.normal, NormLightDir, intersection.material.n)
                        RFresnel(ci,cr,nu)
                    
                
                if T=0. then rcolor+ReflectedRay(intersecMin,scene,Depth,partition)*R
                else rcolor+ReflectedRay(intersecMin,scene,Depth,partition)*R+TransmittedRay(intersecMin,scene,Depth,partition)*T
               
              else
                rcolor

and TransmittedRay (intersection:Intersection,scene:scene,dpt:int,partition:Grid3D list) =
    let RayDir = intersection.ray.uvec
    let LightDir = RayDir.Negate() //Ray that incides on the surface * -1
    let SideRay (ci,index) =   
        // Changes the situation checking from air or to  
       if ci < 0.  then 
        (-ci, 1./index) 
       else
        (ci, index)
    let n = intersection.material.n // With AIR
    let ci = intersection.normal.DotProduct(LightDir) //Cosinus incident angle
    (*
    if ci < 0. then
        // ci < 0 means that goes from inside to outside
        printfn "Hola" 
    else
        printfn "ciao"   *)    
    let (cos_inc,nu) = SideRay(ci, n)
    let inv_n = 1./nu // It is used the inverse = (n_from/n_to)
    let AngCritic n_transm =
        // Obtain Critical angle for TIR
        if n_transm > 1. then
            1.571
        else
            let tir = asin(n_transm) // Pi/2  
            tir
    let ang_critic = AngCritic nu
    let ang_inc = acos(cos_inc)

    if ang_inc < ang_critic then // TIR
        let cos_trans = sqrt(1.-(inv_n*inv_n )*(1.-cos_inc*cos_inc)) // Cosinus transmited
        let vtrans = RayDir.ScaleBy(inv_n) - intersection.normal.ScaleBy(cos_trans - inv_n*cos_inc)
        let TransRay = {uvec=vtrans.Normalize();length=infinity; from =intersection.point;travelled=intersection.ray.travelled}
        // Ray.length is created to allow shadows, it's not required more, so it creates a problem if it's not infinity intersecting objects
        let intersects = Cast_3DGrid(scene, TransRay,partition)//CastRay_nest (scene, TransRay)
        match intersects with
            |[] -> Color(0.0,0.0,0.0)
            |_ -> let intersecMin = (intersects |>List.minBy(fun x -> x.t) )
                  let tcolor = colorAt(intersecMin, scene,partition)
                  let Depth = dpt + 1 //Idem as reflected
                  let LenghtMax = 400.0
                  if intersection.ray.travelled < LenghtMax && Depth < 2 then
                      let (T, R) = 
                        // Whitte illumination model
                        if intersection.material.Fresnel = false then (intersection.material.T, intersection.material.R)
                        else 
                            // Hall illumination model[HALL83]
                            RFresnel(cos_inc,cos_trans,nu)
                                                   
                      if T=0. then tcolor + ReflectedRay(intersecMin,scene,Depth,partition)*R // Don't do trans f not trans next collision
                      else tcolor + TransmittedRay(intersecMin,scene,Depth,partition)*T + ReflectedRay(intersecMin,scene,Depth,partition)*R
                  else
                      tcolor
    else
       Color(0.0,0.0,0.0) // Total Internal Reflection
 
let GlobalIllum (intersection:Intersection, scene:scene,partition:Grid3D list) =
    let DirColor = colorAt(intersection, scene,partition) //First  direct color
    let (T, R) = 
        //Whitte ilumination model
        if intersection.material.Fresnel = false then (intersection.material.T, intersection.material.R)
        else 
            // Hall illumination model[HALL83]
           let (ci,cr,nu)= cosinus_in_tr (intersection.normal, intersection.ray.uvec.Negate(), intersection.material.n)
           RFresnel(ci,cr,nu)
    let RefColor = ReflectedRay (intersection,scene,0,partition)*R // Generate ray reflected
    let TransColor = TransmittedRay (intersection,scene,0,partition)*T //Generate Ray transmited
    DirColor+RefColor+ TransColor
