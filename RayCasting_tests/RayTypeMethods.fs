module RayTypeMethods
// This part of the program will be methods to initialize the types of Raytype.fs

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml
//open RayCore
//#load "RayType.fs"
open RayType
let PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062

// Funtion to generate a clight (Circular light)
let GenerateClight(centre:Point3D,normal:UnitVector3D,radius:float, color:Color, intensityDensity:float) =
    // Initiate a Clight
    let RotationMatrix=Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),normal)
    let Area = PI*radius*radius
    {Centre=centre;                      // Centre of the light-disc
     Normal=normal;                 // Direction in which the light points
    Radius=radius;Area=Area;             // Radius and Area (are not independent)
    color=color; intensityDensity=intensityDensity; // Color of the light and its intensity
    RotationMatrix=RotationMatrix       // Rotation from the UnitVector3D(0.,0.,1.) -> Normal
   }


//Funtion to generate a partial sphere
let GeneratePartialSphere(center, radius, material, zmin,zmax,phimin,phimax, normal:UnitVector3D) =
    let sphere = {center=center; radius=radius; material=material }
    let ObjToWorld = Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),normal)
    let WorldToObj = Matrix3D.RotationTo(normal,UnitVector3D(0.,0.,1.))

    {
    Sphere = sphere;
    zmin=zmin;zmax=zmax;
    phimin=phimin;phimax=phimax;
    ObjToWorld=ObjToWorld;
    WorldToObj=WorldToObj    
    }

let GenerateLensSurface(center, radius, material,diameter:float,axis:UnitVector3D, convex:bool) =
    // Define the surface type defined by a sphere with a maximum radius and an axis
    let sphere = {center=center; radius=radius; material=material }
    let sinth = 0.5*diameter/radius
    let costh =     // Carefull if diameter of the lens is bigger than the 2*radius of the lens => no sense
        if sinth < 1. then sqrt(1.-(sinth*sinth)) 
        else 0.     
    {
    Sphere = sphere;
    CosMin= costh;
    Axis = axis;
    Convex= convex 
    }



/////////////////

// Funtion to generate a cylinder
let GenerateCylinder(radius:float, zmin:float,zmax:float,origin:Point3D,normal:UnitVector3D,material:material) =
    // Initiate a cylider from the basic types
    
    let ObjToWorld = Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),normal)
    let WorldToObj = Matrix3D.RotationTo(normal,UnitVector3D(0.,0.,1.))
    let LBbox = {Pmin = [-radius;-radius;zmin]; Pmax = [radius;radius;zmax]}//lBbox
    let NonTransformedEdges = [LBbox.Pmin;
                               [LBbox.Pmin.[0];LBbox.Pmax.[1];LBbox.Pmin.[2]];
                               [LBbox.Pmax.[0];LBbox.Pmin.[1];LBbox.Pmin.[2]];
                               [LBbox.Pmax.[0];LBbox.Pmax.[1];LBbox.Pmin.[2]];
                               
                               [LBbox.Pmin.[0];LBbox.Pmin.[1];LBbox.Pmax.[2]];
                               [LBbox.Pmax.[0];LBbox.Pmin.[1];LBbox.Pmax.[2]];
                               [LBbox.Pmin.[0];LBbox.Pmax.[1];LBbox.Pmax.[2]];
                               LBbox.Pmax]

    let TransformedEdges = NonTransformedEdges                  // Rotate as should be by normal direction
                           |> List.collect(fun x -> [Point3D(x).TransformBy(m=ObjToWorld)])
    let minTrfEdgesX =  (TransformedEdges |> List.minBy(fun x -> x.X)).X   
    let minTrfEdgesY =  (TransformedEdges |> List.minBy(fun x -> x.Y)).Y
    let minTrfEdgesZ =  (TransformedEdges |> List.minBy(fun x -> x.Z)).Z 
    // Min of the Box in world
    let wPmin = [minTrfEdgesX+origin.X; 
                 minTrfEdgesY+origin.Y; 
                 minTrfEdgesZ+origin.Z]

    let maxTrfEdgesX =  (TransformedEdges |> List.maxBy(fun x -> x.X)).X   
    let maxTrfEdgesY =  (TransformedEdges |> List.maxBy(fun x -> x.Y)).Y 
    let maxTrfEdgesZ =  (TransformedEdges |> List.maxBy(fun x -> x.Z)).Z
    // Max of Box in World
    let wPmax = [maxTrfEdgesX+origin.X; 
                 maxTrfEdgesY+origin.Y;     
                 maxTrfEdgesZ+origin.Z]

    {Radius =radius;
    zmin = zmin; zmax = zmax;
    LBbox = LBbox;
    WBbox = {Pmin=wPmin; Pmax =wPmax};
    Origin = origin;            // Position
    Normal = normal;            // Orientation
    material = material;
    ObjToWorld=ObjToWorld;
    WorldToObj=WorldToObj    
    }

// Example with cylinder to see that it is correctly generated
//let mat1 ={DiffuseLight = Color(0.90,0.90,0.990);SpecularLight = Color(0.51,0.51,0.51);shinness= 6; R=0.50; T=0.50; n= 1.3;Fresnel = true}
//let ciiil = GenerateCylinder(0.5,0.,1.,Point3D(0.,0.0,1.),UnitVector3D(0.,1.0,0.), mat1)
//let ray11 = {uvec = UnitVector3D(0.0,1.0,-1.0); length = infinity; from = Point3D(0.0,0.,1.); travelled = 0.}
//let intersecti = intersect_cyl(ray11,ciiil,2) 
// Works (0.0,0.25,5.0)

let CreateLensBiConvex(r1:float, r2:float, axis:UnitVector3D, th:float, dia:float, startingPoint:Point3D, mat:material)=
    // Function to create a biconvex lens with two lens surface and one cylinder.
    // All defined with respect to s1 -> this means that for cyl and s2 the axis is the opposite
    let naxis = axis.Negate()
    let S1 = GenerateLensSurface(startingPoint+naxis.ScaleBy(r1), r1, mat, dia,axis,true)
    let S1Th = r1*(1.-S1.CosMin)
    let S2 = GenerateLensSurface(startingPoint+naxis.ScaleBy(th-r2), r2, mat, dia,naxis,true)
    let S2Th = r2*(1.-S2.CosMin)
    let cylth = th-S1Th-S2Th
    let cylSide = GenerateCylinder(dia/2., 0.,cylth,startingPoint+naxis.ScaleBy(S1Th),naxis,mat)
    (S1,cylSide,S2)

let CreateLensBiConcave(r1:float, r2:float, axis:UnitVector3D, th:float, dia:float, startingPoint:Point3D, mat:material)=
    // Function to create a biconvex lens with two lens surface and one cylinder.
    // All defined with respect to s1 -> this means that for cyl and s2 the axis is the opposite
    let naxis = axis.Negate()
    let S1 = GenerateLensSurface(startingPoint+axis.ScaleBy(r1), r1, mat, dia,naxis,false)
    let S1Th = r1*(1.-S1.CosMin)
    let S2 = GenerateLensSurface(startingPoint+naxis.ScaleBy(th+r2), r2, mat, dia,axis,false)
    let S2Th = r2*(1.-S2.CosMin)
    let cylth = th+S1Th+S2Th
    let cylSide = GenerateCylinder(dia/2., 0.,cylth,startingPoint+axis.ScaleBy(S1Th),naxis,mat)
    (S1,cylSide,S2)
