module RayCoreOctree


open System
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayType
open BBox 
open RayCore
open RayCoreGrid
open Octree



//
// Check intersection of the Octree
//
// Cast_3DGrid (scene,ray,grid:Grid3D list) // Function to do the grid intersection
let rec Cast_Octree(scene,ray,octreeSystem:OctreeSystem list) =
    let matchOctSystem (octSys, ray,scene)= 
        match octSys with
        | Octree x -> if BBox_intersec( x.Bbox, ray) then                      
                        Cast_Octree(scene,ray,x.Octree)      
                        //if result = [] then
                          //  result
                        //else 
                            //printfn "Intersects something inside the octree"
                            //result
                      else 
                        []
        | Partition x -> if BBox_intersec( x.Bbox, ray) then
                            InsidePartition (x, scene, ray) 
                         else
                            []
    let OctIter = octreeSystem |> List.collect(fun x -> matchOctSystem(x, ray,scene))
    OctIter
