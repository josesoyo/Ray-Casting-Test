#r @"packages\MathNet.Spatial.0.2.0-alpha\lib\net40\MathNet.Spatial.dll"
#r @"packages\MathNet.Numerics.Signed.3.7.0\lib\net40\MathNet.Numerics.dll"
#r @"packages\MathNet.Numerics.FSharp.Signed.3.7.0\lib\net40\MathNet.Numerics.FSharp.dll"

#load "RayCore.fs"
#load "RayColor.fs"


open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayCore
open RayColor


//Definitions of values
//
// Pixels
let PixNumW = 600
let PixNumH = 600
let PixWide = 2.0/float(PixNumW) //Write something
let PixHeigh = 2.0/float(PixNumH)
//
//Camera: observer and sensor
//let sensor = Sensor(PixWide, PixHeigh, PixNumW, PixNumH) 
let camera={EyePoint=Point3D(0.0,-2.5,0.0);LookAt=Vector3D(0.0,2.5,0.0); Up=Vector3D(0.0,0.0,1.0)}// ;film=sensor} 
let light = {origin = Point3D(-0.50,4.50,2.50);color=Color(1.0,1.0,1.0); intensity = 100.0}
let light2 = {origin = Point3D(-0.50,4.50,-2.50);color=Color(1.0,1.0,1.0); intensity = 60.0}

//
// scene
// I define the plane and sphere

let pointWall = Point3D(0.0, 0.0, 10.0)
let WallVect = UnitVector3D(0.0,0.0,1.0) 
let Surf = Plane(pointWall, WallVect)
let Wall = {surf=Surf; Color=0.6}
let mat1= {DiffuseLight = Color(0.2,0.2,0.2);SpecularLight = Color(0.5,0.5,0.9);shinness= 60; R=0.02; T=0.95; n= 1.85} 
let ball = {center=Point3D(-0.0,1.0,0.0); radius=0.25; material=mat1 }
let mat2 ={DiffuseLight = Color(0.2,0.7,0.2);SpecularLight = Color(0.5,0.5,0.9);shinness= 60; R=0.72; T=0.0; n= 1.45} 
let ball2 = {center=Point3D(-1.0,9.0,1.50); radius=2.0; material=mat2 }
// Do the scene5
let Scene = {Camera=camera ;Sphere = [ball;ball2]; EndWorld = Wall; Light=[light;light2]} 
//ball;ball2;ball3;ball4;ball5


//Viewing Coordinate System w u v
let w = camera.LookAt.Normalize()
let u = camera.Up.CrossProduct(w).Normalize()
let v = w.CrossProduct(u)
// Bucle
let bmp = new Bitmap(PixNumW,PixNumH)

//Forward ray tracing
for i in 0..(PixNumW-1) do 
    for j in 0..(PixNumH-1) do
            // create a ray from eye to pixel
            let direct = camera.LookAt+u.ScaleBy(PixWide*float(PixNumW/2-i))+v.ScaleBy(PixHeigh*float(PixNumH/2-j)) // THE REAL ONE SHOULD BE
            let Ray = {uvec=direct.Normalize(); length=infinity; from=camera.EyePoint; travelled=0.0 } //BAD
            let intersects = castRay (Scene, Ray)
            match intersects with
            | [] -> bmp.SetPixel(i,j,Color.Gray)
            | _ ->  let color = GlobalIllum(intersects |>List.minBy(fun x -> x.t), Scene  )//colorAt (intersects |>List.minBy(fun x -> x.t), Scene  )
                     //(fun x -> x) |> |> List.head S
                    //printfn "Color is: %+A" color
                    bmp.SetPixel(i,j,Color.FromArgb(int(255.0*color.r),  int(255.0*color.g) , int(255.0*color.b) ))
            //bmp.SetPixel(i,j,Color.Red) // // selecto the min one
            (*
            //Intersec.X*Intersec.X+Intersec.Y*Intersec.Y |> shading <-shading + sqrt 
            shading <- shading + sqrt(Intersec.X*Intersec.X+Intersec.Y*Intersec.Y) 
            sensor.value.[i,j] <- shading // I do the square
            //
            //
            // To see if it's working the shading that it's a radius
            //
            if 30.0*shading > 254.96 then 
                shading <- 254./30.0
            bmp.SetPixel(i,j,Color.FromArgb(int(30.0*shading),  int(30.0*shading) , int(30.0*shading) ))
            *)


let form = new Form(Text="Rendering test",TopMost=true)
form.Show()

let img = new PictureBox(Dock=DockStyle.Fill)
form.Controls.Add(img)

img.Image <- bmp

//bmp.Save(@"C:\Users\JoseM\Desktop\test_2r.jpg")
//printfn "ended!"
