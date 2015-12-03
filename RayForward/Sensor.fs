module SensorModule
//The things expected to be done in this module are te sensor type
//
//  Functions here:
//      type sensor
//      InitiateSensor -> Create a sensor Type
//      sensorUpdate -> Check photon arriving and update the sensor
//          -FindSensorInter
//          -updateMatrix 
//      SensorToImage

open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml
open System.IO
open System.Windows.Forms
open System.Drawing
open RayType

//
//  Type
//

type Sensor = {//This should be the type
               Origin:Point3D;          // Corner of the sensor
               c1: Point3D
               c2: Point3D              //the points to form a mesh
               Normal:UnitVector3D;     // Direction of the sensor = Dir of camera
               xpix :int; ypix: int     // (1,0,0) y (0,1,0) -> Rotated
               pixsize:float            // Suposed rectangular
               //Up not required since it's up +Y in local coordinates - CAREFULL
               RotationMatrix:Matrix<float> // From local coordinates to global coordinates
               
               //Real sensor properties
               PhotosSature: int // Number of photons that saturate the image
               Rcolor:Matrix<float> // Saves number of photons received
               Gcolor:Matrix<float>
               Bcolor:Matrix<float>
               }

let InitiateSensor (origin:Point3D, normal, xpix, ypix, pixsize, photosSature) =
    // Initiate the sensor 
    // Always based in the normal in Local coordinates being (0,0,1)
    let RotationMatrix = Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),normal)
    //The edges of the sensor:    
    let c1 = 
        let maxx = float(xpix)*pixsize
        let c1s = (Point3D(maxx,0.,0.).TransformBy(m=RotationMatrix))
        Point3D(c1s.X+origin.X,c1s.Y+origin.Y,c1s.Z+origin.Z)
    let c2 = 
        let maxy = float(ypix)*pixsize
        let c2s = (Point3D(0.,maxy,0.).TransformBy(m=RotationMatrix))
        Point3D(c2s.X+origin.X,c2s.Y+origin.Y,c2s.Z+origin.Z)

    //Color Matrices
    //let Rcolor = Double.SparseMatrix(xpix, ypix) //(fun 2 2  -> 0)        // To initialize - GOOD??
    let Mcolor = SparseMatrix.init xpix ypix (fun i j -> 0.)       // To initialize - GOOD??
    // All colors are initialized as zero...
    //let Gcolor = SparseMatrix.init xpix ypix (fun i j  -> 0.)        // To initialize
    //let Bcolor = SparseMatrix.init xpix ypix (fun i j  -> 0.)        // To initialize
    
    // Return the initiated sensor
    {Origin = origin;
    c1= c1 ; c2 = c2;
    Normal = normal;
    xpix = xpix ; ypix = ypix; pixsize = pixsize;
    RotationMatrix = RotationMatrix;
    PhotosSature = photosSature;
    Rcolor = Mcolor; Gcolor = Mcolor; Bcolor = Mcolor
    }
//
// Methods on Sensor
//

let FindSensorInter(sensor:Sensor,pinter:Point3D) =
    // Find the point of intersection in sensor coordinates normal=(0.,0.,1.)
    //
    //Gives an impossible point in case it's not at z=0
    let RotationMatrix= sensor.RotationMatrix//
    let RFromWorldToSensor = RotationMatrix.Inverse()            // Rotates from the coordinates of the world to the sensor
    let TFromWorldToSensor = sensor.Origin.ToVector3D().Negate()   // Translates from world coordinates to sensor coordinates (0,0,0) = Origin
    
    // 1st Translate and 2nd Rotate (the order is important)
    let SensorCoordinatesIntersection = (pinter+TFromWorldToSensor).TransformBy(m=RFromWorldToSensor) // Should be a Point3D

    if SensorCoordinatesIntersection.Z < 1e-10 && SensorCoordinatesIntersection.Z > -1e-10 then SensorCoordinatesIntersection
    else Point3D(infinity,infinity,SensorCoordinatesIntersection.Z)//__RaiseError__  -> sure this will raise it... (to check)
        

let updateMatrix(xpix:int,ypix:int,xPixPos, yPixPos,value:float)=
    //Update a sparse matrix that works as the sensor
    // Exist updateComplexMatrix in GaussianSource.fs
    let mutable maat = SparseMatrix.init xpix ypix (fun i j -> 0.)  // initiate
    //printfn "x: %i y: %i" xPixPos yPixPos
    maat.[xPixPos, yPixPos] <- value                                // Update
    maat



let sensorUpdate(sensor:Sensor,pinter:Point3D,rayforward:RayForward) =
    //Update the sensor if a ray arrives to the sensor (forward funtion)

    let IntersectionAtSensor = FindSensorInter(sensor,pinter)
    let pixs = sensor.pixsize
    //printfn "xvals: %+A %f \n " (IntersectionAtSensor) pixs// IntersectionAtSensor.Y ymax
    let (xPixPos, yPixPos) = ( int(IntersectionAtSensor.X/pixs) , int(IntersectionAtSensor.Y/pixs) )

    let Red = updateMatrix(sensor.xpix,sensor.ypix,xPixPos, yPixPos, (rayforward.color.r*rayforward.intensity))
    let Green = updateMatrix(sensor.xpix,sensor.ypix,xPixPos, yPixPos, (rayforward.color.g*rayforward.intensity))
    let Blue = updateMatrix(sensor.xpix,sensor.ypix,xPixPos, yPixPos, (rayforward.color.b*rayforward.intensity))
    
    {
    Origin = sensor.Origin;
    c1 = sensor.c1; c2 = sensor.c2;
    Normal = sensor.Normal;
    xpix = sensor.xpix;ypix = sensor.ypix;
    pixsize = sensor.pixsize;
    RotationMatrix = sensor.RotationMatrix;
    PhotosSature = sensor.PhotosSature;

    // on color returns the sum between the previous number and the current one
    Rcolor = sensor.Rcolor + Red; 
    Gcolor = sensor.Gcolor + Green;
    Bcolor = sensor.Bcolor + Blue
    }
let sensorSum (sensor:Sensor,s2:Sensor) =
    {
    Origin = sensor.Origin;
    c1 = sensor.c1; c2 = sensor.c2;
    Normal = sensor.Normal;
    xpix = sensor.xpix;ypix = sensor.ypix;
    pixsize = sensor.pixsize;
    RotationMatrix = sensor.RotationMatrix;
    PhotosSature = sensor.PhotosSature;

    // on color returns the sum between the previous number and the current one
    Rcolor = sensor.Rcolor + s2.Rcolor; 
    Gcolor = sensor.Gcolor + s2.Gcolor;
    Bcolor = sensor.Bcolor + s2.Bcolor
    }
    
let SensorToImage(sensor:Sensor,path:string) =
    // Transform the RGB of the sensor into a true RGB for a BMP
    let bmp = new Bitmap(sensor.xpix,sensor.ypix)

    for i in 0..(sensor.xpix-1) do
        for j in 0..(sensor.ypix-1) do
            bmp.SetPixel(i,j,Color.FromArgb(min 255 (int(255.*sensor.Rcolor.[i,j])/sensor.PhotosSature),  
                                            min 255 (int(255.*sensor.Gcolor.[i,j])/sensor.PhotosSature),
                                            min 255 (int(255.*sensor.Bcolor.[i,j])/sensor.PhotosSature) )
                                            )    
    //let path = @"C:\Users\JoseM\Desktop\ForwardTest.jpg"
    bmp.Save(path)
    let form = new Form(Text="Rendering test",TopMost=true)
    form.Show()

    let img = new PictureBox(Dock=DockStyle.Fill)
    form.Controls.Add(img)

    img.Image <- bmp
    printfn "end"
