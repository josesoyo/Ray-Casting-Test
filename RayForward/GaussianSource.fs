module GaussianSource

open System.Numerics
open System.IO
open System.Windows.Forms
open System.Drawing
open MathNet.Numerics.LinearAlgebra
open MathNet.Spatial.Euclidean // requieres System.Xml

open RayType
open RandomMethods
open SensorModule
//This module is going to be used to modelize all the Gaussian Source type and functions
//
//  1 - Type GaussianRay
//  2 - Create the GaussianRay with the basic things required: 
//          waist size and position 




//Ray Types
//type RayFrom = {uvec:UnitVector3D; length: float; from:Point3D; travelled:float} //length is only for light intersection not infinity
//type RayForward = {uvec:UnitVector3D; mlength: float; from:Point3D; travelled:float; color:Color; intensity:float}
type RayGaussian = {uvec:UnitVector3D; mlength: float; from:Point3D; 
                    travelled:float;        // Works as Phase - to compute it
                    intensity:int ;         // Works as a number of photons
                    wl:float                // To compute the phase from dist
                    }

let FromRayGaussianToRayForward(gau:RayGaussian) =
    // Generate a RayForward with the color a dummy value
    {uvec=gau.uvec; mlength= gau.mlength; from=gau.from;
    travelled=gau.travelled;
    color = Color(1.,1.,1.);
    intensity = float(gau.intensity)
    }


// RayFrom + Intensity = intefer because is the number of photons
[<Measure>] type m

let rotateSource (normal:UnitVector3D) =
    //Generate the rotation matrix initially
    // To save computational time
    Matrix3D.RotationTo(UnitVector3D(0.,0.,1.),normal)

let generatePsRandGausianRay (w0:float ,                         // Waist  Radius size
                              distz0:float,                      // Dist. from Waist
                              origin:Point3D,                    // Center of the gausian beam
                              normal:UnitVector3D,               // Direction of the beam
                              rotationMatrix:Matrix<float>,       // Rotate from ez to normal 
                              wl:float) =
    // To generate a pseudo random gausian ray it is required two parameters:
    // waist size and distance from source to waist and the direction of the source (Transformation matrix)
    //
    let w = w0                            // To be improved
    let Rz = distz0                       // Paraxial aproximation
    //
    let randomPos = Samp2DGauss ( w,0.0)         // random [x; y] pos - Samp2DGauss(sigma,mu)
    let lambda = wl//<m>
    let fphase = (randomPos.[0]*randomPos.[0]+randomPos.[1]*randomPos.[1])/(2.*Rz)
    //let phaseTrav = phase%lambda                  //mod
    let RotationMatrix=rotationMatrix             // Rotation matrix from 0,0,1 to normal
    let PosRotated = Point3D(randomPos.[0],randomPos.[1], 0.0).TransformBy(m=RotationMatrix)
    let RayOrigin = Point3D(PosRotated.X+origin.X,PosRotated.Y+origin.Y,PosRotated.Z+origin.Z)
    let GaussDir = 
        if Rz <> infinity then
            Vector3D(randomPos.[0],randomPos.[1],Rz).TransformBy(m=RotationMatrix).Normalize()
        else // if the radius is infinity, the wave is plane
           Vector3D(0.,0.,1.).TransformBy(m=RotationMatrix).Normalize() 

    {uvec=GaussDir; mlength=infinity; from=RayOrigin; 
    travelled = (fphase%lambda);
    intensity = 1;
    wl = lambda
    }

let generatePsRandGausianRayY (w0:float ,                         // Waist  Radius size
                               distz0:float,                      // Dist. from Waist
                               origin:Point3D,                    // Center of the gausian beam
                               normal:UnitVector3D,               // Direction of the beam
                               rotationMatrix:Matrix<float>,       // Rotate from ez to normal 
                               wl:float) =
    //  In Y axis only
    //
    // To generate a pseudo random gausian ray it is required two parameters:
    // waist size and distance from source to waist and the direction of the source (Transformation matrix)
    //
    let w = w0                            // To be improved
    let Rz = distz0                       // Paraxial aproximation
    //
    let randomP = Samp2DGauss ( w,0.0)         // random [x; y] pos - Samp2DGauss(sigma,mu)
    //let pos = sqrt(randomP.[0]*randomP.[0]+randomP.[1]*randomP.[1])
    let randomPos = [0.; randomP.[0]]
    let lambda = wl//<m>
    let fphase = (randomPos.[0]*randomPos.[0]+randomPos.[1]*randomPos.[1])/(2.*Rz)
    //let phaseTrav = phase%lambda                  //mod
    let RotationMatrix=rotationMatrix             // Rotation matrix from 0,0,1 to normal
    let PosRotated = Point3D(randomPos.[0],randomPos.[1], 0.0).TransformBy(m=RotationMatrix)
    let RayOrigin = Point3D(PosRotated.X+origin.X,abs(PosRotated.Y+origin.Y),PosRotated.Z+origin.Z)
    //printfn "orig: %+A rand %+A" RayOrigin randomPos
    let GaussDir = 
        if Rz <> infinity then
            let first = Vector3D(randomPos.[0], randomPos.[1],Rz).TransformBy(m=RotationMatrix)
            UnitVector3D(first.X,abs first.Y,first.Z)
        else // if the radius is infinity, the wave is plane
           Vector3D(0.,0.,1.).TransformBy(m=RotationMatrix).Normalize() 

    {uvec=GaussDir; mlength=infinity; from=RayOrigin; 
    travelled = (fphase%lambda);
    intensity = 1;
    wl = lambda
    }

//              //////////////////
//
// Do some test on complex values
//
//              /////////////////

let ComplexPolar (rad:float, phase:float) =
    // Creates the complex value a+ib from a polar form
    let a = rad*cos(phase)
    let b = rad* sin(phase)
    Complex(a,b)

type ComplexMonoSensor = {//This should be the type
               Origin:Point3D;          // Corner of the sensor
               c1: Point3D
               c2: Point3D              //the points to form a mesh
               Normal:UnitVector3D;     // Direction of the sensor = Dir of camera
               xpix :int; ypix: int     // (1,0,0) y (0,1,0) -> Rotated
               pixsize:float            // Suposed rectangular
               //Up not required since it's up +Y in local coordinates - CAREFULL
               RotationMatrix:Matrix<float> // From local coordinates to global coordinates
               InvRotationMatrix: Matrix<float>;
               //Real sensor properties
               PhotosSature: int // Number of photons that saturate the image
               RSensor:Matrix<float> // Saves real part of photons received
               ISensor:Matrix<float> // Saves imaginary part of the photons received
               }
type ComplexSensor = {//This should be the type
               Origin:Point3D;          // Corner of the sensor
               c1: Point3D
               c2: Point3D              //the points to form a mesh
               Normal:UnitVector3D;     // Direction of the sensor = Dir of camera
               xpix :int; ypix: int     // (1,0,0) y (0,1,0) -> Rotated
               pixsize:float            // Suposed rectangular
               //Up not required since it's up +Y in local coordinates - CAREFULL
               RotationMatrix:Matrix<float> // From local coordinates to global coordinates
               InvRotationMatrix: Matrix<float>;
               //Real sensor properties
               PhotosSature: int // Number of photons that saturate the image
               Sensor:Matrix<Complex> // Saves real part of photons received
               }


let InitiateComplexMonoSensor (origin:Point3D, normal, xpix, ypix, pixsize, photosSature) =
    // Initiate the Complexsensor 
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

    let Svalue = SparseMatrix.init xpix ypix (fun i j -> 0.)       // To initialize sparse matrix- GOOD??
    
    // Return the initiated sensor
    {Origin = origin;                       // Original Corner
    c1= c1 ; c2 = c2;                       // Corners
    Normal = normal;
    xpix = xpix ; ypix = ypix; pixsize = pixsize;
    RotationMatrix = RotationMatrix;        // from z to sensor direction
    InvRotationMatrix =  RotationMatrix.Inverse();
    PhotosSature = photosSature;
    RSensor = Svalue;                       // Real part of the sensor
    ISensor = Svalue                        // Im part of the sensor
    }

// Bug on complex matrix
//let Csmat2 = SparseMatrix.init 2 2 (fun i j -> Complex(2.,2.))       // To initialize - GOOD??
//let Rsmat = SparseMatrix.init 2 2 (fun i j -> 3.*float(i+j))       // To initialize - GOOD??
//Rsmat.[0,0] <- -5.
let FindComplexMonoSensorInter(sensor:ComplexMonoSensor,pinter:Point3D) =
    // Find the point of intersection in sensor coordinates normal=(0.,0.,1.)
    //
    //Gives an impossible point in case it's not at z=0
    let RotationMatrix= sensor.RotationMatrix//
    let RFromWorldToSensor = sensor.InvRotationMatrix//RotationMatrix.Inverse()            // Rotates from the coordinates of the world to the sensor
    let TFromWorldToSensor = sensor.Origin.ToVector3D().Negate()   // Translates from world coordinates to sensor coordinates (0,0,0) = Origin
    
    // 1st Translate and 2nd Rotate (the order is important)
    let SensorCoordinatesIntersection = (pinter+TFromWorldToSensor).TransformBy(m=RFromWorldToSensor) // Should be a Point3D

    if SensorCoordinatesIntersection.Z < 1e-10 && SensorCoordinatesIntersection.Z > -1e-10 then SensorCoordinatesIntersection
    else Point3D(infinity,infinity,SensorCoordinatesIntersection.Z)//__RaiseError__  -> sure this will raise it... (to check)
        


let ComplexMonosensorUpdate(sensor:ComplexMonoSensor,pinter:Point3D,rayforward:RayForward, wl:float) =
    //Update the sensor if a ray arrives to the sensor (forward funtion)

    let IntersectionAtSensor = FindComplexMonoSensorInter(sensor,pinter)
    let pixs = sensor.pixsize
    //printfn "xvals: %+A %f \n " (IntersectionAtSensor) pixs// IntersectionAtSensor.Y ymax
    let (xPixPos, yPixPos) = ( int(IntersectionAtSensor.X/pixs) , int(IntersectionAtSensor.Y/pixs) )
    let phase = ((rayforward.travelled/wl)*2.*PI)%(2.*PI)

    printfn "phase: %f \t travelled: %f and point %+A "phase rayforward.travelled pinter
    let ComplexVal = ComplexPolar(rayforward.intensity, phase)
    let RMat = updateMatrix(sensor.xpix,sensor.ypix,xPixPos, yPixPos, ComplexVal.Real)
    let IMat = updateMatrix(sensor.xpix,sensor.ypix,xPixPos, yPixPos, ComplexVal.Imaginary) // Original
    //let IMat = updateMatrix(sensor.xpix,sensor.ypix,xPixPos, yPixPos, phase)
    
    {
    Origin = sensor.Origin;
    c1 = sensor.c1; c2 = sensor.c2;
    Normal = sensor.Normal;
    xpix = sensor.xpix;ypix = sensor.ypix;
    pixsize = sensor.pixsize;
    RotationMatrix = sensor.RotationMatrix;
    InvRotationMatrix = sensor.InvRotationMatrix;
    PhotosSature = sensor.PhotosSature;

    // on color returns the sum between the previous number and the current one
    RSensor = sensor.RSensor + RMat; 
    ISensor = sensor.ISensor + IMat;
    }


let SensorModulusSquare(sensor:ComplexMonoSensor) =
    let re = sensor.RSensor
    let im = sensor.ISensor
    let resq = re.Map(fun i -> i*i)
    let imsq = im.Map(fun i -> i*i)
    //let resq = re*re
    //let imsq = im*im
    resq+imsq

let sensorCSSum (sensor:ComplexMonoSensor,s2:ComplexMonoSensor) =
    {
    Origin = sensor.Origin;
    c1 = sensor.c1; c2 = sensor.c2;
    Normal = sensor.Normal;
    xpix = sensor.xpix;ypix = sensor.ypix;
    pixsize = sensor.pixsize;
    RotationMatrix = sensor.RotationMatrix;
    InvRotationMatrix = sensor.InvRotationMatrix;
    PhotosSature = sensor.PhotosSature;

    // on color returns the sum between the previous number and the current one
    RSensor = sensor.RSensor + s2.RSensor; 
    ISensor = sensor.ISensor + s2.ISensor
    }

let matrixmax(matrix: Matrix<float>) =
    // Maximum value of a matrix
    let mutable a = -infinity
    for i in 0..(matrix.RowCount-1) do
        for j in 0..(matrix.ColumnCount-1) do
            if matrix.[i,j] > a then a <- matrix.[i,j]
            else a <- a
    a
let matrixmin(matrix: Matrix<float>) =
    // Maximum value of a matrix
    let mutable a = infinity
    for i in 0..(matrix.RowCount-1) do
        for j in 0..(matrix.ColumnCount-1) do
            if matrix.[i,j] < a then a <- matrix.[i,j]
            else a <- a
    a
//matrixmax(Rsmat)

let SensorMonoCromToImage(sensor:ComplexMonoSensor,path:string) =
    // Transform the RGB of the sensor into a true RGB for a BMP
    let bmp = new Bitmap(sensor.xpix,sensor.ypix)
    let monocolor = SensorModulusSquare(sensor)
    let maxmono = 0.9*matrixmax(monocolor)
    printfn "%f" maxmono
    for i in 0..(sensor.xpix-1) do
        for j in 0..(sensor.ypix-1) do
            bmp.SetPixel(i,j,Color.FromArgb(min 255 (abs(int(255.*monocolor.[i,j]/maxmono))),  
                                            min 255 (abs(int(255.*monocolor.[i,j]/maxmono))),
                                            min 255 (abs(int(255.*monocolor.[i,j]/maxmono))) )
                                            )    
    //let path = @"C:\Users\JoseM\Desktop\ForwardTest.jpg"
    bmp.Save(path)
    let form = new Form(Text="Rendering test",TopMost=true)
    form.Show()

    let img = new PictureBox(Dock=DockStyle.Fill)
    form.Controls.Add(img)

    img.Image <- bmp
    printfn "end"


//
//      Pure Complex
//

let InitiateComplexSensor (origin:Point3D, normal, xpix, ypix, pixsize, photosSature) =
    // Initiate the Complexsensor 
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

    let Svalue = SparseMatrix.init xpix ypix (fun i j -> Complex(0.,0.))       // To initialize sparse matrix- GOOD??
    
    // Return the initiated sensor
    {Origin = origin;                       // Original Corner
    c1= c1 ; c2 = c2;                       // Corners
    Normal = normal;
    xpix = xpix ; ypix = ypix; pixsize = pixsize;
    RotationMatrix = RotationMatrix;        // from z to sensor direction
    InvRotationMatrix =  RotationMatrix.Inverse();
    PhotosSature = photosSature;
    Sensor = Svalue;                       // Real part of the sensor
                      // Im part of the sensor
    }


let updateComplexMatrix(xpix:int,ypix:int,xPixPos, yPixPos,value:Complex)=
    //Update a sparse matrix that works as the sensor
    let mutable maat = SparseMatrix.init xpix ypix (fun i j -> Complex(0.,0.))  // initiate
    printfn "x: %i y: %i" xPixPos yPixPos
    maat.[xPixPos, yPixPos] <- value                                // Update
    // Updates on edge pixels
    if xPixPos <> (xpix-1) then
        maat.[xPixPos+1, yPixPos] <- value
    if xPixPos <> 0 then
        maat.[xPixPos-1, yPixPos] <- value
    if yPixPos <> (ypix-1) then
        maat.[xPixPos, yPixPos+1] <- value 
    if yPixPos <> 0 then
        maat.[xPixPos, yPixPos-1] <- value 
    // Double
    if xPixPos <> (xpix-1) &&  yPixPos <> (ypix-1) then
        maat.[xPixPos+1, yPixPos+1] <- value
    if xPixPos <> (xpix-1) &&  yPixPos <> 0 then
        maat.[xPixPos+1, yPixPos-1] <- value
    if xPixPos <> 0 &&  yPixPos <> (ypix-1) then
        maat.[xPixPos-1, yPixPos+1] <- value
    if xPixPos <> 0 &&  yPixPos <> 0 then
        maat.[xPixPos-1, yPixPos-1] <- value

    maat

let FindComplexSensorInter(sensor:ComplexSensor,pinter:Point3D) =
    // Repeated function with a different input
    // Find the point of intersection in sensor coordinates normal=(0.,0.,1.)
    //
    //Gives an impossible point in case it's not at z=0
    let RotationMatrix= sensor.RotationMatrix//
    let RFromWorldToSensor = sensor.InvRotationMatrix//RotationMatrix.Inverse()            // Rotates from the coordinates of the world to the sensor
    let TFromWorldToSensor = sensor.Origin.ToVector3D().Negate()   // Translates from world coordinates to sensor coordinates (0,0,0) = Origin
    
    // 1st Translate and 2nd Rotate (the order is important)
    let SensorCoordinatesIntersection = (pinter+TFromWorldToSensor).TransformBy(m=RFromWorldToSensor) // Should be a Point3D

    if SensorCoordinatesIntersection.Z < 1e-10 && SensorCoordinatesIntersection.Z > -1e-10 then SensorCoordinatesIntersection
    else Point3D(infinity,infinity,SensorCoordinatesIntersection.Z)//__RaiseError__  -> sure this will raise it... (to check)
        


let ComplexsensorUpdate(sensor:ComplexSensor,pinter:Point3D,rayforward:RayForward, wl:float) =
    //Update the sensor if a ray arrives to the sensor (forward funtion)

    let IntersectionAtSensor = FindComplexSensorInter(sensor,pinter)
    let pixs = sensor.pixsize
    //printfn "xvals: %+A %f \n " (IntersectionAtSensor) pixs// IntersectionAtSensor.Y ymax
    let (xPixPos, yPixPos) = ( int(IntersectionAtSensor.X/pixs) , int(IntersectionAtSensor.Y/pixs) )
    let phase = ((rayforward.travelled/wl)*2.*PI)%(2.*PI)

    printfn "phase: %f \t travelled: %f and point %+A "phase rayforward.travelled pinter
    let ComplexVal = ComplexPolar(rayforward.intensity, phase)
    let Sval = updateComplexMatrix(sensor.xpix,sensor.ypix,xPixPos, yPixPos, ComplexVal)
    
    {
    Origin = sensor.Origin;
    c1 = sensor.c1; c2 = sensor.c2;
    Normal = sensor.Normal;
    xpix = sensor.xpix;ypix = sensor.ypix;
    pixsize = sensor.pixsize;
    RotationMatrix = sensor.RotationMatrix;
    InvRotationMatrix = sensor.InvRotationMatrix;
    PhotosSature = sensor.PhotosSature;

    // on color returns the sum between the previous number and the current one
    Sensor = sensor.Sensor + Sval; 
    }


let SensorComplexModulusSquare(sensor:ComplexSensor) =
    let value = sensor.Sensor
    let resq = value.Map(fun i -> i.Magnitude*i.Magnitude)
    resq
let SensorComplexPhase(sensor:ComplexSensor) =
    let value = sensor.Sensor
    let resq = value.Map(fun i -> i.Phase)
    resq


let sensorCSum (sensor:ComplexSensor,s2:ComplexSensor) =
    {
    Origin = sensor.Origin;
    c1 = sensor.c1; c2 = sensor.c2;
    Normal = sensor.Normal;
    xpix = sensor.xpix;ypix = sensor.ypix;
    pixsize = sensor.pixsize;
    RotationMatrix = sensor.RotationMatrix;
    InvRotationMatrix = sensor.InvRotationMatrix;
    PhotosSature = sensor.PhotosSature;

    // on color returns the sum between the previous number and the current one
    Sensor = sensor.Sensor + s2.Sensor; 
    }
let SensorCromToImage(sensor:ComplexSensor,path:string) =
    // Transform the RGB of the sensor into a true RGB for a BMP
    let bmp = new Bitmap(sensor.xpix,sensor.ypix)
    let monocolor = SensorComplexModulusSquare(sensor)
    let maxmono = matrixmax(monocolor)
    printfn "%f" maxmono
    for i in 0..(sensor.xpix-1) do
        for j in 0..(sensor.ypix-1) do
            bmp.SetPixel(i,j,Color.FromArgb(min 255 (abs(int(255.*monocolor.[i,j]/maxmono))),  
                                            min 255 (abs(int(255.*monocolor.[i,j]/maxmono))),
                                            min 255 (abs(int(255.*monocolor.[i,j]/maxmono))) )
                                            )    
    //let path = @"C:\Users\JoseM\Desktop\ForwardTest.jpg"
    bmp.Save(path)
    let form = new Form(Text="Rendering test",TopMost=true)
    form.Show()

    let img = new PictureBox(Dock=DockStyle.Fill)
    form.Controls.Add(img)

    img.Image <- bmp
    printfn "end"
