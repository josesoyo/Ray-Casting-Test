#nowarn "211"
#I "packages/MathNet.Numerics.Signed/lib/net40/"
#I "packages/MathNet.Numerics.Signed.3.7.0/lib/net40/"
#I "packages/MathNet.Numerics.FSharp.Signed/lib/net40/"
#I "packages/MathNet.Numerics.FSharp.Signed.3.7.0/lib/net40/"
#I "../packages/MathNet.Numerics.Signed/lib/net40/"
#I "../packages/MathNet.Numerics.Signed.3.7.0/lib/net40/"
#I "../packages/MathNet.Numerics.FSharp.Signed/lib/net40/"
#I "../packages/MathNet.Numerics.FSharp.Signed.3.7.0/lib/net40/"
#I "../../packages/MathNet.Numerics.Signed/lib/net40/"
#I "../../packages/MathNet.Numerics.Signed.3.7.0/lib/net40/"
#I "../../packages/MathNet.Numerics.FSharp.Signed/lib/net40/"
#I "../../packages/MathNet.Numerics.FSharp.Signed.3.7.0/lib/net40/"
#I "../../../packages/MathNet.Numerics.Signed/lib/net40/"
#I "../../../packages/MathNet.Numerics.Signed.3.7.0/lib/net40/"
#I "../../../packages/MathNet.Numerics.FSharp.Signed/lib/net40/"
#I "../../../packages/MathNet.Numerics.FSharp.Signed.3.7.0/lib/net40/"
#r "MathNet.Numerics.dll"
#r "MathNet.Numerics.FSharp.dll"

open MathNet.Numerics
open MathNet.Numerics.LinearAlgebra

fsi.AddPrinter(fun (matrix:Matrix<float>) -> matrix.ToString())
fsi.AddPrinter(fun (matrix:Matrix<float32>) -> matrix.ToString())
fsi.AddPrinter(fun (matrix:Matrix<complex>) -> matrix.ToString())
fsi.AddPrinter(fun (matrix:Matrix<complex32>) -> matrix.ToString())
fsi.AddPrinter(fun (vector:Vector<float>) -> vector.ToString())
fsi.AddPrinter(fun (vector:Vector<float32>) -> vector.ToString())
fsi.AddPrinter(fun (vector:Vector<complex>) -> vector.ToString())
fsi.AddPrinter(fun (vector:Vector<complex32>) -> vector.ToString())