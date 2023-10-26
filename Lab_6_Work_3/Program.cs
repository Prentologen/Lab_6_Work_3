using System;

public class Quaternion
{
    public double X { get; private set; }
    public double Y { get; private set; }
    public double Z { get; private set; }
    public double W { get; private set; }

    public Quaternion(double x, double y, double z, double w)
    {
        X = x;
        Y = y;
        Z = z;
        W = w;
    }


    public static Quaternion operator +(Quaternion a, Quaternion b)
    {
        return new Quaternion(a.X + b.X, a.Y + b.Y, a.Z + b.Z, a.W + b.W);
    }

    public static Quaternion operator -(Quaternion a, Quaternion b)
    {
        return new Quaternion(a.X - b.X, a.Y - b.Y, a.Z - b.Z, a.W - b.W);
    }

    public static Quaternion operator *(Quaternion a, Quaternion b)
    {
        double x = a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y;
        double y = a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X;
        double z = a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W;
        double w = a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z;

        return new Quaternion(x, y, z, w);
    }


    public double Norm()
    {
        return Math.Sqrt(X * X + Y * Y + Z * Z + W * W);
    }


    public Quaternion Conjugate()
    {
        return new Quaternion(-X, -Y, -Z, W);
    }


    public Quaternion Inverse()
    {
        double norm = Norm();
        double normSquared = norm * norm;

        if (normSquared > 0)
        {
            Quaternion conjugate = Conjugate();
            return new Quaternion(conjugate.X / normSquared, conjugate.Y / normSquared, conjugate.Z / normSquared, conjugate.W / normSquared);
        }
        else
        {
            throw new InvalidOperationException("Cannot calculate the inverse of a zero quaternion.");
        }
    }


    public static bool operator ==(Quaternion a, Quaternion b)
    {
        return a.X == b.X && a.Y == b.Y && a.Z == b.Z && a.W == b.W;
    }

    public static bool operator !=(Quaternion a, Quaternion b)
    {
        return !(a == b);
    }


    public Matrix3x3 ToRotationMatrix()
    {
        double xx = X * X;
        double xy = X * Y;
        double xz = X * Z;
        double xw = X * W;
        double yy = Y * Y;
        double yz = Y * Z;
        double yw = Y * W;
        double zz = Z * Z;
        double zw = Z * W;

        Matrix3x3 matrix = new Matrix3x3(
            1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw),
            2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw),
            2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy)
        );

        return matrix;
    }


    public static Quaternion FromRotationMatrix(Matrix3x3 matrix)
    {
        double trace = matrix.Trace();
        if (trace > 0)
        {
            double S = Math.Sqrt(trace + 1.0) * 2;
            double invS = 1.0 / S;
            double x = (matrix[1, 2] - matrix[2, 1]) * invS;
            double y = (matrix[2, 0] - matrix[0, 2]) * invS;
            double z = (matrix[0, 1] - matrix[1, 0]) * invS;
            double w = 0.25 * S;
            return new Quaternion(x, y, z, w);
        }
        else if (matrix[0, 0] > matrix[1, 1] && matrix[0, 0] > matrix[2, 2])
        {
            double S = Math.Sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2;
            double invS = 1.0 / S;
            double x = 0.25 * S;
            double y = (matrix[0, 1] + matrix[1, 0]) * invS;
            double z = (matrix[2, 0] + matrix[0, 2]) * invS;
            double w = (matrix[1, 2] - matrix[2, 1]) * invS;
            return new Quaternion(x, y, z, w);
        }
        else if (matrix[1, 1] > matrix[2, 2])
        {
            double S = Math.Sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2;
            double invS = 1.0 / S;
            double x = (matrix[0, 1] + matrix[1, 0]) * invS;
            double y = 0.25 * S;
            double z = (matrix[1, 2] + matrix[2, 1]) * invS;
            double w = (matrix[2, 0] - matrix[0, 2]) * invS;
            return new Quaternion(x, y, z, w);
        }
        else
        {
            double S = Math.Sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2;
            double invS = 1.0 / S;
            double x = (matrix[2, 0] + matrix[0, 2]) * invS;
            double y = (matrix[1, 2] + matrix[2, 1]) * invS;
            double z = 0.25 * S;
            double w = (matrix[0, 1] - matrix[1, 0]) * invS;
            return new Quaternion(x, y, z, w);
        }
    }
}

public class Matrix3x3
{
    private double[,] data = new double[3, 3];

    public Matrix3x3(double m11, double m12, double m13, double m21, double m22, double m23, double m31, double m32, double m33)
    {
        data[0, 0] = m11;
        data[0, 1] = m12;
        data[0, 2] = m13;
        data[1, 0] = m21;
        data[1, 1] = m22;
        data[1, 2] = m23;
        data[2, 0] = m31;
        data[2, 1] = m32;
        data[2, 2] = m33;
    }

    public double this[int row, int col]
    {
        get { return data[row, col]; }
        set { data[row, col] = value; }
    }

    public double Trace()
    {
        return data[0, 0] + data[1, 1] + data[2, 2];
    }
}


var q1 = new Quaternion(1, 2, 3, 4);
Quaternion q2 = new Quaternion(5, 6, 7, 8);

Quaternion qSum = q1 + q2;  
Quaternion qDifference = q1 - q2;  
Quaternion qProduct = q1 * q2;  
double norm = q1.Norm(); 
Quaternion conjugate = q1.Conjugate();  
Quaternion inverse = q1.Inverse(); 

bool areEqual = (q1 == q2);  
bool areNotEqual = (q1 != q2);  

Matrix3x3 rotationMatrix = q1.ToRotationMatrix();  
Quaternion fromMatrix = Quaternion.FromRotationMatrix(rotationMatrix);  
