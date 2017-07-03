using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Geom3D
{
    public static class Util
    {
        public static Matrix RotateAroundLine(Vector3 p1, Vector3 p2, double angle)
        {
            Matrix translateToOrigin = Matrix.CreateTranslation(-p1);
            Matrix rotateAroundAxis = Matrix.CreateFromAxisAngle(Util.Normalize(p2 - p1), angle);
            Matrix translateBack = Matrix.CreateTranslation(p1);

            return translateToOrigin * rotateAroundAxis * translateBack;
        }

        public static IEnumerable<Vector3> Apply(Matrix m, IEnumerable<Vector3> vec)
        {
            return vec.Select(v => Vector3.Transform(v, m));
        }

        public static Vector3 Normalize(Vector3 vec)
        {
            var norm = vec;
            norm.Normalize();
            return norm;
        }

        public static Vector3 GetFaceNormal(List<Vector3> face)
        {
            Vector3 dir1 = face[1] - face[0];
            Vector3 dir2 = face[2] - face[0];
            return Util.Normalize(Vector3.Cross(dir1, dir2));
        }

        public static Vector3 GetFaceCentroid(List<Vector3> face)
        {
            Vector3 sum = face.Aggregate((v1, v2) => v1 + v2);
            return sum / (double)face.Count;
        }

        public static Vector3 Lerp(Vector3 p1, Vector3 p2, double t)
        {
            var offset = (p2 - p1) * t;
            return p1 + offset;
        }

        public static bool LineLineIntersect(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, double epsilon, ref Vector3 pa, ref Vector3 pb, ref double mua, ref double mub)
        {
            Vector3 p13 = new Vector3(), p43 = new Vector3(), p21 = new Vector3();
            double d1343, d4321, d1321, d4343, d2121;
            double numer, denom;

            p13.X = p1.X - p3.X;
            p13.Y = p1.Y - p3.Y;
            p13.Z = p1.Z - p3.Z;
            p43.X = p4.X - p3.X;
            p43.Y = p4.Y - p3.Y;
            p43.Z = p4.Z - p3.Z;
            if (Math.Abs(p43.X) < epsilon && Math.Abs(p43.Y) < epsilon && Math.Abs(p43.Z) < epsilon)
                return false;

            p21.X = p2.X - p1.X;
            p21.Y = p2.Y - p1.Y;
            p21.Z = p2.Z - p1.Z;
            if (Math.Abs(p21.X) < epsilon && Math.Abs(p21.Y) < epsilon && Math.Abs(p21.Z) < epsilon)
                return false;

            d1343 = p13.X * p43.X + p13.Y * p43.Y + p13.Z * p43.Z;
            d4321 = p43.X * p21.X + p43.Y * p21.Y + p43.Z * p21.Z;
            d1321 = p13.X * p21.X + p13.Y * p21.Y + p13.Z * p21.Z;
            d4343 = p43.X * p43.X + p43.Y * p43.Y + p43.Z * p43.Z;
            d2121 = p21.X * p21.X + p21.Y * p21.Y + p21.Z * p21.Z;

            denom = d2121 * d4343 - d4321 * d4321;
            if (Math.Abs(denom) < epsilon)
                return false;
            numer = d1343 * d4321 - d1321 * d4343;

            mua = numer / denom;
            mub = (d1343 + d4321 * (mua)) / d4343;

            pa.X = p1.X + mua * p21.X;
            pa.Y = p1.Y + mua * p21.Y;
            pa.Z = p1.Z + mua * p21.Z;
            pb.X = p3.X + mub * p43.X;
            pb.Y = p3.Y + mub * p43.Y;
            pb.Z = p3.Z + mub * p43.Z;

            return true;
        }

        public static double RayTriangleIntersection(Vector3 orig, Vector3 dir, Vector3 v0, Vector3 v1, Vector3 v2)
        {
            Vector3 e1 = v1 - v0;
            Vector3 e2 = v2 - v0;
            // Calculate planes normal vector
            Vector3 pvec = Vector3.Cross(dir, e2);
            double det = Vector3.Dot(e1, pvec);

            // Ray is parallel to plane
            if (det < 1e-8 && det > -1e-8)
            {
                return 0;
            }

            double inv_det = 1 / det;
            Vector3 tvec = orig - v0;
            double u = Vector3.Dot(tvec, pvec) * inv_det;

            if (u < 0 || u > 1)
            {
                return 0;
            }

            Vector3 qvec = Vector3.Cross(tvec, e1);
            double v = Vector3.Dot(dir, qvec) * inv_det;

            if (v < 0 || u + v > 1)
            {
                return 0;
            }

            return Vector3.Dot(e2, qvec) * inv_det;
        }

        static private int coplanar_tri_tri(double[] N, double[] V0, double[] V1, double[] V2, double[] U0, double[] U1, double[] U2)
        {
            double[] A = new double[3];
            short i0, i1;

            A[0] = ((double)(Math.Abs(N[0])));
            A[1] = ((double)(Math.Abs(N[1])));
            A[2] = ((double)(Math.Abs(N[2])));
            if (A[0] > A[1])
            {
                if (A[0] > A[2])
                {
                    i0 = 1;
                    i1 = 2;
                }
                else
                {
                    i0 = 0;
                    i1 = 1;
                }
            }
            else
            {
                if (A[2] > A[1])
                {
                    i0 = 0;
                    i1 = 1;
                }
                else
                {
                    i0 = 0;
                    i1 = 2;
                }
            }

            { double Ax, Ay, Bx, By, Cx, Cy, e, d, f; Ax = V1[i0] - V0[i0]; Ay = V1[i1] - V0[i1]; Bx = U0[i0] - U1[i0]; By = U0[i1] - U1[i1]; Cx = V0[i0] - U0[i0]; Cy = V0[i1] - U0[i1]; f = Ay * Bx - Ax * By; d = By * Cx - Bx * Cy; if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) { e = Ax * Cy - Ay * Cx; if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; } }; Bx = U1[i0] - U2[i0]; By = U1[i1] - U2[i1]; Cx = V0[i0] - U1[i0]; Cy = V0[i1] - U1[i1]; f = Ay * Bx - Ax * By; d = By * Cx - Bx * Cy; if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) { e = Ax * Cy - Ay * Cx; if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; } }; Bx = U2[i0] - U0[i0]; By = U2[i1] - U0[i1]; Cx = V0[i0] - U2[i0]; Cy = V0[i1] - U2[i1]; f = Ay * Bx - Ax * By; d = By * Cx - Bx * Cy; if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) { e = Ax * Cy - Ay * Cx; if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; } }; };
            { double Ax, Ay, Bx, By, Cx, Cy, e, d, f; Ax = V2[i0] - V1[i0]; Ay = V2[i1] - V1[i1]; Bx = U0[i0] - U1[i0]; By = U0[i1] - U1[i1]; Cx = V1[i0] - U0[i0]; Cy = V1[i1] - U0[i1]; f = Ay * Bx - Ax * By; d = By * Cx - Bx * Cy; if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) { e = Ax * Cy - Ay * Cx; if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; } }; Bx = U1[i0] - U2[i0]; By = U1[i1] - U2[i1]; Cx = V1[i0] - U1[i0]; Cy = V1[i1] - U1[i1]; f = Ay * Bx - Ax * By; d = By * Cx - Bx * Cy; if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) { e = Ax * Cy - Ay * Cx; if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; } }; Bx = U2[i0] - U0[i0]; By = U2[i1] - U0[i1]; Cx = V1[i0] - U2[i0]; Cy = V1[i1] - U2[i1]; f = Ay * Bx - Ax * By; d = By * Cx - Bx * Cy; if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) { e = Ax * Cy - Ay * Cx; if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; } }; };
            { double Ax, Ay, Bx, By, Cx, Cy, e, d, f; Ax = V0[i0] - V2[i0]; Ay = V0[i1] - V2[i1]; Bx = U0[i0] - U1[i0]; By = U0[i1] - U1[i1]; Cx = V2[i0] - U0[i0]; Cy = V2[i1] - U0[i1]; f = Ay * Bx - Ax * By; d = By * Cx - Bx * Cy; if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) { e = Ax * Cy - Ay * Cx; if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; } }; Bx = U1[i0] - U2[i0]; By = U1[i1] - U2[i1]; Cx = V2[i0] - U1[i0]; Cy = V2[i1] - U1[i1]; f = Ay * Bx - Ax * By; d = By * Cx - Bx * Cy; if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) { e = Ax * Cy - Ay * Cx; if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; } }; Bx = U2[i0] - U0[i0]; By = U2[i1] - U0[i1]; Cx = V2[i0] - U2[i0]; Cy = V2[i1] - U2[i1]; f = Ay * Bx - Ax * By; d = By * Cx - Bx * Cy; if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) { e = Ax * Cy - Ay * Cx; if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; } }; };

            { double a, b, c, d0, d1, d2; a = U1[i1] - U0[i1]; b = -(U1[i0] - U0[i0]); c = -a * U0[i0] - b * U0[i1]; d0 = a * V0[i0] + b * V0[i1] + c; a = U2[i1] - U1[i1]; b = -(U2[i0] - U1[i0]); c = -a * U1[i0] - b * U1[i1]; d1 = a * V0[i0] + b * V0[i1] + c; a = U0[i1] - U2[i1]; b = -(U0[i0] - U2[i0]); c = -a * U2[i0] - b * U2[i1]; d2 = a * V0[i0] + b * V0[i1] + c; if (d0 * d1 > 0.0) { if (d0 * d2 > 0.0) return 1; } };
            { double a, b, c, d0, d1, d2; a = V1[i1] - V0[i1]; b = -(V1[i0] - V0[i0]); c = -a * V0[i0] - b * V0[i1]; d0 = a * U0[i0] + b * U0[i1] + c; a = V2[i1] - V1[i1]; b = -(V2[i0] - V1[i0]); c = -a * V1[i0] - b * V1[i1]; d1 = a * U0[i0] + b * U0[i1] + c; a = V0[i1] - V2[i1]; b = -(V0[i0] - V2[i0]); c = -a * V2[i0] - b * V2[i1]; d2 = a * U0[i0] + b * U0[i1] + c; if (d0 * d1 > 0.0) { if (d0 * d2 > 0.0) return 1; } };

            return 0;
        }

        static private int GetTriangleTriangleIntersection(double[] V0, double[] V1, double[] V2, double[] U0, double[] U1, double[] U2, double eps)
        {
            double[] E1 = new double[3], E2 = new double[3];
            double[] N1 = new double[3], N2 = new double[3];
            double du0, du1, du2, dv0, dv1, dv2, d1, d2;
            double[] D = new double[3];
            double[] isect1 = new double[2], isect2 = new double[2];
            double du0du1, du0du2, dv0dv1, dv0dv2;
            short index;
            double vp0, vp1, vp2;
            double up0, up1, up2;
            double bb, cc, max;

            { E1[0] = V1[0] - V0[0]; E1[1] = V1[1] - V0[1]; E1[2] = V1[2] - V0[2]; };
            { E2[0] = V2[0] - V0[0]; E2[1] = V2[1] - V0[1]; E2[2] = V2[2] - V0[2]; };
            { N1[0] = E1[1] * E2[2] - E1[2] * E2[1]; N1[1] = E1[2] * E2[0] - E1[0] * E2[2]; N1[2] = E1[0] * E2[1] - E1[1] * E2[0]; };
            d1 = -(N1[0] * V0[0] + N1[1] * V0[1] + N1[2] * V0[2]);

            du0 = (N1[0] * U0[0] + N1[1] * U0[1] + N1[2] * U0[2]) + d1;
            du1 = (N1[0] * U1[0] + N1[1] * U1[1] + N1[2] * U1[2]) + d1;
            du2 = (N1[0] * U2[0] + N1[1] * U2[1] + N1[2] * U2[2]) + d1;

            if (((double)(Math.Abs(du0))) < eps) du0 = 0.0;
            if (((double)(Math.Abs(du1))) < eps) du1 = 0.0;
            if (((double)(Math.Abs(du2))) < eps) du2 = 0.0;
            du0du1 = du0 * du1;
            du0du2 = du0 * du2;

            if (du0du1 > 0.0 && du0du2 > 0.0)
                return 0;

            { E1[0] = U1[0] - U0[0]; E1[1] = U1[1] - U0[1]; E1[2] = U1[2] - U0[2]; };
            { E2[0] = U2[0] - U0[0]; E2[1] = U2[1] - U0[1]; E2[2] = U2[2] - U0[2]; };
            { N2[0] = E1[1] * E2[2] - E1[2] * E2[1]; N2[1] = E1[2] * E2[0] - E1[0] * E2[2]; N2[2] = E1[0] * E2[1] - E1[1] * E2[0]; };
            d2 = -(N2[0] * U0[0] + N2[1] * U0[1] + N2[2] * U0[2]);

            dv0 = (N2[0] * V0[0] + N2[1] * V0[1] + N2[2] * V0[2]) + d2;
            dv1 = (N2[0] * V1[0] + N2[1] * V1[1] + N2[2] * V1[2]) + d2;
            dv2 = (N2[0] * V2[0] + N2[1] * V2[1] + N2[2] * V2[2]) + d2;

            if (((double)(Math.Abs(dv0))) < eps) dv0 = 0.0;
            if (((double)(Math.Abs(dv1))) < eps) dv1 = 0.0;
            if (((double)(Math.Abs(dv2))) < eps) dv2 = 0.0;

            dv0dv1 = dv0 * dv1;
            dv0dv2 = dv0 * dv2;

            if (dv0dv1 > 0.0 && dv0dv2 > 0.0)
                return 0;

            { D[0] = N1[1] * N2[2] - N1[2] * N2[1]; D[1] = N1[2] * N2[0] - N1[0] * N2[2]; D[2] = N1[0] * N2[1] - N1[1] * N2[0]; };

            max = (double)((double)(Math.Abs(D[0])));
            index = 0;
            bb = (double)((double)(Math.Abs(D[1])));
            cc = (double)((double)(Math.Abs(D[2])));
            if (bb > max)
            {
                max = bb;
                index = 1;
            }
            if (cc > max)
            {
                max = cc;
                index = 2;
            }

            vp0 = V0[index];
            vp1 = V1[index];
            vp2 = V2[index];

            up0 = U0[index];
            up1 = U1[index];
            up2 = U2[index];

            double a, b, c, x0, x1;
            { if (dv0dv1 > 0.0) { a = vp2; b = (vp0 - vp2) * dv2; c = (vp1 - vp2) * dv2; x0 = dv2 - dv0; x1 = dv2 - dv1; } else if (dv0dv2 > 0.0) { a = vp1; b = (vp0 - vp1) * dv1; c = (vp2 - vp1) * dv1; x0 = dv1 - dv0; x1 = dv1 - dv2; } else if (dv1 * dv2 > 0.0 || dv0 != 0.0) { a = vp0; b = (vp1 - vp0) * dv0; c = (vp2 - vp0) * dv0; x0 = dv0 - dv1; x1 = dv0 - dv2; } else if (dv1 != 0.0) { a = vp1; b = (vp0 - vp1) * dv1; c = (vp2 - vp1) * dv1; x0 = dv1 - dv0; x1 = dv1 - dv2; } else if (dv2 != 0.0) { a = vp2; b = (vp0 - vp2) * dv2; c = (vp1 - vp2) * dv2; x0 = dv2 - dv0; x1 = dv2 - dv1; } else { return coplanar_tri_tri(N1, V0, V1, V2, U0, U1, U2); } };

            double d, e, f, y0, y1;
            { if (du0du1 > 0.0) { d = up2; e = (up0 - up2) * du2; f = (up1 - up2) * du2; y0 = du2 - du0; y1 = du2 - du1; } else if (du0du2 > 0.0) { d = up1; e = (up0 - up1) * du1; f = (up2 - up1) * du1; y0 = du1 - du0; y1 = du1 - du2; } else if (du1 * du2 > 0.0 || du0 != 0.0) { d = up0; e = (up1 - up0) * du0; f = (up2 - up0) * du0; y0 = du0 - du1; y1 = du0 - du2; } else if (du1 != 0.0) { d = up1; e = (up0 - up1) * du1; f = (up2 - up1) * du1; y0 = du1 - du0; y1 = du1 - du2; } else if (du2 != 0.0) { d = up2; e = (up0 - up2) * du2; f = (up1 - up2) * du2; y0 = du2 - du0; y1 = du2 - du1; } else { return coplanar_tri_tri(N1, V0, V1, V2, U0, U1, U2); } };

            double xx, yy, xxyy, tmp;
            xx = x0 * x1;
            yy = y0 * y1;
            xxyy = xx * yy;

            tmp = a * xxyy;
            isect1[0] = tmp + b * x1 * yy;
            isect1[1] = tmp + c * x0 * yy;

            tmp = d * xxyy;
            isect2[0] = tmp + e * xx * y1;
            isect2[1] = tmp + f * xx * y0;

            if (isect1[0] > isect1[1]) { double val; val = isect1[0]; isect1[0] = isect1[1]; isect1[1] = val; };
            if (isect2[0] > isect2[1]) { double val; val = isect2[0]; isect2[0] = isect2[1]; isect2[1] = val; };

            if (isect1[1] < isect2[0] || isect2[1] < isect1[0]) return 0;
            return 1;
        }

        public static bool HasTriangleTriangleIntersection(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 u0, Vector3 u1, Vector3 u2, double eps)
        {
            return GetTriangleTriangleIntersection(
                       new double[] { v0.X, v0.Y, v0.Z },
                       new double[] { v1.X, v1.Y, v1.Z },
                       new double[] { v2.X, v2.Y, v2.Z },
                       new double[] { u0.X, u0.Y, u0.Z },
                       new double[] { u1.X, u1.Y, u1.Z },
                       new double[] { u2.X, u2.Y, u2.Z },
                       eps) != 0;
        }
    }
}

