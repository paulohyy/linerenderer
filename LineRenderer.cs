using Godot;
using System;

namespace SkipTheBadEngine
{
    /// <summary>
    /// Line renderer with meshes that always faces the camera
    /// </summary>
    public class LineRenderer
    {
        Vector3 ZERO = new Vector3(0, 0, 0);

        private ImmediateGeometry imGeo;
        private ShaderMaterial mat;
        private Camera camera;
        private Vector3[] last;

        private int cornerSmooth;

        private Color startColor;
        private Color endColor;

        private float startThickness;
        private float endThickness;
        private float thickness;
        private float nextThickness;

        private float progress;
        private float progressStep;

        public LineRenderer(Node node, Camera camera, int vertexSmoothCount = 3) // TODO: pick shader, pick color and thickness for each point
        {
            var shader = ResourceLoader.Load(@"Shaders/default.shader");
            mat = new ShaderMaterial();
            mat.SetShader(shader as Shader);

            imGeo = new ImmediateGeometry();
            imGeo.SetCastShadowsSetting(GeometryInstance.ShadowCastingSetting.DoubleSided);
            imGeo.SetMaterialOverride(mat);
            node.AddChild(imGeo);

            this.camera = camera;
            this.cornerSmooth = (vertexSmoothCount / 2);

            last = new Vector3[2];

            startThickness = endThickness = 0.1f;
        }

        private void Corner(Vector3 center, Vector3 pivot)
        {
            const float PiStart = Mathf.PI * 1.5f;
            const float PiEnd = Mathf.PI * 2.5f;
            var axis = (center - camera.GlobalTransform.origin).Normalized();
            var radius = (center - pivot).Normalized() * thickness;

            var array = new Vector3[cornerSmooth + 1];

            for (int i = 0; i < cornerSmooth + 1; i++)
                array[i] = (center + radius.Rotated(axis, Lerp(PiStart, PiEnd, (float)(i) / cornerSmooth)));

            for (int i = 1; i < cornerSmooth + 1; i++)
            {
                AddVertex(array[i - 1]);
                AddVertex(array[i]);
                AddVertex(center);
            }
            
        }

        private void FillJoint(Vector3[] quad, int count)
        {
            for (float i = 0; i < count; i++)
            {
                AddVertex(quad[0]);
                AddVertex((((quad[1].Lerp(quad[2], i / count)) - quad[3]).Normalized() * nextThickness) + quad[3]);
                AddVertex((((quad[1].Lerp(quad[2], (i + 1) / count)) - quad[3]).Normalized() * nextThickness) + quad[3]);
            }
        }

        Vector3[] UP;
        Vector3[] LOW;
        private void Joint(Vector3[] triple, float i, int length, bool stop = false)
        {
            var AB = triple[1] - triple[0];
            var BC = triple[2] - triple[1];
            var orthogonalABStart = (camera.GlobalTransform.origin - ((triple[0] + triple[1]) / 2)).Cross(AB).Normalized() * thickness;
            var orthogonalABEnd = (camera.GlobalTransform.origin - ((triple[0] + triple[1]) / 2)).Cross(AB).Normalized() * nextThickness;
            var orthogonalBC = (camera.GlobalTransform.origin - ((triple[1] + triple[2]) / 2)).Cross(BC).Normalized() * nextThickness;

            UP = new Vector3[4];
            UP[LREnum.A] = last[0] != ZERO ? last[0] : triple[0] + orthogonalABStart;
            UP[LREnum.AB] = triple[1] + orthogonalABEnd;
            UP[LREnum.BC] = triple[1] + orthogonalBC;
            UP[LREnum.C] = triple[2] + orthogonalBC;

            LOW = new Vector3[4];
            LOW[LREnum.A] = last[1] != ZERO ? last[1] : triple[0] - orthogonalABStart;
            LOW[LREnum.AB] = triple[1] - orthogonalABEnd;
            LOW[LREnum.BC] = triple[1] - orthogonalBC;
            LOW[LREnum.C] = triple[2] - orthogonalBC;

            var upperCross = Intersection(UP, out Vector3 upper, out float upperScore);
            var lowerCross = Intersection(LOW, out Vector3 lower, out float lowerScore);
            upperCross = upperCross && upperScore > lowerScore;
            lowerCross = lowerCross && lowerScore > upperScore;
            if (upperCross)
            {
                AddVertex(upper);
                AddVertex(LOW[LREnum.A]);
                AddVertex(UP[LREnum.A]);
                AddVertex(LOW[LREnum.AB]);
                AddVertex(LOW[LREnum.A]);
                AddVertex(upper);
                last[0] = upper;
                last[1] = ZERO;
                FillJoint(new Vector3[] { upper, LOW[LREnum.BC], LOW[LREnum.AB], triple[1] }, 5);
            }
            else if (lowerCross)
            {
                AddVertex(UP[LREnum.AB]);
                AddVertex(LOW[LREnum.A]);
                AddVertex(UP[LREnum.A]);
                AddVertex(UP[LREnum.AB]);
                AddVertex(lower);
                AddVertex(LOW[LREnum.A]);
                last[0] = ZERO;
                last[1] = lower;
                FillJoint(new Vector3[] { lower, UP[LREnum.AB], UP[LREnum.BC], triple[1] }, 5);
            }
            else
            {
                AddVertex(UP[LREnum.AB]);
                AddVertex(LOW[LREnum.A]);
                AddVertex(UP[LREnum.A]);
                AddVertex(UP[LREnum.AB]);
                AddVertex(LOW[LREnum.AB]);
                AddVertex(LOW[LREnum.A]);
                last[0] = ZERO;
                last[1] = ZERO;
                AddVertex(UP[LREnum.AB]);
                AddVertex(UP[LREnum.BC]);
                AddVertex(LOW[LREnum.AB]);
                AddVertex(UP[LREnum.BC]);
                AddVertex(LOW[LREnum.BC]);
                AddVertex(LOW[LREnum.AB]);
            }
            i++;
            if (i == length - 1 && !stop)
                Joint(new Vector3[] { triple[1], triple[2], triple[2] + (triple[2] - triple[1]) }, i, length, true);
        }

        public void Update(Vector3[] points)
        {
            if (points.Length < 3)
                return;

            progressStep = 1.0f / points.Length;
            progress = 0;
            thickness = Lerp(startThickness, endThickness, progress);
            nextThickness = Lerp(startThickness, endThickness, progress + progressStep);

            imGeo.Clear();
            imGeo.Begin(Mesh.PrimitiveType.Triangles);
            last[0] = last[1] = ZERO;

            Corner(points[0], points[1]);
            for (int i = 1; i < points.Length - 1; i++)
            {
                Joint(new Vector3[] { points[i - 1], points[i], points[i + 1] }, i, points.Length);
                thickness = Lerp(startThickness, endThickness, progress);
                nextThickness = Lerp(startThickness, endThickness, progress + progressStep);
                progress += progressStep;
            }
            Corner(points[points.Length - 1], points[points.Length - 2]);

            imGeo.End();
        }

        public void SetColors(Color start, Color end)
        {
            this.startColor = start;
            this.endColor = end;
        }

        public void SetThickness(float startThickness, float endThickness)
        {
            this.startThickness = startThickness;
            this.endThickness = endThickness;
        }

        private void AddVertex(Vector3 vertex)
        {
            imGeo.SetColor(LerpColor(startColor, endColor, progress));
            imGeo.AddVertex(vertex);
        }

        #region Utilities

        bool Intersection(Vector3[] p, out Vector3 intersection, out float absDiv)
        {
            Vector3 cd = p[3] - p[2];
            Vector3 ab = p[1] - p[0];
            float div = cd.y * ab.x - cd.x * ab.y;
            absDiv = Mathf.Abs(div);

            if (absDiv > 0.0001f)
            {
                float ua = (cd.x * (p[0].y - p[2].y) - cd.y * (p[0].x - p[2].x)) / div;
                float ub = (ab.x * (p[0].y - p[2].y) - ab.y * (p[0].x - p[2].x)) / div;

                if (ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f)
                {
                    intersection = p[0] + ua * ab;
                    return true;
                }
            }
            intersection = new Vector3(0, 0, 0);
            return false;
        }

        public static float Lerp(float start, float end, float t)
        {
            return start + (t * (end - start));
        }

        public static Color LerpColor(Color A, Color B, float t)
        {
            return new Color(
                    Lerp(A.r, B.r, t),
                    Lerp(A.g, B.g, t),
                    Lerp(A.b, B.b, t),
                    Lerp(A.a, B.a, t)
                );
        }

        private float magnitude(Vector3 vec)
        {
            return Mathf.Sqrt((vec.x * vec.x) + (vec.y * vec.y) + (vec.z * vec.z));
        }

        #endregion
    }

    internal static class LREnum
    {
        public const short A = 0;
        public const short AB = 1;
        public const short BC = 2;
        public const short C = 3;
    }
}
