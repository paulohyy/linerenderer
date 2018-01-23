using Godot;

namespace SkipTheBadEngine
{
    /// <summary>
    /// Simple line renderer with meshes that always faces the camera.
    /// </summary>
    public class LineRenderer
    {
        private static Vector3 ZERO = new Vector3(0, 0, 0);

        private ImmediateGeometry geometry;
        private ShaderMaterial material;
        private Camera camera;

        private Vector3[] Last;
        private Vector3[] Up;
        private Vector3[] Low;
        private Vector3[] Variables;
        private bool[] Crossed;

        private float[] Score;

        private float progress;
        private float progressStep;

        private int cornerSmooth;
        private int jointSmooth;

        private Color startColor;
        private Color endColor;

        private float startThickness;
        private float endThickness;
        private float thickness;
        private float nextThickness;

        private Vector3[] Segments;
        private float divisor;

        public LineRenderer(Node parent, Camera camera, int cornerSmooth, int jointSmooth, Shader shader)
        {
            material = new ShaderMaterial();
            material.SetShader(shader != null ? shader : ResourceLoader.Load(@"Shaders/default.shader") as Shader);

            geometry = new ImmediateGeometry();
            geometry.SetCastShadowsSetting(GeometryInstance.ShadowCastingSetting.DoubleSided);
            geometry.SetMaterialOverride(material);
            parent.AddChild(geometry);

            this.camera = camera;
            this.cornerSmooth = (cornerSmooth / 2);
            this.jointSmooth = jointSmooth;

            Last = new Vector3[2];
            Up = new Vector3[4];
            Low = new Vector3[4];
            Variables = new Vector3[6];
            Crossed = new bool[2];
            Score = new float[2];
            Segments = new Vector3[2];

            startThickness = endThickness = 0.1f;
        }

        /// <summary>
        /// Line should have at least 3 points.
        /// </summary>
        public void Update(Vector3[] points)
        {
            if (points.Length < 3)
                return;

            progressStep = 1.0f / points.Length;
            progress = 0;
            thickness = Lerp(startThickness, endThickness, progress);
            nextThickness = Lerp(startThickness, endThickness, progress + progressStep);

            geometry.Clear();
            geometry.Begin(Mesh.PrimitiveType.Triangles);
            Last[0] = Last[1] = ZERO;

            Corner(points[0], points[1]);
            for (int i = 1; i < points.Length - 1; i++)
            {
                Joint(new Vector3[] { points[i - 1], points[i], points[i + 1] }, i, points.Length);
                thickness = Lerp(startThickness, endThickness, progress);
                nextThickness = Lerp(startThickness, endThickness, progress + progressStep);
                progress += progressStep;
            }
            Corner(points[points.Length - 1], points[points.Length - 2]);

            geometry.End();
        }

        /// <summary>
        /// Use this if the line has only 2 points.
        /// </summary>
        public void SimpleUpdate(Vector3 A, Vector3 B)
        {
            geometry.Clear();
            geometry.Begin(Mesh.PrimitiveType.Triangles);

            thickness = startThickness;
            Corner(A, B);

            var AB = B - A;
            var orthogonalABStart = (camera.GlobalTransform.origin - ((A + B) / 2)).Cross(AB).Normalized() * startThickness;
            var orthogonalABEnd = (camera.GlobalTransform.origin - ((A + B) / 2)).Cross(AB).Normalized() * endThickness;

            Up[LR.A] = A + orthogonalABStart;
            Up[LR.AB] = B + orthogonalABEnd;
            Low[LR.A] = A - orthogonalABStart;
            Low[LR.AB] = B - orthogonalABEnd;

            AddVertex(Up[LR.A]);
            AddVertex(Up[LR.AB]);
            AddVertex(Low[LR.A]);
            AddVertex(Up[LR.AB]);
            AddVertex(Low[LR.AB]);
            AddVertex(Low[LR.A]);

            thickness = endThickness;
            Corner(B, A);

            geometry.End();
        }

        private void Joint(Vector3[] triple, float i, int length)
        {
            Variables[LR.VarAB] = triple[1] - triple[0];
            Variables[LR.VarBC] = triple[2] - triple[1];
            Variables[LR.CamToCenter] = camera.GlobalTransform.origin - triple[1];
            Variables[LR.OrthABStart] = Variables[LR.CamToCenter].Cross(Variables[LR.VarAB]).Normalized() * thickness;
            Variables[LR.OrthABEnd] = Variables[LR.CamToCenter].Cross(Variables[LR.VarAB]).Normalized() * nextThickness;
            Variables[LR.OrthBC] = Variables[LR.CamToCenter].Cross(Variables[LR.VarBC]).Normalized() * nextThickness;

            Up[LR.A] = Last[0] != ZERO ? Last[LR.Upper] : triple[LR.Left] + Variables[LR.OrthABStart];
            Up[LR.AB] = triple[LR.Center] + Variables[LR.OrthABEnd];
            Up[LR.BC] = triple[LR.Center] + Variables[LR.OrthBC];
            Up[LR.C] = triple[LR.Right] + Variables[LR.OrthBC];

            Low[LR.A] = Last[1] != ZERO ? Last[LR.Lower] : triple[LR.Left] - Variables[LR.OrthABStart];
            Low[LR.AB] = triple[LR.Center] - Variables[LR.OrthABEnd];
            Low[LR.BC] = triple[LR.Center] - Variables[LR.OrthBC];
            Low[LR.C] = triple[LR.Right] - Variables[LR.OrthBC];

            Crossed[LR.Upper] = Intersection(Up, out Vector3 upper, out Score[LR.Upper]);
            Crossed[LR.Lower] = Intersection(Low, out Vector3 lower, out Score[LR.Lower]);
            Crossed[LR.Upper] = Crossed[LR.Upper] && Score[LR.Upper] > Score[LR.Lower];
            Crossed[LR.Lower] = Crossed[LR.Lower] && Score[LR.Lower] > Score[LR.Upper];

            if (Crossed[LR.Upper])
            {
                AddVertex(upper);
                AddVertex(Low[LR.A]);
                AddVertex(Up[LR.A]);
                AddVertex(Low[LR.AB]);
                AddVertex(Low[LR.A]);
                AddVertex(upper);
                Last[0] = upper;
                Last[1] = ZERO;
                FillJoint(new Vector3[] { upper, Low[LR.BC], Low[LR.AB], triple[1] }, jointSmooth);
            }
            else if (Crossed[LR.Lower])
            {
                AddVertex(Up[LR.AB]);
                AddVertex(Low[LR.A]);
                AddVertex(Up[LR.A]);
                AddVertex(Up[LR.AB]);
                AddVertex(lower);
                AddVertex(Low[LR.A]);
                Last[0] = ZERO;
                Last[1] = lower;
                FillJoint(new Vector3[] { lower, Up[LR.AB], Up[LR.BC], triple[1] }, jointSmooth);
            }
            else
            {
                AddVertex(Up[LR.AB]);
                AddVertex(Low[LR.A]);
                AddVertex(Up[LR.A]);
                AddVertex(Up[LR.AB]);
                AddVertex(Low[LR.AB]);
                AddVertex(Low[LR.A]);
                Last[0] = ZERO;
                Last[1] = ZERO;
                AddVertex(Up[LR.AB]);
                AddVertex(Up[LR.BC]);
                AddVertex(Low[LR.AB]);
                AddVertex(Up[LR.BC]);
                AddVertex(Low[LR.BC]);
                AddVertex(Low[LR.AB]);
            }

            i++;
            if (i == length - 1)
                Joint(new Vector3[] { triple[LR.Center], triple[LR.Right], triple[LR.Right] + (triple[LR.Right] - triple[LR.Center]) }, i, length);
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

        private void Corner(Vector3 center, Vector3 pivot)
        {
            var orthogonal =
                (camera.GlobalTransform.origin - center)
                .Cross(center - pivot)
                .Normalized() * thickness;

            var axis = (center - camera.GlobalTransform.origin).Normalized();

            var array = new Vector3[cornerSmooth + 1];
            array[0] = center + orthogonal;
            array[cornerSmooth] = center - orthogonal;

            for (int i = 1; i < cornerSmooth; i++)
                array[i] = center + (orthogonal.Rotated(axis, Lerp(0, Mathf.PI, (float)(i) / cornerSmooth)));

            for (int i = 1; i < cornerSmooth + 1; i++)
            {
                AddVertex(array[i - 1]);
                AddVertex(array[i]);
                AddVertex(center);
            }
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
            geometry.SetColor(LerpColor(startColor, endColor, progress));
            geometry.AddVertex(vertex);
        }

        #region Utilities

        bool Intersection(Vector3[] p, out Vector3 intersection, out float absDiv)
        {
            Segments[LR.CtoD] = p[3] - p[2];
            Segments[LR.AtoB] = p[1] - p[0];
            divisor = Segments[LR.CtoD].y * Segments[LR.AtoB].x - Segments[LR.CtoD].x * Segments[LR.AtoB].y;
            absDiv = Mathf.Abs(divisor);

            if (absDiv > 0.0001f)
            {
                float ua = (Segments[LR.CtoD].x * (p[0].y - p[2].y) - Segments[LR.CtoD].y * (p[0].x - p[2].x)) / divisor;
                float ub = (Segments[LR.AtoB].x * (p[0].y - p[2].y) - Segments[LR.AtoB].y * (p[0].x - p[2].x)) / divisor;

                if (ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f)
                {
                    intersection = p[0] + ua * Segments[LR.AtoB];
                    return true;
                }
            }
            intersection = ZERO;
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

        internal static class LR
        {
            public const short A = 0;
            public const short AB = 1;
            public const short BC = 2;
            public const short C = 3;

            public const short VarAB = 0;
            public const short VarBC = 1;
            public const short CamToCenter = 2;
            public const short OrthABStart = 3;
            public const short OrthABEnd = 4;
            public const short OrthBC = 5;

            public const short Left = 0;
            public const short Center = 1;
            public const short Right = 2;

            public const short Upper = 0;
            public const short Lower = 0;

            public const short AtoB = 0;
            public const short CtoD = 1;
        }

        #endregion
    }
}
