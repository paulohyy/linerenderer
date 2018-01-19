using Godot;

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

        private float progress;
        private float vertexStep;

        private float lenght = 1000;
        private int updateDivisor = 1;
        private int jointDivisor = 1;
        private int cornerDivisor = 1000;

        public LineRenderer(Node node, Camera camera, int vertexSmoothCount = 3) // TODO: pick shader, pick color, pick thickness for each point, round edges
        {
            var shader = ResourceLoader.Load(@"Shaders/default.shader");
            mat = new ShaderMaterial();
            mat.SetShader(shader as Shader);

            imGeo = new ImmediateGeometry();
            imGeo.SetCastShadowsSetting(GeometryInstance.ShadowCastingSetting.DoubleSided);
            imGeo.SetMaterialOverride(mat);
            node.AddChild(imGeo);

            this.camera = camera;
            this.cornerSmooth = vertexSmoothCount;

            last = new Vector3[2];

            startThickness = endThickness = 0.1f;
        }

        private void Corner(Vector3 center, Vector3 pivot)
        {
            var axis = (center - camera.GlobalTransform.origin).Normalized();
            var orthogonal = axis.Cross((center - pivot) / 2);
            var radius = (orthogonal).Normalized();
            var step = Mathf.PI / (cornerSmooth / 2);

            var array = new Vector3[cornerSmooth];

            for (int i = 1; i <= cornerSmooth; i++)
                array[i - 1] = (radius.Rotated(axis, step * i));

            for (int i = 1; i < cornerSmooth; i++)
            {
                AddVertex((thickness * array[i - 1]) + center, cornerDivisor);
                AddVertex((thickness * array[i]) + center, cornerDivisor);
                AddVertex(center, cornerDivisor);
            }

            AddVertex((thickness * array[cornerSmooth - 1]) + center, cornerDivisor);
            AddVertex((thickness * array[0]) + center, cornerDivisor);
            AddVertex(center, cornerDivisor);
        }

        private float magnitude(Vector3 vec)
        {
            return Mathf.Sqrt((vec.x * vec.x) + (vec.y * vec.y) + (vec.z * vec.z));
        }

        private void FillJoint(Vector3[] quad, int count)
        {
            for (int i = 0; i < count; i++)
            {
                var AA = (quad[1].Lerp(quad[2], (float)(i) / count)) - quad[3];
                var BB = (quad[1].Lerp(quad[2], (float)(i + 1) / count)) - quad[3];
                AddVertex(quad[0], jointDivisor);
                AddVertex((AA.Normalized() * thickness) + quad[3], jointDivisor);
                AddVertex((BB.Normalized() * thickness) + quad[3], jointDivisor);
            }
        }

        private void Joint(Vector3[] triple, float i, int length, bool stop = false)
        {
            var AB = triple[1] - triple[0];
            var BC = triple[2] - triple[1];
            var orthogonalAB = (camera.GlobalTransform.origin - ((triple[0] + triple[1]) / 2)).Cross(AB).Normalized() * thickness;
            var orthogonalBC = (camera.GlobalTransform.origin - ((triple[1] + triple[2]) / 2)).Cross(BC).Normalized() * thickness;

            var upperA = last[0] != ZERO ? last[0] : triple[0] + orthogonalAB;
            var lowerA = last[1] != ZERO ? last[1] : triple[0] - orthogonalAB;
            var upperAB = triple[1] + orthogonalAB;
            var lowerAB = triple[1] - orthogonalAB;

            var upperBC = triple[1] + orthogonalBC;
            var lowerBC = triple[1] - orthogonalBC;
            var upperC = triple[2] + orthogonalBC;
            var lowerC = triple[2] - orthogonalBC;

            if (Intersection(upperA, upperAB, upperBC, upperC, out Vector3 upper))
            {
                AddVertex(upper, updateDivisor);
                AddVertex(lowerA, updateDivisor);
                AddVertex(upperA, updateDivisor);
                AddVertex(lowerAB, updateDivisor);
                AddVertex(lowerA, updateDivisor);
                AddVertex(upper, updateDivisor);
                last[0] = upper;
                last[1] = ZERO;
                FillJoint(new Vector3[] { upper, lowerBC, lowerAB, triple[1] }, 3);

            }
            else if (Intersection(lowerA, lowerAB, lowerBC, lowerC, out Vector3 lower))
            {
                AddVertex(upperAB, updateDivisor);
                AddVertex(lowerA, updateDivisor);
                AddVertex(upperA, updateDivisor);
                AddVertex(upperAB, updateDivisor);
                AddVertex(lower, updateDivisor);
                AddVertex(lowerA, updateDivisor);
                last[0] = ZERO;
                last[1] = lower;
                FillJoint(new Vector3[] { lower, upperAB, upperBC, triple[1] }, 3);
            }
            else
            {
                AddVertex(upperAB, updateDivisor);
                AddVertex(lowerA, updateDivisor);
                AddVertex(upperA, updateDivisor);
                AddVertex(upperAB, updateDivisor);
                AddVertex(lowerAB, updateDivisor);
                AddVertex(lowerA, updateDivisor);
                last[0] = ZERO;
                last[1] = ZERO;
                AddVertex(upperAB, updateDivisor);
                AddVertex(upperBC, updateDivisor);
                AddVertex(lowerAB, updateDivisor);
                AddVertex(upperBC, updateDivisor);
                AddVertex(lowerBC, updateDivisor);
                AddVertex(lowerAB, updateDivisor);
            }
            i++;
            if (i == length - 1 && !stop)
                Joint(new Vector3[] { triple[1], triple[2], triple[2] + (triple[2] - triple[1]) }, i, length, true);
        }

        public void Update(Vector3[] points)
        {
            if (points.Length < 3)
                return;

            vertexStep = 1.0f / lenght;
            lenght = 0;
            progress = 0;

            imGeo.Clear();
            imGeo.Begin(Mesh.PrimitiveType.Triangles);
            last[0] = last[1] = ZERO;

            for (int i = 1; i < points.Length - 1; i++)
                Joint(new Vector3[] { points[i - 1], points[i], points[i + 1] }, i, points.Length);

            //var orthogonal = camera.GlobalTransform.origin - points[points.Length - 1].Cross(points[points.Length - 1] - points[points.Length - 2]).Normalized();
            //var left = points[points.Length - 1] - orthogonal;
            //var right = points[points.Length - 1] + orthogonal;
            //FillJoint(new Vector3[] { points[points.Length - 1] + (points[points.Length - 1] - points[points.Length - 2]), left, right, points[points.Length - 1] }, 7);

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

        private void AddVertex(Vector3 vertex, int divisor = 0)
        {
            divisor = (divisor > 0 ? divisor : 1);
            lenght += (float)1 / divisor;
            progress += vertexStep / divisor;
            thickness = Lerp(startThickness, endThickness, progress);
            imGeo.SetColor(LerpColor(startColor, endColor, progress));
            imGeo.AddVertex(vertex);
        }

        bool Intersection(Vector3 a, Vector3 b, Vector3 c, Vector3 d, out Vector3 intersection)
        {
            Vector3 cd = d - c;
            Vector3 ab = b - a;
            float div = cd.y * ab.x - cd.x * ab.y;

            if (Mathf.Abs(div) > 0.1f)
            {
                float ua = (cd.x * (a.y - c.y) - cd.y * (a.x - c.x)) / div;
                float ub = (ab.x * (a.y - c.y) - ab.y * (a.x - c.x)) / div;

                if (ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f)
                {
                    intersection = a + ua * ab;
                    return true;
                }
            }
            intersection = new Vector3(0, 0, 0);
            return false;
        }
    }
}
