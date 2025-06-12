using UnityEngine;

namespace SoftBody.Scripts
{

    public static class MeshFactory
    {
        public static Mesh CreatePlane(float size = 1.0f)
        {
            var mesh = new Mesh
            {
                name = "Procedural Plane"
            };

            var halfSize = size / 2.0f;

            var vertices = new Vector3[]
            {
                new(-halfSize, 0, -halfSize), // Bottom-left
                new(halfSize, 0, -halfSize), // Bottom-right
                new(-halfSize, 0, halfSize), // Top-left
                new(halfSize, 0, halfSize) // Top-right
            };
            mesh.vertices = vertices;

            // Triangles are defined by vertex indices.
            // The order determines the "front" face (winding order).
            var triangles = new int[6]
            {
                // First triangle
                0, 2, 1,
                // Second triangle
                2, 3, 1
            };
            mesh.triangles = triangles;

            // Add default UVs and Normals for proper rendering
            var uv = new Vector2[4]
            {
                new(0, 0),
                new(1, 0),
                new(0, 1),
                new(1, 1)
            };
            mesh.uv = uv;

            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            return mesh;
        }

        public static Mesh CreateCube(float size = 1.0f)
        {
            Mesh mesh = new Mesh();
            mesh.name = "Procedural Cube";

            float halfSize = size / 2.0f;

            // The 8 unique vertices of a cube
            Vector3[] vertices = new Vector3[]
            {
                new Vector3(-halfSize, -halfSize, -halfSize), // 0
                new Vector3(halfSize, -halfSize, -halfSize), // 1
                new Vector3(halfSize, -halfSize, halfSize), // 2
                new Vector3(-halfSize, -halfSize, halfSize), // 3
                new Vector3(-halfSize, halfSize, -halfSize), // 4
                new Vector3(halfSize, halfSize, -halfSize), // 5
                new Vector3(halfSize, halfSize, halfSize), // 6
                new Vector3(-halfSize, halfSize, halfSize) // 7
            };

            // Triangles defined with a correct counter-clockwise (CCW) winding order
            // when viewed from the outside.
            int[] triangles = new int[]
            {
                // --- Corrected Winding Order ---
                // Bottom face (-Y)
                0, 1, 2, // Was 0, 2, 1
                0, 2, 3, // Was 0, 3, 2

                // Top face (+Y)
                4, 6, 5, // Was 4, 5, 6
                4, 7, 6, // Was 4, 6, 7

                // Front face (+Z)
                3, 2, 6, // Was 3, 6, 2
                3, 6, 7, // Was 3, 7, 6

                // Back face (-Z)
                0, 5, 1, // Was 0, 1, 5
                0, 4, 5, // Was 0, 5, 4

                // Left face (-X)
                0, 7, 4, // Was 0, 4, 7
                0, 3, 7, // Was 0, 7, 3

                // Right face (+X)
                1, 6, 2, // Was 1, 2, 6
                1, 5, 6 // Was 1, 6, 5
            };

            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals(); // This will now calculate correct, outward-facing normals
            mesh.RecalculateBounds();

            return mesh;
        }
    }
}