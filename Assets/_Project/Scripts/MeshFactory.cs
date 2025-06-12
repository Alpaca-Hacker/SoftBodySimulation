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
            var mesh = new Mesh();
            mesh.name = "Procedural Cube";

            var halfSize = size / 2.0f;

            var vertices = new Vector3[]
            {
                // Bottom face
                new Vector3(-halfSize, -halfSize, -halfSize), // 0
                new Vector3(halfSize, -halfSize, -halfSize), // 1
                new Vector3(halfSize, -halfSize, halfSize), // 2
                new Vector3(-halfSize, -halfSize, halfSize), // 3

                // Top face
                new Vector3(-halfSize, halfSize, -halfSize), // 4
                new Vector3(halfSize, halfSize, -halfSize), // 5
                new Vector3(halfSize, halfSize, halfSize), // 6
                new Vector3(-halfSize, halfSize, halfSize) // 7
            };

            // This is a "welded" cube, meaning it only has 8 unique vertices.
            // The normals won't look sharp like a standard Unity cube until you unweld it,
            // but for a soft body simulation, this is exactly what we want.
            var triangles = new int[]
            {
                // Bottom face
                0, 2, 1,
                0, 3, 2,

                // Top face
                4, 5, 6,
                4, 6, 7,

                // Front face
                3, 7, 6,
                3, 6, 2,

                // Back face
                0, 1, 5,
                0, 5, 4,

                // Left face
                0, 4, 7,
                0, 7, 3,

                // Right face
                1, 2, 6,
                1, 6, 5
            };

            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            return mesh;
        }
    }
}