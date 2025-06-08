using System.Collections.Generic;
using Codex.Input;
using SoftBody.Scripts.Constraints;
using SoftBody.Scripts.Models;
using UnityEngine;

namespace SoftBody.Scripts
{
    public class SoftBodyCubeCPU : MonoBehaviour
    {
        [Header("Simulation Settings")]
        [SerializeField] private int solverIterations = 10;
        [SerializeField] private float globalCompliance = 0.01f; // Softer = higher value
        [SerializeField] private Vector3 gravity = new(0, -9.81f, 0);
        [SerializeField] private float damping = 0.01f; // Velocity damping factor per step


        [Header("Cube Settings")] 
        [SerializeField] private float cubeSize = 1.0f;
        [SerializeField] private float particleMass = 1.0f;
        [SerializeField] private bool addFaceDiagonals = true;
        [SerializeField] private bool addInternalDiagonals = false;
        
        [Header("Collision")] 
        [SerializeField] private float groundHeight = 0.0f;
        [SerializeField] private float collisionCompliance = 0.0f; // For collision constraints (0 = rigid)
        [SerializeField] private float friction = 0.1f; // Simple friction for collision response

        [Header("Debug")]
        [SerializeField] private InputReaderSO inputReader;

        private List<SoftBodyParticle> _particles = new();
        private List<DistanceConstraint> _distanceConstraints = new();
        private Mesh _displayMesh;
        private Vector3[] _meshVertices;

        private void OnEnable()
        {
            inputReader.Test1PressedEvent += RestartSimulation;
        }
        
        private void OnDisable()
        {
            inputReader.Test1PressedEvent -= RestartSimulation;
        }
        
        private void Start()
        {
            InitMesh();
            CreateCube();
        }
        
        private void RestartSimulation()
        {
            CreateCube();
        }

        private void InitMesh()
        {
            var meshFilter = GetComponent<MeshFilter>();
            
            if (meshFilter.sharedMesh != null) {
                _displayMesh = Instantiate(meshFilter.sharedMesh); 
            } else {
                _displayMesh = new Mesh();
                _displayMesh.name = "SoftBody_DisplayMesh";
                GenerateCubeMeshForDisplay(); 
            }
            meshFilter.mesh = _displayMesh;

            if (_displayMesh.vertexCount != _particles.Count && _particles.Count > 0)
            {
                Debug.LogWarning("Particle count does not match display mesh vertex count! Visuals may be incorrect. Ensure CreateCube particle order matches mesh vertex order.");
            }
            _meshVertices = new Vector3[_displayMesh.vertexCount]; 
        }
        
        private void GenerateCubeMeshForDisplay()
        {
            // This needs to create 8 vertices in the SAME ORDER as the particles are created in CreateCube()
            // And define the 12 triangles (2 per face) for the cube.

            var halfSize = cubeSize / 2.0f; 

            var vertices = new Vector3[]
            {
                new(-halfSize, -halfSize, -halfSize), // 0 Front Bottom Left
                new( halfSize, -halfSize, -halfSize), // 1 Front Bottom Right
                new( halfSize,  halfSize, -halfSize), // 2 Front Top Right
                new(-halfSize,  halfSize, -halfSize), // 3 Front Top Left
                new(-halfSize, -halfSize,  halfSize), // 4 Back Bottom Left
                new( halfSize, -halfSize,  halfSize), // 5 Back Bottom Right
                new( halfSize,  halfSize,  halfSize), // 6 Back Top Right
                new(-halfSize,  halfSize,  halfSize)  // 7 Back Top Left
            };
            
            _displayMesh.vertices = vertices;
            _meshVertices = new Vector3[vertices.Length];
            
            var newVertices = new[]
            {
                // Front face
                vertices[0], vertices[1], vertices[2], vertices[3],
                // Back face
                vertices[5], vertices[4], vertices[7], vertices[6],
                // Top face
                vertices[3], vertices[2], vertices[6], vertices[7], 
                // Bottom face
                vertices[4], vertices[5], vertices[1], vertices[0], 
                // Right face
                vertices[1], vertices[5], vertices[6], vertices[2], 
                // Left face
                vertices[4], vertices[0], vertices[6], vertices[7]  
            };
            _displayMesh.vertices = newVertices; // Use the 24 vertices

            var uvs = new Vector2[24];
            for (var i = 0; i < 6; i++) 
            {
                uvs[i * 4 + 0] = new Vector2(0, 0); // Bottom-left of texture quad
                uvs[i * 4 + 1] = new Vector2(0, 1); // Top-left
                uvs[i * 4 + 2] = new Vector2(1, 1); // Top-right
                uvs[i * 4 + 3] = new Vector2(1, 0); // Bottom-right
            }
            _displayMesh.uv = uvs;
            
            // Each face is a quad, so 2 triangles per face.

            var triangles = new int[36]; // 6 faces * 2 triangles/face * 3 indices/triangle
            for (var i = 0; i < 6; i++)
            {
                var baseIndex = i * 4;
                triangles[i * 6 + 0] = baseIndex + 0;
                triangles[i * 6 + 1] = baseIndex + 1;
                triangles[i * 6 + 2] = baseIndex + 2;

                triangles[i * 6 + 3] = baseIndex + 0;
                triangles[i * 6 + 4] = baseIndex + 2;
                triangles[i * 6 + 5] = baseIndex + 3;
            }
            _displayMesh.triangles = triangles;

            _displayMesh.RecalculateNormals();
            _displayMesh.RecalculateBounds();

            // Define triangles (2 per face, 12 total, 36 indices)
            // var triangles = new int[]
            // {
            //     // // Front face (-Z)
            //     // 0, 2, 1, 0, 3, 2,
            //     // // Back face (+Z)
            //     // 4, 5, 6, 4, 6, 7,
            //     // // Top face (+Y)
            //     // 3, 6, 2, 3, 7, 6,
            //     // // Bottom face (-Y)
            //     // 0, 1, 5, 0, 5, 4,
            //     // // Right face (+X)
            //     // 1, 6, 5, 1, 2, 6,
            //     // // Left face (-X)
            //     // 4, 7, 3, 4, 3, 0
            //
            //     // Front Face (Target Outward Normal: 0,0,-1)
            //     0, 2, 3,  // Flipped from 0,3,2
            //     0, 1, 2,  // Flipped from 0,2,1
            //
            //     // Back Face (Target Outward Normal: 0,0,1)
            //     4, 6, 5,  // Flipped from 4,5,6
            //     4, 7, 6,  // Flipped from 4,6,7
            //
            //     // Top Face (Target Outward Normal: 0,1,0)
            //     7, 2, 6,  // Flipped from 7,6,2
            //     7, 3, 2,  // Flipped from 7,2,3
            //
            //     // Bottom Face (Target Outward Normal: 0,-1,0)
            //     0, 5, 1,  // Flipped from 0,1,5
            //     0, 4, 5,  // Flipped from 0,5,4
            //
            //     // Right Face (Target Outward Normal: 1,0,0)
            //     1, 6, 2,  // Flipped from 1,2,6
            //     1, 5, 6,  // Flipped from 1,6,5
            //
            //     // Left Face (Target Outward Normal: -1,0,0)
            //     4, 3, 7,  // Flipped from 4,7,3
            //     4, 0, 3   // Flipped from 4,3,0
            // };
            // _displayMesh.triangles = triangles;
            //
            // // UVs for cube
            //
            // var uvs = new Vector2[]
            // {
            //     new(0, 0), new(1, 0), new(1, 1), new(0, 1), // Front face
            //     new(0, 0), new(1, 0), new(1, 1), new(0, 1), // Back face
            // };
            // _displayMesh.uv = uvs;
            //
            // _displayMesh.RecalculateNormals();
            // _displayMesh.RecalculateBounds();
        }

        private void CreateCube()
        {
            _particles.Clear();
            _distanceConstraints.Clear();

            var halfSize = cubeSize / 2.0f;

            // Define 8 particles for the cube corners
            var localParticlePositions = new Vector3[]
            {
                new(-halfSize, -halfSize, -halfSize), // 0
                new(halfSize, -halfSize, -halfSize),  // 1
                new(halfSize, halfSize, -halfSize),   // 2
                new(-halfSize, halfSize, -halfSize),  // 3
                new(-halfSize, -halfSize, halfSize),  // 4
                new(halfSize, -halfSize, halfSize),   // 5
                new(halfSize, halfSize, halfSize),    // 6
                new(-halfSize, halfSize, halfSize)    // 7
            };

            for (var i = 0; i < localParticlePositions.Length; i++)
            {
                _particles.Add(new SoftBodyParticle(transform.TransformPoint(localParticlePositions[i]), particleMass, i));
            }

            // Define edge distance constraints (12)
            AddDistanceConstraint(0, 1, globalCompliance);
            AddDistanceConstraint(1, 2, globalCompliance);
            AddDistanceConstraint(2, 3, globalCompliance);
            AddDistanceConstraint(3, 0, globalCompliance);

            AddDistanceConstraint(4, 5, globalCompliance);
            AddDistanceConstraint(5, 6, globalCompliance);
            AddDistanceConstraint(6, 7, globalCompliance);
            AddDistanceConstraint(7, 4, globalCompliance);

            AddDistanceConstraint(0, 4, globalCompliance);
            AddDistanceConstraint(1, 5, globalCompliance);
            AddDistanceConstraint(2, 6, globalCompliance);
            AddDistanceConstraint(3, 7, globalCompliance);

            if (addFaceDiagonals)
            {
                // 6 faces * 2 diagonals per face = 12 constraints
                AddDistanceConstraint(0, 2, globalCompliance); // Bottom face
                AddDistanceConstraint(1, 3, globalCompliance);

                AddDistanceConstraint(4, 6, globalCompliance); // Top face
                AddDistanceConstraint(5, 7, globalCompliance);

                AddDistanceConstraint(0, 5, globalCompliance); // Front face (assuming -z is front)
                AddDistanceConstraint(1, 4, globalCompliance);
                // ... and so on for the other 3 faces
                AddDistanceConstraint(1, 6, globalCompliance); // Right face
                AddDistanceConstraint(2, 5, globalCompliance);

                AddDistanceConstraint(2, 7, globalCompliance); // Back face
                AddDistanceConstraint(3, 6, globalCompliance);

                AddDistanceConstraint(3, 4, globalCompliance); // Left face
                AddDistanceConstraint(0, 7, globalCompliance);
            }

            if (addInternalDiagonals)
            {
                // 4 internal diagonals
                AddDistanceConstraint(0, 6, globalCompliance);
                AddDistanceConstraint(1, 7, globalCompliance);
                AddDistanceConstraint(2, 4, globalCompliance);
                AddDistanceConstraint(3, 5, globalCompliance);
            }
        }

        private void AddDistanceConstraint(int particleAIndex, int particleBIndex, float compliance)
        {
            var particleA = _particles[particleAIndex];
            var particleB = _particles[particleBIndex];
            var restLength = Vector3.Distance(particleA.Position, particleB.Position);
            _distanceConstraints.Add(new DistanceConstraint(particleA, particleB, restLength, compliance, float.MaxValue));
        }


        private void FixedUpdate()
        {
            var dt = Time.fixedDeltaTime;
            if (dt <= 0) return;
            
            // Reset all constraints' lambda values
            foreach (var constraint in _distanceConstraints)
            {
                constraint.ResetLambda();
            }

            // 1. Apply external forces (Gravity) and predict positions
            foreach (var p in _particles)
            {
                p.ClearForces();
                if (p.InverseMass > 0) // Don't apply gravity to static particles
                {
                    p.AddForce(gravity);
                }

                // Update velocity
                p.Velocity += dt * p.InverseMass * p.ExternalForceAccumulator;

                // Apply damping
                p.Velocity *= (1f - Mathf.Clamp01(damping)); // Basic velocity damping

                // Predict next position
                p.PredictedPosition = p.Position + dt * p.Velocity;
            }

            // Initialize constraints for sub-stepping if needed (XPBD specific)
            foreach (var constraint in _distanceConstraints)
            {
                constraint.InitializeForSubstep();
            }


            // 2. Solver iterations (PBD/XPBD core loop)
            for (var i = 0; i < solverIterations; i++)
            {
                // 2a. Solve distance constraints
                foreach (var constraint in _distanceConstraints)
                {
                    constraint.Solve(dt);
                }

                // 2b. Solve collisions (as constraints or direct projections)
                SolveCollisions(dt);
            }

            // 3. Update velocities and positions
            foreach (var p in _particles)
            {
                if (p.InverseMass < 0.00001f) // Static particle
                {
                    p.Velocity = Vector3.zero;
                    p.PredictedPosition = p.Position; // Ensure it doesn't move
                    continue;
                }

                // Update velocity based on position change
                p.Velocity = (p.PredictedPosition - p.Position) / dt;

                // Update final position
                p.Position = p.PredictedPosition;
            }
        }

        private void LateUpdate()
        {
            if (_displayMesh is null || _particles.Count != 8 || _displayMesh.vertexCount != 24) // Check counts
            {
                Debug.LogWarning("Mesh or particle count incorrect for 24-vertex textured cube update.");
                return;
            }

            var currentMeshVertices = new Vector3[24];

            // Original particle references (for clarity)
            var p0 = transform.InverseTransformPoint(_particles[0].Position); // FBL
            var p1 = transform.InverseTransformPoint(_particles[1].Position); // FBR
            var p2 = transform.InverseTransformPoint(_particles[2].Position); // FTR
            var p3 = transform.InverseTransformPoint(_particles[3].Position); // FTL
            var p4 = transform.InverseTransformPoint(_particles[4].Position); // BBL
            var p5 = transform.InverseTransformPoint(_particles[5].Position); // BBR
            var p6 = transform.InverseTransformPoint(_particles[6].Position); // BTR
            var p7 = transform.InverseTransformPoint(_particles[7].Position); // BTL

            // Front face (v0, v1, v2, v3 in newVertices) -> particles (FBL, FBR, FTR, FTL)
            currentMeshVertices[0] = p0;
            currentMeshVertices[1] = p1;
            currentMeshVertices[2] = p2;
            currentMeshVertices[3] = p3;

            // Back face (v5, v4, v7, v6 in newVertices) -> particles (BBR, BBL, BTL, BTR)
            currentMeshVertices[4] = p5;
            currentMeshVertices[5] = p4;
            currentMeshVertices[6] = p7;
            currentMeshVertices[7] = p6;

            // Top face (v3, v2, v6, v7 in newVertices) -> particles (FTL, FTR, BTR, BTL)
            currentMeshVertices[8] =  p3;
            currentMeshVertices[9] =  p2;
            currentMeshVertices[10] = p6;
            currentMeshVertices[11] = p7;

            // Bottom face (v4, v5, v1, v0 in newVertices) -> particles (BBL, BBR, FBR, FBL)
            currentMeshVertices[12] = p4;
            currentMeshVertices[13] = p5;
            currentMeshVertices[14] = p1;
            currentMeshVertices[15] = p0;

            // Right face (v1, v5, v6, v2 in newVertices) -> particles (FBR, BBR, BTR, FTR)
            currentMeshVertices[16] = p1;
            currentMeshVertices[17] = p5;
            currentMeshVertices[18] = p6;
            currentMeshVertices[19] = p2;

            // Left face (v4, v0, v3, v7 in newVertices) -> particles (BBL, FBL, FTL, BTL)
            currentMeshVertices[20] = p4;
            currentMeshVertices[21] = p0;
            currentMeshVertices[22] = p3;
            currentMeshVertices[23] = p7;

            _displayMesh.vertices = currentMeshVertices;

            _displayMesh.RecalculateNormals(); // Normals will also need to be correct for the 24 vertices
            _displayMesh.RecalculateBounds();
        }

        private void SolveCollisions(float deltaTime)
        {
            foreach (var p in _particles)
            {
                if (p.InverseMass < 0.00001f)
                {
                    continue;
                }

                // Ground collision
                if (p.PredictedPosition.y < groundHeight)
                {
                    // Simple projection
                    // Vector3 correction = new Vector3(0, GroundHeight - p.PredictedPosition.y, 0);
                    // p.PredictedPosition += correction;

                    // --- XPBD-style Collision Constraint (Inequality) ---
                    // Constraint C = GroundHeight - p.PredictedPosition.y  (we want C <= 0)
                    // So, if p.PredictedPosition.y < GroundHeight, then C > 0, violation!
                    var C = groundHeight - p.PredictedPosition.y;
                    if (C < 0) // Particle is above or on ground, no violation (or very small penetration)
                    {
                        continue;
                    }

                    var groundPlaneNormal = Vector3.up;

                    // XPBD specific alpha_tilde for collision (can be 0 for hard constraint)
                    var alphaTildeCollision = collisionCompliance / (deltaTime * deltaTime);

                    // Denominator for collision: w_i * |gradient|^2. Gradient is 'normal', magnitude 1.
                    var denominator = p.InverseMass + alphaTildeCollision;
                    if (Mathf.Abs(denominator) < 0.00001f) continue;

                    // For inequality constraints, lambda is typically clamped (lambda >= 0)
                    // Here, we are calculating a deltaLambda for the correction directly.
                    // The "lambda" for this collision step is effectively 0 if we don't accumulate it.
                    var deltaLambdaCollision = (C - alphaTildeCollision * 0f) / denominator; // 0f is current lambda_collision

                    // Apply correction (deltaX)
                    var dPCollision = p.InverseMass * deltaLambdaCollision * groundPlaneNormal;
                    p.PredictedPosition += dPCollision;

                    // --- Simple Friction (apply after position correction) ---
                    // Recalculate velocity relative to the collision point for friction
                    var velocityAtCollisionPoint = (p.PredictedPosition - p.Position) / deltaTime;
                    var velocityTangent = velocityAtCollisionPoint -
                                          Vector3.Dot(velocityAtCollisionPoint, groundPlaneNormal) * groundPlaneNormal;
                    p.PredictedPosition -= velocityTangent * (deltaTime * Mathf.Clamp01(friction)); // Apply friction impulse by adjusting position
                }
            }
        }

        private void OnDrawGizmos()
        {
            if (_particles == null || _particles.Count == 0)
            {
                // If particles aren't initialized yet, draw the intended initial cube shape
                if (Application.isPlaying) return; // Only draw this guide if not playing

                Gizmos.color = Color.gray;
                var halfSize = cubeSize / 2.0f;
                var localPositions = new Vector3[]
                {
                    new(-halfSize, -halfSize, -halfSize), new(halfSize, -halfSize, -halfSize),
                    new(halfSize, halfSize, -halfSize), new(-halfSize, halfSize, -halfSize),
                    new(-halfSize, -halfSize, halfSize), new(halfSize, -halfSize, halfSize),
                    new(halfSize, halfSize, halfSize), new(-halfSize, halfSize, halfSize)
                };
                for (var i = 0; i < localPositions.Length; ++i)
                {
                    Gizmos.DrawSphere(transform.TransformPoint(localPositions[i]), 0.05f);
                }

                // Draw a line for the ground
                Gizmos.color = Color.green;
                Gizmos.DrawLine(new Vector3(-5, groundHeight, 0), new Vector3(5, groundHeight, 0));
                return;
            }


            // Draw particles
            Gizmos.color = Color.yellow;
            foreach (var p in _particles)
            {
                Gizmos.DrawSphere(p.Position, 0.05f);
            }

            // Draw distance constraints
            Gizmos.color = Color.cyan;
            foreach (var constraint in _distanceConstraints)
            {
                if (constraint.ParticleA != null && constraint.ParticleB != null)
                {
                    Gizmos.DrawLine(constraint.ParticleA.Position, constraint.ParticleB.Position);
                }
            }

            // Draw ground
            Gizmos.color = Color.green;
            Gizmos.DrawLine(new Vector3(-5, groundHeight, 0), new Vector3(5, groundHeight, 0));
        }
    }
}
