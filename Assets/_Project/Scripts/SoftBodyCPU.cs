using System.Collections.Generic;
using Codex.Input;
using SoftBody.Scripts.Constraints;
using SoftBody.Scripts.Models;
using UnityEngine;

namespace SoftBody.Scripts
{
    public class SoftBodyCPU : MonoBehaviour
    {
        [Header("Simulation Settings")]
        [SerializeField] private int solverIterations = 10;
        [SerializeField] private float globalCompliance = 0.01f; // Softer = higher value
        [SerializeField] private Vector3 gravity = new(0, -9.81f, 0);
        [SerializeField] private float damping = 0.01f; // Velocity damping factor per step

        [Header("Mesh Settings")] 
        [SerializeField] private Mesh sourceMesh; 
        [SerializeField] private float particleMass = 1.0f;
        
        [Header("Collision")] 
        [SerializeField] private float groundHeight = 0.0f;
        [SerializeField] private float collisionCompliance = 0.0f; // For collision constraints (0 = rigid)
        [SerializeField] private float friction = 0.1f; // Simple friction for collision response

        [Header("Debug")]
        [SerializeField] private InputReaderSO inputReader;
        [SerializeField] private bool showConstraints;

        private List<SoftBodyParticle> _particles = new();
        private List<DistanceConstraint> _distanceConstraints = new();
        private Mesh _displayMesh;
        private Vector3[] _initialMeshVerticesLocal; 
        private Vector3[] _currentMeshVerticesLocal;
        

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
            if (sourceMesh == null)
            {
                var mf = GetComponent<MeshFilter>();
                if (mf != null && mf.sharedMesh != null)
                {
                    sourceMesh = mf.sharedMesh; // Try to get from MeshFilter if not assigned
                    Debug.Log("Used mesh from MeshFilter.");
                } else
                {
                    Debug.LogError("Source Mesh is not assigned and no mesh found in MeshFilter. Cannot initialize soft body.");
                    enabled = false; // Disable the script
                    return;
                }
            }
            
            InitializeSoftBodyFromMesh();
        }
        
        private void RestartSimulation()
        {
           // CreateCube();
           InitializeSoftBodyFromMesh();
        }

        public void InitializeSoftBodyFromMesh() 
        {
            _particles.Clear();
            _distanceConstraints.Clear();

            if (sourceMesh == null)
            {
                Debug.LogError("Source Mesh is null in InitializeSoftBodyFromMesh.");
                return;
            }

            // 1. Instantiate the display mesh and get its initial vertices
            _displayMesh = Instantiate(sourceMesh); 
            
            GetComponent<MeshFilter>().mesh = _displayMesh;
            _initialMeshVerticesLocal = (Vector3[])_displayMesh.vertices.Clone(); 
            _currentMeshVerticesLocal = new Vector3[_initialMeshVerticesLocal.Length];

            // 2. Create Particles from Mesh Vertices
            for (var i = 0; i < _initialMeshVerticesLocal.Length; i++)
            {
                var worldPos = transform.TransformPoint(_initialMeshVerticesLocal[i]);
                _particles.Add(new SoftBodyParticle(worldPos, particleMass, i));
            }
            Debug.Log($"Initialized {_particles.Count} particles from mesh vertices.");

            // 3. Generate Distance Constraints from Mesh Edges
            GenerateDistanceConstraintsFromEdges();
            
        }
        
     
        private void GenerateDistanceConstraintsFromEdges()
        {
            if (_displayMesh == null || _particles.Count == 0) return;

            var edges = new HashSet<System.Tuple<int, int>>();
            var triangles = _displayMesh.triangles;

            for (var i = 0; i < triangles.Length; i += 3)
            {
                var v1Index = triangles[i];
                var v2Index = triangles[i + 1];
                var v3Index = triangles[i + 2];

                AddEdge(edges, v1Index, v2Index);
                AddEdge(edges, v2Index, v3Index);
                AddEdge(edges, v3Index, v1Index);
            }

            foreach (var edge in edges)
            {
                var p1 = _particles[edge.Item1];
                var p2 = _particles[edge.Item2];
                var restLength = Vector3.Distance(p1.Position, p2.Position); // Calculated from initial world positions
                // Ensure AddDistanceConstraint can take a specific restLength
                _distanceConstraints.Add(new DistanceConstraint(p1, p2, restLength, globalCompliance));
            }
            Debug.Log($"Generated {_distanceConstraints.Count} distance constraints from mesh edges.");
        }

        // Helper to add unique edges (treats (a,b) and (b,a) as the same)
        private static void AddEdge(HashSet<System.Tuple<int, int>> edges, int index1, int index2)
        {
            var edge = System.Tuple.Create(Mathf.Min(index1, index2), Mathf.Max(index1, index2));
            edges.Add(edge);
        }
        
        private void AddDistanceConstraint(int particleAIndex, int particleBIndex, float compliance)
        {
            var particleA = _particles[particleAIndex];
            var particleB = _particles[particleBIndex];
            var restLength = Vector3.Distance(particleA.Position, particleB.Position);
            _distanceConstraints.Add(new DistanceConstraint(particleA, particleB, restLength, compliance));
        }


        private void FixedUpdate()
        {
            if (_particles.Count == 0)
            {
                return;
            }
            
            var dt = Time.fixedDeltaTime;
            if (dt <= 0)
            {
                return;
            }
            
            // Reset all constraints' lambda values
            foreach (var constraint in _distanceConstraints)
            {
                constraint.ResetLambda();
            }

            // 1. Apply external forces and predict positions
            foreach (var p in _particles)
            {
                p.ClearForces();
                if (p.InverseMass > 0) { p.AddForce(gravity); }
                p.Velocity += dt * p.InverseMass * p.ExternalForceAccumulator;
                p.Velocity *= (1f - Mathf.Clamp01(damping));
                p.PredictedPosition = p.Position + dt * p.Velocity;
            }

            // Reset lambdas
            foreach (var constraint in _distanceConstraints) { constraint.ResetLambda(); }

            // Solver iterations
            for (var iter = 0; iter < solverIterations; iter++)
            {
                foreach (var constraint in _distanceConstraints) { constraint.Solve(dt); }
                // foreach (var constraint in bendingConstraints) { constraint.Solve(dt); } // If implemented
                SolveCollisions(dt);
            }

            // Update velocities and positions
            foreach (var p in _particles)
            {
                if (p.InverseMass == 0) {
                    p.Velocity = Vector3.zero;
                    p.PredictedPosition = p.Position;
                    continue;
                }
                p.Velocity = (p.PredictedPosition - p.Position) / dt;
                p.Position = p.PredictedPosition;
            }
        }

        private void LateUpdate()
        {
            if (_displayMesh is null || _particles.Count == 0 || _currentMeshVerticesLocal == null || _currentMeshVerticesLocal.Length != _particles.Count)
            {
                if (_particles.Count > 0 && _displayMesh is null && _currentMeshVerticesLocal is null) {
                    _currentMeshVerticesLocal = new Vector3[_particles.Count]; 
                } else {
                    return; 
                }
            }

            for (var i = 0; i < _particles.Count; i++)
            {
                _currentMeshVerticesLocal[i] = transform.InverseTransformPoint(_particles[i].Position);
            }
            _displayMesh!.vertices = _currentMeshVerticesLocal;

            _displayMesh.RecalculateNormals(); // Essential for lighting on deformed mesh
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
                if (!Application.isPlaying || _particles == null || _particles.Count == 0) return;

                Gizmos.color = Color.yellow;
                foreach (var p in _particles)
                {
                    Gizmos.DrawSphere(p.Position, 0.02f); // Adjust radius as needed
                }

                // Optionally draw constraints (can be very slow for complex meshes)

                if (showConstraints)
                {
                    Gizmos.color = Color.cyan;
                    foreach (var constraint in _distanceConstraints)
                    {
                        if (constraint.ParticleA != null && constraint.ParticleB != null)
                        {
                            Gizmos.DrawLine(constraint.ParticleA.Position, constraint.ParticleA.Position);
                        }
                    }
                }
               

                Gizmos.color = Color.green;
                Gizmos.DrawLine(new Vector3(-5, groundHeight, 0) + transform.position, new Vector3(5, groundHeight, 0) + transform.position);
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
