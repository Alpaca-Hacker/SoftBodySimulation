using System;
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
        
        [Header("Constraint Clamping")]
        [SerializeField] private float maxLambdaChangeDist = 0.001f;
        [SerializeField] private float maxLambdaChangeBend = 0.001f;
        
        [Header("Bending Settings")] 
        [SerializeField] private bool useBendingConstraints = true;
        [SerializeField] private float bendingCompliance = 0.1f;

        [Header("Mesh Settings")] [SerializeField]
        private Mesh sourceMesh;

        [SerializeField] private float particleMass = 1.0f;

        [Header("Collision")] [SerializeField] private float groundHeight = 0.0f;
        [SerializeField] private float collisionCompliance = 0.0f; // For collision constraints (0 = rigid)
        [SerializeField] private float friction = 0.1f; // Simple friction for collision response

        [Header("Debug")] 
        [SerializeField] private InputReaderSO inputReader;
        [SerializeField] private bool showConstraints;
        [Header("Test Configurations")]
        public InitializationMode currentInitializationMode = InitializationMode.FromSourceMesh;

        private List<SoftBodyParticleCPU> _particles = new();
        private List<CPUDistanceConstraint> _distanceConstraints = new();
        private List<CPUBendingConstraint> _bendingConstraints = new();
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
            switch (currentInitializationMode)
            {
                case InitializationMode.FromSourceMesh:
                {
                    if (sourceMesh == null)
                    {
                        var mf = GetComponent<MeshFilter>();
                        if (mf != null && mf.sharedMesh != null)
                        {
                            sourceMesh = mf.sharedMesh; // Try to get from MeshFilter if not assigned
                            Debug.Log("Used mesh from MeshFilter.");
                        }
                        else
                        {
                            Debug.LogError(
                                "Source Mesh is not assigned and no mesh found in MeshFilter. Cannot initialize soft body.");
                            enabled = false; // Disable the script
                            return;
                        }
                    }
                    InitializeSoftBodyFromMesh();
                    break;
                }
                case InitializationMode.TestDistance_2Particles:
                {
                    CreateTest_Distance_2Particles();
                    break;
                }
                case InitializationMode.TestDistance_3Particles_Line:
                {
                    CreateTest_Distance_3Particles_Line();
                    break;
                }
                case InitializationMode.TestDistance_3Particles_Triangle:
                {
                    CreateTest_Distance_3Particles_Triangle();
                    break;
                }
                case InitializationMode.TestBending_4Particles_FlatButterfly:
                {
                    CreateTest_Bending_4Particles_FlatButterfly();
                    break;
                }
            }


            Debug.Log("Global compliance: " + globalCompliance);
            Debug.Log("Bending compliance: " + bendingCompliance);
            Debug.Log("Particle mass: " + particleMass);
            Debug.Log("Gravity: " + gravity);
            Debug.Log("Damping: " + damping);
            
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
            _bendingConstraints.Clear();

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
                _particles.Add(new SoftBodyParticleCPU(worldPos, particleMass, i));
            }

            Debug.Log($"Initialized {_particles.Count} particles from mesh vertices.");

            // 3. Generate Distance Constraints from Mesh Edges
            GenerateDistanceConstraintsFromEdges();

            if (useBendingConstraints)
            {
                GenerateBendingConstraints();
            }

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
            var counter = 0;
            foreach (var edge in edges)
            {
                var p1 = _particles[edge.Item1];
                var p2 = _particles[edge.Item2];
                var restLength = Vector3.Distance(p1.Position, p2.Position); // Calculated from initial world positions
                // Ensure AddDistanceConstraint can take a specific restLength
                var debugMode = counter % 1000 == 0;
                if (debugMode)
                {
                    Debug.Log("Added debug mode for constraint between ParticleA(" + p1.Id + ") and ParticleB(" + p2.Id + ")");
                }// Optional debug mode for every 1000th constraint
                _distanceConstraints.Add(new CPUDistanceConstraint(p1, p2, restLength, globalCompliance, maxLambdaChangeDist, debugMode));
                counter++;
            }

            Debug.Log($"Generated {_distanceConstraints.Count} distance constraints from mesh edges.");
        }

        // Helper to add unique edges (treats (a,b) and (b,a) as the same)
        private static void AddEdge(HashSet<System.Tuple<int, int>> edges, int index1, int index2)
        {
            var edge = System.Tuple.Create(Mathf.Min(index1, index2), Mathf.Max(index1, index2));
            edges.Add(edge);
        }

        private void GenerateBendingConstraints()
        {
            if (_displayMesh == null || _particles.Count == 0)
            {
                return;
            }


    // Map: edge (represented by Tuple<int,int> of sorted vertex indices) -> list of triangles (third vertex) sharing this edge
    var edgeToTrianglesMap = new Dictionary<Tuple<int, int>, List<EdgeTriangleInfo>>();
    var triangles = _displayMesh.triangles;

    for (var i = 0; i < triangles.Length; i += 3)
    {
        var v0 = triangles[i];
        var v1 = triangles[i + 1];
        var v2 = triangles[i + 2];

        // Add info for each edge of the triangle
        void AddEdgeInfo(int pA, int pB, int pOther)
        {
            var edgeKey = Tuple.Create(Mathf.Min(pA, pB), Mathf.Max(pA, pB));
            if (!edgeToTrianglesMap.ContainsKey(edgeKey))
            {
                edgeToTrianglesMap[edgeKey] = new List<EdgeTriangleInfo>();
            }

            edgeToTrianglesMap[edgeKey].Add(new EdgeTriangleInfo(pOther));
        }
    
        AddEdgeInfo(v0, v1, v2);
        AddEdgeInfo(v1, v2, v0);
        AddEdgeInfo(v2, v0, v1);
    }

    var debug = true;
    foreach (var entry in edgeToTrianglesMap)
    {
        var edge = entry.Key;
        var trisInfo = entry.Value;

        // We need exactly two triangles sharing an edge for a bending constraint
       
        if (trisInfo.Count == 2)
        {
            var particleA = _particles[edge.Item1]; // Edge vertex 1
            var particleB = _particles[edge.Item2]; // Edge vertex 2
            var particleC = _particles[trisInfo[0].ThirdVertexIndex]; // Tip of triangle 1
            var particleD = _particles[trisInfo[1].ThirdVertexIndex]; // Tip of triangle 2

            // Check for degenerate case (e.g. p2 or p3 is same as p0 or p1, though unlikely with good mesh)
            if (particleC == particleA || particleC == particleB || particleD == particleA || particleD == particleB || particleC == particleD) continue;

            var restAngle = CPUBendingConstraint.CalculateRestAngle(particleA.Position, particleB.Position, particleC.Position, particleD.Position);
            
            _bendingConstraints.Add(new CPUBendingConstraint(particleA, particleB, particleC, particleD, restAngle, bendingCompliance, debug, maxLambdaChangeDist));
            
            debug = false;
        }
    }

    Debug.Log($"Generated {_bendingConstraints.Count} bending constraints.");

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
            foreach (var constraint in _bendingConstraints)
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
                foreach (var constraint in _bendingConstraints)  { constraint.Solve(dt); } 
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
            if (currentInitializationMode != InitializationMode.FromSourceMesh)
            {
                _displayMesh = null;
            }

            if (_displayMesh is null || _particles.Count == 0 || _currentMeshVerticesLocal == null || 
                _currentMeshVerticesLocal.Length != _displayMesh.vertexCount || // Ensure array matches actual mesh vertex count
                _particles.Count != _displayMesh.vertexCount) // Ensure particles match mesh vertices
            {
                // Debug.LogWarning("LateUpdate: Mismatch or not ready to update mesh.");
                return; 
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
            if (!Application.isPlaying && sourceMesh != null && (_particles == null || _particles.Count == 0) )
            {
                // Draw a preview of the source mesh if not playing and particles not initialized
                Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.3f);
                if (GetComponent<MeshFilter>().sharedMesh == sourceMesh) // To avoid drawing if a different mesh is in filter
                {
                    Gizmos.DrawMesh(sourceMesh, transform.position, transform.rotation, transform.localScale);
                }
                Gizmos.color = Color.green;
                Gizmos.DrawLine(new Vector3(-5, groundHeight, 0) + transform.position, new Vector3(5, groundHeight, 0) + transform.position);
                return;
            }

            if (_particles == null || _particles.Count == 0)
            {
                return; // Nothing to draw if particles aren't ready
            }

            // Draw particles
            Gizmos.color = Color.yellow;
            var particleGizmoRadius = 0.03f; // Use a consistent radius
            foreach (var p in _particles)
            {
                Gizmos.DrawSphere(p.Position, particleGizmoRadius);
            }

            // Draw distance constraints
            if (showConstraints && _distanceConstraints != null)
            {
                Gizmos.color = Color.cyan;
                foreach (var constraint in _distanceConstraints)
                {
                    if (constraint.ParticleCPUA != null && constraint.ParticleCPUB != null)
                    {
                        Gizmos.DrawLine(constraint.ParticleCPUA.Position, constraint.ParticleCPUB.Position);
                    }
                }
            }
            
            /*
            if (showConstraints && useBendingConstraints && _bendingConstraints != null)
            {
                Gizmos.color = Color.magenta;
                foreach (var bc in _bendingConstraints)
                {
                    if (bc.ParticleA != null && bc.ParticleB != null && bc.ParticleC != null && bc.ParticleD != null)
                    {
                        Vector3 centerHinge = (bc.ParticleA.Position + bc.ParticleB.Position) * 0.5f;
                        Gizmos.DrawLine(centerHinge, bc.ParticleC.Position);
                        Gizmos.DrawLine(centerHinge, bc.ParticleD.Position);
                    }
                }
            }
            */
            // Draw ground
            Gizmos.color = Color.green;
            // Ensure ground line is drawn relative to world, not just local (unless groundHeight is world space)
            Gizmos.DrawLine(new Vector3(-5, groundHeight, 0), new Vector3(5, groundHeight, 0));
        }
        
        
        // Test configurations for quick setups
        
        private void CreateTest_Distance_2Particles()
        {
            _particles.Clear();
            _distanceConstraints.Clear();
            _bendingConstraints.Clear(); // Ensure this is empty
            useBendingConstraints = false; // Explicitly disable for this test

            // Particle positions (world space)
            var pos0 = transform.TransformPoint(new Vector3(0, 0, 0)); // Use GameObject's transform as origin
            var pos1 = transform.TransformPoint(new Vector3(1, 0, 0));

            // Create particles
            _particles.Add(new SoftBodyParticleCPU(pos0, particleMass, 0));
            _particles.Add(new SoftBodyParticleCPU(pos1, particleMass, 1));

            // Create distance constraint
            var restLength = Vector3.Distance(_particles[0].Position, _particles[1].Position);
            // Make sure 'globalCompliance' is the variable you tune in the inspector
            _distanceConstraints.Add(new CPUDistanceConstraint(_particles[0], _particles[1], restLength, globalCompliance, maxLambdaChangeDist));

            Debug.Log("Created Test: 2 Particles, 1 Distance Constraint.");
            // For this test, DistanceConstraint.Solve() should NOT have internal clamping on deltaLambda yet.
        }
        
        private void CreateTest_Distance_3Particles_Line()
        {
            _particles.Clear();
            _distanceConstraints.Clear();
            _bendingConstraints.Clear();
            useBendingConstraints = false;

            var pos0 = transform.TransformPoint(new Vector3(0, 0, 0));
            var pos1 = transform.TransformPoint(new Vector3(1, 0, 0));
            var pos2 = transform.TransformPoint(new Vector3(2, 0, 0));

            _particles.Add(new SoftBodyParticleCPU(pos0, particleMass, 0));
            _particles.Add(new SoftBodyParticleCPU(pos1, particleMass, 1));
            _particles.Add(new SoftBodyParticleCPU(pos2, particleMass, 2));

            var restLength1 = Vector3.Distance(_particles[0].Position, _particles[1].Position);
            _distanceConstraints.Add(new CPUDistanceConstraint(_particles[0], _particles[1], restLength1, globalCompliance, maxLambdaChangeDist));

            var restLength2 = Vector3.Distance(_particles[1].Position, _particles[2].Position);
            _distanceConstraints.Add(new CPUDistanceConstraint(_particles[1], _particles[2], restLength2, globalCompliance, maxLambdaChangeDist));

            Debug.Log("Created Test: 3 Particles (Line), 2 Distance Constraints.");
        }
        
        private void CreateTest_Distance_3Particles_Triangle()
        {
            _particles.Clear();
            _distanceConstraints.Clear();
            _bendingConstraints.Clear();
            useBendingConstraints = false;

            var pos0 = transform.TransformPoint(new Vector3(0, 0, 0));
            var pos1 = transform.TransformPoint(new Vector3(1, 0, 0));
            var pos2 = transform.TransformPoint(new Vector3(0.5f, Mathf.Sqrt(0.75f), 0)); // Approx 0.866 for equilateral

            _particles.Add(new SoftBodyParticleCPU(pos0, particleMass, 0));
            _particles.Add(new SoftBodyParticleCPU(pos1, particleMass, 1));
            _particles.Add(new SoftBodyParticleCPU(pos2, particleMass, 2));

            var r01 = Vector3.Distance(_particles[0].Position, _particles[1].Position);
            _distanceConstraints.Add(new CPUDistanceConstraint(_particles[0], _particles[1], r01, globalCompliance, maxLambdaChangeDist));

            var r12 = Vector3.Distance(_particles[1].Position, _particles[2].Position);
            _distanceConstraints.Add(new CPUDistanceConstraint(_particles[1], _particles[2], r12, globalCompliance, maxLambdaChangeDist));

            var r20 = Vector3.Distance(_particles[2].Position, _particles[0].Position);
            _distanceConstraints.Add(new CPUDistanceConstraint(_particles[2], _particles[0], r20, globalCompliance, maxLambdaChangeDist));

            Debug.Log("Created Test: 3 Particles (Triangle), 3 Distance Constraints.");
        }
        
        private void CreateTest_Bending_4Particles_FlatButterfly()
{
    _particles.Clear();
    _distanceConstraints.Clear();
    _bendingConstraints.Clear();
    useBendingConstraints = true; // Enable bending for this test

    // Define particles for the butterfly shape (hinge P0-P1)
    // Ensure local positions are used for transform.TransformPoint
    var p0_local = new Vector3(0, 0, 0);
    var p1_local = new Vector3(1, 0, 0);
    var p2_local = new Vector3(0.5f, 0.5f, 0); // Tip of triangle 1
    var p3_local = new Vector3(0.5f, -0.5f, 0); // Tip of triangle 2
                                                  // Initial dihedral angle should be PI (180 degrees)

    // Optionally, perturb one particle slightly to see correction:
    // p2_local = new Vector3(0.5f, 0.4f, 0.1f); 

    _particles.Add(new SoftBodyParticleCPU(transform.TransformPoint(p0_local), particleMass, 0)); // P0
    _particles.Add(new SoftBodyParticleCPU(transform.TransformPoint(p1_local), particleMass, 1)); // P1
    _particles.Add(new SoftBodyParticleCPU(transform.TransformPoint(p2_local), particleMass, 2)); // P2 (ParticleC in your BendingConstraint)
    _particles.Add(new SoftBodyParticleCPU(transform.TransformPoint(p3_local), particleMass, 3)); // P3 (ParticleD in your BendingConstraint)

    var sbp0 = _particles[0];
    var sbp1 = _particles[1];
    var sbp2 = _particles[2];
    var sbp3 = _particles[3];

    // Add essential distance constraints to hold the triangles' shapes
    // Use a stiffer compliance for distance constraints in this specific test to isolate bending behavior
    var distanceTestCompliance = 0.00001f; // Make these fairly stiff
    _distanceConstraints.Add(new CPUDistanceConstraint(sbp0, sbp1, Vector3.Distance(sbp0.Position, sbp1.Position), distanceTestCompliance, maxLambdaChangeDist)); // Hinge
    _distanceConstraints.Add(new CPUDistanceConstraint(sbp0, sbp2, Vector3.Distance(sbp0.Position, sbp2.Position), distanceTestCompliance, maxLambdaChangeDist));
    _distanceConstraints.Add(new CPUDistanceConstraint(sbp1, sbp2, Vector3.Distance(sbp1.Position, sbp2.Position), distanceTestCompliance, maxLambdaChangeDist));
    _distanceConstraints.Add(new CPUDistanceConstraint(sbp0, sbp3, Vector3.Distance(sbp0.Position, sbp3.Position), distanceTestCompliance, maxLambdaChangeDist));
    _distanceConstraints.Add(new CPUDistanceConstraint(sbp1, sbp3, Vector3.Distance(sbp1.Position, sbp3.Position), distanceTestCompliance, maxLambdaChangeDist));
    // Optionally, a constraint between P2 and P3 if they should not pass through each other,
    // but not strictly needed for the bending test itself.
    // _distanceConstraints.Add(new DistanceConstraint(sbp2, sbp3, Vector3.Distance(sbp2.Position, sbp3.Position), distanceTestCompliance));


    // Add THE bending constraint
    // The constructor order: ParticleA, ParticleB (hinge), ParticleC (tip1), ParticleD (tip2)
    var restAngle = CPUBendingConstraint.CalculateRestAngle(sbp0.Position, sbp1.Position, sbp2.Position, sbp3.Position);
    Debug.Log($"Calculated Rest Angle for Butterfly Test: {restAngle * Mathf.Rad2Deg} degrees");

    // Make sure BendingConstraint constructor matches:
    // (pA, pB, pC, pD, angle, compliance, isDebug, maxLambdaChangeVal)
    // The 'maxLambdaChange' from SoftBodyCPU field will be passed.
    _bendingConstraints.Add(new CPUBendingConstraint(sbp0, sbp1, sbp2, sbp3, restAngle, bendingCompliance, true, maxLambdaChangeBend)); // true for isDebugConstraint

    Debug.Log("Created Test: 4 Particles (Butterfly), 5-6 Distance Constraints, 1 Bending Constraint.");
    Debug.Log($"Bending constraint will use maxLambdaChange: {maxLambdaChangeBend} from SoftBodyCPU inspector.");
}
        
        
        
        // Helper class to store triangle info for an edge
        // Stores the third vertex index that forms a triangle with the edge
        
        private class EdgeTriangleInfo
        {

            public readonly int ThirdVertexIndex;

            // public Vector3 Normal; // Could store normal if needed for other things
            public EdgeTriangleInfo(int p2)
            {
                ThirdVertexIndex = p2;
            }
        }
    }
}
