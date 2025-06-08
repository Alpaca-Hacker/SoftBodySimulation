using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Codex.Input;
using SoftBody.Scripts.Models;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class SoftBodyGPU : MonoBehaviour
{

    // --- Shader Property IDs (for performance) ---
    private static readonly int Gravity = Shader.PropertyToID("_Gravity");
    private static readonly int GroundHeight = Shader.PropertyToID("_GroundHeight");
    private static readonly int Damping = Shader.PropertyToID("_Damping");
    private static readonly int DeltaTime = Shader.PropertyToID("_DeltaTime");
    private static readonly int ParticleCount = Shader.PropertyToID("_ParticleCount");
    private static readonly int ConstraintCount = Shader.PropertyToID("_ConstraintCount");
    private static readonly int DistanceCompliance = Shader.PropertyToID("_DistanceCompliance");
    private static readonly int Particles = Shader.PropertyToID("particles");
    private static readonly int Constraints = Shader.PropertyToID("constraints");
    private static readonly int LagrangeMultipliers = Shader.PropertyToID("lagrangeMultipliers");

    // --- Inspector Fields ---
    [Header("Compute Shader")]
    [SerializeField] private ComputeShader computeShader;

    [Header("Simulation Parameters")]
    [Range(1, 200)] public int substeps = 10;
    [Range(1, 10)] public int solverIterations = 3; // Inner loop for stiffness
    public Vector3 gravity = new(0, -9.81f, 0);
    [Range(0, 1f)] public float damping = 0.05f;

    [Header("Constraints")]
    [Tooltip("Compliance of the distance constraints. Lower is stiffer.")]
    public float distanceCompliance = 0.001f; // A value of 0 is stiff PBD

    [Header("Collision")]
    public float groundHeight = 0.0f;
    [Header("Debug")]
    [SerializeField] private InputReaderSO inputReader;

    // --- Private Member Variables ---
    private MeshFilter _meshFilter;
    private ComputeBuffer _particleBuffer, _distanceConstraintBuffer, _lagrangeBuffer;
    private Particle[] _particles; // Used for initialization and mesh updates
    private List<DistanceConstraint> _distanceConstraints;
    private float[] _zeroData; // For clearing the Lagrange buffer
    private int _particleCount, _constraintCount;
    private int _predictKernel, _solveDistancesKernel, _updateStateKernel, _dampingKernel;
    private Vector3[] _verticesForMeshUpdate; 
    private Particle[] _initialParticleState;

    private void OnEnable()
    {
        inputReader.Test1PressedEvent += RestartSimulation;
    }
        
    private void OnDisable()
    {
        inputReader.Test1PressedEvent -= RestartSimulation;
    }

    private void Awake()
    {
        _meshFilter = GetComponent<MeshFilter>();
    }

    private void Start()
    {
        InitializeData();
        
        _verticesForMeshUpdate = new Vector3[_particleCount];

        // --- Create GPU Buffers ---
        _particleBuffer = new ComputeBuffer(_particleCount, Marshal.SizeOf(typeof(Particle)));
        _particleBuffer.SetData(_particles);

        _distanceConstraintBuffer = new ComputeBuffer(_constraintCount, Marshal.SizeOf(typeof(DistanceConstraint)));
        _distanceConstraintBuffer.SetData(_distanceConstraints);

        _lagrangeBuffer = new ComputeBuffer(_constraintCount, sizeof(float));
        _zeroData = new float[_constraintCount];
        _lagrangeBuffer.SetData(_zeroData);

        // --- Find Kernel Indices ---
        _predictKernel = computeShader.FindKernel("PredictPositions");
        _solveDistancesKernel = computeShader.FindKernel("SolveDistances");
        _updateStateKernel = computeShader.FindKernel("UpdateState");
        _dampingKernel = computeShader.FindKernel("ApplyDamping");

        // --- Link Buffers to Kernels ---
        computeShader.SetBuffer(_predictKernel, Particles, _particleBuffer);
        
        computeShader.SetBuffer(_solveDistancesKernel, Particles, _particleBuffer);
        computeShader.SetBuffer(_solveDistancesKernel, Constraints, _distanceConstraintBuffer);
        computeShader.SetBuffer(_solveDistancesKernel, LagrangeMultipliers, _lagrangeBuffer);

        computeShader.SetBuffer(_updateStateKernel, Particles, _particleBuffer);
        
        computeShader.SetBuffer(_dampingKernel, Particles, _particleBuffer);
        
        _initialParticleState = new Particle[_particleCount];
        Array.Copy(_particles, _initialParticleState, _particleCount);
    }

    private void RestartSimulation()
    {
        if (_initialParticleState == null || _particleBuffer == null)
        {
            Debug.LogWarning("Simulation cannot be restarted before it has been initialized.");
            return;
        }

        Debug.Log("Restarting simulation (Optimized)...");

        Array.Copy(_initialParticleState, _particles, _particleCount);
        
        _particleBuffer.SetData(_particles);
        
        if (_lagrangeBuffer != null)
        {
            _lagrangeBuffer.SetData(_zeroData);
        }
    }

    private void FixedUpdate()
    {
        if (_particleBuffer == null) return;

        var substepDeltaTime = Time.fixedDeltaTime / substeps;
        var threadGroups = Mathf.CeilToInt(_particleCount / 64.0f);
        var constraintThreadGroups = Mathf.CeilToInt(_constraintCount / 64.0f);

        // --- Set Uniforms ---
        computeShader.SetInt(ParticleCount, _particleCount);
        computeShader.SetInt(ConstraintCount, _constraintCount);
        computeShader.SetFloat(DistanceCompliance, distanceCompliance);
        computeShader.SetVector(Gravity, gravity);
        computeShader.SetFloat(GroundHeight, groundHeight);
        computeShader.SetFloat(Damping, damping);
        computeShader.SetFloat(DeltaTime, substepDeltaTime);

        // --- Full GPU Simulation Loop ---
        for (var i = 0; i < substeps; i++)
        {
            _lagrangeBuffer.SetData(_zeroData);
            computeShader.Dispatch(_predictKernel, threadGroups, 1, 1);
            
            for (var j = 0; j < solverIterations; j++)
            {
                computeShader.Dispatch(_solveDistancesKernel, constraintThreadGroups, 1, 1);
            }

            computeShader.Dispatch(_updateStateKernel, threadGroups, 1, 1);
        }

        // --- Apply Damping & Visualize ---
        computeShader.Dispatch(_dampingKernel, threadGroups, 1, 1);
        _particleBuffer.GetData(_particles); 

        // 2. Process the data and fill our dedicated mesh vertices array
        for (var i = 0; i < _particleCount; i++)
        {
            _verticesForMeshUpdate[i] = transform.InverseTransformPoint(_particles[i].position);
        }
        
        // 3. Assign the ENTIRE modified array back to the mesh. This is the fix.
        _meshFilter.mesh.vertices = _verticesForMeshUpdate;
        
        // 4. Recalculate normals and bounds for correct rendering.
        _meshFilter.mesh.RecalculateBounds();
        _meshFilter.mesh.RecalculateNormals();
    }

    private void OnDestroy()
    {
        if (_particleBuffer != null) _particleBuffer.Release();
        if (_distanceConstraintBuffer != null) _distanceConstraintBuffer.Release();
        if (_lagrangeBuffer != null) _lagrangeBuffer.Release();
    }

    // --- Initialization Logic ---
    private void InitializeData()
    {
        var initialVertices = _meshFilter.mesh.vertices;
        _particleCount = initialVertices.Length;

        _particles = new Particle[_particleCount];
        for (var i = 0; i < _particleCount; i++)
        {
            _particles[i] = new Particle {
                position = transform.TransformPoint(initialVertices[i]),
                predictedPosition = transform.TransformPoint(initialVertices[i]),
                velocity = Vector3.zero,
                inverseMass = 1.0f
            };
        }

        var triangles = _meshFilter.mesh.triangles;
        var edges = new HashSet<Tuple<int, int>>();
        _distanceConstraints = new List<DistanceConstraint>();
        for (var i = 0; i < triangles.Length; i += 3)
        {
            AddEdge(triangles[i], triangles[i + 1], edges);
            AddEdge(triangles[i + 1], triangles[i + 2], edges);
            AddEdge(triangles[i + 2], triangles[i], edges);
        }
        _constraintCount = _distanceConstraints.Count;
    }

    private void AddEdge(int p1, int p2, HashSet<Tuple<int, int>> edges)
    {
        var edge = new Tuple<int, int>(Mathf.Min(p1, p2), Mathf.Max(p1, p2));
        if (edges.Add(edge))
        {
            _distanceConstraints.Add(new DistanceConstraint {
                p1 = p1,
                p2 = p2,
                restLength = Vector3.Distance(_particles[p1].position, _particles[p2].position)
            });
        }
    }
}