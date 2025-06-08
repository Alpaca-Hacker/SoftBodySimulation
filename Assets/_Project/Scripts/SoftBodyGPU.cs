using System.Runtime.InteropServices;
using SoftBody.Scripts.Models;
using UnityEngine;

public class SoftBodyGPU : MonoBehaviour
{
    [SerializeField] private ComputeShader computeShader;
    [SerializeField] private MeshFilter meshFilter;
    
    private ComputeBuffer _particleBuffer;
    
    private Particle[] _particles;
    private Vector3[] _initialVertices;
    private Vector3[] _verticesForMeshUpdate; 
    private int _particleCount;
    private int _kernelIndex;

    private void Start()
    {
        Debug.Log("--- STARTING SIMULATION SETUP ---");
        _initialVertices= meshFilter.mesh.vertices;
        _particleCount = _initialVertices.Length;
        if (_particleCount == 0)
        {
            Debug.LogError("Mesh has no vertices! Aborting.", this);
            return;
        }
        
        _verticesForMeshUpdate = new Vector3[_particleCount];
        Debug.Log("Step 2: Initialized CPU particle array.");
        
        InitializeParticles();
        
        var stride = Marshal.SizeOf(typeof(Particle));
        
        _particleBuffer = new ComputeBuffer(_particleCount, stride);
        _particleBuffer.SetData(_particles);
        
        Debug.Log($"Step 3: Created ComputeBuffer with {_particleCount} elements and a stride of {stride} bytes.");
        
        if (computeShader == null)
        {
            Debug.LogError("Compute Shader is not assigned in the Inspector! Aborting.", this);
            return;
        }
        
        _kernelIndex = computeShader.FindKernel("CSMain");
        
        if (_kernelIndex < 0)
        {
            Debug.LogError("Could not find kernel 'CSMain' in the compute shader. Check the name.", this);
            return;
        }
        
        computeShader.SetBuffer(_kernelIndex, "particles", _particleBuffer);
        
        Debug.Log("Step 4: Linked buffer to compute shader kernel 'CSMain'.");
        
        Debug.Log("--- SETUP COMPLETE ---");
    }

    private void InitializeParticles()
    {
        _particles = new Particle[_particleCount];
        for (var i = 0; i < _particleCount; i++)
        {
            _particles[i] = new Particle
            {
                position = _initialVertices[i],
                predictedPosition = _initialVertices[i],
                velocity = Vector3.zero,
                inverseMass = 1f // 0 for static particles
            };
        }
    }
    
    private void Update()
    {
        if (_particleBuffer == null || _kernelIndex < 0)
        {
            return;
        }
        
        _kernelIndex = computeShader.FindKernel("CSMain");
        
        var threadGroups = Mathf.CeilToInt((float)_particleCount / 64.0f);
        
        computeShader.Dispatch(_kernelIndex, threadGroups, 1, 1);
        
        //Super slow but needed for testing
        _particleBuffer.GetData(_particles);
        
        for (var i = 0; i < _particleCount; i++)
        {
            _verticesForMeshUpdate[i] = _particles[i].position;
        }
        
        meshFilter.mesh.vertices = _verticesForMeshUpdate;
        meshFilter.mesh.RecalculateNormals();
    }

    private void OnDestroy()
    {
        if (_particleBuffer != null)
        {
            _particleBuffer.Release();
            _particleBuffer = null;
        }
    }
}
