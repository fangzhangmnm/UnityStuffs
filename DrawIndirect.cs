using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DrawIndirect : MonoBehaviour
{
    public Mesh particleMesh;
    public Material particleMaterial;
    public int maxParticles = 100;
    ComputeBuffer argsBuffer;

    void Start()
    {
        uint[] args = new uint[5] { 0, 0, 0, 0, 0 };
        argsBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
        args[0] = (uint)particleMesh.GetIndexCount(0);
        args[1] = (uint)maxParticles;
        args[2] = (uint)particleMesh.GetIndexStart(0);
        args[3] = (uint)particleMesh.GetBaseVertex(0);
        argsBuffer.SetData(args);
    }

    // Update is called once per frame
    void Update()
    {




        //ComputeBuffer buffer_particles;
        //particle[] array_particles;
        //array_particles = new particle[maxParticles];
        //buffer_particles = new ComputeBuffer(maxParticles, 32);
        //particleMaterial.SetBuffer("particles", buffer_particles);

        Bounds infBounds = new Bounds(Vector3.zero, Vector3.positiveInfinity);

        Graphics.DrawMeshInstancedIndirect(particleMesh, 0, particleMaterial, new Bounds(Vector3.zero, new Vector3(100.0f, 100.0f, 100.0f)), argsBuffer);
        
    }
    private void OnDisable()
    {
        argsBuffer.Release();
    }
}
