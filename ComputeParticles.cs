using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class ComputeParticles : MonoBehaviour
{
    public Mesh mesh;
    public Material particleMaterial;
    public float cell_size = 2;
    public float particle_radius = 1;
    public Vector2Int grid_res = new Vector2Int(16, 16);
    public Vector2 gravity;
    public float resistence = 100;
    public float attraction;
    public float drag;
    public int maxParticles = 16;
    public ComputeShader computeShader;

    struct particle
    {
        public Vector2 x;
        public Vector2 v;
        public Vector2 a;
        public float radius;
        public float dummy; //for 128bit alignment
    };
    ComputeBuffer buffer_particles;
    particle[] array_particles;
    ComputeBuffer buffer_indirect_args;
    ComputeBuffer buffer_instances;
    public RenderTexture buffer_grid_ptr;
    public RenderTexture buffer_debug_tex;

    uint[] array_indirect_args;
    int kernel_Verlet;
    int kernel_CleanGrid;
    int kernel_P2G;
    int kernel_WriteInstanceBuffer;
    int kernel_Interaction;

    void Start()
    {
        Debug.Log(SystemInfo.supportedRandomWriteTargetCount);
        Debug.Log(SystemInfo.SupportsRenderTextureFormat(RenderTextureFormat.ARGBInt));
        buffer_particles = new ComputeBuffer(maxParticles, 32);
        buffer_instances = new ComputeBuffer(maxParticles, 48);
        buffer_grid_ptr = new RenderTexture(grid_res.x, grid_res.y, 1, RenderTextureFormat.ARGBInt);
        buffer_grid_ptr.enableRandomWrite = true;
        buffer_grid_ptr.Create();
        buffer_debug_tex = new RenderTexture(grid_res.x, grid_res.y, 1, RenderTextureFormat.ARGBFloat);
        buffer_debug_tex.enableRandomWrite = true;
        buffer_debug_tex.Create();

        kernel_Verlet = computeShader.FindKernel("Verlet");
        computeShader.SetBuffer(kernel_Verlet, "particles", buffer_particles);

        kernel_CleanGrid = computeShader.FindKernel("CleanGrid");
        computeShader.SetTexture(kernel_CleanGrid, "grid_ptr", buffer_grid_ptr);
        computeShader.SetTexture(kernel_CleanGrid, "debug_tex", buffer_debug_tex);

        kernel_P2G = computeShader.FindKernel("P2G");
        computeShader.SetBuffer(kernel_P2G, "particles", buffer_particles);
        computeShader.SetTexture(kernel_P2G, "grid_ptr", buffer_grid_ptr);
        computeShader.SetTexture(kernel_P2G, "debug_tex", buffer_debug_tex);

        kernel_Interaction = computeShader.FindKernel("Interaction");
        computeShader.SetBuffer(kernel_Interaction, "particles", buffer_particles);
        computeShader.SetTexture(kernel_Interaction, "grid_ptr", buffer_grid_ptr);
        computeShader.SetTexture(kernel_Interaction, "debug_tex", buffer_debug_tex);

        kernel_WriteInstanceBuffer = computeShader.FindKernel("WriteInstanceBuffer");
        computeShader.SetBuffer(kernel_WriteInstanceBuffer, "particles", buffer_particles);
        computeShader.SetBuffer(kernel_WriteInstanceBuffer, "instances", buffer_instances);
        
        computeShader.SetFloat("dt", Time.fixedDeltaTime);
        computeShader.SetVector("bounds_size", new Vector4(cell_size*grid_res.x,cell_size*grid_res.y,0,0));
        computeShader.SetVector("bounds_min", -new Vector4(cell_size * grid_res.x, cell_size * grid_res.y, 0, 0)/2);
        computeShader.SetInts("grid_res", new int[] { grid_res.x, grid_res.y });
        computeShader.SetInt("maxParticles", maxParticles);

        array_particles = new particle[maxParticles];
        for(int i = 0; i < maxParticles; ++i)
        {
            array_particles[i].x.x = (Random.value-.5f) * cell_size * grid_res.x;
            array_particles[i].x.y = (Random.value - .5f) * cell_size * grid_res.x;
            array_particles[i].v = Random.insideUnitCircle.normalized*5f;
            array_particles[i].radius = particle_radius;
        }
        buffer_particles.SetData(array_particles);

        
        array_indirect_args = new uint[5] { 0, 0, 0, 0, 0 };
        buffer_indirect_args = new ComputeBuffer(1, 20, ComputeBufferType.IndirectArguments);
        array_indirect_args[0] = mesh.GetIndexCount(0);
        array_indirect_args[1] = (uint)maxParticles;
        buffer_indirect_args.SetData(array_indirect_args);
        particleMaterial.SetBuffer("instances", buffer_instances);
    }
    void OnDisable()
    {
        if (buffer_particles != null) buffer_particles.Release();
        if (buffer_instances != null) buffer_instances.Release();
        if (buffer_indirect_args != null) buffer_indirect_args.Release();
        if (buffer_grid_ptr != null) buffer_grid_ptr.Release();
    }
    
    void Update()
    {
        computeShader.SetVector("parent_pos", transform.position);
        computeShader.SetVector("parent_rot", new Vector4(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w));
        computeShader.SetFloat("parent_scale", (transform.lossyScale.x + transform.lossyScale.y + transform.lossyScale.z) / 3);
        computeShader.Dispatch(kernel_WriteInstanceBuffer, Mathf.CeilToInt(maxParticles / 128f), 1, 1);
        Graphics.DrawMeshInstancedIndirect(mesh, 0, particleMaterial, new Bounds(Vector3.zero, Vector3.positiveInfinity), buffer_indirect_args);
    }
    void FixedUpdate()
    {
        computeShader.SetFloat("resistence", resistence);
        computeShader.SetFloat("attraction", attraction);
        computeShader.SetVector("gravity", gravity);
        computeShader.SetFloat("drag", drag);

        computeShader.Dispatch(kernel_Verlet, Mathf.CeilToInt(maxParticles / 128f), 1, 1);
        computeShader.Dispatch(kernel_CleanGrid, Mathf.CeilToInt(grid_res.x/16f), Mathf.CeilToInt(grid_res.y / 16f), 1);
        for(int p2g_phase = 0; p2g_phase < 4; ++p2g_phase)
        {
            computeShader.SetInt("p2g_phase", p2g_phase);
            computeShader.Dispatch(kernel_P2G, Mathf.CeilToInt(maxParticles / 128f), 1, 1);
        }
        computeShader.Dispatch(kernel_Interaction, Mathf.CeilToInt(maxParticles / 128f), 1, 1);
        
    }
}
