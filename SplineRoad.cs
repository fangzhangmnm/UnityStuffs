using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[ExecuteInEditMode]
[RequireComponent(typeof(Spline))]
[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class SplineRoad : MonoBehaviour
{
    Spline spline;
    MeshFilter filter;
    MeshCollider meshCollider;
    Mesh mesh;
    public int subSegments = 9;
    public float width = 1f;
    public float uvRepeatPerSegment = 2;
    public Vector3 bias = new Vector3(0, 0.01f, 0);
    
    private void Reset()
    {
        spline = GetComponent<Spline>();
        filter = GetComponent<MeshFilter>();
        mesh = filter.sharedMesh = new Mesh();
        meshCollider = GetComponent<MeshCollider>();
        if (meshCollider) meshCollider.sharedMesh = mesh;
        mesh.name = "RoadMesh";
    }
    public void Generate()
    {
        if (mesh == null)
        {
            spline = GetComponent<Spline>();
            filter = GetComponent<MeshFilter>();
            mesh = filter.sharedMesh = new Mesh();
            meshCollider = GetComponent<MeshCollider>();
            if (meshCollider) meshCollider.sharedMesh = mesh;
            mesh.name = "RoadMesh";
        }
        int n = subSegments * spline.segments;
        Vector3[] vertices = mesh.vertices != null && mesh.vertices.Length == n * 2+4 ? mesh.vertices : new Vector3[n * 2+4];
        Vector2[] uv = mesh.uv != null && mesh.uv.Length == n * 2+4 ? mesh.uv : new Vector2[n * 2+4];
        int[] triangles = mesh.triangles != null && mesh.triangles.Length == n*6? mesh.triangles : new int[n * 6];
        for (int i = 0; i <= n; ++i)
        {
            float t = (float)i / n;
            Vector3 p = spline.GetPoint(t) + bias.x * spline.GetNormalLocal(t, Vector3.up) + bias.y * Vector3.up + bias.z * spline.GetTangentLocal(t);
            Vector3 right = spline.GetNormalLocal(t, Vector3.up);
            vertices[2 * i] = p - right * width / 2;
            vertices[2 * i + 1] = p + right * width / 2;
            uv[2 * i] = new Vector2(0, uvRepeatPerSegment * i / subSegments);
            uv[2 * i + 1] = new Vector2(1, uvRepeatPerSegment * i / subSegments);
            if (i < n)
            {
                triangles[6 * i] = (2 * i + 2);
                triangles[6 * i + 1] = (2 * i + 1);
                triangles[6 * i + 2] = (2 * i + 0);
                triangles[6 * i + 3] = (2 * i + 2);
                triangles[6 * i + 4] = (2 * i + 3);
                triangles[6 * i + 5] = (2 * i + 1);
            }
        }
        vertices[n * 2 + 2] = spline.GetPointLocal(0);//for [v] alignment use
        vertices[n * 2 + 3] = spline.GetPointLocal(1);
        mesh.triangles = null;
        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }
}

[CustomEditor(typeof(SplineRoad))]
public class SplineRoadEditor : Editor
{

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
    }
    private void OnSceneGUI()
    {
        (target as SplineRoad).Generate();
    }
}
