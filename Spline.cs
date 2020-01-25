using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System;

//https://catlikecoding.com/unity/tutorials/curves-and-splines/
public class Spline : MonoBehaviour
{
    static Vector3 f(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3,float t)
    {
        t = Mathf.Clamp01(t);float omt = 1 - t;
        return omt * omt * omt * p0
            + 3 * omt * omt * t * p1
            + 3 * omt * t * t * p2
            + t * t * t * p3;
    }
    static Vector3 df(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        t = Mathf.Clamp01(t); float omt = 1 - t;
        return 3 * omt * omt * (p1 - p0)
            + 6 * omt * t * (p2 - p1)
            + 3 * t * t * (p3 - p2);
    }
    static Vector3 ddf(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        t = Mathf.Clamp01(t); 
        return -6 * (-p2 + p1 * (2 - 3 * t) + p0 * (-1 + t) + 3 * p2 * t - p3 * t);
    }
    public int segments { get { return loop ? points.Length : points.Length - 1; } }
    public enum PointType { Free,Aligned,Mirrored };
    [System.Serializable]
    public struct Point
    {
        public Vector3 position;
        public Vector3 leftTangent;
        public Vector3 rightTangent;
        public PointType type;
    }
    public Point[] points;
    public bool loop;
    public bool drawCurveInEditorWindow;
    public void Reset()
    {
        points = new Point[2];
        points[0].leftTangent = new Vector3(0, 0, -1);
        points[0].position = new Vector3(0, 0, 0);
        points[0].rightTangent = new Vector3(0, 0, 1);
        points[1].leftTangent = new Vector3(1, 0, 0);
        points[1].position = new Vector3(1, 0, 1);
        points[1].rightTangent = new Vector3(1, 0, 2);
        points[0].type = PointType.Mirrored;
        points[1].type = PointType.Mirrored;
        loop = false;
        drawCurveInEditorWindow = true;
    }
    private void OnDrawGizmos()
    {
        if (drawCurveInEditorWindow)
        {
            Gizmos.color = Color.white;
            for (int i = 0; i < points.Length - 1; ++i)
            {
                Vector3[] p = Handles.MakeBezierPoints(
                    transform.TransformPoint(points[i].position),
                    transform.TransformPoint(points[i + 1].position),
                    transform.TransformPoint(points[i].rightTangent),
                    transform.TransformPoint(points[i + 1].leftTangent),
                    10);
                for (int j = 0; j < 9; ++j)
                    Gizmos.DrawLine(p[j], p[j + 1]);
            }
            if (loop)
            {
                Vector3[] p = Handles.MakeBezierPoints(
                    transform.TransformPoint(points[points.Length - 1].position),
                    transform.TransformPoint(points[0].position),
                    transform.TransformPoint(points[points.Length - 1].rightTangent),
                    transform.TransformPoint(points[0].leftTangent),
                    10);
                for (int j = 0; j < 9; ++j)
                    Gizmos.DrawLine(p[j], p[j + 1]);
            }
        }
    }
    public Vector3 GetPointLocal(float t)
    {
        t = loop?t - Mathf.Floor(t):Mathf.Clamp01(t);
        int n = loop ? points.Length : points.Length - 1;
        int i = Mathf.FloorToInt(t * n);
        t = t * n - i;
        int j = (i + 1) % points.Length;
        return f(points[i].position, points[i].rightTangent, points[j].leftTangent, points[j].position,t);
    }
    public Vector3 GetDerivativeLocal(float t)
    {
        t = loop ? t - Mathf.Floor(t) : Mathf.Clamp01(t);
        int n = loop ? points.Length : points.Length - 1;
        int i = Mathf.FloorToInt(t * n);
        t = t * n - i;
        int j = (i + 1) % points.Length;
        return df(points[i].position, points[i].rightTangent, points[j].leftTangent, points[j].position, t);
    }
    public Vector3 GetSecondDerivativeLocal(float t)
    {
        t = loop ? t - Mathf.Floor(t) : Mathf.Clamp01(t);
        int n = loop ? points.Length : points.Length - 1;
        int i = Mathf.FloorToInt(t * n);
        t = t * n - i;
        int j = (i + 1) % points.Length;
        return ddf(points[i].position, points[i].rightTangent, points[j].leftTangent, points[j].position, t);
    }
    public float ProjectLocal(Vector3 pointLocal, float start = 0, float end = 1, int nIter = 10)
    {
        if (!loop) { start = Mathf.Clamp01(start); end = Mathf.Clamp01(end); }
        int loopOffset = Mathf.FloorToInt(start);
        start -= loopOffset; end -= loopOffset;if(end>start+1)end -= Mathf.Floor(end - start);
        float min = float.PositiveInfinity, mint = 0;
        //Debug.Log($"start={start},end={end},{Mathf.FloorToInt(start * segments)},{Mathf.CeilToInt(end * segments)}");
         for (int ii = Mathf.FloorToInt(start*segments); ii < Mathf.CeilToInt(end*segments); ++ii)
        {
            int i = ii % points.Length;
            int j = (ii + 1) % points.Length;
            float s = Mathf.Clamp01(start * segments - ii);
            float e = Mathf.Clamp01(end * segments - ii);
            //Debug.Log($"i={i},j={j},s={s},e={e},start={start},end={end}");
            float t = ProjectSegment(pointLocal, points[i].position, points[i].rightTangent, points[j].leftTangent, points[j].position,
                s,e,nIter);
            float d = Vector3.Distance(pointLocal, f(points[i].position, points[i].rightTangent, points[j].leftTangent, points[j].position, t));
            if (d < min)
            {
                min = d;
                mint = (ii + t) / segments;
            }
        }
        return mint + loopOffset;
    }
    float ProjectSegment(Vector3 q,Vector3 p0,Vector3 p1,Vector3 p2, Vector3 p3, float start = 0, float end = 1, int nIter=10)
    {
        float dt = Mathf.Clamp((end-start) / 5, 0.1f,1f) , tmin = 0, min = float.PositiveInfinity;
        for(float t = start; t < end + dt; t += dt)
        {
            float d = Vector3.Distance(q, f(p0, p1, p2, p3, t));
            if (d < min) { min = d; tmin = t; }
        }
        //https://en.wikipedia.org/wiki/Newton%27s_method_in_optimization
        for(int i = 0; i < nIter; ++i)
        {
            Vector3 p = f(p0, p1, p2, p3, tmin);
            Vector3 dp= df(p0, p1, p2, p3, tmin);
            Vector3 ddp = ddf(p0, p1, p2, p3, tmin);
            float dd = 2 * Vector3.Dot( p- q, dp);
            float ddd = 2 * Vector3.Dot(p - q, ddp) + 2 * dp.sqrMagnitude;
            tmin -= Mathf.Clamp(dd / ddd, -dt, dt);
            tmin = Mathf.Clamp(tmin, start, end);
        }
        return tmin;
    }
    public Vector3 GetPoint(float t) { return transform.TransformPoint(GetPointLocal(t)); }
    public Vector3 GetDerivative(float t) { return transform.TransformVector(GetDerivativeLocal(t)); }
    public float Project(Vector3 point, float start = 0, float end = 1, int nIter = 10) { return ProjectLocal(transform.InverseTransformPoint(point), start, end, nIter); }
    public Vector3 GetTangentLocal(float t) { return GetDerivativeLocal(t).normalized; }
    public Vector3 GetTangent(float t) { return GetDerivative(t).normalized; }
    public Vector3 GetNormalLocal(float t,Vector3 up) { return Vector3.Cross(up, GetTangentLocal(t)).normalized; }
    public Vector3 GetNormalNoShear(float t, Vector3 up) { return Vector3.Cross(up, GetTangent(t)).normalized; }//!=transform.TransformVector(getNormalLocal(t,up))
}

[CustomEditor(typeof(Spline))]
public class SplineEditor : Editor
{
    Spline spline;
    Spline.Point[] points;
    Transform transform;
    int selectedIndex = 0, selectedIndex1 = 0;
    Quaternion handleRotation;
    static Color[] modeColors = {
        Color.white,
        Color.yellow,
        Color.cyan
    };
    private void OnSceneGUI()
    {
        spline = target as Spline;
        points = spline.points;
        transform = spline.transform;
        handleRotation = Tools.pivotRotation == PivotRotation.Local ? transform.rotation : Quaternion.identity;
        
        for (int i = 0; i < points.Length; ++i)
        {
            Handles.color = Color.gray;
            Handles.DrawLine(transform.TransformPoint(points[i].position),
                transform.TransformPoint(points[i].leftTangent));
            Handles.DrawLine(transform.TransformPoint(points[i].position),
                transform.TransformPoint(points[i].rightTangent));

            Handles.color = modeColors[(int)points[i].type];
            float s = HandleUtility.GetHandleSize(transform.TransformPoint(points[i].position));
            if (Handles.Button(transform.TransformPoint(points[i].position), handleRotation, s*0.04f, s*0.06f, Handles.DotHandleCap))
            {
                Repaint();
                selectedIndex = i;
                selectedIndex1 = 0;
            }
            s = HandleUtility.GetHandleSize(transform.TransformPoint(points[i].leftTangent));
            if (Handles.Button(transform.TransformPoint(points[i].leftTangent), handleRotation, s * 0.04f, s * 0.06f, Handles.DotHandleCap))
            {
                Repaint();
                selectedIndex = i;
                selectedIndex1 = 1;
            }
            s = HandleUtility.GetHandleSize(transform.TransformPoint(points[i].rightTangent));
            if (Handles.Button(transform.TransformPoint(points[i].rightTangent), handleRotation, s * 0.04f, s * 0.06f, Handles.DotHandleCap))
            {
                Repaint();
                selectedIndex = i;
                selectedIndex1 = 2;
            }

            Vector3 newPosition;
            if (selectedIndex == i && selectedIndex1 == 0)
            {
                EditorGUI.BeginChangeCheck();
                newPosition = PositionHandleLocal(points[i].position);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(spline, "Edit Spline");
                    points[i].leftTangent += newPosition - points[i].position;
                    points[i].rightTangent += newPosition - points[i].position;
                    points[i].position = newPosition;
                }
            }
            if (selectedIndex == i && selectedIndex1 == 1)
            {
                EditorGUI.BeginChangeCheck();
                newPosition = PositionHandleLocal(points[i].leftTangent);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(spline, "Edit Spline");
                    points[i].leftTangent = newPosition;
                    if (points[i].type == Spline.PointType.Mirrored)
                        points[i].rightTangent = Mirror(points[i].rightTangent, points[i].position, points[i].leftTangent);
                    else if (points[i].type == Spline.PointType.Aligned)
                        points[i].rightTangent = Align(points[i].rightTangent, points[i].position, points[i].leftTangent);
                }
            }
            if (selectedIndex == i && selectedIndex1 == 2)
            {
                EditorGUI.BeginChangeCheck();
                newPosition = PositionHandleLocal(points[i].rightTangent);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(spline, "Edit Spline");
                    points[i].rightTangent = newPosition;
                    if (points[i].type == Spline.PointType.Mirrored)
                        points[i].leftTangent = Mirror(points[i].leftTangent, points[i].position, points[i].rightTangent);
                    else if (points[i].type == Spline.PointType.Aligned)
                        points[i].leftTangent = Align(points[i].leftTangent, points[i].position, points[i].rightTangent);
                }
            }
        }
    }

    public override void OnInspectorGUI()
    {
        spline = target as Spline;
        points = spline.points;
        transform = spline.transform;
        handleRotation = Tools.pivotRotation == PivotRotation.Local ? transform.rotation : Quaternion.identity;

        for (int i = 0; i < points.Length; ++i)
        {
            Vector3 newPosition;
            Spline.PointType newType;
            if (selectedIndex == i && selectedIndex1 == 0)
            {
                EditorGUI.BeginChangeCheck();
                GUILayout.Label("Selected Point");
                newPosition = EditorGUILayout.Vector3Field("Position", points[i].position);
                newType = (Spline.PointType)EditorGUILayout.EnumPopup("type", points[i].type);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(spline, "Edit Spline");
                    points[i].leftTangent += newPosition - points[i].position;
                    points[i].rightTangent += newPosition - points[i].position;
                    points[i].position = newPosition;
                    points[i].type = newType;
                    if (points[i].type == Spline.PointType.Mirrored)
                        points[i].rightTangent = Mirror(points[i].rightTangent, points[i].position, points[i].leftTangent);
                    else if (points[i].type == Spline.PointType.Aligned)
                        points[i].rightTangent = Align(points[i].rightTangent, points[i].position, points[i].leftTangent);
                }
            }
            if (selectedIndex == i && selectedIndex1 == 1)
            {
                EditorGUI.BeginChangeCheck();
                GUILayout.Label("Selected Point");
                newPosition = EditorGUILayout.Vector3Field("Position", points[i].leftTangent);
                newType = (Spline.PointType)EditorGUILayout.EnumPopup("type", points[i].type);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(spline, "Edit Spline");
                    points[i].leftTangent = newPosition;
                    points[i].type = newType;
                    if (points[i].type == Spline.PointType.Mirrored)
                        points[i].rightTangent = Mirror(points[i].rightTangent, points[i].position, points[i].leftTangent);
                    else if (points[i].type == Spline.PointType.Aligned)
                        points[i].rightTangent = Align(points[i].rightTangent, points[i].position, points[i].leftTangent);
                }
            }
            if (selectedIndex == i && selectedIndex1 == 2)
            {
                EditorGUI.BeginChangeCheck();
                GUILayout.Label("Selected Point");
                newPosition = EditorGUILayout.Vector3Field("Position", points[i].rightTangent);
                newType = (Spline.PointType)EditorGUILayout.EnumPopup("type", points[i].type);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(spline, "Edit Spline");
                    points[i].rightTangent = newPosition;
                    points[i].type = newType;
                    if (points[i].type == Spline.PointType.Mirrored)
                        points[i].leftTangent = Mirror(points[i].leftTangent, points[i].position, points[i].rightTangent);
                    else if (points[i].type == Spline.PointType.Aligned)
                        points[i].leftTangent = Align(points[i].leftTangent, points[i].position, points[i].rightTangent);

                }
            }
        }
        if (GUILayout.Button("Add Point"))
        {
            Undo.RecordObject(spline, "Edit Spline");
            Array.Resize(ref spline.points, points.Length + 1);
            points = spline.points;
            points[points.Length - 1].position = points[points.Length - 2].position + (points[points.Length - 2].rightTangent - points[points.Length - 2].position)*1.5f ;
            points[points.Length - 1].leftTangent = points[points.Length - 1].position - (points[points.Length - 2].rightTangent - points[points.Length - 2].position);
            points[points.Length - 1].rightTangent = points[points.Length - 1].position + (points[points.Length - 2].rightTangent - points[points.Length - 2].position);
            points[points.Length - 1].type = points[points.Length - 2].type;
        }
        if (GUILayout.Button("Remove Point"))
        {
            Undo.RecordObject(spline, "Edit Spline");
            Array.Resize(ref spline.points, points.Length - 1);
            points = spline.points;
        }
        EditorGUI.BeginChangeCheck();
        bool newLoop = EditorGUILayout.Toggle("Loop", spline.loop);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(spline, "Edit Spline");
            spline.loop = newLoop;
        }
        EditorGUI.BeginChangeCheck();
        bool newBool = EditorGUILayout.Toggle("Draw Curve In Editor Window", spline.drawCurveInEditorWindow);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(spline, "Edit Spline");
            spline.drawCurveInEditorWindow = newBool;
        }
    }
    Vector3 Mirror(Vector3 target, Vector3 origin, Vector3 other)
    {
        return 2 * origin - other;
    }
    Vector3 Align(Vector3 target,Vector3 origin,Vector3 other)
    {
        return (origin - other).magnitude > 0 ? origin + (origin - other).normalized * (target-origin).magnitude : target;
    }
    Vector3 PositionHandleLocal(Vector3 point)
    {
        return
            transform.InverseTransformPoint(
                Handles.PositionHandle(
                    transform.TransformPoint(point),
                    handleRotation));
    }
}