using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class SplineWalker : MonoBehaviour
{
    public Spline spline;
    public float duration = 5f;
    public enum LoopType { Once,Loop, PingPong };
    public LoopType loopType = LoopType.PingPong;
    public bool lookAt = true;
    
    public float progress = 0;
    public Vector3 bias = Vector3.zero;
    public bool playing = true;
    
    
    void Update()
    {
        if (Application.isPlaying &&  playing)
        {
            progress += Time.deltaTime / duration;
            if(loopType==LoopType.Once && progress > 1)
            {
                progress = 1;
                playing = false;
            }
        }
        Do();
    }
    void Do()
    {
        if (spline == null) return;
        float t = progress;
        if (loopType == LoopType.Loop)
            t = progress - Mathf.Floor(progress);
        else if (loopType == LoopType.PingPong)
            t = 1 - Mathf.Abs((progress - 2 * Mathf.Floor(progress / 2)) - 1);
        else
            t = Mathf.Clamp01(t);
        transform.position = spline.GetPoint(t) + bias.x * spline.GetNormalLocal(t,Vector3.up) +bias.y*Vector3.up + bias.z * spline.GetTangentLocal(t);
        if (lookAt) transform.LookAt(spline.GetPoint(t) + spline.GetDerivative(t), Vector3.up);
    }
}
