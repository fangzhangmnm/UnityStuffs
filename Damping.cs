using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DampingVector3
{
    public Vector3 externalForce = Vector3.zero;
    public Vector3 speed = Vector3.zero,acceleration=Vector3.zero;
    public Vector3 current;
    public float KP, KD, minDT;
    public DampingVector3(float period, float ratio)
    {
        float omega = (2 * Mathf.PI) / period;
        KP = omega * omega;
        KD = 2 * ratio * omega;
        minDT = 0.5f / omega;
    }
    public Vector3 Update(Vector3 target, float dt)
    {
        if (dt > minDT)
        {
            for (int i = 0; i < Mathf.FloorToInt(dt / minDT); ++i)
                Update(target, minDT);
            return Update(target, dt - Mathf.Floor(dt / minDT) * minDT);
        }
        Vector3 P = target - current;
        acceleration = KP * P - KD * speed + externalForce;
        speed += acceleration * dt;
        Vector3 delta = speed * dt;
        float d = P.magnitude;
        float dd = Vector3.Dot(delta, P) / (d * d);
        delta -= P * (dd - Mathf.Min(1, dd));
        if (d < 0.001f) delta = Vector3.zero;
        return current += delta;
    }
    public void Reset(Vector3 target)
    {
        current = target;
        speed = Vector3.zero;
    }
}
public class DerivativeVector3
{
    public Vector3 oldValue=Vector3.zero, derivative=Vector3.zero;
    public float dampingTime;
    public DerivativeVector3(float dampingTime) { this.dampingTime = dampingTime; }
    public Vector3 Update(Vector3 newValue,float dt)
    {
        Vector3 newDerivative = (newValue - oldValue) / dt;
        oldValue = newValue;
        return derivative=Vector3.Lerp(derivative,newDerivative , dt / (dt + dampingTime));
    }
    public void Reset(Vector3 target)
    {
        oldValue = target;
        derivative = Vector3.zero;
    }
    public void SetOld(Vector3 oldValue)
    {
        this.oldValue = oldValue;
    }
}
public class DampingFloat
{
    public float externalForce = 0;
    public float speed = 0, acceleration = 0;
    public float current;
    public float KP, KD, minDT;
    public DampingFloat(float period, float ratio)
    {
        float omega = (2 * Mathf.PI) / period;
        KP = omega * omega;
        KD = 2 * ratio * omega;
        minDT = 0.5f / omega;
    }
    public float Update(float target, float dt)
    {
        if (dt > minDT)
        {
            for (int i = 0; i < Mathf.FloorToInt(dt / minDT); ++i)
                Update(target, minDT);
            return Update(target, dt - Mathf.Floor(dt / minDT) * minDT);
        }
        float P = target - current;
        acceleration = KP * P - KD * speed + externalForce;
        speed += acceleration * dt;
        float delta = speed * dt;
        float d = Mathf.Abs(P);
        float dd = delta*P / (d * d);
        delta -= P * (dd - Mathf.Min(1, dd));
        if (d < 0.001f) delta = 0;
        return current += delta;
    }
    public void Reset(float target)
    {
        current = target;
        speed = 0;
    }
}