using UnityEngine;
public class DoubleKeyCheck
{
    public DoubleKeyCheck(float threshold) { this.threshold = threshold; }
    float threshold, t; KeyCode lastKey = 0;
    void Update(float dt) { t += dt; }
    public bool GetKeyDownDouble(KeyCode key)
    {
        if (Input.GetKeyDown(key))
        {
            if (lastKey == key && t < threshold)
            {
                lastKey = 0;
                return true;
            }
            else
            {
                lastKey = key;
                t = 0;
            }
        }
        return false;
    }
}