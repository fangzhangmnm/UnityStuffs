using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DebugTimeSlider : MonoBehaviour
{
    [Range(0,2)]
    public float timeScale = 1f;
    void Start()
    {
        Time.timeScale = timeScale;
    }
    private void Update()
    {
        if(Application.isEditor)
            if (Input.GetKeyDown(KeyCode.P))
                Time.timeScale = timeScale = timeScale != 1f ? 1f : 0.2f;
    }

    private void OnValidate()
    {
        if(Application.isPlaying)
            Time.timeScale = timeScale;

    }
}
