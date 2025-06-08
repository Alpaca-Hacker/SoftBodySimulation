using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class FPSDisplay : MonoBehaviour
{
    public TMP_Text fpsText;

    float deltaTime = 0.0f;

    void Update()
    {
        deltaTime += (Time.unscaledDeltaTime - deltaTime) * 0.1f;
        var fps = 1.0f / deltaTime;
        fpsText.text = $"FPS: {Mathf.Ceil(fps)}";
    }
}