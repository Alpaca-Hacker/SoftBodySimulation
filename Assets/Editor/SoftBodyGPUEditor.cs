using UnityEditor;
using UnityEngine;

namespace SoftBody.Editor
{
    [CustomEditor(typeof(SoftBodyGPU))]
    public class SoftBodyGPUEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            var softBody = (SoftBodyGPU)target;
            
            EditorGUILayout.Space();
            
            if (GUILayout.Button("Restart Simulation"))
            {
                if (Application.isPlaying)
                {
                    softBody.RestartSimulation();
                }
                else
                {
                    Debug.LogWarning("Can only restart simulation in Play Mode.");
                }
            }
        }
    }
}