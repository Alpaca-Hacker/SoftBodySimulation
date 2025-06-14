using SoftBody.Scripts;
using UnityEditor;
using UnityEngine;

namespace SoftBody.Editor
{
    [CustomEditor(typeof(SoftBodySimulator))]
    public class SoftBodyEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            var softBody = (SoftBodySimulator)target;
        
            EditorGUILayout.HelpBox("XPBD Soft Body Physics Simulator\n" +
                                    "This component creates a soft body cube that can be deformed and interacted with.\n" +
                                    "Adjust the settings below to customize the behavior.", MessageType.Info);
        
            DrawDefaultInspector();
        
            EditorGUILayout.Space();
        
            if (GUILayout.Button("Regenerate Mesh"))
            {
                // Trigger mesh regeneration
                EditorApplication.delayCall += () =>
                {
                    if (softBody != null)
                    {
                        softBody.SendMessage("OnValidate", SendMessageOptions.DontRequireReceiver);
                    }
                };
            }
        
            if (Application.isPlaying)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Runtime Controls", EditorStyles.boldLabel);
            
                if (GUILayout.Button("Add Upward Force"))
                {
                    softBody.AddForce(Vector3.up * 10f, softBody.transform.position, 2f);
                }
            
                if (GUILayout.Button("Pin Center"))
                {
                    softBody.SetPinned(softBody.transform.position, 1f, true);
                }
            
                if (GUILayout.Button("Unpin All"))
                {
                    softBody.SetPinned(softBody.transform.position, 100f, false);
                }
            }
        }
    }
}
