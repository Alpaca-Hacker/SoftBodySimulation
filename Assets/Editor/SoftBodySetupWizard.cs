using SoftBody.Scripts;
using UnityEngine;
using UnityEditor;

namespace SoftBody.Editor
{


    public class SoftBodySetupWizard : ScriptableWizard
    {
        [Header("Quick Setup")] public GameObject targetObject;
        public ComputeShader computeShader;

        [Header("Preset")] public SoftBodyPreset preset;

        [MenuItem("GameObject/3D Object/Soft Body Cube")]
        static void CreateSoftBodyCube()
        {
            // Create new GameObject
            GameObject softBodyObj = new GameObject("Soft Body Cube");

            // Add required components
            MeshFilter meshFilter = softBodyObj.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = softBodyObj.AddComponent<MeshRenderer>();

            // Create and assign default material
            Material defaultMaterial = new Material(Shader.Find("Standard"));
            defaultMaterial.color = Color.cyan;
            // defaultMaterial.metallic = 0.2f;
            // defaultMaterial.smoothness = 0.8f;
            meshRenderer.material = defaultMaterial;

            SoftBodySimulator simulator = softBodyObj.AddComponent<SoftBodySimulator>();

            // Try to find compute shader automatically
            string[] guids = AssetDatabase.FindAssets("XPBDSoftBody t:ComputeShader");
            if (guids.Length > 0)
            {
                string path = AssetDatabase.GUIDToAssetPath(guids[0]);
                ComputeShader cs = AssetDatabase.LoadAssetAtPath<ComputeShader>(path);

                // Use reflection to set the compute shader
                var field = typeof(SoftBodySimulator).GetField("computeShader",
                    System.Reflection.BindingFlags.NonPublic |
                    System.Reflection.BindingFlags.Instance);
                field?.SetValue(simulator, cs);
            }

            // Add debugger component
            softBodyObj.AddComponent<SoftBodyDebugger>();

            // Set default position
            softBodyObj.transform.position = Vector3.zero;

            // Select the new object
            Selection.activeGameObject = softBodyObj;

            Debug.Log("Soft Body Cube created! The system will automatically detect floor colliders.");
        }

        [MenuItem("Tools/Soft Body Setup Wizard")]
        static void CreateWizard()
        {
            ScriptableWizard.DisplayWizard<SoftBodySetupWizard>("Soft Body Setup", "Setup");
        }

        void OnWizardCreate()
        {
            if (targetObject == null)
            {
                EditorUtility.DisplayDialog("Error", "Please assign a target object.", "OK");
                return;
            }

            // Add components if they don't exist
            if (targetObject.GetComponent<MeshFilter>() == null)
                targetObject.AddComponent<MeshFilter>();

            if (targetObject.GetComponent<MeshRenderer>() == null)
                targetObject.AddComponent<MeshRenderer>();

            SoftBodySimulator simulator = targetObject.GetComponent<SoftBodySimulator>();
            if (simulator == null)
                simulator = targetObject.AddComponent<SoftBodySimulator>();

            // Apply compute shader
            if (computeShader != null)
            {
                var field = typeof(SoftBodySimulator).GetField("computeShader",
                    System.Reflection.BindingFlags.NonPublic |
                    System.Reflection.BindingFlags.Instance);
                field?.SetValue(simulator, computeShader);
            }

            // Apply preset if available
            if (preset != null)
            {
                preset.ApplyToSoftBody(simulator);
            }

            // Add interactor component
            if (targetObject.GetComponent<SoftBodyInteractor>() == null)
                targetObject.AddComponent<SoftBodyInteractor>();

            EditorUtility.SetDirty(targetObject);
            Debug.Log($"Soft body setup complete for {targetObject.name}!");
        }
    }
}