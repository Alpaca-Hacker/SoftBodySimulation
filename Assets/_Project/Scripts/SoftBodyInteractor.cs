using UnityEngine;

namespace SoftBody.Scripts
{
    public class SoftBodyInteractor : MonoBehaviour
{
    [Header("Interaction Settings")]
    public float forceStrength = 50f;
    public float interactionRadius = 2f;
    public KeyCode forceKey = KeyCode.Space;
    public KeyCode pinKey = KeyCode.P;
    public KeyCode unpinKey = KeyCode.U;
    
    [Header("Visual Feedback")]
    public bool showGizmos = true;
    public Color gizmoColor = Color.yellow;
    
    private SoftBodySimulator softBody;
    [SerializeField] Camera playerCamera;
    
    private void Start()
    {
        softBody = FindObjectOfType<SoftBodySimulator>();
        playerCamera = Camera.main;
        
        if (playerCamera == null)
            this.enabled = false;
    }
    
    private void Update()
    {
        if (!Application.isPlaying)
        {
            return;
        }
        HandleInput();
    }
    
    private void HandleInput()
    {
        if (!softBody || !playerCamera) return;
        
        var mouseWorldPos = GetMouseWorldPosition();
        
        if (Input.GetKeyDown(forceKey))
        {
            var forceDirection = Vector3.up;
            softBody.AddForce(forceDirection * forceStrength, mouseWorldPos, interactionRadius);
        }
        
        if (Input.GetKeyDown(pinKey))
        {
            softBody.SetPinned(mouseWorldPos, interactionRadius, true);
        }
        
        if (Input.GetKeyDown(unpinKey))
        {
            softBody.SetPinned(mouseWorldPos, interactionRadius, false);
        }
        
        // Mouse interaction
        if (Input.GetMouseButton(0))
        {
            var forceDirection = (mouseWorldPos - softBody.transform.position).normalized;
            softBody.AddForce(forceDirection * forceStrength * 0.1f, mouseWorldPos, interactionRadius);
        }
    }
    
    private Vector3 GetMouseWorldPosition()
    {
        var ray = playerCamera.ScreenPointToRay(Input.mousePosition);
        var plane = new Plane(Vector3.forward, softBody.transform.position);
        
        if (plane.Raycast(ray, out var distance))
        {
            return ray.GetPoint(distance);
        }
        
        return softBody.transform.position;
    }
    
    private void OnDrawGizmos()
    {
        if (!showGizmos) return;
        
        Gizmos.color = gizmoColor;
        var mousePos = GetMouseWorldPosition();
        Gizmos.DrawWireSphere(mousePos, interactionRadius);
    }
}
}