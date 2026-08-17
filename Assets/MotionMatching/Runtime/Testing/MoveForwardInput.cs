using UnityEngine;

namespace MotionMatching.Testing
{
public class MoveForwardInput : MonoBehaviour
{
    public DirectionControlInput MMController;
    
    private void Update()
    {
        MMController.SetMovementDirection(new Vector2(0, 1));
    }
    
}
}