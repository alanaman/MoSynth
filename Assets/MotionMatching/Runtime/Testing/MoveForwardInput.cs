using UnityEngine;

namespace MotionMatching.Testing
{
public class MoveForwardInput : MonoBehaviour
{
    public SimpleMMController MMController;
    
    private void Update()
    {
        MMController.SetVelocity(new Vector2(0, 1));
    }
    
}
}