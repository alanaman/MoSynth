using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.TextCore.Text;

namespace MotionMatching
{
    /// <summary>
    /// This class provides a simple interface to control a character with Motion Matching.
    /// Use the SetVelocity function to set the desired velocity of the character.
    /// The character will move in the direction of the velocity. 
    /// The magnitude is used to set the speed of the character. 
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class SimpleMMController : MonoBehaviour
    {
        private SpringCharacterController _characterController;

        private void Awake()
        {
            _characterController = GetComponentInChildren<SpringCharacterController>();
        }

        /// <summary>
        /// Use this function to set the desired velocity of the character.
        /// </summary>
        /// <param name="velocity">The desired direction and speed of the character.</param>
        public void SetVelocity(Vector2 velocity)
        {
            _characterController.SetMovementDirection(velocity);
        }
    }
}
