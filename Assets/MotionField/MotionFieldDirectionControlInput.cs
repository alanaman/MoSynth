using UnityEngine;

namespace MotionField
{
/// <summary>
/// Steers the motion field from a 2D movement input (gamepad stick or WASD), interpreted
/// relative to a reference transform's facing. Wire UserInput's Player.Move event to
/// <see cref="SetMovementDirection"/>.
/// </summary>
public class MotionFieldDirectionControlInput : MotionFieldControlInput
{
    [Tooltip("Input is taken relative to this transform's facing. Falls back to the main camera.")]
    [SerializeField]
    private Transform inputReference;

    private Vector2 _inputMovement;

    /// <summary>Latest 2D movement input; x is strafe, y is forward. Wired to UserInput.</summary>
    public void SetMovementDirection(Vector2 movementDirection)
    {
        _inputMovement = movementDirection;
    }

    protected override Vector3 GetDesiredWorldDirection()
    {
        // UserInput only forwards .performed, so a released stick may never report zero;
        // treating small input as "keep the last heading" matches that, and the policy
        // cannot stand still anyway.
        if (_inputMovement.sqrMagnitude < 1e-4f) return Vector3.zero;

        var reference = inputReference;
        if (reference == null && Camera.main != null) reference = Camera.main.transform;
        if (reference == null) return new Vector3(_inputMovement.x, 0f, _inputMovement.y);

        var right = reference.right;
        var ahead = reference.forward;
        right.y = 0f;
        ahead.y = 0f;
        return right.normalized * _inputMovement.x + ahead.normalized * _inputMovement.y;
    }
}
}
