using UnityEngine;
using UnityEngine.Splines;

namespace AnimationTools
{
/// <summary>Common surface over the control-input hierarchies (MotionMatching, MotionField) so
/// synthesis-agnostic tools can drive and inspect them.</summary>
public interface IMotionSynthesisControlInput
{
    MotionSynthesisComponent Synthesizer { get; }
}

/// <summary>A control input that steers a character along a spline.</summary>
public interface IMotionSynthesisSplineControlInput : IMotionSynthesisControlInput
{
    SplineContainer SplineContainer { get; set; }

    /// <summary>Desired locomotion speed in m/s; <see cref="float.NaN"/> when the input has no speed model.</summary>
    float TargetSpeed { get; }
}

/// <summary>A control input steered by a 2D movement direction (stick/WASD-style).</summary>
public interface IMotionSynthesisDirectionControlInput : IMotionSynthesisControlInput
{
    void SetMovementDirection(Vector2 movementDirection);
}
}
