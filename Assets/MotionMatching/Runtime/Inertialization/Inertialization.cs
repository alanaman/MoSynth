using System;
using AnimationTools;
using Unity.Collections;
using Unity.Mathematics;

namespace MotionMatching
{
/// <summary>
/// Inertialization is a type of blending between two poses.
///
/// Typically to blend between two poses, we use crossfade.
/// Both poses are stored queried during the transition,
/// and they are blended during the transition.
///
/// The basic idea of inertialization is when we change the pose,
/// we change a target pose instantly but compute offsets that
/// take us from the current pose to the target one.
/// Then, we decay this offset using a polynomial function or springs.
/// Decaying the offset will progressively take us to the target pose.
/// </summary>
[Serializable]
public class Inertialization : MoSynthStage
{
    #region OLD_IMPL

    [NonSerialized] public quaternion[] InertializedRotations;
    private float3[] _inertializedAngularVelocities;
    [NonSerialized] public float3 InertializedHips;
    [NonSerialized] public float3 InertializedHipsVelocity;

    private quaternion[] _offsetRotations;
    private float3[] _offsetAngularVelocities;
    private float3 _offsetHips;

    private float3 _offsetHipsVelocity;

    // Contacts
    private float3 _offsetLeftContact;
    private float3 _offsetLeftContactVelocity;
    private float3 _offsetRightContact;
    private float3 _offsetRightContactVelocity;


    #endregion

    MotionSynthesisComponent _owner;

    public Inertialization()
    {

    }

    public Inertialization(int jointCount)
    {
        InertializedRotations = new quaternion[jointCount];
        _inertializedAngularVelocities = new float3[jointCount];
        _offsetRotations = new quaternion[jointCount];
        for (int i = 0; i < jointCount; i++) _offsetRotations[i] = quaternion.identity; // init to a valid quaternion
        _offsetAngularVelocities = new float3[jointCount];
    }

    /// <summary>
    /// Stores the output of this stage from the previous frame.
    /// Inertialization is applied from this pose to the input pose passed <see cref="Inertialization.Apply"/>.
    /// </summary>
    /// <remarks>
    /// If inertialization needs to be applied from the currently
    /// displayed pose, the inverted retargeting to needs to be
    /// fed to this stage.
    /// </remarks>
    private PoseBuffer _currentPose;

    private bool _isPoseInitialized = false;
    public float halfLife = 0.1f;

    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        _owner = motionSynthesisComponent;
        _currentPose = PoseBuffer.Allocate(motionSynthesisComponent.PoseLayout, Allocator.Persistent);
    }

    public override void OnDestroy()
    {
        if (_currentPose.IsCreated) _currentPose.Dispose();
    }

    public override bool Apply(PoseBuffer pose, float deltaTime)
    {
        if (!_isPoseInitialized)
        {
            _currentPose.CopyFrom(pose);
            _isPoseInitialized = true;
            return true;
        }

        var rotations = pose.Rotations;
        var angularVelocities = pose.AngularVelocities;
        var currentRotations = _currentPose.Rotations;
        var currentAngularVelocities = _currentPose.AngularVelocities;

        // Update the rotational inertialization for joint local rotations
        for (int i = 1; i < rotations.Length; i++)
        {
            var targetRot = rotations[i];
            var targetAngularVelocity = angularVelocities[i];
            var currentRot = currentRotations[i];
            var currentAngularVelocity = currentAngularVelocities[i];
            (rotations[i], angularVelocities[i]) = InertializeJointUpdate(
                currentRot, currentAngularVelocity,
                targetRot, targetAngularVelocity, deltaTime);

        }

        // Update the linear inertialization for hips
        var positions = pose.Positions;
        var velocities = pose.Velocities;
        var currentPositions = _currentPose.Positions;
        var currentVelocities = _currentPose.Velocities;
        var targetHipPos = positions[1];
        var targetHipVel = velocities[1];
        var currentHipPos = currentPositions[1];
        var currentHipVel = currentVelocities[1];
        (positions[1], velocities[1]) = InertializeJointUpdate(
            currentHipPos, currentHipVel,
            targetHipPos, targetHipVel,
            deltaTime
        );

        _currentPose.CopyFrom(pose);
        return true;
    }

    (quaternion newRot, float3 newAngularVel) InertializeJointUpdate(
        in quaternion currentRot, in float3 currentAngularVel,
        in quaternion targetRot, in float3 targetAngularVel,
        float deltaTime
    )
    {
        var offsetRot = math.mul(math.inverse(targetRot), currentRot);
        offsetRot = math.normalizesafe(MathExtensions.Abs(offsetRot));
        var offsetAngularVel = currentAngularVel - targetAngularVel;
        Spring.DecaySpringDamperImplicit(ref offsetRot, ref offsetAngularVel, halfLife, deltaTime);
        var newRot = math.mul(targetRot, offsetRot);
        var newAngularVel = targetAngularVel + offsetAngularVel;
        return (newRot, newAngularVel);
    }

    (float3 newPos, float3 newVel) InertializeJointUpdate(float3 currentPos, float3 currentVel,
        float3 targetPos, float3 targetVel, float deltaTime)
    {
        var offsetPos = currentPos - targetPos;
        var offsetVel = currentVel - targetVel;
        Spring.DecaySpringDamperImplicit(ref offsetPos, ref offsetVel, halfLife, deltaTime);
        var newPos = targetPos + offsetPos;
        var newVel = targetVel + offsetVel;
        return (newPos, newVel);
    }

    #region OLD_IMPL

    /// <summary>
    /// It takes as input the current state of the source pose and the target pose.
    /// It sets up the inertialization, which can then by updated by calling Update(...).
    /// </summary>
    public void PoseTransition(PoseSet poseSet, int sourcePoseIndex, int targetPoseIndex)
    {
        var sourcePose = poseSet.GetPoseBuffer(sourcePoseIndex);
        var targetPose = poseSet.GetPoseBuffer(targetPoseIndex);

        var sourceRotations = sourcePose.Rotations;
        var targetRotations = targetPose.Rotations;
        var sourceAngularVelocities = sourcePose.AngularVelocities;
        var targetAngularVelocities = targetPose.AngularVelocities;

        // Set up the inertialization for joint local rotations (no simulation bone)
        for (int i = 1; i < sourceRotations.Length; i++)
        {
            quaternion sourceJointRotation = sourceRotations[i];
            quaternion targetJointRotation = targetRotations[i];
            float3 sourceJointAngularVelocity = sourceAngularVelocities[i];
            float3 targetJointAngularVelocity = targetAngularVelocities[i];
            InertializeJointTransition(sourceJointRotation, sourceJointAngularVelocity,
                targetJointRotation, targetJointAngularVelocity,
                ref _offsetRotations[i], ref _offsetAngularVelocities[i]);
        }

        // Set up the inertialization for hips
        var sourcePositions = sourcePose.Positions;
        var targetPositions = targetPose.Positions;
        var sourceVelocities = sourcePose.Velocities;
        var targetVelocities = targetPose.Velocities;
        float3 sourceHips = sourcePositions[1];
        float3 targetHips = targetPositions[1];
        float3 sourceHipsVelocity = sourceVelocities[1];
        float3 targetHipsVelocity = targetVelocities[1];
        InertializeJointTransition(sourceHips, sourceHipsVelocity,
            targetHips, targetHipsVelocity,
            ref _offsetHips, ref _offsetHipsVelocity);
    }

    /// <summary>
    /// It takes as input the current state of the source contact and the target contact.
    /// It sets up the inertialization, which can then by updated by calling UpdateContact(...).
    /// </summary>
    public void LeftContactTransition(float3 sourceLeftContact, float3 sourceLeftContactVelocity,
        float3 targetLeftContact, float3 targetLeftContactVelocity)
    {
        InertializeJointTransition(sourceLeftContact, sourceLeftContactVelocity,
            targetLeftContact, targetLeftContactVelocity,
            ref _offsetLeftContact, ref _offsetLeftContactVelocity);
    }

    /// <summary>
    /// It takes as input the current state of the source contact and the target contact.
    /// It sets up the inertialization, which can then by updated by calling UpdateContact(...).
    /// </summary>
    public void RightContactTransition(float3 sourceRightContact, float3 sourceRightContactVelocity,
        float3 targetRightContact, float3 targetRightContactVelocity)
    {
        InertializeJointTransition(sourceRightContact, sourceRightContactVelocity,
            targetRightContact, targetRightContactVelocity,
            ref _offsetRightContact, ref _offsetRightContactVelocity);
    }

    public void UpdateLeftContact(float3 targetPos, float3 targetVelocity, float halfLife, float deltaTime,
        out float3 newPos, out float3 newVel)
    {
        InertializeJointUpdate(targetPos, targetVelocity,
            halfLife, deltaTime,
            ref _offsetLeftContact, ref _offsetLeftContactVelocity,
            out newPos, out newVel);
    }

    public void UpdateRightContact(float3 targetPos, float3 targetVelocity, float halfLife, float deltaTime,
        out float3 newPos, out float3 newVel)
    {
        InertializeJointUpdate(targetPos, targetVelocity,
            halfLife, deltaTime,
            ref _offsetRightContact, ref _offsetRightContactVelocity,
            out newPos, out newVel);
    }

    /// <summary>
    /// Updates the inertialization decaying the offset from the source pose (specified in PoseTransition(...))
    /// to the target pose.
    /// </summary>
    public void Update(PoseBuffer targetPose, float halfLife, float deltaTime)
    {
        var targetRotations = targetPose.Rotations;
        var targetAngularVelocities = targetPose.AngularVelocities;

        // Update the inertialization for joint local rotations
        for (int i = 1; i < targetRotations.Length; i++)
        {
            quaternion targetJointRotation = targetRotations[i];
            float3 targetAngularVelocity = targetAngularVelocities[i];
            InertializeJointUpdate(targetJointRotation, targetAngularVelocity,
                halfLife, deltaTime,
                ref _offsetRotations[i], ref _offsetAngularVelocities[i],
                out InertializedRotations[i], out _inertializedAngularVelocities[i]);
        }

        // Update the inertialization for hips
        var targetPositions = targetPose.Positions;
        var targetVelocities = targetPose.Velocities;
        float3 targetHips = targetPositions[1];
        float3 targetRootVelocity = targetVelocities[1];
        InertializeJointUpdate(targetHips, targetRootVelocity,
            halfLife, deltaTime,
            ref _offsetHips, ref _offsetHipsVelocity,
            out InertializedHips, out InertializedHipsVelocity);
    }

    // /// <summary>
    // /// Updates the inertialization decaying the offset from the source contact (specified in ContactTransition(...))
    // /// to the target contact.
    // /// </summary>
    // public void UpdateContact(float3 leftTargetPos, float3 leftTargetVelocity, float3 rightTargetPos, float3 rightTargetVelocity, float halfLife, float deltaTime)
    // {
    //     // Update the inertialization for contacts
    //     InertializeJointUpdate(leftTargetPos, leftTargetVelocity,
    //                            halfLife, deltaTime,
    //                            ref OffsetLeftContact, ref OffsetLeftContactVelocity,
    //                            out InertializedLeftContact, out InertializedLeftContactVelocity);
    //     InertializeJointUpdate(rightTargetPos, rightTargetVelocity,
    //                            halfLife, deltaTime,
    //                            ref OffsetRightContact, ref OffsetRightContactVelocity,
    //                            out InertializedRightContact, out InertializedRightContactVelocity);
    // }


    /// <summary>
    /// Compute the offsets from the source pose to the target pose.
    /// Offsets are in/out since we may start a inertialization in the middle of another inertialization.
    /// </summary>
    public static void InertializeJointTransition(quaternion sourceRot, float3 sourceAngularVel,
        quaternion targetRot, float3 targetAngularVel,
        ref quaternion offsetRot, ref float3 offsetAngularVel)
    {
        offsetRot = math.normalizesafe(MathExtensions.Abs(math.mul(math.inverse(targetRot),
            math.mul(sourceRot, offsetRot))));
        offsetAngularVel = (sourceAngularVel + offsetAngularVel) - targetAngularVel;
    }

    /// <summary>
    /// Compute the offsets from the source pose to the target pose.
    /// Offsets are in/out since we may start a inertialization in the middle of another inertialization.
    /// </summary>
    public static void InertializeJointTransition(float3 currentPos, float3 currentVel,
        float3 targetPos, float3 targetVel,
        ref float3 offset, ref float3 offsetVel)
    {
        offset = (currentPos + offset) - targetPos;
        offsetVel = (currentVel + offsetVel) - targetVel;
    }

    /// <summary>
    /// Compute the offsets from the source pose to the target pose.
    /// Offsets are in/out since we may start a inertialization in the middle of another inertialization.
    /// </summary>
    public static void InertializeJointTransition(float source, float sourceVel,
        float target, float targetVel,
        ref float offset, ref float offsetVel)
    {
        offset = (source + offset) - target;
        offsetVel = (sourceVel + offsetVel) - targetVel;
    }

    /// <summary>
    /// Updates the inertialization decaying the offset and applying it to the target pose
    /// </summary>
    public static void InertializeJointUpdate(quaternion targetRot, float3 targetAngularVel,
        float halfLife, float deltaTime,
        ref quaternion offsetRot, ref float3 offsetAngularVel,
        out quaternion newRot, out float3 newAngularVel)
    {
        Spring.DecaySpringDamperImplicit(ref offsetRot, ref offsetAngularVel, halfLife, deltaTime);
        newRot = math.mul(targetRot, offsetRot);
        newAngularVel = targetAngularVel + offsetAngularVel;
    }

    /// <summary>
    /// Updates the inertialization decaying the offset and applying it to the target pose
    /// </summary>
    public static void InertializeJointUpdate(float3 target, float3 targetVel,
        float halfLife, float deltaTime,
        ref float3 offset, ref float3 offsetVel,
        out float3 newValue, out float3 newVel)
    {
        Spring.DecaySpringDamperImplicit(ref offset, ref offsetVel, halfLife, deltaTime);
        newValue = target + offset;
        newVel = targetVel + offsetVel;
    }

    /// <summary>
    /// Updates the inertialization decaying the offset and applying it to the target pose
    /// </summary>
    public static void InertializeJointUpdate(float target, float targetVel,
        float halfLife, float deltaTime,
        ref float offset, ref float offsetVel,
        out float newValue, out float newVel)
    {
        Spring.DecaySpringDamperImplicit(ref offset, ref offsetVel, halfLife, deltaTime);
        newValue = target + offset;
        newVel = targetVel + offsetVel;
    }

    #endregion
}
}