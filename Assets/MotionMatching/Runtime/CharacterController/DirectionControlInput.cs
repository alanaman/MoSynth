using System;
using AnimationTools;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using UnityEngine.Serialization;

namespace MotionMatching
{
// Adjustment between Character Controller and Motion Matching Character Entity
/* https://theorangeduck.com/page/code-vs-data-driven-displacement */

public class DirectionControlInput : MotionMatchingControlInput
{
    [Header("Features")] public string trajectoryPositionFeatureName = "FuturePosition";

    public string trajectoryDirectionFeatureName = "FutureDirection";

    [Header("General")] public float maxSpeed = 1.0f;
    [Range(0.0f, 1.0f)] public float responsivenessPositions = 0.75f;
    [Range(0.0f, 1.0f)] public float responsivenessDirections = 0.75f;
    public float minimumVelocityClamp = 0.01f;

    [Tooltip(
        "Controls when to consider that the input has suddenly changed. Used to recompute MotionMatching. -1.0f: Never. 1.0f: Always")]
    [Range(-1.0f, 1.0f)]
    public float inputBigChangeThreshold = 0.5f;

    [Range(0.0f, 2.0f)] public float
        positionAdjustmentHalflife =
            0.1f; // Time needed to move half of the distance between MotionMatching and the CharacterController

    [FormerlySerializedAs("rotationAdjustmentHalflife")] [Range(0.0f, 2.0f)]
    public float rotationAdjustmentHalfLife = 0.1f;

    [Range(0.0f, 2.0f)] public float
        posMaximumAdjustmentRatio =
            0.1f; // Ratio between the adjustment and the character's velocity to clamp the adjustment

    [Range(0.0f, 2.0f)] public float
        rotMaximumAdjustmentRatio =
            0.1f; // Ratio between the adjustment and the character's velocity to clamp the adjustment

    public bool doClamping = true;

    [Range(0.0f, 2.0f)] public float
        maxDistanceMmAndCharacterController = 0.1f; // Max distance between MotionMatching and the CharacterController

    [Header("DEBUG")] public bool debugCurrent = true;
    public bool debugPrediction = true;
    public bool debugClamping = true;
    // --------------------------------------------------------------------------

    // PRIVATE ------------------------------------------------------------------
    // Input --------------------------------------------------------------------
    private float2 _inputMovement;

    private bool _orientationFixed;

    // Rotation and Predicted Rotation ------------------------------------------
    private quaternion _desiredRotation; // Desired Rotation/Direction
    private quaternion[] _predictedRotations;
    private float3 _angularVelocity;

    private float3[] _predictedAngularVelocities;

    // Position and Predicted Position ------------------------------------------
    private float2[] _predictedPosition;
    private float2 _velocity;
    private float2[] _predictedVelocity;
    private float2 _acceleration;

    private float2[] _predictedAcceleration;

    // Features -----------------------------------------------------------------
    private int _trajectoryPosFeatureIndex;
    private int _trajectoryRotFeatureIndex;
    private int[] _trajectoryPosPredictionFrames;
    private int[] _trajectoryRotPredictionFrames;

    private int NumberPredictionPos
    {
        get { return _trajectoryPosPredictionFrames.Length; }
    }

    private int NumberPredictionRot
    {
        get { return _trajectoryRotPredictionFrames.Length; }
    }

    private void Start()
    {
        // Get the feature indices
        _trajectoryPosFeatureIndex = -1;
        _trajectoryRotFeatureIndex = -1;
        var mmData = motionSynthesizer.GetMmData();
        for (var i = 0; i < mmData.trajectoryFeatures.Count; ++i)
        {
            if (mmData.trajectoryFeatures[i].name == trajectoryPositionFeatureName)
                _trajectoryPosFeatureIndex = i;
            if (mmData.trajectoryFeatures[i].name == trajectoryDirectionFeatureName)
                _trajectoryRotFeatureIndex = i;
        }

        Debug.Assert(_trajectoryPosFeatureIndex != -1, "Trajectory Position Feature not found");
        Debug.Assert(_trajectoryRotFeatureIndex != -1, "Trajectory Direction Feature not found");

        _trajectoryPosPredictionFrames =
            mmData.trajectoryFeatures[_trajectoryPosFeatureIndex].predictionFrames;
        _trajectoryRotPredictionFrames =
            mmData.trajectoryFeatures[_trajectoryRotFeatureIndex].predictionFrames;
        // TODO: generalize this... allow different number of prediction frames for different features
        Debug.Assert(_trajectoryPosPredictionFrames.Length == _trajectoryRotPredictionFrames.Length,
            "Trajectory Position and Trajectory Direction Prediction Frames must be the same for SpringCharacterController");
        for (var i = 0; i < _trajectoryPosPredictionFrames.Length; ++i)
        {
            Debug.Assert(_trajectoryPosPredictionFrames[i] == _trajectoryRotPredictionFrames[i],
                "Trajectory Position and Trajectory Direction Prediction Frames must be the same for SpringCharacterController");
        }

        _predictedPosition = new float2[NumberPredictionPos];
        _predictedVelocity = new float2[NumberPredictionPos];
        _predictedAcceleration = new float2[NumberPredictionPos];
        _desiredRotation = quaternion.LookRotation(transform.forward, transform.up);
        _predictedRotations = new quaternion[NumberPredictionRot];
        _predictedAngularVelocities = new float3[NumberPredictionRot];
    }

    // Input a change in the movement direction
    public void SetMovementDirection(Vector2 movementDirection)
    {
        var prevInputMovement = _inputMovement;
        _inputMovement = movementDirection;
        // Desired Rotation
        if (!_orientationFixed && math.length(movementDirection) > 0.0001f)
        {
            var desiredDirection = math.normalize(movementDirection);
            _desiredRotation =
                quaternion.LookRotation(new float3(desiredDirection.x, 0.0f, desiredDirection.y), transform.up);
        }

        // Input Changed Quickly
        if (math.dot(prevInputMovement, _inputMovement) < inputBigChangeThreshold)
        {
            NotifyInputChangedQuickly();
        }
    }

    public void SwapFixOrientation()
    {
        _orientationFixed = !_orientationFixed;
    }

    protected override void OnUpdate()
    {
        // Rotations
        quaternion currentRotation = transform.rotation;
        PredictRotations(currentRotation, DatabaseDeltaTime);
        // Update Current Rotation
        var newRot = ComputeNewRot(currentRotation);

        // Positions
        var desiredSpeed = _inputMovement * maxSpeed;
        var currentPos = new float2(transform.position.x, transform.position.z);
        // Predict
        PredictPositions(currentPos, desiredSpeed, DatabaseDeltaTime);
        // Update Current Position
        var newPos = ComputeNewPos(currentPos, desiredSpeed);

        // Update Character Controller
        if (math.lengthsq(_velocity) > minimumVelocityClamp * minimumVelocityClamp)
        {
            // Update Transform
            transform.position = new float3(newPos.x, transform.position.y, newPos.y);
            transform.rotation = newRot;
        }

        // if (DoClamping) ClampMotionMatching();
    }

    private void PredictRotations(quaternion currentRotation, float averagedDeltaTime)
    {
        for (var i = 0; i < NumberPredictionRot; i++)
        {
            // Init Predicted values
            _predictedRotations[i] = currentRotation;
            _predictedAngularVelocities[i] = _angularVelocity;
            // Predict
            Spring.SimpleSpringDamperImplicit(ref _predictedRotations[i], ref _predictedAngularVelocities[i],
                _desiredRotation, 1.0f - responsivenessDirections,
                _trajectoryRotPredictionFrames[i] * averagedDeltaTime);
        }
    }

    /* https://theorangeduck.com/page/spring-roll-call#controllers */
    private void PredictPositions(float2 currentPos, float2 desiredSpeed, float averagedDeltaTime)
    {
        var lastPredictionFrames = 0;
        for (var i = 0; i < NumberPredictionPos; ++i)
        {
            if (i == 0)
            {
                _predictedPosition[i] = currentPos;
                _predictedVelocity[i] = _velocity;
                _predictedAcceleration[i] = _acceleration;
            }
            else
            {
                _predictedPosition[i] = _predictedPosition[i - 1];
                _predictedVelocity[i] = _predictedVelocity[i - 1];
                _predictedAcceleration[i] = _predictedAcceleration[i - 1];
            }

            var diffPredictionFrames = _trajectoryPosPredictionFrames[i] - lastPredictionFrames;
            lastPredictionFrames = _trajectoryPosPredictionFrames[i];
            Spring.CharacterPositionUpdate(ref _predictedPosition[i], ref _predictedVelocity[i],
                ref _predictedAcceleration[i],
                desiredSpeed, 1.0f - responsivenessPositions, diffPredictionFrames * averagedDeltaTime);
        }
    }

    private quaternion ComputeNewRot(quaternion currentRotation)
    {
        var newRotation = currentRotation;
        Spring.SimpleSpringDamperImplicit(ref newRotation, ref _angularVelocity, _desiredRotation,
            1.0f - responsivenessDirections, Time.deltaTime);
        return newRotation;
    }

    private float2 ComputeNewPos(float2 currentPos, float2 desiredSpeed)
    {
        var newPos = currentPos;
        Spring.CharacterPositionUpdate(ref newPos, ref _velocity, ref _acceleration, desiredSpeed,
            1.0f - responsivenessPositions, Time.deltaTime);
        return newPos;
    }

    private void ClampMotionMatching()
    {
        // Clamp Position
        float3 characterController = transform.position;
        var mmPos = motionSynthesizer.RootPosition;
        if (math.distance(characterController, mmPos) > maxDistanceMmAndCharacterController)
        {
            float3 newMotionMatchingPos =
                maxDistanceMmAndCharacterController * math.normalize(mmPos - characterController) +
                characterController;
            motionSynthesizer.SetPosAdjustment(newMotionMatchingPos - mmPos);
        }
    }

    private void AdjustCharacterPosition()
    {
        float3 characterController = transform.position;
        var mmPos = motionSynthesizer.RootPosition;
        var differencePosition = characterController - mmPos;
        // Damp the difference using the adjustment halflife and dt
        var adjustmentPosition =
            Spring.DampAdjustmentImplicit(differencePosition, positionAdjustmentHalflife, Time.deltaTime);
        // Clamp adjustment if the length is greater than the character velocity
        // multiplied by the ratio
        var maxLength = posMaximumAdjustmentRatio * math.length(motionSynthesizer.RootVelocity) * Time.deltaTime;
        if (math.length(adjustmentPosition) > maxLength)
        {
            adjustmentPosition = maxLength * math.normalize(adjustmentPosition);
        }

        // Move the simulation bone towards the simulation object
        motionSynthesizer.SetPosAdjustment(adjustmentPosition);
    }

    private void AdjustCharacterRotation()
    {
        quaternion characterController = transform.rotation;
        var mmRot = motionSynthesizer.RootRotation;
        // Find the difference in rotation (from character to simulation object)
        // Note: if numerically unstable, try quaternion.Normalize(quaternion.Inverse(characterController) * motionMatching)
        var differenceRotation = math.mul(math.inverse(mmRot), characterController);
        // Damp the difference using the adjustment halflife and dt
        var adjustmentRotation =
            Spring.DampAdjustmentImplicit(differenceRotation, rotationAdjustmentHalfLife, Time.deltaTime);
        // Clamp adjustment if the length is greater than the character angular velocity
        // multiplied by the ratio
        var maxLength = rotMaximumAdjustmentRatio * math.length(motionSynthesizer.RootAngularVelocity) * Time.deltaTime;
        if (math.length(MathExtensions.QuaternionToScaledAngleAxis(adjustmentRotation)) > maxLength)
        {
            adjustmentRotation = MathExtensions.QuaternionFromScaledAngleAxis(
                maxLength * math.normalize(
                    MathExtensions.QuaternionToScaledAngleAxis(adjustmentRotation)));
        }

        // Rotate the simulation bone towards the simulation object
        motionSynthesizer.SetRotAdjustment(adjustmentRotation);
    }

    public quaternion GetCurrentRotation()
    {
        return transform.rotation;
    }

    public override float3 GetPosition()
    {
        return transform.position;
    }

    // TODO: the trajectory construction should be inside the animation system
    // and not the character controller. Move it

    public override void GetTrajectoryFeature(
        TrajectoryFeatureChannel feature, int index,
        Transform character, Span<float> output
    )
    {
        if (feature.name == "FutureSphere")
        {
            output[0] = 0.0f;
        }
        else
        {
            if (!feature.simulationBone) Debug.Assert(false, "Trajectory should be computed using the SimulationBone");
            switch (feature.featureType)
            {
                case TrajectoryFeatureChannel.Type.Position:
                    var world = _predictedPosition[index];
                    float3 local = character.InverseTransformPoint(new float3(world.x, 0.0f, world.y));
                    output[0] = local.x;
                    output[1] = local.z;
                    break;
                case TrajectoryFeatureChannel.Type.Direction:
                    var dirProjected = GetWorldSpaceDirectionPrediction(index);
                    float3 localDir =
                        character.InverseTransformDirection(new Vector3(dirProjected.x, 0.0f, dirProjected.y));
                    output[0] = localDir.x;
                    output[1] = localDir.z;
                    break;
                default:
                    Debug.Assert(false, "Unknown feature type: " + feature.featureType);
                    break;
            }
        }
    }

    private float2 GetWorldSpaceDirectionPrediction(int index)
    {
        var dir = math.mul(_predictedRotations[index], new float3(0, 0, 1));
        return math.normalize(new float2(dir.x, dir.z));
    }

    public override float3 GetWorldInitPosition()
    {
        return transform.position;
    }

    public override float3 GetWorldInitDirection()
    {
        return transform.forward;
    }

    public override float GetTargetSpeed()
    {
        return math.length(_predictedVelocity[^1]);
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        const float radius = 0.05f;
        const float vectorReduction = 0.5f;
        const float verticalOffset = 0.05f;
        var transformPos = (Vector3)GetPosition() + Vector3.up * verticalOffset;
        if (debugCurrent)
        {
            // Draw Current Position & Velocity
            Gizmos.color = new Color(1.0f, 0.3f, 0.1f, 1.0f);
            Gizmos.DrawSphere(transformPos, radius);
            GizmosExtensions.DrawLine(transformPos,
                transformPos + ((Quaternion)GetCurrentRotation() * Vector3.forward) * vectorReduction, 3);
        }

        if (_predictedPosition == null || _predictedRotations == null) return;

        if (debugPrediction)
        {
            // Draw Predicted Position & Velocity
            Gizmos.color = new Color(0.6f, 0.3f, 0.8f, 1.0f);
            for (var i = 0; i < _predictedPosition.Length; ++i)
            {
                var predictedPos = new float3(_predictedPosition[i].x, verticalOffset, _predictedPosition[i].y);
                var predictedDir = GetWorldSpaceDirectionPrediction(i);
                var predictedDir3D = new float3(predictedDir.x, 0.0f, predictedDir.y);
                Gizmos.DrawSphere(predictedPos, radius);
                GizmosExtensions.DrawLine(predictedPos, predictedPos + predictedDir3D * vectorReduction, 3);
            }
        }

        if (debugClamping)
        {
            // Draw Clamp Circle
            if (doClamping)
            {
                Gizmos.color = new Color(0.1f, 1.0f, 0.1f, 1.0f);
                GizmosExtensions.DrawWireCircle(transformPos, maxDistanceMmAndCharacterController, quaternion.identity);
            }
        }
    }
#endif
}
}