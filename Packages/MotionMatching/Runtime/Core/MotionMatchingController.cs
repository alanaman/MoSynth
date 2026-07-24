using System;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine.Serialization;

namespace MotionMatching
{

// Simulation bone is the transform
public class MotionMatchingController : MonoBehaviour
{
    public event Action OnSkeletonTransformUpdated;

    [Header("General")] 
    
    public MotionMatchingCharacterController characterController;
    public MotionMatchingData mmData;

    [SerializeReference] [SubclassSelector]
    public MotionMatchingSearch mmSearch = new BvhMotionMatchingSearch();

    public bool lockFPS = true;

    /// <summary>
    /// The interval in seconds between two Motion Matching searches when there are no sudden input changes.
    /// </summary>
    [FormerlySerializedAs("searchTime")] public float searchInterval = 10.0f / 60.0f;


    public bool inertialize = true; // Should inertialize transitions after a big change of the pose

    public bool footLock = true; // Should lock the feet to the ground when contact information is true

    public float footUnlockDistance = 0.2f; // Distance from actual pose to IK target to unlock the feet

    [Tooltip("The time needed to move half of the distance between the source to the target pose")] [Range(0.0f, 1.0f)]
    public float inertializeHalfLife = 0.1f;
    
    [Tooltip("How important is the trajectory (future positions + future directions)")]
    [Range(0.0f, 1.0f)]
    public float responsiveness = 1.0f;

    [Tooltip("How important is the current pose")] [Range(0.0f, 1.0f)]
    public float quality = 1.0f;

    [HideInInspector]
    public float[] featureWeights;


    public float3 Velocity { get; private set; }
    public float3 AngularVelocity { get; private set; }
    public float DatabaseFrameTime { get; private set; }
    private int _databaseFrameRate;
    public PoseSet PoseSet { get; private set; }
    public FeatureSet FeatureSet { get; private set; }
    public float SearchTimeLeft { get; private set; }
    /// <summary>
    /// The transforms of the <see cref="GameObject"/> skeleton controlled
    /// by this <see cref="MotionMatchingController"/>.
    /// </summary>
    public Transform[] SkeletonTransforms { get; private set; }

    public NativeArray<float> QueryFeature => _queryFeature;

    public NativeArray<float> FeaturesWeightsNativeArray { get; private set; }
    
    /// <summary>
    /// Current frame index in the pose/feature set
    /// </summary>
    public int CurrentFrame { get; private set; }
    /// <summary>
    /// Frame before the last Motion Matching Search
    /// </summary>
    public int LastMmSearchFrame { get; private set; } 
    public NativeArray<bool> TagMask { get; private set; }

    private Vector3 _animationSpaceOriginPos;
    private Quaternion _animationSpaceOriginRot;
    private Quaternion _inverseAnimationSpaceOriginRot;
    
    /// <summary>
    /// Position of the transform right after motion matching search
    /// </summary>
    private Vector3 _mmTransformOriginPos;
    /// <summary>
    /// Rotation of the transform right after motion matching search
    /// </summary>
    private Quaternion _mmTransformOriginRot; 
    /// <summary>
    /// Current frame index as float to keep track of variable frame rate
    /// </summary>
    private float _currentFrameTime; 

    private Inertialization _inertialization;

    // Foot Lock
    public bool IsLeftFootContact { get; private set; }

    public bool IsRightFootContact { get; private set; }

    // Target position of the toes
    public float3 LeftToesContactTarget { get; private set; }
    public float3 RightToesContactTarget { get; private set; }
    private float3 _leftFootContact, _rightFootContact; // Position of the foot
    private float3 _leftFootPoleContact, _rightFootPoleContact; // Forward vector of the knee
    private float3 _leftLowerLegLocalForward, _rightLowerLegLocalForward;
    public int LeftToesIndex { get; private set; }
    private int _leftFootIndex;
    private int _leftLowerLegIndex;
    private int _leftUpperLegIndex;

    public int RightToesIndex { get; private set; }
    private int _rightFootIndex;
    private int _rightLowerLegIndex;
    private int _rightUpperLegIndex;

    // Other
    private bool _isDestroyed;
    private NativeArray<float> _queryFeature;

    private void Awake()
    {
        PoseSet = mmData.GetOrImportPoseSet();
        FeatureSet = mmData.GetOrImportFeatureSet();

        // Skeleton
        SkeletonTransforms = new Transform[PoseSet.Skeleton.Joints.Count];
        SkeletonTransforms[0] = transform; // Simulation Bone
        for (var j = 1; j < PoseSet.Skeleton.Joints.Count; j++)
        {
            // Joints
            var joint = PoseSet.Skeleton.Joints[j];
            var t = new GameObject().transform;
            t.name = joint.name;
            t.SetParent(SkeletonTransforms[joint.parentIndex], false);
            t.localPosition = joint.localOffset;
            SkeletonTransforms[j] = t;
        }

        // Inertialization
        _inertialization = new Inertialization(PoseSet.Skeleton);

        // FPS
        DatabaseFrameTime = PoseSet.FrameTime;
        _databaseFrameRate = Mathf.RoundToInt(1.0f / DatabaseFrameTime);
        if (lockFPS)
        {
            Application.targetFrameRate = _databaseFrameRate;
            Debug.Log("[Motion Matching] Updated Target FPS: " + Application.targetFrameRate);
        }
        else
        {
            Application.targetFrameRate = -1;
            Debug.LogWarning(
                "[Motion Matching] LockFPS is not set. Motion Matching will malfunction if the application frame rate is higher than the animation database.");
        }

        // Other initialization
        var numberFeatures = mmData.TrajectoryFeatures.Count + mmData.PoseFeatures.Count +
                             mmData.EnvironmentFeatures.Count;
        if (featureWeights == null || featureWeights.Length != numberFeatures)
        {
            var newWeights = new float[numberFeatures];
            for (var i = 0; i < newWeights.Length; ++i) newWeights[i] = 1.0f;
            for (var i = 0; i < Mathf.Min(featureWeights.Length, newWeights.Length); i++)
                newWeights[i] = featureWeights[i];
            featureWeights = newWeights;
        }

        FeaturesWeightsNativeArray = new NativeArray<float>(FeatureSet.FeatureSize, Allocator.Persistent);
        _queryFeature = new NativeArray<float>(FeatureSet.FeatureSize, Allocator.Persistent);

        // Tags
        TagMask = new NativeArray<bool>(FeatureSet.NumberFeatureVectors, Allocator.Persistent);
        DisableQueryTag();

        // Foot Lock
        var skeleton = PoseSet.Skeleton;
        LeftToesIndex = skeleton.GetJointIndex(HumanBodyBones.LeftToes);
        _leftFootIndex = skeleton.GetJointIndex(HumanBodyBones.LeftFoot);
        _leftLowerLegIndex = skeleton.GetJointIndex(HumanBodyBones.LeftLowerLeg);
        _leftUpperLegIndex = skeleton.GetJointIndex(HumanBodyBones.LeftUpperLeg);
        RightToesIndex = skeleton.GetJointIndex(HumanBodyBones.RightToes);
        _rightFootIndex = skeleton.GetJointIndex(HumanBodyBones.RightFoot);
        _rightLowerLegIndex = skeleton.GetJointIndex(HumanBodyBones.RightLowerLeg);
        _rightUpperLegIndex = skeleton.GetJointIndex(HumanBodyBones.RightUpperLeg);
        _leftLowerLegLocalForward = mmData.GetLocalForward(_leftLowerLegIndex);
        _rightLowerLegLocalForward = mmData.GetLocalForward(_rightLowerLegIndex);

        // Init Pose
        SkeletonTransforms[0].SetPositionAndRotation(characterController.GetWorldInitPosition(),
            quaternion.LookRotation(characterController.GetWorldInitDirection(), Vector3.up));

        // Search first Frame valid (to start with a valid pose)
        for (var i = 0; i < FeatureSet.NumberFeatureVectors; i++)
        {
            if (FeatureSet.IsValidFeature(i) && TagMask[i])
            {
                LastMmSearchFrame = i;
                CurrentFrame = i;
                UpdateAnimationSpaceOrigin();
                break;
            }
        }

        // Search Strategy
        mmSearch ??= MotionMatchingSearch.Default;
        mmSearch.Initialize(FeatureSet, TagMask, FeaturesWeightsNativeArray);
        
    }

    private void OnEnable()
    {
        characterController.OnUpdated += OnCharacterControllerUpdated;
        characterController.OnInputChangedQuickly += OnInputChangedQuickly;
    }

    private void OnDisable()
    {
        characterController.OnUpdated -= OnCharacterControllerUpdated;
        characterController.OnInputChangedQuickly -= OnInputChangedQuickly;
    }

    private void OnCharacterControllerUpdated(float deltaTime)
    {
        PROFILE.BEGIN_SAMPLE_PROFILING("Motion Matching Total");
        if (SearchTimeLeft <= 0)
        {
            // Motion Matching
            var currentDistance = PrepareQueryVector(out var isCurrentValid);
            PROFILE.BEGIN_SAMPLE_PROFILING("Motion Matching Search");
            var bestFrame = mmSearch.FindBestFrame(_queryFeature, currentDistance);
            PROFILE.END_SAMPLE_PROFILING("Motion Matching Search");
            // Check if use current or best
            if (isCurrentValid && bestFrame == -1) bestFrame = CurrentFrame;
            Debug.Assert(bestFrame != -1,
                "Motion Matching is not able to find any valid pose. Maybe the motion database is empty or the query tag used produces an empty set of poses?");
            const int ignoreSurrounding = 20; // ignore near frames
            if (math.abs(bestFrame - CurrentFrame) > ignoreSurrounding)
            {
                // Inertialize
                if (inertialize)
                {
                    _inertialization.PoseTransition(PoseSet, CurrentFrame, bestFrame);
                }

                LastMmSearchFrame = CurrentFrame;
                _currentFrameTime =
                    bestFrame + math.frac(
                        _currentFrameTime); // the fractional part is the error accumulated, add it to the current to avoid drifting
                CurrentFrame = bestFrame;
                UpdateAnimationSpaceOrigin();
            }

            SearchTimeLeft = searchInterval;
        }
        else
        {
            SearchTimeLeft -= deltaTime;
        }

        // Always advance one (bestFrame from motion matching is the best match to the current frame, but we want to move to the next frame)
        // Ideally the applications runs at 1.0f/FrameTime fps (to match the database) however, as this may not happen, we may need to skip some frames
        // from the database, e.g., if 1.0f/FrameTime = 60 and our game runes at 30, we need to advance 2 frames at each update
        // However, as we are using Application.targetFrameRate=1.0f/FrameTime, we do not consider the case where the application runs faster than the database
        _currentFrameTime += _databaseFrameRate * deltaTime; // DatabaseFrameRate / (1.0f / deltaTime)
        CurrentFrame = (int)math.floor(_currentFrameTime);

        UpdateTransformAndSkeleton(CurrentFrame);
        PROFILE.END_SAMPLE_PROFILING("Motion Matching Total");
    }

    private void UpdateAnimationSpaceOrigin()
    {
        PoseSet.GetPose(CurrentFrame, out var mmPose);
        _animationSpaceOriginPos = mmPose.JointLocalPositions[0];
        _animationSpaceOriginRot = mmPose.JointLocalRotations[0];
        _inverseAnimationSpaceOriginRot = Quaternion.Inverse(mmPose.JointLocalRotations[0]);
        _mmTransformOriginPos = SkeletonTransforms[0].position;
        _mmTransformOriginRot = SkeletonTransforms[0].rotation;
    }

    private void OnInputChangedQuickly()
    {
        SearchTimeLeft = 0; // Force search
    }

    private float PrepareQueryVector(out bool isCurrentValid)
    {
        // Weights
        UpdateAndGetFeatureWeights();

        // Init Query Vector
        // TODO: here we are using using the features of the current motion-matched frame.
        // should we use the current character skeleton pose instead?
        FeatureSet.GetFeature(QueryFeature, CurrentFrame);
        FillQueryVector();

        // Get next feature vector (when doing motion-matching search, they need less error than this)
        var currentDistance = float.MaxValue;
        isCurrentValid = FeatureSet.IsValidFeature(CurrentFrame) && TagMask[CurrentFrame];
        if (isCurrentValid)
        {
            currentDistance = 0.0f;
            // the pose is the same... the distance is only the trajectory
            for (var j = 0; j < FeatureSet.PoseOffset; j++)
            {
                var diff = FeatureSet.GetFeatures()[CurrentFrame * FeatureSet.FeatureSize + j] - QueryFeature[j];
                currentDistance += diff * diff * FeaturesWeightsNativeArray[j];
            }
        }

        return currentDistance;
    }

    public void FillQueryVector()
    {
        var offset = 0;
        for (var i = 0; i < mmData.TrajectoryFeatures.Count; i++)
        {
            var feature = mmData.TrajectoryFeatures[i];
            for (var p = 0; p < feature.FramesPrediction.Length; ++p)
            {
                var featureSize = feature.GetSize();
                Debug.Assert(featureSize > 0, "Trajectory feature size must be larger than 0");
                NativeArray<float> featureVector = new(featureSize, Allocator.Temp);
                characterController.GetTrajectoryFeature(feature, p, SkeletonTransforms[0], featureVector);
                for (var j = 0; j < featureSize; j++)
                {
                    _queryFeature[offset + j] = featureVector[j];
                }

                offset += featureSize;
            }
        }

        // Normalize (only trajectory... because current FeatureVector is already normalized)
        FeatureSet.NormalizeTrajectory(QueryFeature);

        if (FeatureSet.EnvironmentOffset.Length > 0)
        {
            offset = FeatureSet.EnvironmentOffset[0];
            for (var i = 0; i < mmData.EnvironmentFeatures.Count; i++)
            {
                var feature = mmData.EnvironmentFeatures[i];
                for (var p = 0; p < feature.FramesPrediction.Length; p++)
                {
                    var featureSize = feature.GetSize();
                    Debug.Assert(featureSize > 0, "Environment feature size must be larger than 0");
                    NativeArray<float> featureVector = new(featureSize, Allocator.Temp);
                    characterController.GetEnvironmentFeature(feature, p, SkeletonTransforms[0], featureVector);
                    for (var j = 0; j < featureSize; j++)
                    {
                        _queryFeature[offset + j] = featureVector[j];
                    }

                    offset += featureSize;
                }
            }
        }
    }

    private void UpdateTransformAndSkeleton(int frameIndex)
    {
        
        PoseSet.GetPose(frameIndex, out var pose);
        if (inertialize)
        {
            _inertialization.Update(pose, inertializeHalfLife, Time.deltaTime);
        }
        
        // Simulation Bone
        float3 previousPosition = SkeletonTransforms[0].position;
        quaternion previousRotation = SkeletonTransforms[0].rotation;
        // animation space to local space
        var localSpacePos = _inverseAnimationSpaceOriginRot * 
                            (pose.JointLocalPositions[0] - _animationSpaceOriginPos);
        var localSpaceRot = _inverseAnimationSpaceOriginRot * pose.JointLocalRotations[0];
        
        // local space to world space
        SkeletonTransforms[0].SetPositionAndRotation(
            _mmTransformOriginRot * localSpacePos + _mmTransformOriginPos,
            math.mul(_mmTransformOriginRot, localSpaceRot));
        // update velocity and angular velocity
        Velocity = ((float3)SkeletonTransforms[0].position - previousPosition) / Time.deltaTime;
        AngularVelocity =
            MathExtensions.AngularVelocity(previousRotation, SkeletonTransforms[0].rotation, Time.deltaTime);
        // Joints
        if (inertialize)
        {
            for (int i = 1; i < _inertialization.InertializedRotations.Length; i++)
            {
                SkeletonTransforms[i].localRotation = _inertialization.InertializedRotations[i];
            }
        }
        else
        {
            for (int i = 1; i < pose.JointLocalRotations.Length; i++)
            {
                SkeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
            }
        }

        // Hips Position
        SkeletonTransforms[1].localPosition =
            inertialize ? _inertialization.InertializedHips : pose.JointLocalPositions[1];
        // Foot Lock
        UpdateFootLock(pose);
        // Post-processing the transforms
        OnSkeletonTransformUpdated?.Invoke();
    }

    private void UpdateFootLock(PoseVector targetPose)
    {
        float3 currentLeftToesPosition = SkeletonTransforms[LeftToesIndex].position;
        float3 currentRightToesPosition = SkeletonTransforms[RightToesIndex].position;
        // Compute input contact position velocity
        var currentLeftToesVelocity = (currentLeftToesPosition - (float3)LeftToesContactTarget) / Time.deltaTime;
        var currentRightToesVelocity = (currentRightToesPosition - (float3)RightToesContactTarget) / Time.deltaTime;
        LeftToesContactTarget = currentLeftToesPosition;
        RightToesContactTarget = currentRightToesPosition;

        // Update Inertializer
        float3 leftContactPosition;
        float3 leftContactVelocity;
        float3 rightContactPosition;
        float3 rightContactVelocity;
        if (IsLeftFootContact)
        {
            _inertialization.UpdateLeftContact(_leftFootContact, float3.zero, inertializeHalfLife, Time.deltaTime, out leftContactPosition, out leftContactVelocity);
        }
        else
        {
            _inertialization.UpdateLeftContact(currentLeftToesPosition, currentLeftToesVelocity, inertializeHalfLife, Time.deltaTime, out leftContactPosition, out leftContactVelocity);
        }

        if (IsRightFootContact)
        {
            _inertialization.UpdateRightContact(_rightFootContact, float3.zero, inertializeHalfLife, Time.deltaTime, out rightContactPosition, out rightContactVelocity);
        }
        else
        {
            _inertialization.UpdateRightContact(currentRightToesPosition, currentRightToesVelocity, inertializeHalfLife, Time.deltaTime, out rightContactPosition, out rightContactVelocity);
        }

        // If the contact point is too far from the current input position
        // unlock the contact
        var unlockLeftContact =
            IsLeftFootContact &&
            (math.length(_leftFootContact - currentLeftToesPosition) > footUnlockDistance);
        var unlockRightContact =
            IsRightFootContact &&
            (math.length(_rightFootContact - currentRightToesPosition) > footUnlockDistance);

        // If the contact was previously inactive and now it is active,
        // transition to the locked contact state
        // Also, make sure the inertialization returns an almost 0 velocity before locking
        if (!IsLeftFootContact && targetPose.LeftFootContact &&
            math.length(leftContactVelocity) < mmData.ContactVelocityThreshold)
        {
            // Contact point is the current position of the foot
            // projected onto the ground + foot height
            IsLeftFootContact = true;
            _leftFootContact = leftContactPosition;
            // LeftFootContact.y =  // TODO: Add foot height
            var leftLowerLeg = SkeletonTransforms[_leftLowerLegIndex];
            _leftFootPoleContact = math.mul(leftLowerLeg.rotation, _leftLowerLegLocalForward);

            if (inertialize)
            {
                _inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity,
                    _leftFootContact, float3.zero);
            }
            else
            {
                _inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity,
                    currentLeftToesPosition, currentLeftToesVelocity);
            }
        }
        // If we need to unlock or previously in contact but now not
        // we transition to the input position
        else if (unlockLeftContact || (IsLeftFootContact && !targetPose.LeftFootContact))
        {
            IsLeftFootContact = false;

            if (inertialize)
            {
                _inertialization.LeftContactTransition(_leftFootContact, float3.zero, currentLeftToesPosition,
                    currentLeftToesVelocity);
            }
            else
            {
                _inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity,
                    currentLeftToesPosition, currentLeftToesVelocity);
            }
        }

        // Same for Right Foot
        if (!IsRightFootContact && targetPose.RightFootContact &&
            math.length(rightContactVelocity) < mmData.ContactVelocityThreshold)
        {
            IsRightFootContact = true;
            _rightFootContact = rightContactPosition;
            // RightFootContact.y = 0.0f;
            var rightLowerLeg = SkeletonTransforms[_rightLowerLegIndex];
            _rightFootPoleContact = math.mul(rightLowerLeg.rotation, _rightLowerLegLocalForward);

            if (inertialize)
            {
                _inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity,
                    _rightFootContact, float3.zero);
            }
            else
            {
                _inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity,
                    currentRightToesPosition, currentRightToesVelocity);
            }
        }
        else if (unlockRightContact || (IsRightFootContact && !targetPose.RightFootContact))
        {
            IsRightFootContact = false;

            if (inertialize)
            {
                _inertialization.RightContactTransition(_rightFootContact, float3.zero, currentRightToesPosition,
                    currentRightToesVelocity);
            }
            else
            {
                _inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity,
                    currentRightToesPosition, currentRightToesVelocity);
            }
        }

        // IK to place the foot
        if (footLock)
        {
            // Left Foot IK
            TwoJointIK.Solve(
                (Vector3)leftContactPosition + (SkeletonTransforms[_leftFootIndex].position -
                                                SkeletonTransforms[LeftToesIndex].position),
                SkeletonTransforms[_leftUpperLegIndex],
                SkeletonTransforms[_leftLowerLegIndex],
                SkeletonTransforms[_leftFootIndex],
                _leftFootPoleContact);
            // Right Foot IK
            TwoJointIK.Solve(
                (Vector3)rightContactPosition + (SkeletonTransforms[_rightFootIndex].position -
                                                 SkeletonTransforms[RightToesIndex].position),
                SkeletonTransforms[_rightUpperLegIndex],
                SkeletonTransforms[_rightLowerLegIndex],
                SkeletonTransforms[_rightFootIndex],
                _rightFootPoleContact);
        }
    }

    void UpdateSkeletonTransformsFromPose(PoseVector targetPose)
    {
        // animation space to local space
        var localSpacePos = math.mul(_inverseAnimationSpaceOriginRot,
            targetPose.JointLocalPositions[0] - _animationSpaceOriginPos);
        var localSpaceRot = math.mul(_inverseAnimationSpaceOriginRot, targetPose.JointLocalRotations[0]);
        
        // Simulation Bone
        float3 currentPos = SkeletonTransforms[0].position;
        quaternion currentRot = SkeletonTransforms[0].rotation;
        // local space to world space
        var newPos = _mmTransformOriginRot * localSpacePos + _mmTransformOriginPos;
        var newRot = _mmTransformOriginRot * localSpaceRot;

        Velocity = ((float3)SkeletonTransforms[0].position - currentPos) / Time.deltaTime;
        AngularVelocity = MathExtensions.AngularVelocity(
            currentRot, SkeletonTransforms[0].rotation, Time.deltaTime);
        
        SkeletonTransforms[0].SetPositionAndRotation(newPos,newRot);

        for (var i = 1; i < targetPose.JointLocalRotations.Length; i++)
        {
            SkeletonTransforms[i].localRotation = targetPose.JointLocalRotations[i];
        }
        
        // Hips Position
        SkeletonTransforms[1].localPosition =
            inertialize ? _inertialization.InertializedHips : targetPose.JointLocalPositions[1];
    }
    
    /// <summary>
    /// Disables any previous set tag or query so searches are performed over the entire pose set
    /// </summary>
    public void DisableQueryTag()
    {
        var job = new DisableTagBurst
        {
            TagMask = TagMask,
        };
        job.Schedule().Complete();
        OnInputChangedQuickly();
    }

    /// <summary>
    /// Motion Matching will only search over those poses belonging to the tag
    /// </summary>
    public void SetQueryTag(string tagName)
    {
        var animTag = PoseSet.GetTag(tagName);
        // TODO: cache results to avoid duplicated computations...
        var job = new SetTagBurst
        {
            MaximumFramesPrediction = PoseSet.MaximumFramesPrediction,
            TagMask = TagMask,
            StartRanges = animTag.GetStartRanges(),
            EndRanges = animTag.GetEndRanges(),
        };
        job.Schedule().Complete();
        OnInputChangedQuickly();
    }

    /// <summary>
    /// Motion Matching will only search over those poses belonging to the query tag
    /// </summary>
    public void SetQueryTag(QueryTag query)
    {
        query.ComputeRanges(PoseSet);
        var job = new SetTagBurst
        {
            MaximumFramesPrediction = PoseSet.MaximumFramesPrediction,
            TagMask = TagMask,
            StartRanges = query.GetStartRanges(),
            EndRanges = query.GetEndRanges(),
        };
        job.Schedule().Complete();
        OnInputChangedQuickly();
    }

    /// <summary>
    /// Adds an offset to the current transform space (useful to move the character to a different position)
    /// Simply changing the transform won't work because motion matching applies root motion based on the current motion matching search space
    /// </summary>
    public void SetPosAdjustment(Vector3 posAdjustment)
    {
        _mmTransformOriginPos += posAdjustment;
    }

    /// <summary>
    /// Adds a rot offset to the current transform space (useful to rotate the character to a different direction)
    /// Simply changing the transform won't work because motion matching applies root motion based on the current motion matching search space
    /// </summary>
    public void SetRotAdjustment(Quaternion rotAdjustment)
    {
        _mmTransformOriginRot = rotAdjustment * _mmTransformOriginRot;
    }

    public void SetCurrentFrame(int frame)
    {
        CurrentFrame = frame;
    }

    public NativeArray<float> UpdateAndGetFeatureWeights()
    {
        var featuresWeightsNativeArray = FeaturesWeightsNativeArray;
        var offset = 0;
        for (var i = 0; i < mmData.TrajectoryFeatures.Count; i++)
        {
            var feature = mmData.TrajectoryFeatures[i];
            var featureSize = feature.GetSize();
            var weight = featureWeights[i] * responsiveness;
            for (var p = 0; p < feature.FramesPrediction.Length; ++p)
            {
                for (var f = 0; f < featureSize; f++)
                {
                    featuresWeightsNativeArray[offset + f] = weight;
                }

                offset += featureSize;
            }
        }

        for (var i = 0; i < mmData.PoseFeatures.Count; i++)
        {
            var weight = featureWeights[i + mmData.TrajectoryFeatures.Count] * quality;
            featuresWeightsNativeArray[offset + 0] = weight;
            featuresWeightsNativeArray[offset + 1] = weight;
            featuresWeightsNativeArray[offset + 2] = weight;
            offset += 3;
        }

        for (var i = 0; i < mmData.EnvironmentFeatures.Count; i++)
        {
            var feature = mmData.EnvironmentFeatures[i];
            var featureSize = feature.GetSize();
            var baseWeight = featureWeights[i + mmData.TrajectoryFeatures.Count + mmData.PoseFeatures.Count];
            for (var p = 0; p < feature.FramesPrediction.Length; ++p)
            {
                for (var f = 0; f < featureSize; f++)
                {
                    featuresWeightsNativeArray[offset + f] = baseWeight;
                }

                offset += featureSize;
            }
        }

        return featuresWeightsNativeArray;
    }

    public float3 GetMainPositionFeature(int trajectoryIndex)
    {
        float3 characterOrigin = SkeletonTransforms[0].position;
        float3 characterForward = SkeletonTransforms[0].forward;
        var characterRot = quaternion.LookRotation(characterForward, math.up());
        // Find Main Position Trajectory Index
        var t = -1;
        for (var i = 0; i < mmData.TrajectoryFeatures.Count; i++)
        {
            if (mmData.TrajectoryFeatures[i].IsMainPositionFeature)
            {
                t = i;
                break;
            }
        }

        if (t == -1)
        {
            Debug.LogError("[Motion Matching] No Main Position Trajectory Feature found.");
            return characterOrigin;
        }

        var value = FeatureSet.Get3DValuePositionOrDirectionFeature(mmData.TrajectoryFeatures[t], CurrentFrame, t,
            trajectoryIndex, isEnvironment: false);
        value = characterOrigin + math.mul(characterRot, value);
        return value;
    }

    public float4 GetEnvironmentFeature(string featureName, int trajectoryIndex)
    {
        float3 characterForward = SkeletonTransforms[0].forward;
        var characterRot = quaternion.LookRotation(characterForward, math.up());
        var t = -1;
        for (var i = 0; i < mmData.EnvironmentFeatures.Count; i++)
        {
            if (mmData.EnvironmentFeatures[i].Name == featureName)
            {
                t = i;
                break;
            }
        }

        if (t == -1)
        {
            Debug.LogError("[Motion Matching] No Environment Feature with name " + featureName + " found.");
            return float4.zero;
        }

        var value = FeatureSet.Get4DEnvironmentFeature(CurrentFrame, t, trajectoryIndex);
        var primaryDistance = value.x;
        var secondaryDistance = value.y;
        float3 primaryAxisUnitCharacterSpace = new(value.z, 0.0f, value.w);
        var primaryAxisUnitWorldSpace = math.mul(characterRot, primaryAxisUnitCharacterSpace);
        float2 primaryAxisUnit = new(primaryAxisUnitWorldSpace.x, primaryAxisUnitWorldSpace.z);
        float2 secondaryAxisUnit = new(-primaryAxisUnit.y, primaryAxisUnit.x);
        return new float4(primaryAxisUnit * primaryDistance, secondaryAxisUnit * secondaryDistance);
    }

    private void OnDestroy()
    {
        if (_isDestroyed) return;
        _isDestroyed = true;
        mmData.Dispose();
        mmSearch.Dispose();
        if (QueryFeature.IsCreated) QueryFeature.Dispose();
        if (FeaturesWeightsNativeArray.IsCreated) FeaturesWeightsNativeArray.Dispose();
        if (TagMask.IsCreated) TagMask.Dispose();
    }

    private void OnApplicationQuit()
    {
        OnDestroy();
    }

#if UNITY_EDITOR

    public Matrix4x4 GetSimulationBoneWorldSpaceTransform(PoseVector futurePose)
    {
        // animation space to local space
        var localSpacePos = math.mul(_inverseAnimationSpaceOriginRot,
            futurePose.JointLocalPositions[0] - _animationSpaceOriginPos);
        var localSpaceRot = math.mul(_inverseAnimationSpaceOriginRot, futurePose.JointLocalRotations[0]);
        // local space to world space
        var simulationBonePos = _mmTransformOriginRot * localSpacePos + _mmTransformOriginPos;
        var simulationBoneRot = math.mul(_mmTransformOriginRot, localSpaceRot);

        var simulationBoneTransform = Matrix4x4.TRS(simulationBonePos, simulationBoneRot, Vector3.one);
        return simulationBoneTransform;
    }
#endif
}
}