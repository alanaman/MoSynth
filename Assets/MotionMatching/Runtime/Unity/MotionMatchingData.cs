using System;
using System.Collections.Generic;
using AnimationTools;
using UnityEngine;
using Unity.Mathematics;
using System.IO;

namespace MotionMatching
{
/// <summary>
/// Defines all data used for Motion Matching in one avatar
/// Contains animation clips, feature definitions, and other data
/// </summary>
[CreateAssetMenu(fileName = "MotionMatchingData", menuName = "MotionMatching/MotionMatchingData")]
public class MotionMatchingData : ScriptableObject, IPoseSetSource
{
    [SerializeField]
    public List<AnnotatedAnimationClip> animationClips = new();

    // TODO: Implement Savitzky-Golay filter or similar low-pass filter in Unity (before I was using Python implementation)
    //public bool SmoothSimulationBone; // Smooth the simulation bone (articial root added during pose extraction) using Savitzky-Golay filter
    public float
        contactVelocityThreshold =
            0.15f; // Minimum velocity of the foot to be considered in movement and not in contact with the ground

    [SerializeField]
    [Tooltip("Bone whose velocity drives foot-contact detection; leave unset to pick by name (LeftToe/RightToe).")]
    private BoneTransform leftContactBone = new();

    [SerializeField]
    [Tooltip("Bone whose velocity drives foot-contact detection; leave unset to pick by name (LeftToe/RightToe).")]
    private BoneTransform rightContactBone = new();

    public List<TrajectoryFeatureChannel> trajectoryFeatures = new();

    public List<PoseFeatureChannel> poseFeatures = new();

    private PoseSet _poseSet;

    public FeatureSet FeatureSet
    {
        get => _featureSet;
        private set => _featureSet = value;
    }

    // Information extracted from the rig's rest pose
    [SerializeField]
    private float3[] jointsLocalForward; // Local forward vector of each joint

    private FeatureSet _featureSet;
    public bool JointsLocalForwardError => jointsLocalForward == null;

    // IPoseSetSource --- the subset of this asset the pose-database pipeline actually reads.
    // Exposed as properties so a consumer that only wants a pose database (MotionField's config)
    // can supply one without also being a full Motion Matching asset.
    public List<AnnotatedAnimationClip> AnimationClips => animationClips;
    public float ContactVelocityThreshold => contactVelocityThreshold;
    public string LeftContactBoneName => leftContactBone?.BoneName;
    public string RightContactBoneName => rightContactBone?.BoneName;

    /// <summary>
    /// Frames of lookahead the longest trajectory feature needs. Poses closer than this to the end
    /// of a clip cannot be used for prediction.
    /// </summary>
    public int MaximumFramesPrediction
    {
        get
        {
            int maximum = 0;
            foreach (var t in trajectoryFeatures)
            {
                if (t.predictionFrames.Length > 0 && t.predictionFrames[^1] > maximum)
                {
                    maximum = t.predictionFrames[^1];
                }
            }

            return maximum;
        }
    }

    public PoseSet GetOrImportPoseSet()
    {
        if (_poseSet == null)
        {
            PROFILE.BEGIN_SAMPLE_PROFILING("Pose Import");
            PoseSerializer serializer = new PoseSerializer();
            if (!serializer.Deserialize(GetAssetPath(), name, out PoseSet poseSet))
            {
                Debug.LogWarning("Failed to read pose set. Creating it in runtime instead.");
                ImportPoseSet();
#if UNITY_EDITOR
                PROFILE.BEGIN_SAMPLE_PROFILING("Pose Serialize");
                PoseSerializer poseSerializer = new PoseSerializer();
                poseSerializer.Serialize(_poseSet, GetAssetPath(), this.name);
                PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Serialize");
#endif
            }
            else
            {
                _poseSet = poseSet;
            }

            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Import");
        }

        return _poseSet;
    }

    public void ImportPoseSet()
    {
        _poseSet = new PoseSet();
        _poseSet.SetSkeletonFromBvh(animationClips[0].Skeleton);
        for (int i = 0; i < animationClips.Count; i++)
        {
            // Extract poses
            if (!PoseExtractor.Extract(animationClips[i], _poseSet, this))
            {
                Debug.LogWarning("[FeatureDebug] Failed to extract poseSet from AnimationDat. Animation Index: " + i);
            }
        }

        _poseSet.ConvertTagsToNativeArrays();
        Debug.Log("Number of poses: " + _poseSet.NumberPoses);
    }

    public FeatureSet GetOrImportFeatureSet()
    {
        if (FeatureSet == null)
        {
            PROFILE.BEGIN_SAMPLE_PROFILING("Feature Import");
            FeatureSerializer serializer = new FeatureSerializer();
            if (!serializer.Deserialize(GetAssetPath(), name, this, out FeatureSet featureSet))
            {
                Debug.LogWarning("Failed to read feature set. Creating it in runtime instead.");
                ImportFeatureSet();
#if UNITY_EDITOR
                PROFILE.BEGIN_SAMPLE_PROFILING("Feature Serialize");
                FeatureSerializer featureSerializer = new FeatureSerializer();
                featureSerializer.Serialize(FeatureSet, this, GetAssetPath(), this.name);
                PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Serialize");
#endif
            }
            else
            {
                FeatureSet = featureSet;
            }

            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Import");
        }

        return FeatureSet;
    }

    public void ImportFeatureSet()
    {
        _poseSet = GetOrImportPoseSet();
        FeatureSet = new FeatureSet(this);
        FeatureSet.Extract(_poseSet, this);
        FeatureSet.NormalizeFeatures();
    }

    /// <summary>
    /// Per-joint local forward axis, derived from the rig's rest pose. The exported FBX rig is a
    /// T-Pose facing Unity forward, so the character's forward is +Z and its right is +X; each joint
    /// stores whichever of those axes it should be considered to face, pulled into its own local frame.
    /// </summary>
    public void ComputeJointsLocalForward()
    {
        var skeleton = animationClips[0].Skeleton;

        jointsLocalForward = new float3[skeleton.BoneCount + 1]; // +1 for the simulation bone
        jointsLocalForward[0] = math.forward();

        for (var i = 1; i < jointsLocalForward.Length; i++)
        {
            var boneIndex = i - 1;
            var boneName = skeleton.GetBone(boneIndex).name;

            // Arms point sideways in a T-Pose, so their "forward" is the character's right axis
            var worldForward = math.forward();
            if (BoneNameConventions.IsLeftArmBone(boneName))
            {
                worldForward = -math.right();
            }
            else if (BoneNameConventions.IsRightArmBone(boneName))
            {
                worldForward = math.right();
            }

            jointsLocalForward[i] =
                math.mul(math.inverse(skeleton.RestCharacterRotation(boneIndex)), worldForward);
        }
    }

    /// <summary>
    /// Returns the local forward vector of the given joint index (after adding simulation bone).
    /// Vector computed from the rig's rest pose by <see cref="ComputeJointsLocalForward"/>.
    /// </summary>
    public float3 GetLocalForward(int jointIndex)
    {
        Debug.Assert(!JointsLocalForwardError, "JointsLocalForward is not initialized");
        return jointsLocalForward[jointIndex];
    }

    public string GetAssetPath()
    {
        string path = Path.Combine(Application.streamingAssetsPath, "MMDatabases", name);
#if UNITY_EDITOR
        if (!Directory.Exists(path))
        {
            Directory.CreateDirectory(path);
        }
#endif
        return path;
    }
}
}