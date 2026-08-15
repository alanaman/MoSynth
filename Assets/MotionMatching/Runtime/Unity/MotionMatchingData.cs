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
/// Contains animation clips, mapping between the skeleton and Mecanim, and other data
/// </summary>
[CreateAssetMenu(fileName = "MotionMatchingData", menuName = "MotionMatching/MotionMatchingData")]
public class MotionMatchingData : ScriptableObject, IPoseSetSource
{
    // TODO: DefaultHipsForward... detect/suggest automatically? try to fix automatically at BVHAnimation level? 
    // (if it is fixed some code can be deleted... all code related to DefaultHipsForward and in the UpdateTransform() when correcting the hips forward)

    [SerializeField]
    public List<AnnotatedAnimationClip> animationClips = new();

    [Tooltip("Animation with T-Pose, Animation with a T-Pose in the first frame, used for retargeting")]
    public AnnotatedAnimationClip tPoseAnimationClip; // Animation with a TPose in the first frame, used for retargeting

    public float3
        hipsForwardLocalVector = new(0, 0, 1); // Local vector (axis) pointing in the forward direction of the hips

    public float3 hipsUpLocalVector = new(0, 1, 0); // Local vector (axis) pointing in the up direction of the hips

    // TODO: Implement Savitzky-Golay filter or similar low-pass filter in Unity (before I was using Python implementation)
    //public bool SmoothSimulationBone; // Smooth the simulation bone (articial root added during pose extraction) using Savitzky-Golay filter
    public float
        contactVelocityThreshold =
            0.15f; // Minimum velocity of the foot to be considered in movement and not in contact with the ground

    public List<JointToMecanim> animationChannelToMecanim = new();

    public List<TrajectoryFeatureChannel> trajectoryFeatures = new();

    public List<PoseFeatureChannel> poseFeatures = new();

    private PoseSet _poseSet;

    public FeatureSet FeatureSet
    {
        get => _featureSet;
        private set => _featureSet = value;
    }

    // Information extracted form T-Pose
    [SerializeField]
    private float3[] jointsLocalForward; // Local forward vector of each joint 

    private FeatureSet _featureSet;
    public bool JointsLocalForwardError => jointsLocalForward == null;

    // IPoseSetSource --- the subset of this asset the pose-database pipeline actually reads.
    // Exposed as properties so a consumer that only wants a pose database (MotionField's config)
    // can supply one without also being a full Motion Matching asset.
    public List<AnnotatedAnimationClip> AnimationClips => animationClips;
    public float3 HipsForwardLocalVector => hipsForwardLocalVector;
    public float ContactVelocityThreshold => contactVelocityThreshold;
    public IReadOnlyList<JointToMecanim> AnimationChannelToMecanim => animationChannelToMecanim;

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

    private void ImportAnimations()
    {
        PROFILE.BEGIN_SAMPLE_PROFILING("BVH Import");
        foreach (var animData in animationClips)
        {
            // Add Mecanim mapping information
            animData.UpdateMecanimInformation(this);
        }

        PROFILE.END_AND_PRINT_SAMPLE_PROFILING("BVH Import");
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
        ImportAnimations();
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

    public void ComputeJointsLocalForward()
    {
        // Import T-Pose
        var skeleton = tPoseAnimationClip.Skeleton;
        var skeletonData = skeleton.GetSkeletonData();
        var pose = tPoseAnimationClip.GetFrame(0);
        var localRotations = pose.Rotations;

        jointsLocalForward = new float3[skeleton.BoneCount + 1]; // +1 for the simulation bone
        // Find forward character vector by projecting hips forward vector onto the ground
        float3 hipsWorldForwardProjected = math.mul(localRotations[0], hipsForwardLocalVector);
        hipsWorldForwardProjected.y = 0;
        hipsWorldForwardProjected = math.normalize(hipsWorldForwardProjected);
        // Find right character vector by rotating Y-Axis 90 degrees (Unity is Left-Handed and Y-Axis is Up)
        float3 hipsWorldRightProjected =
            math.mul(quaternion.AxisAngle(math.up(), math.radians(90.0f)), hipsWorldForwardProjected);
        // Compute JointsLocalForward based on the T-Pose
        jointsLocalForward[0] = math.forward();
        for (int i = 1; i < jointsLocalForward.Length; i++)
        {
            quaternion worldRot = PoseFK.CharacterRotation(pose, skeletonData, i - 1);
            // Change to Local
            if (!TryGetMecanimBone(skeleton.GetBone(i - 1).name, out HumanBodyBones bone))
            {
                Debug.LogWarning("[FeatureDebug] Failed to find Mecanim bone for joint " +
                                 skeleton.GetBone(i - 1).name);
            }

            float3 worldForward = hipsWorldForwardProjected;
            if (HumanBodyBonesExtensions.IsLeftArmBone(bone))
            {
                worldForward = -hipsWorldRightProjected;
            }
            else if (HumanBodyBonesExtensions.IsRightArmBone(bone))
            {
                worldForward = hipsWorldRightProjected;
            }

            jointsLocalForward[i] = math.mul(math.inverse(worldRot), worldForward);
        }
    }

    /// <summary>
    /// Returns the local forward vector of the iven joint index (after adding simulation bone)
    /// Vector computed from the T-Pose BVH and HipsForwardLocalVector
    /// </summary>
    public float3 GetLocalForward(int jointIndex)
    {
        Debug.Assert(!JointsLocalForwardError, "JointsLocalForward is not initialized");
        return jointsLocalForward[jointIndex];
    }

    public bool TryGetMecanimBone(string jointName, out HumanBodyBones bone)
    {
        for (int i = 0; i < animationChannelToMecanim.Count; i++)
        {
            if (animationChannelToMecanim[i].name != jointName) continue;
            bone = animationChannelToMecanim[i].mecanimBone;
            return true;
        }

        bone = HumanBodyBones.LastBone;
        return false;
    }

    public bool TryGetJointName(HumanBodyBones bone, out string jointName)
    {
        for (int i = 0; i < animationChannelToMecanim.Count; i++)
        {
            if (animationChannelToMecanim[i].mecanimBone == bone)
            {
                jointName = animationChannelToMecanim[i].name;
                return true;
            }
        }

        jointName = "";
        return false;
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