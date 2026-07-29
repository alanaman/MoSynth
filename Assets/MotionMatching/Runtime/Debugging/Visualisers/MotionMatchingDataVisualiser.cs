using System.Collections.Generic;
using MotionMatching;
using Unity.Mathematics;
using UnityEngine;
using static MotionMatching.MotionMatchingData;

namespace MotionMatching.Editor
{
/// <summary>
/// Import a BVH, create PoseSet and FeatureSet and visualize it using Gizmos.
/// </summary>
public class MotionMatchingDataVisualiser : MonoBehaviour
{
    public MotionMatchingData motionMatchingData;
    public bool play;
    public float spheresRadius = 0.05f;
    public bool lockFPS = true;
    public bool debugTrajectory = true;
    public bool debugPose = true;
    public bool debugEnvironment = true;
    public bool debugContacts = true;

    private PoseSet _poseSet;
    private FeatureSet _featureSet;
    private Transform[] _skeletonTransforms;
    [SerializeField] private int currentFrame;

    private void Awake()
    {
        // PoseSet
        _poseSet = motionMatchingData.GetOrImportPoseSet();

        // FeatureSet
        _featureSet = motionMatchingData.GetOrImportFeatureSet();

        // Skeleton
        _skeletonTransforms = new Transform[_poseSet.Skeleton.Joints.Count];
        _skeletonTransforms[0] = transform; // Simulation Bone
        for (int j = 1; j < _poseSet.Skeleton.Joints.Count; j++)
        {
            // Joints
            Skeleton.Joint joint = _poseSet.Skeleton.Joints[j];
            Transform t = (new GameObject()).transform;
            t.name = joint.name;
            t.SetParent(_skeletonTransforms[joint.parentIndex], false);
            t.localPosition = joint.localOffset;
            _skeletonTransforms[j] = t;
        }

        // FPS
        if (lockFPS)
        {
            Application.targetFrameRate = Mathf.RoundToInt(1.0f / _poseSet.FrameTime);
            Debug.Log("[BVHDebug] Updated Target FPS: " + Application.targetFrameRate);
        }
        else
        {
            Application.targetFrameRate = -1;
        }
    }

    private void Update()
    {
        if (play)
        {
            _poseSet.GetPose(currentFrame, out PoseVector pose);
            _skeletonTransforms[0].localPosition = pose.JointLocalPositions[0];
            _skeletonTransforms[1].localPosition = pose.JointLocalPositions[1];
            for (int i = 0; i < pose.JointLocalRotations.Length; i++)
            {
                _skeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
            }
            currentFrame = (currentFrame + 1) % _poseSet.NumberPoses;
        }
        else
        {
            currentFrame = 0;
            _skeletonTransforms[0].localPosition = float3.zero;
            for (int i = 0; i < _skeletonTransforms.Length; i++)
            {
                _skeletonTransforms[i].localRotation = quaternion.identity;
            }
        }
    }

    private void OnDestroy()
    {
        motionMatchingData.Dispose();
        _poseSet.Dispose();
        _featureSet.Dispose();
    }

    private void OnApplicationQuit()
    {
        motionMatchingData.Dispose();
        _poseSet.Dispose();
        _featureSet.Dispose();
    }


#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        // Skeleton
        if (_skeletonTransforms == null || _poseSet == null) return;

        Gizmos.color = Color.red;
        for (int i = 2; i < _skeletonTransforms.Length; i++) // skip Simulation Bone
        {
            Transform t = _skeletonTransforms[i];
            GizmosExtensions.DrawLine(t.parent.position, t.position, 3);
        }

        if (!play) return;
        // Character
        int currentFrame = math.max(0, this.currentFrame - 1); // FeatureDebug increments CurrentFrame after update... OnDrawGizmos is called after update
        _poseSet.GetPose(currentFrame, out PoseVector pose);
        FeatureSet.GetWorldOriginCharacter(pose, out float3 characterOrigin, out float3 characterForward);
        Gizmos.color = new Color(1.0f, 0.0f, 0.5f, 1.0f);
        Gizmos.DrawSphere(characterOrigin, spheresRadius);
        GizmosExtensions.DrawArrow(characterOrigin, characterOrigin + characterForward, thickness: 3);

        // Forward Trajectory Direction Features
        Gizmos.color = Color.gray;
        for (int t = 0; t < motionMatchingData.TrajectoryFeatures.Count; t++)
        {
            var trajectoryFeature = motionMatchingData.TrajectoryFeatures[t];
            if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Direction &&
                !trajectoryFeature.SimulationBone)
            {
                if (!_poseSet.Skeleton.TryFind(trajectoryFeature.Bone, out Skeleton.Joint joint)) Debug.Assert(false, "Bone not found");
                float3 dir = _skeletonTransforms[joint.index].TransformDirection(motionMatchingData.GetLocalForward(joint.index));
                float3 jointPos = _skeletonTransforms[joint.index].position;
                GizmosExtensions.DrawArrow(jointPos, jointPos + dir * 0.5f, 0.1f, thickness: 3);
            }
        }

        // Contacts
        if (debugContacts)
        {
            if (!_poseSet.Skeleton.TryFind(HumanBodyBones.LeftToes, out Skeleton.Joint leftToesJoint)) Debug.Assert(false, "Bone not found");
            if (!_poseSet.Skeleton.TryFind(HumanBodyBones.RightToes, out Skeleton.Joint rightToesJoint)) Debug.Assert(false, "Bone not found");
            int leftToesIndex = leftToesJoint.index;
            int rightToesIndex = rightToesJoint.index;
            Gizmos.color = Color.green;
            if (pose.LeftFootContact)
            {
                Gizmos.DrawSphere(_skeletonTransforms[leftToesIndex].position, spheresRadius);
            }
            if (pose.RightFootContact)
            {
                Gizmos.DrawSphere(_skeletonTransforms[rightToesIndex].position, spheresRadius);
            }
        }

        // Feature Set
        if (_featureSet == null) return;

        DrawFeatureGizmos(_featureSet, motionMatchingData, spheresRadius, currentFrame, characterOrigin, characterForward,
            _skeletonTransforms, _poseSet.Skeleton, Color.blue, debugPose: debugPose, debugTrajectory: debugTrajectory, debugEnvironment: debugEnvironment);
    }

    private static List<float3> _positionFeatures = new();
    public static void DrawFeatureGizmos(FeatureSet set, MotionMatchingData mmData, float spheresRadius, int currentFrame,
        float3 characterOrigin, float3 characterForward, Transform[] joints, Skeleton skeleton,
        Color trajectoryColor, bool debugPose = true, bool debugTrajectory = true, bool debugEnvironment = true)
    {
        if (!set.IsValidFeature(currentFrame)) return;

        quaternion characterRot = quaternion.LookRotation(characterForward, math.up());

        // TODO: find a better way to store this information
        _positionFeatures.Clear();

        // Trajectory Features ---------------------------------------------------------------------------
        // Find the Main Position Feature (if exists)
        for (int t = 0; t < mmData.TrajectoryFeatures.Count; t++)
        {
            var trajectoryFeature = mmData.TrajectoryFeatures[t];
            if (trajectoryFeature.IsMainPositionFeature)
            {
                Debug.Assert(trajectoryFeature.FeatureType == TrajectoryFeature.Type.Position, "The main position feature should be of type Position");
                for (int p = 0; p < trajectoryFeature.FramesPrediction.Length; p++)
                {
                    float3 value = set.Get3DValuePositionOrDirectionFeature(trajectoryFeature, currentFrame, t, p, isEnvironment: false);
                    value = characterOrigin + math.mul(characterRot, value);
                    _positionFeatures.Add(value);
                }
            }
        }
        // Draw Trajectory Features
        if (debugTrajectory)
        {
            for (int t = 0; t < mmData.TrajectoryFeatures.Count; t++)
            {
                var trajectoryFeature = mmData.TrajectoryFeatures[t];
                for (int p = 0; p < trajectoryFeature.FramesPrediction.Length; p++)
                {
                    DrawTrajectoryPoint(trajectoryFeature, set, currentFrame, t, p, trajectoryColor, characterOrigin, characterForward,
                        characterRot, spheresRadius, joints, skeleton, isEnvironment: false);
                }
            }
        }
        // Pose Features ---------------------------------------------------------------------------
        if (debugPose)
        {
            Gizmos.color = new Color(0.0f, 0.8f, 0.8f);
            for (int p = 0; p < mmData.PoseFeatures.Count; p++)
            {
                var poseFeature = mmData.PoseFeatures[p];
                float3 value = set.GetPoseFeature(currentFrame, p, true);
                switch (poseFeature.FeatureType)
                {
                    case PoseFeature.Type.Position:
                        value = characterOrigin + math.mul(characterRot, value);
                        Gizmos.DrawSphere(value, spheresRadius);
                        break;
                    case PoseFeature.Type.Velocity:
                        value = math.mul(characterRot, value);
                        if (math.length(value) > 0.001f)
                        {
                            skeleton.TryFind(poseFeature.Bone, out Skeleton.Joint joint);
                            float3 jointPos = joints[joint.index].position;
                            GizmosExtensions.DrawArrow(jointPos, jointPos + value * 0.2f, 0.25f * math.length(value) * 0.2f, thickness: 4, useDepth: false);
                        }
                        break;
                }
            }
        }

        // Environment Features ---------------------------------------------------------------------------
        if (debugEnvironment)
        {
            for (int t = 0; t < mmData.EnvironmentFeatures.Count; t++)
            {
                var environmentFeature = mmData.EnvironmentFeatures[t];
                for (int p = 0; p < environmentFeature.FramesPrediction.Length; p++)
                {
                    DrawTrajectoryPoint(environmentFeature, set, currentFrame, t, p, trajectoryColor, characterOrigin, characterForward,
                        characterRot, spheresRadius, joints, skeleton, isEnvironment: true);
                }
            }
        }
    }

    private static void DrawTrajectoryPoint(TrajectoryFeature trajectoryFeature, FeatureSet set, int currentFrame, int trajectoryFeatureIndex,
        int predictionIndex, Color trajectoryColor, float3 characterOrigin, float3 characterForward,
        quaternion characterRot, float spheresRadius, Transform[] joints, Skeleton skeleton, bool isEnvironment)
    {
        int t = trajectoryFeatureIndex;
        int p = predictionIndex;
        //Gizmos.color = trajectoryColor * (1.25f - (float)p / trajectoryFeature.FramesPrediction.Length);
        Gizmos.color = trajectoryColor + (new Color(1.0f, 1.0f, 1.0f) - trajectoryColor) * ((float)p / trajectoryFeature.FramesPrediction.Length);
        if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Position ||
            trajectoryFeature.FeatureType == TrajectoryFeature.Type.Direction)
        {
            float3 value = set.Get3DValuePositionOrDirectionFeature(trajectoryFeature, currentFrame, t, p, isEnvironment);
            switch (trajectoryFeature.FeatureType)
            {
                case TrajectoryFeature.Type.Position:
                    value = characterOrigin + math.mul(characterRot, value);
                    Gizmos.DrawSphere(value, spheresRadius);
                    break;
                case TrajectoryFeature.Type.Direction:
                    float3 jointPos;
                    if (trajectoryFeature.SimulationBone)
                    {
                        jointPos = _positionFeatures.Count > 0 ? _positionFeatures[p] : float3.zero;
                    }
                    else
                    {
                        if (!skeleton.TryFind(trajectoryFeature.Bone, out Skeleton.Joint joint)) Debug.Assert(false, "Bone not found");
                        jointPos = joints[joint.index].position;
                    }
                    value = math.mul(characterRot, value);
                    //GizmosExtensions.DrawArrow(jointPos, jointPos + value, 0.1f, thickness: 3);
                    GizmosExtensions.DrawArrow(jointPos, jointPos + value * 0.4f, 0.15f, thickness: 4);
                    break;
            }
        }
        else if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Custom1D)
        {
            Feature1DExtractor featureExtractor = trajectoryFeature.FeatureExtractor as Feature1DExtractor;
            float value = isEnvironment ? set.Get1DEnvironmentFeature(currentFrame, t, p) : set.Get1DTrajectoryFeature(currentFrame, t, p, true);
            featureExtractor.DrawGizmos(value, spheresRadius, characterOrigin, characterForward, joints, skeleton, _positionFeatures.Count > 0 ? _positionFeatures[p] : float3.zero);
        }
        else if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Custom2D)
        {
            Feature2DExtractor featureExtractor = trajectoryFeature.FeatureExtractor as Feature2DExtractor;
            float2 value = isEnvironment ? set.Get2DEnvironmentFeature(currentFrame, t, p) : set.Get2DTrajectoryFeature(currentFrame, t, p, true);
            featureExtractor.DrawGizmos(value, spheresRadius, characterOrigin, characterForward, joints, skeleton, _positionFeatures.Count > 0 ? _positionFeatures[p] : float3.zero);
        }
        else if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Custom3D)
        {
            Feature3DExtractor featureExtractor = trajectoryFeature.FeatureExtractor as Feature3DExtractor;
            float3 value = isEnvironment ? set.Get3DEnvironmentFeature(currentFrame, t, p) : set.Get3DTrajectoryFeature(currentFrame, t, p, true);
            featureExtractor.DrawGizmos(value, spheresRadius, characterOrigin, characterForward, joints, skeleton, _positionFeatures.Count > 0 ? _positionFeatures[p] : float3.zero);
        }
        else if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Custom4D)
        {
            Feature4DExtractor featureExtractor = trajectoryFeature.FeatureExtractor as Feature4DExtractor;
            float4 value = isEnvironment ? set.Get4DEnvironmentFeature(currentFrame, t, p) : set.Get4DTrajectoryFeature(currentFrame, t, p, true);
            featureExtractor.DrawGizmos(value, spheresRadius, characterOrigin, characterForward, joints, skeleton, _positionFeatures.Count > 0 ? _positionFeatures[p] : float3.zero);
        }
    }
#endif
}
}
