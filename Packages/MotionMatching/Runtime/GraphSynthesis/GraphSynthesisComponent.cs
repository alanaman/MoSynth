using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    public class GraphSynthesisComponent : MotionSynthesizer
    {
        public MotionMatchingData mmData;
        public MoSynthGraph graph;

        public PoseVector CurrentPose;
        private Skeleton _skeleton;
        public Skeleton Skeleton => _skeleton;

        public Transform[] skeletonTransforms;
        private Animator _animator;

        [SerializeField]
        [Tooltip("The avatar that maps from mecanim template to character transforms.")]
        private Avatar avatar;

        [Tooltip("Whether to animate the root position by Motion Matching or not.")]
        public bool rootPositionsMask = true;

        public override MotionMatchingData MmData => mmData;
        public override float3 RootVelocity { get; protected set; }
        public override float3 RootAngularVelocity { get; protected set; }
        public override float3 RootPosition { get; protected set; }
        public override quaternion RootRotation { get; protected set; }

        private List<MoSynthStageNode> _executionOrder = new List<MoSynthStageNode>();

        private void Awake()
        {
            _animator = GetComponent<Animator>();

            if (graph != null)
            {
                // Simple topological sort / extraction of nodes.
                // Assuming nodes will execute based on how we define later,
                // for now we'll just gather them.
                foreach (var node in graph.Nodes)
                {
                    if (node is MoSynthStageNode stageNode)
                    {
                        _executionOrder.Add(stageNode);
                    }
                }

                // TODO: Implement proper topological sort based on connections
            }

            _skeleton = null;
            foreach (var node in _executionOrder)
            {
                _skeleton = node.GetSkeleton(_skeleton);
            }

            if (_skeleton == null)
            {
                _skeleton = mmData.GetOrImportPoseSet().Skeleton;
            }

            InitSkeletonTransformsArray();
            InitCurrentPose();
            ApplyTransformOffsetsFromSkeleton();

            foreach (var node in _executionOrder)
            {
                node.Init(this);
            }
        }

        private void InitSkeletonTransformsArray()
        {
            var poseSet = mmData.GetOrImportPoseSet();
            var poseJoints = poseSet.Skeleton.Joints;

            skeletonTransforms = new Transform[poseJoints.Count];
            skeletonTransforms[0] = transform;
            for (var i = 1; i < poseJoints.Count; i++)
            {
                skeletonTransforms[i] = _animator.GetBoneTransform(poseJoints[i].type);
            }
        }

        private void ApplyTransformOffsetsFromSkeleton()
        {
            var poseSet = mmData.GetOrImportPoseSet();
            var animJoints = poseSet.Skeleton.Joints;

            for (var i = 1; i < animJoints.Count; i++)
            {
                var skeletonTransform = _animator.GetBoneTransform(animJoints[i].type);
                var skeletonBone = GetSkeletonBone(_animator.avatar.humanDescription, animJoints[i].type);
                skeletonTransform.localPosition = skeletonBone.position;
            }
        }

        public SkeletonBone GetSkeletonBone(HumanDescription humanDescription, HumanBodyBones boneEnum)
        {
            if (boneEnum < 0 || boneEnum >= HumanBodyBones.LastBone)
            {
                throw new ArgumentException("Invalid HumanBodyBones enum value.");
            }

            var targetHumanName = HumanTrait.BoneName[(int)boneEnum];
            var targetRigBoneName = humanDescription.human.First(b => b.humanName == targetHumanName).boneName;

            return humanDescription.skeleton.First(skeletonBone => skeletonBone.name == targetRigBoneName);
        }

        private void LateUpdate()
        {
            ConstructCurrentPoseFromSkeletonTransforms();

            var pose = new PoseVector(CurrentPose);

            foreach (var node in _executionOrder)
            {
                node.Apply(pose, Time.deltaTime);
            }

            ApplyPoseToSkeletonTransforms(pose);
        }

        void InitCurrentPose()
        {
            CurrentPose = new PoseVector(_skeleton.Joints.Count);

            for (var i = 0; i < skeletonTransforms.Length; i++)
            {
                CurrentPose.JointLocalRotations[i] = skeletonTransforms[i].localRotation;
                CurrentPose.JointLocalPositions[i] = skeletonTransforms[i].localPosition;
                CurrentPose.JointLocalVelocities[i] = float3.zero;
                CurrentPose.JointLocalAngularVelocities[i] = float3.zero;
            }
        }

        void ConstructCurrentPoseFromSkeletonTransforms()
        {
            for (var i = 0; i < CurrentPose.JointLocalAngularVelocities.Length; i++)
            {
                var inverseLocalRotation = Quaternion.Inverse(CurrentPose.JointLocalRotations[i]);
                CurrentPose.JointLocalAngularVelocities[i] = (skeletonTransforms[i].localRotation * inverseLocalRotation).eulerAngles;
                CurrentPose.JointLocalVelocities[i] = (float3)skeletonTransforms[i].localPosition - CurrentPose.JointLocalPositions[i];
            }

            CurrentPose.JointLocalPositions[0] = skeletonTransforms[0].localPosition;
            CurrentPose.JointLocalRotations[0] = skeletonTransforms[0].localRotation;

            for (var i = 1; i < skeletonTransforms.Length; i++)
            {
                CurrentPose.JointLocalRotations[i] = skeletonTransforms[i].localRotation;
            }

            CurrentPose.JointLocalPositions[1] = skeletonTransforms[1].localPosition;
        }

        private void ApplyPoseToSkeletonTransforms(PoseVector pose)
        {
            for (var i = 0; i < _skeleton.Joints.Count; i++)
            {
                skeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
            }

            if (rootPositionsMask)
            {
                skeletonTransforms[0].localPosition = pose.JointLocalPositions[0];
            }
        }

        public override void SetRotAdjustment(quaternion adjustmentRotation)
        {
            throw new NotImplementedException();
        }

        public override void SetPosAdjustment(float3 adjustmentPosition)
        {
            throw new NotImplementedException();
        }

        public override float3 GetMainPositionFeature(int trajectoryIndex)
        {
            throw new NotImplementedException();
        }

        public override float4 GetEnvironmentFeature(string featureName, int trajectoryIndex)
        {
            throw new NotImplementedException();
        }
    }
}
