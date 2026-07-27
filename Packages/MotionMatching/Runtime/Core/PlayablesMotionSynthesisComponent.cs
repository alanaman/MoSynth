using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Playables;

namespace MotionMatching
{
    [RequireComponent(typeof(Animator))]
    public class PlayablesMotionSynthesisComponent : MotionSynthesizer
    {
        public MotionMatchingData mmData;

        [SerializeReference] [SubclassSelector]
        public List<MoSynthStage> stages = new();

        [Tooltip("Whether to animate the root position by Motion Matching or not.")]
        public bool rootPositionsMask = true;

        [Tooltip("Index of the stage to feed into the motion matching target pose port. -1 for none.")]
        public int feedbackStageIndex = -1;

        public override PoseVector CurrentPose => _currentPose;
        private PoseVector _currentPose;
        public Skeleton Skeleton => _skeleton;

        private Skeleton _skeleton;
        private Transform[] _skeletonTransforms;
        private Animator _animator;
        private PlayableGraph _graph;
        private ScriptPlayable<MoSynthPlayableBehaviour>[] _stagePlayables;

        public override MotionMatchingData MmData => mmData;
        public override float3 RootVelocity { get; protected set; }
        public override float3 RootAngularVelocity { get; protected set; }
        public override float3 RootPosition { get; protected set; }
        public override quaternion RootRotation { get; protected set; }

        private void Awake()
        {
            _animator = GetComponent<Animator>();

            if (mmData == null) return;
            stages.RemoveAll(stage => stage == null);
            if (stages.Count == 0) return;

            _skeleton = null;
            foreach (var stage in stages)
            {
                _skeleton = stage.GetSkeleton(_skeleton);
            }
            if (_skeleton == null) return;

            InitSkeletonTransformsArray();
            InitCurrentPose();
            ApplyTransformOffsetsFromSkeleton();

            foreach (var stage in stages)
            {
                stage.Init(this);
            }

            BuildPlayableGraph();
        }

        private void BuildPlayableGraph()
        {
            _graph = PlayableGraph.Create("Motion Synthesis Graph");
            _graph.SetTimeUpdateMode(DirectorUpdateMode.Manual);

            if (stages.Count == 0) return;

            _stagePlayables = new ScriptPlayable<MoSynthPlayableBehaviour>[stages.Count];
            for (int i = 0; i < stages.Count; i++)
            {
                _stagePlayables[i] = ScriptPlayable<MoSynthPlayableBehaviour>.Create(_graph, 1);
                var bhv = _stagePlayables[i].GetBehaviour();
                bhv.Initialize(stages[i], _skeleton, _stagePlayables[i]);
            }

            // Connect them linearly: stage[i] output -> stage[i+1] input
            for (int i = 0; i < stages.Count - 1; i++)
            {
                _graph.Connect(_stagePlayables[i], 0, _stagePlayables[i + 1], 0);
            }

            // A playable output that pulls from the last stage
            var playableOutput = ScriptPlayableOutput.Create(_graph, "Output");
            playableOutput.SetSourcePlayable(_stagePlayables[^1]);

            _graph.Play();
        }

        private void InitSkeletonTransformsArray()
        {
            var poseSet = mmData.GetOrImportPoseSet();
            var poseJoints = poseSet.Skeleton.Joints;

            _skeletonTransforms = new Transform[poseJoints.Count];
            _skeletonTransforms[0] = transform;
            for (var i = 1; i < poseJoints.Count; i++)
            {
                _skeletonTransforms[i] = _animator.GetBoneTransform(poseJoints[i].type);
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
            var humanBone = humanDescription.human.FirstOrDefault(b => b.humanName == targetHumanName);
            if (humanBone.boneName == null) return new SkeletonBone();

            return humanDescription.skeleton.FirstOrDefault(skeletonBone => skeletonBone.name == humanBone.boneName);
        }

        void InitCurrentPose()
        {
            _currentPose = new PoseVector(_skeleton.Joints.Count);

            for (var i = 0; i < _skeletonTransforms.Length; i++)
            {
                _currentPose.JointLocalRotations[i] = _skeletonTransforms[i].localRotation;
                _currentPose.JointLocalPositions[i] = _skeletonTransforms[i].localPosition;
                _currentPose.JointLocalVelocities[i] = float3.zero;
                _currentPose.JointLocalAngularVelocities[i] = float3.zero;
            }
            _currentPose.LeftFootContact = false;
            _currentPose.RightFootContact = false;
        }

        private void LateUpdate()
        {
            if (stages.Count == 0) return;

            ConstructCurrentPoseFromSkeletonTransforms();

            // Feed the initial pose to the first stage
            var firstBhv = _stagePlayables[0].GetBehaviour();
            firstBhv.OutputPose.CopyFrom(_currentPose);

            // Handle feedback for motion matching target pose
            if (feedbackStageIndex >= 0 && feedbackStageIndex < _stagePlayables.Length)
            {
                var feedbackPose = _stagePlayables[feedbackStageIndex].GetBehaviour().OutputPose;
                for (int i = 0; i < stages.Count; i++)
                {
                    if (stages[i] is MotionMatchingStage mmStage)
                    {
                        mmStage.TargetPose = feedbackPose;
                    }
                }
            }

            _graph.Evaluate(Time.deltaTime);

            var lastBhv = _stagePlayables[^1].GetBehaviour();
            ApplyPoseToSkeletonTransforms(lastBhv.OutputPose);
        }

        void ConstructCurrentPoseFromSkeletonTransforms()
        {
            for (var i = 0; i < _currentPose.JointLocalAngularVelocities.Length; i++)
            {
                var inverseLocalRotation = Quaternion.Inverse(_currentPose.JointLocalRotations[i]);
                _currentPose.JointLocalAngularVelocities[i] = (_skeletonTransforms[i].localRotation * inverseLocalRotation).eulerAngles;
                _currentPose.JointLocalVelocities[i] = (float3)_skeletonTransforms[i].localPosition - _currentPose.JointLocalPositions[i];
            }

            _currentPose.JointLocalPositions[0] = _skeletonTransforms[0].localPosition;
            _currentPose.JointLocalRotations[0] = _skeletonTransforms[0].localRotation;

            for (var i = 1; i < _skeletonTransforms.Length; i++)
            {
                _currentPose.JointLocalRotations[i] = _skeletonTransforms[i].localRotation;
            }
            _currentPose.JointLocalPositions[1] = _skeletonTransforms[1].localPosition;
        }

        private void ApplyPoseToSkeletonTransforms(PoseVector pose)
        {
            for (var i = 0; i < _skeleton.Joints.Count; i++)
            {
                _skeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
            }

            if (rootPositionsMask)
            {
                _skeletonTransforms[0].localPosition = pose.JointLocalPositions[0];
            }

            // Apply Hips position
            if (_skeleton.Joints.Count > 1)
            {
                _skeletonTransforms[1].localPosition = pose.JointLocalPositions[1];
            }
        }

        private void OnDestroy()
        {
            if (_graph.IsValid())
            {
                _graph.Destroy();
            }
        }

        private float3 _posAdjustment;
        private quaternion _rotAdjustment = quaternion.identity;

        public override void SetRotAdjustment(quaternion adjustmentRotation)
        {
            _rotAdjustment = math.mul(adjustmentRotation, _rotAdjustment);
        }

        public override void SetPosAdjustment(float3 adjustmentPosition)
        {
            _posAdjustment += adjustmentPosition;
        }
        public override float3 GetMainPositionFeature(int trajectoryIndex) => float3.zero;
        public override float4 GetEnvironmentFeature(string featureName, int trajectoryIndex) => float4.zero;
    }
}
