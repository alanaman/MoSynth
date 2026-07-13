using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace MotionMatching
{
/// <summary>
/// Import a BVH and visualize it using Gizmos.
/// </summary>
public class BvhVisualiser : MonoBehaviour
{
    [FormerlySerializedAs("_animation")] public BvhAnimation bvhAnimation;
    public bool play;
    public float spheresRadius = 0.1f;
    public bool lockFPS = true;

    private Transform[] _skeletonBoneTransforms;
    private Transform _skeletonRoot;
    private int _currentFrame;

    private void Awake()
    {
        if (!bvhAnimation) return;
        SetupSkeleton();

        if (lockFPS)
        {
            Application.targetFrameRate = (int)(1.0f / bvhAnimation.FrameTime);
            Debug.Log("[BVHDebug] Updated Target FPS: " + Application.targetFrameRate);
        }
        else
        {
            Application.targetFrameRate = -1;
        }
    }

    private void SetupSkeleton()
    {
        _skeletonBoneTransforms = EnsureSkeletonHierarchy(bvhAnimation.Skeleton);
        for (int i = 0; i < _skeletonBoneTransforms.Length; i++)
        {
            _skeletonBoneTransforms[i].localPosition = bvhAnimation.Skeleton.Joints[i].localOffset;
        }
    }

    private void Update()
    {
        if (bvhAnimation == null) return;
        if (play)
        {
            BvhAnimation.Frame frame = bvhAnimation.Frames[_currentFrame];
            _skeletonBoneTransforms[0].localPosition = frame.rootMotion;
            for (int i = 0; i < frame.localRotations.Length; i++)
            {
                _skeletonBoneTransforms[i].localRotation = frame.localRotations[i];
            }
            _currentFrame = (_currentFrame + 1) % bvhAnimation.Frames.Length;
        }
        else
        {
            _currentFrame = 0;
            _skeletonBoneTransforms[0].localPosition = Vector3.zero;
            foreach (var t in _skeletonBoneTransforms)
            {   
                t.localRotation = Quaternion.identity;
            }
        }
    }

    private void OnValidate()
    {
        if(bvhAnimation == null) return;
        SetupSkeleton();
        
        if (lockFPS)
        {
            Application.targetFrameRate = (int)(1.0f / bvhAnimation.FrameTime);
            Debug.Log("[BVHDebug] Updated Target FPS: " + Application.targetFrameRate);
        }
        else
        {
            Application.targetFrameRate = -1;
        }
    }
    
    /// <summary>
    /// Ensures the GameObject hierarchy matches the provided Skeleton structure.
    /// Existing bones are preserved; missing bones are created at their proper local offsets.
    /// </summary>
    private Transform[] EnsureSkeletonHierarchy(Skeleton skeleton)
    {
        if (skeleton?.Joints == null || skeleton.Joints.Count == 0)
        {
            return Array.Empty<Transform>();
        }

        // Cache for transforms to avoid redundant lookups and handle unsorted lists
        var boneTransforms = new Transform[skeleton.Joints.Count];

        // Iterate through all joints to guarantee every bone is processed
        for (int i = 0; i < skeleton.Joints.Count; i++)
        {
            GetOrCreateBone(i);
        }

        return boneTransforms;

        // Local helper function to recursively resolve/create a bone and its parents
        Transform GetOrCreateBone(int jointIndex)
        {
            // Invalid index or root returns the component's base transform
            if (jointIndex < 0 || jointIndex >= skeleton.Joints.Count)
            {
                return this.transform;
            }

            // Return immediately if this bone has already been resolved
            if (boneTransforms[jointIndex])
            {
                return boneTransforms[jointIndex];
            }

            var joint = skeleton.Joints[jointIndex];

            // Resolve the parent transform first (recursion ensures parents exist before children)
            var parentTransform = this.transform; 
            if (joint.parentIndex >= 0)
            {
                parentTransform = GetOrCreateBone(joint.parentIndex);
            }

            // Check if the bone already exists as a direct child of the resolved parent
            var existingBone = parentTransform.Find(joint.name);

            if (existingBone != null)
            {
                // Bone exists, link it to the cache without modifying it
                boneTransforms[jointIndex] = existingBone;
            }
            else
            {
                // Bone is missing, create it
                var newBone = new GameObject(joint.name);
                
                // SetParent with worldPositionStays = false to preserve local transform identity
                newBone.transform.SetParent(parentTransform, false);
                
                boneTransforms[jointIndex] = newBone.transform;
            }

            return boneTransforms[jointIndex];
        }
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (_skeletonBoneTransforms == null || bvhAnimation == null || bvhAnimation.EndSites == null) return;

        Gizmos.color = Color.red;
        for (int i = 1; i < _skeletonBoneTransforms.Length; i++)
        {
            Transform t = _skeletonBoneTransforms[i];
            GizmosExtensions.DrawLine(t.parent.position, t.position, 3);
        }
        // Uncomment to show end sites
        // foreach (BVHAnimation.EndSite endSite in Animation.EndSites)
        // {
        //     Transform t = Skeleton[endSite.ParentIndex];
        //     GizmosExtensions.DrawLine(t.position, t.TransformPoint(endSite.Offset), 3);
        // }

        Gizmos.color = new Color(1.0f, 0.3f, 0.1f, 1.0f);
        foreach (Transform t in _skeletonBoneTransforms)
        {
            if (t.name == "End Site") continue;
            Gizmos.DrawWireSphere(t.position, spheresRadius);
        }
    }
#endif
}



}
