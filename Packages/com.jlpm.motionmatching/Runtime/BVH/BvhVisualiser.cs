using System;
using MotionMatching;
using UnityEngine;

namespace BVH
{
/// <summary>
/// Import a BVH and visualize it using Gizmos.
/// </summary>
[ExecuteInEditMode]
public class BvhVisualiser : MonoBehaviour
{
    public TextAsset bvh;
    public float unitScale = 1;
    public bool play;
    public float spheresRadius = 0.1f;
    public bool lockFPS = true;

    private BvhAnimation _animation;
    private Transform[] _skeletonBoneTransforms;
    private Transform _skeletonRoot;
    private int _currentFrame;

    private void Awake()
    {
        if (!bvh) return;
        _animation = BvhImporter.Import(bvh, unitScale);
        SetupSkeleton();

        if (lockFPS)
        {
            Application.targetFrameRate = (int)(1.0f / _animation.FrameTime);
            Debug.Log("[BVHDebug] Updated Target FPS: " + Application.targetFrameRate);
        }
        else
        {
            Application.targetFrameRate = -1;
        }
    }

    private void SetupSkeleton()
    {
        _skeletonBoneTransforms = EnsureSkeletonHierarchy(_animation.Skeleton);
        for (int i = 0; i < _skeletonBoneTransforms.Length; i++)
        {
            _skeletonBoneTransforms[i].localPosition = _animation.Skeleton.Joints[i].LocalOffset;
        }
    }

    private void Update()
    {
        if (_animation == null) return;
        if (play)
        {
            BvhAnimation.Frame frame = _animation.Frames[_currentFrame];
            _skeletonBoneTransforms[0].localPosition = frame.RootMotion;
            for (int i = 0; i < frame.LocalRotations.Length; i++)
            {
                _skeletonBoneTransforms[i].localRotation = frame.LocalRotations[i];
            }
            _currentFrame = (_currentFrame + 1) % _animation.Frames.Length;
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
        if(bvh == null) return;
        _animation = BvhImporter.Import(bvh, unitScale);
        SetupSkeleton();
        
        if (lockFPS)
        {
            Application.targetFrameRate = (int)(1.0f / _animation.FrameTime);
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
            if (joint.ParentIndex >= 0)
            {
                parentTransform = GetOrCreateBone(joint.ParentIndex);
            }

            // Check if the bone already exists as a direct child of the resolved parent
            var existingBone = parentTransform.Find(joint.Name);

            if (existingBone != null)
            {
                // Bone exists, link it to the cache without modifying it
                boneTransforms[jointIndex] = existingBone;
            }
            else
            {
                // Bone is missing, create it
                var newBone = new GameObject(joint.Name);
                
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
        if (_skeletonBoneTransforms == null || _animation == null || _animation.EndSites == null) return;

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
