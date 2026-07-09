using System;
using System.Collections;
using System.Collections.Generic;
using MotionMatching;
using UnityEngine;
using UnityEngine.Serialization;

/// <summary>
/// Import a BVH and visualize it using Gizmos.
/// </summary>
[ExecuteInEditMode]
public class BVHDebug : MonoBehaviour
{
    public TextAsset bvh;
    public float unitScale = 1;
    public int BVHIndex = 0;
    public bool play;
    public float spheresRadius = 0.1f;
    public bool lockFPS = true;

    private BVHAnimation _animation;
    private Transform[] _skeleton;
    private Transform _skeletonRoot;
    private int _currentFrame;

    private void Awake()
    {
        if (bvh == null) return;
        BVHImporter importer = new BVHImporter();
        _animation = importer.Import(bvh, unitScale);

        _skeleton = new Transform[_animation.Skeleton.Joints.Count];
        foreach (Skeleton.Joint joint in _animation.Skeleton.Joints)
        {
            Transform t = (new GameObject()).transform;
            t.name = joint.Name;
            if (joint.Index == 0) t.SetParent(transform, false);
            else t.SetParent(_skeleton[joint.ParentIndex], false);
            t.localPosition = joint.LocalOffset;
            _skeleton[joint.Index] = t;
        }

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

    private void Update()
    {
        if (_animation == null) return;
        if (play)
        {
            BVHAnimation.Frame frame = _animation.Frames[_currentFrame];
            _skeleton[0].localPosition = frame.RootMotion;
            for (int i = 0; i < frame.LocalRotations.Length; i++)
            {
                _skeleton[i].localRotation = frame.LocalRotations[i];
            }
            _currentFrame = (_currentFrame + 1) % _animation.Frames.Length;
        }
        else
        {
            _currentFrame = 0;
            _skeleton[0].localPosition = Vector3.zero;
            for (int i = 0; i < _skeleton.Length; i++)
            {
                _skeleton[i].localRotation = Quaternion.identity;
            }
        }
    }

    private void OnValidate()
    {
        if(bvh == null) return;
        BVHImporter importer = new BVHImporter();
        _animation = importer.Import(bvh, unitScale);

        _skeleton = new Transform[_animation.Skeleton.Joints.Count];
        foreach (Skeleton.Joint joint in _animation.Skeleton.Joints)
        {
            Transform t = (new GameObject()).transform;
            t.name = joint.Name;
            if (joint.Index == 0) t.SetParent(transform, false);
            else t.SetParent(_skeleton[joint.ParentIndex], false);
            t.localPosition = joint.LocalOffset;
            _skeleton[joint.Index] = t;
        }

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

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (_skeleton == null || _animation == null || _animation.EndSites == null) return;

        Gizmos.color = Color.red;
        for (int i = 1; i < _skeleton.Length; i++)
        {
            Transform t = _skeleton[i];
            GizmosExtensions.DrawLine(t.parent.position, t.position, 3);
        }
        // Uncomment to show end sites
        // foreach (BVHAnimation.EndSite endSite in Animation.EndSites)
        // {
        //     Transform t = Skeleton[endSite.ParentIndex];
        //     GizmosExtensions.DrawLine(t.position, t.TransformPoint(endSite.Offset), 3);
        // }

        Gizmos.color = new Color(1.0f, 0.3f, 0.1f, 1.0f);
        foreach (Transform t in _skeleton)
        {
            if (t.name == "End Site") continue;
            Gizmos.DrawWireSphere(t.position, spheresRadius);
        }
    }
#endif
}
