using System;
using UnityEngine;
using System.Collections.Generic;

[Serializable]
public class BoneNode
{
    public string id;
    public string displayName = "New Bone";
    public Rect rect = new Rect(50, 50, 60, 30);
    [SerializeReference]
    public Transform boneTransform;
}

public class SkeletonUIManager : MonoBehaviour
{
    [HideInInspector]
    public List<BoneNode> boneNodes = new List<BoneNode>();
}
