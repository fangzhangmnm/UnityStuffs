using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public enum BTStatus { Pending,Running,Failed,Succeed};
public abstract class BTNode
{
    BTStatus m_status;
    public BTStatus status { get { return m_status; } }
    protected abstract BTStatus Update(float dt);
    protected virtual void Start() { }
    protected virtual void Finish() { }
    public BTStatus Traversal(float dt)
    {
        //Debug.Log($"Enter {name},{m_status}");
        if (!(m_status == BTStatus.Pending || m_status == BTStatus.Running))
        {
            //Debug.Log($"Exit {name},{m_status}");
            return m_status;
        }
        else
        {
            if (m_status == BTStatus.Pending) Start();
            m_status = Update(dt);
            if (m_status !=BTStatus.Running) Finish();
            //Debug.Log($"Exit {name},{m_status}");
            return m_status;
        }
    }
    protected List<BTNode> children=new List<BTNode>();
    public BTNode AddNode(BTNode node=null) { if(node!=null)children.Add(node);return this; }
    public BTNode AddNodes(BTNode[] nodes = null) { if (nodes != null) foreach (var n in nodes) AddNode(n);return this; }
    public virtual void Reset()
    {
        //Debug.Log($"{name},Reset");
        m_status = BTStatus.Pending;
        foreach (var n in children)
            n.Reset();
    }
    public string name;
    public BTNode(string name="Node")  { this.name = name; }
}
public class BTSequencer: BTNode
{
    public BTSequencer(string name = "Sequencer") : base(name) { }
    protected override BTStatus Update(float dt)
    {
        foreach(var n in children)
            switch (n.Traversal(dt))
            {
                case BTStatus.Running:
                    return BTStatus.Running;
                case BTStatus.Failed:
                    return BTStatus.Failed;
                case BTStatus.Succeed:
                    continue;
            }
        return BTStatus.Succeed;
    }
}
public class BTSelector : BTNode
{
    public BTSelector(string name = "Selector") : base(name) { }
    protected override BTStatus Update(float dt)
    {
        foreach (var n in children)
            switch (n.Traversal(dt))
            {
                case BTStatus.Running:
                    return BTStatus.Running;
                case BTStatus.Failed:
                    continue;
                case BTStatus.Succeed:
                    return BTStatus.Succeed;
            }
        return BTStatus.Failed;
    }
}
public class BTParallel : BTNode
{
    public BTParallel(string name = "Parallel") : base(name) { }
    protected override BTStatus Update(float dt)
    {
        bool hasRunningChild = false;
        foreach (var n in children)
            switch (n.Traversal(dt))
            {
                case BTStatus.Running:
                    hasRunningChild = true;
                    continue;
                case BTStatus.Failed:
                    return BTStatus.Failed;
                case BTStatus.Succeed:
                    continue;
            }
        return hasRunningChild ? BTStatus.Running : BTStatus.Succeed;
    }
}
public class BTRepeatUntilFail : BTNode
{
    public BTRepeatUntilFail(string name = "RepeatUntilFail") : base(name) { }
    protected override BTStatus Update(float dt)
    {
        Debug.Assert(children.Count == 1);
        while (true)
        {
            switch (children[0].Traversal(dt))
            {
                case BTStatus.Running:
                    return BTStatus.Running;
                case BTStatus.Failed:
                    return BTStatus.Succeed;
                case BTStatus.Succeed:
                    children[0].Reset();
                    break;
            }
        }
    }
}
public class BTInvert : BTNode
{
    public BTInvert(string name = "Invert") : base(name) { }
    protected override BTStatus Update(float dt)
    {
        Debug.Assert(children.Count == 1);

        switch (children[0].Traversal(dt))
        {
            case BTStatus.Running:
                return BTStatus.Running;
            case BTStatus.Failed:
                return BTStatus.Succeed;
            case BTStatus.Succeed:
                return BTStatus.Failed;
        }
        return BTStatus.Failed;
    }
}
public class BTUpdateDelegate : BTNode
{
    public delegate BTStatus UpdateDelegateType(float dt);
    UpdateDelegateType m_delegate;
    public BTUpdateDelegate(UpdateDelegateType del,string name = "UpdateDelegate") : base(name) { m_delegate = del; }
    protected override BTStatus Update(float dt){return m_delegate(dt);}
}
public class BTInstantDelegate : BTNode
{
    public delegate void InstantDelegateType();
    InstantDelegateType m_delegate;
    public BTInstantDelegate(InstantDelegateType del, string name = "InstantDelegate") : base(name) { m_delegate = del; }
    protected override BTStatus Update(float dt) { m_delegate();return BTStatus.Succeed; }
}
public class BTCheckDelegate : BTNode
{
    public delegate bool CheckDelegateType();
    CheckDelegateType m_delegate;
    public BTCheckDelegate(CheckDelegateType del, string name = "CheckDelegate") : base(name) { m_delegate = del; }
    protected override BTStatus Update(float dt) { return m_delegate() ? BTStatus.Succeed : BTStatus.Failed; }
}


public class BTree : MonoBehaviour
{
    BTNode root;
    public int a = 10, b = 6;
    void Start()
    {
        root = new BTSequencer().AddNodes(new BTNode[]
        {
            new BTRepeatUntilFail().AddNodes(new BTNode[]
            {
                new BTSequencer().AddNodes(new BTNode[]
                {
                    new BTCheckDelegate(() => a != b),
                    new BTRepeatUntilFail().AddNodes(new BTNode[]
                    {
                        new BTParallel().AddNodes(new BTNode[]
                        {
                            new BTCheckDelegate(() => a > b),
                            new BTInstantDelegate(() =>  a -= b ),
                        }),
                    }),
                    new BTRepeatUntilFail().AddNodes(new BTNode[]
                    {
                        new BTParallel().AddNodes(new BTNode[]
                        {
                            new BTCheckDelegate(() => b > a),
                            new BTInstantDelegate(() =>  b -= a ),
                        }),
                    }),
                }),
            }),
            new BTUpdateDelegate((dt) => { Debug.Log($"Done! {a},{b}"); return BTStatus.Succeed; }),
        });
    }

    // Update is called once per frame
    void Update()
    {
        if (root.status != BTStatus.Succeed)
        {
            Debug.Log("time passed");
            root.Traversal(Time.fixedDeltaTime);
        }
    }
}
