using System.Collections;
using System.Collections.Generic;
using UnityEngine;
[RequireComponent(typeof(PlayerCharacterControl))]
[RequireComponent(typeof(Animator))]
public class PlayerCharacterAnimation : MonoBehaviour
{
    PlayerCharacterControl control;
    Animator anim;
    void Start()
    {
        control = GetComponent<PlayerCharacterControl>();
        anim = GetComponent<Animator>();

    }
    private void Update()
    {
        anim.SetFloat("Run",control.bodyVelocityW.magnitude/control.targetSpeed*2);
        anim.SetBool("IsFalling",!control.isGrounded);
        if (Input.GetButtonDown("Fire1"))
            anim.SetTrigger("Attack");
    }
    Vector3 leftFootTarget, rightFootTarget, bodyTarget;
    //[SerializeField, ReadOnly] Vector3 leftFootLocal, rightFootLocal, bodyLocal;
    DampingVector3 leftFootLocal = new DampingVector3(0.05f, 1f), rightFootLocal = new DampingVector3(0.05f, 1f), bodyLocal = new DampingVector3(0.05f, 1f);
    //DampingVector3 leftFootDamped = new DampingVector3(0.05f, 1f), rightFootDamped = new DampingVector3(0.05f, 1f), bodyDamped = new DampingVector3(0.05f, 1f);
    DampingFloat bodyDownLocal = new DampingFloat(0.3f, 1f), bodyTiltX = new DampingFloat(0.3f, 1f), bodyTiltZ = new DampingFloat(0.3f, 1f);
    DampingFloat bodyReactY = new DampingFloat(0.3f, 1f);//TODO
    Vector3 leftFootAnimPos, rightFootAnimPos, bodyAnimPos;
    bool needInitIK = true;
    float maxFootRaise = 0.4f;
    private void OnAnimatorIK(int layerIndex)
    {
        anim.SetIKPositionWeight(AvatarIKGoal.LeftFoot, 1);
        anim.SetIKPositionWeight(AvatarIKGoal.RightFoot, 1);
        anim.SetIKRotationWeight(AvatarIKGoal.LeftFoot, 1);
        anim.SetIKRotationWeight(AvatarIKGoal.RightFoot, 1);

        leftFootAnimPos = anim.GetBoneTransform(HumanBodyBones.LeftFoot).position - transform.up * anim.leftFeetBottomHeight;
        rightFootAnimPos = anim.GetBoneTransform(HumanBodyBones.RightFoot).position - transform.up * anim.rightFeetBottomHeight;
        bodyAnimPos = anim.bodyPosition;
        leftFootTarget = leftFootAnimPos;
        rightFootTarget = rightFootAnimPos;
        bodyTarget = bodyAnimPos;
        if (needInitIK)
        {
            leftFootLocal.current = transform.InverseTransformPoint(leftFootTarget);
            rightFootLocal.current = transform.InverseTransformPoint(rightFootTarget);
            bodyLocal.current = transform.InverseTransformPoint(bodyTarget);
            //bodyDamped.current = (bodyTarget);
            //leftFootDamped.current = (leftFootTarget);
            //rightFootDamped.current = (rightFootTarget);
            needInitIK = false;
        }
        float accX = -Mathf.Clamp(Vector3.Dot(control.accelerationWREstimated, transform.right) / control.maxAcceleration, -1, 1);
        float accZ = Mathf.Clamp(Vector3.Dot(control.accelerationWREstimated, transform.forward) / control.maxAcceleration, -1, 1);
        float tiltX = 0, tiltZ = 0;
        if (control.isGrounded)
        {
            RaycastHit hitInfo;
            float leftFootHeight = Physics.Raycast(leftFootAnimPos + transform.up * maxFootRaise, -transform.up, out hitInfo, 1.0f) ? hitInfo.distance - maxFootRaise : 0;
            float rightFootHeight = Physics.Raycast(rightFootAnimPos + transform.up * maxFootRaise, -transform.up, out hitInfo, 1.0f) ? hitInfo.distance - maxFootRaise : 0;
            if (transform.InverseTransformPoint(leftFootAnimPos).y > 0.05f) leftFootHeight = 0;
            if (transform.InverseTransformPoint(rightFootAnimPos).y > 0.05f) rightFootHeight = 0;
            float bodyDown = bodyDownLocal.Update(Mathf.Max(leftFootHeight, rightFootHeight), Time.deltaTime);
            leftFootHeight = Mathf.Clamp(leftFootHeight, -maxFootRaise, bodyDown);
            rightFootHeight = Mathf.Clamp(rightFootHeight, -maxFootRaise, bodyDown);

            leftFootTarget = leftFootAnimPos - transform.up * leftFootHeight;
            rightFootTarget = rightFootAnimPos - transform.up * rightFootHeight;
            bodyTarget -= transform.up * bodyDown;


            tiltZ = accX * 10f;
            tiltX = accZ * (accZ > 0 ? 20f : 10f);
        }
        anim.bodyRotation = anim.bodyRotation * Quaternion.Euler(bodyTiltX.Update(tiltX, Time.deltaTime), 0, bodyTiltZ.Update(tiltZ, Time.deltaTime));

        bodyReactY.externalForce = Mathf.Clamp(-Vector3.Dot(control.accelerationWREstimated, transform.up),-10,10);
        float bodyReactYDelta = bodyReactY.Update(bodyTarget.y, Time.deltaTime) - bodyTarget.y;
        bodyTarget.y += bodyReactYDelta;
        if(!control.isGrounded)
        {
            //leftFootTarget.y += bodyReactYDelta/2;
            //rightFootTarget.y += bodyReactYDelta/2;
        }


        anim.bodyPosition = transform.TransformPoint(bodyLocal.Update(transform.InverseTransformPoint(bodyTarget), Time.deltaTime));
        anim.SetIKPosition(AvatarIKGoal.LeftFoot, transform.TransformPoint(leftFootLocal.Update(transform.InverseTransformPoint(leftFootTarget), Time.deltaTime)) + transform.up * anim.leftFeetBottomHeight);
        anim.SetIKPosition(AvatarIKGoal.RightFoot, transform.TransformPoint(rightFootLocal.Update(transform.InverseTransformPoint(rightFootTarget), Time.deltaTime)) + transform.up * anim.rightFeetBottomHeight);
        //anim.bodyPosition = (bodyDamped.Update((bodyTarget), Time.deltaTime));
        //anim.SetIKPosition(AvatarIKGoal.LeftFoot, (leftFootDamped.Update((leftFootTarget), Time.deltaTime)) + transform.up * anim.leftFeetBottomHeight);
        //anim.SetIKPosition(AvatarIKGoal.RightFoot, (rightFootDamped.Update((rightFootTarget), Time.deltaTime)) + transform.up * anim.rightFeetBottomHeight);
    }
}
