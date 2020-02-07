using System.Collections;
using System.Collections.Generic;
using UnityEngine;
[RequireComponent(typeof(PlayerCharacterControl))]
[RequireComponent(typeof(Animator))]
public class PlayerCharacterAnimation : MonoBehaviour
{
    PlayerCharacterControl control;
    Animator anim;
    CapsuleCollider capsule;
    void Start()
    {
        control = GetComponent<PlayerCharacterControl>();
        anim = GetComponent<Animator>();
    }
    private void Update()
    {
        anim.SetFloat("Run",control.bodyVelocityW.magnitude/control.runSpeed*2);
        anim.SetFloat("Climb", control.bodyVelocityW.magnitude / control.climbSpeed);
        anim.SetBool("IsWalkable", control.isWalkable);
        anim.SetBool("IsGrounded",control.isGrounded);
        anim.SetBool("IsLedging", control.isLedging);
        anim.SetBool("IsClimbing", control.isClimbing);
        if(control.isWalkable)
        if (Input.GetButtonDown("Fire1"))
            anim.SetTrigger("Attack");
    }
    Transform attachReference = null, lastAttachReference = null;bool updateAttach;
    Vector3 leftFootTarget, rightFootTarget, leftHandTarget, rightHandTarget, bodyTarget;
    Vector3 leftHandAttachWR, rightHandAttachWR, leftFootAttachWR, rightFootAttachWR;
    DampingVector3 leftFootLocal = new DampingVector3(0.05f, 1f), rightFootLocal = new DampingVector3(0.05f, 1f), leftHandLocal = new DampingVector3(0.05f, 1f), rightHandLocal = new DampingVector3(0.05f, 1f), 
        bodyLocal = new DampingVector3(0.05f, 1f);
    DampingFloat bodyDownLocal = new DampingFloat(0.3f, 1f), bodyTiltX = new DampingFloat(0.3f, 1f), bodyTiltZ = new DampingFloat(0.3f, 1f);
    DampingFloat bodyReactY = new DampingFloat(0.3f, 1f);
    Vector3 leftFootAnimPos, rightFootAnimPos,leftHandAnimPos,rightHandAnimPos, bodyAnimPos;
    bool needInitIK = true;
    float maxFootRaise = 0.4f;
    private void OnAnimatorIK(int layerIndex)
    {

        anim.SetIKPositionWeight(AvatarIKGoal.LeftFoot, 1);
        anim.SetIKPositionWeight(AvatarIKGoal.RightFoot, 1);
        anim.SetIKRotationWeight(AvatarIKGoal.LeftFoot, 1);
        anim.SetIKRotationWeight(AvatarIKGoal.RightFoot, 1);
        anim.SetIKPositionWeight(AvatarIKGoal.LeftHand, 1);
        anim.SetIKPositionWeight(AvatarIKGoal.RightHand, 1);
        anim.SetIKRotationWeight(AvatarIKGoal.LeftHand, 0);//TODO
        anim.SetIKRotationWeight(AvatarIKGoal.RightHand, 0);

        leftHandAnimPos = anim.GetBoneTransform(HumanBodyBones.LeftHand).position;
        rightHandAnimPos = anim.GetBoneTransform(HumanBodyBones.RightHand).position;
        leftFootAnimPos = anim.GetBoneTransform(HumanBodyBones.LeftFoot).position - transform.up * anim.leftFeetBottomHeight;
        rightFootAnimPos = anim.GetBoneTransform(HumanBodyBones.RightFoot).position - transform.up * anim.rightFeetBottomHeight;
        bodyAnimPos = anim.bodyPosition;

        leftHandTarget = leftHandAnimPos;
        rightHandTarget = rightHandAnimPos;
        leftFootTarget = leftFootAnimPos;
        rightFootTarget = rightFootAnimPos;
        bodyTarget = bodyAnimPos;

        attachReference = control.attachReference;
        updateAttach =  attachReference!=lastAttachReference && attachReference!=null;
        lastAttachReference = attachReference;

        if (updateAttach)
        {
            leftHandAttachWR = attachReference.InverseTransformPoint(leftHandAnimPos);
            rightHandAttachWR = attachReference.InverseTransformPoint(rightHandAnimPos);
            leftFootAttachWR = attachReference.InverseTransformPoint(leftFootAnimPos);
            rightFootAttachWR = attachReference.InverseTransformPoint(rightFootAnimPos);
        }

        if (needInitIK)
        {
            leftFootLocal.current = transform.InverseTransformPoint(leftFootTarget);
            rightFootLocal.current = transform.InverseTransformPoint(rightFootTarget);
            leftHandLocal.current = transform.InverseTransformPoint(leftHandTarget);
            rightHandLocal.current = transform.InverseTransformPoint(rightHandTarget);
            bodyLocal.current = transform.InverseTransformPoint(bodyTarget);
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
            leftHandTarget-= transform.up * bodyDown;
            rightHandTarget -= transform.up * bodyDown;


            tiltZ = accX * 10f;
            tiltX = accZ * (accZ > 0 ? 20f : 10f);
        }
        if (attachReference != null)
        {
            bool leftHandAttach, rightHandAttach, leftFootAttach, rightFootAttach;
            if (control.isClimbing)
            {

            }
            /*
            RaycastHit hitInfo;
            Vector3 castDir = control.isClimbing ? transform.forward : -transform.up;


            AvatarIKGoal onWallHand = Vector3.Dot(leftHandAnimPos - rightHandAnimPos, transform.forward) > 0 ? AvatarIKGoal.LeftHand : AvatarIKGoal.RightHand;
            AvatarIKGoal onWallFoot = Vector3.Dot(leftFootAnimPos - rightFootAnimPos, transform.forward) > 0 ? AvatarIKGoal.LeftFoot : AvatarIKGoal.RightFoot;
            bool leftHandOnWall = Physics.Raycast(anim.GetIKPosition(AvatarIKGoal.LeftHand) - transform.forward * .2f, transform.forward, out hitInfo, .4f);
            bool rightHandOnWall = Physics.Raycast(anim.GetIKPosition(AvatarIKGoal.LeftHand) - transform.forward * .2f, transform.forward, out hitInfo, .4f);
            */
        }

        if (control.isLedging)
        {
            leftHandTarget = control.attachReference.TransformPoint( control.ledgeTargetR);
            rightHandTarget = leftHandTarget;
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
        anim.SetIKPosition(AvatarIKGoal.LeftHand, transform.TransformPoint(leftHandLocal.Update(transform.InverseTransformPoint(leftHandTarget), Time.deltaTime)));
        anim.SetIKPosition(AvatarIKGoal.RightHand, transform.TransformPoint(rightHandLocal.Update(transform.InverseTransformPoint(rightHandTarget), Time.deltaTime)));

    }
}
