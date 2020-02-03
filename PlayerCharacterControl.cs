using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(CapsuleCollider))]
[RequireComponent(typeof(Rigidbody))]
public class PlayerCharacterControl : MonoBehaviour
{
    CapsuleCollider capsule;
    Rigidbody body;
    bool inputJump;
    public float targetSpeed = 3f, maxAcceleration = 10f;
    [SerializeField, ReadOnly] Vector2 inputStick, inputStickRaw;
    [SerializeField, ReadOnly] public Vector3 velocityWR, accelerationWREstimated, bodyVelocityW;
    DerivativeVector3 accelerationSolverWR = new DerivativeVector3(0.1f);
    DerivativeVector3 attachVelocitySolverW = new DerivativeVector3(0.1f);

    [SerializeField, ReadOnly] public bool isGrounded, canClimb, isClimbing, canLedgeStart, isLedging, willFallFar;
    //can be climbing while grounded
    Transform attachReference = null, lastAttachReference = null;
    Vector3 lastAttachPositionW, lastAttachPositionR;
    public float lastGroundTime;
    bool lastGroundNoJump;
    Vector3 groundPositionW, groundNormalW, climbNormalW, ledgeTargetW;
    Vector3 gravity;
    float stepOffset = 0.3f;
    float slopeLimit = 70f;
    void Start()
    {
        Debug.Assert(transform.lossyScale == Vector3.one);
        Cursor.lockState = CursorLockMode.Locked;
        capsule = GetComponent<CapsuleCollider>();
        body = GetComponent<Rigidbody>();

        PhysicMaterial pm = new PhysicMaterial();pm.dynamicFriction = 0f;pm.staticFriction = 0f;pm.frictionCombine = PhysicMaterialCombine.Minimum;capsule.sharedMaterial = pm;
        body.useGravity = false;
        groundNormalW = transform.up;
    }
    void Update()
    {
        //Input
        inputStick = Vector2.ClampMagnitude(new Vector2(Input.GetAxis("Horizontal"),  Input.GetAxis("Vertical")), 1);
        inputStickRaw = Vector2.ClampMagnitude(new Vector2(Input.GetAxisRaw("Horizontal"), Input.GetAxisRaw("Vertical")), 1);
        if (debugRunY) inputStick.y=inputStickRaw.y = 1;
        if (Input.GetButton("Jump")) inputJump = true;

        //Attach Behavior
        if (attachReference != null)
        {
            attachVelocitySolverW.SetOld(lastAttachPositionW);
            attachVelocitySolverW.Update(attachReference.TransformPoint(lastAttachPositionR),Time.deltaTime);
            Vector3 referenceMovementW = (attachReference.TransformPoint(lastAttachPositionR) - lastAttachPositionW);
            body.position += referenceMovementW; transform.position = body.position;
            lastAttachPositionR = attachReference.InverseTransformPoint(transform.position);
            lastAttachPositionW = transform.position;
        }

    }
    private void FixedUpdate()
    {
        gizmosedSphereCastData.Clear();
        gravity = Physics.gravity;
        Vector3 gravityUp = gravity.magnitude > 0 ? -gravity.normalized : transform.up;
        Vector3 inputWRunJump = Vector3.ProjectOnPlane(Camera.main.transform.TransformDirection(new Vector3(inputStick.x, 0, inputStick.y)), transform.up).normalized * inputStick.magnitude;
        Vector3 inputWClimb = transform.TransformDirection(new Vector3(inputStick.x, inputStick.y, 0));

        //Detect Ground
        RaycastHit groundHit;
        isGrounded = GizmosedSphereCast(transform.TransformPoint(new Vector3(0, capsule.radius * 1.4f, 0)), capsule.radius * .9f, -transform.up, out groundHit, capsule.radius * 1f+stepOffset, Color.black);
        if( Vector3.Dot(velocityWR, groundNormalW) > .2f)isGrounded=false;//use old normal here
        if (Vector3.Dot(gravityUp, groundHit.normal) < Mathf.Cos(Mathf.Deg2Rad * slopeLimit)) isGrounded = false;
        groundPositionW = isGrounded ? groundHit.point : transform.position;
        groundNormalW = isGrounded ? groundHit.normal : transform.up;
        if (isGrounded) { lastGroundTime = 0; lastGroundNoJump = true; } else lastGroundTime += Time.fixedDeltaTime;
        if (isGrounded) transform.position = Vector3.ProjectOnPlane(transform.position, transform.up) + transform.up * Vector3.Dot(transform.up, groundPositionW);

        RaycastHit willFallHit;
        willFallFar = !GizmosedRayCast(groundPositionW + transform.up*capsule.radius * 2.5f+Vector3.ProjectOnPlane(body.velocity,transform.up)*0.3f, -transform.up, out willFallHit, capsule.radius * 3.5f, Color.black);

        //Process Stairs
        RaycastHit stairsHit;
        if (isGrounded && Vector3.Dot(inputWRunJump, transform.forward) > 0.1f)
        {
            if (!GizmosedSphereCast(transform.TransformPoint(new Vector3(0, capsule.radius * .9f + stepOffset, 0)), capsule.radius * .9f, transform.forward, out stairsHit, 2 * capsule.radius, Color.white))
                if (GizmosedSphereCast(transform.TransformPoint(new Vector3(0, capsule.radius * .9f + stepOffset, capsule.radius)), capsule.radius * .9f, -transform.up, out stairsHit, Mathf.Max(stepOffset - capsule.radius * .3f, 0), Color.white))
                    if (Vector3.Dot(gravityUp, stairsHit.normal) > Mathf.Cos(Mathf.Deg2Rad * slopeLimit)) 
                        transform.position += transform.up * (stepOffset - stairsHit.distance);
        }

        //Detect Climb
        RaycastHit climbHit;
        canClimb = GizmosedSphereCast(transform.TransformPoint(new Vector3(0,capsule.height * .75f,0)), capsule.radius * .9f, transform.forward, out climbHit, capsule.height * .25f, Color.white);
        if (Vector3.Dot(gravityUp, climbHit.normal) > Mathf.Cos(Mathf.Deg2Rad * slopeLimit)) canClimb = false;
        climbNormalW = canClimb ? climbHit.normal : -transform.forward;

        if (!isClimbing && canClimb && Vector3.Dot(body.velocity,climbNormalW)<.1f &&lastGroundTime>.5f)
        {//TODO jump up and climb velo.y<0 and velo.z==0
            isClimbing = true;
            inputJump = false;
        }

        if (!canClimb || isGrounded) isClimbing = false;
        Vector3 inputW = isClimbing ? inputWClimb : inputWRunJump;

        //TODO clamp climb out
        //TODO climb other wall
        //TODO lean and climb


        //Detece Ledge
        RaycastHit ledgeHit;
        canLedgeStart = false;
        bool canLedgeContinue = false;
        //can also vault when running
        if (!GizmosedSphereCast(transform.transform.TransformPoint(new Vector3(0, capsule.height - capsule.radius, 0)), capsule.radius * .9f, transform.forward, out ledgeHit, capsule.radius * 2.1f, Color.white))
             canLedgeContinue = GizmosedSphereCast(transform.transform.TransformPoint(new Vector3(0, capsule.height - capsule.radius, capsule.radius * 2.1f)), capsule.radius * .9f, -transform.up, out ledgeHit, capsule.height - capsule.radius - stepOffset, Color.white);
        canLedgeStart = canLedgeContinue;
        ledgeTargetW = canLedgeContinue ? ledgeHit.point : transform.position;
        Vector3 ledgeVectorW = ledgeTargetW - transform.position;

        if (!isLedging && canLedgeStart && inputJump)
            if(Vector3.Dot(inputW, ledgeVectorW.normalized)>0 || inputJump)
            {
                isLedging = true;
                inputJump = false;
            }
        if (!canLedgeContinue) isLedging = false;

        Vector3 ledgePlaneW = Vector3.Cross(ledgeVectorW, transform.right).normalized;
        //ledge and climb, ground can coexist


        //Process Jump
        if (inputJump)
        {
            if(isGrounded || lastGroundTime<.3f && lastGroundNoJump)
            {
                if (!willFallFar || !isGrounded)
                {
                    velocityWR += transform.up * 5f;
                    isClimbing = false;
                    isGrounded = false;
                    //todo anticipitation
                    inputJump = false;
                    lastGroundNoJump = false;
                }
                else inputJump = true;//wait for next frame
            }
            else if (isClimbing)
            {
                if (inputStickRaw.magnitude < 0.3f)
                    velocityWR = -transform.forward * 3f;
                else
                {
                    Vector3 inputWClimbRaw = transform.TransformDirection(new Vector3(inputStickRaw.x, inputStickRaw.y, 0));
                    velocityWR += Vector3.ProjectOnPlane(inputWClimbRaw, climbNormalW).normalized* inputStickRaw.magnitude * 5f;
                }

                isClimbing = false;
                isGrounded = false;
                inputJump = false;
            }
            else
                inputJump = false;
        }

        //ClimbJump

        //Jump Anticipation and Delay







        //Rotation Update
        //TODO rotate when climb
        //TODO rotate when jump
        //float velocityAngle = Vector3.ProjectOnPlane(body.velocity, transform.up).magnitude > .1f ? Mathf.Rad2Deg * Mathf.Atan2(Vector3.Dot(velocityWR, transform.right), Vector3.Dot(velocityWR, transform.forward)) : 0;
        Vector3 targetRotationForwardW = Vector3.ProjectOnPlane(transform.forward, gravityUp).normalized;
        if (targetRotationForwardW.magnitude==0) targetRotationForwardW = Vector3.Dot(gravityUp, targetRotationForwardW) > 0 ? -transform.up : transform.up;
        if (Vector3.ProjectOnPlane(body.velocity, gravityUp).magnitude > .5f) targetRotationForwardW = Vector3.ProjectOnPlane(body.velocity, gravityUp).normalized;
        if (Vector3.ProjectOnPlane(inputWRunJump, gravityUp).magnitude > .5f) targetRotationForwardW = Vector3.ProjectOnPlane(inputWRunJump, gravityUp).normalized;
        if (isClimbing && !isLedging) targetRotationForwardW =-climbNormalW;
        Quaternion targetRotation=Quaternion.LookRotation(targetRotationForwardW, gravityUp);
        transform.rotation=Quaternion.RotateTowards(transform.rotation, targetRotation, Time.fixedDeltaTime * 720);

        //Attach Behavior
        //Executes in Update
        attachReference = null;
        if (isGrounded) attachReference = groundHit.transform;
        if (isClimbing) attachReference = climbHit.transform; //Climbing attach has higher priority
        if (attachReference != null && attachReference != lastAttachReference) { lastAttachPositionR = attachReference.InverseTransformPoint(transform.position); lastAttachPositionW = transform.position; }
        lastAttachReference = attachReference;


        //Velocity Update
        //TODO bug when wall perpendicular to camera solution input align player
        //if(isClimbing)inputW = transform.TransformDirection(new Vector3(inputRaw.x, inputRaw.y,0 ));
        Vector3 inputPlaneW = isClimbing ? transform.forward : transform.up;
        Vector3 velocityPlaneW = isGrounded ? groundNormalW : (isClimbing ? climbNormalW : transform.up);
        if (isLedging) velocityPlaneW = ledgePlaneW;

        Vector3 targetProjectedVelocityW = targetSpeed * Vector3.ProjectOnPlane(Vector3.ProjectOnPlane(inputW, inputPlaneW), velocityPlaneW).normalized * inputW.magnitude;
        if (!isClimbing && !isGrounded && !isLedging) velocityWR += gravity * Time.fixedDeltaTime;
        else velocityWR = Vector3.ProjectOnPlane(velocityWR, velocityPlaneW);
        if (isLedging) velocityWR = ledgeVectorW.normalized * targetSpeed;
        velocityWR += Vector3.ClampMagnitude(targetProjectedVelocityW - Vector3.ProjectOnPlane(velocityWR, velocityPlaneW), Time.fixedDeltaTime * maxAcceleration);
        accelerationWREstimated = accelerationSolverWR.Update(velocityWR + attachVelocitySolverW.derivative, Time.fixedDeltaTime);//should before velocity update
        bodyVelocityW = body.velocity;
        body.velocity = velocityWR;


    }
    #region(Gizmos)
    struct GizmosedSphereCastData { public Vector3 origin; public float radius; public Vector3 direction; public float maxDistance; public bool result; public Vector3 point; public Vector3 normal; public Color gizmosColor; public int type; }
    List<GizmosedSphereCastData> gizmosedSphereCastData = new List<GizmosedSphereCastData>();
    bool GizmosedSphereCast(Vector3 origin,float radius, Vector3 direction, out RaycastHit raycastHit,float maxDistance,Color gizmosColor)
    {
        GizmosedSphereCastData data = new GizmosedSphereCastData();
        data.type = 0;
        data.origin = origin;data.radius = radius;data.direction = direction;data.maxDistance = maxDistance;data.gizmosColor = gizmosColor;
        data.result= Physics.SphereCast(origin, radius, direction, out raycastHit, maxDistance);
        data.point = raycastHit.point;
        data.normal = raycastHit.normal;
        gizmosedSphereCastData.Add(data);
        return data.result;
    }
    bool GizmosedRayCast(Vector3 origin, Vector3 direction, out RaycastHit raycastHit,float maxDistance,Color gizmosColor)
    {
        GizmosedSphereCastData data = new GizmosedSphereCastData();
        data.type = 1;
        data.origin = origin;  data.direction = direction; data.maxDistance = maxDistance; data.gizmosColor = gizmosColor;
        data.result = Physics.Raycast(origin, direction, out raycastHit, maxDistance);
        data.point = raycastHit.point;
        data.normal = raycastHit.normal;
        gizmosedSphereCastData.Add(data);
        return data.result;
    }
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;
        foreach(var data in gizmosedSphereCastData)
        {
            Gizmos.color = data.gizmosColor;
            if (data.type == 0)
            {
                Gizmos.DrawWireSphere(data.origin, data.radius);
                Gizmos.DrawWireSphere(data.origin + data.direction * data.maxDistance, data.radius);
            }
            Gizmos.DrawLine(data.origin, data.origin + data.direction * data.maxDistance);
            if (data.result)
            {
                Gizmos.DrawSphere(data.point, 0.03f);
                Gizmos.DrawLine(data.point, data.point + data.normal * 0.1f);
            }
        }
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.TransformPoint(new Vector3(0,capsule.height*.5f,0)), transform.TransformPoint(new Vector3(0, capsule.height * .5f, 0)) + accelerationWREstimated / maxAcceleration);
        Gizmos.DrawLine(transform.position, ledgeTargetW);
    }
    #endregion

    public bool debugRunY = false;
}