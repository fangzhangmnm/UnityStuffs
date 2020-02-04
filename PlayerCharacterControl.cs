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
    public float runSpeed = 3f, climbSpeed = 0.5f, jumpSpeed = 5f, climbJumpSpeed = 1.5f;
    public float targetSpeed, maxAcceleration = 10f;
    public float stepOffset = 0.3f,slopeLimit = 50f;
    [SerializeField, ReadOnly] Vector2 inputStick, inputStickRaw;
    [SerializeField, ReadOnly] public Vector3 velocityWR, accelerationWREstimated, bodyVelocityW;
    DerivativeVector3 accelerationSolverWR = new DerivativeVector3(0.1f);
    DerivativeVector3 attachVelocitySolverW = new DerivativeVector3(0.1f);

    [SerializeField, ReadOnly] public bool isGrounded=true, isWalkable,canClimb, isClimbing, canLedgeStart, isLedging, willFallFar,willClimbFall, isJumped;
    //can be climbing while grounded
    Transform attachReference = null, lastAttachReference = null;
    Vector3 lastAttachPositionW, lastAttachPositionR;
    public float lastGroundTime, cancelClimbTime;
    //bool lastGroundNoJump;
    public Vector3 groundPositionW, groundNormalW, climbPositionW,climbNormalW, ledgeTargetW;
    Vector3 gravity;
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
        if (Input.GetButtonDown("Jump")) inputJump = true;

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
        //TODO turn when climb
        //TODO hand ik
        //TODO stairs bug




        gizmosedSphereCastData.Clear();
        gravity = Physics.gravity;
        Vector3 gravityUp = gravity.magnitude > 0 ? -gravity.normalized : transform.up;
        Vector3 inputWRunJump = Vector3.ProjectOnPlane(Camera.main.transform.TransformDirection(new Vector3(inputStick.x, 0, inputStick.y)), transform.up).normalized * inputStick.magnitude;
        Vector3 inputWClimb = transform.TransformDirection(new Vector3(inputStick.x, inputStick.y, 0));

        //Detect Ground and Walkable
        //(Ground can be not walkable because of slope)
        RaycastHit groundHit;
        isGrounded = GizmosedSphereCast(transform.TransformPoint(new Vector3(0, capsule.radius * 1.4f, 0)), capsule.radius * .9f, -transform.up, out groundHit, capsule.radius * 1f+stepOffset, Color.black);
        if( Vector3.Dot(velocityWR, groundNormalW) > .2f)isGrounded=false;//use old normal here
        isWalkable = isGrounded;
        if (Vector3.Dot(gravityUp, groundHit.normal) < Mathf.Cos(Mathf.Deg2Rad * slopeLimit)) isWalkable = false;
        groundPositionW = isGrounded ? groundHit.point : transform.position;
        groundNormalW = isGrounded ? groundHit.normal : transform.up;
        if (isGrounded)  lastGroundTime = 0; else lastGroundTime += Time.fixedDeltaTime;
        //if (isGrounded && Vector3.Dot(body.velocity,groundNormalW)>=0) transform.position = Vector3.ProjectOnPlane(transform.position, transform.up) + transform.up * Vector3.Dot(transform.up, groundPositionW);

        RaycastHit willFallHit;
        willFallFar = !GizmosedRayCast(groundPositionW + transform.up*capsule.radius * 2.5f+Vector3.ProjectOnPlane(body.velocity,transform.up)*.1f, -transform.up, out willFallHit, capsule.radius * 3.5f, Color.black);

        //Process Stairs
        RaycastHit stairsHit;
        if (isWalkable && Vector3.Dot(inputWRunJump, transform.forward) > 0.1f)
        {
            if (!GizmosedSphereCast(transform.TransformPoint(new Vector3(0, capsule.radius * .9f + stepOffset, 0)), capsule.radius * .9f, transform.forward, out stairsHit, 2 * capsule.radius, Color.white))
                if (GizmosedSphereCast(transform.TransformPoint(new Vector3(0, capsule.radius * .9f + stepOffset, capsule.radius)), capsule.radius * .9f, -transform.up, out stairsHit, Mathf.Max(stepOffset - capsule.radius * .3f, 0), Color.white))
                    if (Vector3.Dot(gravityUp, stairsHit.normal) > Mathf.Cos(Mathf.Deg2Rad * slopeLimit))
                    {
                        float toRaise = Vector3.Dot(stairsHit.point - transform.position, transform.up) * 1.1f;
                        transform.position += transform.up * toRaise;
                    }
        }

        //Detece Ledge
        RaycastHit ledgeHit;
        canLedgeStart = false;
        bool canLedgeContinue = false;
        //can also vault when running
        if (!GizmosedSphereCast(transform.transform.TransformPoint(new Vector3(0, capsule.height - capsule.radius, 0)), capsule.radius * .9f, transform.forward, out ledgeHit, capsule.radius * 2.1f, Color.white))
            canLedgeContinue = GizmosedSphereCast(transform.transform.TransformPoint(new Vector3(0, capsule.height - capsule.radius, capsule.radius * 2.1f)), capsule.radius * .9f, -transform.up, out ledgeHit, capsule.height - capsule.radius - stepOffset, Color.white);
        if (Vector3.Dot(ledgeHit.normal, gravityUp) < Mathf.Cos(Mathf.Deg2Rad * slopeLimit))
            canLedgeContinue = false;
        canLedgeStart = canLedgeContinue;
        ledgeTargetW = canLedgeContinue ? ledgeHit.point : transform.position;
        Vector3 ledgeVectorW = ledgeTargetW - transform.position;

        if (!isLedging && canLedgeStart && inputJump)
            if (Vector3.Dot(isClimbing ? inputWClimb : inputWRunJump, ledgeVectorW.normalized) > 0 || inputJump)
            {
                isLedging = true;
                inputJump = false;
            }
        if (!canLedgeContinue) isLedging = false;
        if (isLedging) isClimbing = isWalkable = false;
        Vector3 ledgePlaneW = Vector3.Cross(ledgeVectorW, transform.right).normalized;
        //ledge and climb, ground can coexist

        //Detect Climb
        RaycastHit climbHit;
        canClimb = GizmosedSphereCast(transform.TransformPoint(new Vector3(0,capsule.height * .7f,0)), capsule.radius * .9f, transform.forward, out climbHit, capsule.height * .25f, Color.white);
        //TODO check neighs
        climbNormalW = canClimb ? climbHit.normal : -transform.forward;
        climbPositionW = canClimb ? climbHit.point : transform.position;
        if (Vector3.Dot(gravityUp, climbHit.normal) > Mathf.Cos(Mathf.Deg2Rad * slopeLimit)) canClimb = false;
        if (isGrounded) canClimb = false;

        if (!canClimb) isClimbing = false;
        if (!canClimb || isClimbing) cancelClimbTime = 0; else cancelClimbTime -= Time.fixedDeltaTime;
        if (canClimb && Vector3.Dot(body.velocity, climbNormalW) <.1f && cancelClimbTime <= 0)
        {
            if (!isClimbing) velocityWR = Vector3.zero;
            isClimbing = true;
        }

        RaycastHit willClimbFallHit;
        willClimbFall= !GizmosedRayCast(transform.TransformPoint(new Vector3(0, capsule.height * .7f, 0))+inputWClimb*climbSpeed*.1f, transform.forward, out willClimbFallHit, capsule.height * .25f+capsule.radius, Color.white);
        if (willClimbFall)
        {
            inputWClimb = Vector3.zero;
        }
        




        //TODO clamp climb out
        //TODO climb other wall
        //TODO lean and climb





        //Process Jump
        if (isGrounded || isLedging || isClimbing) isJumped = false;
        if (inputJump)
        {
            if(isGrounded || (lastGroundTime<.1f && !isJumped))
            {
                if (!willFallFar || !isGrounded)
                {
                    velocityWR += transform.up * jumpSpeed;
                    isClimbing = false;
                    isGrounded = false;
                    isWalkable = false;
                    //todo anticipitation
                    inputJump = false;
                    isJumped = true;
                    cancelClimbTime = .1f;
                }
                else inputJump = inputJump = true;//wait for next frame
            }
            else if (isClimbing)
            {
                if (inputStickRaw.magnitude < 0.3f)
                {
                    isClimbing = false;
                    inputJump = false;

                    velocityWR += Vector3.zero;
                    cancelClimbTime = float.PositiveInfinity;
                }
                else
                {
                    inputJump = false;
                    /*
                    isClimbing = false;
                    inputJump = false;
                    isClimbJumping = true;

                    cancelClimbTime = .5f;
                    Vector3 inputWClimbRaw = transform.TransformDirection(new Vector3(inputStickRaw.x, inputStickRaw.y, 0));
                    velocityWR += Vector3.ProjectOnPlane(inputWClimbRaw, climbNormalW).normalized * inputStickRaw.magnitude * climbJumpSpeed;
                    velocityWR += Vector3.ProjectOnPlane(-gravity, climbNormalW) * 2 * cancelClimbTime * .1f;
                    */
                }

            }
            else
                inputJump = false;//Discard trigger
        }

        //Attach Behavior
        //Executes in Update
        attachReference = null;
        if (isWalkable) attachReference = groundHit.transform;
        if (isClimbing) attachReference = climbHit.transform; //Climbing attach has higher priority
        if (attachReference != null && attachReference != lastAttachReference) { lastAttachPositionR = attachReference.InverseTransformPoint(transform.position); lastAttachPositionW = transform.position; }
        lastAttachReference = attachReference;


        //Velocity Update
        //TODO bug when wall perpendicular to camera solution input align player
        //if(isClimbing)inputW = transform.TransformDirection(new Vector3(inputRaw.x, inputRaw.y,0 ));
        targetSpeed = isClimbing ? climbSpeed : runSpeed;//will be more
        Vector3 inputPlaneW = isClimbing ? transform.forward : transform.up;
        Vector3 velocityPlaneW = isWalkable ? groundNormalW : (isClimbing ? climbNormalW : transform.up);
        float velocityPlaneDistance = isWalkable ? Vector3.Dot(transform.position - groundPositionW, transform.up) : (isClimbing ? Vector3.Dot(transform.position - climbPositionW, climbNormalW) - capsule.radius : 0);
        if (isLedging) velocityPlaneW = ledgePlaneW;
        
        Vector3 targetProjectedVelocityW = targetSpeed * Vector3.ProjectOnPlane(Vector3.ProjectOnPlane(isClimbing ? inputWClimb : inputWRunJump, inputPlaneW), velocityPlaneW).normalized * inputStick.magnitude;
        if (isClimbing || isWalkable || isLedging)
        {
            float vy = Vector3.Dot(velocityWR, velocityPlaneW);
            vy = Mathf.Clamp(vy, -velocityPlaneDistance / Time.fixedDeltaTime, 0);
            velocityWR = Vector3.ProjectOnPlane(velocityWR, velocityPlaneW) + vy * velocityPlaneW;
        }
        else
        {
            velocityWR += gravity * Time.fixedDeltaTime;
        }
        if (isLedging) velocityWR = ledgeVectorW.normalized * targetSpeed;
        velocityWR += Vector3.ClampMagnitude(targetProjectedVelocityW - Vector3.ProjectOnPlane(velocityWR, velocityPlaneW), Time.fixedDeltaTime * maxAcceleration);
        accelerationWREstimated = accelerationSolverWR.Update(velocityWR + attachVelocitySolverW.derivative, Time.fixedDeltaTime);//should before velocity update
        bodyVelocityW = body.velocity;
        body.velocity = velocityWR;

        //Rotation Update
        //TODO rotate when climb
        //TODO rotate when jump
        //float velocityAngle = Vector3.ProjectOnPlane(body.velocity, transform.up).magnitude > .1f ? Mathf.Rad2Deg * Mathf.Atan2(Vector3.Dot(velocityWR, transform.right), Vector3.Dot(velocityWR, transform.forward)) : 0;
        Vector3 targetRotationForwardW = Vector3.ProjectOnPlane(transform.forward, gravityUp).normalized;
        if (targetRotationForwardW.magnitude == 0) targetRotationForwardW = Vector3.Dot(gravityUp, targetRotationForwardW) > 0 ? -transform.up : transform.up;
        if (Vector3.ProjectOnPlane(body.velocity, gravityUp).magnitude > .5f) targetRotationForwardW = Vector3.ProjectOnPlane(body.velocity, gravityUp).normalized;
        if (Vector3.ProjectOnPlane(inputWRunJump, gravityUp).magnitude > .5f) targetRotationForwardW = Vector3.ProjectOnPlane(inputWRunJump, gravityUp).normalized;
        if (isClimbing && !isLedging) targetRotationForwardW = -climbNormalW;
        Quaternion targetRotation = Quaternion.LookRotation(targetRotationForwardW, gravityUp);
        transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, Time.fixedDeltaTime * 720);


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
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.TransformPoint(new Vector3(0, capsule.height * .5f, 0)), transform.TransformPoint(new Vector3(0, capsule.height * .5f, 0)) + velocityWR / runSpeed);
    }
    #endregion

    public bool debugRunY = false;
}