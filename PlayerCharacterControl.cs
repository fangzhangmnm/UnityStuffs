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
    public Transform attachReference = null;Transform lastAttachReference = null;
    Vector3 lastAttachPositionW, lastAttachPositionR;
    public float lastWalkableTime, cancelClimbTime;
    //bool lastGroundNoJump;
    public Vector3 groundPositionW, groundNormalW, climbPositionW,climbNormalW, ledgeTargetR;
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
        //TODO stairs bug
        //TODO hand ik handfoot attach
        //TODO bounce out when jumping touching edge
        //TODO footstep sound



        gizmosedSphereCastData.Clear();
        gravity = Physics.gravity;
        Vector3 gravityUp = gravity.magnitude > 0 ? -gravity.normalized : transform.up;
        Vector3 inputWRunJump = Vector3.ProjectOnPlane(Camera.main.transform.TransformDirection(new Vector3(inputStick.x, 0, inputStick.y)), transform.up).normalized * inputStick.magnitude;
        Vector3 inputWClimb = transform.TransformDirection(new Vector3(inputStick.x, inputStick.y, 0));

        //Detect Ground and Walkable
        //(Ground can be not walkable because of slope)
        RaycastHit groundHit;
        isGrounded = GizmosedSphereCast(transform.TransformPoint(new Vector3(0, capsule.radius * 1.5f, 0)), capsule.radius * .5f, -gravityUp, out groundHit, capsule.radius * 1.1f+stepOffset, Color.black);
        if( Vector3.Dot(velocityWR, groundNormalW) > .2f)isGrounded=false;//use old normal here
        isWalkable = isGrounded;
        if (Vector3.Dot(gravityUp, groundHit.normal) < Mathf.Cos(Mathf.Deg2Rad * slopeLimit)) isWalkable = false;
        //Debug.Log(Vector3.Dot(gravityUp, groundHit.normal));
        groundPositionW = isGrounded ? groundHit.point : transform.position;
        groundNormalW = isGrounded ? groundHit.normal : transform.up;
        //if (isGrounded && Vector3.Dot(body.velocity,groundNormalW)>=0) transform.position = Vector3.ProjectOnPlane(transform.position, transform.up) + transform.up * Vector3.Dot(transform.up, groundPositionW);

        RaycastHit willFallHit;
        willFallFar = !GizmosedRayCast(groundPositionW + transform.up*capsule.radius * 2.5f+Vector3.ProjectOnPlane(body.velocity,transform.up)*.1f, -transform.up, out willFallHit, capsule.radius * 3.5f, Color.black);

        //Process Stairs
        RaycastHit stairsHit;
        if (isWalkable && Vector3.Dot(inputWRunJump, transform.forward) > 0.1f)
        {
            if (!GizmosedSphereCast(transform.TransformPoint(new Vector3(0, capsule.radius * .9f + stepOffset, 0)), capsule.radius * .9f, transform.forward, out stairsHit, 2 * capsule.radius, Color.white))
                if (GizmosedSphereCast(transform.TransformPoint(new Vector3(0, capsule.radius * .9f + stepOffset, capsule.radius)), capsule.radius * .9f, -gravityUp, out stairsHit, Mathf.Max(stepOffset - capsule.radius * .3f, 0), Color.white))
                    if (Vector3.Dot(gravityUp, stairsHit.normal) > Mathf.Cos(Mathf.Deg2Rad * slopeLimit))
                    {
                        float toRaise = Vector3.Dot(stairsHit.point - transform.position, transform.up) * 1.1f;
                        transform.position += transform.up * Mathf.Min(toRaise,Time.fixedDeltaTime*runSpeed);
                    }
        }

        //Detece Ledge
        RaycastHit ledgeHit;
        canLedgeStart = false;
        bool canLedgeContinue = false;
        //can also vault when running
        if (!GizmosedSphereCast(transform.transform.TransformPoint(new Vector3(0, capsule.height - capsule.radius, 0)), capsule.radius * .5f, transform.forward, out ledgeHit, capsule.radius * 2.1f, Color.white))
            canLedgeContinue = GizmosedSphereCast(transform.transform.TransformPoint(new Vector3(0, capsule.height - capsule.radius, capsule.radius * 2.1f)), capsule.radius * .5f, -transform.up, out ledgeHit, capsule.height, Color.white);
        if (Vector3.Dot(ledgeHit.normal, gravityUp) < Mathf.Cos(Mathf.Deg2Rad * slopeLimit))
            canLedgeContinue = false;
        canLedgeStart = canLedgeContinue && ledgeHit.distance<capsule.height-capsule.radius-stepOffset;

        //ledgeTargetR = canLedgeContinue ? ledgeHit.point : transform.position;

        if (!isLedging && canLedgeStart && inputJump)
            if (Vector3.Dot(isClimbing ? inputWClimb : inputWRunJump, (ledgeHit.point - transform.position).normalized) > 0 || inputJump)
            {
                ledgeTargetR = ledgeHit.transform.InverseTransformPoint(ledgeHit.point);
                attachReference = ledgeHit.transform;
                isLedging = true;
                isGrounded = isWalkable = false;
                inputJump = false;
            }
        Vector3 ledgeVectorW = attachReference == null ? Vector3.zero : attachReference.TransformPoint(ledgeTargetR) - transform.position;
        if (!canLedgeContinue || isWalkable) isLedging = false;
        if (isLedging) isClimbing = isWalkable=isGrounded = false;
        Vector3 ledgePlaneW = Vector3.Cross(ledgeVectorW, transform.right).normalized;
        //if (isLedging) Debug.Log(attachReference);
        //ledge and climb, ground can coexist

        //Detect Climb
        RaycastHit climbHit,tmpClimbHit;
        float canClimbDist = capsule.height * .4f;
        canClimb = GizmosedSphereCast(transform.TransformPoint(new Vector3(0,capsule.height * .7f,0)), capsule.radius * .5f, transform.forward, out climbHit, canClimbDist+ capsule.height * .7f*Mathf.Tan(Mathf.Deg2Rad*(90-slopeLimit)), Color.white);
        if (Vector3.Dot(gravityUp, climbHit.normal) > Mathf.Cos(Mathf.Deg2Rad * slopeLimit)) canClimb = false;
        if (canClimb) {//detect lean surfaces
            float s = Vector3.Dot(climbHit.normal, transform.up);
            if (s >= .9f) canClimb = false; else
            {
                float t = Mathf.Max(0,s) / Mathf.Sqrt(1 - s * s);
                if (climbHit.distance > canClimbDist + capsule.height * .7f * t) canClimb= false;
                //Debug.Log($"{canClimb}:{climbHit.distance }vs{canClimbDist}+{capsule.height * .7f * t},{t}");
            }
        }
        if (canClimb)
        {
            Vector3 wallRight = Vector3.ProjectOnPlane(transform.right, climbHit.normal).normalized;//WRONG, should use stepoffset
            if (Vector3.Dot(inputWClimb, transform.right) > 0 && GizmosedRayCast(climbHit.point+climbHit.normal*.1f*capsule.radius, wallRight, out tmpClimbHit, capsule.radius * 1.1f, Color.white)) climbHit = tmpClimbHit;
            if (Vector3.Dot(inputWClimb, transform.right) < 0 && GizmosedRayCast(climbHit.point + climbHit.normal * .1f * capsule.radius, -wallRight, out tmpClimbHit, capsule.radius * 1.1f, Color.white)) climbHit = tmpClimbHit;
        }
        //TODO check neighs
        climbNormalW = canClimb ? climbHit.normal : -transform.forward;
        climbPositionW = canClimb ? climbHit.point : transform.TransformPoint(new Vector3(0, capsule.height * .7f, 0));
        float climbWallDist = Mathf.Max(0,climbHit.distance - capsule.radius * .15f);
        if (isWalkable) canClimb = false;

        if (!canClimb) isClimbing = false;
        if (!canClimb || isClimbing) cancelClimbTime = 0; else cancelClimbTime -= Time.fixedDeltaTime;
        if (canClimb &&!isClimbing && Vector3.Dot(body.velocity, climbNormalW) <.1f && cancelClimbTime <= 0 )// && Vector3.Dot(body.velocity,gravityUp)<=.1f
        {
            velocityWR = Vector3.zero;
            isClimbing = true;
        }
        if (isClimbing)
        {
            transform.position -= climbHit.normal * Mathf.Clamp(climbWallDist,0,Time.fixedDeltaTime*climbSpeed); 
        }
        
        RaycastHit willClimbFallHit;
        if(Vector3.Dot(inputWClimb,transform.up)>0)
            willClimbFall = !GizmosedRayCast(transform.TransformPoint(new Vector3(0, capsule.height * .7f, 0)) + inputWClimb.normalized * (climbSpeed * .1f), transform.forward, out willClimbFallHit, canClimbDist + capsule.radius * .5f, Color.white);
        else
            willClimbFall = !GizmosedSphereCast(transform.TransformPoint(new Vector3(0, capsule.height * .7f, 0))+inputWClimb.normalized*(climbSpeed*.1f), capsule.radius*.5f, transform.forward, out willClimbFallHit, canClimbDist, Color.white);
        willClimbFall = willClimbFall || Vector3.Dot(gravityUp, willClimbFallHit.normal) > Mathf.Cos(Mathf.Deg2Rad * slopeLimit);
        if (willClimbFall)
        {
            inputWClimb = Vector3.zero;
        }




        //TODO clamp climb out
        //TODO climb other wall
        //TODO lean and climb





        //Process Jump
        if (isWalkable) lastWalkableTime = 0; else lastWalkableTime += Time.fixedDeltaTime;
        if (isGrounded || isLedging || isClimbing) isJumped = false;
        if (inputJump)
        {
            if(isWalkable || (lastWalkableTime<.1f && !isJumped))
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
                    cancelClimbTime = .5f;
                    Vector3 inputWClimbRaw = transform.TransformDirection(new Vector3(inputStickRaw.x, inputStickRaw.y, 0));
                    velocityWR += Vector3.ProjectOnPlane(inputWClimbRaw, climbNormalW).normalized * inputStickRaw.magnitude * climbJumpSpeed;
                    velocityWR += Vector3.ProjectOnPlane(-gravity, climbNormalW) * 2 * cancelClimbTime;
                    */
                }

            }
            else
                inputJump = false;//Discard trigger
        }

        //Attach Behavior
        //Executes in Update
        if (!isLedging)
        {
            if (isWalkable) attachReference = groundHit.transform;
            if (isClimbing) attachReference = climbHit.transform; //Climbing attach has higher priority
        }
        if (attachReference != null && attachReference != lastAttachReference) { lastAttachPositionR = attachReference.InverseTransformPoint(transform.position); lastAttachPositionW = transform.position; }
        lastAttachReference = attachReference;

        
        //Velocity Update
        //TODO bug when wall perpendicular to camera solution input align player
        //if(isClimbing)inputW = transform.TransformDirection(new Vector3(inputRaw.x, inputRaw.y,0 ));
        targetSpeed = isClimbing ? climbSpeed : runSpeed;//will be more
        Vector3 velocityPlaneW = isWalkable ? groundNormalW : (isClimbing ? climbNormalW : transform.up);
        float velocityPlaneDistance = isWalkable ? Vector3.Dot(transform.position - groundPositionW, groundNormalW) : (isClimbing ? Vector3.Dot(transform.position - climbPositionW, climbNormalW) - capsule.radius : 0);
        if (velocityPlaneDistance < 0) velocityPlaneDistance = 0;
        if (isLedging) velocityPlaneW = ledgePlaneW;

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

        

        Vector3 targetProjectedVelocityW = targetSpeed * Vector3.ProjectOnPlane(isClimbing ? inputWClimb : inputWRunJump, velocityPlaneW).normalized * inputStick.magnitude;
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
        float rotateSpeed = (isWalkable) ? 720 : 180;//to avoid jitter caused by rotate capsule into wall and then climb detect raycast went wrong //TODO beter solution

        //Vector3 rotateCenterW = transform.TransformPoint(Vector3.up * capsule.height / 2);
        transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, Time.fixedDeltaTime * rotateSpeed);
        //transform.position+= rotateCenterW- transform.TransformPoint(Vector3.up * capsule.height / 2);

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
        if(isLedging)
            Gizmos.DrawLine(transform.position, attachReference.TransformPoint(ledgeTargetR));
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.TransformPoint(new Vector3(0, capsule.height * .5f, 0)), transform.TransformPoint(new Vector3(0, capsule.height * .5f, 0)) + velocityWR / runSpeed);
    }
    #endregion

    public bool debugRunY = false;
}