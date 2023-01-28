using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FrictionModel
{
    public float maxDegreeAdherence;
    public bool isSliding;
    public float speedSlide = 0;
    
    public FrictionModel(float maxDegreeAdherence)
    {
        isSliding = false;
        this.maxDegreeAdherence = maxDegreeAdherence;
    }

    public bool CheckForces(Vector3 speedCharacter, Vector3 floorNormal)
    {
        Vector3 reactionNormal = floorNormal;
        Vector3 reactionFriction = - speedCharacter;
        Vector3 totalReaction = reactionNormal + reactionFriction;

        float thetaAngle = Vector3.Angle(reactionNormal, totalReaction);
        
        
        if (!isSliding && thetaAngle > maxDegreeAdherence)
        {
            //Debug.Log("Normal " + reactionNormal + " Friction " + reactionFriction + " Theta angle " + thetaAngle);
            //Debug.DrawRay(DeformTerrainMaster.Instance.leftFootCollider.transform.position, totalReaction / 2, Color.blue, 1f);
            //Debug.Log("Start sliding");
            speedSlide = (speedCharacter.magnitude - totalReaction.magnitude) * thetaAngle;
            isSliding = true;
        }
        else if (isSliding && thetaAngle < maxDegreeAdherence * 0.80f) // Random value here
        {
            //Debug.Log("Normal " + reactionNormal + " Friction " + reactionFriction + " Theta angle " + thetaAngle);
            //Debug.DrawRay(DeformTerrainMaster.Instance.leftFootCollider.transform.position, totalReaction / 2, Color.blue, 1f);
            //Debug.Log("Stop sliding");
            isSliding = false;
        }
        
        return isSliding; 
    }

    static public Vector3 GetFloorNormalFromFeet(bool right, LayerMask ground)
    {
        bool grounded = right
            ? DeformTerrainMaster.Instance.isRightFootGrounded
            : DeformTerrainMaster.Instance.isLeftFootGrounded;
        Collider collider = right
            ? DeformTerrainMaster.Instance.rightFootCollider
            : DeformTerrainMaster.Instance.leftFootCollider;
        if (grounded)
        {
            RaycastHit hit;
            Ray ray = new Ray(collider.transform.position, Vector3.down);
            if (Physics.Raycast(ray, out hit, 10, ground))
            {
                return hit.normal;
            }
        }
        return Vector3.zero;
    }
}
