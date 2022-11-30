﻿/****************************************************
 * File: DeformTerrainMaster.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 01/08/2021
   * Project: Real-Time Locomotion on Soft Grounds with Dynamic Footprints
   * Last update: 07/02/2022
*****************************************************/

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public enum sourceDeformation
{
    useUI,
    useTerrainPrefabs,
    useManualValues
};

/// <summary>
/// Master class where we calculate all forces taking place during gait and call footprint child class.
/// </summary>
public class DeformTerrainMaster : MonoBehaviour
{
    #region Instance Fields

    [Header("Bipedal - (SET UP)")]
    [Tooltip("Your character GameObject")]
    public GameObject myBipedalCharacter;
    [Tooltip("Collider attached to Left Foot to leave footprints")]
    public Collider leftFootCollider;
    [Tooltip("Collider attached to Right Foot to leave footprints")]
    public Collider rightFootCollider;
    [Tooltip("RB attached to Left Foot used for velocity estimation")]
    public Rigidbody leftFootRB;
    [Tooltip("RB attached to Right Foot for velocity estimation")]
    public Rigidbody rightFootRB;
    [Tooltip("Particle system attached to Left Foot to leave footprints particles")]
    public ParticleSystem leftFootPS;
    [Tooltip("Particle system attached to Right Foot to leave footprints particles")]
    public ParticleSystem rightFootPS;

    [Header("Source of Deformation - (SET UP)")]
    public sourceDeformation deformationChoice;

    [Header("Terrain Deformation - Contact Time Settings - (SET UP)")]
    public float timePassed = 0f;
    [Tooltip("Time that the terrain requires to absorve the force from the hitting foot. More time results in a smaller require force. On the other hand, for less time, the terrain requires a larger force to stop the foot.")]
    public float contactTime = 0.1f;
    [Tooltip("Small delay, sometimes needed, to give the system enough time to perform the deformation.")]
    private float offset = 0.2f; // 0.5f

    [Header("Terrain Prefabs - Settings - (SET UP)")]
    [Space(10)]
    public double youngModulusSnow = 200000;
    public float timeSnow = 0.2f;
    public bool bumpSnow = false;
    public float poissonRatioSnow = 0.1f;
    public int filterIterationsSnow = 0;
    public int quantitySnow = 10;
    public float sizeSnow = 0.2f;
    public float densitySnow = 10;
    public float projectionAngleSnow = 15;
    [Space(10)]
    public double youngModulusDrySand = 600000;
    public float timeDrySand = 0.3f;
    public bool bumpSand = false;
    public float poissonRatioSand = 0.2f;
    public int filterIterationsSand = 5;
    public int quantityDrySand = 40;
    public float sizeDrySand = 0.02f;
    public float densityDrySand = 0.5f;
    public float projectionAngleDrySand = 30;
    [Space(10)]
    public double youngModulusMud = 350000;
    public float timeMud = 0.8f;
    public bool bumpMud = false;
    public float poissonRatioMud = 0.4f;
    public int filterIterationsMud = 2;
    public int quantityMud = 10;
    public float sizeMud = 0.05f;
    public float densityMud = 1;
    public float projectionAngleMud = 45;
    [Space(10)]
    public double youngModulusSoil = 350000;
    public float timeSoil = 0.8f;
    public bool bumpSoil = false;
    public float poissonRatioSoil = 0.4f;
    public int filterIterationsSoil = 2;
    public int quantitySoil = 4;
    public float sizeSoil = 0.05f;
    public float densitySoil = 1;
    public float projectionAngleSoil = 22;

    [Header("Bipedal - System Info")]
    [Space(20)]
    public float mass;
    public bool printFeetPositions = false;
    public bool isLeftFootGrounded;
    public bool isRightFootGrounded;
    public float weightInLeftFoot;
    public float weightInRightFoot;
    public float heightIKLeft;
    public float heightIKRight;
    public Vector3 centerGridLeftFootHeight;
    public Vector3 centerGridRightFootHeight;
    private Vector3 centerGridLeftFoot;
    private Vector3 centerGridRightFoot;
    private Vector3 newIKLeftPosition;
    private Vector3 newIKRightPosition;
    private Vector3 oldIKLeftPosition;
    private Vector3 oldIKRightPosition;
    private Animator _anim;
    private IKFeetPlacement _feetPlacement = null;
    private bool wasLeftFootGrounded;
    private bool wasRightFootGrounded;

    [Header("Bipedal - Physics - Debug")]
    [Space(20)]
    public bool printFeetForces = false;
    public bool drawWeightForces = false;
    public bool drawMomentumForces = false;
    public bool drawGRForces = false;
    public bool drawFeetForces = false;
    public bool drawVelocities = false;
    public bool drawNewVelocities = false;

    [Header("Bipedal - Physics - Weight Forces Info")]
    public Vector3 weightForce;
    public Vector3 weightForceLeft;
    public Vector3 weightForceRight;

    [Header("Bipedal - Physics - Feet Velocities Info")]
    public Vector3 feetSpeedLeft = Vector3.zero;
    public Vector3 feetSpeedRight = Vector3.zero;

    [Header("Bipedal - Physics - Impulse and Momentum Forces Info")]
    public Vector3 feetImpulseLeft = Vector3.zero;
    public Vector3 feetImpulseRight = Vector3.zero;
    public Vector3 momentumForce = Vector3.zero;
    public Vector3 momentumForceLeft = Vector3.zero;
    public Vector3 momentumForceRight = Vector3.zero;

    [Header("Bipedal - Physics - GRF and Feet Forces Info")]
    public Vector3 totalGRForce;
    public Vector3 totalGRForceLeft;
    public Vector3 totalGRForceRight;
    public Vector3 totalForceFoot;
    public Vector3 totalForceLeftFoot;
    public Vector3 totalForceRightFoot;

    [Header("Max and Min Feet Forces Info")]
    private float minTotalForceLeftFootZ = 0f;
    private float maxTotalForceLeftFootZ = 0f;
    private float minTotalForceRightFootZ = 0f;
    private float maxTotalForceRightFootZ = 0f;
    private float minTotalForceLeftFootZNorm = 0f;
    private float maxTotalForceLeftFootZNorm = 0f;
    private float minTotalForceRightFootZNorm = 0f;
    private float maxTotalForceRightFootZNorm = 0f; 
    private float minTotalForceLeftFootZOld = 0f;
    private float maxTotalForceLeftFootZOld = 0f;
    private float minTotalForceRightFootZOld = 0f;
    private float maxTotalForceRightFootZOld = 0f;

    [Header("UI for DEMO mode")]
    [Space(20)]
    public Slider youngSlider;
    public Slider timeSlider;
    public Slider poissonSlider;
    public Slider iterationsSlider;
    public Toggle activateToggleDef;
    public Toggle activateToggleBump;
    public Toggle activateToggleGauss;
    public Toggle activateToggleShowGrid;
    public Toggle activateToggleShowBump;
    public Toggle activateToggleShowForceModel;

    #endregion

    #region Read-only & Static Fields

    // Types of brushes
    private BrushPhysicalFootprint brushPhysicalFootprint;

    // Terrain Properties
    private Terrain terrain;
    private Collider terrain_collider;
    private TerrainData terrain_data;
    private Vector3 terrain_size;
    private int heightmap_width;
    private int heightmap_height;
    private float[,] heightmap_data;
    private float[,] heightmap_data_constant;
    private float[,] heightmap_data_filtered;

    // Additional
    private bool oldIsMoving = false;
    private bool isMoving = false;
    private bool oldIsJumping = false;
    private bool isJumping = false;
    private int provCounter = 0;

    // Foot particles
    private bool leftFootParticlesLock = false;
    private bool rightFootParticlesLock = false;

    #endregion

    #region Plotting

    //               Extra for plotting              //
    // ============================================= //
    [UPyPlot.UPyPlotController.UPyProbe]
    private float weightForceLeftYFloat = 0f;
    [UPyPlot.UPyPlotController.UPyProbe]
    private float weightForceRightYFloat = 0f;
    [UPyPlot.UPyPlotController.UPyProbe]
    private float weightForceYFloat = 0f;

    [UPyPlot.UPyPlotController.UPyProbe]
    private float momentumForceLeftYFloat = 0f;
    [UPyPlot.UPyPlotController.UPyProbe]
    private float momentumForceRightYFloat = 0f;
    [UPyPlot.UPyPlotController.UPyProbe]
    private float momentumForceYFloat = 0f;

    [UPyPlot.UPyPlotController.UPyProbe]
    private float totalGRForceLeftYFloat;
    [UPyPlot.UPyPlotController.UPyProbe]
    private float totalGRForceRightYFloat;
    [UPyPlot.UPyPlotController.UPyProbe]
    private float totalGRForceYFloat;
    // ============================================= //

    #endregion

    #region Unity Methods

    void Start()
    {
        // 1. Extract terrain information
        if (!terrain)
        {
            //terrain = Terrain.activeTerrain;
            terrain = myBipedalCharacter.GetComponent<RigidBodyControllerSimpleAnimator>().currentTerrain;
            Debug.Log("[INFO] First terrain: " + terrain.name);
        }

        terrain_collider = terrain.GetComponent<Collider>();
        terrain_data = terrain.terrainData;
        terrain_size = terrain_data.size;
        heightmap_width = terrain_data.heightmapResolution;
        heightmap_height = terrain_data.heightmapResolution;
        heightmap_data = terrain_data.GetHeights(0, 0, heightmap_width, heightmap_height);
        heightmap_data_constant = terrain_data.GetHeights(0, 0, heightmap_width, heightmap_height);
        heightmap_data_filtered = terrain_data.GetHeights(0, 0, heightmap_width, heightmap_height);
        brushPhysicalFootprint = null;

        // 2. Get classes
        _feetPlacement = FindObjectOfType<IKFeetPlacement>();

        // 3. Retrieve components and attributes from character
        mass = myBipedalCharacter.GetComponent<Rigidbody>().mass;
        _anim = myBipedalCharacter.GetComponent<Animator>();

        // Old Feet Y-component position
        oldIKLeftPosition = _anim.GetBoneTransform(HumanBodyBones.LeftFoot).position;
        oldIKRightPosition = _anim.GetBoneTransform(HumanBodyBones.RightFoot).position;
        
        oldIsMoving = _anim.GetBool("isWalking");
        oldIsJumping = _anim.GetBool("isJumping"); // NEW
    }

    void Update()
    {
        // 1. Define source of deformation where we are
        DefineSourceDeformation();

        // 2. If we change the terrain, we change the data as well - both must have different GameObject names
        UpdateTerrain();

        // 3. Saving other variables for debugging purposes
        UpdateFeetPositions();

        // 4. Calculate Force Model
        CalculateForceModel();

        // Quadrupeds Information //
        /////////// TODO ///////////

        // 5. Print feet and force information if wished
        DebugForceModel();

        // 6. Apply procedural brush to deform the terrain based on the force model
        ApplyDeformation();
    }

    void LateUpdate()
    {
        ApplyFootParticles();
    }

    public void FixedUpdate()
    {
        // 1. Calculate Velocity for the feet
        CalculateFeetVelocity();
    }

    #endregion

    #region Instance Methods

    private void ApplyFootParticles()
    {
        // // If foot grounded we unlock particle system so that it can be played again
        // if (isLeftFootGrounded)
        //     leftFootParticlesLock = false;
        // if (isRightFootGrounded)
        //     rightFootParticlesLock = false;
        //
        // // If foot is not grounded we lock particle system so that it can't be played again
        // if (momentumForceLeft.y < 0.0f)
        // {
        //     leftFootParticlesLock = true;
        //     leftFootPS.Play();
        // }
        // if (momentumForceRight.y < 0.0f)
        // {
        //     rightFootParticlesLock = true;
        //     rightFootPS.Play();
        // }

        var t = myBipedalCharacter.GetComponent<RigidBodyControllerSimpleAnimator>().currentTerrain.name;

        if (wasLeftFootGrounded && !isLeftFootGrounded)
        {
            leftFootPS.emissionRate = getEmissionRate(feetSpeedLeft.y, t);
            var mainModule = leftFootPS.main;
            mainModule.startSize = getStartSize(t);
            var ps_shape = leftFootPS.shape;
            ps_shape.angle = getShapeAngle(t);
            leftFootPS.Play();
        }

        wasLeftFootGrounded = isLeftFootGrounded;
        
        if (wasRightFootGrounded && !isRightFootGrounded)
        {
            rightFootPS.emissionRate = getEmissionRate(feetSpeedRight.y, t);
            var mainModule = rightFootPS.main;
            mainModule.startSize = getStartSize(t);
            var ps_shape = rightFootPS.shape;
            ps_shape.angle = getShapeAngle(t);
            rightFootPS.Play();
        }

        wasRightFootGrounded = isRightFootGrounded;
        
    }

    private float getEmissionRate(float speed, string terrain)
    {
        float emissionRate = 100 * speed;
        switch (terrain)
        {
            case "Snow":
                emissionRate += quantitySnow;
                emissionRate /= densitySnow;
                break;
            case "Dry Sand":
                emissionRate += quantityDrySand;
                emissionRate /= densityDrySand;
                break;
            case "Mud":
                emissionRate += quantityMud;
                emissionRate /= densityMud;
                break;
            default:
                emissionRate += quantitySoil;
                emissionRate /= densitySoil;
                break;
        }
        return emissionRate;
    }

    private float getStartSize(string terrain)
    {
        switch (terrain)
        {
            case "Snow":
                return sizeSnow;
            case "Dry Sand":
                return sizeDrySand;
            case "Mud":
                return sizeMud;
            default:
                return sizeSoil;
        }
    }

    private float getShapeAngle(string terrain)
    {
        switch (terrain)
        {
            case "Snow":
                return projectionAngleSnow;
            case "Dry Sand":
                return projectionAngleDrySand;
            case "Mud":
                return projectionAngleMud;
            default:
                return projectionAngleSoil;
        }
    }

    private void ApplyDeformation()
    {
        // A. Apply brush to feet
        if (brushPhysicalFootprint)
        {
            // Brush is only called if we are within the contactTime
            // Due to the small values, the provisional solution requires to add an offset to give the system enough time to create the footprint
            timePassed += Time.deltaTime;
            if (timePassed <= contactTime + offset)
            {
                // Brush that takes limbs positions and creates physically-based footprints
                brushPhysicalFootprint.CallFootprint(_feetPlacement.LeftFootIKPosition.x, _feetPlacement.LeftFootIKPosition.z,
                    _feetPlacement.RightFootIKPosition.x, _feetPlacement.RightFootIKPosition.z);
            }
        }

        // B. Provisional: We reset the time passed everytime when we lift the feet
        // Not very accurate, it would be better to create a time variable per feet and pass it though the method
        if ((!isLeftFootGrounded || !isRightFootGrounded) && (isMoving || isJumping))
        {
            timePassed = 0f;
        }

        // C. Provisional: When is still (once) - Stopping when reaching the deformation required was not giving very good results -> TODO: Improve!
        if ((!isMoving || !isJumping) && (!isLeftFootGrounded || !isRightFootGrounded) && provCounter <= 3)
        {
            timePassed = 0f;
            provCounter += 1;
        }

        // D. Provisional: Each time I change motion, resets the time
        isMoving = _anim.GetBool("isWalking");
        if (isMoving != oldIsMoving)
        {
            timePassed = 0f;
            oldIsMoving = isMoving;
            provCounter = 0;
        }

        isJumping = _anim.GetBool("isJumping"); // NEW
        if (isJumping != oldIsJumping)
        {
            timePassed = 0f;
            oldIsJumping = isJumping;
            provCounter = 0;
        }
    }

    private void DebugForceModel()
    {
        // A. Print the position of the feet in both systems (world and grid)
        if (printFeetPositions)
        {
            Debug.Log("[INFO] Left Foot Coords (World): " + _feetPlacement.LeftFootIKPosition.ToString());
            Debug.Log("[INFO] Left Foot Coords (Grid): " + centerGridLeftFoot.ToString());
            Debug.Log("[INFO] Right Foot Coords (World): " + _feetPlacement.RightFootIKPosition.ToString());
            Debug.Log("[INFO] Right Foot Coords (Grid): " + centerGridRightFoot.ToString());
            Debug.Log("-----------------------------------------");
        }

        // B. Print the forces
        if (printFeetForces)
        {
            Debug.Log("[INFO] Weight Force: " + weightForce);

            Debug.Log("[INFO] Left Foot Speed: " + feetSpeedLeft);
            Debug.Log("[INFO] Right Foot Speed: " + feetSpeedRight);

            Debug.Log("[INFO] Left Foot Impulse: " + feetImpulseLeft);
            Debug.Log("[INFO] Right Foot Impulse: " + feetImpulseRight);

            Debug.Log("[INFO] Left Foot Momentum: " + momentumForceLeft);
            Debug.Log("[INFO] Right Foot Momentum: " + momentumForceRight);

            Debug.Log("[INFO] GRF Left Foot: " + totalGRForceLeft);
            Debug.Log("[INFO] GRF Right Foot: " + totalGRForceRight);

            Debug.Log("[INFO] Total Force Left Foot: " + totalForceLeftFoot);
            Debug.Log("[INFO] Total Force Right Foot: " + totalForceRightFoot);
            Debug.Log("-----------------------------------------");
        }
    }

    private void CalculateForceModel()
    {
        // A. Bipedal -- _anim.pivotWeight only for bipeds
        if (isLeftFootGrounded && isRightFootGrounded)
        {
            weightInLeftFoot = (1 - _anim.pivotWeight);
            weightInRightFoot = (_anim.pivotWeight);
        }
        else
        {
            if (!isLeftFootGrounded)
            {
                weightInLeftFoot = 0f;
                weightInRightFoot = 1f;
            }
            else if (!isRightFootGrounded)
            {
                weightInLeftFoot = 1f;
                weightInRightFoot = 0f;
            }
        }
        
        //       Bipedal Force Model       //
        // =============================== //
        if (brushPhysicalFootprint)
        {
            if (!isRightFootGrounded && !isLeftFootGrounded) // NEW
            {
                weightInLeftFoot = 0f;
                weightInRightFoot = 0f;
            }

            // B. Weight Forces - Negative Y-component
            weightForce = mass * (Physics.gravity);
            weightForceLeft = weightForce * (weightInLeftFoot);
            weightForceRight = weightForce * (weightInRightFoot);

            //               Extra for plotting              //
            // ============================================= //
            weightForceLeftYFloat = weightForceLeft.y;
            weightForceRightYFloat = weightForceRight.y;
            weightForceYFloat = weightForce.y;
            // ============================================= //

            // Weight Force is already zero if the foot is not grounded - however, we draw only when foot is grounded
            if (drawWeightForces && (!isMoving || !isJumping))
            {
                DrawForce.ForDebug3D(centerGridLeftFootHeight, weightForceLeft, Color.blue, 0.0025f);
                DrawForce.ForDebug3D(centerGridRightFootHeight, weightForceRight, Color.blue, 0.0025f);
            }
            else
            {
                if (drawWeightForces && isLeftFootGrounded)
                {
                    DrawForce.ForDebug3D(centerGridLeftFootHeight, weightForceLeft, Color.blue, 0.0025f);
                }

                if (drawWeightForces && isRightFootGrounded)
                {
                    DrawForce.ForDebug3D(centerGridRightFootHeight, weightForceRight, Color.blue, 0.0025f);
                }
            }

            //--------------

            // Only velocities going downward //
            // ================================ //

            // B. Impulse per foot - Linear Momentum change (final velocity for the feet is 0)
            //feetImpulseLeft = mass * weightInLeftFoot * (Vector3.zero - feetSpeedLeft);
            //feetImpulseRight = mass * weightInRightFoot * (Vector3.zero - feetSpeedRight);

            // New Velocity version //
            //////////////////////////

            if (feetSpeedLeft.y <= 0f)
            {
                feetImpulseLeft = mass * weightInLeftFoot * (Vector3.zero - feetSpeedLeft);
            }
            else
            {
                feetImpulseLeft = Vector3.zero;
            }

            if (feetSpeedRight.y <= 0f)
                feetImpulseRight = mass * weightInRightFoot * (Vector3.zero - feetSpeedRight);
            else
                feetImpulseRight = Vector3.zero;

            // ================================== //

            //--------------

            // C. Momentum force exerted by ground to each foot - Calculated using Impulse and Contact Time
            // Positive (upward) if foot lands (negative velocity)
            // Negative (downward) if foot rises (positive velocity)
            momentumForceLeft = feetImpulseLeft / contactTime;
            momentumForceRight = feetImpulseRight / contactTime;
            momentumForce = momentumForceLeft + momentumForceRight;

            //  Extra for plotting (only positive values - when feet hit the ground) //
            // ===================================================================== //
            if (momentumForceLeft.y > 0f)
                momentumForceLeftYFloat = momentumForceLeft.y;

            if (momentumForceRight.y > 0f)
                momentumForceRightYFloat = momentumForceRight.y;

            if (momentumForce.y > 0f)
                momentumForceYFloat = momentumForce.y;
            // ===================================================================== //

            // Momentum Forces are created when we hit the ground (that is, when such forces are positive in y, and the feet are grounded)
            if (drawMomentumForces && (isMoving || isJumping) && isLeftFootGrounded)
            {
                DrawForce.ForDebug3D(centerGridLeftFootHeight, -momentumForceLeft, Color.red, 0.0025f);
            }

            if (drawMomentumForces && (isMoving || isJumping) && isRightFootGrounded)
            {
                DrawForce.ForDebug3D(centerGridRightFootHeight, -momentumForceRight, Color.red, 0.0025f);
            }

            //--------------

            // D. GRF (Ground Reaction Force) that the ground exerts to each foot
            totalGRForceLeft = momentumForceLeft - (weightForceLeft);
            totalGRForceRight = momentumForceRight - (weightForceRight);
            totalGRForce = totalGRForceLeft + totalGRForceRight;

            //               Extra for plotting              //
            // ============================================= //
            totalGRForceLeftYFloat = totalGRForceLeft.y;
            totalGRForceRightYFloat = totalGRForceRight.y;
            totalGRForceYFloat = totalGRForce.y;
            // ============================================= //

            // Color for GR Forces
            Color darkGreen = new Color(0.074f, 0.635f, 0.062f, 1f);

            // GRF is already zero if the foot is not grounded - however, we draw only when foot is grounded
            if (drawGRForces && (!isMoving || !isJumping))
            {
                DrawForce.ForDebug3D(centerGridLeftFootHeight, totalGRForceLeft, darkGreen, 0.0025f);
                DrawForce.ForDebug3D(centerGridRightFootHeight, totalGRForceRight, darkGreen, 0.0025f);
            }
            else
            {
                if (drawGRForces && isLeftFootGrounded)
                {
                    DrawForce.ForDebug3D(centerGridLeftFootHeight, totalGRForceLeft, darkGreen, 0.0025f);
                }

                if (drawGRForces && isRightFootGrounded)
                {
                    DrawForce.ForDebug3D(centerGridRightFootHeight, totalGRForceRight, darkGreen, 0.0025f);
                }
            }

            //--------------

            // E. Reaction Force for the feet (3rd Newton Law)
            totalForceLeftFoot = -totalGRForceLeft;
            totalForceRightFoot = -totalGRForceRight;
            totalForceFoot = totalForceLeftFoot + totalForceRightFoot;

            //--------------

            // Save max/min values reached for the feet forces in Z
            /*
            maxTotalForceLeftFootZOld = totalForceLeftFoot.z;
            if(maxTotalForceLeftFootZOld > maxTotalForceLeftFootZ)
            {
                maxTotalForceLeftFootZ = maxTotalForceLeftFootZOld;
                maxTotalForceLeftFootZNorm = totalForceLeftFoot.normalized.z;
            }
            minTotalForceLeftFootZOld = totalForceLeftFoot.z;
            if (minTotalForceLeftFootZOld < minTotalForceLeftFootZ)
            {
                minTotalForceLeftFootZ = minTotalForceLeftFootZOld;
                minTotalForceLeftFootZNorm = totalForceLeftFoot.normalized.z;
            }
            
            // Reset Values
            if (!isLeftFootGrounded)
            {
                maxTotalForceLeftFootZ = 0f;
                minTotalForceLeftFootZ = 0f;
                maxTotalForceLeftFootZNorm = 0f;
                minTotalForceLeftFootZNorm = 0f;
            }

            // Save max/min values reached for the feet forces in Z
            maxTotalForceRightFootZOld = totalForceRightFoot.z;
            if (maxTotalForceRightFootZOld > maxTotalForceRightFootZ)
            {
                maxTotalForceRightFootZ = maxTotalForceRightFootZOld;
                maxTotalForceRightFootZNorm = totalForceRightFoot.normalized.z;

            }
            minTotalForceRightFootZOld = totalForceRightFoot.z;
            if (minTotalForceRightFootZOld < minTotalForceRightFootZ)
            {
                minTotalForceRightFootZ = minTotalForceRightFootZOld;
                minTotalForceRightFootZNorm = totalForceRightFoot.normalized.z;
            }

            // Reset Values
            if (!isRightFootGrounded)
            {
                maxTotalForceRightFootZ = 0f;
                minTotalForceRightFootZ = 0f;
                maxTotalForceRightFootZNorm = 0f;
                minTotalForceRightFootZNorm = 0f;
            }
            */

            //--------------

            // Feet Forces are created when we hit the ground (that is, when the Y-component of the Momentum Force is positive)
            // Only when the feet rise up, Feet Forces do not exist. The muscle is the responsable to lift the foot up
            // Also, the foot must be grounded to have a Feet Force actuating onto the ground
            if (drawFeetForces && (!isMoving || !isJumping))
            {
                DrawForce.ForDebug3D(centerGridLeftFootHeight, totalForceLeftFoot, Color.black, 0.0025f);
                DrawForce.ForDebug3D(centerGridRightFootHeight, totalForceRightFoot, Color.black, 0.0025f);
            }
            else
            {
                // Only when Momentum Force is upward
                if (drawFeetForces && momentumForceLeft.y > 0f && isLeftFootGrounded)
                {
                    DrawForce.ForDebug3D(centerGridLeftFootHeight, totalForceLeftFoot, Color.black, 0.0025f);
                }

                if (drawFeetForces && momentumForceRight.y > 0f && isRightFootGrounded)
                {
                    DrawForce.ForDebug3D(centerGridRightFootHeight, totalForceRightFoot, Color.black, 0.0025f);
                }
            }
        }
    }

    private void UpdateFeetPositions()
    {
        // A. Save height of feet IK
        heightIKLeft = _feetPlacement.LeftFootIKPosition.y;
        heightIKRight = _feetPlacement.RightFootIKPosition.y;

        // B. Flags to detect when the foot is on the ground
        isLeftFootGrounded = _feetPlacement.isLeftFootGrounded;
        isRightFootGrounded = _feetPlacement.isRightFootGrounded;

        // C. Grid-based feet positions and heights
        centerGridLeftFoot = World2Grid(_feetPlacement.LeftFootIKPosition.x, _feetPlacement.LeftFootIKPosition.z);
        centerGridRightFoot = World2Grid(_feetPlacement.RightFootIKPosition.x, _feetPlacement.RightFootIKPosition.z);
        centerGridLeftFootHeight = new Vector3(_feetPlacement.LeftFootIKPosition.x, Get(centerGridLeftFoot.x, centerGridLeftFoot.z), _feetPlacement.LeftFootIKPosition.z);
        centerGridRightFootHeight = new Vector3(_feetPlacement.RightFootIKPosition.x, Get(centerGridRightFoot.x, centerGridRightFoot.z), _feetPlacement.RightFootIKPosition.z);
    }

    private void UpdateTerrain()
    {
        // A. Every time we change to other terrainData, we update
        if (terrain.name != myBipedalCharacter.GetComponent<RigidBodyControllerSimpleAnimator>().currentTerrain.name)
        {
            // Extract terrain information
            terrain = myBipedalCharacter.GetComponent<RigidBodyControllerSimpleAnimator>().currentTerrain;
            Debug.Log("[INFO] Updating to new terrain: " + terrain.name);

            terrain_collider = terrain.GetComponent<Collider>();
            terrain_data = terrain.terrainData;
            terrain_size = terrain_data.size;
            heightmap_width = terrain_data.heightmapResolution;
            heightmap_height = terrain_data.heightmapResolution;
            heightmap_data = terrain_data.GetHeights(0, 0, heightmap_width, heightmap_height);
            heightmap_data_constant = terrain_data.GetHeights(0, 0, heightmap_width, heightmap_height);
            heightmap_data_filtered = terrain_data.GetHeights(0, 0, heightmap_width, heightmap_height);
        }
    }

    private void DefineSourceDeformation()
    {
        // A. Select type of deformation (UI, Prefabs or Manual)
        switch (deformationChoice)
        {
            case sourceDeformation.useUI:
                if (brushPhysicalFootprint)
                {
                    // Time is managed from the master script - others from the footprint script
                    contactTime = timeSlider.value;

                    // Force Model UI
                    if(!(activateToggleShowForceModel is null))
                    {
                        drawWeightForces = activateToggleShowForceModel.isOn;
                        drawMomentumForces = activateToggleShowForceModel.isOn;
                        drawGRForces = activateToggleShowForceModel.isOn;
                        drawFeetForces = activateToggleShowForceModel.isOn;
                    }

                }
                break;
            case sourceDeformation.useManualValues:
                break;
            case sourceDeformation.useTerrainPrefabs:

                if (brushPhysicalFootprint)
                {
                    if (terrain.CompareTag("Snow"))
                        DefineSnow();
                    else if (terrain.CompareTag("Dry Sand"))
                        DefineDrySand();
                    else if (terrain.CompareTag("Mud"))
                        DefineMud();
                    else if (terrain.CompareTag("Soil"))
                        DefineSoil();
                    else
                        DefineDefault();
                }
                break;
        }
    }

    private void CalculateFeetVelocity()
    {
        // Left to compare with the new velocity
        newIKLeftPosition = _anim.GetBoneTransform(HumanBodyBones.LeftFoot).position; // Before: LeftFoot
        newIKRightPosition = _anim.GetBoneTransform(HumanBodyBones.RightFoot).position;
        var mediaLeft = (newIKLeftPosition - oldIKLeftPosition);
        var mediaRight = (newIKRightPosition - oldIKRightPosition);

        feetSpeedLeft = new Vector3((mediaLeft.x / Time.fixedDeltaTime), (mediaLeft.y / Time.fixedDeltaTime), (mediaLeft.z / Time.fixedDeltaTime));
        feetSpeedRight = new Vector3((mediaRight.x / Time.fixedDeltaTime), (mediaRight.y / Time.fixedDeltaTime), (mediaRight.z / Time.fixedDeltaTime));

        oldIKLeftPosition = newIKLeftPosition;
        oldIKRightPosition = newIKRightPosition;

        newIKLeftPosition = _anim.GetBoneTransform(HumanBodyBones.LeftFoot).position;
        newIKRightPosition = _anim.GetBoneTransform(HumanBodyBones.RightFoot).position;

        if (drawVelocities)
        {
            DrawForce.ForDebug3D(newIKLeftPosition, -feetSpeedLeft, Color.cyan, 0.0025f);
            DrawForce.ForDebug3D(newIKRightPosition, -feetSpeedRight, Color.cyan, 0.0025f);
        }

        // Calculate New Velocity for the feet //
        // =================================== //

        feetSpeedLeft = leftFootRB.velocity;
        feetSpeedRight = rightFootRB.velocity;

        if (drawNewVelocities)
        {
            DrawForce.ForDebug3DVelocity(oldIKLeftPosition, feetSpeedLeft, Color.cyan, 1f);
            DrawForce.ForDebug3DVelocity(oldIKRightPosition, feetSpeedRight, Color.cyan, 1f);

        }
    }

    #endregion

    #region Prefabs

    // Methods use to define new materials
    public void DefineSnow()
    {
        brushPhysicalFootprint.YoungM = youngModulusSnow;
        contactTime = timeSnow;
        brushPhysicalFootprint.FilterIte = filterIterationsSnow;
        brushPhysicalFootprint.PoissonRatio = poissonRatioSnow;
        brushPhysicalFootprint.ActivateBump = bumpSnow;

    }

    public void DefineDrySand()
    {
        brushPhysicalFootprint.YoungM = youngModulusDrySand;
        contactTime = timeDrySand;
        brushPhysicalFootprint.FilterIte = filterIterationsSand;
        brushPhysicalFootprint.PoissonRatio = poissonRatioSand;
        brushPhysicalFootprint.ActivateBump = bumpSand;

    }

    public void DefineMud()
    {
        brushPhysicalFootprint.YoungM = youngModulusMud;
        contactTime = timeMud;
        brushPhysicalFootprint.FilterIte = filterIterationsMud;
        brushPhysicalFootprint.PoissonRatio = poissonRatioMud;
        brushPhysicalFootprint.ActivateBump = bumpMud;
    }

    public void DefineSoil()
    {
        brushPhysicalFootprint.YoungM = youngModulusSoil;
        contactTime = timeSoil;
        brushPhysicalFootprint.FilterIte = filterIterationsSoil;
        brushPhysicalFootprint.PoissonRatio = poissonRatioSoil;
        brushPhysicalFootprint.ActivateBump = bumpSoil;
    }

    public void DefineDefault()
    {
        brushPhysicalFootprint.YoungM = 750000;
        brushPhysicalFootprint.FilterIte = 0;
        brushPhysicalFootprint.PoissonRatio = 0f;
        brushPhysicalFootprint.ActivateBump = false;
    }


    // ========================= //
    // Define here your material // 

    //public void DefineExample()
    //{
    //    brushPhysicalFootprint.YoungM = youngModulusExample;
    //    brushPhysicalFootprint.FilterIte = timeExamlpe;
    //    brushPhysicalFootprint.PoissonRatio = filterIterationsExample;
    //    brushPhysicalFootprint.ActivateBump = bumpExample;
    //}

    // ========================= //

    #endregion

    #region Getters

    public Vector3 Get3(int x, int z)
    {
        return new Vector3(x, Get(x, z), z);
    }
    public Vector3 Get3(float x, float z)
    {
        return new Vector3(x, Get(x, z), z);
    }
    public Vector3 GetInterp3(float x, float z)
    {
        return new Vector3(x, GetInterp(x, z), z);
    }

    // Given one node of the heightmap, get the height
    public float Get(int x, int z)
    {
        x = (x + heightmap_width) % heightmap_width;
        z = (z + heightmap_height) % heightmap_height;
        return heightmap_data[z, x] * terrain_data.heightmapScale.y;
    }
    public float Get(float x, float z)
    {
        return Get((int)x, (int)z);
    }

    // Get entire array with heightmap without being scaled
    public float[,] GetHeightmap()
    {
        // IMPORTANT: When getting a value, must be multiplied by terrain_data.heightmapScale.y!
        return heightmap_data;
    }

    // Given one node of the heightmap (constant at start), get the height
    public float GetConstant(int x, int z)
    {
        x = (x + heightmap_width) % heightmap_width;
        z = (z + heightmap_height) % heightmap_height;
        return heightmap_data_constant[z, x] * terrain_data.heightmapScale.y;
    }
    public float GetConstant(float x, float z)
    {
        return GetConstant((int)x, (int)z);
    }

    // Get entire array with initial constant heightmap without being scaled
    public float[,] GetConstantHeightmap()
    {
        // IMPORTANT: When getting a value, must be multiplied by terrain_data.heightmapScale.y!
        return heightmap_data_constant;
    }
    // Given one node of the heightmap, get the height (post-filter version)
    public float GetFiltered(int x, int z)
    {
        x = (x + heightmap_width) % heightmap_width;
        z = (z + heightmap_height) % heightmap_height;
        return heightmap_data_filtered[z, x] * terrain_data.heightmapScale.y;
    }
    public float GetFiltered(float x, float z)
    {
        return GetFiltered((int)x, (int)z);
    }

    // Get entire array with post-filtered heightmap
    public float[,] GetFilteredHeightmap()
    {
        // IMPORTANT: When getting a value, must be multiplied by terrain_data.heightmapScale.y!
        return heightmap_data_filtered;
    }

    public float GetInterp(float x, float z)
    {
        return terrain_data.GetInterpolatedHeight(x / heightmap_width,
                                                  z / heightmap_height);
    }
    public float GetSteepness(float x, float z)
    {
        return terrain_data.GetSteepness(x / heightmap_width,
                                         z / heightmap_height);
    }
    public Vector3 GetNormal(float x, float z)
    {
        return terrain_data.GetInterpolatedNormal(x / heightmap_width,
                                                  z / heightmap_height);
    }

    #endregion

    #region Setters

    // Given one node of the heightmap, set the height
    public void Set(int x, int z, float val)
    {
        x = (x + heightmap_width) % heightmap_width;
        z = (z + heightmap_height) % heightmap_height;
        heightmap_data[z, x] = val / terrain_data.heightmapScale.y;
    }

    public void Set(float x, float z, float val)
    {
        Set((int)x, (int)z, val);
    }

    #endregion

    #region Terrain Methods

    // Get dimensions of the heightmap grid
    public Vector3 GridSize()
    {
        return new Vector3(heightmap_width, 0.0f, heightmap_height);
    }

    // Get real dimensions of the terrain (World Space)
    public Vector3 TerrainSize()
    {
        return terrain_size;
    }

    // Get terrain data
    public TerrainData GetTerrainData()
    {
        return terrain_data;
    }

    // Convert from Grid Space to World Space
    public Vector3 Grid2World(Vector3 grid)
    {
        return new Vector3(grid.x * terrain_data.heightmapScale.x,
                           grid.y,
                           grid.z * terrain_data.heightmapScale.z);
    }

    public Vector3 Grid2World(float x, float y, float z)
    {
        return Grid2World(new Vector3(x, y, z));
    }

    public Vector3 Grid2World(float x, float z)
    {
        return Grid2World(x, 0.0f, z);
    }

    // Convert from World Space to Grid Space
    public Vector3 World2Grid(Vector3 grid)
    {
        return new Vector3(grid.x / terrain_data.heightmapScale.x,
                           grid.y,
                           grid.z / terrain_data.heightmapScale.z);
    }

    public Vector3 World2Grid(float x, float y, float z)
    {
        return World2Grid(new Vector3(x, y, z));
    }

    public Vector3 World2Grid(float x, float z)
    {
        return World2Grid(x, 0.0f, z);
    }

    // Reset to flat terrain
    public void Reset()
    {
        for (int z = 0; z < heightmap_height; z++)
        {
            for (int x = 0; x < heightmap_width; x++)
            {
                heightmap_data[z, x] = 0;
            }
        }

        Save();
    }

    // Smooth terrain
    public void AverageSmooth()
    {
        for (int z = 10; z < heightmap_height - 10; z++)
        {
            for (int x = 10; x < heightmap_width - 10; x++)
            {
                float n = 2.0f * 2 + 1.0f;
                float sum = 0;
                for (int szi = -2; szi <= 2; szi++)
                {
                    for (int sxi = -2; sxi <= 2; sxi++)
                    {
                        sum += heightmap_data[z + szi, x + sxi];
                    }
                }

                heightmap_data[z, x] = sum / (n * n);
            }
        }

        Save();
    }

    // Calculate Kernel
    public static float[,] CalculateKernel(int length, float sigma)
    {
        float[,] Kernel = new float[length, length];
        float sumTotal = 0f;

        int kernelRadius = length / 2;
        double distance = 0f;

        float calculatedEuler = 1.0f / (2.0f * (float)Math.PI * sigma * sigma);

        for (int idY = -kernelRadius; idY <= kernelRadius; idY++)
        {
            for (int idX = -kernelRadius; idX <= kernelRadius; idX++)
            {
                distance = ((idX * idX) + (idY * idY)) / (2 * (sigma * sigma));

                Kernel[idY + kernelRadius, idX + kernelRadius] = calculatedEuler * (float)Math.Exp(-distance);

                sumTotal += Kernel[idY + kernelRadius, idX + kernelRadius];
            }
        }

        for (int y = 0; y < length; y++)
        {
            for (int x = 0; x < length; x++)
            {
                Kernel[y, x] = Kernel[y, x] * (1.0f / sumTotal);
            }
        }

        return Kernel;
    }

    // Gaussian Filter (Custom Kernel)
    public void GaussianBlurCustom()
    {
        float[,] kernel = CalculateKernel(3, 1f);

        for (int z = 10; z < heightmap_height - 10; z++)
        {
            for (int x = 10; x < heightmap_width - 10; x++)
            {

                heightmap_data[z, x] =
                    kernel[0, 0] * heightmap_data[z - 1, x - 1]
                    + kernel[0, 1] * heightmap_data[z - 1, x]
                    + kernel[0, 2] * heightmap_data[z - 1, x + 1]
                    + kernel[1, 0] * heightmap_data[z, x - 1]
                    + kernel[1, 1] * heightmap_data[z, x]
                    + kernel[1, 2] * heightmap_data[z, x + 1]
                    + kernel[2, 0] * heightmap_data[z + 1, x - 1]
                    + kernel[2, 1] * heightmap_data[z + 1, x]
                    + kernel[2, 2] * heightmap_data[z + 1, x + 1];
            }
        }

        Save();
    }

    // Gaussian Blur 3x3
    public void GaussianBlur3()
    {
        for (int z = 10; z < heightmap_height - 10; z++)
        {
            for (int x = 10; x < heightmap_width - 10; x++)
            {

                heightmap_data[z, x] =
                    heightmap_data[z - 1, x - 1]
                    + 2 * heightmap_data[z - 1, x]
                    + 1 * heightmap_data[z - 1, x + 1]
                    + 2 * heightmap_data[z, x - 1]
                    + 4 * heightmap_data[z, x]
                    + 2 * heightmap_data[z, x + 1]
                    + 1 * heightmap_data[z + 1, x - 1]
                    + 2 * heightmap_data[z + 1, x]
                    + 1 * heightmap_data[z + 1, x + 1];

                heightmap_data[z, x] *= 1.0f / 16.0f;

            }
        }

        Save();

    }

    // Gaussian Blur 5x5
    public void GaussianBlur5()
    {
        for (int z = 10; z < heightmap_height - 10; z++)
        {
            for (int x = 10; x < heightmap_width - 10; x++)
            {

                heightmap_data[z, x] =
                    heightmap_data[z - 2, x - 2]
                    + 4 * heightmap_data[z - 2, x - 1]
                    + 6 * heightmap_data[z - 2, x]
                    + heightmap_data[z - 2, x + 2]
                    + 4 * heightmap_data[z - 2, x + 1]
                    + 4 * heightmap_data[z - 1, x + 2]
                    + 16 * heightmap_data[z - 1, x + 1]
                    + 4 * heightmap_data[z - 1, x - 2]
                    + 16 * heightmap_data[z - 1, x - 1]
                    + 24 * heightmap_data[z - 1, x]
                    + 6 * heightmap_data[z, x - 2]
                    + 24 * heightmap_data[z, x - 1]
                    + 6 * heightmap_data[z, x + 2]
                    + 24 * heightmap_data[z, x + 1]
                    + 36 * heightmap_data[z, x]
                    + heightmap_data[z + 2, x - 2]
                    + 4 * heightmap_data[z + 2, x - 1]
                    + 6 * heightmap_data[z + 2, x]
                    + heightmap_data[z + 2, x + 2]
                    + 4 * heightmap_data[z + 2, x + 1]
                    + 4 * heightmap_data[z + 1, x + 2]
                    + 16 * heightmap_data[z + 1, x + 1]
                    + 4 * heightmap_data[z + 1, x - 2]
                    + 16 * heightmap_data[z + 1, x - 1]
                    + 24 * heightmap_data[z + 1, x];

                heightmap_data[z, x] *= 1.0f / 256.0f;

            }
        }

        Save();

    }

    // Register changes made to the terrain
    public void Save()
    {
        terrain_data.SetHeights(0, 0, heightmap_data);
    }

    // Get and set active brushes
    public void SetFootprintBrush(BrushPhysicalFootprint brush)
    {
        Debug.Log("[INFO] Setting brush to " + brush);
        brushPhysicalFootprint = brush;
    }
    public BrushPhysicalFootprint GetFootprintBrush()
    {
        return brushPhysicalFootprint;
    }

    #endregion
}
