﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/// <summary>
/// Brush to create dynamic footprints on the heighmap terrain. 
/// First, it analyzes the cells to be affected based on the ray-cast and feet colliders.
/// Then, it calculates the displacement per cell based on contact area and character weight / force applied.
/// </summary>
public class PhysicalFootprint : TerrainBrushPhysicalFootprint
{
    #region Variables

    [Header("Physically-based Footprints Deformation")]
    public bool applyFootprints = false;
    public bool applyBumps = false;

    [Header("Deformation - Debug")]
    public bool showGridDebugLeft = false;
    public bool showGridDebugRight = false;
    public bool showGridBumpDebug = false;
    public bool showGridBumpFrontBack = false;
    public bool printTerrainInformation = false;
    public bool printDeformationInformation = false;

    [Header("Deformation - Grid Settings")]
    [Range(0, 20)] public int gridSize = 10;
    [Range(0f, 1f)] public float rayDistance = 0.1f;
    [Range(0f, 1f)] public float offsetRay = 0.04f;

    [Header("Deformation - Number of hits")]
    public int counterHitsLeft;
    public int counterHitsRight;

    [Header("Deformation - Contact Area Feet-Ground")]
    public float areaCell;
    public float areaTotal = 0f;
    public float areaTotalLeft = 0f;
    public float areaTotalRight = 0f;
    private float oldAreaTotalLeft = 0f;
    private float oldAreaTotalRight = 0f;
    private float lenghtCellX;
    private float lenghtCellZ;

    [Header("Deformation - Volume Rod Approximation")]
    public double volumeTotalLeft = 0f; // TEST
    public double volumeTotalRight = 0f; // TEST
    public double volumeOriginalLeft = 0f; // TEST
    public double volumeOriginalRight = 0f; // TEST
    public double volumeTotalBumpLeft = 0f; // TEST
    public double volumeTotalBumpRight = 0f; // TEST
    public double volumeOriginalBumpLeft = 0f; // TEST
    public double volumeOriginalBumpRight = 0f; // TEST
    public double volumeVariationLeft; // TEST
    public double volumeVariationRight; // TEST

    [Header("Deformation - Pressure (Stress) by feet")]
    public float pressureStress;
    public float pressureStressLeft;
    public float pressureStressRight;

    [Header("Terrain Deformation - Settings")]
    //public bool usePredefinedGround = false;
    [Range(100000, 1000000)] public double youngModulus = 1000000;
    public float originaLength = 1f;
    [Range(0, 0.05f)] public double bumpHeightDeformation = 0.03f; // TEST - It should be calculated directly from the young Modulus system

    [Header("Terrain Deformation - Info")]
    public double heightCellDisplacementYoungLeft = 0f;
    public double heightCellDisplacementYoungRight = 0f;
    public float displacementLeft;
    public float displacementRight;
    private double oldHeightCellDisplacementYoungLeft = 0f;
    private double oldHeightCellDisplacementYoungRight = 0f;

    [Header("Bump Deformation - Settings")]
    public int offsetBumpGrid = 2;
    public int neighbourSearchArea = 1;

    [Header("Bump Deformation - Settings (EXPERIMENTAL)")]
    [Range(0, 0.5f)] public float poissonRatio = 0.4f; // TEST
    public double newBumpHeightDeformationLeft = 0f; // TEST
    public double newBumpHeightDeformationRight = 0f; // TEST
    public double strainLong; // TEST
    public double strainTrans; // TEST

    [Header("Bump Deformation - Info")]
    public int neighbourCellsLeft;
    public int neighbourCellsRight;
    public float neighbourAreaTotalLeft;
    public float neighbourAreaTotalRight;
    private float bumpDisplacementLeftBack; // TEST
    private float bumpDisplacementRightBack; // TEST
    private float bumpDisplacementLeftFront; // TEST
    private float bumpDisplacementRightFront; // TEST
    private float oldNeighbourAreaTotalLeft;
    private float oldNeighbourAreaTotalRight;

    [Header("Terrain Deformation - Bump Vector3 Coordinates")]
    public List<Vector3> neighboursPositionsRightFront = new List<Vector3>(); // TEST
    public List<Vector3> neighboursPositionsLeftFront = new List<Vector3>(); // TEST
    public List<Vector3> neighboursPositionsRightBack = new List<Vector3>(); // TEST
    public List<Vector3> neighboursPositionsLeftBack = new List<Vector3>(); // TEST

    [Header("Filter")]
    public bool applyFilterLeft = false;
    public bool applyFilterRight = false;
    public int filterIterationsLeftFoot = 2;
    public int filterIterationsRightFoot = 2;
    public int marginAroundGrid = 3;
    private int filterIterationsLeftCounter = 0;
    private int filterIterationsRightCounter = 0;
    private bool isFilteredLeft = false;
    private bool isFilteredRight = false;
    private bool applyFilterLeft2 = false;
    private bool applyFilterRight2 = false;

    // Others for filtering
    [Range(0, 5)] private int gridSizeKernel = 1;
    //public bool applyPostFilter = false;
    //public bool applyPreFilterLeft = false;
    //public bool applyPreFilterRight = false;

    // Others - Not used
    private float[,] heightMapLeft;
    private float[,] heightMapRight;
    private float[,] heightMapLeftFiltered;
    private float[,] heightMapRightFiltered;
    private int[,] heightMapLeftBool;
    private int[,] heightMapRightBool;

    #endregion

    /// <summary>
    /// Method that takes the IK positions for each feet and apply displacement to ground.
    /// </summary>
    /// <param name="xLeft"></param>
    /// <param name="zLeft"></param>
    /// <param name="xRight"></param>
    /// <param name="zRight"></param>
    public override void DrawFootprint(int xLeft, int zLeft, int xRight, int zRight)
    {

        //       Initial Declarations      //
        // =============================== //

        //Test
        if (UseTerrainPrefabs)
        {
            youngModulus = YoungM;
        }

        // Reset counter hits
        counterHitsLeft = 0;
        counterHitsRight = 0;
        neighbourCellsLeft = 0;
        neighbourCellsRight = 0;

        // TEST
        neighboursPositionsRightFront.Clear();
        neighboursPositionsLeftFront.Clear();
        neighboursPositionsRightBack.Clear();
        neighboursPositionsLeftBack.Clear();

        // Heightmaps for each foot
        float[,] heightMapLeft = new float[2 * gridSize + 1, 2 * gridSize + 1];
        float[,] heightMapRight = new float[2 * gridSize + 1, 2 * gridSize + 1];
        int[,] heightMapLeftBool = new int[2 * gridSize + 1, 2 * gridSize + 1];
        int[,] heightMapRightBool = new int[2 * gridSize + 1, 2 * gridSize + 1];

        // Warning: Supossing that terrain is squared!
        if (printTerrainInformation)
        {
            Debug.Log("[INFO] Length Terrain - X: " + terrain.TerrainSize().x);
            Debug.Log("[INFO] Length Terrain - Z: " + terrain.TerrainSize().z);
            Debug.Log("[INFO] Number of heightmap cells: " + (terrain.GridSize().x - 1));
            Debug.Log("[INFO] Lenght of one cell - X: " + (terrain.TerrainSize().x / (terrain.GridSize().x - 1)));
            Debug.Log("[INFO] Lenght of one cell - Z: " + (terrain.TerrainSize().z / (terrain.GridSize().z - 1)));
            Debug.Log("[INFO] Area of one cell: " + (terrain.TerrainSize().x / (terrain.GridSize().x - 1)) * (terrain.TerrainSize().z / (terrain.GridSize().z - 1)));
        }

        // Calculate area per cell outside the loop
        lenghtCellX = terrain.TerrainSize().x / (terrain.GridSize().x - 1);
        lenghtCellZ = terrain.TerrainSize().z / (terrain.GridSize().z - 1);
        areaCell = lenghtCellX * lenghtCellZ;

        //    Contact Area Calculation     //
        // =============================== //

        // 2D iteration for both feet
        // It counts the number of hits, save the classified cell in a list and debug ray-casting
        for (int zi = -gridSize; zi <= gridSize; zi++)
        {
            for (int xi = -gridSize; xi <= gridSize; xi++)
            {
                // Calculate each cell position wrt World and Heightmap - Left Foot
                // The sensors that counts the number of hits always remain on the surface
                Vector3 rayGridLeft = new Vector3(xLeft + xi, terrain.Get(xLeft + xi, zLeft + zi) - offsetRay, zLeft + zi);
                Vector3 rayGridWorldLeft = terrain.Grid2World(rayGridLeft);

                // Calculate each cell position wrt World and Heightmap - Right Foot
                // The sensors that counts the number of hits always remain on the surface
                Vector3 rayGridRight = new Vector3(xRight + xi, terrain.Get(xRight + xi, zRight + zi) - offsetRay, zRight + zi);
                Vector3 rayGridWorldRight = terrain.Grid2World(rayGridRight);

                //------//

                // Create each ray for the grid (wrt World) - Left
                RaycastHit leftFootHit;
                Ray upRayLeftFoot = new Ray(rayGridWorldLeft, Vector3.up);

                // Create each ray for the grid (wrt World) - Right
                RaycastHit rightFootHit;
                Ray upRayRightFoot = new Ray(rayGridWorldRight, Vector3.up);

                //------//

                // If hits the Left Foot, increase counter and add cell to be affected
                if (LeftFootCollider.Raycast(upRayLeftFoot, out leftFootHit, rayDistance))
                {
                    // Cell contacting directly
                    heightMapLeftBool[zi + gridSize, xi + gridSize] = 2;
                    counterHitsLeft++;

                    if (showGridDebugLeft)
                        Debug.DrawRay(rayGridWorldLeft, Vector3.up * rayDistance, Color.blue);
                }
                else
                {
                    // No contact
                    heightMapLeftBool[zi + gridSize, xi + gridSize] = 0;

                    if (showGridDebugLeft)
                        Debug.DrawRay(rayGridWorldLeft, Vector3.up * rayDistance, Color.red);
                }

                // If hits the Right Foot, increase counter and add cell to be affected
                if (RightFootCollider.Raycast(upRayRightFoot, out rightFootHit, rayDistance))
                {
                    // Cell contacting directly
                    heightMapRightBool[zi + gridSize, xi + gridSize] = 2;
                    counterHitsRight++;

                    if (showGridDebugRight)
                        Debug.DrawRay(rayGridWorldRight, Vector3.up * rayDistance, Color.blue);
                }
                else
                {
                    // No contact
                    heightMapRightBool[zi + gridSize, xi + gridSize] = 0;

                    if (showGridDebugRight)
                        Debug.DrawRay(rayGridWorldRight, Vector3.up * rayDistance, Color.red);
                }
            }
        }

        // Terrain Deformation is affected by an increasing value of the contact area, therefore the deformation
        // will be defined by the maximum contact area in each frame
        oldAreaTotalLeft = ((counterHitsLeft) * areaCell);
        if (oldAreaTotalLeft >= areaTotalLeft)
        {
            areaTotalLeft = ((counterHitsLeft) * areaCell);
            volumeTotalLeft = areaTotalLeft * (originaLength - heightCellDisplacementYoungLeft); // TEST
            volumeOriginalLeft = areaTotalLeft * (originaLength); // TEST
        }

        oldAreaTotalRight = ((counterHitsRight) * areaCell);
        if (oldAreaTotalRight >= areaTotalRight)
        {
            areaTotalRight = ((counterHitsRight) * areaCell);
            volumeTotalRight = areaTotalRight * (originaLength - heightCellDisplacementYoungRight); // TEST
            volumeOriginalRight = areaTotalRight * (originaLength); // TEST
        }

        // Total Area and Volume for both feet
        areaTotal = areaTotalLeft + areaTotalRight;

        //        Detecting Contour        //
        // =============================== //

        if (IsRightFootGrounded)
        {
            // We don't need to check the whole grid - just in the 5x5 inner grid is enough
            for (int zi = -gridSize + offsetBumpGrid; zi <= gridSize - offsetBumpGrid; zi++)
            {
                for (int xi = -gridSize + offsetBumpGrid; xi <= gridSize - offsetBumpGrid; xi++)
                {
                    // If the cell was not in contact, it's a potential neighbour (countour) cell
                    if (heightMapRightBool[zi + gridSize, xi + gridSize] == 0)
                    {
                        // Only checking adjacent cells - increasing this would allow increasing the area of the bump
                        for (int zi_sub = -neighbourSearchArea; zi_sub <= neighbourSearchArea; zi_sub++)
                        {
                            for (int xi_sub = -neighbourSearchArea; xi_sub <= neighbourSearchArea; xi_sub++)
                            {
                                if (heightMapRightBool[zi + zi_sub + gridSize, xi + xi_sub + gridSize] == 2)
                                {
                                    Vector3 rayGridRight = new Vector3(xRight + xi, terrain.Get(xRight + xi, zRight + zi) - offsetRay, zRight + zi);
                                    Vector3 rayGridWorldRight = terrain.Grid2World(rayGridRight);

                                    if (showGridBumpDebug)
                                        Debug.DrawRay(rayGridWorldRight, Vector3.up * 0.2f, Color.yellow);

                                    // Mark that cell as a countour point
                                    heightMapRightBool[zi + gridSize, xi + gridSize] = 1;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (IsLeftFootGrounded)
        {
            // We don't need to check the whole grid - just in the 5x5 inner grid is enough
            for (int zi = -gridSize + offsetBumpGrid; zi <= gridSize - offsetBumpGrid; zi++)
            {
                for (int xi = -gridSize + offsetBumpGrid; xi <= gridSize - offsetBumpGrid; xi++)
                {
                    // If the cell was not in contact, it's a potential neighbour (countour) cell
                    if (heightMapLeftBool[zi + gridSize, xi + gridSize] == 0)
                    {
                        // Only checking adjacent cells - increasing this would allow increasing the area of the bump
                        for (int zi_sub = -neighbourSearchArea; zi_sub <= neighbourSearchArea; zi_sub++)
                        {
                            for (int xi_sub = -neighbourSearchArea; xi_sub <= neighbourSearchArea; xi_sub++)
                            {
                                if (heightMapLeftBool[zi + zi_sub + gridSize, xi + xi_sub + gridSize] == 2)
                                {
                                    Vector3 rayGridLeft = new Vector3(xLeft + xi, terrain.Get(xLeft + xi, zLeft + zi) - offsetRay, zLeft + zi);
                                    Vector3 rayGridWorldLeft = terrain.Grid2World(rayGridLeft);

                                    if(showGridBumpDebug)
                                        Debug.DrawRay(rayGridWorldLeft, Vector3.up * 0.2f, Color.yellow);

                                    // Mark that cell as a countour point
                                    heightMapLeftBool[zi + gridSize, xi + gridSize] = 1;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        // Calculating number of neightbouring hits to later get the area
        for (int zi = -gridSize + offsetBumpGrid; zi <= gridSize - offsetBumpGrid; zi++)
        {
            for (int xi = -gridSize + offsetBumpGrid; xi <= gridSize - offsetBumpGrid; xi++)
            {
                if (heightMapLeftBool[zi + gridSize, xi + gridSize] == 1)
                {
                    // Each neightbour cell in world space
                    Vector3 rayGridLeft = new Vector3(xLeft + xi, terrain.Get(xLeft + xi, zLeft + zi) - offsetRay, zLeft + zi);
                    Vector3 rayGridWorldLeft = terrain.Grid2World(rayGridLeft);

                    // Position of the neighbour relative to the foot
                    Vector3 relativePos = rayGridWorldLeft - LeftFootCollider.transform.position;

                    // Check if is in front/back of the foot
                    if(Vector3.Dot(LeftFootCollider.transform.forward, relativePos) > 0.0f)
                    {
                        // Store the Vector3 positions in a dynamic array
                        neighboursPositionsLeftFront.Add(rayGridWorldLeft);

                        // TEST - 3 is contour in FRONT
                        heightMapLeftBool[zi + gridSize, xi + gridSize] = 3;

                        if (showGridBumpFrontBack)
                            Debug.DrawRay(LeftFootCollider.transform.position, relativePos, Color.red);
                    }
                    else
                    {
                        // Store the Vector3 positions in a dynamic array
                        neighboursPositionsLeftBack.Add(rayGridWorldLeft);

                        // TEST - 4 is contour in BACK
                        heightMapLeftBool[zi + gridSize, xi + gridSize] = 4;

                        if (showGridBumpFrontBack)
                            Debug.DrawRay(LeftFootCollider.transform.position, relativePos, Color.blue);
                    }

                    neighbourCellsLeft++;
                }

                if (heightMapRightBool[zi + gridSize, xi + gridSize] == 1)
                {
                    // Each neightbour cell in world space
                    Vector3 rayGridRight = new Vector3(xRight + xi, terrain.Get(xRight + xi, zRight + zi) - offsetRay, zRight + zi);
                    Vector3 rayGridWorldRight = terrain.Grid2World(rayGridRight);

                    // Position of the neighbour relative to the foot
                    Vector3 relativePos = rayGridWorldRight - RightFootCollider.transform.position;

                    // Check if is in front/back of the foot
                    if (Vector3.Dot(RightFootCollider.transform.forward, relativePos) > 0.0f)
                    {
                        // Store the Vector3 positions in a dynamic array
                        neighboursPositionsRightFront.Add(rayGridWorldRight);

                        // TEST - 3 is contour in FRONT
                        heightMapRightBool[zi + gridSize, xi + gridSize] = 3;

                        if (showGridBumpFrontBack)
                            Debug.DrawRay(RightFootCollider.transform.position, relativePos, Color.red);
                    }
                    else
                    {
                        // Store the Vector3 positions in a dynamic array
                        neighboursPositionsRightBack.Add(rayGridWorldRight);

                        // TEST - 4 is contour in BACK
                        heightMapRightBool[zi + gridSize, xi + gridSize] = 4;

                        if (showGridBumpFrontBack)
                            Debug.DrawRay(RightFootCollider.transform.position, relativePos, Color.blue);
                    }

                    neighbourCellsRight++;
                }
            }
        }

        //

        // Calculate the neightbour area for each foot
        oldNeighbourAreaTotalLeft = ((neighbourCellsLeft) * areaCell);
        if (oldNeighbourAreaTotalLeft >= neighbourAreaTotalLeft)
        {
            neighbourAreaTotalLeft = ((neighbourCellsLeft) * areaCell);

            // Here calculate the necessary displacement! - TODO
            //volumeTotalBumpLeft = neighbourAreaTotalLeft * (originaLength - heightCellDisplacementYoungLeft); // TEST
            volumeOriginalBumpLeft = neighbourAreaTotalLeft * (originaLength); // TEST - TODO
            ///////
        }

        oldNeighbourAreaTotalRight = ((neighbourCellsRight) * areaCell);
        if (oldNeighbourAreaTotalRight >= neighbourAreaTotalRight)
        {
            neighbourAreaTotalRight = ((neighbourCellsRight) * areaCell);

            // Here calculate the necessary displacement! - TODO
            //volumeTotalBumpLeft = neighbourAreaTotalLeft * (originaLength - heightCellDisplacementYoungLeft); // TEST
            volumeOriginalBumpRight = neighbourAreaTotalRight * (originaLength); // TEST - TODO
            ///////
        }

        //       Physics Calculation       //
        // =============================== //

        // Calculate Pressure applicable per frame - if no contact, there is no pressure
        // The three values should be similar, since pressure is based on the contact area

        // Pressure by left feet
        if (counterHitsLeft == 0)
            pressureStressLeft = 0f;
        else
            pressureStressLeft = (TotalForceLeftY) / areaTotalLeft;

        // Pressure by right feet
        if (counterHitsRight == 0)
            pressureStressRight = 0f;
        else
            pressureStressRight = (TotalForceRightY) / areaTotalRight;

        // Total pressure
        if (counterHitsLeft == 0 || counterHitsRight == 0)
            pressureStress = 0f;
        else
            pressureStress = (TotalForceY) / areaTotal;


        //     Deformation Calculation     //
        // =============================== //

        // Given area, pressure and terrain parameters, we calculate the displacement on the terrain
        // The decrement will depend also on the ContactTime used to calculate the corresponding force
        // As for the area, we keep the maximum value

        oldHeightCellDisplacementYoungLeft = pressureStressLeft * (originaLength / (youngModulus));
        if (oldHeightCellDisplacementYoungLeft >= heightCellDisplacementYoungLeft)
        {
            // We use abs. value but for compression, the change in length is negative
            heightCellDisplacementYoungLeft = pressureStressLeft * (originaLength / youngModulus);

            // TEST - Calculate bump
            newBumpHeightDeformationLeft = ((areaTotalLeft * (originaLength - heightCellDisplacementYoungLeft) - volumeVariationLeft) / areaTotalLeft) - originaLength;
        }

        oldHeightCellDisplacementYoungRight = pressureStressRight * (originaLength / (youngModulus));
        if (oldHeightCellDisplacementYoungRight >= heightCellDisplacementYoungRight)
        {
            // We use abs. value but for compression, the change in length is negative
            heightCellDisplacementYoungRight = pressureStressRight * (originaLength / youngModulus);

            // TEST - Calculate bump
            newBumpHeightDeformationRight = ((areaTotalRight * (originaLength - heightCellDisplacementYoungRight) - volumeVariationRight) / areaTotalRight) - originaLength;
        }

        // Given the entire deformation in Y, we calculate the corresponding frame-based deformation based on the frame-time.
        displacementLeft = (Time.deltaTime * (float)heightCellDisplacementYoungLeft) / ContactTime;
        displacementRight = (Time.deltaTime * (float)heightCellDisplacementYoungRight) / ContactTime;

        // Given the  deformation in Y for the bump, we calculate the corresponding frame-based deformation based on the frame-time.
        bumpDisplacementLeftBack = (Time.deltaTime * (float)bumpHeightDeformation) / ContactTime;
        bumpDisplacementRightBack = (Time.deltaTime * (float)bumpHeightDeformation) / ContactTime;
        bumpDisplacementLeftFront = (Time.deltaTime * (float)bumpHeightDeformation) / ContactTime;
        bumpDisplacementRightFront = (Time.deltaTime * (float)bumpHeightDeformation) / ContactTime;

        //     Physics+ Calculation     //
        // =============================== //

        // TEST - Strains (compression)
        strainLong = -(heightCellDisplacementYoungRight) / originaLength;
        strainTrans = poissonRatio * strainLong;

        // TEST - If Poisson is 0.5 : ideal imcompressible material (no change in volume)
        volumeVariationLeft = (1 - 2 * poissonRatio) * (heightCellDisplacementYoungLeft / originaLength) * volumeOriginalLeft;
        volumeVariationRight = (1 - 2 * poissonRatio) * (heightCellDisplacementYoungRight / originaLength) * volumeOriginalRight;

        //        Apply Deformation        //
        // =============================== //

        // 2D iteration Deformation
        // Once we have the displacement, we saved the actual result of applying it to the terrain (only when the foot is grounded)
        if (IsLeftFootGrounded)
        {
            StartCoroutine(DecreaseTerrainLeft(heightMapLeft, heightMapLeftBool, xLeft, zLeft));
        }
        else if(!IsLeftFootGrounded)
        {
            // Every time we lift the foot, we reset the variables and stop the coroutines.
            heightCellDisplacementYoungLeft = 0;
            StopAllCoroutines();
        }

        if (IsRightFootGrounded)
        {
            StartCoroutine(DecreaseTerrainRight(heightMapRight, heightMapRightBool, xRight, zRight));

        }
        else if(!IsRightFootGrounded)
        {
            heightCellDisplacementYoungRight = 0;
            StopAllCoroutines();
        }

        //         Apply Smoothing         //
        // =============================== //

        // First smoothing version
        if (applyFilterLeft)
        {
            // Provisional: When do we smooth?
            if (IsLeftFootGrounded && !IsRightFootGrounded)
            {
                if (!isFilteredLeft)
                {
                    NewFilterHeightMap(xLeft, zLeft, heightMapLeft);
                    filterIterationsLeftCounter++;
                }

                if (filterIterationsLeftCounter >= filterIterationsLeftFoot)
                {
                    isFilteredLeft = true;
                }
            }
            else
            {
                isFilteredLeft = false;
                filterIterationsLeftCounter = 0;
            }
        }

        if (applyFilterRight)
        {
            // Provisional: When do we smooth?
            if (IsRightFootGrounded && !IsLeftFootGrounded)
            {
                if (!isFilteredRight)
                {
                    NewFilterHeightMap(xRight, zRight, heightMapRight);
                    filterIterationsRightCounter++;
                }

                if (filterIterationsRightCounter >= filterIterationsRightFoot)
                {
                    isFilteredRight = true;
                }
            }
            else
            {
                isFilteredRight = false;
                filterIterationsRightCounter = 0;
            }
        }
    }

    IEnumerator DecreaseTerrainLeft(float[,] heightMapLeft, int[,] heightMapLeftBool, int xLeft, int zLeft)
    {
        // 1. Apply frame-per-frame deformation ("displacement")
        for (int zi = -gridSize; zi <= gridSize; zi++)
        {
            for (int xi = -gridSize; xi <= gridSize; xi++)
            {
                // Calculate each cell position wrt World and Heightmap - Left Foot
                Vector3 rayGridLeft = new Vector3(xLeft + xi, terrain.Get(xLeft + xi, zLeft + zi), zLeft + zi);
                Vector3 rayGridWorldLeft = terrain.Grid2World(rayGridLeft);

                // Create each ray for the grid (wrt World) - Left
                RaycastHit leftFootHit;
                Ray upRayLeftFoot = new Ray(rayGridWorldLeft, Vector3.up);

                // If hits the Left Foot and the cell was classified with 2 (direct contact):
                if (LeftFootCollider.Raycast(upRayLeftFoot, out leftFootHit, rayDistance) && (heightMapLeftBool[zi + gridSize, xi + gridSize] == 2))
                {
                    // Cell contacting directly - Decrease until limit reached
                    if (terrain.Get(rayGridLeft.x, rayGridLeft.z) >= terrain.GetConstant(rayGridLeft.x, rayGridLeft.z) - heightCellDisplacementYoungLeft)
                    {
                        heightMapLeft[zi + gridSize, xi + gridSize] = terrain.Get(rayGridLeft.x, rayGridLeft.z) - (displacementLeft);
                    }
                    else
                    {
                        heightMapLeft[zi + gridSize, xi + gridSize] = terrain.Get(rayGridLeft.x, rayGridLeft.z);
                    }
                }
                else if (!LeftFootCollider.Raycast(upRayLeftFoot, out leftFootHit, rayDistance) && (heightMapLeftBool[zi + gridSize, xi + gridSize] == 4) && applyBumps)
                {
                    // If ray does not hit and is classified as BACK neightbour, we create a bump.
                    if (terrain.Get(rayGridLeft.x, rayGridLeft.z) <= terrain.GetConstant(rayGridLeft.x, rayGridLeft.z) + bumpHeightDeformation)
                    {
                        heightMapLeft[zi + gridSize, xi + gridSize] = terrain.Get(rayGridLeft.x, rayGridLeft.z) + (bumpDisplacementLeftBack * MaxTotalForceLeftFootZNorm);
                    }
                    else
                    {
                        heightMapLeft[zi + gridSize, xi + gridSize] = terrain.Get(rayGridLeft.x, rayGridLeft.z);
                    }
                }
                else
                {
                    // If is out of reach
                    heightMapLeft[zi + gridSize, xi + gridSize] = terrain.Get(rayGridLeft.x, rayGridLeft.z);
                }
            }
        }

        // This version of smoothing, does not set the terrain, only filters (return) the new heightmap
        // TODO - NOT WORKING YET - See function
        if (applyFilterLeft2)
        {
            // Provisional: When do we smooth?
            if (IsLeftFootGrounded && !IsRightFootGrounded)
            {
                if (!isFilteredLeft)
                {
                    heightMapLeft = NewFilterHeightMapReturn(xLeft, zLeft, heightMapLeft);
                    filterIterationsLeftCounter++;
                }

                if (filterIterationsLeftCounter >= filterIterationsLeftFoot)
                {
                    isFilteredLeft = true;
                }
            }
            else
            {
                isFilteredLeft = false;
                filterIterationsLeftCounter = 0;
            }
        }

        // 2. Save terrain
        if (applyFootprints)
        {
            for (int zi = -gridSize; zi <= gridSize; zi++)
            {
                for (int xi = -gridSize; xi <= gridSize; xi++)
                {
                    Vector3 rayGridLeft = new Vector3(xLeft + xi, terrain.Get(xLeft + xi, zLeft + zi), zLeft + zi);
                    terrain.Set(rayGridLeft.x, rayGridLeft.z, heightMapLeft[zi + gridSize, xi + gridSize]);
                }
            }
        }

        yield return null;
    }

    IEnumerator DecreaseTerrainRight(float[,] heightMapRight, int[,] heightMapRightBool, int xRight, int zRight)
    {
        // 1. Apply frame-per-frame deformation ("displacement")
        for (int zi = -gridSize; zi <= gridSize; zi++)
        {
            for (int xi = -gridSize; xi <= gridSize; xi++)
            {
                // Calculate each cell position wrt World and Heightmap - Right Foot
                Vector3 rayGridRight = new Vector3(xRight + xi, terrain.Get(xRight + xi, zRight + zi), zRight + zi);
                Vector3 rayGridWorldRight = terrain.Grid2World(rayGridRight);

                // Create each ray for the grid (wrt World) - Right
                RaycastHit rightFootHit;
                Ray upRayRightFoot = new Ray(rayGridWorldRight, Vector3.up);

                // If hits the Right Foot and the cell was classified with 2 (direct contact):
                if (RightFootCollider.Raycast(upRayRightFoot, out rightFootHit, rayDistance) && (heightMapRightBool[zi + gridSize, xi + gridSize] == 2))
                {
                    // Cell contacting directly - Decrease until limit reached
                    if (terrain.Get(rayGridRight.x, rayGridRight.z) >= terrain.GetConstant(rayGridRight.x, rayGridRight.z) - heightCellDisplacementYoungRight)
                    {
                        heightMapRight[zi + gridSize, xi + gridSize] = terrain.Get(rayGridRight.x, rayGridRight.z) - (displacementRight);
                    }
                    else
                    {
                        heightMapRight[zi + gridSize, xi + gridSize] = terrain.Get(rayGridRight.x, rayGridRight.z);
                    }
                }
                else if (!RightFootCollider.Raycast(upRayRightFoot, out rightFootHit, rayDistance) && (heightMapRightBool[zi + gridSize, xi + gridSize] == 4) && applyBumps)
                {
                    // If ray does not hit and is classified as BACK neightbour, we create a bump.
                    if (terrain.Get(rayGridRight.x, rayGridRight.z) <= terrain.GetConstant(rayGridRight.x, rayGridRight.z) + bumpHeightDeformation)
                    {
                        heightMapRight[zi + gridSize, xi + gridSize] = terrain.Get(rayGridRight.x, rayGridRight.z) + (bumpDisplacementRightBack * MaxTotalForceRightFootZNorm);
                    }
                    else
                    {
                        heightMapRight[zi + gridSize, xi + gridSize] = terrain.Get(rayGridRight.x, rayGridRight.z);
                    }
                }
                else
                {
                    // If is out of reach
                    heightMapRight[zi + gridSize, xi + gridSize] = terrain.Get(rayGridRight.x, rayGridRight.z);
                }
            }
        }

        // This version of smoothing, does not set the terrain, only filters (return) the new heightmap
        // TODO - NOT WORKING YET - See function
        if (applyFilterRight2)
        {
            // Provisional: When do we smooth?
            if (IsRightFootGrounded && !IsLeftFootGrounded)
            {
                if (!isFilteredRight)
                {
                    heightMapRight = NewFilterHeightMapReturn(xRight, zRight, heightMapRight);
                    filterIterationsRightCounter++;
                }

                if (filterIterationsRightCounter >= filterIterationsRightFoot)
                {
                    isFilteredRight = true;
                }
            }
            else
            {
                isFilteredRight = false;
                filterIterationsRightCounter = 0;
            }
        }

        // 2. Save terrain
        if (applyFootprints)
        {
            for (int zi = -gridSize; zi <= gridSize; zi++)
            {
                for (int xi = -gridSize; xi <= gridSize; xi++)
                {
                    Vector3 rayGridRight = new Vector3(xRight + xi, terrain.Get(xRight + xi, zRight + zi), zRight + zi);
                    terrain.Set(rayGridRight.x, rayGridRight.z, heightMapRight[zi + gridSize, xi + gridSize]);
                }
            }
        }

        yield return null;
    }

    // Pre-filter Methods

    private float[,] FilterBufferLeft(float[,] heightMapLeft, int[,] heightMapLeftBool)
    {
        float[,] heightMapLeftFiltered = new float[2 * gridSize + 1, 2 * gridSize + 1];

        for (int zi = -gridSize; zi <= gridSize; zi++)
        {
            for (int xi = -gridSize; xi <= gridSize; xi++)
            {
                if (heightMapLeftBool[zi + gridSize, xi + gridSize] == 0)
                {
                    heightMapLeftFiltered[zi + gridSize, xi + gridSize] = heightMapLeft[zi + gridSize, xi + gridSize];
                }
                else
                {
                    float n = 2.0f * gridSizeKernel + 1.0f;
                    float sum = 0;

                    for (int szi = -gridSizeKernel; szi <= gridSizeKernel; szi++)
                    {
                        for (int sxi = -gridSizeKernel; sxi <= gridSizeKernel; sxi++)
                        {
                            sum += heightMapLeft[gridSize + szi, gridSize + sxi];
                        }
                    }

                    heightMapLeftFiltered[zi + gridSize, xi + gridSize] = sum / (n * n);
                }
            }
        }

        return heightMapLeftFiltered;
    }

    private float[,] FilterBufferRight(float[,] heightMapRight, int[,] heightMapRightBool)
    {
        float[,] heightMapRightFiltered = new float[2 * gridSize + 1, 2 * gridSize + 1];

        for (int zi = -gridSize; zi <= gridSize; zi++)
        {
            for (int xi = -gridSize; xi <= gridSize; xi++)
            {
                if (heightMapRightBool[zi + gridSize, xi + gridSize] == 0)
                {
                    heightMapRightFiltered[zi + gridSize, xi + gridSize] = heightMapRight[zi + gridSize, xi + gridSize];
                }
                else
                {
                    float n = 2.0f * gridSizeKernel + 1.0f;
                    float sum = 0;

                    for (int szi = -gridSizeKernel; szi <= gridSizeKernel; szi++)
                    {
                        for (int sxi = -gridSizeKernel; sxi <= gridSizeKernel; sxi++)
                        {
                            sum += heightMapRight[gridSize + szi, gridSize + sxi];
                        }
                    }

                    heightMapRightFiltered[zi + gridSize, xi + gridSize] = sum / (n * n);
                }
            }
        }

        return heightMapRightFiltered;
    }

    // Post-filter Methods

    // Old Gaussian Blur
    private void FilterHeightmap(int zLeft, int xLeft, float[,] heightmap)
    {
        float[,] result = TerrainData.GetHeights(0, 0, (int)terrain.GridSize().x, (int)terrain.GridSize().z);

        for (int zi = -gridSize; zi < gridSize; zi++)
        {
            for (int xi = -gridSize; xi < gridSize; xi++)
            {
                result[zLeft + zi, xLeft + xi] = 
                    heightmap[zLeft + zi - 2, xLeft + xi - 2]
                    + 4 * heightmap[zLeft + zi - 2, xLeft + xi - 1]
                    + 6 * heightmap[zLeft + zi - 2, xLeft + xi]
                    + heightmap[zLeft + zi - 2, xLeft + xi + 2]
                    + 4 * heightmap[zLeft + zi - 2, xLeft + xi + 1]
                    + 4 * heightmap[zLeft + zi - 1, xLeft + xi + 2]
                    + 16 * heightmap[zLeft + zi - 1, xLeft + xi + 1]
                    + 4 * heightmap[zLeft + zi - 1, xLeft + xi - 2]
                    + 16 * heightmap[zLeft + zi - 1, xLeft + xi - 1]
                    + 24 * heightmap[zLeft + zi - 1, xLeft + xi]
                    + 6 * heightmap[zLeft + zi, xLeft + xi - 2]
                    + 24 * heightmap[zLeft + zi, xLeft + xi - 1]
                    + 6 * heightmap[zLeft + zi, xLeft + xi + 2]
                    + 24 * heightmap[zLeft + zi, xLeft + xi + 1]
                    + 36 * heightmap[zLeft + zi, xLeft + xi]
                    + heightmap[zLeft + zi + 2, xLeft + xi - 2]
                    + 4 * heightmap[zLeft + zi + 2, xLeft + xi - 1]
                    + 6 * heightmap[zLeft + zi + 2, xLeft + xi]
                    + heightmap[zLeft + zi + 2, xLeft + xi + 2]
                    + 4 * heightmap[zLeft + zi + 2, xLeft + xi + 1]
                    + 4 * heightmap[zLeft + zi + 1, xLeft + xi + 2]
                    + 16 * heightmap[zLeft + zi + 1, xLeft + xi + 1]
                    + 4 * heightmap[zLeft + zi + 1, xLeft + xi - 2]
                    + 16 * heightmap[zLeft + zi + 1, xLeft + xi - 1]
                    + 24 * heightmap[zLeft + zi + 1, xLeft + xi];

                result[zLeft + zi, xLeft + xi] *= 1.0f / 256.0f;
            }
        }

        HeightMapFiltered = result;
    }

    // New Gaussian Blur (3x3) 
    public void NewFilterHeightMap(int x, int z, float[,] heightMap)
    {
        float[,] heightMapFiltered = new float[2 * gridSize + 1, 2 * gridSize + 1];

        for (int zi = -gridSize + marginAroundGrid; zi <= gridSize - marginAroundGrid; zi++)
        {
            for (int xi = -gridSize + marginAroundGrid; xi <= gridSize - marginAroundGrid; xi++)
            {
                Vector3 rayGridLeft = new Vector3(x + xi, terrain.Get(x + xi, z + zi), z + zi);

                heightMapFiltered[zi + gridSize, xi + gridSize] =
                    heightMap[zi + gridSize - 1, xi + gridSize - 1]
                    + 2 * heightMap[zi + gridSize - 1, xi + gridSize]
                    + 1 * heightMap[zi + gridSize - 1, xi + gridSize + 1]
                    + 2 * heightMap[zi + gridSize, xi + gridSize - 1]
                    + 4 * heightMap[zi + gridSize, xi + gridSize]
                    + 2 * heightMap[zi + gridSize, xi + gridSize + 1]
                    + 1 * heightMap[zi + gridSize + 1, xi + gridSize - 1]
                    + 2 * heightMap[zi + gridSize + 1, xi + gridSize]
                    + 1 * heightMap[zi + gridSize + 1, xi + gridSize + 1];

                heightMapFiltered[zi + gridSize, xi + gridSize] *= 1.0f / 16.0f;

                terrain.Set(rayGridLeft.x, rayGridLeft.z, (heightMapFiltered[zi + gridSize, xi + gridSize]));
            }
        }
    }

    // New Gaussian Blur (3x3) - Return (TODO STILL NOT WORK - Version 1 works fine, so skipping)
    public float[,] NewFilterHeightMapReturn(int x, int z, float[,] heightMap)
    {
        float[,] heightMapFiltered = new float[2 * gridSize + 1, 2 * gridSize + 1];

        for (int zi = -gridSize; zi <= gridSize; zi++)
        {
            for (int xi = -gridSize; xi <= gridSize; xi++)
            {
                // It avoids the offset or border
                if((zi > -gridSize + marginAroundGrid) && (zi < gridSize - marginAroundGrid) &&
                    (xi > -gridSize + marginAroundGrid) && (xi < gridSize - marginAroundGrid))
                {
                    Vector3 rayGridLeft = new Vector3(x + xi, terrain.Get(x + xi, z + zi), z + zi);

                    heightMapFiltered[zi + gridSize, xi + gridSize] =
                        heightMap[zi + gridSize - 1, xi + gridSize - 1]
                        + 2 * heightMap[zi + gridSize - 1, xi + gridSize]
                        + 1 * heightMap[zi + gridSize - 1, xi + gridSize + 1]
                        + 2 * heightMap[zi + gridSize, xi + gridSize - 1]
                        + 4 * heightMap[zi + gridSize, xi + gridSize]
                        + 2 * heightMap[zi + gridSize, xi + gridSize + 1]
                        + 1 * heightMap[zi + gridSize + 1, xi + gridSize - 1]
                        + 2 * heightMap[zi + gridSize + 1, xi + gridSize]
                        + 1 * heightMap[zi + gridSize + 1, xi + gridSize + 1];

                    heightMapFiltered[zi + gridSize, xi + gridSize] *= 1.0f / 16.0f;

                }
                else
                {
                    heightMapFiltered[zi + gridSize, xi + gridSize] = heightMap[zi + gridSize, xi + gridSize];
                }

            }
        }

        return heightMapFiltered;
    }

    // TEST - Calculate Barycentric Coordinates Right
    /*
    private void computeBarycentricCoordinatesRight(Vector3 center, List<Vector3> neighboursPositionsRight)
    {
        float weightSumRight = 0;

        for (int i = 0; i < neighboursPositionsRight.Count; i++)
        {
            int prev = (i + neighboursPositionsRight.Count - 1) % neighboursPositionsRight.Count;
            int next = (i + 1) % neighboursPositionsRight.Count;

            allSumCotRight.Add(contangAnglePreviousRight(center, neighboursPositionsRight[i], neighboursPositionsRight[prev], i, prev) + contangAngleNextRight(center, neighboursPositionsRight[i], neighboursPositionsRight[next], i, next));
            //allSumCotRight[i] = contangAnglePreviousRight(center, neighboursPositionsRight[i], neighboursPositionsRight[prev], i, prev) + contangAngleNextRight(center, neighboursPositionsRight[i], neighboursPositionsRight[next], i, next);

            neighboursWeightsRight.Add(allSumCotRight[i] / Vector3.Distance(center, neighboursPositionsRight[i]));
            weightSumRight += neighboursWeightsRight[i];
        }

        for (int i = 0; i < neighboursWeightsRight.Count; i++)
        {
            neighboursWeightsRight[i] /= weightSumRight;
        }
    }

    private float contangAnglePreviousRight(Vector3 p, Vector3 j, Vector3 neighbour, int vertex, int vertex_neightbour)
    {
        var pj = p - j;
        var bc = neighbour - j;

        float angle = Mathf.Atan2(Vector3.Cross(pj, bc).magnitude, Vector3.Dot(pj, bc));
        float angleCot = 1f / Mathf.Tan(angle);

        //allAnglesPrevRight[vertex] = angle * Mathf.Rad2Deg;

        return angleCot;
    }

    private float contangAngleNextRight(Vector3 p, Vector3 j, Vector3 neighbour, int vertex, int vertex_neightbour)
    {
        var pj = p - j;
        var bc = neighbour - j;

        float angle = Mathf.Atan2(Vector3.Cross(pj, bc).magnitude, Vector3.Dot(pj, bc));
        float angleCot = 1f / Mathf.Tan(angle);

        //allAnglesNextRight[vertex] = angle * Mathf.Rad2Deg;

        return angleCot;
    }
    */
}
