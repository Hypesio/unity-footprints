/****************************************************
 * File: PhysicalFootprint.cs
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

/// <summary>
/// Brush to create dynamic footprints on the heighmap terrain. 
/// First, it analyzes the cells to be affected based on the ray-cast and feet colliders.
/// Then, it calculates the displacement per cell based on contact area and character weight / force applied.
/// </summary>
public class PhysicalFootprint : TerrainBrushPhysicalFootprint
{
    #region Instance Fields

    [Header("Physically-based Footprints Deformation - (SET UP)")]
    public bool applyFootprints = false;
    public bool applyBumps = false;
    private bool applyModulatedBumps = true;

    [Header("Terrain Compression - (SET UP)")]
    [Space(20)]
    [Range(100000, 10000000)] public double youngModulus = 1000000;
    public float originalLengthZero = 0.3f;

    [Header("Vertical Displacement - (SET UP)")]
    [Space(20)]
    [Range(0, 0.5f)] public float poissonR = 0.4f;

    [Header("Compression Grid - (SET UP)")]
    [Space(20)]
    [Range(0, 20)] public int gridSize = 15;
    [Range(0f, 1f)] public float raycastDistance = 0.075f;
    [Range(0f, 1f)] public float offsetRay = 0.04f;

    [Header("Vertical Displcamenet Grid - (SET UP)")]
    [Space(20)]
    public int offsetBumpGrid = 4;
    public int neighboursSearchArea = 4;

    [Header("Gaussian Filtering - (SET UP)")]
    [Space(20)]
    public bool applyFilterLeft = false;
    public bool applyFilterRight = false;
    public int filterIterationsLeftFoot = 2;
    public int filterIterationsRightFoot = 2;
    [Space(10)]
    public int marginAroundGrid = 3;
    private int filterIterationsLeftCounter = 0;
    private int filterIterationsRightCounter = 0;
    public bool isFilteredLeft = false;
    public bool isFilteredRight = false;
    [Range(0, 5)] private int gridSizeKernel = 1;

    [Header("Grids - Debug")]
    [Space(20)]
    public bool showGridDebugLeft = false;
    public bool showGridDebugRight = false;
    public bool showGridBumpDebug = false;
    public bool showGridBumpAidDebug = false;
    public bool printTerrainInformation = false;

    [Header("Grids - Number of hits")]
    [Space(20)]
    public int counterHitsLeft;
    public int counterHitsRight;
    public int neighbourCellsLeft;
    public int neighbourCellsRight;
    public List<Vector3> neighboursPositionsRight = new List<Vector3>();
    public List<Vector3> neighboursPositionsLeft = new List<Vector3>();

    [Header("Grids - Contact Area Feet-Ground")]
    [Space(20)]
    public float lenghtCellX;
    public float lenghtCellZ;
    public float areaCell;
    [Space(10)]
    public float areaTotal = 0f;
    public float areaTotalLeft = 0f;
    public float areaTotalRight = 0f;
    [Space(10)]
    public float neighbourAreaTotalLeft;
    public float neighbourAreaTotalRight;
    private float oldAreaTotalLeft = 0f;
    private float oldAreaTotalRight = 0f;

    [Header("Terrain Deformation - Pressure")]
    [Space(20)]
    public float pressureStress;
    public float pressureStressLeft;
    public float pressureStressRight;

    [Header("Terrain Compression - Displacement")]
    [Space(20)]
    public double heightCellDisplacementYoungLeft = 0f;
    public double heightCellDisplacementYoungRight = 0f;
    [Space(10)]
    public float displacementLeft;
    public float displacementRight;
    private double oldHeightCellDisplacementYoungLeft = 0f;
    private double oldHeightCellDisplacementYoungRight = 0f;

    [Header("Vertical Displacement - Displacement")]
    public double bumpHeightDeformationLeft = 0f;
    public double bumpHeightDeformationRight = 0f;
    [Space(10)]
    public float bumpDisplacementLeft;
    public float bumpDisplacementRight;
    private float oldNeighbourAreaTotalLeft;
    private float oldNeighbourAreaTotalRight;

    [Header("Terrain Deformation - Volume Rod Approximation")]
    [Space(20)]
    public double volumeOriginalLeft = 0f; // Original volume under left foot
    public double volumeOriginalRight = 0f; // Original volume under right foot
    public double volumeTotalLeft = 0f; // Volume left after deformation
    public double volumeTotalRight = 0f; // Volume left after deformation
    public double volumeVariationPoissonLeft; // Volume change due to compressibility of the material
    public double volumeVariationPoissonRight; // Volume change due to compressibility of the material
    public double volumeDifferenceLeft; // Volume difference pre/post deformation without taking into account compressibility
    public double volumeDifferenceRight; // Volume difference pre/post deformation without taking into account compressibility
    public double volumeNetDifferenceLeft; // Volume difference pre/post deformation taking into account compressibility
    public double volumeNetDifferenceRight; // Volume difference pre/post deformation taking into account compressibility
    public double volumeCellLeft; // Volume/cell distributed over countour
    public double volumeCellRight; // Volume/cell distributed over countour

    // TEST - Modulated Bump
    [Header("New - Modulated Bump")]
    [Space(20)]
    private float maxDLeft; // Distance value to one vertex
    private float maxDRight;
    private float dLeft; // Distance to a cell
    private float dRight;
    private float weightMult = 0.2f;

    #endregion

    #region Read-only & Static Fields
    
    public enum CellState {
        NoContact = 0, 
        Contour = 1, 
        Contact = 2
    }
    // Force for bump
    private Vector3 forcePositionLeft;
    private Vector3 forcePositionLeft2D;
    private Vector3 forcePositionLeft2DWorld;
    private Vector3 forcePositionRight;
    private Vector3 forcePositionRight2D;
    private Vector3 forcePositionRight2DWorld;

    // Others
    private float[,] heightMapLeft;
    private float[,] heightMapRight;
    private float[,] heightMapLeftFiltered;
    private float[,] heightMapRightFiltered;
    private int[,] heightMapLeftBool;
    private int[,] heightMapRightBool;
    private double strainLong;
    private double strainTrans;

    #endregion

    #region Instance Methods

    /// <summary>
    /// Method that takes the IK positions for each feet and apply displacement to ground.
    /// </summary>
    /// <param name="xLeft"></param>
    /// <param name="zLeft"></param>
    /// <param name="xRight"></param>
    /// <param name="zRight"></param>
    public override void DrawFootprint(int xLeft, int zLeft, int xRight, int zRight)
    {
        #region Initial Declarations

        // 1. If activated, takes the prefab information from the master script
        if (DeformationChoice == sourceDeformation.useTerrainPrefabs)
        {
            youngModulus = YoungM;
            poissonR = PoissonRatio;
            applyBumps = ActivateBump;

            if (FilterIte != 0)
            {
                applyFilterLeft = true;
                applyFilterRight = true;
                filterIterationsLeftFoot = FilterIte;
                filterIterationsRightFoot = FilterIte;
            }
            else if (FilterIte == 0)
            {
                applyFilterLeft = false;
                applyFilterRight = false;
            }
        }

        // 2. If activated, takes the UI information from the interface
        if (DeformationChoice == sourceDeformation.useUI)
        {
            applyFootprints = ActivateToggleDef.isOn;
            applyBumps = ActivateToggleBump.isOn;
            applyFilterLeft = ActivateToggleGauss.isOn;
            applyFilterRight = ActivateToggleGauss.isOn;

            showGridDebugLeft = ActivateToggleShowGrid.isOn;
            showGridDebugRight = ActivateToggleShowGrid.isOn;
            showGridBumpDebug = ActivateToggleShowBump.isOn;

            youngModulus = YoungSlider.value;
            poissonR = PoissonSlider.value;
            filterIterationsLeftFoot = (int)IterationsSlider.value;
            filterIterationsRightFoot = (int)IterationsSlider.value;
        }

        // 3. Reset counter hits and lists
        counterHitsLeft = 0;
        counterHitsRight = 0;
        neighbourCellsLeft = 0;
        neighbourCellsRight = 0;
        neighboursPositionsRight.Clear();
        neighboursPositionsLeft.Clear();

        // 4. Heightmaps for each foot
        float[,] heightMapLeft = new float[2 * gridSize + 1, 2 * gridSize + 1];
        float[,] heightMapRight = new float[2 * gridSize + 1, 2 * gridSize + 1];
        int[,] heightMapLeftBool = new int[2 * gridSize + 1, 2 * gridSize + 1];
        int[,] heightMapRightBool = new int[2 * gridSize + 1, 2 * gridSize + 1];

        

        #endregion

        #region Terrain Declaration

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

        // 5. Calculate area per cell outside the loop
        lenghtCellX = terrain.TerrainSize().x / (terrain.GridSize().x - 1);
        lenghtCellZ = terrain.TerrainSize().z / (terrain.GridSize().z - 1);
        areaCell = lenghtCellX * lenghtCellZ;

        #endregion

        #region Contact Area Calculation

        // 2D iteration for both feet
        // 1. It counts the number of hits, save the classified cell in a list and debug ray-casting
        for (int zi = -gridSize; zi <= gridSize; zi++)
        {
            for (int xi = -gridSize; xi <= gridSize; xi++)
            {
                // A. Calculate each cell position wrt World and Heightmap - Left Foot
                // The sensors that counts the number of hits always remain on the surface
                Vector3 rayGridLeft = new Vector3(xLeft + xi, terrain.Get(xLeft + xi, zLeft + zi) - offsetRay, zLeft + zi);
                Vector3 rayGridWorldLeft = terrain.Grid2World(rayGridLeft);

                // A. Calculate each cell position wrt World and Heightmap - Right Foot
                // The sensors that counts the number of hits always remain on the surface
                Vector3 rayGridRight = new Vector3(xRight + xi, terrain.Get(xRight + xi, zRight + zi) - offsetRay, zRight + zi);
                Vector3 rayGridWorldRight = terrain.Grid2World(rayGridRight);

                //------//

                // B. Create each ray for the grid (wrt World) - Left
                RaycastHit leftFootHit;
                Ray upRayLeftFoot = new Ray(rayGridWorldLeft, Vector3.up);

                // B. Create each ray for the grid (wrt World) - Right
                RaycastHit rightFootHit;
                Ray upRayRightFoot = new Ray(rayGridWorldRight, Vector3.up);

                //------//

                // C. If hits the Left Foot, increase counter and add cell to be affected
                if (LeftFootCollider.Raycast(upRayLeftFoot, out leftFootHit, raycastDistance))
                {
                    // Cell contacting directly
                    heightMapLeftBool[zi + gridSize, xi + gridSize] = 2;
                    counterHitsLeft++;

                    if (showGridDebugLeft)
                        Debug.DrawRay(rayGridWorldLeft, Vector3.up * raycastDistance, Color.blue);
                }
                else
                {
                    // No contact
                    heightMapLeftBool[zi + gridSize, xi + gridSize] = 0;

                    if (showGridDebugLeft)
                        Debug.DrawRay(rayGridWorldLeft, Vector3.up * raycastDistance, Color.red);
                }
                
                // C. If hits the Right Foot, increase counter and add cell to be affected
                if (RightFootCollider.Raycast(upRayRightFoot, out rightFootHit, raycastDistance))
                {
                    // Cell contacting directly
                    heightMapRightBool[zi + gridSize, xi + gridSize] = (int)CellState.Contact;
                    counterHitsRight++;

                    if (showGridDebugRight)
                        Debug.DrawRay(rayGridWorldRight, Vector3.up * raycastDistance, Color.blue);
                }
                else
                {
                    // No contact
                    heightMapRightBool[zi + gridSize, xi + gridSize] = (int)CellState.NoContact;

                    if (showGridDebugRight)
                        Debug.DrawRay(rayGridWorldRight, Vector3.up * raycastDistance, Color.red);
                }
            }
        }

        // 2. Terrain Deformation is affected by an increasing value of the contact area, therefore the deformation
        // will be defined by the maximum contact area in each frame
        oldAreaTotalLeft = ((counterHitsLeft) * areaCell);
        if (oldAreaTotalLeft >= areaTotalLeft)
        {
            // Area of contact
            areaTotalLeft = ((counterHitsLeft) * areaCell);

            // Volume under the foot for that recent calculated area
            volumeOriginalLeft = areaTotalLeft * (originalLengthZero);
        }

        oldAreaTotalRight = ((counterHitsRight) * areaCell);
        if (oldAreaTotalRight >= areaTotalRight)
        {
            // Area of contact
            areaTotalRight = ((counterHitsRight) * areaCell);

            // Volume under the foot for that recent calculated area
            volumeOriginalRight = areaTotalRight * (originalLengthZero);
        }

        // Total Area and Volume for both feet
        areaTotal = areaTotalLeft + areaTotalRight;

        #endregion

        #region Detecting Contour

        if (IsRightFootGrounded)
        {
            // 1. We don't need to check the whole grid - just in the 5x5 inner grid is enough
            for (int zi = -gridSize + offsetBumpGrid; zi <= gridSize - offsetBumpGrid; zi++)
            {
                for (int xi = -gridSize + offsetBumpGrid; xi <= gridSize - offsetBumpGrid; xi++)
                {
                    // A. If the cell was not in contact, it's a potential neighbour (countour) cell
                    if (heightMapRightBool[zi + gridSize, xi + gridSize] == (int)CellState.NoContact)
                    {
                        // B. Only checking adjacent cells - increasing this would allow increasing the area of the bump
                        for (int zi_sub = -neighboursSearchArea; zi_sub <= neighboursSearchArea; zi_sub++)
                        {
                            for (int xi_sub = -neighboursSearchArea; xi_sub <= neighboursSearchArea; xi_sub++)
                            {
                                // C. If there is a contact point around the cell
                                if (heightMapRightBool[zi + zi_sub + gridSize, xi + xi_sub + gridSize] == (int)CellState.Contact)
                                {
                                    Vector3 rayGridRight = new Vector3(xRight + xi, terrain.Get(xRight + xi, zRight + zi) - offsetRay, zRight + zi);
                                    Vector3 rayGridWorldRight = terrain.Grid2World(rayGridRight);

                                    // D. Mark that cell as a countour point
                                    heightMapRightBool[zi + gridSize, xi + gridSize] = (int)CellState.Contour;
                                    break;
                                }
                            }
                        }
                    }

                    if (heightMapRightBool[zi + gridSize, xi + gridSize] == (int)CellState.Contour)
                    {
                        // C. Each neightbour cell in world space
                        Vector3 rayGridRight = new Vector3(xRight + xi, terrain.Get(xRight + xi, zRight + zi) - offsetRay, zRight + zi);
                        Vector3 rayGridWorldRight = terrain.Grid2World(rayGridRight);

                        // Also store in general array
                        neighboursPositionsRight.Add(rayGridWorldRight);

                        // D. Counter neightbours
                        neighbourCellsRight++;

                        if (showGridBumpDebug)
                            Debug.DrawRay(rayGridWorldRight, Vector3.up * raycastDistance, Color.yellow);
                    }
                }
            }
        }

        if (IsLeftFootGrounded)
        {
            // 1. We don't need to check the whole grid - just in the 5x5 inner grid is enough
            for (int zi = -gridSize + offsetBumpGrid; zi <= gridSize - offsetBumpGrid; zi++)
            {
                for (int xi = -gridSize + offsetBumpGrid; xi <= gridSize - offsetBumpGrid; xi++)
                {
                    // If the cell was not in contact, it's a potential neighbour (countour) cell
                    if (heightMapLeftBool[zi + gridSize, xi + gridSize] == 0)
                    {
                        // Only checking adjacent cells - increasing this would allow increasing the area of the bump
                        for (int zi_sub = -neighboursSearchArea; zi_sub <= neighboursSearchArea; zi_sub++)
                        {
                            for (int xi_sub = -neighboursSearchArea; xi_sub <= neighboursSearchArea; xi_sub++)
                            {
                                if (heightMapLeftBool[zi + zi_sub + gridSize, xi + xi_sub + gridSize] == 2)
                                {
                                    Vector3 rayGridLeft = new Vector3(xLeft + xi, terrain.Get(xLeft + xi, zLeft + zi) - offsetRay, zLeft + zi);
                                    Vector3 rayGridWorldLeft = terrain.Grid2World(rayGridLeft);

                                    // Mark that cell as a countour point
                                    heightMapLeftBool[zi + gridSize, xi + gridSize] = 1;
                                    break;
                                }
                            }
                        }

                        if (heightMapLeftBool[zi + gridSize, xi + gridSize] == 1)
                        {
                            // C. Each neightbour cell in world space
                            Vector3 rayGridLeft = new Vector3(xLeft + xi, terrain.Get(xLeft + xi, zLeft + zi) - offsetRay, zLeft + zi);
                            Vector3 rayGridWorldLeft = terrain.Grid2World(rayGridLeft);

                            // Also store in general array
                            neighboursPositionsLeft.Add(rayGridWorldLeft);

                            // D. Counter neightbours
                            neighbourCellsLeft++;

                            if (showGridBumpDebug)
                                Debug.DrawRay(rayGridWorldLeft, Vector3.up * raycastDistance, Color.yellow);

                        }
                    }
                }
            }
        }

        #endregion
        
        // === Modulated Bump === //
        float[,] weightsBumpLeft = PhysicalFootprintWeights.SetupWeights(gridSize, offsetBumpGrid, heightMapLeftBool);
        float[,] weightsBumpRight = PhysicalFootprintWeights.SetupWeights(gridSize,  offsetBumpGrid, heightMapRightBool);
        
        if (applyModulatedBumps)
        {
            Vector3 speed = DeformTerrainMaster.Instance.chestSpeed;
            //Vector3 speed = (DeformTerrainMaster.Instance.feetSpeedLeft + DeformTerrainMaster.Instance.feetSpeedRight) / 2;
            LayerMask ground = LayerMask.GetMask("Ground");
            if (IsLeftFootGrounded)
            {
                // Vector3 speed = DeformTerrainMaster.Instance.feetSpeedLeft;
                //Debug.Log("[Deform] Left feet speed " + speed);
                
                Vector3 normalLeft = FrictionModel.GetFloorNormalFromFeet(false, ground);
                Vector3 slopeLeft = Vector3.Normalize(Vector3.ProjectOnPlane(Vector3.down, normalLeft));
                Debug.Log("[Deform] moveDirectionLeft " + DeformTerrainMaster.Instance.chestSpeed);
                
                weightsBumpLeft = PhysicalFootprintWeights.UpdateWeightsUsingFloor(weightsBumpLeft, heightMapLeftBool,
                    gridSize, speed, neighbourCellsLeft);
                
            }

            if (IsRightFootGrounded)
            {
                // Vector3 speed = DeformTerrainMaster.Instance.feetSpeedRight;
                //Debug.Log("[Deform] right feet speed " + speed)
                
                Vector3 normalRight = FrictionModel.GetFloorNormalFromFeet(true, ground);
                Vector3 slopeRight = Vector3.Normalize(Vector3.ProjectOnPlane(Vector3.down, normalRight));
                // Debug.Log("[Deform] moveDirectionRight " + moveDirectionRight.y);
                
                weightsBumpRight = PhysicalFootprintWeights.UpdateWeightsUsingFloor(weightsBumpRight, heightMapRightBool,
                    gridSize, speed, neighbourCellsRight);
                
            }
        }
        
        #region Neighbour Area Calculation

        // 3. Calculate the neightbour area for each foot
        oldNeighbourAreaTotalLeft = ((neighbourCellsLeft) * areaCell);
        if (oldNeighbourAreaTotalLeft >= neighbourAreaTotalLeft)
        {
            // Area of bump
            neighbourAreaTotalLeft = ((neighbourCellsLeft) * areaCell);
        }

        oldNeighbourAreaTotalRight = ((neighbourCellsRight) * areaCell);
        if (oldNeighbourAreaTotalRight >= neighbourAreaTotalRight)
        {
            // Area of bump
            neighbourAreaTotalRight = ((neighbourCellsRight) * areaCell);
        }

        #endregion

        #region Physics Calculation

        // 1. Calculate Pressure applicable per frame - if no contact, there is no pressure
        // The three values should be similar, since pressure is based on the contact area

        // Pressure by Left Foot
        if (counterHitsLeft == 0)
            pressureStressLeft = 0f;
        else
            pressureStressLeft = (TotalForceLeftY) / areaTotalLeft;

        // Pressure by Right Foot
        if (counterHitsRight == 0)
            pressureStressRight = 0f;
        else
            pressureStressRight = (TotalForceRightY) / areaTotalRight;

        // Total Pressure
        if (counterHitsLeft == 0 || counterHitsRight == 0)
            pressureStress = 0f;
        else
            pressureStress = (TotalForceY) / areaTotal;

        #endregion

        #region Deformation Calculation

        // 1. Given area, pressure and terrain parameters, we calculate the displacement on the terrain
        // The decrement will depend also on the ContactTime used to calculate the corresponding force

        // 2. As for the area, we keep the maximum value
        oldHeightCellDisplacementYoungLeft = pressureStressLeft * (originalLengthZero / (youngModulus));
        if (oldHeightCellDisplacementYoungLeft >= heightCellDisplacementYoungLeft)
        {
            // We use abs. value but for compression, the change in length is negative
            heightCellDisplacementYoungLeft = pressureStressLeft * (originalLengthZero / youngModulus);

            // Resulting volume under the left foot after displacement - CHANGED
            volumeTotalLeft = areaTotalLeft * (originalLengthZero + (-heightCellDisplacementYoungLeft));

            // Calculate the difference in volume, takes into account the compressibility and estimate volume up per neighbour cell
            volumeDifferenceLeft =  volumeTotalLeft - volumeOriginalLeft; // NEGATIVE CHANGE
            volumeNetDifferenceLeft = -volumeDifferenceLeft + volumeVariationPoissonLeft; // Calculate directly the volume in the bump upwards (positive)
            
            // Distribute volume
            volumeCellLeft = volumeNetDifferenceLeft / neighbourCellsLeft;

            // 2. In this case, we do it with volume. Remember: must be negative for later.
            bumpHeightDeformationLeft = volumeCellLeft / areaCell;
        }

        oldHeightCellDisplacementYoungRight = pressureStressRight * (originalLengthZero / (youngModulus));
        if (oldHeightCellDisplacementYoungRight >= heightCellDisplacementYoungRight)
        {
            // We use abs. value but for compression, the change in length is negative
            heightCellDisplacementYoungRight = pressureStressRight * (originalLengthZero / youngModulus);

            // Resulting volume under the right foot after displacement
            volumeTotalRight = areaTotalRight * (originalLengthZero + (-heightCellDisplacementYoungRight));

            // Calculate the difference in volume, takes into account the compressibility and estimate volume up per neighbour cell
            volumeDifferenceRight = volumeTotalRight - volumeOriginalRight; // NEGATIVE CHANGE
            volumeNetDifferenceRight = -volumeDifferenceRight + volumeVariationPoissonRight; // Calculate directly the volume in the bump upwards (positive)

            // Distribute volume
            volumeCellRight = volumeNetDifferenceRight / neighbourCellsRight;

            // 2. In this case, we do it with volume. Remember: must be negative for later.
            bumpHeightDeformationRight = volumeCellRight / areaCell;
        }

        // 3. Given the entire deformation in Y, we calculate the corresponding frame-based deformation based on the frame-time.
        displacementLeft = (Time.deltaTime * (float)heightCellDisplacementYoungLeft) / ContactTime;
        displacementRight = (Time.deltaTime * (float)heightCellDisplacementYoungRight) / ContactTime;

        // Given the  deformation in Y for the bump, we calculate the corresponding frame-based deformation based on the frame-time.
        bumpDisplacementLeft = (Time.deltaTime * (float)bumpHeightDeformationLeft) / ContactTime;
        bumpDisplacementRight = (Time.deltaTime * (float)bumpHeightDeformationRight) / ContactTime;

        #endregion

        #region Physics+ Calculation

        // Strains (compression) - Info
        strainLong = -(heightCellDisplacementYoungRight) / originalLengthZero;
        strainTrans = poissonR * strainLong;

        // 1. If Poisson is 0.5 : ideal imcompressible material (no change in volume) - Compression : -/delta_L
        volumeVariationPoissonLeft = (1 - 2 * poissonR) * (-heightCellDisplacementYoungLeft / originalLengthZero) * volumeOriginalLeft; // NEGATIVE CHANGE
        volumeVariationPoissonRight = (1 - 2 * poissonR) * (-heightCellDisplacementYoungRight / originalLengthZero) * volumeOriginalRight;

        #endregion

        #region Apply Deformation

        // 2D iteration Deformation
        // Once we have the displacement, we saved the actual result of applying it to the terrain (only when the foot is grounded)
        if (IsLeftFootGrounded)
        {
            StartCoroutine(DecreaseTerrain(false, heightMapLeft, heightMapLeftBool, weightsBumpLeft, xLeft, zLeft));
        }
        else if(!IsLeftFootGrounded)
        {
            // Every time we lift the foot, we reset the variables and stop the coroutines.
            heightCellDisplacementYoungLeft = 0;
            StopAllCoroutines();
        }

        if (IsRightFootGrounded)
        {
            StartCoroutine(DecreaseTerrain(true, heightMapRight, heightMapRightBool, weightsBumpRight, xRight, zRight));
        }
        else if(!IsRightFootGrounded)
        {
            heightCellDisplacementYoungRight = 0;
            StopAllCoroutines();
        }

        #endregion
    }
    
    IEnumerator DecreaseTerrain(bool right, float[,] heightMap, int[,] heightMapBool, float[,] weightsBump, int x, int z)
    {
        Collider footCollider = right ? RightFootCollider : LeftFootCollider;
        float bumpDisplacement = right ? bumpDisplacementRight : bumpDisplacementLeft;
        double heightCellDisplacementYoung = right ? heightCellDisplacementYoungRight : heightCellDisplacementYoungLeft;
        float displacement = right ? displacementRight : displacementLeft;
        double bumpHeightDeformation = right ? bumpHeightDeformationRight : bumpHeightDeformationLeft;
        // Debug.Log("Height total deformation " + bumpHeightDeformation);
        // 1. Apply frame-per-frame deformation ("displacement")
        for (int zi = -gridSize; zi <= gridSize; zi++)
        {
            for (int xi = -gridSize; xi <= gridSize; xi++)
            {
                // A. Calculate each cell position wrt World and Heightmap - Left Foot
                Vector3 rayGrid = new Vector3(x + xi, terrain.Get(x + xi, z + zi), z + zi);
                Vector3 rayGridWorld = terrain.Grid2World(rayGrid);

                // B. Create each ray for the grid (wrt World) - Left
                RaycastHit footHit;
                Ray upRayFoot = new Ray(rayGridWorld, Vector3.up);

                // C. If hits the Left Foot and the cell was classified with 2 (direct contact) or 1 (countour):
                if (footCollider.Raycast(upRayFoot, out footHit, raycastDistance) && (heightMapBool[zi + gridSize, xi + gridSize] == 2))
                {
                    // D. Cell contacting directly - Decrease until limit reached
                    if (terrain.Get(rayGrid.x, rayGrid.z) >= terrain.GetConstant(rayGrid.x, rayGrid.z) - heightCellDisplacementYoung)
                    {
                        // E. Substract
                        heightMap[zi + gridSize, xi + gridSize] = terrain.Get(rayGrid.x, rayGrid.z) - (displacement);
                    }
                    else
                    {
                        // F. Keep same
                        heightMap[zi + gridSize, xi + gridSize] = terrain.Get(rayGrid.x, rayGrid.z);
                    }
                }
                // If the cell if a contour and bump should be apply
                else if (!footCollider.Raycast(upRayFoot, out footHit, raycastDistance) && (heightMapBool[zi + gridSize, xi + gridSize] == 1) && applyBumps)
                {
                    if(terrain.Get(rayGrid.x, rayGrid.z) <= terrain.GetConstant(rayGrid.x, rayGrid.z) + bumpHeightDeformation * (weightsBump[zi + gridSize, xi + gridSize]))
                    {
                        heightMap[zi + gridSize, xi + gridSize] = (terrain.Get(rayGrid.x, rayGrid.z) + (bumpDisplacement));
                    }
                    else
                    {
                        heightMap[zi + gridSize, xi + gridSize] = terrain.Get(rayGrid.x, rayGrid.z); 
                    }
                }
                else
                {
                    // J. If is out of reach
                    heightMap[zi + gridSize, xi + gridSize] = terrain.Get(rayGrid.x, rayGrid.z);
                }
            }
        }

        // 2. Applying filtering in frame-basis
        if (!right && applyFilterLeft)
        {
            if (IsLeftFootGrounded && !IsRightFootGrounded)
            {
                if (!isFilteredLeft)
                {
                    heightMap = NewFilterHeightMapReturn(x, z, heightMap);
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
        else if (right && applyFilterRight)
        {
            if (IsRightFootGrounded && !IsLeftFootGrounded)
            {
                if (!isFilteredRight)
                {
                    heightMap = NewFilterHeightMapReturn(x, z, heightMap);
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

        // 3. Save terrain
        if (applyFootprints)
        {
            for (int zi = -gridSize; zi <= gridSize; zi++)
            {
                for (int xi = -gridSize; xi <= gridSize; xi++)
                {
                    Vector3 rayGrid = new Vector3(x + xi, terrain.Get(x + xi, z + zi), z + zi);
                    terrain.Set(rayGrid.x, rayGrid.z, heightMap[zi + gridSize, xi + gridSize]);
                }
            }
        }

        yield return null;
    }
    
    #endregion

    #region Post-filter Methods

    // New-version Gaussian Blur (3x3) with return 
    public float[,] NewFilterHeightMapReturn(int x, int z, float[,] heightMap)
    {
        float[,] heightMapFiltered = new float[2 * gridSize + 1, 2 * gridSize + 1];

        // Places outside filtering will remain the same
        heightMapFiltered = heightMap;

        for (int zi = -gridSize + marginAroundGrid; zi <= gridSize - marginAroundGrid; zi++)
        {
            for (int xi = -gridSize + marginAroundGrid; xi <= gridSize - marginAroundGrid; xi++)
            {
                Vector3 rayGridLeft = new Vector3(x + xi, terrain.Get(x + xi, z + zi), z + zi);

                heightMapFiltered[zi + gridSize, xi + gridSize] =
                    0 * heightMap[zi + gridSize - 1, xi + gridSize - 1]
                    + 0 * heightMap[zi + gridSize - 1, xi + gridSize]
                    + 5 * heightMap[zi + gridSize - 1, xi + gridSize + 1]
                    + 0 * heightMap[zi + gridSize, xi + gridSize - 1]
                    + 1 * heightMap[zi + gridSize, xi + gridSize]
                    + 5 * heightMap[zi + gridSize, xi + gridSize + 1]
                    + 0 * heightMap[zi + gridSize + 1, xi + gridSize - 1]
                    + 0 * heightMap[zi + gridSize + 1, xi + gridSize]
                    + 5 * heightMap[zi + gridSize + 1, xi + gridSize + 1];

                heightMapFiltered[zi + gridSize, xi + gridSize] *= 1.0f / 11.0f;
            }
        }

        return heightMapFiltered;
    }

    #endregion
    
}
