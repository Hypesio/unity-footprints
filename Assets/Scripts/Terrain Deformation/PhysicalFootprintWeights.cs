using UnityEngine;

public static class PhysicalFootprintWeights 
{
    // Setup the weights value for the grid 
    public static float[,] SetupWeights(int gridSize, int offsetBumpGrid, int[,] heightMapBool)
    {
        float[,] weightsBump = new float[2 * gridSize + 1, 2 * gridSize + 1];
        
        for (int zi = -gridSize + offsetBumpGrid; zi <= gridSize - offsetBumpGrid; zi++)
        {
            for (int xi = -gridSize + offsetBumpGrid; xi <= gridSize - offsetBumpGrid; xi++)
            {
                if (heightMapBool[zi + gridSize, xi + gridSize] == 1)
                {
                    weightsBump[zi + gridSize, xi + gridSize] = 1f;
                }
            }
        }

        return weightsBump;
    }

    // Progress on the grid on a direction, update grid and return 'true' if reach the end of the grid
    private static bool MoveArrow(int[,] heightBoolMap, ref float[,] grid, int minGrid, int maxGrid,
        ref int[,] gridWays, ref int[] passFeet, int arrowIndex, int speed, bool yDirection, float speedForce,
        float[] valuesDeformation)
    {
        int x = gridWays[arrowIndex, 1];
        int y = gridWays[arrowIndex, 0];
        int xyTested = yDirection ? y : x;
        int startTested = yDirection ? y : x;
        
        while (xyTested != startTested + speed && xyTested >= minGrid && xyTested < maxGrid)
        {
            // If we are on the feet
            if (heightBoolMap[y, x] == (int)PhysicalFootprint.CellState.Contact)
            {
                passFeet[arrowIndex] = 0;
                grid[y, x] = 0;
            }
            // If we are after the feet in the speed direction
            else if (passFeet[arrowIndex] != -1 && passFeet[arrowIndex] < valuesDeformation.Length)
            {
                grid[y, x] += valuesDeformation[passFeet[arrowIndex]++]; //Mathf.Min(1f + speedForce, 2f);
            }

            if (yDirection)
                y = speed > 0 ? y + 1 : y - 1;
            else
                x = speed > 0 ? x + 1 : x - 1;
            xyTested = yDirection ? y : x;
        }

        gridWays[arrowIndex,0] = yDirection ? startTested + speed : y; 
        gridWays[arrowIndex,1] = yDirection ? x : startTested + speed; 
        
        return xyTested < minGrid || xyTested >= maxGrid;
    }

    // Pass on the grid after the main process is done. Will improve visual results
    private static float[,] PostTreatmentWeights(float[,] weightsBump, int minGrid, int maxGrid, int nbNeighboor)
    {
        // Keep the right volume
        int nbDeformedCells = 0;
        float sum = 0;
        for (int i = minGrid; i < maxGrid; i++)
        {
            for (int j = minGrid; j < maxGrid; j++)
            {
                if (weightsBump[i, j] > 0)
                {
                    sum += weightsBump[i, j];
                    nbDeformedCells++;
                }
            }
        }

        float ratio = (sum / nbDeformedCells);
        float added_ratio = DeformTerrainMaster.Instance.deformationVolumeMultiplicater;
        if (nbDeformedCells != 0 && nbNeighboor != 0)
            added_ratio *= (float)nbNeighboor / (float)nbDeformedCells;
        
        for (int i = minGrid; i < maxGrid; i++)
        {
            for (int j = minGrid; j < maxGrid; j++)
            {
                if (weightsBump[i, j] > 0)
                {
                    weightsBump[i, j] /= ratio;
                    weightsBump[i, j] *= added_ratio;
                }
            }
        }

        return weightsBump;
    }

    // Return an array of cell weights using the curve and length specified 
    private static float[] ComputeValuesDeformation(AnimationCurve curve, int len, float mutiplicator = 1)
    {
        float[] values = new float[len];
        float step = curve.keys[curve.length - 1].time / values.Length;
        for (int i = 0; i < values.Length; i++)
        {
            values[i] = curve.Evaluate(step * i) * mutiplicator;
        }
        return values;
    }
    
    // Will add weights around the feet depending on the curve specified in DeformTerrainMaster
    private static void DeformationAroundFeet(int[,] heightBoolMap, ref float[,] grid, int minGrid, int maxGrid, float importance)
    {
        float[] valuesDeformation = ComputeValuesDeformation(DeformTerrainMaster.Instance.borderDeformationShape,
            DeformTerrainMaster.Instance.borderLenDeformation, importance);

        int gridSize = maxGrid - minGrid;
        Vector2 feetCenter = new Vector2(gridSize / 2, gridSize / 2);
        // Iterate on the grid to search 
        for (int i = minGrid; i < maxGrid; i++)
        {
            for (int j = minGrid; j < maxGrid; j++)
            {
                // If we are on a border of the feet put weight around the cell
                if (heightBoolMap[i, j] == (int)PhysicalFootprint.CellState.Contour)
                {
                    Vector2 direction = (new Vector2(i, j) - feetCenter).normalized;
                    direction = GetVectorForGrid(direction);
                    int progress = 0;
                    Vector2 actualPos = new Vector2(i, j);
                    while (progress < valuesDeformation.Length && actualPos.x > minGrid && actualPos.x < maxGrid &&
                    actualPos.y > minGrid && actualPos.y < maxGrid)
                    {
                        for (int x = 0; x < Mathf.Abs(direction.x) && progress < valuesDeformation.Length; x++)
                        {
                            if (actualPos.x < minGrid || actualPos.x >= maxGrid)
                                break;
                            if (actualPos.y < minGrid || actualPos.y >= maxGrid)
                                break;
                            //useless
                            //grid[Mathf.RoundToInt(actualPos.x), 0] = 0;
                            //grid[0, Mathf.RoundToInt(actualPos.y)] = 0;
                            grid[Mathf.RoundToInt(actualPos.x), Mathf.RoundToInt(actualPos.y)] =
                                valuesDeformation[progress];
                            if (direction.x < 0)
                                actualPos.x--;
                            else
                                actualPos.x++;
                            progress++;
                        }

                        for (int y = 0; y < Mathf.Abs(direction.y) && progress < valuesDeformation.Length; y++)
                        {
                            if (actualPos.x < minGrid || actualPos.x >= maxGrid)
                                break;
                            if (actualPos.y < minGrid || actualPos.y >= maxGrid)
                                break;
                            
                            grid[Mathf.RoundToInt(actualPos.x), Mathf.RoundToInt(actualPos.y)] =
                                valuesDeformation[progress];
                            if (direction.y < 0)
                                actualPos.y--;
                            else
                                actualPos.y++;
                            progress++;
                        }
                        
                    }
                }
            }
        }
    }
        
    // **
    // Will progress on the weight grid in the direction of the speed vector and change weights value depending 
    // on speed magnitude and if the weights is before or after the feet in the speed direction
    // **
    public static float[,] UpdateWeightsUsingSpeed(float[,] weightsBump, int[,] heightMapBool, int gridSize,
        Vector3 speed, int nbNeighboor = 1)
    {
        int minGrid = 0;
        int maxGrid = 2 * gridSize + 1;
        return UpdateWeightsUsingSpeed(weightsBump, heightMapBool, minGrid, maxGrid, speed, nbNeighboor);
    }

    private static Vector2 GetVectorForGrid(Vector2 speed)
    {
        Vector2 res = new Vector2();
        if (Mathf.Abs(speed.x) > Mathf.Abs(speed.y))
        {
            res.x = Mathf.RoundToInt(speed.x / speed.y);
            if (speed.x < 0 && res.x > 0 || speed.x > 0 && res.x < 0)
                res.x = -res.x;
            res.y = 1; 
            if (speed.y < 0)
                res.y = -1; 
        }
        else
        {
            res.x = 1;
            if (speed.x < 0)
                res.x = -1;
            res.y = Mathf.RoundToInt(speed.y / speed.x);
            if (speed.y < 0 && res.y > 0 || speed.y > 0 && res.y < 0)
                res.y = -res.y;
        }

        return res;
    }

    public static float[,] UpdateWeightsUsingSpeed(float[,] weightsBump, int[,] heightMapBool, int minGrid, int maxGrid,
        Vector3 speed, int nbNeighboor)
    {
        float speedImportance = Mathf.Clamp(Vector3.Magnitude(speed) / DeformTerrainMaster.Instance.maxSpeedDeformation, 0.0f, 1.0f);
        Debug.Log("Speed importance " + speedImportance + "border importance " + (1.0f - speedImportance) + " speed " + speed);

        DeformationAroundFeet(heightMapBool, ref weightsBump, minGrid, maxGrid, 1.0f - speedImportance);
        if (speedImportance < 0.01)
        {
            // Speed is too much low to affect bump
            return weightsBump;
        }
        
        float[] valuesDeformation = ComputeValuesDeformation(DeformTerrainMaster.Instance.behindDeformationShape,
            DeformTerrainMaster.Instance.behindLenDeformation, speedImportance);
        
        float speedForce = Mathf.Clamp(speed.magnitude / 2.5f, 0, 1);
        int sizeGrid = maxGrid - minGrid;
        // Set the speed of the "arrows" in the grid
        Vector2 speedVector = GetVectorForGrid(new Vector2(speed.x, speed.z));
        int xSpeed = Mathf.RoundToInt(speedVector.x);
        int ySpeed = Mathf.RoundToInt(speedVector.y);

        // Arrows that will pass on the grid
        int[,] gridWays = new int[sizeGrid * 2, 2];
        bool[] gridWaysStopped = new bool[sizeGrid * 2];
        int[] gridWaysPassFeet = new int[sizeGrid * 2];
        
        // Set start position. Will be on two sides of the grid
        for (int i = 0; i < sizeGrid; i++)
        {
            gridWaysStopped[i] = false;
            gridWaysPassFeet[i] = -1;
            // Start at the beginning or at the end of a top / bottom side
            gridWays[i, 0] = ySpeed > 0 ? minGrid : maxGrid - 1;
            gridWays[i, 1] = i;
        }
        for (int i = sizeGrid; i < sizeGrid * 2; i++)
        {
            gridWaysStopped[i] = false;
            gridWaysPassFeet[i] = -1;
            // Start at the beginning or at the end of right / left side
            gridWays[i, 0] = i - sizeGrid;
            gridWays[i, 1] = xSpeed > 0 ? minGrid : maxGrid - 1;
        }
        
        // Iterate on all the grid
        int touchEnd = 0;
        int max_iter = 50;
        int iter = 0;
        int contourCount = 0; 
        while (touchEnd < sizeGrid * 2 && iter < max_iter)
        {
            // Iterate on all "arrows"
            for (int arrowIndex = 0; arrowIndex < sizeGrid * 2 && touchEnd < sizeGrid * 2; arrowIndex++)
            {
                if (gridWaysStopped[arrowIndex])
                    continue;
                
                // Start by moving in the strongest direction
                if (ySpeed > xSpeed)
                {
                    // Move in y direction than in x
                    gridWaysStopped[arrowIndex] = MoveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                        ref gridWays, ref gridWaysPassFeet, arrowIndex, ySpeed, true,speedForce, valuesDeformation);
                    if (!gridWaysStopped[arrowIndex])
                    {
                        gridWaysStopped[arrowIndex] = MoveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                            ref gridWays, ref gridWaysPassFeet, arrowIndex, xSpeed, false, speedForce, valuesDeformation);
                    }

                    touchEnd = gridWaysStopped[arrowIndex] ? touchEnd + 1 : touchEnd;
                }
                else
                {
                    // Move in x direction than in y
                    gridWaysStopped[arrowIndex] = MoveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                        ref gridWays, ref gridWaysPassFeet, arrowIndex, xSpeed, false, speedForce, valuesDeformation);
                    if (!gridWaysStopped[arrowIndex])
                        gridWaysStopped[arrowIndex] = MoveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                            ref gridWays, ref gridWaysPassFeet, arrowIndex, ySpeed, true, speedForce, valuesDeformation);
                    touchEnd = gridWaysStopped[arrowIndex] ? touchEnd + 1 : touchEnd;
                }
            }

            iter++;
        }
        
        if (iter >= max_iter)
            Debug.LogWarning("[UpdateWeights] REACH MAX ITERATION ON GRID");
        
        //PrintGrid(weightsBump, minGrid, maxGrid);
        //Debug.Log("After treatment");
        weightsBump = PostTreatmentWeights(weightsBump, minGrid, maxGrid, nbNeighboor);
        //PrintGrid(weightsBump, minGrid, maxGrid);
        return weightsBump;
    }
    
   /*     public static float[,] UpdateWeightsUsingFloor(float[,] weightsBump, int[,] heightMapBool, int gridSize,
        Vector3 speed, int nbNeighboor = 1)
    {
        int minGrid = 0;
        int maxGrid = 2 * gridSize + 1;
        return UpdateWeightsUsingFloor(weightsBump, heightMapBool, minGrid, maxGrid, speed, nbNeighboor);
    }
    

    public static float[,] UpdateWeightsUsingFloor(float[,] weightsBump, int[,] heightMapBool, int minGrid, int maxGrid,
        Vector3 speed, int nbNeighboor)
    {
        // TODO find the right value
        float speedForce = speed.y;
        //Debug.Log(speedForce);

        if (Vector3.Magnitude(speed) < 0.1)
        {
            // Speed is too much low to affect bump
            return weightsBump;
        }
        
        int xSpeed;
        int ySpeed;
        //PrintGrid(heightMapBool, minGrid, maxGrid);
        
        int sizeGrid = maxGrid - minGrid;
   
        // Set the speed of the "arrows" in the grid
        if (Mathf.Abs(speed.x) > Mathf.Abs(speed.z))
        {
            xSpeed = Mathf.RoundToInt(speed.x / speed.z);
            if (speed.x < 0 && xSpeed > 0 || speed.x > 0 && xSpeed < 0)
                xSpeed = -xSpeed;
            ySpeed = 1; 
            if (speed.z < 0)
                ySpeed = -1; 
        }
        else
        {
            xSpeed = 1;
            if (speed.x < 0)
                xSpeed = -1;
            ySpeed = Mathf.RoundToInt(speed.z / speed.x);
            if (speed.z < 0 && ySpeed > 0 || speed.z > 0 && ySpeed < 0)
                ySpeed = -ySpeed;
        }
        
        //Debug.Log("Speed for grid x/z: " + xSpeed + "/" + ySpeed);
        
        // Arrows that will pass on the grid
        int[,] gridWays = new int[sizeGrid * 2, 2];
        bool[] gridWaysStopped = new bool[sizeGrid * 2];
        bool[] gridWaysPassFeet = new bool[sizeGrid * 2];
        
        // Set start position. Will be on two sides of the grid
        for (int i = 0; i < sizeGrid; i++)
        {
            gridWaysStopped[i] = false;
            gridWaysPassFeet[i] = false;
            // Start at the beginning or at the end of a top / bottom side
            gridWays[i, 0] = ySpeed > 0 ? minGrid : maxGrid - 1;
            gridWays[i, 1] = i;
        }
        for (int i = sizeGrid; i < sizeGrid * 2; i++)
        {
            gridWaysStopped[i] = false;
            gridWaysPassFeet[i] = false;
            // Start at the beginning or at the end of right / left side
            gridWays[i, 0] = i - sizeGrid;
            gridWays[i, 1] = xSpeed > 0 ? minGrid : maxGrid - 1;
        }
        
        // Iterate on all the grid
        int touchEnd = 0;
        int max_iter = 50;
        int iter = 0;
        int contourCount = 0; 
        while (touchEnd < sizeGrid * 2 && iter < max_iter)
        {
            // Iterate on all "arrows"
            for (int arrowIndex = 0; arrowIndex < sizeGrid * 2 && touchEnd < sizeGrid * 2; arrowIndex++)
            {
                if (gridWaysStopped[arrowIndex])
                    continue;
                
                // Start by moving in the strongest direction
                if (ySpeed > xSpeed)
                {
                    // Move in y direction than in x
                    gridWaysStopped[arrowIndex] = MoveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                        ref gridWays, ref gridWaysPassFeet, arrowIndex, ySpeed, true,speedForce);
                    if (!gridWaysStopped[arrowIndex])
                    {
                        gridWaysStopped[arrowIndex] = MoveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                            ref gridWays, ref gridWaysPassFeet, arrowIndex, xSpeed, false, speedForce);
                    }

                    touchEnd = gridWaysStopped[arrowIndex] ? touchEnd + 1 : touchEnd;
                }
                else
                {
                    // Move in x direction than in y
                    gridWaysStopped[arrowIndex] = MoveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                        ref gridWays, ref gridWaysPassFeet, arrowIndex, xSpeed, false, speedForce);
                    if (!gridWaysStopped[arrowIndex])
                        gridWaysStopped[arrowIndex] = MoveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                            ref gridWays, ref gridWaysPassFeet, arrowIndex, ySpeed, true, speedForce);
                    touchEnd = gridWaysStopped[arrowIndex] ? touchEnd + 1 : touchEnd;
                }
            }

            iter++;
        }
        
        if (iter >= max_iter)
            Debug.LogWarning("[UpdateWeights] REACH MAX ITERATION ON GRID");
        
        //PrintGrid(weightsBump, minGrid, maxGrid);
        //Debug.Log("After treatment");
        weightsBump = PostTreatmentWeights(weightsBump, minGrid, maxGrid, nbNeighboor);
        //PrintGrid(weightsBump, minGrid, maxGrid);
        return weightsBump;
    }*/

    // [Debug] Print the specified grid in the console
    private static void PrintGrid(float[,] grid, int start, int end)
    {
        string line = ""; 
        for (int i = start; i < end; i++)
        {
            for (int j = start; j < end; j++)
            {
                line += grid[i, j].ToString("0.0") + " | ";
            }

            line += '\n';
        }
        Debug.Log(line);
    }

    public static void TestGrid(Vector3 speed, int gridSize, int[,] heightBoolMap)
    {
        if (Vector3.Magnitude(speed) < 0.001)
        {
            Debug.Log("Not Enough Speed, dont change grid contour");
            return;
        }

        float[,] grid = new float[gridSize, gridSize];
        
        grid = UpdateWeightsUsingSpeed(grid, heightBoolMap,0, gridSize, speed, 1);
        
        PrintGrid(grid, 0, gridSize);
    }
}
