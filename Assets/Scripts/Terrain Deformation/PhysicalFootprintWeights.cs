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
    private static bool moveArrow(int[,] heightBoolMap, ref float[,] grid, int minGrid, int maxGrid,
        ref int[,] gridWays, ref bool[] passFeet, int arrowIndex, int speed, bool yDirection, float speedForce)
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
                passFeet[arrowIndex] = true;
                //grid[y, x] = 1f;
            }
            // If we are on a contour
            else if (heightBoolMap[y, x] == (int)PhysicalFootprint.CellState.Contour)
            {
                // TODO Static useless value for the moment on the grid
                if (passFeet[arrowIndex])
                {
                    grid[y, x] = 2f; //1f + speedForce;
                }
                else
                {
                    grid[y, x] = 0f; //Mathf.Max(1f - speedForce, 0);
                }
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
        float sum = 0;
        for (int i = minGrid; i < maxGrid; i++)
        {
            for (int j = minGrid; j < maxGrid; j++)
            {
                if (weightsBump[i, j] > 0)
                {
                    sum += weightsBump[i, j];
                }
            }
        }

        float ratio = sum / (nbNeighboor); 
        
        for (int i = minGrid; i < maxGrid; i++)
        {
            for (int j = minGrid; j < maxGrid; j++)
            {
                if (weightsBump[i, j] > 0)
                {
                    weightsBump[i, j] /= ratio; 
                }
            }
        }

        return weightsBump;
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
    

    public static float[,] UpdateWeightsUsingSpeed(float[,] weightsBump, int[,] heightMapBool, int minGrid, int maxGrid,
        Vector3 speed, int nbNeighboor)
    {
        // TODO find the right value
        float speedForce = speed.magnitude * 1f; 
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
                    gridWaysStopped[arrowIndex] = moveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                        ref gridWays, ref gridWaysPassFeet, arrowIndex, ySpeed, true,speedForce);
                    if (!gridWaysStopped[arrowIndex])
                    {
                        gridWaysStopped[arrowIndex] = moveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                            ref gridWays, ref gridWaysPassFeet, arrowIndex, xSpeed, false, speedForce);
                    }

                    touchEnd = gridWaysStopped[arrowIndex] ? touchEnd + 1 : touchEnd;
                }
                else
                {
                    // Move in x direction than in y
                    gridWaysStopped[arrowIndex] = moveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                        ref gridWays, ref gridWaysPassFeet, arrowIndex, xSpeed, false, speedForce);
                    if (!gridWaysStopped[arrowIndex])
                        gridWaysStopped[arrowIndex] = moveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
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
    }
    
        public static float[,] UpdateWeightsUsingFloor(float[,] weightsBump, int[,] heightMapBool, int gridSize,
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
        
        Debug.Log("Speed for grid x/z: " + xSpeed + "/" + ySpeed);
        
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
                    gridWaysStopped[arrowIndex] = moveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                        ref gridWays, ref gridWaysPassFeet, arrowIndex, ySpeed, true,speedForce);
                    if (!gridWaysStopped[arrowIndex])
                    {
                        gridWaysStopped[arrowIndex] = moveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                            ref gridWays, ref gridWaysPassFeet, arrowIndex, xSpeed, false, speedForce);
                    }

                    touchEnd = gridWaysStopped[arrowIndex] ? touchEnd + 1 : touchEnd;
                }
                else
                {
                    // Move in x direction than in y
                    gridWaysStopped[arrowIndex] = moveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
                        ref gridWays, ref gridWaysPassFeet, arrowIndex, xSpeed, false, speedForce);
                    if (!gridWaysStopped[arrowIndex])
                        gridWaysStopped[arrowIndex] = moveArrow(heightMapBool, ref weightsBump, minGrid, maxGrid,
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
    }

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
