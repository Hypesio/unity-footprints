using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class PhysicalFootprintWeights 
{
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
    public static float[,] UpdateWeights(float[,] weightsBump, int[,] heightMapBool, int gridSize, int offsetBumpGrid)
    {
        // Just the half back of the grid
        // TODO Take the number of cell in the contour to preserve volume moved
        // TODO Use the speed vector to determine the value of each cell
        for (int i = -gridSize + offsetBumpGrid; i < gridSize - offsetBumpGrid; i++)
        {
            for (int j = -gridSize + offsetBumpGrid; j < gridSize - offsetBumpGrid; j++)
            {
                // If we are on a contour we increase the weight
                if (heightMapBool[i + gridSize, j + gridSize] == 1)
                {
                    // Value if arbitrary for the moment
                    if (j < 0)
                        weightsBump[i + gridSize, j + gridSize] *= 3f;    
                    else 
                        weightsBump[i + gridSize, j + gridSize] /= 3f;
                    
                }
            }
        }

        return weightsBump;
    }
}
