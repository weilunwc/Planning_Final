/*
 RRT.cpp
 
 by Peter Jan, Kyungzun Rim
 Nov. 20 2018
 */

#include "RRT.h"

RRT::RRT(double* map,int xSize,int ySize,int zSize,double* start,double* goal)
{
    map_ = map;
    xSize_ = xSize;
    ySize_ = ySize;
    zSize_ = zSize;
    start_ = start;
    goal_ = goal;
    int numofDOFs = 3; //xSize ySize zSize
    
    class Node_RRT
    {
    public:
        vector<double> node_from;
        vector<double> node_to;
        double* pos = 0;
        int group;
        int index;
    };
    
    
    void get_current_point(int x, int *y)
    {
        if (params->UsingYIndex)
        {
            *y = params->XIndex;
            *x = params->YIndex;
            if (params->Flipped)
                *x = -*x;
        }
        else
        {
            *x = params->XIndex;
            *y = params->YIndex;
            if (params->Flipped)
                *y = -*y;
        }
    }
    
    
    
    int get_next_point(bresenham_param_t *params)
    {
        if (params->XIndex == params->X2)
        {
            return 0;
        }
        
        params->XIndex += params->Increment;
        if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
            params->DTerm += params->IncrE;
        else
        {
            params->DTerm += params->IncrNE;
            params->YIndex += params->Increment;
        }
        
        return 1;
        
    }
    
    // discuss - The function to check whether interpolation collides or not
    int IsValidLineSegment(double x0, double y0, double x1, double y1, double*    map,
                           int x_size,
                           int y_size)
    {
        
        bresenham_param_t params;
        int nX, nY;
        short unsigned int nX0, nY0, nX1, nY1;
        //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
        
        //make sure the line segment is inside the environment
        if(x0 < 0 || x0 >= x_size ||
           x1 < 0 || x1 >= x_size ||
           y0 < 0 || y0 >= y_size ||
           y1 < 0 || y1 >= y_size)
            return 0;
        
        
        
        ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
        ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);
        //iterate through the points on the segment
        get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
        do {
            get_current_point(&params, &nX, &nY);
            if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
                return 0;
        } while (get_next_point(&params));
        return 1;
    }
    
    
    //discuss - We have to check the map information to check whether it is valid
    int IsValidinConfiguration(double* positions, int numofDOFs, double*map,int x_size, int y_size)
    {
        double x0,y0,x1,y1;
        int i;
        
        //iterate through all the links starting with the base
        x1 = ((double)x_size)/2.0;
        y1 = 0;
        for(i = 0; i < numofDOFs; i++)
        {
            //compute the corresponding line segment
            x0 = x1;
            y0 = y1;
            x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-positions[i]);
            y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-positions[i]);
            //check the validity of the corresponding line segment
            if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
                return 0;
        }
        return 1;
        
    }
    
    // []--copy to ---> [[  ][  ][  ]...[  ][  ][  ][  ].....[  ][  ][  ]]
    void copytolist(double* positions,double* object_list,int row, int numofDOFs)
    {
        for(int f_1 = 0; f_1 < numofDOFs; f_1 ++)
        {
            object_list[row * numofDOFs + f_1] = positions[f_1];
        }
        return;
        
    }
    
    //Euclidian Distance (discuss when we add the battery or the angle
    //-> we can make more functions)
    double Distance(double* first, double* second,int numofDOFs)
    {
        double dist_mid1 = 0;
        double dist_mid2 = 0;
        for(int l = 0; l<numofDOFs; l++)
        {
            dist_mid1 = sqrt((first[l]-second[l])*(first[l]-second[l]));
            dist_mid2 += dist_mid1;
        }
        return dist_mid2;
    }
    
    
    
    static void planner(
                        double* map,
                        int xSize,
                        int ySize,
                        int zSize,
                        double* start,
                        double* goal,
                        int numofDOFs,)
    
    {
        /***********************1 make the variables****************************/
        //no plan by default
        double*** plan = NULL;
        int* planlength = 0;
        
        int numofDOFs = 3; //xSize ySize zSize
        
        // length of nodes in the list
        int K =40;
        int Numofrandpoints = 300;
        double position_state_rrt[numofDOFs];
        
        //instead of using 2 dimension(time and position), I use 1-Dim list
        // Numofrandpoints -> just for the maximum case
        Node_RRT Treelist_rrt[K+2]; // to add startQ and goalQ
        
        double positions_path_rrt[numofDOFs * (Numofrandpoints+1)];
        double sample_positions[numofDOFs * (Numofrandpoints+1)];
        
        
        double dist_rrt = 0;
        double epsilon = 0.2;   //discuss
        bool collisionok =1;
        bool collisionok2 =1;
        int count_tree = 0;
        int num_interpole = 4;
        
        // Save the start to Treelist & positions_path_rrt
        Treelist_rrt[0].state = start;
        count_tree ++; // count the number of data set in the TreeList
        
        //TODO  I have to save the goal in the final step
        copytolist(startQ,positions_path_rrt,0, numofDOFs,);
        
        /*
         * 1. Make random points
         *
         * 2. Choose one random point and find the distance
         *    between the point and points in poistions_tree_rrt
         *
         * 3. Choose the closest one and check whether distance
         *    longer than epsilon
         *
         * 4. Set the previous and next in the nodes_list_rrt
         *
         *
         *
         *
         */
        
        while(count_tree < K)
        {
            //1. make Random config # : Numofrandpoints
            for(int i = 0; i < Numofrandpoints; i++)
            {
                bool collision = 1;
                while(collision)
                {
                    //discuss whether we should make the value of position integer
                    //maked the sample point (X,Y,Z)
                    position_state_rrt[0] = fmod(rand(),max_X);
                    position_state_rrt[1] = fmod(rand(),max_Y);
                    position_state_rrt[2] = fmod(rand(),max_Z);
                    
                    //check collision
                    collision = !(IsValidArmConfiguration(position_state_rrt, numofDOFs, map, x_size, y_size));
                }
                //Find the free space point
                // copy new sample point to the list named sample_poistions
                
                copytoposition_list(position_state_rrt,i, numofDOFs,sample_positions); //save random points in the list
                
            }
            
            
            vector<double> Dists_rrt;
            
            //for each sample point, find the distance and save it in Dists_rrt
            for(int i =0; i<Numofrandpoints; i++)
            {
                vector<double> Dists_rrt;
                for(int j =0; j<count_tree; j++)
                {
                    dist_rrt = Distance(&(positions_path_rrt[j]),&(sample_positions[i]),numofDOFs);
                    Dists_rrt.push_back(dist_rrt);
                }
                
                int min_node_mid = distance(Dists_rrt.begin(),min_element(Dists_rrt.begin(),Dists_rrt.end()));
                
                
                // When the minimum distance is bigger than epsilon extend by epsilon
                if(Dists_rrt.at(min_node_mid)>epsilon)
                {
                    
                    double tmp_dist = 0;
                    double tmp_dist1 = 0;
                    for(int m=0; m < numofDOFs; m++)
                    {
                        //find the distance between the closest node in the treee and node i
                        tmp_dist = positions_tree_rrt[min_node_mid * numofDOFs + m] - sample_positions[i*numofDOFs + m];
                        tmp_dist1 = fabs(tmp_dist);
                        
                        //for x,y,z elements, extend by size epsilon from the closest node in the tree
                        position_state_rrt[m]= positions_tree_rrt[min_node_mid * numofDOFs + m] + tmp_dist / tmp_dist1 * epsilon;
                    }
                }
                else
                {
                    for(int m=0; m < numofDOFs; m++)
                    {
                        position_state_rrt[m]= sample_positions[min_node_mid * numofDOFs + m];
                    }
                    
                }
                /*
                 final_path_rrt[count_tree].state = position_state_rrt; // to add startQ and goalQ
                 */
                // If the extended position is not valid go to next point rather than finding the trapped point.
                if(!(IsValidmapConfiguration(position_state_rrt, numofDOFs, map, x_size, y_size))) continue;
                
                // count_tree == number of explored random points <- used as index of the list
                count_tree ++ ;
                
                // discuss save the found point to the path list
                for(int f_1 = 0; f_1 < numofDOFs; f_1 ++)
                {
                    
                    positions_tree_rrt[count_tree * numofDOFs + f_1] = position_state_rrt[f_1];
                    
                }
                
                collisionok = collisioncheck(&(angles_path_rrt[count_tree *numofDOFs]),goalQ,numofDOFs,map,x_size,y_size,num_interpole);
                
                // if there is no obstacles between extended point and goal point we can add the point and finish finding the path
                if(collisionok)
                {
                    copytopos_list_rrt(goalQ,count_tree, numofDOFs,positions_path_rrt);
                    cout<<"point9"<<endl;
                    break;
                }
                
            }
            
        }
        // save the planner in the tree -> change
        *plan = (double**) malloc(100*sizeof(double*));
        for(int i = 0; i<count_tree+1 ; i++)
        {
            (*plan)[i] = &(positions_path_rrt[i * numofDOFs]);
        }
        
        *planlength = count_tree+1 ;
        
        return;
        
    }
    
}

RRT::~RRT()
{
    //Might want to do something here later if we allocate on heap
}

void RRT::plan()
{
    
}

void RRT::connect(double* sample)
{
    
}

void RRT::nearestNeighbor(double* sample)
{
    
}

void RRT::extend(double* sample)
{
    
}
