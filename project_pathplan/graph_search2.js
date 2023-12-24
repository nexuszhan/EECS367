/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function initSearchGraph() {

    // create the search queue
    visit_queue = [];

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };

            // STENCIL: determine whether this graph node should be the start
            //   point for the search
            if (xpos === q_init[0] && ypos === q_init[1])
            {
                G[iind][jind].distance = 0;
                G[iind][jind].priority = 0;
                G[iind][jind].queued = true;
                minheap_insert(visit_queue, [iind,jind]);
                draw_2D_configuration([xpos,ypos], "queued");
            }
        }
    }
}

function iterateGraphSearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   When search is complete ("failed" or "succeeded") set the global variable 
    //   search_iterate to false. 
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location
    if (visit_queue.length === 0)
    {
        search_iterate = false;
        return "failed";
    }
    var cur_pos = minheap_extract(visit_queue);
    var cur_node = G[cur_pos[0]][cur_pos[1]];
    if (cur_node.x === q_goal[0] && cur_node.y === q_goal[1])
    {
        drawHighlightedPathGraph(cur_node);
        search_iterate = false;
        return "succeeded";
    }
    cur_node.visited = true;
    draw_2D_configuration([cur_node.x,cur_node.y],"visited");
    var steps = [[-1,0],[1,0],[0,-1],[0,1]];
    var i;
    for (i=0; i<4; i++)
    {
        var new_iind = cur_node.iind + steps[i][0];
        var new_jind = cur_node.jind + steps[i][1];
        if (new_iind>=0 && new_iind<=800 && new_jind>=0 && new_jind<=800)
        {
            var new_node = G[new_iind][new_jind];
            if (!new_node.visited && !new_node.queued && !testCollsion([new_node.x,new_node.y]))
            {
                new_node.parent = cur_node;
                new_node.distance = cur_node.distance + eps;
                var line_dist_square = (q_goal[0]-new_node.x)*(q_goal[0]-new_node.x) + (q_goal[1]-new_node.y)*(q_goal[1]-new_node.y);
                new_node.priority = cur_node.distance + eps + Math.sqrt(line_dist_square);
                new_node.queued = true;
                minheap_insert(visit_queue, new_node);
                draw_2D_configuration([new_node.x,new_node.y],"queued");
            }
        }
    }
    return "iterating";
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    heap.push(new_element);
    var cur = heap.length-1;
    var pos1 = heap[cur];
    var pos2 = heap[Math.floor((cur-1)/2)];
    while ((cur>0) && G[pos1[0]][pos1[1]].priority < G[pos2[0]][pos2[1]].priority)
    {
        heap[cur] = pos2;
        heap[Math.floor((cur-1)/2)] = pos1;
        cur = Math.floor((cur-1)/2);
    }
}
    
    // define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    var top = heap[0];
    var last = heap.pop();
    if (heap.length === 1)
        return top;
    heap[0] = last;
    var i = 0;
    while ((2*i+1) <= heap.length)
    {
        var j = 2*i+1;
        if (j < heap.length && G[heap[j][0]][heap[j][1]].priority > G[heap[j+1][0]][heap[j+1][1]].priority)
            j++;
        if (G[heap[i][0]][heap[i][1]].priority <= G[heap[j][0]][heap[j][1]].priority)
            break;
        var tmp = heap[i];
        heap[i] = heap[j];
        heap[j] = tmp;
        i = j;
    }

    return top;
}   
