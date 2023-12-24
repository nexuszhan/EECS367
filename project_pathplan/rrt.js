/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

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

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree

    /* var q_rand = randomConfig();
    
    var state;
    var q_new;
    [state, q_new] = extend(T_a, q_rand);
    
    if (state === "Trapped")
    {
        var tmp = Object.assign({}, T_a);
        Object.assign(T_a, T_b);
        Object.assign(T_b, tmp);
    }
    else
    {
        if (connect(T_b, q_new) === "Reached")
        {
            var path = findPath(T_b, q_new);
            drawHighlightedPath(path);
            return "succeeded";
        }
        else
            return "failed";
    } 

    return "extended"; */
    if (search_iter_count > search_max_iterations)
    {
        search_iterate = false;
        return "failed";
    }

    q_rand = randomConfig();

    var nearest_index = findNearest(T_a, q_rand);
    var q_near = T_a.vertices[nearest_index].vertex;

    var q_new = newConfig(q_rand, q_near);

    if (!testCollision(q_new))
    {
        insertTreeVertex(T_a, q_new);
        insertTreeEdge(T_a, nearest_index, T_a.newest);
        search_visited++;

        T_a.vertices[T_a.newest].parent = nearest_index; //

        var nearest_index2 = findNearest(T_b, q_new);
        var q_near2 = T_b.vertices[nearest_index2].vertex;

        var q_new2 = newConfig(q_new, q_near2);

        if (!testCollision(q_new2))
        {
            insertTreeVertex(T_b, q_new2);
            insertTreeEdge(T_b, nearest_index2, T_b.newest);
            search_visited++;

            T_b.vertices[T_b.newest].parent = nearest_index2; //

            do
            {
                var q_new3 = newConfig(q_new, q_new2);
                if (!testCollision(q_new3))
                {
                    insertTreeVertex(T_b, q_new3);
                    insertTreeEdge(T_b, T_b.newest-1, T_b.newest);
                    search_visited++;

                    T_b.vertices[T_b.newest].parent = T_b.newest-1; //

                    q_new2 = q_new3;
                }
                else
                {
                    //search_iterate = false;
                    //return "failed";
                    break;
                }
            } while (!(Math.abs(q_new[0]-q_new2[0]) <= eps/2 && Math.abs(q_new[1]-q_new2[1]) <= eps/2))
        }

        if (Math.abs(q_new[0]-q_new2[0]) <= eps/2 && Math.abs(q_new[1]-q_new2[1]) <= eps/2)
        {
            /* var path = findPath(T_b, q_new);
            drawHighlightedPath(path); */
            var path = [T_a.vertices[T_a.newest], T_b.vertices[T_b.newest]];
            var cur_node = Object.assign({}, T_a.vertices[T_a.newest]);

            while (typeof cur_node.parent != "undefined")
            {
                cur_node = Object.assign({}, T_a.vertices[cur_node.parent]);
                path.unshift(cur_node);
            }

            cur_node = Object.assign({}, T_b.vertices[T_b.newest]);
            while (typeof cur_node.parent != "undefined")
            {
                cur_node = Object.assign({}, T_b.vertices[cur_node.parent]);
                path.push(cur_node);
            }
            
            drawHighlightedPath(path);
            path_length = eps * path.length;
            search_iterate = false;
            return "succeeded";
        }
    }

    if (T_b.vertices.length < T_a.vertices.length)
    {
        var tmp = Object.assign({}, T_a);
        Object.assign(T_a, T_b);
        Object.assign(T_b, tmp);
    }
    search_iter_count++;
    
    return "extended";
}

function iterateRRTStar() {

}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath

function randomConfig()
{
    var xMin = -1; //-2;  // Minimum value of x coordinate
    var xMax = 5; //7;  // Maximum value of x coordinate
    var yMin = -1 //-2;  // Minimum value of y coordinate
    var yMax = 5 //7;  // Maximum value of y coordinate

    var xRand = Math.random() * (xMax - xMin) + xMin;  // Random value for x coordinate
    var yRand = Math.random() * (yMax - yMin) + yMin;  // Random value for y coordinate

    return [xRand, yRand]; 
}

function findNearest(tree, q)
{
    var i;
    var nearest_index = -1;
    var nearest_dist = Infinity;
    for (i=0; i<tree.vertices.length; i++)
    {
        var vertex = tree.vertices[i].vertex;
        var dist = Math.sqrt((q[0]-vertex[0])*(q[0]-vertex[0]) + (q[1]-vertex[1])*(q[1]-vertex[1]));
        if (dist < nearest_dist)
        {
            nearest_index = i;
            nearest_dist = dist;
        }
    }
    return nearest_index;
}

function newConfig(q, q_near)
{
    var dx = q[0] - q_near[0];
    var dy = q[1] - q_near[1];
    var theta = Math.atan2(dy, dx);
    
    var q_new = [q_near[0] + eps * Math.cos(theta), q_near[1] + eps * Math.sin(theta)];

    return q_new;
}

function extend(tree, q)
{
    var nearest_index = findNearest(tree, q);
    //console.log(nearest_index);
    var q_near = tree.vertices[nearest_index].vertex;
    //console.log(q_near);
    
    var dx = q[0] - q_near[0];
    var dy = q[1] - q_near[1];
    var theta = Math.atan2(dy, dx);
    
    var q_new = [q_near[0] + 0.1*Math.cos(theta), q_near[1] + 0.1*Math.sin(theta)];
    //console.log(q_new);

    if (!testCollision(q_new))
    {
        insertTreeVertex(tree, q_new);
        insertTreeEdge(tree, nearest_index, tree.newest);

        tree.vertices[tree.newest].parent = nearest_index; //
    }
    else
        return ["Trapped", q_new];

    if (Math.abs(q_new[0]-q[0]) <= eps/2 && Math.abs(q_new[1]-q[1]) <= eps/2)
        return ["Reached", q_new];
    else
        return ["Advanced", q_new];
}

function connect(tree, q)
{
    var state;
    do
    {
        state = extend(tree, q);
    }
    while (state === "Advanced")
    return state;
}

function findPath(tree, q)
{
    var path = [q];
    var cur_node = Object.assign({}, tree.vertices[tree.newest]);
    while (typeof cur_node.parent != "undefined")
    {
        cur_node = Object.assign({}, tree.vertices[cur_node.parent]);
        path.unshift(cur_node.vertex);
    }
    
    return path;
}