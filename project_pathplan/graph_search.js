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
    //visit_queue = [];
    visit_queue = new AVLTree();

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
            if (Math.abs(xpos-q_init[0])<eps/2 && Math.abs(ypos-q_init[1])<eps/2)
            {
                G[iind][jind].distance = 0;
                G[iind][jind].priority = 0;
                G[iind][jind].queued = true;
                //minheap_insert(visit_queue, G[iind][jind]);
                visit_queue.insert(G[iind][jind]);
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
    //var cur_node = minheap_extract(visit_queue);
    var cur_node = visit_queue.pop();
    if (Math.abs(cur_node.x-q_goal[0])<eps/2 && Math.abs(cur_node.y-q_goal[1])<eps/2)
    {
        drawHighlightedPathGraph(cur_node);
        search_iterate = false;
        return "succeeded";
    }
    cur_node.visited = true;
    search_visited++;
    draw_2D_configuration([cur_node.x,cur_node.y],"visited");
    var steps = [[-1,0],[1,0],[0,-1],[0,1]];
    var i;
    for (i=0; i<4; i++)
    {
        var new_iind = cur_node.i + steps[i][0];
        var new_jind = cur_node.j+ steps[i][1];
        if (new_iind>=0 && new_iind<800 && new_jind>=0 && new_jind<800)
        {
            var new_node = G[new_iind][new_jind];
            if (!new_node.visited && !new_node.queued && !testCollision([new_node.x,new_node.y]))
            {
                new_node.parent = cur_node;
                new_node.distance = cur_node.distance + eps;
                var line_dist_square = (q_goal[0]-new_node.x)*(q_goal[0]-new_node.x) + (q_goal[1]-new_node.y)*(q_goal[1]-new_node.y);
                new_node.priority = cur_node.distance + eps + Math.sqrt(line_dist_square);
                new_node.queued = true;
                //minheap_insert(visit_queue, new_node);
                visit_queue.insert(new_node);
                draw_2D_configuration([new_node.x,new_node.y],"queued");
            }
        }
    }
    return "iterating";
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////
function minheap_insert(heap, new_element) {
    heap.push(new_element);
    var cur = heap.length-1;
    while ((cur>0) && heap[cur].priority < heap[Math.floor((cur-1)/2)].priority)
    {
        var tmp = heap[cur];
        heap[cur] = heap[Math.floor((cur-1)/2)];
        heap[Math.floor((cur-1)/2)] = tmp;
        cur = Math.floor((cur-1)/2);
    }
}

function minheap_extract(heap) {
    var top = heap[0];
    var last = heap.pop();
    if (heap.length === 0)
        return top;
    heap[0] = last;
    var i = 0;
    while ((2*i+1) < heap.length)
    {
        var j = 2*i+1;
        if (j < (heap.length-1) && heap[j].priority > heap[j+1].priority)
            j++;
        if (heap[i].priority <= heap[j].priority)
            break;
        var tmp = heap[i];
        heap[i] = heap[j];
        heap[j] = tmp;
        i = j;
    }

    return top;
}   


class AVLNode
{
    //constructor(value, priority)
    constructor(value)
    {
        this.value = value;
        this.priority = value.priority; //priority;
        this.height = 1;
        this.left = null;
        this.right = null;
    }
}

class AVLTree
{
    constructor() 
    {
        this.root = null;
    }
  
    // Helper function to get the height of a node
    getHeight(node) 
    {
        if (node === null) 
            return 0;
        return node.height;
    }
  
    // Helper function to get the balance factor of a node
    getBalance(node)
    {
        if (node === null)
            return 0;
        return this.getHeight(node.left) - this.getHeight(node.right);
    }
  
    // Helper function to update the height of a node
    updateHeight(node) 
    {
        node.height = 1 + Math.max(this.getHeight(node.left), this.getHeight(node.right));
    }
  
    // Helper function to perform a right rotation on a node
    rotateRight(node) 
    {
        const newRoot = node.left;
        const temp = newRoot.right;
        newRoot.right = node;
        node.left = temp;
        this.updateHeight(node);
        this.updateHeight(newRoot);
        return newRoot;
    }
  
    // Helper function to perform a left rotation on a node
    rotateLeft(node)
    {
        const newRoot = node.right;
        const temp = newRoot.left;
        newRoot.left = node;
        node.right = temp;
        this.updateHeight(node);
        this.updateHeight(newRoot);
        return newRoot;
    }
  
    // Helper function to insert a new node into the AVL tree
    insertHelper(node, value, priority)
    {
        // Perform standard BST insertion
        if (node === null) 
            return new AVLNode(value);//, priority);
        
        //var priority = this.priority;
        if (priority < node.priority) 
            node.left = this.insertHelper(node.left, value, priority);
        else 
            node.right = this.insertHelper(node.right, value, priority);
  
        // Update the height of the current node
        this.updateHeight(node);
    
        // Check if the node is now unbalanced
        var balance = this.getBalance(node);
  
        // Left-left case
        if (balance > 1 && priority < node.left.priority) 
            return this.rotateRight(node);
    
        // Right-right case
        if (balance < -1 && priority > node.right.priority)
            return this.rotateLeft(node);
  
        // Left-right case
        if (balance > 1 && priority > node.left.priority) 
        {
            node.left = this.rotateLeft(node.left);
            return this.rotateRight(node);
        }
    
        // Right-left case
        if (balance < -1 && priority < node.right.priority) 
        {
            node.right = this.rotateRight(node.right);
            return this.rotateLeft(node);
        }
  
        // Return the current node
        return node;
    }
  
    // Public function to insert a new value into the AVL tree with a given priority
    insert(value)//, priority) 
    {
        this.root = this.insertHelper(this.root, value, value.priority);//, this.priority);
    }
  
    // Helper function to find the node with the minimum priority
    findMin(node)
    {
        if (node === null)
            return null;
        if (node.left === null)
            return node;
        return this.findMin(node.left);
    }

    // Public function to remove the node with the minimum priority
    removeMin(node) 
    {
        // Base case: empty tree
        if (node === null)
            return null;

        // Base case: found the minimum priority node
        if (node.left === null)
            return node.right;

        // Recursively remove the minimum priority node from the left subtree
        node.left = this.removeMin(node.left);

        // Update the height of the current node
        this.updateHeight(node);

        // Rebalance the AVL tree if necessary
        const balance = this.getBalance(node);

        // Left-left case
        if (balance > 1 && this.getBalance(node.left) >= 0) 
            return this.rotateRight(node);

        // Left-right case
        if (balance > 1 && this.getBalance(node.left) < 0)
        {
            node.left = this.rotateLeft(node.left);
            return this.rotateRight(node);
        }

        // Return the current node
        return node;
    }

    // Public function to remove the node with the minimum priority and return its value
    pop() 
    {
        const minNode = this.findMin(this.root);
        if (minNode === null)
            return null;

        this.root = this.removeMin(this.root);
        return minNode.value;
    }

  // Public function to get the value with the minimum priority without removing it
    top() 
    {
        const minNode = this.findMin(this.root);
        if (minNode === null)
            return null;

        return minNode.value;
    }
}
