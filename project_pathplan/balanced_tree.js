class AVLNode
{
    //constructor(value, priority)
    constructor(value)
    {
        this.value = value;
        //this.priority = value; //priority;
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
    insertHelper(node, value) //, priority)
    {
        // Perform standard BST insertion
        if (node === null) 
            return new AVLNode(value);//, priority);
        
        var priority = value;
        if (priority < node.value) 
            node.left = this.insertHelper(node.left, value); //, priority);
        else 
            node.right = this.insertHelper(node.right, value); //, priority);
  
        // Update the height of the current node
        this.updateHeight(node);
    
        // Check if the node is now unbalanced
        var balance = this.getBalance(node);
  
        // Left-left case
        if (balance > 1 && priority < node.left.value) 
            return this.rotateRight(node);
    
        // Right-right case
        if (balance < -1 && priority > node.right.value)
            return this.rotateLeft(node);
  
        // Left-right case
        if (balance > 1 && priority > node.left.value) 
        {
            node.left = this.rotateLeft(node.left);
            return this.rotateRight(node);
        }
    
        // Right-left case
        if (balance < -1 && priority < node.right.value) 
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
        this.root = this.insertHelper(this.root, value);//, this.priority);
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
