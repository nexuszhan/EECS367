/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    heap.push(new_element);
    var cur = heap.length-1;
    while ((cur>0) && heap[cur] < heap[Math.floor((cur-1)/2)])
    {
        var tmp = heap[cur];
        heap[cur] = heap[Math.floor((cur-1)/2)];
        heap[Math.floor((cur-1)/2)] = tmp;
        cur = Math.floor((cur-1)/2);
    }
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
//Note: because the minheap_insert function is an object, we can assign 
//      a reference to the function within the minheap object, which can be called
//      as minheap.insert


// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    var top = heap[0];
    var last = heap.pop();
    if (heap.length === 0)
        return top;
    heap[0] = last;
    var i = 0;
    while ((2*i+1) < heap.length)
    {
        var j = 2*i+1;
        if (j < (heap.length-1) && heap[j] > heap[j+1])
            j++;
        if (heap[i] <= heap[j])
            break;
        var tmp = heap[i];
        heap[i] = heap[j];
        heap[j] = tmp;
        i = j;
    }

    return top;
}

// assign extract function within minheaper object
minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object


